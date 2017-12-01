#define _USE_MATH_DEFINES

#include<cmath>
#include"BodySimulator.h"
#include"DxLib.h"
#include"input.h"

//-----------------BodySimulator-----------------
const int BodySimulator::writeCountMax=30*30;
const int BodySimulator::captureFps=30;
const int BodySimulator::drawFps=60;
const Vector2D BodySimulator::kinectSize=Vector2D(512,424);
const Vector2D BodySimulator::graphPos=Vector2D(100,60);
const Vector2D BodySimulator::graphSize=Vector2D(BodySimulator::writeCountMax,360);
const std::string BodySimulator::sectionStr="##################";

BodySimulator::BodySimulator()
	:m_fileWriteFlag(false),m_writeCount(0),m_mode(0),m_playFrame(0.0),m_playRate(1.0),m_dataMin(300),m_dataMax(-300),
	m_font(CreateFontToHandle("メイリオ",12,1,-1)),m_playFlag(true),m_beforeRClickFrame(0),m_startSectionIndex(0)
{
	//センサーの起動
	m_pSensor=nullptr;
	ErrorCheck(GetDefaultKinectSensor(&m_pSensor),"You can't get Kinect Sensor.");
	ErrorCheck(m_pSensor->Open(),"You can't activate Kinect Sensor.");
	BOOLEAN isOpen=false;
	ErrorCheck(m_pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
	//BodySensorの起動
	m_pBodyKinectSensor=std::shared_ptr<BodyKinectSensor>(new BodyKinectSensor(m_pSensor));
	//DepthKinectSensorの起動
	m_pDepthKinectSensor=std::shared_ptr<DepthKinectSensor>(new DepthKinectSensor(kinectSize,m_pSensor));
	//GraphDataBuilderの起動
	m_pGraphDataBuilder=std::shared_ptr<GraphDataBuilder>(new GraphDataBuilder(Vector2D(kinectSize.x*2,0)));
	//m_writeFile,m_readFileは、必要になり次第初期化する。

}

BodySimulator::~BodySimulator(){
	m_pSensor->Close();
	m_pSensor->Release();

	m_writeFile.close();
	
	DeleteFontToHandle(m_font);
}

bool BodySimulator::ReadFile(const char *filename){
	std::ifstream readFile(filename);
	//データを全て読み込むが、量が多いので上手くreserveしながら読み込む
	m_playData.clear();
	m_playData.reserve(writeCountMax);//最大記録フレーム数によりreserve

	std::vector<std::vector<BodyKinectSensor::JointPosition>> frameData;
	frameData.reserve(BodyKinectSensor::bodyNum);

	std::vector<BodyKinectSensor::JointPosition> bodyData;
	bodyData.reserve(JointType_Count);
	
	size_t bodyindex=0,jointindex=0;
	std::string parts="";
	parts.reserve(50);
	bool inBracketsFlag=false;//現在"(~~)"の中を読み取っているかどうか
	char ch='a';//初期値はテキトー
	try{
		while(true){
			ch=readFile.get();
			if(ch==EOF){
				//ファイルの末尾到達時
				break;
			} else if(ch=='\n'){
				//1行読み終わる時(==1フレーム読み終わり)
				m_playData.push_back(frameData);
				frameData.clear();
				bodyData.clear();
				bodyindex=0;
				jointindex=0;
				parts="";
				inBracketsFlag=false;
			} else{
				if(inBracketsFlag){
					//()内読み取り時(=')'を探している)
					parts.push_back(ch);
					if(ch==')'){
						inBracketsFlag=!inBracketsFlag;
						//jointPositionsに格納
						if(bodyindex<BodyKinectSensor::bodyNum && jointindex<JointType_Count){
							bodyData.push_back(BodyKinectSensor::JointPosition(parts));
							//index系の更新
							jointindex++;
							if(jointindex>=JointType_Count){
								jointindex=0;
								bodyindex++;
								frameData.push_back(bodyData);
								bodyData.clear();
							}
						}
						parts.clear();
					}
				} else{
					//()外読み取り時(='('を探している)
					if(ch=='('){
						inBracketsFlag=!inBracketsFlag;
						parts.push_back(ch);
					}
				}
			}
		}
	}catch(const std::exception &e){
		//メモリ関連のエラー対策
		printf(e.what());
		return false;
	}
	//グラフについてのデータ読み取り
	DataBuild();
	readFile.close();

	//ファイル名を拡張子以外保存
	std::string fname(filename);
	size_t index=0;
	for(size_t size=fname.size();index<size;index++){
		if(fname[index]=='.'){
			break;
		}
	}
	m_playDataName.clear();
	m_playDataName.reserve(index);
	for(size_t i=0;i<index;i++){
		m_playDataName.push_back(fname[i]);
	}

	//再生データの初期化
	m_playFlag=true;
	m_section.clear();
	m_beforeRClickFrame=0;
	m_startSectionIndex=0;

	return true;
}

void BodySimulator::DataBuild(){
	size_t playdatasize=m_playData.size();
	m_data.clear();
	m_data.reserve(playdatasize);
	for(size_t i=0;i<playdatasize;i++){
		double data=0.0;
		for(size_t j=0,bodynum=m_playData[i].size();j<bodynum;j++){
			bool flag=false;
			for(size_t k=0,jointnum=m_playData[i][j].size();k<jointnum;k++){
				if(!(m_playData[i][j][k]==BodyKinectSensor::JointPosition())){
					flag=true;
					break;
				}
			}
			if(flag){
				data=m_pGraphDataBuilder->CalData(m_playData[i][j]);
				break;
			}
		}
		m_data.push_back(data);
		if(i!=0){
			m_dataMin=std::fmin(m_dataMin,data);
			m_dataMax=std::fmax(m_dataMax,data);
		} else{
			m_dataMin=data;
			m_dataMax=data;
		}
	}
	//フレーム数を初期化、イメージも更新
	m_playFrame=0.0;
	UpdateImage();
}

int BodySimulator::CalReadIndex()const{
	return (int)(m_playFrame*captureFps/drawFps);
}

double BodySimulator::CalPlayFrame(int index)const{
	return ((double)index)*drawFps/captureFps;
}

bool BodySimulator::JudgeMouseInGraph()const{
	Vector2D relativeMouse=GetMousePointVector2D()-graphPos;
	return (relativeMouse.x>=0 && relativeMouse.x<=graphSize.x && relativeMouse.y>=0 && relativeMouse.y<=graphSize.y);
}

void BodySimulator::UpdateImage(){
	if(m_mode==1){
		int index=CalReadIndex();
		if(index>=0 && index<(int)m_playData.size()){
			m_pBodyKinectSensor->Update(m_playData[index]);
		}
	}
}

void BodySimulator::WriteSections(){
	//保存先ファイル名作成
	const std::string filename=m_playDataName+"_section.txt";
	//ファイルを開く
	std::ofstream ofs(filename.c_str(),std::ios_base::trunc);
	if(!ofs){
		return;
	}
	//ファイルに書き出し
	int playdatasize=(int)m_playData.size();
	for(const std::pair<int,int> &pair:m_section){
		//開始と末尾を算出
		int top=std::fmin(pair.first,pair.second);
		int bottom=std::fmax(pair.first,pair.second);
		//書き出し
		if(top>=0 && bottom<playdatasize){
			//区切り文字列と改行の書き出し
			ofs<<sectionStr<<std::endl;
			//1フレームずつ書き出し
			for(int i=top;i<=bottom;i++){
				m_pBodyKinectSensor->OutputJointPoitions(ofs,m_playData[i]);
			}
		}
	}
	ofs.close();

	//書き出し終了が分かるように再生を始める
	m_playFlag=true;
}

int BodySimulator::Update(){
	switch(m_mode){
		//記録モード
	case(0):
		printfDx("RecordingDataMode\n");
		//depth
		//データの読み取り

		//depth
		printfDx("depth:\n");
		m_pDepthKinectSensor->Update();

		//body
		printfDx("body:\n");
		m_pBodyKinectSensor->Update();
		//情報の記録をするかのフラグの更新
		if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
			m_fileWriteFlag=!m_fileWriteFlag;
			if(m_fileWriteFlag){
				//記録開始時はファイルを開き、m_writeCountを0にする
				m_writeCount=0;
				m_writeFile.open("SaveData/"+to_string_0d(0,3)+".txt",std::ios_base::trunc);
				if(!m_writeFile){
					//ファイルを開けなければ記録開始しない
					m_fileWriteFlag=false;
				}
			} else{
				//記録終了時はファイルを閉じる
				m_writeFile.close();
			}
		}
		//ファイル出力
		printfDx("fileWriteFlag:\n");
		printfDx((m_fileWriteFlag && !(!m_writeFile)) ? "true\n":"false\n");
		if(m_fileWriteFlag){
			//書き込みすぎ判定
			m_writeCount++;
			if(m_writeCount>writeCountMax){
				m_fileWriteFlag=false;
			}
			//body位置の出力
			m_pBodyKinectSensor->OutputJointPoitions(m_writeFile);
		}
		//記録データ再生モードへの移行処理
		if(keyboard_get(KEY_INPUT_0)==1){
			if(ReadFile(("SaveData/"+to_string_0d(0,3)+".txt").c_str())){
				//読み込み成功時のみ、再生モードへ
				m_mode=1;
			}
		}
		break;
		//再生モード
	case(1):
		printfDx("PlayingDataMode\n");
		//秒数更新
		if(mouse_get(MOUSE_INPUT_LEFT)>0 && JudgeMouseInGraph()){
			//グラフ内で左クリック時、再生時間をそこに合わせて、イメージ更新
			m_playFrame=CalPlayFrame((GetMousePointVector2D().x-graphPos.x)*writeCountMax/graphSize.x);
			UpdateImage();
		} else if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
			//Enterキー入力で先頭から再生し、イメージ更新
			m_playFrame=0.0;
			UpdateImage();
		} else if(keyboard_get(KEY_INPUT_RSHIFT)==1){
			//右シフトキー入力で再生停止の切り替え
			m_playFlag=!m_playFlag;
		} else if(keyboard_get(KEY_INPUT_LEFT)==1){
			//左キー入力で再生位置を戻し、イメージ更新
			m_playFrame-=m_playRate;
			UpdateImage();
		} else if(keyboard_get(KEY_INPUT_RIGHT)==1){
			//右キー入力で再生位置を進め、イメージ更新
			m_playFrame+=m_playRate;
			UpdateImage();
		}
		//イメージ再生画面
		if(mouse_get(MOUSE_INPUT_RIGHT)<=0){
			//右クリックが押されていない場合は普通に再生
			int a=CalReadIndex();
			if(m_playFlag && a<(int)m_playData.size()){
				//まだデータ終端までいっておらず、かつ再生モードになっている時、フレーム数を更新
				m_playFrame+=m_playRate;
			}
			int b=CalReadIndex();//この値がaに一致している時は読み込みは行わず、前フレームと同じ画像を描画する
			if(a!=b){
				UpdateImage();
			}
		} else{
			//右クリックが押されている場合は、マウスの位置に従ってイメージを更新
			size_t index=(size_t)(std::fmin(m_playData.size()-1,(size_t)(std::fmax(0,GetMousePointVector2D().x-graphPos.x))));
			m_pBodyKinectSensor->Update(m_playData[index]);
		}
		//入力インターフェース
		if(m_pGraphDataBuilder->Update()==1){
			//m_dataFactoryを更新した時はDataBuild()を使用する
			DataBuild();
		}
		//グラフデータ切り取り操作
		int rframe=mouse_get(MOUSE_INPUT_RIGHT);
		if(rframe==1){
			//右クリック開始
			m_startSectionIndex=CalReadIndex();//開始indexの保存
		} else if(rframe==0 && m_beforeRClickFrame>0){
			//右クリックを離した直後
			Vector2D mousePos=GetMousePointVector2D();
			if(JudgeMouseInGraph()){
				m_section.push_back(std::pair<int,int>(m_startSectionIndex,(mousePos-graphPos).x*writeCountMax/graphSize.x));//区間の保存
			}
		}
		m_beforeRClickFrame=rframe;
		//セクションデータ出力操作
		if(keyboard_get(KEY_INPUT_S)==10){
			WriteSections();
		}
		//場面遷移
		if(keyboard_get(KEY_INPUT_BACK)==1){
			//Backキー入力で記録モードに戻る
			m_mode=0;
		}
		break;
	}
	//全モード共通処理
	//再生モードにおける再生速度調整
	if(keyboard_get(KEY_INPUT_Z)>0){
		//遅くする
		m_playRate=std::fmax(0.1,m_playRate-0.05);
	} else if(keyboard_get(KEY_INPUT_X)>0){
		//1.0倍に戻す
		m_playRate=1.0;
	} else if(keyboard_get(KEY_INPUT_C)>0){
		//速くする
		m_playRate=m_playRate+0.05;
	}
	printfDx("playRate:\n%f",m_playRate);


	//特に終了条件は無いので0を常に返す
	return 0;
}

void BodySimulator::Draw()const{
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	switch(m_mode){
	case(0):
		//データ記録時
		m_pDepthKinectSensor->Draw(depthPos);
		m_pBodyKinectSensor->Draw(m_pSensor,depthPos,kinectSize,xyPos,kinectSize,zyPos,kinectSize);
		break;
	case(1):
		//再生時
		m_pBodyKinectSensor->Draw(m_pSensor,Vector2D(-3000,-3000),kinectSize,xyPos,kinectSize,zyPos,kinectSize);//(depth画像に対するbodyボーンは描画しない)
		//グラフ描画
		{
			//現在存在している区間の表示
			for(const std::pair<int,int> &section:m_section){
				DrawBox(graphPos.x+section.first,graphPos.y,graphPos.x+section.second,graphPos.y+graphSize.y,GetColor(128,128,255),TRUE);
			}
			//現在作っている区間の表示
			if(mouse_get(MOUSE_INPUT_RIGHT)>1){
				DrawBox(graphPos.x+m_startSectionIndex,graphPos.y,GetMousePointVector2D().x,graphPos.y+graphSize.y,GetColor(255,255,0),TRUE);
			}

			//折れ線の描画
			for(size_t i=0,datanum=m_data.size();i<datanum;i++){
				DrawPixel(graphPos.x+i,graphPos.y+(int)(graphSize.y/std::fmax(m_dataMax-m_dataMin,0.00001)*(m_dataMax-m_data[i])),GetColor(128,255,255));
			}
			//再生時間の描画
			DrawLine(graphPos.x+CalReadIndex(),graphPos.y,graphPos.x+CalReadIndex(),graphPos.y+graphSize.y,GetColor(128,128,128),1);
			//現在の値の表示
			if(CalReadIndex()>=0 && CalReadIndex()<(int)m_data.size()){
				//配列外参照をする可能性があるので弾く。配列外参照時は描画しない。
				int dataY=graphPos.y+(int)(graphSize.y/std::fmax(m_dataMax-m_dataMin,0.00001)*(m_dataMax-m_data[CalReadIndex()]));
				DrawLine(graphPos.x,dataY,graphPos.x+graphSize.x,dataY,GetColor(128,128,128),1);
			}
			//data最大値の表示
			DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
			DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(m_dataMax),GetColor(255,255,255),m_font);
			//data最小値の表示
			DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
			DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(m_dataMin),GetColor(255,255,255),m_font);
			
			//読み込みデータインターフェースの描画
			m_pGraphDataBuilder->Draw();
		}
		break;
	}
	
}
