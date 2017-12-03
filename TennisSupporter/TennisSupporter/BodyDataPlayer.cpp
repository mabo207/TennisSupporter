#define _USE_MATH_DEFINES

#include<cmath>
#include"BodyDataPlayer.h"
#include"DxLib.h"
#include"input.h"

//-----------------BodyDataPlayer-----------------
const Vector2D BodyDataPlayer::graphPos=Vector2D(100,60);
const Vector2D BodyDataPlayer::graphSize=Vector2D(BodyDataPlayer::writeCountMax,360);
const std::string BodyDataPlayer::sectionStr="##################";

BodyDataPlayer::BodyDataPlayer(int font,const char *filename)
	:IBodySimulateScene(MODE::PLAYER),m_playFrame(0.0),m_playRate(1.0),
	m_font(font),m_playFlag(true),m_beforeRClickFrame(0),m_startSectionIndex(0),m_graphUnity(false)
{
	//BodyVirtualKinectSensorの起動
	m_graphSingleData.m_pBodyVirtualKinectSensor=std::shared_ptr<BodyVirtualKinectSensor>(new BodyVirtualKinectSensor());
	//GraphDataBuilderの起動
	m_pGraphDataBuilder=std::shared_ptr<GraphDataBuilder>(new GraphDataBuilder(Vector2D(kinectSize.x*2,0)));
	//データの読み取り
	ReadFile(filename);
}

BodyDataPlayer::~BodyDataPlayer(){}

bool BodyDataPlayer::ReadFile(const char *filename){
	std::ifstream readFile(filename);
	//データを全て読み込むが、量が多いので上手くreserveしながら読み込む
	m_graphSingleData.m_playData.clear();
	m_graphSingleData.m_playData.reserve(writeCountMax);//最大記録フレーム数によりreserve

	std::vector<std::vector<IBodyKinectSensor::JointPosition>> frameData;
	frameData.reserve(IBodyKinectSensor::bodyNum);

	std::vector<IBodyKinectSensor::JointPosition> bodyData;
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
				m_graphSingleData.m_playData.push_back(frameData);
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
						if(bodyindex<IBodyKinectSensor::bodyNum && jointindex<JointType_Count){
							bodyData.push_back(IBodyKinectSensor::JointPosition(parts));
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
	} catch(const std::exception &e){
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

void BodyDataPlayer::DataBuild(){
	m_graphSingleData.DataBuild(m_pGraphDataBuilder);
	//フレーム数を初期化、イメージも更新
	m_playFrame=0.0;
	UpdateImage();
}

int BodyDataPlayer::CalReadIndex()const{
	return (int)(m_playFrame*captureFps/drawFps);
}

double BodyDataPlayer::CalPlayFrame(int index)const{
	return ((double)index)*drawFps/captureFps;
}

bool BodyDataPlayer::JudgeMouseInGraph()const{
	Vector2D relativeMouse=GetMousePointVector2D()-graphPos;
	return (relativeMouse.x>=0 && relativeMouse.x<=graphSize.x && relativeMouse.y>=0 && relativeMouse.y<=graphSize.y);
}

void BodyDataPlayer::UpdateImage(){
	m_graphSingleData.UpdateVirtualSensor(CalReadIndex());
}

void BodyDataPlayer::WriteSections(){
	//保存先ファイル名作成
	const std::string filename=m_playDataName+"_section.txt";
	//ファイルを開く
	std::ofstream ofs(filename.c_str(),std::ios_base::trunc);
	if(!ofs){
		return;
	}
	//ファイルに書き出し
	int playdatasize=(int)m_graphSingleData.m_playData.size();
	for(const std::pair<int,int> &pair:m_section){
		//開始と末尾を算出
		int top=(int)std::fmin(pair.first,pair.second);
		int bottom=(int)std::fmax(pair.first,pair.second);
		//書き出し
		if(top>=0 && bottom<playdatasize){
			//区切り文字列と改行の書き出し
			ofs<<sectionStr<<std::endl;
			//1フレームずつ書き出し
			for(int i=top;i<=bottom;i++){
				m_graphSingleData.m_pBodyVirtualKinectSensor->OutputJointPoitions(ofs,m_graphSingleData.m_playData[i]);
			}
		}
	}
	ofs.close();

	//書き出し終了が分かるように再生を始める
	m_playFlag=true;
}

int BodyDataPlayer::Update(){
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
		if(m_playFlag && a<(int)m_graphSingleData.m_playData.size()){
			//まだデータ終端までいっておらず、かつ再生モードになっている時、フレーム数を更新
			m_playFrame+=m_playRate;
		}
		int b=CalReadIndex();//この値がaに一致している時は読み込みは行わず、前フレームと同じ画像を描画する
		if(a!=b){
			UpdateImage();
		}
	} else{
		//右クリックが押されている場合は、マウスの位置に従ってイメージを更新
		size_t index=(size_t)(std::fmin(m_graphSingleData.m_playData.size()-1,(size_t)(std::fmax(0,GetMousePointVector2D().x-graphPos.x))));
		m_graphSingleData.UpdateVirtualSensor(index);
	}
	//入力インターフェース
	if(m_pGraphDataBuilder->Update()==1){
		//m_dataFactoryを更新した時はDataBuild()を使用する
		DataBuild();
	}
	//グラフデータ切り取り操作
	int rframe=mouse_get(MOUSE_INPUT_RIGHT);
	if(rframe==1 && !m_graphUnity){
		//右クリック開始
		m_startSectionIndex=CalReadIndex();//開始indexの保存
	} else if(rframe==0 && m_beforeRClickFrame>0 && !m_graphUnity){
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
	//グラフの軸基準の切り替え
	if(keyboard_get(KEY_INPUT_U)==1){
		m_graphUnity=!m_graphUnity;
	}
	//再生速度調整
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


	//場面遷移
	if(keyboard_get(KEY_INPUT_BACK)==1){
		//Backキー入力で記録モードに戻る
		return 1;
	}
	return 0;
}

void BodyDataPlayer::Draw()const{
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	//再生時
	m_graphSingleData.m_pBodyVirtualKinectSensor->Draw(nullptr,Vector2D(-3000,-3000),kinectSize,xyPos,kinectSize,zyPos,kinectSize);//(depth画像に対するbodyボーンは描画しない)
	//グラフ描画
	if(m_graphUnity){
		//統一基準(1pxにつき1フレームという基準が消滅する)
		//最大最小値の設定
		const double dataTop=m_pGraphDataBuilder->DataMax(),dataBottom=m_pGraphDataBuilder->DataMin();
		//1フレームに対するピクセル数の計算
		const double frameRateToPixel=((double)writeCountMax)/m_graphSingleData.m_data.size();
		//現在存在している区間の表示
		for(const std::pair<int,int> &section:m_section){
			DrawBox(graphPos.x+(int)(section.first*frameRateToPixel),graphPos.y,graphPos.x+(int)(section.second*frameRateToPixel),graphPos.y+graphSize.y,GetColor(128,128,255),TRUE);
		}
		//現在作っている区間の表示
		if(mouse_get(MOUSE_INPUT_RIGHT)>1){
			DrawBox(graphPos.x+(int)(m_startSectionIndex*frameRateToPixel),graphPos.y,GetMousePointVector2D().x,graphPos.y+graphSize.y,GetColor(255,255,0),TRUE);
		}

		//折れ線の描画
		for(size_t i=0,datanum=m_graphSingleData.m_data.size();i<datanum;i++){
			DrawCircle(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[i])),1,GetColor(128,255,255),TRUE);
		}
		for(size_t i=0,datanum=m_graphSingleData.m_data.size();i+1<datanum;i++){
			DrawLine(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[i])),graphPos.x+(int)((i+1)*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[i+1])),GetColor(128,128,255),1);
		}
		//再生時間の描画
		DrawLine(graphPos.x+(int)(CalReadIndex()*frameRateToPixel),graphPos.y,graphPos.x+(int)(CalReadIndex()*frameRateToPixel),graphPos.y+graphSize.y,GetColor(128,128,128),1);
		//現在の値の表示
		if(CalReadIndex()>=0 && CalReadIndex()<(int)m_graphSingleData.m_data.size()){
			//配列外参照をする可能性があるので弾く。配列外参照時は描画しない。
			int dataY=graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[CalReadIndex()]));
			DrawLine(graphPos.x,dataY,graphPos.x+graphSize.x,dataY,GetColor(128,128,128),1);
		}
		//data最大値の表示
		DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(dataTop),GetColor(255,255,255),m_font);
		//data最小値の表示
		DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(dataBottom),GetColor(255,255,255),m_font);
	} else{
		//データ依存の基準
		//現在存在している区間の表示
		for(const std::pair<int,int> &section:m_section){
			DrawBox(graphPos.x+section.first,graphPos.y,graphPos.x+section.second,graphPos.y+graphSize.y,GetColor(128,128,255),TRUE);
		}
		//現在作っている区間の表示
		if(mouse_get(MOUSE_INPUT_RIGHT)>1){
			DrawBox(graphPos.x+m_startSectionIndex,graphPos.y,GetMousePointVector2D().x,graphPos.y+graphSize.y,GetColor(255,255,0),TRUE);
		}

		//折れ線の描画
		for(size_t i=0,datanum=m_graphSingleData.m_data.size();i<datanum;i++){
			DrawPixel(graphPos.x+i,graphPos.y+(int)(graphSize.y/std::fmax(m_graphSingleData.m_dataMax-m_graphSingleData.m_dataMin,0.00001)*(m_graphSingleData.m_dataMax-m_graphSingleData.m_data[i])),GetColor(128,255,255));
		}
		//再生時間の描画
		DrawLine(graphPos.x+CalReadIndex(),graphPos.y,graphPos.x+CalReadIndex(),graphPos.y+graphSize.y,GetColor(128,128,128),1);
		//現在の値の表示
		if(CalReadIndex()>=0 && CalReadIndex()<(int)m_graphSingleData.m_data.size()){
			//配列外参照をする可能性があるので弾く。配列外参照時は描画しない。
			int dataY=graphPos.y+(int)(graphSize.y/std::fmax(m_graphSingleData.m_dataMax-m_graphSingleData.m_dataMin,0.00001)*(m_graphSingleData.m_dataMax-m_graphSingleData.m_data[CalReadIndex()]));
			DrawLine(graphPos.x,dataY,graphPos.x+graphSize.x,dataY,GetColor(128,128,128),1);
		}
		//data最大値の表示
		DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(m_graphSingleData.m_dataMax),GetColor(255,255,255),m_font);
		//data最小値の表示
		DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(m_graphSingleData.m_dataMin),GetColor(255,255,255),m_font);

	}
	//読み込みデータインターフェースの描画
	m_pGraphDataBuilder->Draw();
}

