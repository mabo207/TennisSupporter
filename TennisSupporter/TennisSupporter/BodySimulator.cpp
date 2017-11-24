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

BodySimulator::BodySimulator()
	:m_fileWriteFlag(false),m_writeCount(0),m_mode(0),m_playFrame(0),m_playRate(1.0),m_dataMin(300),m_dataMax(-300)
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
	//m_writeFile,m_readFileは、必要になり次第初期化する。

}

BodySimulator::~BodySimulator(){
	m_pSensor->Close();
	m_pSensor->Release();

	m_writeFile.close();
	m_readFile.close();
}

bool BodySimulator::ReadFile(const char *filename){
	std::ifstream readFile(filename);
	if(!m_readFile){
		//読み込み失敗時の処理
		return false;
	}
	//データを全て読み込むが、量が多いので上手くreserveしながら読み込む
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
	DataBuild(JointType_SpineBase);

	return true;
}

void BodySimulator::DataBuild(JointType jointtype){
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
				data=m_playData[i][j][jointtype].Z;
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
}

void BodySimulator::DataBuild(JointType edge,JointType point1,JointType point2){
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
				data=m_playData[i][j][edge].CalculateAngle(m_playData[i][j][point1],m_playData[i][j][point2]);
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
}

int BodySimulator::CalReadIndex()const{
	return (int)(m_playFrame*captureFps*m_playRate/drawFps);
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
				m_playFrame=0;
			}
		}
		break;
		//再生モード
	case(1):
		printfDx("PlayingDataMode\n");
		int a=CalReadIndex();
		m_playFrame++;
		int b=CalReadIndex();//この値がaに一致している時は読み込みは行わず、前フレームと同じ画像を描画する
		if(!m_readFile || a==b){
			//特に何もしない
		} else{
			if(b<m_playData.size()){
				m_pBodyKinectSensor->Update(m_playData[b]);
			}else{
				m_mode=0;
			}
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
			const Vector2D graphPos(20,20);
			const int graphHeight=360;
			for(size_t i=0,datanum=m_data.size();i<datanum;i++){
				DrawPixel(graphPos.x+i,graphPos.y+(int)(graphHeight/std::fmax(m_dataMax-m_dataMin,0.00001)*(m_dataMax-m_data[i])),GetColor(128,255,255));
			}
			DrawLine(graphPos.x+CalReadIndex(),graphPos.y,graphPos.x+CalReadIndex(),graphPos.y+graphHeight,GetColor(128,128,128),1);
		}
		break;
	}
	
}
