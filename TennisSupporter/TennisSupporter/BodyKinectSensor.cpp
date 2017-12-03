#include"BodyKinectSensor.h"
#include"DxLib.h"

//-------------------BodyKinectSensor-------------------
BodyKinectSensor::BodyKinectSensor(IKinectSensor *pSensor){
	//source
	IBodyFrameSource *pBodySource;
	ErrorCheck(pSensor->get_BodyFrameSource(&pBodySource),"You can't get source.");
	//reader
	ErrorCheck(pBodySource->OpenReader(&m_pBodyReader),"You can't open reader.");
}

BodyKinectSensor::~BodyKinectSensor(){}

int BodyKinectSensor::Update(){
	//一時変数の用意
	IBodyFrame *pBodyFrame=nullptr;
	IBody *pBodies[bodyNum];
	for(size_t i=0;i<bodyNum;i++){
		pBodies[i]=nullptr;
	}
	//更新作業
	try{
		ErrorCheck(m_pBodyReader->AcquireLatestFrame(&pBodyFrame),"aquaire failed\n");//直近フレームのbodyデータの取得
		ErrorCheck(pBodyFrame->GetAndRefreshBodyData(6,pBodies),"access failed\n");//bodyデータをpBodiesに格納
		pBodyFrame->Release();//これ以降pBodyFrameは使わない
		printfDx("success\n");
		for(int j=0;j<bodyNum;j++){
			//関節の実座標位置をjointPositionsに格納
			Joint joints[JointType::JointType_Count];
			pBodies[j]->GetJoints(JointType::JointType_Count,joints);//関節位置の実座標の取得
			//jointPositionsに関節の実座標を格納
			for(int i=0;i<JointType_Count;i++){
				m_jointPositions[j][i]=JointPosition(joints[i].Position);
			}
		}
	} catch(const std::exception &e){
		printfDx(e.what());
	}

	return 0;//返す値はテキトー
}

int BodyKinectSensor::Update(std::ifstream &readFile){
	size_t bodyindex=0,jointindex=0;
	std::string parts="";
	parts.reserve(50);
	bool inBracketsFlag=false;//現在"(~~)"の中を読み取っているかどうか
	char ch='a';//初期値はテキトー
	while(true){
		ch=readFile.get();
		if(ch=='\n' || ch==EOF){
			//1行読み終わるか、ファイルの末尾到達時
			break;
		} else{
			if(inBracketsFlag){
				//()内読み取り時(=')'を探している)
				parts.push_back(ch);
				if(ch==')'){
					inBracketsFlag=!inBracketsFlag;
					//jointPositionsに格納
					if(bodyindex<bodyNum && jointindex<JointType_Count){
						m_jointPositions[bodyindex][jointindex]=JointPosition(parts);
						//index系の更新
						jointindex++;
						if(jointindex>=JointType_Count){
							jointindex=0;
							bodyindex++;
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

	//値を返す。chがEOFかどうかを伝える
	if(ch==EOF){
		return 1;//データ読み込みが終了する事を伝える
	} else{
		return 0;
	}
}
