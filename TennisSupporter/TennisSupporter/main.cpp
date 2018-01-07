#define _USE_MATH_DEFINES
#include<cmath>
#include<cassert>
#include"DxLib.h"
#include<Kinect.h>
#include"input.h"
#include"KinectTools.h"
#include"BodySimulator.h"

#include<set>
#include<vector>
#include<string>
#include<fstream>
#include<iostream>

#include<time.h>

struct JointPosition{
	static const float defaultfloat;
	float X;
	float Y;
	float Z;
	//X,Y,Zの値を直接入力する
	JointPosition(float i_X=defaultfloat,float i_Y=defaultfloat,float i_Z=defaultfloat)
		:X(i_X),Y(i_Y),Z(i_Z){}
	//_CameraSpacePointより初期化する
	JointPosition(_CameraSpacePoint pos)
		:X(pos.X),Y(pos.Y),Z(pos.Z){}
	//"(X,Y,Z)"という形式の文字列を読み取って初期化する
	JointPosition(const std::string &str){
		unsigned int size=str.size();
		int process=0;//0:"("の読み取り 1:Xの読み取り 2:Yの読み取り 3:Zの読み取り
		std::string parts="";
		parts.reserve(15);
		try{
			for(unsigned int i=0;i<size;i++){
				switch(process){
				case(0):
					if(str[i]=='('){
						process++;
						parts.clear();
					}
					break;
				case(1):
					if(str[i]==','){
						process++;
						X=std::stof(parts);
						parts.clear();
					} else{
						parts.push_back(str[i]);
					}
					break;
				case(2):
					if(str[i]==','){
						process++;
						Y=std::stof(parts);
						parts.clear();
					} else{
						parts.push_back(str[i]);
					}
					break;
				case(3):
					if(str[i]==')'){
						process++;
						Z=std::stof(parts);
						parts.clear();
					} else{
						parts.push_back(str[i]);
					}
					break;
				}
			}
		}catch(const std::exception &e){
			//桁数が大きすぎた時の処理
			JointPosition j=JointPosition();
			X=j.X;
			Y=j.Y;
			Z=j.Z;
		}
	}
	//"(X,Y,Z)"という文字列を出力する
	std::string GetString()const{
		return "("+std::to_string(X)+","+std::to_string(Y)+","+std::to_string(Z)+")";
	}
	//_CameraSpacePointを作成
	_CameraSpacePoint GetCameraSpacePoint()const{
		_CameraSpacePoint c;
		c.X=X;
		c.Y=Y;
		c.Z=Z;
		return c;
	}
	//Joint::PositionがJointPositionのようになっているJointを返す。その他の要素はテキトー。
	Joint CreateJoint()const{
		return CreateJoint(JointType_SpineBase);
	}
	Joint CreateJoint(_JointType type)const{
		return CreateJoint(type,TrackingState_NotTracked);
	}
	Joint CreateJoint(_TrackingState state)const{
		return CreateJoint(JointType_SpineBase,state);
	}
	Joint CreateJoint(_JointType type,_TrackingState state)const{
		Joint j;
		j.JointType=type;
		j.TrackingState=state;
		j.Position.X=X;
		j.Position.Y=Y;
		j.Position.Z=Z;
		return j;
	}
};
const float JointPosition::defaultfloat=0.0001;

void DepthSimulate(bool objectcheck,Vector2D KinectSize)noexcept(false){
	//kinectの初期化
	//sensor
	IKinectSensor *pSensor=nullptr;
	ErrorCheck(GetDefaultKinectSensor(&pSensor),"You can't get Kinect Sensor.");
	ErrorCheck(pSensor->Open(),"You can't activate Kinect Sensor.");
	BOOLEAN isOpen=false;
	ErrorCheck(pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
	//source
	IDepthFrameSource *pDepthSource;
	ErrorCheck(pSensor->get_DepthFrameSource(&pDepthSource),"You can't get source.");
	//reader
	IDepthFrameReader *pDepthReader;
	ErrorCheck(pDepthSource->OpenReader(&pDepthReader),"You can't open reader.");
	//memory
	unsigned int bufferSize=KinectSize.x*KinectSize.y*sizeof(unsigned short);
	unsigned short *bufferMat=nullptr;//情報取得用ポインタ（Kinectが情報取得のために確保したメモリにアクセスするので、毎フレームポインタの値が変わる）
	unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//直前に読み取ったdepth画像を保持する配列
	unsigned char *readMat=new unsigned char[KinectSize.x*KinectSize.y/8];//物体検出の際に既に物体があるかを調べたピクセルかを0(false)か1(true)で記憶して計算を効率化する。sizeof(char)=1byte=8bitは環境に依存しないのを利用し、(x,y)の判定はreadMat[(x+y*KinectSize.x)/8]の上から((x+y*KinectSize.x)%8)bit目(一番上を0bit目とする)に格納。
	if(drawMat==nullptr || readMat==nullptr){
		throw(std::runtime_error("Memory is not satisfied."));
	}
	for(int i=0;i<KinectSize.x*KinectSize.y;i++){
		drawMat[i]=0;
	}
	std::vector<std::set<int>> contoursVec={};//輪郭線一覧

	//アプリケーション動作
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//ゲーム本体
		//キー情報更新
		input_update();
		//描画
		clsDx();
		const unsigned short *adress1=bufferMat;
		printfDx("BufferMat:%x\ncontoursVec size:%d\n",bufferMat,contoursVec.size());
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				int color=drawMat[x+y*KinectSize.x];
				int c=color*255/8000;//値を0~255の範囲内に収める。drawmat内にある数値は500～8000で、mm単位での物体までの距離である
				DrawPixel(x,y,GetColor(c,c,c));
			}
		}
		//輪郭線描画
		for(const std::set<int> &set:contoursVec){
			for(const int &index:set){
				int x=index%KinectSize.x;
				int y=index/KinectSize.x;
				DrawPixel(x,y,GetColor(0,0,255));
			}
		}
		//計算処理
		//データの読み取り
		try{
			IDepthFrame *pDepthFrame=nullptr;
			ErrorCheck(pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error");
			ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error");
			printfDx("success");
			//読み取りができたのでdrawMatを更新。鏡像なので左右反転させる。
			const unsigned short *adress2=bufferMat;
			if(bufferMat!=nullptr){
				for(int y=0;y<KinectSize.y;y++){
					for(int x=0;x<KinectSize.x;x++){
						drawMat[(KinectSize.x-x-1)+y*KinectSize.x]=bufferMat[x+y*KinectSize.x];
					}
				}
			}
			pDepthFrame->Release();//bufferMatの参照が終了したのでpDepthFrameを開放する
		} catch(const std::exception &e){
			printfDx(e.what());
		}

		//物体検出
		//認識できなかったピクセル部分のdepthを0にする。
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				if(drawMat[x+y*KinectSize.x]>8000){
					drawMat[x+y*KinectSize.x]=0;
				}
			}
		}
		if(objectcheck){
			//readMat初期化
			for(int i=0;i<KinectSize.x*KinectSize.y/8;i++){
				readMat[i]=0;
			}
			//contoursVec初期化
			contoursVec.clear();
			//定数定義
			const int acceptdistance=300;//50mm以内のdepth距離の違いがあるピクセルに進んでいく
										 //物体が検出できたピクセルを検出し、物体認識をしていく
			for(int i=0;i<KinectSize.x*KinectSize.y;i++){
				bool insertflag=true;//次に調べる物体をcontoursVecに追加するかどうか
									 //まだ物体検出がされておらず、物体検出したピクセルを見つけたら
				if(drawMat[i]>0 && (readMat[i/8] & 1<<(7-i%8))==0){
					//そのピクセルから時計回りに距離が近いピクセルを探していくことで、物体の輪郭を求める
					int index=i;
					std::set<int> contours={};//輪郭を表すピクセルの集合
					int indextable[8][2]={{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};
					int finalJ=3;//検索するindextableが毎回のdo~whileループ内で同じ箇所から始まらないように（対称な点の次の点から始まるように）、直前でどのindextableを検索したかを記録しておく。初期値はぶっちゃけなんでもよい。
					do{
						//現在のx,y座標を求める
						int x=index%KinectSize.x,y=index/KinectSize.x;
						//周囲8ピクセルのどこにdepth距離の近いピクセルがあるかを検索してindexの更新
						for(int j=(finalJ+5)%8;j!=(finalJ+4)%8;j=(j+1)%8){
							int xx=x+indextable[j][0];
							int yy=y+indextable[j][1];
							//(xx,yy)のピクセルが存在しているかの判定
							if(xx>=0 && xx<KinectSize.x && yy>=0 && yy<KinectSize.y){
								//depth距離が近いかの判定
								int nextindex=xx+yy*KinectSize.x;//調べるピクセルに対応する配列番号
								int d1=drawMat[index];
								int d2=drawMat[nextindex];
								if(std::abs(drawMat[index]-drawMat[nextindex])<=acceptdistance){
									index=nextindex;
									finalJ=j;
									break;
								}
							}
						}
						//輪郭集合に情報を格納する
						contours.insert(index);
						//readMatを更新
						unsigned char byte=1<<(7-index%8);
						if((readMat[index/8] & byte)!=0){
							//次のピクセルが既にreadMatが0なら、そのピクセルは既に検出された別の物体に属しているので、これ以上調べても無駄になる。輪郭集合に入れない事も伝えつつ探索終了する。
							insertflag=false;
							break;
						} else{
							readMat[index/8]=readMat[index/8] | byte;
						}
					} while(index!=i);
					if(insertflag){
						//既に調べた物体でなければ
						if(contours.size()>8){
							//ある程度の大きさがあれば
							contoursVec.push_back(contours);
						}
					}
				}
			}
		}
		//終了検出
		if(keyboard_get(KEY_INPUT_ESCAPE)>0){
			break;
		}
	}

	//終了処理
	if(drawMat!=nullptr){
		delete drawMat;
	}
	if(readMat!=nullptr){
		delete readMat;
	}


	pSensor->Close();
	pSensor->Release();

}

void BodySimulate(Vector2D KinectSize){
	const int circlesize=3;
	//フォント作成
	//int font=CreateFontToHandle("メイリオ",circlesize*3/2,1,-1);

	//kinectの初期化
	//bodyについて
	//sensor
	IKinectSensor *pSensor=nullptr;
	ErrorCheck(GetDefaultKinectSensor(&pSensor),"You can't get Kinect Sensor.");
	ErrorCheck(pSensor->Open(),"You can't activate Kinect Sensor.");
	BOOLEAN isOpen=false;
	ErrorCheck(pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
	//source
	IBodyFrameSource *pBodySource;
	ErrorCheck(pSensor->get_BodyFrameSource(&pBodySource),"You can't get source.");
	//reader
	IBodyFrameReader *pBodyReader;
	ErrorCheck(pBodySource->OpenReader(&pBodyReader),"You can't open reader.");
	
	//毎フレーム更新するもの
	//Body
	const size_t BodyNum=6;
	IBodyFrame *pBodyFrame=nullptr;
	IBody *pBodies[BodyNum];
	for(size_t i=0;i<BodyNum;i++){
		pBodies[i]=nullptr;
	}
	//Joint::Position
	JointPosition jointPositions[BodyNum][JointType_Count];

	//可視化のためにdepth画像も表示
	//source
	IDepthFrameSource *pDepthSource;
	ErrorCheck(pSensor->get_DepthFrameSource(&pDepthSource),"You can't get source.");
	//reader
	IDepthFrameReader *pDepthReader;
	ErrorCheck(pDepthSource->OpenReader(&pDepthReader),"You can't open reader.");
	//memory
	unsigned int bufferSize=KinectSize.x*KinectSize.y*sizeof(unsigned short);
	unsigned short *bufferMat=nullptr;//情報取得用ポインタ（Kinectが情報取得のために確保したメモリにアクセスするので、毎フレームポインタの値が変わる）
	unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//直前に読み取ったdepth画像を保持する配列
	unsigned char *readMat=new unsigned char[KinectSize.x*KinectSize.y/8];//物体検出の際に既に物体があるかを調べたピクセルかを0(false)か1(true)で記憶して計算を効率化する。sizeof(char)=1byte=8bitは環境に依存しないのを利用し、(x,y)の判定はreadMat[(x+y*KinectSize.x)/8]の上から((x+y*KinectSize.x)%8)bit目(一番上を0bit目とする)に格納。
	if(drawMat==nullptr || readMat==nullptr){
		throw(std::runtime_error("Memory is not satisfied."));
	}
	for(int i=0;i<KinectSize.x*KinectSize.y;i++){
		drawMat[i]=0;
	}
	//boneのどこをどう繋げるかのデータ
	std::vector<std::pair<_JointType,_JointType>> bonePairs={
		std::make_pair<_JointType,_JointType>(JointType_Head,JointType_Neck),
		std::make_pair<_JointType,_JointType>(JointType_Neck,JointType_SpineShoulder),
		std::make_pair<_JointType,_JointType>(JointType_SpineShoulder,JointType_ShoulderRight),
		std::make_pair<_JointType,_JointType>(JointType_ShoulderRight,JointType_ElbowRight),
		std::make_pair<_JointType,_JointType>(JointType_ElbowRight,JointType_WristRight),
		std::make_pair<_JointType,_JointType>(JointType_WristRight,JointType_HandRight),
		std::make_pair<_JointType,_JointType>(JointType_HandRight,JointType_HandTipRight),
		std::make_pair<_JointType,_JointType>(JointType_HandRight,JointType_ThumbRight),
		std::make_pair<_JointType,_JointType>(JointType_SpineShoulder,JointType_ShoulderLeft),
		std::make_pair<_JointType,_JointType>(JointType_ShoulderLeft,JointType_ElbowLeft),
		std::make_pair<_JointType,_JointType>(JointType_ElbowLeft,JointType_WristLeft),
		std::make_pair<_JointType,_JointType>(JointType_WristLeft,JointType_HandLeft),
		std::make_pair<_JointType,_JointType>(JointType_HandLeft,JointType_HandTipLeft),
		std::make_pair<_JointType,_JointType>(JointType_HandLeft,JointType_ThumbLeft),
		std::make_pair<_JointType,_JointType>(JointType_SpineShoulder,JointType_SpineMid),
		std::make_pair<_JointType,_JointType>(JointType_SpineMid,JointType_SpineBase),
		std::make_pair<_JointType,_JointType>(JointType_SpineBase,JointType_HipRight),
		std::make_pair<_JointType,_JointType>(JointType_HipRight,JointType_KneeRight),
		std::make_pair<_JointType,_JointType>(JointType_KneeRight,JointType_AnkleRight),
		std::make_pair<_JointType,_JointType>(JointType_AnkleRight,JointType_FootRight),
		std::make_pair<_JointType,_JointType>(JointType_SpineBase,JointType_HipLeft),
		std::make_pair<_JointType,_JointType>(JointType_HipLeft,JointType_KneeLeft),
		std::make_pair<_JointType,_JointType>(JointType_KneeLeft,JointType_AnkleLeft),
		std::make_pair<_JointType,_JointType>(JointType_AnkleLeft,JointType_FootLeft)
	};

	//取得したデータを記録する場所
	bool fileWriteFlag=false;//ファイル入力をするかどうか
	int writeCount=0;//書き込んでいる時間の計測
	const int writeCountMax=30*30;//これ以上の時間書き込まないようにする
	std::ofstream writeFile;
	
	//記録した物を再生する際に用いるデータ
	bool playDataFlag=false;//再生するかどうか
	int playFlame=0;//今何フレーム目を再生しているか
	std::ifstream readFile;
	const int captureFps=30;//撮影データのfps
	const int drawFps=60;//描画時のfps
	double playRate=1.0;//再生速度
	
	//アプリケーション動作
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//ゲーム本体
		//キー情報更新
		input_update();
		
		//描画
		clsDx();
		//depth画像描画(データ記録時のみ描画する)
		if(!playDataFlag){
			for(int y=0;y<KinectSize.y;y++){
				for(int x=0;x<KinectSize.x;x++){
					int color=drawMat[x+y*KinectSize.x];
					int c=color*255/8000;//値を0~255の範囲内に収める。drawmat内にある数値は500～8000で、mm単位での物体までの距離である
					DrawPixel(x,y,GetColor(c,c,c));
				}
			}
		}
		//複数あるbodyそれぞれに対して処理を行う
		for(size_t j=0;j<BodyNum;j++){
			//各関節の位置の取得
			Vector2D jointsPos[JointType::JointType_Count];//関節のdepth画像描画位置
			Vector2D jointsXY[JointType::JointType_Count];//関節のxy画像描画位置
			Vector2D jointsZY[JointType::JointType_Count];//関節のzy画像描画位置
			for(size_t i=0;i<JointType::JointType_Count;i++){
				//depth画像に入れる関節の位置:jointsPos
				try{
					ICoordinateMapper *mapper;
					ErrorCheck(pSensor->get_CoordinateMapper(&mapper),"mapper failed\n");
					DepthSpacePoint point;//opencv系の座標。すなわちdxlibと同じ。
					//mapper->MapCameraPointToDepthSpace(joints[i].Position,&point);
					mapper->MapCameraPointToDepthSpace(jointPositions[j][i].GetCameraSpacePoint(),&point);
					jointsPos[i]=Vector2D(KinectSize.x-(int)point.X,(int)point.Y);//鏡像なので反転して取得
				} catch(const std::exception &e){
					printfDx(e.what());
				}
				//xy画像・zy画像に入れる関節の位置:jointsXY,jointsZY
				const double hAngle=70.0,vAngle=60.0;
				const float halfDepthRange=8.0/2;
				float posz=jointPositions[j][i].Z;
				if(posz==0.0){
					//0除算を防ぐ。本来は例外処理するべき
					posz=JointPosition::defaultfloat;
				}
				//xy画像の中心は(KinectSize.x/2,KinectSize.y*3/2)に描画
				jointsXY[i]=Vector2D((int)(KinectSize.x/2*jointPositions[j][i].X/jointPositions[j][i].Z/std::sin(hAngle/360*M_PI)),(int)(KinectSize.y/2*jointPositions[j][i].Y/jointPositions[j][i].Z/std::sin(vAngle/360*M_PI)));//通常の座標系における中心からの相対距離
				jointsXY[i]=Vector2D(jointsXY[i].x,-jointsXY[i].y)+Vector2D(KinectSize.x/2,KinectSize.y*3/2);//DXライブラリの座標系に変換し、更に絶対位置に変換
				//zy画像の中心は(KinectSize.x*3/2,KinectSize.y*3/2)に描画
				jointsZY[i]=Vector2D((int)(KinectSize.x*(halfDepthRange/2-jointPositions[j][i].Z)/halfDepthRange),(int)(KinectSize.x*jointPositions[j][i].Y/halfDepthRange));//現実世界の座標系での中心からの相対距離
				jointsZY[i]=Vector2D(jointsZY[i].x,-jointsZY[i].y)+KinectSize*3/2;//DXライブラリの座標系に変換し、更に絶対位置に変換
			}
			//各関節の描画
			for(size_t i=0;i<JointType::JointType_Count;i++){
				DrawCircle(jointsPos[i].x,jointsPos[i].y,circlesize,GetColor(0,255,0),FALSE);//depth画像
				DrawCircle(jointsXY[i].x,jointsXY[i].y,circlesize,GetColor(0,255,0),FALSE);//xy画像
				DrawCircle(jointsZY[i].x,jointsZY[i].y,circlesize,GetColor(0,255,0),FALSE);//zy座標
			}
			//各ボーンの描画
			for(const auto &pair:bonePairs){
				//depth画像
				Vector2D pos[2]={jointsPos[pair.first],jointsPos[pair.second]};
				DrawLine(pos[0].x,pos[0].y,pos[1].x,pos[1].y,GetColor(255,0,0),1);
				//xy画像
				Vector2D posXY[2]={jointsXY[pair.first],jointsXY[pair.second]};
				DrawLine(posXY[0].x,posXY[0].y,posXY[1].x,posXY[1].y,GetColor(255,0,0),1);
				//zy画像
				Vector2D posZY[2]={jointsZY[pair.first],jointsZY[pair.second]};
				DrawLine(posZY[0].x,posZY[0].y,posZY[1].x,posZY[1].y,GetColor(255,0,0),1);
			}
		}
/*
		//全てのbodyに対して、xy画像とzy画像を描画する
		for(auto pBody:pBodies){
			if(pBody!=nullptr){
				//現実世界における、各bodyのkinectからの座標を取得する。mm単位のdepth画像ではなく、skeltonから取得しているので単位はm。
				Joint joints[JointType::JointType_Count];
				pBody->GetJoints(JointType::JointType_Count,joints);
				//各関節点の描画位置を記録
				Vector2D jointsXY[JointType::JointType_Count];
				Vector2D jointsZY[JointType::JointType_Count];
				for(int i=0;i<JointType::JointType_Count;i++){
					const double hAngle=70.0,vAngle=60.0;
					const float halfDepthRange=8.0/2;
					//xy画像の中心は(KinectSize.x/2,KinectSize.y*3/2)に描画
					float posz=joints[i].Position.Z;
					if(posz==0.0){
						posz=0.001;//本来は例外処理をするべき
					} else{
						int a=0;
					}
					jointsXY[i]=Vector2D((int)(KinectSize.x/2*joints[i].Position.X/joints[i].Position.Z/std::sin(hAngle/360*M_PI)),(int)(KinectSize.y/2*joints[i].Position.Y/joints[i].Position.Z/std::sin(vAngle/360*M_PI)));//通常の座標系における中心からの相対距離
					jointsXY[i]=Vector2D(jointsXY[i].x,-jointsXY[i].y)+Vector2D(KinectSize.x/2,KinectSize.y*3/2);//DXライブラリの座標系に変換し、更に絶対位置に変換
					//zy画像の中心は(KinectSize.x*3/2,KinectSize.y*3/2)に描画
					jointsZY[i]=Vector2D((int)(KinectSize.x*(halfDepthRange/2-joints[i].Position.Z)/halfDepthRange),(int)(KinectSize.x*joints[i].Position.Y/halfDepthRange));//現実世界の座標系での中心からの相対距離
					jointsZY[i]=Vector2D(jointsZY[i].x,-jointsZY[i].y)+KinectSize*3/2;//DXライブラリの座標系に変換し、更に絶対位置に変換
				}
				//各関節の描画
				for(int i=0;i<JointType::JointType_Count;i++){
					DrawCircle(jointsXY[i].x,jointsXY[i].y,circlesize,GetColor(0,255,0),FALSE);
					DrawCircle(jointsZY[i].x,jointsZY[i].y,circlesize,GetColor(0,255,0),FALSE);
				}
				//各ボーンの描画
				for(const auto &pair:bonePairs){
					Vector2D posXY[2]={jointsXY[pair.first],jointsXY[pair.second]};
					Vector2D posZY[2]={jointsZY[pair.first],jointsZY[pair.second]};
					DrawLine(posXY[0].x,posXY[0].y,posXY[1].x,posXY[1].y,GetColor(255,0,0),1);
					DrawLine(posZY[0].x,posZY[0].y,posZY[1].x,posZY[1].y,GetColor(255,0,0),1);
				}
			}
		}
//*/
		
		//情報更新
		if(!playDataFlag){
			//情報の記録を行えるモードの時
			printfDx("RecordingDataMode\n");
			//depth
			//データの読み取り
			printfDx("depth:\n");
			try{
				IDepthFrame *pDepthFrame=nullptr;
				ErrorCheck(pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error\n");
				ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error\n");
				printfDx("success\n");
				//読み取りができたのでdrawMatを更新。鏡像なので左右反転させる。
				if(bufferMat!=nullptr){
					for(int y=0;y<KinectSize.y;y++){
						for(int x=0;x<KinectSize.x;x++){
							drawMat[(KinectSize.x-x-1)+y*KinectSize.x]=bufferMat[x+y*KinectSize.x];
						}
					}
				}
				pDepthFrame->Release();//bufferMatの参照が終了したのでpDepthFrameを開放する
			} catch(const std::exception &e){
				printfDx(e.what());
			}
			//body
			printfDx("body:\n");
			try{
				ErrorCheck(pBodyReader->AcquireLatestFrame(&pBodyFrame),"aquaire failed\n");//直近フレームのbodyデータの取得
				ErrorCheck(pBodyFrame->GetAndRefreshBodyData(6,pBodies),"access failed\n");//bodyデータをpBodiesに格納
				pBodyFrame->Release();//これ以降pBodyFrameは使わない
				printfDx("success\n");
				for(int j=0;j<BodyNum;j++){
					//関節の実座標位置をjointPositionsに格納
					Joint joints[JointType::JointType_Count];
					pBodies[j]->GetJoints(JointType::JointType_Count,joints);//関節位置の実座標の取得
					//jointPositionsに関節の実座標を格納
					for(int i=0;i<JointType_Count;i++){
						jointPositions[j][i]=JointPosition(joints[i].Position);
					}
				}
			} catch(const std::exception &e){
				printfDx(e.what());
			}
			//情報の記録をするかのフラグの更新
			if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
				fileWriteFlag=!fileWriteFlag;
				if(fileWriteFlag){
					//記録開始時はファイルを開き、writeCountを0にする
					writeCount=0;
					writeFile.open("SaveData/"+to_string_0d(0,3)+".txt",std::ios_base::trunc);
					if(!writeFile){
						//ファイルを開けなければ記録開始しない
						fileWriteFlag=false;
					}
				} else{
					//記録終了時はファイルを閉じる
					writeFile.close();
				}
			}
			//ファイル出力
			printfDx("fileWriteFlag:\n");
			printfDx((fileWriteFlag && !(!writeFile)) ? "true\n":"false\n");
			if(fileWriteFlag && !(!writeFile)){
				//書き込みすぎ判定
				writeCount++;
				if(writeCount>writeCountMax){
					fileWriteFlag=false;
				}
				//body位置の出力
				//形式は1行につき、1フレームでのjointPositions[i][j]の各要素を(X,Y,Z)という形式にして出力。
				for(int j=0;j<BodyNum;j++){
					for(int i=0;i<JointType_Count;i++){
						writeFile<<jointPositions[j][i].GetString();
					}
				}
				writeFile<<std::endl;//1フレーム内の全ての出力が終了したので改行を出力
			}
			//記録データ再生モードへの移行処理
			if(keyboard_get(KEY_INPUT_0)==1){
				readFile.open("SaveData/"+to_string_0d(0,3)+".txt");
				if(!readFile){
					//読み込み失敗時の処理
				}else{
					//読み込み成功時のみ、再生モードへ
					playDataFlag=true;
					playFlame=0;
				}
			}
		}else{
			//記録したものの再生を行うモード
			printfDx("PlayingDataMode\n");
			int a=playFlame*captureFps*playRate/drawFps;
			playFlame++;
			int b=playFlame*captureFps*playRate/drawFps;//この値がaに一致している時は読み込みは行わず、前フレームと同じ画像を描画する
			if(!readFile || a==b){
				//特に何もしない
			}else{
				//ファイルを1行読み込みながら、jointPositionsにデータを格納
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
					}else{
						if(inBracketsFlag){
							//()内読み取り時(=')'を探している)
							parts.push_back(ch);
							if(ch==')'){
								inBracketsFlag=!inBracketsFlag;
								//jointPositionsに格納
								if(bodyindex<BodyNum && jointindex<JointType_Count){
									jointPositions[bodyindex][jointindex]=JointPosition(parts);
									//index系の更新
									jointindex++;
									if(jointindex>=JointType_Count){
										jointindex=0;
										bodyindex++;
									}
								}
								parts.clear();
							}
						}else{
							//()外読み取り時(='('を探している)
							if(ch=='('){
								inBracketsFlag=!inBracketsFlag;
								parts.push_back(ch);
							}
						}
					}
				}
				//ファイル末尾に到達したら、再生モードは終了し記録モードに戻る
				if(ch==EOF){
					readFile.close();
					playDataFlag=!playDataFlag;
				}
			}
		}
		//再生モードにおける再生速度調整
		if(keyboard_get(KEY_INPUT_Z)>0){
			//遅くする
			playRate=std::fmax(0.1,playRate-0.05);
		} else if(keyboard_get(KEY_INPUT_X)>0){
			//1.0倍に戻す
			playRate=1.0;
		} else if(keyboard_get(KEY_INPUT_C)>0){
			//速くする
			playRate=playRate+0.05;
		}
		printfDx("playRate:\n%f",playRate);

		//終了検出
		if(keyboard_get(KEY_INPUT_ESCAPE)>0){
			break;
		}
	}

	//終了処理

	pSensor->Close();
	pSensor->Release();

	writeFile.close();
	readFile.close();
	//DeleteFontToHandle(font);

}

void Simulate(){
	BodySimulator bs;
	//アプリケーション動作
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//ゲーム本体
		//キー情報更新
		input_update();

		//描画
		clsDx();
		bs.Draw();

		//情報更新
		int index=bs.Update();

		//終了検出
		if(keyboard_get(KEY_INPUT_ESCAPE)>0 || index!=0){
			break;
		}
	}

}

int WINAPI WinMain(HINSTANCE,HINSTANCE,LPSTR,int){
	try{
		const Vector2D KinectSize(512,424);
		//dxライブラリの初期化
		//画面モードの設定(一応こんな感じ)
		SetGraphMode(KinectSize.x*2+500,KinectSize.y*2,16);
		//タイトルメニュー文字
		SetMainWindowText("TennisSupporter");
		//ウインドウサイズの変更
		SetWindowSizeExtendRate(0.5);
		//ウインドウサイズの変更をできるようにする
		SetWindowSizeChangeEnableFlag(TRUE);
		//アイコンの設定
		SetWindowIconID(101);
		//非アクティブ状態での処理の続行のフラグ
		SetAlwaysRunFlag(TRUE);

		if(ChangeWindowMode(TRUE) != 0) {
			throw(std::runtime_error("ChangeWindowMode(TRUE) failed."));
		}
		if(DxLib_Init() != 0) {
			throw(std::runtime_error("DxLib_Init() failed."));
		}
		if(SetDrawScreen(DX_SCREEN_BACK) != 0) {
			DxLib_End();
			throw(std::runtime_error("SetDrawScreen(DX_SCREEN_BACK) failed."));
		}

		//入力機構の初期化
		InitInputControler();

		//実行
		//DepthSimulate(false,KinectSize);
		//BodySimulate(KinectSize);
		Simulate();

		//終了処理
		DeleteInputControler();//入力機構の解放
		DxLib_End();


		return 0;
	} catch(const std::exception &e){
		assert(e.what());
		return 1;
	}
}

