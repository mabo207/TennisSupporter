#define _USE_MATH_DEFINES
#include<cmath>
#include"DxLib.h"
#include"input.h"
#include"IBodyKinectSensor.h"

//-------------------IBodyKinectSensor::JointPosition-------------------
const float IBodyKinectSensor::JointPosition::defaultfloat=(float)0.0000;

//X,Y,Zの値を直接入力する
IBodyKinectSensor::JointPosition::JointPosition(float i_X,float i_Y,float i_Z)
	:X(i_X),Y(i_Y),Z(i_Z){}

//_CameraSpacePointより初期化する
IBodyKinectSensor::JointPosition::JointPosition(_CameraSpacePoint pos)
	:X(pos.X),Y(pos.Y),Z(pos.Z){}

//"(X,Y,Z)"という形式の文字列を読み取って初期化する
IBodyKinectSensor::JointPosition::JointPosition(const std::string &str){
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
	} catch(const std::exception &e){
		//桁数が大きすぎた時の処理
		JointPosition j=JointPosition();
		X=j.X;
		Y=j.Y;
		Z=j.Z;
	}
}

//==の実装
bool IBodyKinectSensor::JointPosition::operator==(const JointPosition &otherobj)const{
	return (this->X==otherobj.X && this->Y==otherobj.Y && this->Z==otherobj.Z);
}

//"(X,Y,Z)"という文字列を出力する
std::string IBodyKinectSensor::JointPosition::GetString()const{
	return "("+std::to_string(X)+","+std::to_string(Y)+","+std::to_string(Z)+")";
}

//_CameraSpacePointを作成
_CameraSpacePoint IBodyKinectSensor::JointPosition::GetCameraSpacePoint()const{
	_CameraSpacePoint c;
	c.X=X;
	c.Y=Y;
	c.Z=Z;
	return c;
}

//Joint::PositionがJointPositionのようになっているJointを返す。その他の要素はテキトー。
Joint IBodyKinectSensor::JointPosition::CreateJoint()const{
	return CreateJoint(JointType_SpineBase);
}

Joint IBodyKinectSensor::JointPosition::CreateJoint(_JointType type)const{
	return CreateJoint(type,TrackingState_NotTracked);
}

Joint IBodyKinectSensor::JointPosition::CreateJoint(_TrackingState state)const{
	return CreateJoint(JointType_SpineBase,state);
}

Joint IBodyKinectSensor::JointPosition::CreateJoint(_JointType type,_TrackingState state)const{
	Joint j;
	j.JointType=type;
	j.TrackingState=state;
	j.Position.X=X;
	j.Position.Y=Y;
	j.Position.Z=Z;
	return j;
}

//自分から２つの別のJointPositionへのベクトルの交わる角度を求める(0～180度)
double IBodyKinectSensor::JointPosition::CalculateAngle(JointPosition v1,JointPosition v2)const{
	//３次元ユークリッド平面での内積を用いて求める
	double innerProduct=(double)((v1.X-this->X)*(v2.X-this->X)+(v1.Y-this->Y)*(v2.Y-this->Y)+(v1.Z-this->Z)*(v2.Z-this->Z));
	double distanceV1=std::sqrt((double)((v1.X-this->X)*(v1.X-this->X)+(v1.Y-this->Y)*(v1.Y-this->Y)+(v1.Z-this->Z)*(v1.Z-this->Z)));
	double distanceV2=std::sqrt((double)((v2.X-this->X)*(v2.X-this->X)+(v2.Y-this->Y)*(v2.Y-this->Y)+(v2.Z-this->Z)*(v2.Z-this->Z)));
	//内積の公式:innerProduct=distanceV1*distanceV2*cos(rad)
	double rad;
	try{
		rad=std::acos(innerProduct/distanceV1/distanceV2);
	} catch(const std::exception &e){
		printfDx(e.what());
		printfDx("\n");
		rad=0.0;
	}
	return rad;
}

//-------------------IBodyKinectSensor-------------------
const std::vector<std::pair<_JointType,_JointType>> IBodyKinectSensor::bonePairs={
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

bool IBodyKinectSensor::BodyIndexSignificance(size_t bodyIndex)const{
	//配列外参照の処理
	if(bodyIndex>=bodyNum){
		return false;
	}
	//本処理
	bool flag=false;
	for(size_t i=0;i<JointType_Count;i++){
		if(!(m_jointPositions[bodyIndex][i]==JointPosition())){
			flag=true;
			break;
		}
	}
	return flag;
}

IBodyKinectSensor::IBodyKinectSensor(){}

IBodyKinectSensor::~IBodyKinectSensor(){}

void IBodyKinectSensor::OutputJointPoitions(std::ofstream &writeFile)const{
	if(!(!writeFile)){
		//body位置の出力
		//形式は1行につき、1フレームでのjointPositions[i][j]の各要素を(X,Y,Z)という形式にして出力。
		for(int j=0;j<bodyNum;j++){
			for(int i=0;i<JointType_Count;i++){
				writeFile<<m_jointPositions[j][i].GetString();
			}
		}
		writeFile<<std::endl;//1フレーム内の全ての出力が終了したので改行を出力
	}

}

void IBodyKinectSensor::Draw(IKinectSensor *pSensor,Vector2D depthPos,Vector2D depthSize,Vector2D xyPos,Vector2D xySize,Vector2D zyPos,Vector2D zySize)const{
	const int circlesize=6;//関節を表す円の半径
						   //複数あるbodyそれぞれに対して処理を行う
	for(size_t j=0;j<bodyNum;j++){
		//各関節の位置の取得
		Vector2D jointsPos[JointType::JointType_Count];//関節のdepth画像描画位置
		Vector2D jointsXY[JointType::JointType_Count];//関節のxy画像描画位置
		Vector2D jointsZY[JointType::JointType_Count];//関節のzy画像描画位置
		for(size_t i=0;i<JointType::JointType_Count;i++){
			//depth画像に入れる関節の位置:jointsPos
			if(pSensor!=nullptr){
				try{
					ICoordinateMapper *mapper;
					ErrorCheck(pSensor->get_CoordinateMapper(&mapper),"mapper failed\n");
					DepthSpacePoint point;//opencv系の座標。すなわちdxlibと同じ。
										  //mapper->MapCameraPointToDepthSpace(joints[i].Position,&point);
					mapper->MapCameraPointToDepthSpace(m_jointPositions[j][i].GetCameraSpacePoint(),&point);
					jointsPos[i]=Vector2D(depthSize.x-(int)point.X,(int)point.Y);//鏡像なので反転して取得。左上からの相対位置
					jointsPos[i]=jointsPos[i]+(depthPos-depthSize/2);
				} catch(const std::exception &e){
					printfDx(e.what());
				}
			}
			//xy画像・zy画像に入れる関節の位置:jointsXY,jointsZY
			const double hAngle=70.0,vAngle=60.0;
			const float halfDepthRange=8.0/2;
			float posz=m_jointPositions[j][i].Z;
			if(posz==0.0){
				//0除算を防ぐ。本来は例外処理するべき
				posz=JointPosition::defaultfloat;
			}
			//xy画像の中心は(xySize.x/2,xySize.y*3/2)に描画
			//jointsXY[i]=Vector2D((int)(xySize.x/2*m_jointPositions[j][i].X/m_jointPositions[j][i].Z/std::sin(hAngle/360*M_PI)),(int)(xySize.y/2*m_jointPositions[j][i].Y/m_jointPositions[j][i].Z/std::sin(vAngle/360*M_PI)));//通常の座標系における中心からの相対描画距離(非正射影)
			jointsXY[i]=Vector2D((int)(xySize.x*m_jointPositions[j][i].X/halfDepthRange),(int)(xySize.x*m_jointPositions[j][i].Y/halfDepthRange));//通常の座標系における中心からの相対描画距離(正射影)
			jointsXY[i]=Vector2D(jointsXY[i].x,-jointsXY[i].y)+xyPos;//DXライブラリの座標系に変換し、更に絶対位置に変換
																	 //zy画像の中心は(zySize.x*3/2,zySize.y*3/2)に描画
			jointsZY[i]=Vector2D((int)(zySize.x*(halfDepthRange/2-m_jointPositions[j][i].Z)/halfDepthRange),(int)(zySize.x*m_jointPositions[j][i].Y/halfDepthRange));//通常の座標系での中心からの相対描画距離
			jointsZY[i]=Vector2D(jointsZY[i].x,-jointsZY[i].y)+zyPos;//DXライブラリの座標系に変換し、更に絶対位置に変換
		}
		//各関節の描画
		for(size_t i=0;i<JointType::JointType_Count;i++){
			unsigned int color=GetColor(0,255,0);
			if(i==JointType_ElbowRight || i==JointType_AnkleRight || i==JointType_FootRight || i==JointType_HandRight || i==JointType_HandTipRight || i==JointType_HipRight || i==JointType_KneeRight || i==JointType_ShoulderRight || i==JointType_ThumbRight || i==JointType_WristRight){
				color=GetColor(0,128,255);
			}
			if(pSensor!=nullptr){
				DrawCircle(jointsPos[i].x,jointsPos[i].y,circlesize,color,TRUE);//depth画像
			}
			DrawCircle(jointsXY[i].x,jointsXY[i].y,circlesize,color,FALSE);//xy画像
			DrawCircle(jointsZY[i].x,jointsZY[i].y,circlesize,color,FALSE);//zy座標
		}
		//各ボーンの描画
		for(const auto &pair:bonePairs){
			//depth画像
			if(pSensor!=nullptr){
				Vector2D pos[2]={jointsPos[pair.first],jointsPos[pair.second]};
				DrawLine(pos[0].x,pos[0].y,pos[1].x,pos[1].y,GetColor(255,0,0),1);
			}
			//xy画像
			Vector2D posXY[2]={jointsXY[pair.first],jointsXY[pair.second]};
			DrawLine(posXY[0].x,posXY[0].y,posXY[1].x,posXY[1].y,GetColor(255,0,0),1);
			//zy画像
			Vector2D posZY[2]={jointsZY[pair.first],jointsZY[pair.second]};
			DrawLine(posZY[0].x,posZY[0].y,posZY[1].x,posZY[1].y,GetColor(255,0,0),1);
		}
	}
}

IBodyKinectSensor::JointPosition IBodyKinectSensor::GetJointPosition(_JointType jointType)const{
	for(size_t i=0;i<bodyNum;i++){
		if(BodyIndexSignificance(i)){
			return GetJointPosition(i,jointType);
		}
	}
	//ここから先は例外処理。bodyが１つも見つからなかった時の処理
	return JointPosition();
}

IBodyKinectSensor::JointPosition IBodyKinectSensor::GetJointPosition(size_t bodyIndex,_JointType jointType)const{
	return m_jointPositions[bodyIndex][jointType];
}

double IBodyKinectSensor::GetRadian(_JointType edge,_JointType point1,_JointType point2)const{
	for(size_t i=0;i<bodyNum;i++){
		if(BodyIndexSignificance(i)){
			return GetRadian(i,edge,point1,point2);
		}
	}
	//ここから先は例外処理。bodyが１つも見つからなかった時の処理
	return 0.0;
}

double IBodyKinectSensor::GetRadian(size_t bodyIndex,_JointType edge,_JointType point1,_JointType point2)const{
	return m_jointPositions[bodyIndex][edge].CalculateAngle(m_jointPositions[bodyIndex][point1],m_jointPositions[bodyIndex][point2]);
}
