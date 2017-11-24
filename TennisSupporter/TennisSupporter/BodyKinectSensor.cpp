#define _USE_MATH_DEFINES
#include<cmath>
#include"BodyKinectSensor.h"
#include"DxLib.h"
#include"input.h"

//-------------------BodyKinectSensor::JointPosition-------------------
const float BodyKinectSensor::JointPosition::defaultfloat=(float)0.0001;

//X,Y,Z�̒l�𒼐ړ��͂���
BodyKinectSensor::JointPosition::JointPosition(float i_X,float i_Y,float i_Z)
	:X(i_X),Y(i_Y),Z(i_Z){}

//_CameraSpacePoint��菉��������
BodyKinectSensor::JointPosition::JointPosition(_CameraSpacePoint pos)
	:X(pos.X),Y(pos.Y),Z(pos.Z){}

//"(X,Y,Z)"�Ƃ����`���̕������ǂݎ���ď���������
BodyKinectSensor::JointPosition::JointPosition(const std::string &str){
	unsigned int size=str.size();
	int process=0;//0:"("�̓ǂݎ�� 1:X�̓ǂݎ�� 2:Y�̓ǂݎ�� 3:Z�̓ǂݎ��
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
		//�������傫���������̏���
		JointPosition j=JointPosition();
		X=j.X;
		Y=j.Y;
		Z=j.Z;
	}
}

//==�̎���
bool BodyKinectSensor::JointPosition::operator==(const JointPosition &otherobj)const{
	return (this->X==otherobj.X && this->Y==otherobj.Y && this->X==otherobj.X);
}

//"(X,Y,Z)"�Ƃ�����������o�͂���
std::string BodyKinectSensor::JointPosition::GetString()const{
	return "("+std::to_string(X)+","+std::to_string(Y)+","+std::to_string(Z)+")";
}

//_CameraSpacePoint���쐬
_CameraSpacePoint BodyKinectSensor::JointPosition::GetCameraSpacePoint()const{
	_CameraSpacePoint c;
	c.X=X;
	c.Y=Y;
	c.Z=Z;
	return c;
}

//Joint::Position��JointPosition�̂悤�ɂȂ��Ă���Joint��Ԃ��B���̑��̗v�f�̓e�L�g�[�B
Joint BodyKinectSensor::JointPosition::CreateJoint()const{
	return CreateJoint(JointType_SpineBase);
}

Joint BodyKinectSensor::JointPosition::CreateJoint(_JointType type)const{
	return CreateJoint(type,TrackingState_NotTracked);
}

Joint BodyKinectSensor::JointPosition::CreateJoint(_TrackingState state)const{
	return CreateJoint(JointType_SpineBase,state);
}

Joint BodyKinectSensor::JointPosition::CreateJoint(_JointType type,_TrackingState state)const{
	Joint j;
	j.JointType=type;
	j.TrackingState=state;
	j.Position.X=X;
	j.Position.Y=Y;
	j.Position.Z=Z;
	return j;
}

//��������Q�̕ʂ�JointPosition�ւ̃x�N�g���̌����p�x�����߂�(0�`180�x)
double BodyKinectSensor::JointPosition::CalculateAngle(JointPosition v1,JointPosition v2)const{
	//�R�������[�N���b�h���ʂł̓��ς�p���ċ��߂�
	double innerProduct=(double)((v1.X-this->X)*(v2.X-this->X)+(v1.Y-this->Y)*(v2.Y-this->Y)+(v1.Z-this->Z)*(v2.Z-this->Z));
	double distanceV1=std::sqrt((double)((v1.X-this->X)*(v1.X-this->X)+(v1.Y-this->Y)*(v1.Y-this->Y)+(v1.Z-this->Z)*(v1.Z-this->Z)));
	double distanceV2=std::sqrt((double)((v2.X-this->X)*(v2.X-this->X)+(v2.Y-this->Y)*(v2.Y-this->Y)+(v2.Z-this->Z)*(v2.Z-this->Z)));
	//���ς̌���:innerProduct=distanceV1*distanceV2*cos(rad)
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

//-------------------BodyKinectSensor-------------------
const std::vector<std::pair<_JointType,_JointType>> BodyKinectSensor::bonePairs={
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

bool BodyKinectSensor::BodyIndexSignificance(size_t bodyIndex)const{
	//�z��O�Q�Ƃ̏���
	if(bodyIndex>=bodyNum){
		return false;
	}
	//�{����
	bool flag=false;
	for(size_t i=0;i<JointType_Count;i++){
		if(!(m_jointPositions[bodyIndex][i]==JointPosition())){
			flag=true;
			break;
		}
	}
	return flag;
}

BodyKinectSensor::BodyKinectSensor(IKinectSensor *pSensor){
	//source
	IBodyFrameSource *pBodySource;
	ErrorCheck(pSensor->get_BodyFrameSource(&pBodySource),"You can't get source.");
	//reader
	ErrorCheck(pBodySource->OpenReader(&m_pBodyReader),"You can't open reader.");
}

BodyKinectSensor::~BodyKinectSensor(){}

void BodyKinectSensor::OutputJointPoitions(std::ofstream &writeFile)const{
	if(!(!writeFile)){
		//body�ʒu�̏o��
		//�`����1�s�ɂ��A1�t���[���ł�jointPositions[i][j]�̊e�v�f��(X,Y,Z)�Ƃ����`���ɂ��ďo�́B
		for(int j=0;j<bodyNum;j++){
			for(int i=0;i<JointType_Count;i++){
				writeFile<<m_jointPositions[j][i].GetString();
			}
		}
		writeFile<<std::endl;//1�t���[�����̑S�Ă̏o�͂��I�������̂ŉ��s���o��
	}

}

int BodyKinectSensor::Update(){
	//�ꎞ�ϐ��̗p��
	IBodyFrame *pBodyFrame=nullptr;
	IBody *pBodies[bodyNum];
	for(size_t i=0;i<bodyNum;i++){
		pBodies[i]=nullptr;
	}
	//�X�V���
	try{
		ErrorCheck(m_pBodyReader->AcquireLatestFrame(&pBodyFrame),"aquaire failed\n");//���߃t���[����body�f�[�^�̎擾
		ErrorCheck(pBodyFrame->GetAndRefreshBodyData(6,pBodies),"access failed\n");//body�f�[�^��pBodies�Ɋi�[
		pBodyFrame->Release();//����ȍ~pBodyFrame�͎g��Ȃ�
		printfDx("success\n");
		for(int j=0;j<bodyNum;j++){
			//�֐߂̎����W�ʒu��jointPositions�Ɋi�[
			Joint joints[JointType::JointType_Count];
			pBodies[j]->GetJoints(JointType::JointType_Count,joints);//�֐߈ʒu�̎����W�̎擾
			//jointPositions�Ɋ֐߂̎����W���i�[
			for(int i=0;i<JointType_Count;i++){
				m_jointPositions[j][i]=JointPosition(joints[i].Position);
			}
		}
	} catch(const std::exception &e){
		printfDx(e.what());
	}

	return 0;//�Ԃ��l�̓e�L�g�[
}

int BodyKinectSensor::Update(std::ifstream &readFile){
	size_t bodyindex=0,jointindex=0;
	std::string parts="";
	parts.reserve(50);
	bool inBracketsFlag=false;//����"(~~)"�̒���ǂݎ���Ă��邩�ǂ���
	char ch='a';//�����l�̓e�L�g�[
	while(true){
		ch=readFile.get();
		if(ch=='\n' || ch==EOF){
			//1�s�ǂݏI��邩�A�t�@�C���̖������B��
			break;
		} else{
			if(inBracketsFlag){
				//()���ǂݎ�莞(=')'��T���Ă���)
				parts.push_back(ch);
				if(ch==')'){
					inBracketsFlag=!inBracketsFlag;
					//jointPositions�Ɋi�[
					if(bodyindex<bodyNum && jointindex<JointType_Count){
						m_jointPositions[bodyindex][jointindex]=JointPosition(parts);
						//index�n�̍X�V
						jointindex++;
						if(jointindex>=JointType_Count){
							jointindex=0;
							bodyindex++;
						}
					}
					parts.clear();
				}
			} else{
				//()�O�ǂݎ�莞(='('��T���Ă���)
				if(ch=='('){
					inBracketsFlag=!inBracketsFlag;
					parts.push_back(ch);
				}
			}
		}
	}

	//�l��Ԃ��Bch��EOF���ǂ�����`����
	if(ch==EOF){
		return 1;//�f�[�^�ǂݍ��݂��I�����鎖��`����
	} else{
		return 0;
	}
}

int BodyKinectSensor::Update(const std::vector<std::vector<JointPosition>> &frameData){
	for(size_t i=0,topsize=frameData.size();i<bodyNum;i++){
		//�z��̑傫�����L�^
		size_t secondsize=0;
		if(i<topsize){
			secondsize=frameData[i].size();
		}
		//�i�[
		for(size_t j=0;j<JointType_Count;j++){
			if(i<topsize && j<secondsize){
				m_jointPositions[i][j]=frameData[i][j];
			} else{
				//frameData�̔z��O�Q�Ƃ��N���鎞�̓S�~�f�[�^���i�[
				m_jointPositions[i][j]=JointPosition();
			}
		}
	}
	return 0;
}

void BodyKinectSensor::Draw(IKinectSensor *pSensor,Vector2D depthPos,Vector2D depthSize,Vector2D xyPos,Vector2D xySize,Vector2D zyPos,Vector2D zySize)const{
	const int circlesize=3;//�֐߂�\���~�̔��a
	//��������body���ꂼ��ɑ΂��ď������s��
	for(size_t j=0;j<bodyNum;j++){
		//�e�֐߂̈ʒu�̎擾
		Vector2D jointsPos[JointType::JointType_Count];//�֐߂�depth�摜�`��ʒu
		Vector2D jointsXY[JointType::JointType_Count];//�֐߂�xy�摜�`��ʒu
		Vector2D jointsZY[JointType::JointType_Count];//�֐߂�zy�摜�`��ʒu
		for(size_t i=0;i<JointType::JointType_Count;i++){
			//depth�摜�ɓ����֐߂̈ʒu:jointsPos
			try{
				ICoordinateMapper *mapper;
				ErrorCheck(pSensor->get_CoordinateMapper(&mapper),"mapper failed\n");
				DepthSpacePoint point;//opencv�n�̍��W�B���Ȃ킿dxlib�Ɠ����B
				//mapper->MapCameraPointToDepthSpace(joints[i].Position,&point);
				mapper->MapCameraPointToDepthSpace(m_jointPositions[j][i].GetCameraSpacePoint(),&point);
				jointsPos[i]=Vector2D(depthSize.x-(int)point.X,(int)point.Y);//�����Ȃ̂Ŕ��]���Ď擾�B���ォ��̑��Έʒu
				jointsPos[i]=jointsPos[i]+(depthPos-depthSize/2);
			} catch(const std::exception &e){
				printfDx(e.what());
			}
			//xy�摜�Ezy�摜�ɓ����֐߂̈ʒu:jointsXY,jointsZY
			const double hAngle=70.0,vAngle=60.0;
			const float halfDepthRange=8.0/2;
			float posz=m_jointPositions[j][i].Z;
			if(posz==0.0){
				//0���Z��h���B�{���͗�O��������ׂ�
				posz=JointPosition::defaultfloat;
			}
			//xy�摜�̒��S��(xySize.x/2,xySize.y*3/2)�ɕ`��
			//jointsXY[i]=Vector2D((int)(xySize.x/2*m_jointPositions[j][i].X/m_jointPositions[j][i].Z/std::sin(hAngle/360*M_PI)),(int)(xySize.y/2*m_jointPositions[j][i].Y/m_jointPositions[j][i].Z/std::sin(vAngle/360*M_PI)));//�ʏ�̍��W�n�ɂ����钆�S����̑��Ε`�拗��(�񐳎ˉe)
			jointsXY[i]=Vector2D((int)(xySize.x*m_jointPositions[j][i].X/halfDepthRange),(int)(xySize.x*m_jointPositions[j][i].Y/halfDepthRange));//�ʏ�̍��W�n�ɂ����钆�S����̑��Ε`�拗��(���ˉe)
			jointsXY[i]=Vector2D(jointsXY[i].x,-jointsXY[i].y)+xyPos;//DX���C�u�����̍��W�n�ɕϊ����A�X�ɐ�Έʒu�ɕϊ�
			//zy�摜�̒��S��(zySize.x*3/2,zySize.y*3/2)�ɕ`��
			jointsZY[i]=Vector2D((int)(zySize.x*(halfDepthRange/2-m_jointPositions[j][i].Z)/halfDepthRange),(int)(zySize.x*m_jointPositions[j][i].Y/halfDepthRange));//�ʏ�̍��W�n�ł̒��S����̑��Ε`�拗��
			jointsZY[i]=Vector2D(jointsZY[i].x,-jointsZY[i].y)+zyPos;//DX���C�u�����̍��W�n�ɕϊ����A�X�ɐ�Έʒu�ɕϊ�
		}
		//�e�֐߂̕`��
		for(size_t i=0;i<JointType::JointType_Count;i++){
			DrawCircle(jointsPos[i].x,jointsPos[i].y,circlesize,GetColor(0,255,0),FALSE);//depth�摜
			DrawCircle(jointsXY[i].x,jointsXY[i].y,circlesize,GetColor(0,255,0),FALSE);//xy�摜
			DrawCircle(jointsZY[i].x,jointsZY[i].y,circlesize,GetColor(0,255,0),FALSE);//zy���W
		}
		//�e�{�[���̕`��
		for(const auto &pair:bonePairs){
			//depth�摜
			Vector2D pos[2]={jointsPos[pair.first],jointsPos[pair.second]};
			DrawLine(pos[0].x,pos[0].y,pos[1].x,pos[1].y,GetColor(255,0,0),1);
			//xy�摜
			Vector2D posXY[2]={jointsXY[pair.first],jointsXY[pair.second]};
			DrawLine(posXY[0].x,posXY[0].y,posXY[1].x,posXY[1].y,GetColor(255,0,0),1);
			//zy�摜
			Vector2D posZY[2]={jointsZY[pair.first],jointsZY[pair.second]};
			DrawLine(posZY[0].x,posZY[0].y,posZY[1].x,posZY[1].y,GetColor(255,0,0),1);
		}
	}
}

BodyKinectSensor::JointPosition BodyKinectSensor::GetJointPosition(_JointType jointType)const{
	for(size_t i=0;i<bodyNum;i++){
		if(BodyIndexSignificance(i)){
			return GetJointPosition(i,jointType);
		}
	}
	//���������͗�O�����Bbody���P��������Ȃ��������̏���
	return JointPosition();
}

BodyKinectSensor::JointPosition BodyKinectSensor::GetJointPosition(size_t bodyIndex,_JointType jointType)const{
	return m_jointPositions[bodyIndex][jointType];
}

double BodyKinectSensor::GetRadian(_JointType edge,_JointType point1,_JointType point2)const{
	for(size_t i=0;i<bodyNum;i++){
		if(BodyIndexSignificance(i)){
			return GetRadian(i,edge,point1,point2);
		}
	}
	//���������͗�O�����Bbody���P��������Ȃ��������̏���
	return 0.0;
}

double BodyKinectSensor::GetRadian(size_t bodyIndex,_JointType edge,_JointType point1,_JointType point2)const{
	return m_jointPositions[bodyIndex][edge].CalculateAngle(m_jointPositions[bodyIndex][point1],m_jointPositions[bodyIndex][point2]);
}
