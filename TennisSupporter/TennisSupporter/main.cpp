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
	//X,Y,Z�̒l�𒼐ړ��͂���
	JointPosition(float i_X=defaultfloat,float i_Y=defaultfloat,float i_Z=defaultfloat)
		:X(i_X),Y(i_Y),Z(i_Z){}
	//_CameraSpacePoint��菉��������
	JointPosition(_CameraSpacePoint pos)
		:X(pos.X),Y(pos.Y),Z(pos.Z){}
	//"(X,Y,Z)"�Ƃ����`���̕������ǂݎ���ď���������
	JointPosition(const std::string &str){
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
		}catch(const std::exception &e){
			//�������傫���������̏���
			JointPosition j=JointPosition();
			X=j.X;
			Y=j.Y;
			Z=j.Z;
		}
	}
	//"(X,Y,Z)"�Ƃ�����������o�͂���
	std::string GetString()const{
		return "("+std::to_string(X)+","+std::to_string(Y)+","+std::to_string(Z)+")";
	}
	//_CameraSpacePoint���쐬
	_CameraSpacePoint GetCameraSpacePoint()const{
		_CameraSpacePoint c;
		c.X=X;
		c.Y=Y;
		c.Z=Z;
		return c;
	}
	//Joint::Position��JointPosition�̂悤�ɂȂ��Ă���Joint��Ԃ��B���̑��̗v�f�̓e�L�g�[�B
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
	//kinect�̏�����
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
	unsigned short *bufferMat=nullptr;//���擾�p�|�C���^�iKinect�����擾�̂��߂Ɋm�ۂ����������ɃA�N�Z�X����̂ŁA���t���[���|�C���^�̒l���ς��j
	unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//���O�ɓǂݎ����depth�摜��ێ�����z��
	unsigned char *readMat=new unsigned char[KinectSize.x*KinectSize.y/8];//���̌��o�̍ۂɊ��ɕ��̂����邩�𒲂ׂ��s�N�Z������0(false)��1(true)�ŋL�����Čv�Z������������Bsizeof(char)=1byte=8bit�͊��Ɉˑ����Ȃ��̂𗘗p���A(x,y)�̔����readMat[(x+y*KinectSize.x)/8]�̏ォ��((x+y*KinectSize.x)%8)bit��(��ԏ��0bit�ڂƂ���)�Ɋi�[�B
	if(drawMat==nullptr || readMat==nullptr){
		throw(std::runtime_error("Memory is not satisfied."));
	}
	for(int i=0;i<KinectSize.x*KinectSize.y;i++){
		drawMat[i]=0;
	}
	std::vector<std::set<int>> contoursVec={};//�֊s���ꗗ

	//�A�v���P�[�V��������
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//�Q�[���{��
		//�L�[���X�V
		input_update();
		//�`��
		clsDx();
		const unsigned short *adress1=bufferMat;
		printfDx("BufferMat:%x\ncontoursVec size:%d\n",bufferMat,contoursVec.size());
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				int color=drawMat[x+y*KinectSize.x];
				int c=color*255/8000;//�l��0~255�͈͓̔��Ɏ��߂�Bdrawmat���ɂ��鐔�l��500�`8000�ŁAmm�P�ʂł̕��̂܂ł̋����ł���
				DrawPixel(x,y,GetColor(c,c,c));
			}
		}
		//�֊s���`��
		for(const std::set<int> &set:contoursVec){
			for(const int &index:set){
				int x=index%KinectSize.x;
				int y=index/KinectSize.x;
				DrawPixel(x,y,GetColor(0,0,255));
			}
		}
		//�v�Z����
		//�f�[�^�̓ǂݎ��
		try{
			IDepthFrame *pDepthFrame=nullptr;
			ErrorCheck(pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error");
			ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error");
			printfDx("success");
			//�ǂݎ�肪�ł����̂�drawMat���X�V�B�����Ȃ̂ō��E���]������B
			const unsigned short *adress2=bufferMat;
			if(bufferMat!=nullptr){
				for(int y=0;y<KinectSize.y;y++){
					for(int x=0;x<KinectSize.x;x++){
						drawMat[(KinectSize.x-x-1)+y*KinectSize.x]=bufferMat[x+y*KinectSize.x];
					}
				}
			}
			pDepthFrame->Release();//bufferMat�̎Q�Ƃ��I�������̂�pDepthFrame���J������
		} catch(const std::exception &e){
			printfDx(e.what());
		}

		//���̌��o
		//�F���ł��Ȃ������s�N�Z��������depth��0�ɂ���B
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				if(drawMat[x+y*KinectSize.x]>8000){
					drawMat[x+y*KinectSize.x]=0;
				}
			}
		}
		if(objectcheck){
			//readMat������
			for(int i=0;i<KinectSize.x*KinectSize.y/8;i++){
				readMat[i]=0;
			}
			//contoursVec������
			contoursVec.clear();
			//�萔��`
			const int acceptdistance=300;//50mm�ȓ���depth�����̈Ⴂ������s�N�Z���ɐi��ł���
										 //���̂����o�ł����s�N�Z�������o���A���̔F�������Ă���
			for(int i=0;i<KinectSize.x*KinectSize.y;i++){
				bool insertflag=true;//���ɒ��ׂ镨�̂�contoursVec�ɒǉ����邩�ǂ���
									 //�܂����̌��o������Ă��炸�A���̌��o�����s�N�Z������������
				if(drawMat[i]>0 && (readMat[i/8] & 1<<(7-i%8))==0){
					//���̃s�N�Z�����玞�v���ɋ������߂��s�N�Z����T���Ă������ƂŁA���̗̂֊s�����߂�
					int index=i;
					std::set<int> contours={};//�֊s��\���s�N�Z���̏W��
					int indextable[8][2]={{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};
					int finalJ=3;//��������indextable�������do~while���[�v���œ����ӏ�����n�܂�Ȃ��悤�Ɂi�Ώ̂ȓ_�̎��̓_����n�܂�悤�Ɂj�A���O�łǂ�indextable���������������L�^���Ă����B�����l�͂Ԃ����Ⴏ�Ȃ�ł��悢�B
					do{
						//���݂�x,y���W�����߂�
						int x=index%KinectSize.x,y=index/KinectSize.x;
						//����8�s�N�Z���̂ǂ���depth�����̋߂��s�N�Z�������邩����������index�̍X�V
						for(int j=(finalJ+5)%8;j!=(finalJ+4)%8;j=(j+1)%8){
							int xx=x+indextable[j][0];
							int yy=y+indextable[j][1];
							//(xx,yy)�̃s�N�Z�������݂��Ă��邩�̔���
							if(xx>=0 && xx<KinectSize.x && yy>=0 && yy<KinectSize.y){
								//depth�������߂����̔���
								int nextindex=xx+yy*KinectSize.x;//���ׂ�s�N�Z���ɑΉ�����z��ԍ�
								int d1=drawMat[index];
								int d2=drawMat[nextindex];
								if(std::abs(drawMat[index]-drawMat[nextindex])<=acceptdistance){
									index=nextindex;
									finalJ=j;
									break;
								}
							}
						}
						//�֊s�W���ɏ����i�[����
						contours.insert(index);
						//readMat���X�V
						unsigned char byte=1<<(7-index%8);
						if((readMat[index/8] & byte)!=0){
							//���̃s�N�Z��������readMat��0�Ȃ�A���̃s�N�Z���͊��Ɍ��o���ꂽ�ʂ̕��̂ɑ����Ă���̂ŁA����ȏ㒲�ׂĂ����ʂɂȂ�B�֊s�W���ɓ���Ȃ������`���T���I������B
							insertflag=false;
							break;
						} else{
							readMat[index/8]=readMat[index/8] | byte;
						}
					} while(index!=i);
					if(insertflag){
						//���ɒ��ׂ����̂łȂ����
						if(contours.size()>8){
							//������x�̑傫���������
							contoursVec.push_back(contours);
						}
					}
				}
			}
		}
		//�I�����o
		if(keyboard_get(KEY_INPUT_ESCAPE)>0){
			break;
		}
	}

	//�I������
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
	//�t�H���g�쐬
	//int font=CreateFontToHandle("���C���I",circlesize*3/2,1,-1);

	//kinect�̏�����
	//body�ɂ���
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
	
	//���t���[���X�V�������
	//Body
	const size_t BodyNum=6;
	IBodyFrame *pBodyFrame=nullptr;
	IBody *pBodies[BodyNum];
	for(size_t i=0;i<BodyNum;i++){
		pBodies[i]=nullptr;
	}
	//Joint::Position
	JointPosition jointPositions[BodyNum][JointType_Count];

	//�����̂��߂�depth�摜���\��
	//source
	IDepthFrameSource *pDepthSource;
	ErrorCheck(pSensor->get_DepthFrameSource(&pDepthSource),"You can't get source.");
	//reader
	IDepthFrameReader *pDepthReader;
	ErrorCheck(pDepthSource->OpenReader(&pDepthReader),"You can't open reader.");
	//memory
	unsigned int bufferSize=KinectSize.x*KinectSize.y*sizeof(unsigned short);
	unsigned short *bufferMat=nullptr;//���擾�p�|�C���^�iKinect�����擾�̂��߂Ɋm�ۂ����������ɃA�N�Z�X����̂ŁA���t���[���|�C���^�̒l���ς��j
	unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//���O�ɓǂݎ����depth�摜��ێ�����z��
	unsigned char *readMat=new unsigned char[KinectSize.x*KinectSize.y/8];//���̌��o�̍ۂɊ��ɕ��̂����邩�𒲂ׂ��s�N�Z������0(false)��1(true)�ŋL�����Čv�Z������������Bsizeof(char)=1byte=8bit�͊��Ɉˑ����Ȃ��̂𗘗p���A(x,y)�̔����readMat[(x+y*KinectSize.x)/8]�̏ォ��((x+y*KinectSize.x)%8)bit��(��ԏ��0bit�ڂƂ���)�Ɋi�[�B
	if(drawMat==nullptr || readMat==nullptr){
		throw(std::runtime_error("Memory is not satisfied."));
	}
	for(int i=0;i<KinectSize.x*KinectSize.y;i++){
		drawMat[i]=0;
	}
	//bone�̂ǂ����ǂ��q���邩�̃f�[�^
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

	//�擾�����f�[�^���L�^����ꏊ
	bool fileWriteFlag=false;//�t�@�C�����͂����邩�ǂ���
	int writeCount=0;//��������ł��鎞�Ԃ̌v��
	const int writeCountMax=30*30;//����ȏ�̎��ԏ������܂Ȃ��悤�ɂ���
	std::ofstream writeFile;
	
	//�L�^���������Đ�����ۂɗp����f�[�^
	bool playDataFlag=false;//�Đ����邩�ǂ���
	int playFlame=0;//�����t���[���ڂ��Đ����Ă��邩
	std::ifstream readFile;
	const int captureFps=30;//�B�e�f�[�^��fps
	const int drawFps=60;//�`�掞��fps
	double playRate=1.0;//�Đ����x
	
	//�A�v���P�[�V��������
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//�Q�[���{��
		//�L�[���X�V
		input_update();
		
		//�`��
		clsDx();
		//depth�摜�`��(�f�[�^�L�^���̂ݕ`�悷��)
		if(!playDataFlag){
			for(int y=0;y<KinectSize.y;y++){
				for(int x=0;x<KinectSize.x;x++){
					int color=drawMat[x+y*KinectSize.x];
					int c=color*255/8000;//�l��0~255�͈͓̔��Ɏ��߂�Bdrawmat���ɂ��鐔�l��500�`8000�ŁAmm�P�ʂł̕��̂܂ł̋����ł���
					DrawPixel(x,y,GetColor(c,c,c));
				}
			}
		}
		//��������body���ꂼ��ɑ΂��ď������s��
		for(size_t j=0;j<BodyNum;j++){
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
					mapper->MapCameraPointToDepthSpace(jointPositions[j][i].GetCameraSpacePoint(),&point);
					jointsPos[i]=Vector2D(KinectSize.x-(int)point.X,(int)point.Y);//�����Ȃ̂Ŕ��]���Ď擾
				} catch(const std::exception &e){
					printfDx(e.what());
				}
				//xy�摜�Ezy�摜�ɓ����֐߂̈ʒu:jointsXY,jointsZY
				const double hAngle=70.0,vAngle=60.0;
				const float halfDepthRange=8.0/2;
				float posz=jointPositions[j][i].Z;
				if(posz==0.0){
					//0���Z��h���B�{���͗�O��������ׂ�
					posz=JointPosition::defaultfloat;
				}
				//xy�摜�̒��S��(KinectSize.x/2,KinectSize.y*3/2)�ɕ`��
				jointsXY[i]=Vector2D((int)(KinectSize.x/2*jointPositions[j][i].X/jointPositions[j][i].Z/std::sin(hAngle/360*M_PI)),(int)(KinectSize.y/2*jointPositions[j][i].Y/jointPositions[j][i].Z/std::sin(vAngle/360*M_PI)));//�ʏ�̍��W�n�ɂ����钆�S����̑��΋���
				jointsXY[i]=Vector2D(jointsXY[i].x,-jointsXY[i].y)+Vector2D(KinectSize.x/2,KinectSize.y*3/2);//DX���C�u�����̍��W�n�ɕϊ����A�X�ɐ�Έʒu�ɕϊ�
				//zy�摜�̒��S��(KinectSize.x*3/2,KinectSize.y*3/2)�ɕ`��
				jointsZY[i]=Vector2D((int)(KinectSize.x*(halfDepthRange/2-jointPositions[j][i].Z)/halfDepthRange),(int)(KinectSize.x*jointPositions[j][i].Y/halfDepthRange));//�������E�̍��W�n�ł̒��S����̑��΋���
				jointsZY[i]=Vector2D(jointsZY[i].x,-jointsZY[i].y)+KinectSize*3/2;//DX���C�u�����̍��W�n�ɕϊ����A�X�ɐ�Έʒu�ɕϊ�
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
/*
		//�S�Ă�body�ɑ΂��āAxy�摜��zy�摜��`�悷��
		for(auto pBody:pBodies){
			if(pBody!=nullptr){
				//�������E�ɂ�����A�ebody��kinect����̍��W���擾����Bmm�P�ʂ�depth�摜�ł͂Ȃ��Askelton����擾���Ă���̂ŒP�ʂ�m�B
				Joint joints[JointType::JointType_Count];
				pBody->GetJoints(JointType::JointType_Count,joints);
				//�e�֐ߓ_�̕`��ʒu���L�^
				Vector2D jointsXY[JointType::JointType_Count];
				Vector2D jointsZY[JointType::JointType_Count];
				for(int i=0;i<JointType::JointType_Count;i++){
					const double hAngle=70.0,vAngle=60.0;
					const float halfDepthRange=8.0/2;
					//xy�摜�̒��S��(KinectSize.x/2,KinectSize.y*3/2)�ɕ`��
					float posz=joints[i].Position.Z;
					if(posz==0.0){
						posz=0.001;//�{���͗�O����������ׂ�
					} else{
						int a=0;
					}
					jointsXY[i]=Vector2D((int)(KinectSize.x/2*joints[i].Position.X/joints[i].Position.Z/std::sin(hAngle/360*M_PI)),(int)(KinectSize.y/2*joints[i].Position.Y/joints[i].Position.Z/std::sin(vAngle/360*M_PI)));//�ʏ�̍��W�n�ɂ����钆�S����̑��΋���
					jointsXY[i]=Vector2D(jointsXY[i].x,-jointsXY[i].y)+Vector2D(KinectSize.x/2,KinectSize.y*3/2);//DX���C�u�����̍��W�n�ɕϊ����A�X�ɐ�Έʒu�ɕϊ�
					//zy�摜�̒��S��(KinectSize.x*3/2,KinectSize.y*3/2)�ɕ`��
					jointsZY[i]=Vector2D((int)(KinectSize.x*(halfDepthRange/2-joints[i].Position.Z)/halfDepthRange),(int)(KinectSize.x*joints[i].Position.Y/halfDepthRange));//�������E�̍��W�n�ł̒��S����̑��΋���
					jointsZY[i]=Vector2D(jointsZY[i].x,-jointsZY[i].y)+KinectSize*3/2;//DX���C�u�����̍��W�n�ɕϊ����A�X�ɐ�Έʒu�ɕϊ�
				}
				//�e�֐߂̕`��
				for(int i=0;i<JointType::JointType_Count;i++){
					DrawCircle(jointsXY[i].x,jointsXY[i].y,circlesize,GetColor(0,255,0),FALSE);
					DrawCircle(jointsZY[i].x,jointsZY[i].y,circlesize,GetColor(0,255,0),FALSE);
				}
				//�e�{�[���̕`��
				for(const auto &pair:bonePairs){
					Vector2D posXY[2]={jointsXY[pair.first],jointsXY[pair.second]};
					Vector2D posZY[2]={jointsZY[pair.first],jointsZY[pair.second]};
					DrawLine(posXY[0].x,posXY[0].y,posXY[1].x,posXY[1].y,GetColor(255,0,0),1);
					DrawLine(posZY[0].x,posZY[0].y,posZY[1].x,posZY[1].y,GetColor(255,0,0),1);
				}
			}
		}
//*/
		
		//���X�V
		if(!playDataFlag){
			//���̋L�^���s���郂�[�h�̎�
			printfDx("RecordingDataMode\n");
			//depth
			//�f�[�^�̓ǂݎ��
			printfDx("depth:\n");
			try{
				IDepthFrame *pDepthFrame=nullptr;
				ErrorCheck(pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error\n");
				ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error\n");
				printfDx("success\n");
				//�ǂݎ�肪�ł����̂�drawMat���X�V�B�����Ȃ̂ō��E���]������B
				if(bufferMat!=nullptr){
					for(int y=0;y<KinectSize.y;y++){
						for(int x=0;x<KinectSize.x;x++){
							drawMat[(KinectSize.x-x-1)+y*KinectSize.x]=bufferMat[x+y*KinectSize.x];
						}
					}
				}
				pDepthFrame->Release();//bufferMat�̎Q�Ƃ��I�������̂�pDepthFrame���J������
			} catch(const std::exception &e){
				printfDx(e.what());
			}
			//body
			printfDx("body:\n");
			try{
				ErrorCheck(pBodyReader->AcquireLatestFrame(&pBodyFrame),"aquaire failed\n");//���߃t���[����body�f�[�^�̎擾
				ErrorCheck(pBodyFrame->GetAndRefreshBodyData(6,pBodies),"access failed\n");//body�f�[�^��pBodies�Ɋi�[
				pBodyFrame->Release();//����ȍ~pBodyFrame�͎g��Ȃ�
				printfDx("success\n");
				for(int j=0;j<BodyNum;j++){
					//�֐߂̎����W�ʒu��jointPositions�Ɋi�[
					Joint joints[JointType::JointType_Count];
					pBodies[j]->GetJoints(JointType::JointType_Count,joints);//�֐߈ʒu�̎����W�̎擾
					//jointPositions�Ɋ֐߂̎����W���i�[
					for(int i=0;i<JointType_Count;i++){
						jointPositions[j][i]=JointPosition(joints[i].Position);
					}
				}
			} catch(const std::exception &e){
				printfDx(e.what());
			}
			//���̋L�^�����邩�̃t���O�̍X�V
			if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
				fileWriteFlag=!fileWriteFlag;
				if(fileWriteFlag){
					//�L�^�J�n���̓t�@�C�����J���AwriteCount��0�ɂ���
					writeCount=0;
					writeFile.open("SaveData/"+to_string_0d(0,3)+".txt",std::ios_base::trunc);
					if(!writeFile){
						//�t�@�C�����J���Ȃ���΋L�^�J�n���Ȃ�
						fileWriteFlag=false;
					}
				} else{
					//�L�^�I�����̓t�@�C�������
					writeFile.close();
				}
			}
			//�t�@�C���o��
			printfDx("fileWriteFlag:\n");
			printfDx((fileWriteFlag && !(!writeFile)) ? "true\n":"false\n");
			if(fileWriteFlag && !(!writeFile)){
				//�������݂�������
				writeCount++;
				if(writeCount>writeCountMax){
					fileWriteFlag=false;
				}
				//body�ʒu�̏o��
				//�`����1�s�ɂ��A1�t���[���ł�jointPositions[i][j]�̊e�v�f��(X,Y,Z)�Ƃ����`���ɂ��ďo�́B
				for(int j=0;j<BodyNum;j++){
					for(int i=0;i<JointType_Count;i++){
						writeFile<<jointPositions[j][i].GetString();
					}
				}
				writeFile<<std::endl;//1�t���[�����̑S�Ă̏o�͂��I�������̂ŉ��s���o��
			}
			//�L�^�f�[�^�Đ����[�h�ւ̈ڍs����
			if(keyboard_get(KEY_INPUT_0)==1){
				readFile.open("SaveData/"+to_string_0d(0,3)+".txt");
				if(!readFile){
					//�ǂݍ��ݎ��s���̏���
				}else{
					//�ǂݍ��ݐ������̂݁A�Đ����[�h��
					playDataFlag=true;
					playFlame=0;
				}
			}
		}else{
			//�L�^�������̂̍Đ����s�����[�h
			printfDx("PlayingDataMode\n");
			int a=playFlame*captureFps*playRate/drawFps;
			playFlame++;
			int b=playFlame*captureFps*playRate/drawFps;//���̒l��a�Ɉ�v���Ă��鎞�͓ǂݍ��݂͍s�킸�A�O�t���[���Ɠ����摜��`�悷��
			if(!readFile || a==b){
				//���ɉ������Ȃ�
			}else{
				//�t�@�C����1�s�ǂݍ��݂Ȃ���AjointPositions�Ƀf�[�^���i�[
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
					}else{
						if(inBracketsFlag){
							//()���ǂݎ�莞(=')'��T���Ă���)
							parts.push_back(ch);
							if(ch==')'){
								inBracketsFlag=!inBracketsFlag;
								//jointPositions�Ɋi�[
								if(bodyindex<BodyNum && jointindex<JointType_Count){
									jointPositions[bodyindex][jointindex]=JointPosition(parts);
									//index�n�̍X�V
									jointindex++;
									if(jointindex>=JointType_Count){
										jointindex=0;
										bodyindex++;
									}
								}
								parts.clear();
							}
						}else{
							//()�O�ǂݎ�莞(='('��T���Ă���)
							if(ch=='('){
								inBracketsFlag=!inBracketsFlag;
								parts.push_back(ch);
							}
						}
					}
				}
				//�t�@�C�������ɓ��B������A�Đ����[�h�͏I�����L�^���[�h�ɖ߂�
				if(ch==EOF){
					readFile.close();
					playDataFlag=!playDataFlag;
				}
			}
		}
		//�Đ����[�h�ɂ�����Đ����x����
		if(keyboard_get(KEY_INPUT_Z)>0){
			//�x������
			playRate=std::fmax(0.1,playRate-0.05);
		} else if(keyboard_get(KEY_INPUT_X)>0){
			//1.0�{�ɖ߂�
			playRate=1.0;
		} else if(keyboard_get(KEY_INPUT_C)>0){
			//��������
			playRate=playRate+0.05;
		}
		printfDx("playRate:\n%f",playRate);

		//�I�����o
		if(keyboard_get(KEY_INPUT_ESCAPE)>0){
			break;
		}
	}

	//�I������

	pSensor->Close();
	pSensor->Release();

	writeFile.close();
	readFile.close();
	//DeleteFontToHandle(font);

}

void Simulate(){
	BodySimulator bs;
	//�A�v���P�[�V��������
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//�Q�[���{��
		//�L�[���X�V
		input_update();

		//�`��
		clsDx();
		bs.Draw();

		//���X�V
		int index=bs.Update();

		//�I�����o
		if(keyboard_get(KEY_INPUT_ESCAPE)>0 || index!=0){
			break;
		}
	}

}

int WINAPI WinMain(HINSTANCE,HINSTANCE,LPSTR,int){
	try{
		const Vector2D KinectSize(512,424);
		//dx���C�u�����̏�����
		//��ʃ��[�h�̐ݒ�(�ꉞ����Ȋ���)
		SetGraphMode(KinectSize.x*2+500,KinectSize.y*2,16);
		//�^�C�g�����j���[����
		SetMainWindowText("TennisSupporter");
		//�E�C���h�E�T�C�Y�̕ύX
		SetWindowSizeExtendRate(0.5);
		//�E�C���h�E�T�C�Y�̕ύX���ł���悤�ɂ���
		SetWindowSizeChangeEnableFlag(TRUE);
		//�A�C�R���̐ݒ�
		SetWindowIconID(101);
		//��A�N�e�B�u��Ԃł̏����̑��s�̃t���O
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

		//���͋@�\�̏�����
		InitInputControler();

		//���s
		//DepthSimulate(false,KinectSize);
		//BodySimulate(KinectSize);
		Simulate();

		//�I������
		DeleteInputControler();//���͋@�\�̉��
		DxLib_End();


		return 0;
	} catch(const std::exception &e){
		assert(e.what());
		return 1;
	}
}

