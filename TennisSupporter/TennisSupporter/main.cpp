#include<cassert>
#include"DxLib.h"
#include<Kinect.h>
#include"input.h"

#include<set>
#include<vector>

void ErrorCheck(HRESULT hresult,const char *msg)noexcept(false){
	if(hresult!=S_OK){
		throw(std::runtime_error(msg));
	}
}

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
	

	IBodyFrame *pBodyFrame=nullptr;
	IBody *pBodies[6];
	for(size_t i=0;i<6;i++){
		pBodies[i]=nullptr;
	}

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


	//�A�v���P�[�V��������
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//�Q�[���{��
		//�L�[���X�V
		input_update();
		
		//�`��
		clsDx();
		//depth�摜�`��
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				int color=drawMat[x+y*KinectSize.x];
				int c=color*255/8000;//�l��0~255�͈͓̔��Ɏ��߂�Bdrawmat���ɂ��鐔�l��500�`8000�ŁAmm�P�ʂł̕��̂܂ł̋����ł���
				DrawPixel(x,y,GetColor(c,c,c));
			}
		}
		//��������body���ꂼ��ɑ΂��ď������s��
		for(auto pBody:pBodies){
			if(pBody==nullptr){
				continue;
			}
			try{
				BOOLEAN flag;
				ErrorCheck(pBody->get_IsTracked(&flag),"");
			} catch(const std::exception &e){
				continue;
			}
			//�֐߂̎擾
			Joint joints[JointType::JointType_Count];
			pBody->GetJoints(JointType::JointType_Count,joints);
			//�e�֐߂ɑ΂��鏈��
			for(Joint joint:joints){
				try{
					ICoordinateMapper *mapper;
					ErrorCheck(pSensor->get_CoordinateMapper(&mapper),"mapper failed\n");
					DepthSpacePoint point;//opencv�n�̍��W�B���Ȃ킿dxlib�Ɠ����B
					mapper->MapCameraPointToDepthSpace(joint.Position,&point);
					DrawCircle(KinectSize.x-(int)point.X,(int)point.Y,5,GetColor(0,255,0),TRUE);//�����Ȃ̂Ŕ��]�����ĕ\��
				} catch(const std::exception &e){
					printfDx(e.what());
				}
			}
		}
		
		//���X�V
		//depth
		//�f�[�^�̓ǂݎ��
		printfDx("depth:\n");
		try{
			IDepthFrame *pDepthFrame=nullptr;
			ErrorCheck(pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error\n");
			ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error\n");
			printfDx("success\n");
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
		//body
		printfDx("body:\n");
		try{
			ErrorCheck(pBodyReader->AcquireLatestFrame(&pBodyFrame),"aquaire failed\n");//���߃t���[����body�f�[�^�̎擾
			ErrorCheck(pBodyFrame->GetAndRefreshBodyData(6,pBodies),"access failed\n");//body�f�[�^��pBodies�Ɋi�[
			pBodyFrame->Release();//����ȍ~pBodyFrame�͎g��Ȃ�
			printfDx("success\n");
		} catch(const std::exception &e){
			printfDx(e.what());
		}
		


		//�I�����o
		if(keyboard_get(KEY_INPUT_ESCAPE)>0){
			break;
		}
	}

	//�I������

	pSensor->Close();
	pSensor->Release();

}

int WINAPI WinMain(HINSTANCE,HINSTANCE,LPSTR,int){
	try{
		const Vector2D KinectSize(512,424);
		//dx���C�u�����̏�����
		//��ʃ��[�h�̐ݒ�(�ꉞ����Ȋ���)
		SetGraphMode(KinectSize.x,KinectSize.y,16);
		//�^�C�g�����j���[����
		SetMainWindowText("TennisSupporter");
		//�E�C���h�E�T�C�Y�̕ύX
		SetWindowSizeExtendRate(1.0);
		//�E�C���h�E�T�C�Y�̕ύX���ł���悤�ɂ���
		SetWindowSizeChangeEnableFlag(FALSE);
		//�A�C�R���̐ݒ�
		SetWindowIconID(101);


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
		BodySimulate(KinectSize);

		//�I������
		DeleteInputControler();//���͋@�\�̉��
		DxLib_End();


		return 0;
	} catch(const std::exception &e){
		assert(e.what());
		return 1;
	}
}

