#include<iostream>
#include<sstream>
#include<cassert>
#include"DxLib.h"
#include<Kinect.h>
#include"input.h"

void ErrorCheck(HRESULT hresult,const char *msg)noexcept(false){
	if(hresult!=S_OK){
		throw(std::runtime_error(msg));
	}
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
		unsigned short *bufferMat=nullptr;//���擾�p
		unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//�`��p
		if(drawMat==nullptr){
			throw(std::runtime_error("Memory is not satisfied."));
		}
		for(int i=0;i<KinectSize.x*KinectSize.y;i++){
			drawMat[i]=0;
		}
		

		//���͋@�\�̏�����
		InitInputControler();

		//�A�v���P�[�V��������
		while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
			//�Q�[���{��
			//�L�[���X�V
			input_update();
			//�`��
			clsDx();
			const unsigned short *adress1=bufferMat;
			printfDx("%x\n",bufferMat);
			for(int y=0;y<KinectSize.y;y++){
				for(int x=0;x<KinectSize.x;x++){
					int color=drawMat[x+y*KinectSize.x];
					int c=color*255/8000;//�l��0~255�͈͓̔��Ɏ��߂�Bdrawmat���ɂ��鐔�l��500�`8000�ŁAmm�P�ʂł̕��̂܂ł̋����ł���
					if(c>255){
						//�ϑ��͈͂𒴂����Ƃ���͍����`�悷��
						c=0;
					}
					DrawPixel(x,y,GetColor(c,c,c));
				}
			}
			//�v�Z����
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
			//�I�����o
			if(keyboard_get(KEY_INPUT_ESCAPE)>0){
				break;
			}
		}

		//�I������
		if(drawMat!=nullptr){
			delete drawMat;
		}

		DeleteInputControler();//���͋@�\�̉��

		pSensor->Close();
		pSensor->Release();

		DxLib_End();


		return 0;
	} catch(const std::exception &e){
		assert(e.what());
		return 1;
	}
}

