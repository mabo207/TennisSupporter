#include"BodySimulator.h"
#include"DxLib.h"
#include"input.h"

//-----------------BodySimulator-----------------
const int BodySimulator::writeCountMax=30*30;
const int BodySimulator::captureFps=30;
const int BodySimulator::drawFps=60;
const Vector2D BodySimulator::kinectSize=Vector2D(512,424);

BodySimulator::BodySimulator()
	:m_fileWriteFlag(false),m_writeCount(0),m_playDataFlag(false),m_playFlame(0),m_playRate(1.0)
{
	//�Z���T�[�̋N��
	m_pSensor=nullptr;
	ErrorCheck(GetDefaultKinectSensor(&m_pSensor),"You can't get Kinect Sensor.");
	ErrorCheck(m_pSensor->Open(),"You can't activate Kinect Sensor.");
	BOOLEAN isOpen=false;
	ErrorCheck(m_pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
	//BodySensor�̋N��
	m_pBodyKinectSensor=std::shared_ptr<BodyKinectSensor>(new BodyKinectSensor(m_pSensor));
	//DepthKinectSensor�̋N��
	m_pDepthKinectSensor=std::shared_ptr<DepthKinectSensor>(new DepthKinectSensor(kinectSize,m_pSensor));
	//m_writeFile,m_readFile�́A�K�v�ɂȂ莟�揉��������B
}

BodySimulator::~BodySimulator(){
	m_pSensor->Close();
	m_pSensor->Release();

	m_writeFile.close();
	m_readFile.close();
}

int BodySimulator::Update(){
	if(!m_playDataFlag){
		//���̋L�^���s���郂�[�h�̎�
		printfDx("RecordingDataMode\n");
		//depth
		//�f�[�^�̓ǂݎ��

		//depth
		printfDx("depth:\n");
		m_pDepthKinectSensor->Update();

		//body
		printfDx("body:\n");
		m_pBodyKinectSensor->Update();
		//���̋L�^�����邩�̃t���O�̍X�V
		if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
			m_fileWriteFlag=!m_fileWriteFlag;
			if(m_fileWriteFlag){
				//�L�^�J�n���̓t�@�C�����J���Am_writeCount��0�ɂ���
				m_writeCount=0;
				m_writeFile.open("SaveData/"+to_string_0d(0,3)+".txt",std::ios_base::trunc);
				if(!m_writeFile){
					//�t�@�C�����J���Ȃ���΋L�^�J�n���Ȃ�
					m_fileWriteFlag=false;
				}
			} else{
				//�L�^�I�����̓t�@�C�������
				m_writeFile.close();
			}
		}
		//�t�@�C���o��
		printfDx("fileWriteFlag:\n");
		printfDx((m_fileWriteFlag && !(!m_writeFile)) ? "true\n":"false\n");
		if(m_fileWriteFlag){
			//�������݂�������
			m_writeCount++;
			if(m_writeCount>writeCountMax){
				m_fileWriteFlag=false;
			}
			//body�ʒu�̏o��
			m_pBodyKinectSensor->OutputJointPoitions(m_writeFile);
		}
		//�L�^�f�[�^�Đ����[�h�ւ̈ڍs����
		if(keyboard_get(KEY_INPUT_0)==1){
			m_readFile.open("SaveData/"+to_string_0d(0,3)+".txt");
			if(!m_readFile){
				//�ǂݍ��ݎ��s���̏���
			} else{
				//�ǂݍ��ݐ������̂݁A�Đ����[�h��
				m_playDataFlag=true;
				m_playFlame=0;
			}
		}
	} else{
		//�L�^�������̂̍Đ����s�����[�h
		printfDx("PlayingDataMode\n");
		int a=(int)(m_playFlame*captureFps*m_playRate/drawFps);
		m_playFlame++;
		int b=(int)(m_playFlame*captureFps*m_playRate/drawFps);//���̒l��a�Ɉ�v���Ă��鎞�͓ǂݍ��݂͍s�킸�A�O�t���[���Ɠ����摜��`�悷��
		if(!m_readFile || a==b){
			//���ɉ������Ȃ�
		}else{
			//�t�@�C����1�s�ǂݍ��݂Ȃ���AjointPositions�Ƀf�[�^���i�[
			int index=m_pBodyKinectSensor->Update(m_readFile);
			//�t�@�C�������ɓ��B������A�Đ����[�h�͏I�����L�^���[�h�ɖ߂�
			if(index!=0){
				m_readFile.close();
				m_playDataFlag=!m_playDataFlag;
			}
		}
	}
	//�Đ����[�h�ɂ�����Đ����x����
	if(keyboard_get(KEY_INPUT_Z)>0){
		//�x������
		m_playRate=std::fmax(0.1,m_playRate-0.05);
	} else if(keyboard_get(KEY_INPUT_X)>0){
		//1.0�{�ɖ߂�
		m_playRate=1.0;
	} else if(keyboard_get(KEY_INPUT_C)>0){
		//��������
		m_playRate=m_playRate+0.05;
	}
	printfDx("playRate:\n%f",m_playRate);

	//���ɏI�������͖����̂�0����ɕԂ�
	return 0;
}

void BodySimulator::Draw()const{
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	//depth�摜�`��(�f�[�^�L�^���̂ݕ`�悷��)
	if(!m_playDataFlag){
		m_pDepthKinectSensor->Draw(depthPos);
	}
	//��������body���ꂼ��ɑ΂��ď������s��
	m_pBodyKinectSensor->Draw(m_pSensor,depthPos,kinectSize,xyPos,kinectSize,zyPos,kinectSize);
}
