#define _USE_MATH_DEFINES

#include<cmath>
#include"BodyPhotographer.h"
#include"DxLib.h"
#include"input.h"

//-----------------BodyPhotographer-----------------
BodyPhotographer::BodyPhotographer(IKinectSensor *pSensor)
	:IBodySimulateScene(MODE::PHOTOGRAPHER),m_pSensor(pSensor),m_fileWriteFlag(false),m_writeCount(0)
{
	//BodySensor�̋N��
	m_pBodyKinectSensor=std::shared_ptr<BodyKinectSensor>(new BodyKinectSensor(m_pSensor));
	//DepthKinectSensor�̋N��
	m_pDepthKinectSensor=std::shared_ptr<DepthKinectSensor>(new DepthKinectSensor(kinectSize,m_pSensor));
	//m_writeFile�́A�K�v�ɂȂ莟�揉��������B

}

BodyPhotographer::~BodyPhotographer(){
	m_writeFile.close();
}

int BodyPhotographer::Update(){
	printfDx("RecordingDataMode\n");
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
		//�Đ����[�h��
		return 1;
	}
	return 0;
}

void BodyPhotographer::Draw()const{
	//�ʒu�̒�`
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	//�`�施��
	m_pDepthKinectSensor->Draw(depthPos);
	m_pBodyKinectSensor->Draw(m_pSensor,depthPos,kinectSize,xyPos,kinectSize,zyPos,kinectSize);
	
}
