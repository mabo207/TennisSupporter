#define _USE_MATH_DEFINES

#include<cmath>
#include<assert.h>
#include"BodyPhotographer.h"
#include"DxLib.h"
#include"input.h"


//-----------------BodyPhotographer-----------------
BodyPhotographer::BodyPhotographer(int font,IKinectSensor *pSensor)
	:IBodySimulateScene(MODE::PHOTOGRAPHER,font),m_pSensor(pSensor),m_fileWriteFlag(false),m_writeCount(0)
	,m_bigFont(CreateFontToHandle("���C���I",32,3,-1))
{
	//BodySensor�̋N��
	m_pBodyKinectSensor=std::shared_ptr<BodyKinectSensor>(new BodyKinectSensor(m_pSensor));
	//DepthKinectSensor�̋N��
	m_pDepthKinectSensor=std::shared_ptr<DepthKinectSensor>(new DepthKinectSensor(kinectSize,m_pSensor));
	//m_writeFile�́A�K�v�ɂȂ莟�揉��������B
	//�o�̓t�@�C�����̐ݒ�
	m_writeFileName=SearchFileName();
}

BodyPhotographer::~BodyPhotographer(){
	m_writeFile.close();
	DeleteFontToHandle(m_bigFont);
}

void BodyPhotographer::FinishFileWrite(){
	m_fileWriteFlag=false;//�t�@�C������������ł��Ȃ����𖾊m�ɂ���
	m_writeFile.close();//�t�@�C�������
	//���ɗp����t�@�C�������������Ċi�[����
	m_writeFileName=SearchFileName();
}

std::string BodyPhotographer::SearchFileName()const{
	//�t�@�C�������u"SaveData/"+to_string_0d(n,4)+".txt"�v�ŕ\�������Ƃ����O��ŒT�����s��
	for(int i=0;i<100;i++){
		std::string name="SaveData/"+to_string_0d(i,4)+".txt";
		if(!JudgeFileExist(name)){
			//�t�@�C�������݂��Ă��Ȃ���Ύ��͂����ɏ������ނ��̂Ƃ���B
			//�f���p�ɑS��0000�ɏ�������
			name="SaveData/0000.txt";
			return name;
		}
	}
	assert(false);//�{�������ɗ��Ă͂����Ȃ��̂ŁA����𖾊m�ɓ`����
	return "SaveData/err.txt";
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
		if(!m_fileWriteFlag){
			//�L�^�J�n���̓t�@�C�����J���Am_writeCount��0�ɂ���
			m_writeCount=0;
			if(!JudgeFileExist(m_writeFileName)){
				//�����̃t�@�C���ɏ㏑�����s���Ȃ����Ƃ��m�F���Ă���t�@�C�����J��
				m_writeFile.open(m_writeFileName,std::ios_base::trunc);
			} else{
				//�����̃t�@�C���̏㏑�����N�������ꍇ�́A�o�̓t�@�C������ς����Ƃ̂ݍs��
				//m_writeFileName=SearchFileName();
				m_writeFile.open(m_writeFileName,std::ios_base::trunc);//�f���Ȃ̂ŏ㏑��������
			}
			if(!m_writeFile){
				//�t�@�C�����J���Ȃ���΋L�^�J�n���Ȃ�
				m_fileWriteFlag=false;
			} else{
				//�t�@�C�����J���΋L�^���J�n����
				m_fileWriteFlag=true;
			}
		} else{
			//�L�^�I�����̓t�@�C�������
			FinishFileWrite();
		}
	}
	//�t�@�C���o��
	printfDx("fileWriteFlag:\n");
	printfDx((m_fileWriteFlag && !(!m_writeFile)) ? "true\n":"false\n");
	if(m_fileWriteFlag){
		//�������݂�������
		m_writeCount++;
		if(m_writeCount>writeCountMax){
			FinishFileWrite();
		}
		//body�ʒu�̏o��
		m_pBodyKinectSensor->OutputJointPoitions(m_writeFile);
	}
	//��ԑJ��
	if(keyboard_get(KEY_INPUT_1)==1){
		//�Đ����[�h��
		return 1;
	} else if(keyboard_get(KEY_INPUT_2)==1){
		//���̓��[�h��
		return 2;
	} else if(keyboard_get(KEY_INPUT_3)==1){
		//�㋉�ҍĐ����[�h��
		return 3;
	} else if(keyboard_get(KEY_INPUT_4)==1){
		//�㋉�ҕ��̓��[�h��
		return 4;
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
	//�o�͐�t�@�C�������o��
	DrawStringToHandle(zyPos.x,depthPos.y,("out filename: "+m_writeFileName).c_str(),GetColor(255,255,255),m_bigFont);
	//��������̕`��(�������̓e�L�g�[�A�ǂ����݂͂����Ȃ�)
	int po=DrawStringNewLineToHandle(1350,50,0,0,10000,10000,GetColor(255,255,255),m_font,GetFontSizeToHandle(m_font),
		"Enter : begin photograph\n"
		"1 : play \"0000.txt\"\n"
		"2 : analyze \"0000_section.txt\"\n"
		"3 : play expert data\n"
		"4 : analyze expert data\n"
	);
	//�^�悵�Ă��邩������悤��
	if(m_fileWriteFlag){
		int dx,dy;
		GetWindowSize(&dx,&dy);
		DrawLine(0,0,0,dy,GetColor(255,0,0),10);
		DrawLine(0,dy,dx,dy,GetColor(255,0,0),10);
		DrawLine(dx,dy,dx,0,GetColor(255,0,0),10);
		DrawLine(dx,0,0,0,GetColor(255,0,0),10);
	}
}
