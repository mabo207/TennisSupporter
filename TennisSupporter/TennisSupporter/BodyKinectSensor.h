#ifndef DEF_BODYKINECTSENSOR_H
#define DEF_BODYKINECTSENSOR_H

#include"IBodyKinectSensor.h"
#include<iostream>

//body�v�f��ǂݎ��kinect�Z���T�[���Ǘ�����
class BodyKinectSensor:public IBodyKinectSensor{
	//�^�E�񋓑�
public:
	
	//�萔
public:

	//�ϐ�
protected:
	IBodyFrameReader *m_pBodyReader;

	//�֐�
protected:

public:
	BodyKinectSensor(IKinectSensor *pSensor);
	~BodyKinectSensor();
	int Update();//kinect��������擾���X�V����
	int Update(std::ifstream &readFile);//�e�L�X�g�f�[�^��������P�s���擾���X�V����
};

#endif // !DEF_BODYKINECTSENSOR_H
#pragma once
