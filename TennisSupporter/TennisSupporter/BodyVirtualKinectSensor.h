#ifndef DEF_BODYVIRTUALKINECTSENSOR_H
#define DEF_BODYVIRTUALKINECTSENSOR_H

#include"IBodyKinectSensor.h"

//kinect��p���Ȃ����ABodyKinectSensor�̂悤�ȃf�[�^�Ǘ��E�`����s���N���X
class BodyVirtualKinectSensor:public IBodyKinectSensor{
	//�^�E�񋓑�
public:

	//�萔
public:

	//�ϐ�
protected:

	//�֐�
protected:

public:
	BodyVirtualKinectSensor();
	~BodyVirtualKinectSensor();
	int Update(const std::vector<std::vector<JointPosition>> &frameData);//�ǂݍ��ݍς݃f�[�^��p���čX�V����
	void OutputJointPoitions(std::ofstream &writeFile,const std::vector<std::vector<JointPosition>> &frameData)const;//writeFile�Ɉ����̃f�[�^����ǂݎ���jointPositions��1�s�ŏo�͂���

};

#endif // !DEF_BODYVIRTUALKINECTSENSOR_H
#pragma once
