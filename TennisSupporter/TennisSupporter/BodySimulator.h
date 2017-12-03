#ifndef DEF_BODYSIMULATOR_H
#define DEF_BODYSIMULATOR_H

#include"IBodySimulateScene.h"
#include"GraphDataBuilder.h"

//Kinect��body�v�f��ǂݎ��A�L�^�E�Đ�����@�\
class BodySimulator{
	//�񋓑́E�^

	//�萔

	//�ϐ�
protected:
	IKinectSensor *m_pSensor;//kinect�̃Z���T�[
	std::shared_ptr<IBodySimulateScene> m_pScene;//���ݍs���Ă��鏈��

	int m_font;

   //�֐�
protected:

public:
	BodySimulator();
	~BodySimulator();
	int Update();
	void Draw()const;

};

#endif // !DEF_BODYSIMULATOR_H
#pragma once
