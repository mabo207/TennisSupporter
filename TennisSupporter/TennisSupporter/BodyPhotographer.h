#ifndef DEF_BODYPHOTOGRAPHER_H
#define DEF_BODYPHOTOGRAPHER_H

#include"DepthKinectSensor.h"
#include"BodyKinectSensor.h"
#include<memory>
#include"IBodySimulateScene.h"

//body�v�f�̎B�e���s���N���X
class BodyPhotographer:public IBodySimulateScene{
	//�^�E�񋓑�

	//�萔

	//�ϐ�
protected:
	IKinectSensor *m_pSensor;//kinect�̃Z���T�[(���̂̐���͂��̃N���X�ł͍s��Ȃ�)

	std::shared_ptr<BodyKinectSensor> m_pBodyKinectSensor;//Body�����̍X�V���s��
	std::shared_ptr<DepthKinectSensor> m_pDepthKinectSensor;//depth�����̍X�V���s��

	
	//�f�[�^�L�^�̍ۂɗp����f�[�^
	std::ofstream m_writeFile;//�f�[�^�̏������ݐ�
	bool m_fileWriteFlag;//�t�@�C�����͂����邩�ǂ���
	int m_writeCount;//��������ł��鎞�Ԃ̌v��

	//�֐�
protected:
	
public:
	BodyPhotographer(IKinectSensor *pSensor);
	~BodyPhotographer();
	int Update();
	void Draw()const;
};

#endif // !DEF_BODYPHOTOGRAPHER_H
#pragma once
