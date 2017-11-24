#ifndef DEF_BODYSIMULATOR_H
#define DEF_BODYSIMULATOR_H

#include<memory>
#include"BodyKinectSensor.h"
#include"DepthKinectSensor.h"

//Kinect��body�v�f��ǂݎ��A�L�^�E�Đ�����@�\
class BodySimulator{
	//�񋓑́E�^

	//�萔
	static const int writeCountMax;//����ȏ�̎��Ԃ̓f�[�^���������܂Ȃ��悤�ɂ���
	static const int captureFps;//�B�e�f�[�^��fps
	static const int drawFps;//�`�掞��fps
	static const Vector2D kinectSize;//KinectV2�̎擾�\�ȉ摜�T�C�Y�ibody,depth�j

	//�ϐ�
protected:
	IKinectSensor *m_pSensor;//kinect�̃Z���T�[
	std::shared_ptr<BodyKinectSensor> m_pBodyKinectSensor;//Body�����̍X�V���s��
	std::shared_ptr<DepthKinectSensor> m_pDepthKinectSensor;//depth�����̍X�V���s��

	//�f�[�^�L�^�̍ۂɗp����f�[�^
	std::ofstream m_writeFile;//�f�[�^�̏������ݐ�
	bool m_fileWriteFlag;//�t�@�C�����͂����邩�ǂ���
	int m_writeCount;//��������ł��鎞�Ԃ̌v��
	
	//�L�^���������Đ�����ۂɗp����f�[�^
	bool m_playDataFlag;//�Đ����邩�ǂ���
	int m_playFlame;//�����t���[���ڂ��Đ����Ă��邩
	std::ifstream m_readFile;//�Đ��f�[�^�̓ǂݍ��ݐ�
	double m_playRate;//�Đ����x


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
