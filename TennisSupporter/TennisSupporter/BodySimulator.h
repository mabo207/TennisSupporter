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

	int m_mode;//���ݍs���Ă���̂��ǂ̃��[�h�Ȃ̂�(0:�L�^ 1:�Đ� 2:�Đ��������̂̃O���t��)

	//�f�[�^�L�^�̍ۂɗp����f�[�^
	std::ofstream m_writeFile;//�f�[�^�̏������ݐ�
	bool m_fileWriteFlag;//�t�@�C�����͂����邩�ǂ���
	int m_writeCount;//��������ł��鎞�Ԃ̌v��
	
	//�L�^���������Đ�����ۂɗp����f�[�^
	double m_playFrame;//�����t���[���ڂ��Đ����Ă��邩
	std::ifstream m_readFile;//�Đ��f�[�^�̓ǂݍ��ݐ�
	double m_playRate;//�Đ����x
	std::vector<std::vector<std::vector<BodyKinectSensor::JointPosition>>> m_playData;//�t�@�C���S�̂�ǂݍ��񂾃f�[�^���i�[����ϐ��Bm_playData[flameIndex][bodyIndex][JointType]�Ƃ����悤�ɂ��ėv�f���Ăяo���B�ő�1MB�B
	std::vector<double> m_data;//�O���t������f�[�^
	double m_dataMin,m_dataMax;//m_data�̍ő�l�ŏ��l
	int m_font;//�O���t�ɕ\�����镶����font

	//�֐�
protected:
	int CalReadIndex()const;//m_playFrame����Am_playData�̂ǂ̔ԍ��̃f�[�^��ǂݍ��߂Ηǂ����v�Z����B
	bool ReadFile(const char *filename);
	void DataBuild(JointType jointtype);
	void DataBuild(JointType edge,JointType point1,JointType point2);

public:
	BodySimulator();
	~BodySimulator();
	int Update();
	void Draw()const;
};

#endif // !DEF_BODYSIMULATOR_H
#pragma once
