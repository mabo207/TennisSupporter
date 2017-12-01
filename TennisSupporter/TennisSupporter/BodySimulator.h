#ifndef DEF_BODYSIMULATOR_H
#define DEF_BODYSIMULATOR_H

#include"DepthKinectSensor.h"
#include"GraphDataBuilder.h"

//Kinect��body�v�f��ǂݎ��A�L�^�E�Đ�����@�\
class BodySimulator{
	//�񋓑́E�^

	//�萔
	static const int writeCountMax;//����ȏ�̎��Ԃ̓f�[�^���������܂Ȃ��悤�ɂ���
	static const int captureFps;//�B�e�f�[�^��fps
	static const int drawFps;//�`�掞��fps
	static const Vector2D kinectSize;//KinectV2�̎擾�\�ȉ摜�T�C�Y�ibody,depth�j
	static const Vector2D graphPos,graphSize;//�O���t�̈ʒu�Ƒ傫��
	static const std::string sectionStr;//section�f�[�^�����o���̍ۂ�section�̋�؂蕶����

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
	std::string m_playDataName;//�Đ����Ă���f�[�^�̃t�@�C����(�g���q������)
	double m_playFrame;//�����t���[���ڂ��Đ����Ă��邩
	double m_playRate;//�Đ����x
	std::vector<std::vector<std::vector<BodyKinectSensor::JointPosition>>> m_playData;//�t�@�C���S�̂�ǂݍ��񂾃f�[�^���i�[����ϐ��Bm_playData[flameIndex][bodyIndex][JointType]�Ƃ����悤�ɂ��ėv�f���Ăяo���B�ő�1MB�B
	std::vector<double> m_data;//�O���t������f�[�^
	double m_dataMin,m_dataMax;//m_data�̍ő�l�ŏ��l
	std::shared_ptr<GraphDataBuilder> m_pGraphDataBuilder;//�f�[�^���̍X�V���Ǘ�����
	bool m_playFlag;//�Đ����s�����ǂ���
	std::vector<std::pair<int,int>> m_section;//�O���t�f�[�^�̐؂�����
	int m_beforeRClickFrame;//���O�t���[���ɂ�����E�N���b�N�t���[����
	int m_startSectionIndex;//�؂����ԕێ��J�n�̍ۂ̃t���[����

	int m_font;//�O���t�ɕ\�����镶����font

	//�֐�
protected:
	int CalReadIndex()const;//m_playFrame����Am_playData�̂ǂ̔ԍ��̃f�[�^��ǂݍ��߂Ηǂ����v�Z����B
	double CalPlayFrame(int index)const;//CalReadIndex�̋t�Z
	bool ReadFile(const char *filename);
	void DataBuild();
	bool JudgeMouseInGraph()const;
	void UpdateImage();
	void WriteSections();//m_section�ɓ��Ă͂܂�f�[�^��S�ď����o��
	
public:
	BodySimulator();
	~BodySimulator();
	int Update();
	void Draw()const;
};

#endif // !DEF_BODYSIMULATOR_H
#pragma once
