#ifndef DEF_BODYDATAPLAYER_H
#define DEF_BODYDATAPLAYER_H

#include"IBodySimulateScene.h"
#include"BodyVirtualKinectSensor.h"
#include"GraphDataBuilder.h"
#include"GraphSingleData.h"
#include<memory>

class BodyDataPlayer:public IBodySimulateScene{
	//�񋓑́E�^

	//�萔
	static const Vector2D graphPos,graphSize;//�O���t�̈ʒu�Ƒ傫��
	static const std::string sectionStr;//section�f�[�^�����o���̍ۂ�section�̋�؂蕶����

	//�ϐ�
protected:
/*
	std::vector<std::vector<std::vector<IBodyKinectSensor::JointPosition>>> m_playData;//�t�@�C���S�̂�ǂݍ��񂾃f�[�^���i�[����ϐ��Bm_playData[flameIndex][bodyIndex][JointType]�Ƃ����悤�ɂ��ėv�f���Ăяo���B�ő�1MB�B
	std::shared_ptr<BodyVirtualKinectSensor> m_pBodyVirtualKinectSensor;//Body�����̍X�V���s��
	std::vector<double> m_data;//�O���t������f�[�^
	double m_dataMin,m_dataMax;//m_data�̍ő�l�ŏ��l
//*/
	GraphSingleData m_graphSingleData;

	//�L�^���������Đ�����ۂɗp����f�[�^
	std::string m_playDataName;//�Đ����Ă���f�[�^�̃t�@�C����(�g���q������)
	double m_playFrame;//�����t���[���ڂ��Đ����Ă��邩
	double m_playRate;//�Đ����x
	std::shared_ptr<GraphDataBuilder> m_pGraphDataBuilder;//�f�[�^���̍X�V���Ǘ�����
	bool m_playFlag;//�Đ����s�����ǂ���
	std::vector<std::pair<int,int>> m_section;//�O���t�f�[�^�̐؂�����
	int m_beforeRClickFrame;//���O�t���[���ɂ�����E�N���b�N�t���[����
	int m_startSectionIndex;//�؂����ԕێ��J�n�̍ۂ̃t���[����
	bool m_graphUnity;//�O���t�̏c�����̊�𓝈ꂷ�邩

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
	BodyDataPlayer(int font,const char *filename);
	~BodyDataPlayer();
	int Update();
	void Draw()const;
};


#endif // !DEF_BODYDATAPLAYER_H
#pragma once
