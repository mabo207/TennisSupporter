#ifndef DEF_DATAANALYZER_H
#define DEF_DATAANALYZER_H

//������body�f�[�^���O���t�����Ĕ�r���郂�[�h
#include"IBodySimulateScene.h"
#include"BodyVirtualKinectSensor.h"
#include"GraphDataBuilder.h"
#include"GraphSingleData.h"
#include<memory>

class DataAnalyzer:public IBodySimulateScene{
	//�񋓑́E�^

	//�萔
	static const Vector2D graphPos,graphSize;//�O���t�̈ʒu�Ƒ傫��
	static const std::string sectionStr;//section�f�[�^�����o���̍ۂ�section�̋�؂蕶����

	//�ϐ�
protected:
	std::vector<GraphSingleData> m_graphData;

	//�L�^���������Đ�����ۂɗp����f�[�^
	double m_playFrame;//�����t���[���ڂ��Đ����Ă��邩
	double m_playRate;//�Đ����x
	std::shared_ptr<GraphDataBuilder> m_pGraphDataBuilder;//�f�[�^���̍X�V���Ǘ�����
	bool m_playFlag;//�Đ����s�����ǂ���
	bool m_graphUnity;//�O���t�̏c�����̊�𓝈ꂷ�邩
	bool m_widthUnity;//�O���t�̉����ψꉻ���s����

	double m_extend;//�O���t�̊g�嗦
	double m_dataAverage;//�f�[�^�̕��ϒl
	size_t m_dataSizeMax;//m_graphData�̊e�f�[�^�Q�̃f�[�^���̂����ő�̂���

	//int m_font;//�O���t�ɕ\�����镶����font(���N���X�̃t�H���g�Ɠ������̂�p����)

	//�֐�
protected:
	int CalReadIndex()const;//m_playFrame����Am_playData�̂ǂ̔ԍ��̃f�[�^��ǂݍ��߂Ηǂ����v�Z����B
	double CalPlayFrame(int index)const;//CalReadIndex�̋t�Z
	bool ReadFile(const char *filename);
	void DataBuild();
	bool JudgeMouseInGraph()const;
	void UpdateImage();
	void UpdateImage(int index);
	
public:
	DataAnalyzer(int font,const char *filename);
	~DataAnalyzer();
	int Update();
	void Draw()const;
};


#endif // !DEF_DATAANALYZER_H
#pragma once
