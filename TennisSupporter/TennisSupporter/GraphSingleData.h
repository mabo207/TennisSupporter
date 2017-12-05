#ifndef DEF_GRAPHSINGLEDATA_H
#define DEF_GRAPHSINGLEDATA_H

#include"BodyVirtualKinectSensor.h"
#include"GraphDataBuilder.h"
#include<memory>

//�O���t�ɐ܂�����P�`�悷��̂ɕK�v�ȃf�[�^
struct GraphSingleData{
	//�ϐ�
	std::vector<std::vector<std::vector<IBodyKinectSensor::JointPosition>>> m_playData;//�t�@�C���S�̂�ǂݍ��񂾃f�[�^���i�[����ϐ��Bm_playData[flameIndex][bodyIndex][JointType]�Ƃ����悤�ɂ��ėv�f���Ăяo���B�ő�1MB�B
	//�ȉ��A�����y���̂��ߌv�Z���ʂ��i�[���邽�߂̕ϐ�
	std::shared_ptr<BodyVirtualKinectSensor> m_pBodyVirtualKinectSensor;//Body�����̍X�V���s��
	std::vector<double> m_data;//�O���t������f�[�^
	double m_dataMin,m_dataMax;//m_data�̍ő�l�ŏ��l

	//�֐�
	GraphSingleData();
	~GraphSingleData();
	void DataBuild(std::shared_ptr<const GraphDataBuilder> pGraphDataBuilder);
	void UpdateVirtualSensor(const int index);
};

#endif // !DEF_GRAPHSINGLEDATA_H
#pragma once
