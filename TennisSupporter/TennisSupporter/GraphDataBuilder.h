#ifndef DEF_GRAPHDATABUILDER_H
#define DEF_GRAPHDATABUILDER_H

#include"BodyKinectSensor.h"
#include<memory>
#include<map>

//�O���t������ۂɁA�ǂ̃f�[�^���O���t�����邩�̎w��̓��͂��󂯕t����
class GraphDataBuilder{
	//�^�E�񋓑�
	//���͂��󂯕t�������ʂ��i�[���A���̕ϐ��Ɋ�Â��ăO���t�ɂ���ۂ̒l���o�͂���N���X
	//���N���X
	struct IDataFactory{
		//�֐�
		virtual double ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const=0;
		virtual void Draw(Vector2D pos)const=0;
		//�ÓI�֐�
		static std::shared_ptr<IDataFactory> CreateFactory(const std::vector<JointType> &input);
	};
	//�ʒu�ɑ΂��ėp����N���X
	struct PosDataFactory:public IDataFactory{
		//�ϐ�
		const JointType type;
		//�֐�
		PosDataFactory(JointType i_type);
		double ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const;
		void Draw(Vector2D pos)const;
	};
	//�p�x�ɑ΂��ėp����N���X
	struct AngleDataFactory:public IDataFactory{
		//�萔
		static const size_t indexNum=3;
		//�ϐ�
		const JointType type[indexNum];
		//�֐�
		AngleDataFactory(JointType point1,JointType point2,JointType point3);
		double ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const;
		void Draw(Vector2D pos)const;
	};

	//�萔
protected:
	static const std::map<JointType,Vector2D> relativeInputPos;//�ǂ�JointType���ǂ��̓��͉~�ɑΉ����Ă��邩��\��map�B���΍��W�B
	static const int circleSize;

	//�ϐ�
protected:
	const Vector2D m_position;//�ݒ��ʂ̕`��ʒu�i����j
	std::shared_ptr<IDataFactory> m_dataFactory;//�ǂ̂悤�ɂ��ăf�[�^����邩���Ǘ�����

	//���͎���̊Ǘ�
	std::vector<JointType> m_input;//�}�E�X���{�^���������ςȂ��Œʂ���joint�Q
	int m_inpFrame;//�O�t���[���ɂ�����}�E�X���{�^���̉����Ă����t���[����
	
	//�֐�
protected:
	

public:
	GraphDataBuilder(Vector2D position);
	~GraphDataBuilder();
	int Update();//�}�E�X���{�^���𗣂����u�Ԃ�1��Ԃ�
	void Draw()const;
	double CalData(const std::vector<BodyKinectSensor::JointPosition> &playData)const;
};

#endif // !DEF_GRAPHDATABUILDER_H
#pragma once
