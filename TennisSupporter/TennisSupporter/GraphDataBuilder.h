#ifndef DEF_GRAPHDATABUILDER_H
#define DEF_GRAPHDATABUILDER_H

#include"IBodyKinectSensor.h"
#include<memory>
#include<map>

//�O���t������ۂɁA�ǂ̃f�[�^���O���t�����邩�̎w��̓��͂��󂯕t����
class GraphDataBuilder{
	//�^�E�񋓑�
	//���͂��󂯕t�������ʂ��i�[���A���̕ϐ��Ɋ�Â��ăO���t�ɂ���ۂ̒l���o�͂���N���X
	//���N���X
	struct IDataFactory{
		//�֐�
		virtual double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const=0;
		virtual void Draw(Vector2D pos)const=0;
		virtual double DataMax()const=0;
		virtual double DataMin()const=0;
		virtual std::vector<JointType> IGetInput()const=0;
		virtual std::string IGetFactoryType()const=0;
		//�ÓI�֐�
	};
	//�ʒu�ɑ΂��ėp����N���X
	struct PosDataFactory:public IDataFactory{
		//�ϐ�
		const JointType type;
		const double nVecX;//�f�[�^�Z�o�̍ۂ̖@���x�N�g����x�v�f
		const double nVecY;//�f�[�^�Z�o�̍ۂ̖@���x�N�g����y�v�f
		const double nVecZ;//�f�[�^�Z�o�̍ۂ̖@���x�N�g����z�v�f
		//�֐�
		PosDataFactory(JointType i_type,double i_nVecX,double i_nVecY,double i_nVecZ);
		double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const;//�_�ƕ��ʂ̋������Z�o����B
		double DataMax()const;
		double DataMin()const;
		void Draw(Vector2D pos)const;
		std::vector<JointType> IGetInput()const;
		std::string IGetFactoryType()const;
	};
	//�p�x�ɑ΂��ėp����N���X
	struct AngleDataFactory:public IDataFactory{
		//�萔
		static const size_t indexNum=3;
		//�ϐ�
		const JointType type[indexNum];
		//�֐�
		AngleDataFactory(JointType point1,JointType point2,JointType point3);
		double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const;
		double DataMax()const;
		double DataMin()const;
		void Draw(Vector2D pos)const;
		std::vector<JointType> IGetInput()const;
		std::string IGetFactoryType()const;
	};

	//�萔
protected:
	//�^�l�ԃC���^�[�t�F�[�X����
	static const std::map<JointType,Vector2D> relativeInputPos;//�ǂ�JointType���ǂ��̓��͉~�ɑΉ����Ă��邩��\��map�B���΍��W�B
	//�x�N�g���ݒ�C���^�[�t�F�[�X����
	static const Vector2D xzVectorBoxPos,yVectorBoxPos,boxSize;//xz�x�N�g���ݒ�{�b�N�X�̈ʒu,y�x�N�g���ݒ�{�b�N�X�̈ʒu�A�{�b�N�X�̑傫��
	static const int boxCircleSize;//xz�x�N�g�����ɂ���~�ʂ̑傫��
	static const int axisSize;//xz�x�N�g�����ɂ��鎲�̒���
	static const Vector2D xzBoxCircleCenterPos;//xz�x�N�g�����ɂ���~�ʂ̒��S
	static const Vector2D xBoxPos,zBoxPos;//xz�x�N�g�����ɂ���x��,z���̈ʒu
	//���p���
	static const int circleSize;//�o������~�̑傫��
	static const int squareSize;//�o�����鐳���`�̑傫��

	//�ϐ�
protected:
	const Vector2D m_position;//�ݒ��ʂ̕`��ʒu�i����j
	std::shared_ptr<IDataFactory> m_dataFactory;//�ǂ̂悤�ɂ��ăf�[�^����邩���Ǘ�����

	//���͎���̊Ǘ�
	//�^�l�ԃC���^�[�t�F�[�X����
	std::vector<JointType> m_input;//�}�E�X���{�^���������ςȂ��Œʂ���joint�Q
	//�x�N�g���ݒ�C���^�[�t�F�[�X����
	double m_xzAngle;//xz�x�N�g���ݒ�C���^�[�t�F�[�X�ɂ��p�x���͒l�i�����v���ɐ��A�����x���������j
	bool m_xzOrY;//xz�x�N�g��(true)��y�x�N�g��(false)�̂ǂ�����ϑ����邩
	//���p���
	bool m_updateDataFactoryFlag;//���}�E�X�{�^���𗣂�������dataFactory���X�V���邩�ǂ���
	int m_inpFrame;//�O�t���[���ɂ�����}�E�X���{�^���̉����Ă����t���[����
	int m_font;

	//�֐�
protected:
	void CreateFactory(const std::vector<JointType> &input);//�O���t�f�[�^�r���_�[�̃C���^�t�F�[�X�̓��͌��ʂ�p���āAm_dataFactory���쐬����֐��B���z�I�ȓ��͂��󂯕t������悤�Ɋ֐ߓ��͈͂����œn���B

public:
	GraphDataBuilder(Vector2D position,int font);
	~GraphDataBuilder();
	int Update();//�}�E�X���{�^���𗣂����u�Ԃ�1��Ԃ�
	void Draw()const;
	double CalData(const std::vector<IBodyKinectSensor::JointPosition> &playData)const;
	double DataMax()const;
	double DataMin()const;
	std::string GetFactoryType()const;
};

#endif // !DEF_GRAPHDATABUILDER_H
#pragma once
