#ifndef DEF_BODYKINECTSENSOR_H
#define DEF_BODYKINECTSENSOR_H

#include<vector>
#include<Kinect.h>
#include<fstream>
#include<iostream>
#include"KinectTools.h"

//body�v�f��ǂݎ��kinect�Z���T�[���Ǘ�����
class BodyKinectSensor{
	//�^�E�񋓑�
	struct JointPosition{
		static const float defaultfloat;
		float X;
		float Y;
		float Z;
		//X,Y,Z�̒l�𒼐ړ��͂���
		JointPosition(float i_X=defaultfloat,float i_Y=defaultfloat,float i_Z=defaultfloat);
		//_CameraSpacePoint��菉��������
		JointPosition(_CameraSpacePoint pos);
		//"(X,Y,Z)"�Ƃ����`���̕������ǂݎ���ď���������
		JointPosition(const std::string &str);
		//"(X,Y,Z)"�Ƃ�����������o�͂���
		std::string GetString()const;
		//_CameraSpacePoint���쐬
		_CameraSpacePoint GetCameraSpacePoint()const;
		//Joint::Position��JointPosition�̂悤�ɂȂ��Ă���Joint��Ԃ��B���̑��̗v�f�̓e�L�g�[�B
		Joint CreateJoint()const;
		Joint CreateJoint(_JointType type)const;
		Joint CreateJoint(_TrackingState state)const;
		Joint CreateJoint(_JointType type,_TrackingState state)const;
	};
	
	//�萔
protected:
	static const std::vector<std::pair<_JointType,_JointType>> bonePairs;
	static const size_t bodyNum=6;

	//�ϐ�
protected:
	JointPosition m_jointPositions[bodyNum][JointType_Count];
	IBodyFrameReader *m_pBodyReader;

	//�֐�
public:
	BodyKinectSensor(IKinectSensor *pSensor);
	~BodyKinectSensor();
	void OutputJointPoitions(std::ofstream &writeFile)const;//writeFile�Ɍ��݂�jointPositions��1�s�ŏo�͂���
	int Update();//kinect��������擾���X�V����
	int Update(std::ifstream &readFile);//�e�L�X�g�f�[�^��������P�s���擾���X�V����
	void Draw(IKinectSensor *pSensor,Vector2D depthPos,Vector2D depthSize,Vector2D xyPos,Vector2D xySize,Vector2D zyPos,Vector2D zySize)const;

};

#endif // !DEF_BODYKINECTSENSOR_H
#pragma once
