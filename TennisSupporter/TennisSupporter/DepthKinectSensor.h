#ifndef DEF_DEPTHKINECTSENSOR_H
#define DEF_DEPTHKINECTSENSOR_H

#include"Kinect.h"
#include"KinectTools.h"

//body�v�f��ǂݎ��kinect�Z���T�[���Ǘ�����
class DepthKinectSensor{
	//�^�E�񋓑�

	//�萔
	
	//�ϐ�
protected:
	const Vector2D kinectSize;
	IDepthFrameReader *m_pDepthReader;
	unsigned short *m_drawMat;

	//�֐�
protected:

public:
	DepthKinectSensor(Vector2D i_kinectSize,IKinectSensor *pSensor);
	~DepthKinectSensor();
	int Update();
	void Draw(Vector2D depthPos)const;
};

#endif // !DEF_DEPTHKINECTSENSOR_H
#pragma once
