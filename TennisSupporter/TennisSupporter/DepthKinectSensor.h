#ifndef DEF_DEPTHKINECTSENSOR_H
#define DEF_DEPTHKINECTSENSOR_H

#include"Kinect.h"
#include"KinectTools.h"

//body要素を読み取るkinectセンサーを管理する
class DepthKinectSensor{
	//型・列挙体

	//定数
	
	//変数
protected:
	const Vector2D kinectSize;
	IDepthFrameReader *m_pDepthReader;
	unsigned short *m_drawMat;

	//関数
protected:

public:
	DepthKinectSensor(Vector2D i_kinectSize,IKinectSensor *pSensor);
	~DepthKinectSensor();
	int Update();
	void Draw(Vector2D depthPos)const;
};

#endif // !DEF_DEPTHKINECTSENSOR_H
#pragma once
