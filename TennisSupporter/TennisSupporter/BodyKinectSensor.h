#ifndef DEF_BODYKINECTSENSOR_H
#define DEF_BODYKINECTSENSOR_H

#include"IBodyKinectSensor.h"
#include<iostream>

//body要素を読み取るkinectセンサーを管理する
class BodyKinectSensor:public IBodyKinectSensor{
	//型・列挙体
public:
	
	//定数
public:

	//変数
protected:
	IBodyFrameReader *m_pBodyReader;

	//関数
protected:

public:
	BodyKinectSensor(IKinectSensor *pSensor);
	~BodyKinectSensor();
	int Update();//kinectから情報を取得し更新する
	int Update(std::ifstream &readFile);//テキストデータから情報を１行分取得し更新する
};

#endif // !DEF_BODYKINECTSENSOR_H
#pragma once
