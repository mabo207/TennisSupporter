#ifndef DEF_BODYSIMULATOR_H
#define DEF_BODYSIMULATOR_H

#include"IBodySimulateScene.h"
#include"GraphDataBuilder.h"

//Kinectでbody要素を読み取り、記録・再生する機能
class BodySimulator{
	//列挙体・型

	//定数

	//変数
protected:
	IKinectSensor *m_pSensor;//kinectのセンサー
	std::shared_ptr<IBodySimulateScene> m_pScene;//現在行っている処理

	int m_font;

   //関数
protected:

public:
	BodySimulator();
	~BodySimulator();
	int Update();
	void Draw()const;

};

#endif // !DEF_BODYSIMULATOR_H
#pragma once
