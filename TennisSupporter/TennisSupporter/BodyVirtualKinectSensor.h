#ifndef DEF_BODYVIRTUALKINECTSENSOR_H
#define DEF_BODYVIRTUALKINECTSENSOR_H

#include"IBodyKinectSensor.h"

//kinectを用いないが、BodyKinectSensorのようなデータ管理・描画を行うクラス
class BodyVirtualKinectSensor:public IBodyKinectSensor{
	//型・列挙体
public:

	//定数
public:

	//変数
protected:

	//関数
protected:

public:
	BodyVirtualKinectSensor();
	~BodyVirtualKinectSensor();
	int Update(const std::vector<std::vector<JointPosition>> &frameData);//読み込み済みデータを用いて更新する
	void OutputJointPoitions(std::ofstream &writeFile,const std::vector<std::vector<JointPosition>> &frameData)const;//writeFileに引数のデータから読み取れるjointPositionsを1行で出力する

};

#endif // !DEF_BODYVIRTUALKINECTSENSOR_H
#pragma once
