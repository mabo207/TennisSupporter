#ifndef DEF_IBODYKINECTSENSOR_H
#define DEF_IBODYKINECTSENSOR_H

#include<vector>
#include<Kinect.h>
#include<fstream>
#include<map>
#include"KinectTools.h"

//BodyKinectSensorの描画・データ構造を表す
class IBodyKinectSensor{
	//型・列挙体
public:
	struct JointPosition{
		static const float defaultfloat;
		float X;
		float Y;
		float Z;
		//X,Y,Zの値を直接入力する
		JointPosition(float i_X=defaultfloat,float i_Y=defaultfloat,float i_Z=defaultfloat);
		//_CameraSpacePointより初期化する
		JointPosition(_CameraSpacePoint pos);
		//"(X,Y,Z)"という形式の文字列を読み取って初期化する
		JointPosition(const std::string &str);
		//==の実装
		bool operator==(const JointPosition &otherobj)const;
		//"(X,Y,Z)"という文字列を出力する
		std::string GetString()const;
		//_CameraSpacePointを作成
		_CameraSpacePoint GetCameraSpacePoint()const;
		//Joint::PositionがJointPositionのようになっているJointを返す。その他の要素はテキトー。
		Joint CreateJoint()const;
		Joint CreateJoint(_JointType type)const;
		Joint CreateJoint(_TrackingState state)const;
		Joint CreateJoint(_JointType type,_TrackingState state)const;
		//自分から２つの別のJointPositionへのベクトルの交わる角度を求める(0～180度)
		double CalculateAngle(JointPosition v1,JointPosition v2)const;
	};

	//定数
public:
	static const std::vector<std::pair<_JointType,_JointType>> bonePairs;//体のつながり方を記録する
	static const size_t bodyNum=6;//Kinectが認識できる人間の上限数
	static const std::map<_JointType,std::string> jointName;//関節点の単語名

	//変数
protected:
	JointPosition m_jointPositions[bodyNum][JointType_Count];

	//関数
protected:
	bool BodyIndexSignificance(size_t bodyIndex)const;//bodyIndex番目のデータは有意なデータかを判定する（全てのJointPositionが(0,0,0)でないかどうか）

public:
	IBodyKinectSensor();
	virtual ~IBodyKinectSensor();
	void OutputJointPoitions(std::ofstream &writeFile)const;//writeFileに現在のjointPositionsを1行で出力する
	void Draw(IKinectSensor *pSensor,Vector2D depthPos,Vector2D depthSize,Vector2D xyPos,Vector2D xySize,Vector2D zyPos,Vector2D zySize)const;
	//情報取得のために用いる
	JointPosition GetJointPosition(_JointType jointType)const;//１番最初に見えるbodyを自動判定して実行
	JointPosition GetJointPosition(size_t bodyIndex,_JointType jointType)const;
	double GetRadian(_JointType edge,_JointType point1,_JointType point2)const;//１番最初に見えるbodyを自動判定して実行
	double GetRadian(size_t bodyIndex,_JointType edge,_JointType point1,_JointType point2)const;
};

#endif // !DEF_IBODYKINECTSENSOR_H
#pragma once
