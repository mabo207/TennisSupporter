#ifndef DEF_BODYKINECTSENSOR_H
#define DEF_BODYKINECTSENSOR_H

#include<vector>
#include<Kinect.h>
#include<fstream>
#include<iostream>
#include"KinectTools.h"

//body要素を読み取るkinectセンサーを管理する
class BodyKinectSensor{
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
protected:
	static const std::vector<std::pair<_JointType,_JointType>> bonePairs;
public:
	static const size_t bodyNum=6;

	//変数
protected:
	JointPosition m_jointPositions[bodyNum][JointType_Count];
	IBodyFrameReader *m_pBodyReader;

	//関数
protected:
	bool BodyIndexSignificance(size_t bodyIndex)const;//bodyIndex番目のデータは有意なデータかを判定する（全てのJointPositionが(0,0,0)でないかどうか）

public:
	BodyKinectSensor(IKinectSensor *pSensor);
	~BodyKinectSensor();
	void OutputJointPoitions(std::ofstream &writeFile)const;//writeFileに現在のjointPositionsを1行で出力する
	int Update();//kinectから情報を取得し更新する
	int Update(std::ifstream &readFile);//テキストデータから情報を１行分取得し更新する
	int Update(const std::vector<std::vector<JointPosition>> &frameData);//読み込み済みデータを用いて更新する
	void Draw(IKinectSensor *pSensor,Vector2D depthPos,Vector2D depthSize,Vector2D xyPos,Vector2D xySize,Vector2D zyPos,Vector2D zySize)const;
	//情報取得のために用いる
	JointPosition GetJointPosition(_JointType jointType)const;//１番最初に見えるbodyを自動判定して実行
	JointPosition GetJointPosition(size_t bodyIndex,_JointType jointType)const;
	double GetRadian(_JointType edge,_JointType point1,_JointType point2)const;//１番最初に見えるbodyを自動判定して実行
	double GetRadian(size_t bodyIndex,_JointType edge,_JointType point1,_JointType point2)const;
};

#endif // !DEF_BODYKINECTSENSOR_H
#pragma once
