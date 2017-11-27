#ifndef DEF_GRAPHDATABUILDER_H
#define DEF_GRAPHDATABUILDER_H

#include"BodyKinectSensor.h"
#include<memory>
#include<map>

//グラフ化する際に、どのデータをグラフ化するかの指定の入力を受け付ける
class GraphDataBuilder{
	//型・列挙体
	//入力を受け付けた結果を格納し、その変数に基づいてグラフにする際の値を出力するクラス
	//基底クラス
	struct IDataFactory{
		//関数
		virtual double ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const=0;
		virtual void Draw()const=0;
		//静的関数
		static std::shared_ptr<IDataFactory> CreateFactory(const std::vector<JointType> &input);
	};
	//位置に対して用いるクラス
	struct PosDataFactory:public IDataFactory{
		//変数
		const JointType type;
		//関数
		PosDataFactory(JointType i_type);
		double ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const;
		void Draw()const;
	};
	//角度に対して用いるクラス
	struct AngleDataFactory:public IDataFactory{
		//定数
		static const size_t indexNum=3;
		//変数
		const JointType type[indexNum];
		//関数
		AngleDataFactory(JointType point1,JointType point2,JointType point3);
		double ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const;
		void Draw()const;
	};

	//定数
protected:
	static const std::map<JointType,Vector2D> relativeInputPos;//どのJointTypeがどこの入力円に対応しているかを表すmap。相対座標。
	static const int circleSize;

	//変数
protected:
	const Vector2D m_position;//設定画面の描画位置（左上）
	std::shared_ptr<IDataFactory> m_dataFactory;//どのようにしてデータを作るかを管理する

	//入力周りの管理
	std::vector<JointType> m_input;//マウス左ボタン押しっぱなしで通ったjoint群
	int m_inpFrame;//前フレームにおけるマウス左ボタンの押していたフレーム数
	
	//関数
protected:
	

public:
	GraphDataBuilder(Vector2D position);
	~GraphDataBuilder();
	int Update();
	void Draw()const;
};

#endif // !DEF_GRAPHDATABUILDER_H
#pragma once
