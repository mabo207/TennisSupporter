#ifndef DEF_GRAPHDATABUILDER_H
#define DEF_GRAPHDATABUILDER_H

#include"IBodyKinectSensor.h"
#include<memory>
#include<map>

//グラフ化する際に、どのデータをグラフ化するかの指定の入力を受け付ける
class GraphDataBuilder{
	//型・列挙体
	//入力を受け付けた結果を格納し、その変数に基づいてグラフにする際の値を出力するクラス
	//基底クラス
	struct IDataFactory{
		//関数
		virtual double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const=0;
		virtual void Draw(Vector2D pos)const=0;
		virtual double DataMax()const=0;
		virtual double DataMin()const=0;
		virtual std::vector<JointType> IGetInput()const=0;
		virtual std::string IGetFactoryType()const=0;
		//静的関数
	};
	//位置に対して用いるクラス
	struct PosDataFactory:public IDataFactory{
		//変数
		const JointType type;
		const double nVecX;//データ算出の際の法線ベクトルのx要素
		const double nVecY;//データ算出の際の法線ベクトルのy要素
		const double nVecZ;//データ算出の際の法線ベクトルのz要素
		//関数
		PosDataFactory(JointType i_type,double i_nVecX,double i_nVecY,double i_nVecZ);
		double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const;//点と平面の距離を算出する。
		double DataMax()const;
		double DataMin()const;
		void Draw(Vector2D pos)const;
		std::vector<JointType> IGetInput()const;
		std::string IGetFactoryType()const;
	};
	//角度に対して用いるクラス
	struct AngleDataFactory:public IDataFactory{
		//定数
		static const size_t indexNum=3;
		//変数
		const JointType type[indexNum];
		//関数
		AngleDataFactory(JointType point1,JointType point2,JointType point3);
		double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const;
		double DataMax()const;
		double DataMin()const;
		void Draw(Vector2D pos)const;
		std::vector<JointType> IGetInput()const;
		std::string IGetFactoryType()const;
	};

	//定数
protected:
	//某人間インターフェース周り
	static const std::map<JointType,Vector2D> relativeInputPos;//どのJointTypeがどこの入力円に対応しているかを表すmap。相対座標。
	//ベクトル設定インターフェース周り
	static const Vector2D xzVectorBoxPos,yVectorBoxPos,boxSize;//xzベクトル設定ボックスの位置,yベクトル設定ボックスの位置、ボックスの大きさ
	static const int boxCircleSize;//xzベクトル内にある円弧の大きさ
	static const int axisSize;//xzベクトル内にある軸の長さ
	static const Vector2D xzBoxCircleCenterPos;//xzベクトル内にある円弧の中心
	static const Vector2D xBoxPos,zBoxPos;//xzベクトル内にあるx箱,z箱の位置
	//共用情報
	static const int circleSize;//出現する円の大きさ
	static const int squareSize;//出現する正方形の大きさ

	//変数
protected:
	const Vector2D m_position;//設定画面の描画位置（左上）
	std::shared_ptr<IDataFactory> m_dataFactory;//どのようにしてデータを作るかを管理する

	//入力周りの管理
	//某人間インターフェース周り
	std::vector<JointType> m_input;//マウス左ボタン押しっぱなしで通ったjoint群
	//ベクトル設定インターフェース周り
	double m_xzAngle;//xzベクトル設定インターフェースによる角度入力値（反時計回りに正、基準線はx軸横向き）
	bool m_xzOrY;//xzベクトル(true)かyベクトル(false)のどちらを観測するか
	//共用情報
	bool m_updateDataFactoryFlag;//左マウスボタンを離した時にdataFactoryを更新するかどうか
	int m_inpFrame;//前フレームにおけるマウス左ボタンの押していたフレーム数
	int m_font;

	//関数
protected:
	void CreateFactory(const std::vector<JointType> &input);//グラフデータビルダーのインタフェースの入力結果を用いて、m_dataFactoryを作成する関数。仮想的な入力も受け付けられるように関節入力は引数で渡す。

public:
	GraphDataBuilder(Vector2D position,int font);
	~GraphDataBuilder();
	int Update();//マウス左ボタンを離した瞬間は1を返す
	void Draw()const;
	double CalData(const std::vector<IBodyKinectSensor::JointPosition> &playData)const;
	double DataMax()const;
	double DataMin()const;
	std::string GetFactoryType()const;
};

#endif // !DEF_GRAPHDATABUILDER_H
#pragma once
