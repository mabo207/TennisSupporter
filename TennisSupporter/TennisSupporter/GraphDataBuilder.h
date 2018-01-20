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
	//距離に対して用いるクラス
	struct LengthDataFactory:public IDataFactory{
		//定数
		static const size_t indexNum=2;
		//変数
		const JointType type[indexNum];
		const bool xFlag;//x成分を計算するか
		const bool yFlag;//y成分を計算するか
		const bool zFlag;//z成分を計算するか
		//関数
		LengthDataFactory(JointType point1,JointType point2,bool i_xFlag,bool i_yFlag,bool i_zFlag);
		double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const;
		double DataMax()const;
		double DataMin()const;
		void Draw(Vector2D pos)const;
		std::vector<JointType> IGetInput()const;
		std::string IGetFactoryType()const;
	};
	//に対して用いるクラス
	struct SlopeDataFactory:public IDataFactory{
		enum ElementType{
			elX,
			elY,
			elZ
		};
		//定数
		static const size_t indexNum=2;
		//変数
		const JointType type[indexNum];
		const ElementType divideEle;
		const ElementType dividedEle;
		//関数
	private:
		double CalculateDiff(ElementType ele,const std::vector<IBodyKinectSensor::JointPosition> &data)const;
		std::string ElementToStr(ElementType ele)const;
	public:
		SlopeDataFactory(JointType point1,JointType point2,ElementType i_dividedEle,ElementType i_divideEle);
		double ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const;
		double DataMax()const;
		double DataMin()const;
		void Draw(Vector2D pos)const;
		std::vector<JointType> IGetInput()const;
		std::string IGetFactoryType()const;
	};


	//インターフェースで用いる列挙体
	struct TanCalKind{
		enum Kind{
			ZDIVX,//z/x
			END
		};
		static const std::string str[END];
	};
	struct TwoPointCalKind{
		enum Kind{
			LENGTH,//距離計算
			TAN,//tan
			END
		};
		static const std::string str[END];
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
	static const Vector2D lengthBoxPos;//距離計算設定インターフェースの場所
	static const Vector2D tanBoxPos;//tan計算設定インターフェースの場所
	static const Vector2D twoPointBoxSize;//2関節点関連のインターフェースの四角形の大きさ

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
	//2関節点時の計算種類
	TwoPointCalKind::Kind m_twoPointCalKind;//2関節点選択時の計算方法
	//距離計算設定
	bool m_xLengthFlag;//距離計算の時にx要素を用いるかどうか
	bool m_yLengthFlag;//距離計算の時にy要素を用いるかどうか
	bool m_zLengthFlag;//距離計算の時にz要素を用いるかどうか
	//ベクトルの成分tan計算設定
	TanCalKind::Kind m_tanCalKind;//tan計算の時にどの要素のtanを求めるか
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
