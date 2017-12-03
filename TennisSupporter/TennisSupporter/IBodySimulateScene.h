#ifndef DEF_BODYSIMULATESCENE_H
#define DEF_BODYSIMULATESCENE_H

#include"KinectTools.h"

//シミュレータ上で共通して用いる定数とクラス形式を定義する
class IBodySimulateScene{
	//列挙体・型
public:
	//継承先のクラスがどのようなクラスかを定義する列挙体
	struct MODE{
		enum TYPE{
			PHOTOGRAPHER
			,PLAYER
			,END
		};
		static const TYPE link(int num);
	};

	//定数
public:
	static const int writeCountMax;//これ以上の時間はデータを書き込まないようにする
	static const int captureFps;//撮影データのfps
	static const int drawFps;//描画時のfps
	static const Vector2D kinectSize;//KinectV2の取得可能な画像サイズ（body,depth）

	//変数
protected:
	const MODE::TYPE m_type;

	//関数
public:
	IBodySimulateScene(MODE::TYPE type):m_type(type){}
	virtual ~IBodySimulateScene(){}
	virtual int Update()=0;
	virtual void Draw()const=0;
	MODE::TYPE GetType()const{
		return m_type;
	}
};

#endif // !DEF_BODYSIMULATECONSTPALS_H
#pragma once
