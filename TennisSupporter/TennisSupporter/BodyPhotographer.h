#ifndef DEF_BODYPHOTOGRAPHER_H
#define DEF_BODYPHOTOGRAPHER_H

#include"DepthKinectSensor.h"
#include"BodyKinectSensor.h"
#include<memory>
#include"IBodySimulateScene.h"

//body要素の撮影を行うクラス
class BodyPhotographer:public IBodySimulateScene{
	//型・列挙体

	//定数

	//変数
protected:
	IKinectSensor *m_pSensor;//kinectのセンサー(実体の制御はこのクラスでは行わない)

	std::shared_ptr<BodyKinectSensor> m_pBodyKinectSensor;//Body部分の更新を行う
	std::shared_ptr<DepthKinectSensor> m_pDepthKinectSensor;//depth部分の更新を行う

	
	//データ記録の際に用いるデータ
	std::ofstream m_writeFile;//データの書き込み先
	bool m_fileWriteFlag;//ファイル入力をするかどうか
	int m_writeCount;//書き込んでいる時間の計測
	std::string m_writeFileName;//データの書き込みを行うファイル名

	//描画のために用いるデータ
	int m_bigFont;//フォント

	//関数
protected:
	void FinishFileWrite();//ファイル書き込み終了時に呼び出す関数
	std::string SearchFileName()const;//次に使うファイル名を検索する

public:
	BodyPhotographer(int font,IKinectSensor *pSensor);
	~BodyPhotographer();
	int Update();
	void Draw()const;
};

#endif // !DEF_BODYPHOTOGRAPHER_H
#pragma once
