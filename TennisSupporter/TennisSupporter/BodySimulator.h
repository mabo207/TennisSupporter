#ifndef DEF_BODYSIMULATOR_H
#define DEF_BODYSIMULATOR_H

#include<memory>
#include"BodyKinectSensor.h"
#include"DepthKinectSensor.h"

//Kinectでbody要素を読み取り、記録・再生する機能
class BodySimulator{
	//列挙体・型

	//定数
	static const int writeCountMax;//これ以上の時間はデータを書き込まないようにする
	static const int captureFps;//撮影データのfps
	static const int drawFps;//描画時のfps
	static const Vector2D kinectSize;//KinectV2の取得可能な画像サイズ（body,depth）

	//変数
protected:
	IKinectSensor *m_pSensor;//kinectのセンサー
	std::shared_ptr<BodyKinectSensor> m_pBodyKinectSensor;//Body部分の更新を行う
	std::shared_ptr<DepthKinectSensor> m_pDepthKinectSensor;//depth部分の更新を行う

	int m_mode;//現在行っているのがどのモードなのか(0:記録 1:再生 2:再生したもののグラフ化)

	//データ記録の際に用いるデータ
	std::ofstream m_writeFile;//データの書き込み先
	bool m_fileWriteFlag;//ファイル入力をするかどうか
	int m_writeCount;//書き込んでいる時間の計測
	
	//記録した物を再生する際に用いるデータ
	double m_playFrame;//今何フレーム目を再生しているか
	std::ifstream m_readFile;//再生データの読み込み先
	double m_playRate;//再生速度
	std::vector<std::vector<std::vector<BodyKinectSensor::JointPosition>>> m_playData;//ファイル全体を読み込んだデータを格納する変数。m_playData[flameIndex][bodyIndex][JointType]というようにして要素を呼び出す。最大1MB。
	std::vector<double> m_data;//グラフ化するデータ
	double m_dataMin,m_dataMax;//m_dataの最大値最小値
	int m_font;//グラフに表示する文字のfont

	//関数
protected:
	int CalReadIndex()const;//m_playFrameから、m_playDataのどの番号のデータを読み込めば良いか計算する。
	bool ReadFile(const char *filename);
	void DataBuild(JointType jointtype);
	void DataBuild(JointType edge,JointType point1,JointType point2);

public:
	BodySimulator();
	~BodySimulator();
	int Update();
	void Draw()const;
};

#endif // !DEF_BODYSIMULATOR_H
#pragma once
