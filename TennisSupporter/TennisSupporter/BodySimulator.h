#ifndef DEF_BODYSIMULATOR_H
#define DEF_BODYSIMULATOR_H

#include"DepthKinectSensor.h"
#include"GraphDataBuilder.h"

//Kinectでbody要素を読み取り、記録・再生する機能
class BodySimulator{
	//列挙体・型

	//定数
	static const int writeCountMax;//これ以上の時間はデータを書き込まないようにする
	static const int captureFps;//撮影データのfps
	static const int drawFps;//描画時のfps
	static const Vector2D kinectSize;//KinectV2の取得可能な画像サイズ（body,depth）
	static const Vector2D graphPos,graphSize;//グラフの位置と大きさ
	static const std::string sectionStr;//sectionデータ書き出しの際のsectionの区切り文字列

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
	std::string m_playDataName;//再生しているデータのファイル名(拡張子を除く)
	double m_playFrame;//今何フレーム目を再生しているか
	double m_playRate;//再生速度
	std::vector<std::vector<std::vector<BodyKinectSensor::JointPosition>>> m_playData;//ファイル全体を読み込んだデータを格納する変数。m_playData[flameIndex][bodyIndex][JointType]というようにして要素を呼び出す。最大1MB。
	std::vector<double> m_data;//グラフ化するデータ
	double m_dataMin,m_dataMax;//m_dataの最大値最小値
	std::shared_ptr<GraphDataBuilder> m_pGraphDataBuilder;//データ化の更新を管理する
	bool m_playFlag;//再生を行うかどうか
	std::vector<std::pair<int,int>> m_section;//グラフデータの切り取り区間
	int m_beforeRClickFrame;//直前フレームにおける右クリックフレーム数
	int m_startSectionIndex;//切り取り区間保持開始の際のフレーム数

	int m_font;//グラフに表示する文字のfont

	//関数
protected:
	int CalReadIndex()const;//m_playFrameから、m_playDataのどの番号のデータを読み込めば良いか計算する。
	double CalPlayFrame(int index)const;//CalReadIndexの逆算
	bool ReadFile(const char *filename);
	void DataBuild();
	bool JudgeMouseInGraph()const;
	void UpdateImage();
	void WriteSections();//m_sectionに当てはまるデータを全て書き出す
	
public:
	BodySimulator();
	~BodySimulator();
	int Update();
	void Draw()const;
};

#endif // !DEF_BODYSIMULATOR_H
#pragma once
