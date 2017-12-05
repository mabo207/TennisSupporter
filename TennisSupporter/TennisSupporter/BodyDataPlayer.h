#ifndef DEF_BODYDATAPLAYER_H
#define DEF_BODYDATAPLAYER_H

#include"IBodySimulateScene.h"
#include"BodyVirtualKinectSensor.h"
#include"GraphDataBuilder.h"
#include"GraphSingleData.h"
#include<memory>

class BodyDataPlayer:public IBodySimulateScene{
	//列挙体・型

	//定数
	static const Vector2D graphPos,graphSize;//グラフの位置と大きさ
	static const std::string sectionStr;//sectionデータ書き出しの際のsectionの区切り文字列

	//変数
protected:
/*
	std::vector<std::vector<std::vector<IBodyKinectSensor::JointPosition>>> m_playData;//ファイル全体を読み込んだデータを格納する変数。m_playData[flameIndex][bodyIndex][JointType]というようにして要素を呼び出す。最大1MB。
	std::shared_ptr<BodyVirtualKinectSensor> m_pBodyVirtualKinectSensor;//Body部分の更新を行う
	std::vector<double> m_data;//グラフ化するデータ
	double m_dataMin,m_dataMax;//m_dataの最大値最小値
//*/
	GraphSingleData m_graphSingleData;

	//記録した物を再生する際に用いるデータ
	std::string m_playDataName;//再生しているデータのファイル名(拡張子を除く)
	double m_playFrame;//今何フレーム目を再生しているか
	double m_playRate;//再生速度
	std::shared_ptr<GraphDataBuilder> m_pGraphDataBuilder;//データ化の更新を管理する
	bool m_playFlag;//再生を行うかどうか
	std::vector<std::pair<int,int>> m_section;//グラフデータの切り取り区間
	int m_beforeRClickFrame;//直前フレームにおける右クリックフレーム数
	int m_startSectionIndex;//切り取り区間保持開始の際のフレーム数
	bool m_graphUnity;//グラフの縦横軸の基準を統一するか

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
	BodyDataPlayer(int font,const char *filename);
	~BodyDataPlayer();
	int Update();
	void Draw()const;
};


#endif // !DEF_BODYDATAPLAYER_H
#pragma once
