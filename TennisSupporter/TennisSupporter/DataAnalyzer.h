#ifndef DEF_DATAANALYZER_H
#define DEF_DATAANALYZER_H

//複数のbodyデータをグラフ化して比較するモード
#include"IBodySimulateScene.h"
#include"BodyVirtualKinectSensor.h"
#include"GraphDataBuilder.h"
#include"GraphSingleData.h"
#include<memory>

class DataAnalyzer:public IBodySimulateScene{
	//列挙体・型

	//定数
	static const Vector2D graphPos,graphSize;//グラフの位置と大きさ
	static const std::string sectionStr;//sectionデータ書き出しの際のsectionの区切り文字列

	//変数
protected:
	std::vector<GraphSingleData> m_graphData;

	//記録した物を再生する際に用いるデータ
	double m_playFrame;//今何フレーム目を再生しているか
	double m_playRate;//再生速度
	std::shared_ptr<GraphDataBuilder> m_pGraphDataBuilder;//データ化の更新を管理する
	bool m_playFlag;//再生を行うかどうか
	bool m_graphUnity;//グラフの縦横軸の基準を統一するか
	bool m_widthUnity;//グラフの横軸均一化を行うか

	double m_extend;//グラフの拡大率
	double m_dataAverage;//データの平均値
	size_t m_dataSizeMax;//m_graphDataの各データ群のデータ数のうち最大のもの

	//int m_font;//グラフに表示する文字のfont(基底クラスのフォントと同じものを用いる)

	//関数
protected:
	int CalReadIndex()const;//m_playFrameから、m_playDataのどの番号のデータを読み込めば良いか計算する。
	double CalPlayFrame(int index)const;//CalReadIndexの逆算
	bool ReadFile(const char *filename);
	void DataBuild();
	bool JudgeMouseInGraph()const;
	void UpdateImage();
	void UpdateImage(int index);
	
public:
	DataAnalyzer(int font,const char *filename);
	~DataAnalyzer();
	int Update();
	void Draw()const;
};


#endif // !DEF_DATAANALYZER_H
#pragma once
