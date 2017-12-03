#ifndef DEF_GRAPHSINGLEDATA_H
#define DEF_GRAPHSINGLEDATA_H

#include"BodyVirtualKinectSensor.h"
#include"GraphDataBuilder.h"
#include<memory>

//グラフに折れ線を１つ描画するのに必要なデータ
struct GraphSingleData{
	//変数
	std::vector<std::vector<std::vector<IBodyKinectSensor::JointPosition>>> m_playData;//ファイル全体を読み込んだデータを格納する変数。m_playData[flameIndex][bodyIndex][JointType]というようにして要素を呼び出す。最大1MB。
	//以下、処理軽減のため計算結果を格納するための変数
	std::shared_ptr<BodyVirtualKinectSensor> m_pBodyVirtualKinectSensor;//Body部分の更新を行う
	std::vector<double> m_data;//グラフ化するデータ
	double m_dataMin,m_dataMax;//m_dataの最大値最小値

	//関数
	GraphSingleData();
	~GraphSingleData();
	void DataBuild(std::shared_ptr<const GraphDataBuilder> pGraphDataBuilder);
	void UpdateVirtualSensor(const int index);
};

#endif // !DEF_GRAPHSINGLEDATA_H
#pragma once
