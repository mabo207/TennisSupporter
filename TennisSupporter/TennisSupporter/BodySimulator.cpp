#define _USE_MATH_DEFINES

#include<cmath>
#include"BodySimulator.h"
#include"DxLib.h"
#include"input.h"

#include"BodyDataPlayer.h"
#include"BodyPhotographer.h"
#include"DataAnalyzer.h"

//-----------------BodySimulator-----------------
BodySimulator::BodySimulator()
	:m_font(CreateFontToHandle("メイリオ",12,1,-1))
{
	//センサーの起動
	m_pSensor=nullptr;
	ErrorCheck(GetDefaultKinectSensor(&m_pSensor),"You can't get Kinect Sensor.");
	ErrorCheck(m_pSensor->Open(),"You can't activate Kinect Sensor.");
	BOOLEAN isOpen=false;
	ErrorCheck(m_pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
	//m_pSceneの初期化
	m_pScene=std::shared_ptr<IBodySimulateScene>(new BodyPhotographer(m_pSensor));
}

BodySimulator::~BodySimulator(){
	//センサーの解放
	m_pSensor->Close();
	m_pSensor->Release();
	//フォントの解放
	DeleteFontToHandle(m_font);
}

int BodySimulator::Update(){
	//更新作業
	int index=m_pScene->Update();
	//状態遷移
	switch(m_pScene->GetType()){
	case(IBodySimulateScene::MODE::PHOTOGRAPHER):
		if(index==1){
			//場面を再生モードに変更
			m_pScene=std::shared_ptr<IBodySimulateScene>(
				new BodyDataPlayer(
					m_font
					,("SaveData/"+to_string_0d(0,4)+".txt").c_str()));
		} else if(index==2){
			//場面を分析モードに変更
			m_pScene=std::shared_ptr<IBodySimulateScene>(
				new DataAnalyzer(
					m_font
					,("SaveData/"+to_string_0d(0,4)+"_section.txt").c_str()));
		}
		break;
	case(IBodySimulateScene::MODE::PLAYER):
		if(index==1){
			//場面を録画モードに変更
			m_pScene=std::shared_ptr<IBodySimulateScene>(new BodyPhotographer(m_pSensor));
		}
		break;
	case(IBodySimulateScene::MODE::ANALYZER):
		if(index==1){
			//場面を録画モードに変更
			m_pScene=std::shared_ptr<IBodySimulateScene>(new BodyPhotographer(m_pSensor));
		}
		break;
	default:
		return -1;
	}

	return 0;
}

void BodySimulator::Draw()const{
	m_pScene->Draw();
}


