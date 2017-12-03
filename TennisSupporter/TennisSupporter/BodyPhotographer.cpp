#define _USE_MATH_DEFINES

#include<cmath>
#include"BodyPhotographer.h"
#include"DxLib.h"
#include"input.h"

//-----------------BodyPhotographer-----------------
BodyPhotographer::BodyPhotographer(IKinectSensor *pSensor)
	:IBodySimulateScene(MODE::PHOTOGRAPHER),m_pSensor(pSensor),m_fileWriteFlag(false),m_writeCount(0)
{
	//BodySensorの起動
	m_pBodyKinectSensor=std::shared_ptr<BodyKinectSensor>(new BodyKinectSensor(m_pSensor));
	//DepthKinectSensorの起動
	m_pDepthKinectSensor=std::shared_ptr<DepthKinectSensor>(new DepthKinectSensor(kinectSize,m_pSensor));
	//m_writeFileは、必要になり次第初期化する。

}

BodyPhotographer::~BodyPhotographer(){
	m_writeFile.close();
}

int BodyPhotographer::Update(){
	printfDx("RecordingDataMode\n");
	//depth
	printfDx("depth:\n");
	m_pDepthKinectSensor->Update();

	//body
	printfDx("body:\n");
	m_pBodyKinectSensor->Update();
	//情報の記録をするかのフラグの更新
	if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
		m_fileWriteFlag=!m_fileWriteFlag;
		if(m_fileWriteFlag){
			//記録開始時はファイルを開き、m_writeCountを0にする
			m_writeCount=0;
			m_writeFile.open("SaveData/"+to_string_0d(0,3)+".txt",std::ios_base::trunc);
			if(!m_writeFile){
				//ファイルを開けなければ記録開始しない
				m_fileWriteFlag=false;
			}
		} else{
			//記録終了時はファイルを閉じる
			m_writeFile.close();
		}
	}
	//ファイル出力
	printfDx("fileWriteFlag:\n");
	printfDx((m_fileWriteFlag && !(!m_writeFile)) ? "true\n":"false\n");
	if(m_fileWriteFlag){
		//書き込みすぎ判定
		m_writeCount++;
		if(m_writeCount>writeCountMax){
			m_fileWriteFlag=false;
		}
		//body位置の出力
		m_pBodyKinectSensor->OutputJointPoitions(m_writeFile);
	}
	//記録データ再生モードへの移行処理
	if(keyboard_get(KEY_INPUT_0)==1){
		//再生モードへ
		return 1;
	}
	return 0;
}

void BodyPhotographer::Draw()const{
	//位置の定義
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	//描画命令
	m_pDepthKinectSensor->Draw(depthPos);
	m_pBodyKinectSensor->Draw(m_pSensor,depthPos,kinectSize,xyPos,kinectSize,zyPos,kinectSize);
	
}
