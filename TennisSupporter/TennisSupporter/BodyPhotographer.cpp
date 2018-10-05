#define _USE_MATH_DEFINES

#include<cmath>
#include<assert.h>
#include"BodyPhotographer.h"
#include"DxLib.h"
#include"input.h"


//-----------------BodyPhotographer-----------------
BodyPhotographer::BodyPhotographer(int font,IKinectSensor *pSensor)
	:IBodySimulateScene(MODE::PHOTOGRAPHER,font),m_pSensor(pSensor),m_fileWriteFlag(false),m_writeCount(0)
	,m_bigFont(CreateFontToHandle("メイリオ",32,3,-1))
{
	//BodySensorの起動
	m_pBodyKinectSensor=std::shared_ptr<BodyKinectSensor>(new BodyKinectSensor(m_pSensor));
	//DepthKinectSensorの起動
	m_pDepthKinectSensor=std::shared_ptr<DepthKinectSensor>(new DepthKinectSensor(kinectSize,m_pSensor));
	//m_writeFileは、必要になり次第初期化する。
	//出力ファイル名の設定
	m_writeFileName=SearchFileName();
}

BodyPhotographer::~BodyPhotographer(){
	m_writeFile.close();
	DeleteFontToHandle(m_bigFont);
}

void BodyPhotographer::FinishFileWrite(){
	m_fileWriteFlag=false;//ファイルを書き込んでいない事を明確にする
	m_writeFile.close();//ファイルを閉じる
	//次に用いるファイル名を検索して格納する
	m_writeFileName=SearchFileName();
}

std::string BodyPhotographer::SearchFileName()const{
	//ファイル名が「"SaveData/"+to_string_0d(n,4)+".txt"」で表現されるという前提で探索を行う
	for(int i=0;i<100;i++){
		std::string name="SaveData/"+to_string_0d(i,4)+".txt";
		if(!JudgeFileExist(name)){
			//ファイルが存在していなければ次はそこに書き込むものとする。
			//デモ用に全て0000に書き込む
			name="SaveData/0000.txt";
			return name;
		}
	}
	assert(false);//本来ここに来てはいけないので、それを明確に伝える
	return "SaveData/err.txt";
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
		if(!m_fileWriteFlag){
			//記録開始時はファイルを開き、m_writeCountを0にする
			m_writeCount=0;
			if(!JudgeFileExist(m_writeFileName)){
				//既存のファイルに上書きが行われないことを確認してからファイルを開く
				m_writeFile.open(m_writeFileName,std::ios_base::trunc);
			} else{
				//既存のファイルの上書きが起こった場合は、出力ファイル名を変える作業のみ行う
				//m_writeFileName=SearchFileName();
				m_writeFile.open(m_writeFileName,std::ios_base::trunc);//デモなので上書きを許す
			}
			if(!m_writeFile){
				//ファイルを開けなければ記録開始しない
				m_fileWriteFlag=false;
			} else{
				//ファイルが開けば記録を開始する
				m_fileWriteFlag=true;
			}
		} else{
			//記録終了時はファイルを閉じる
			FinishFileWrite();
		}
	}
	//ファイル出力
	printfDx("fileWriteFlag:\n");
	printfDx((m_fileWriteFlag && !(!m_writeFile)) ? "true\n":"false\n");
	if(m_fileWriteFlag){
		//書き込みすぎ判定
		m_writeCount++;
		if(m_writeCount>writeCountMax){
			FinishFileWrite();
		}
		//body位置の出力
		m_pBodyKinectSensor->OutputJointPoitions(m_writeFile);
	}
	//状態遷移
	if(keyboard_get(KEY_INPUT_1)==1){
		//再生モードへ
		return 1;
	} else if(keyboard_get(KEY_INPUT_2)==1){
		//分析モードへ
		return 2;
	} else if(keyboard_get(KEY_INPUT_3)==1){
		//上級者再生モードへ
		return 3;
	} else if(keyboard_get(KEY_INPUT_4)==1){
		//上級者分析モードへ
		return 4;
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
	//出力先ファイル名を出力
	DrawStringToHandle(zyPos.x,depthPos.y,("out filename: "+m_writeFileName).c_str(),GetColor(255,255,255),m_bigFont);
	//操作説明の描画(横幅等はテキトー、どうせはみださない)
	int po=DrawStringNewLineToHandle(1350,50,0,0,10000,10000,GetColor(255,255,255),m_font,GetFontSizeToHandle(m_font),
		"Enter : begin photograph\n"
		"1 : play \"0000.txt\"\n"
		"2 : analyze \"0000_section.txt\"\n"
		"3 : play expert data\n"
		"4 : analyze expert data\n"
	);
	//録画しているか分かるように
	if(m_fileWriteFlag){
		int dx,dy;
		GetWindowSize(&dx,&dy);
		DrawLine(0,0,0,dy,GetColor(255,0,0),10);
		DrawLine(0,dy,dx,dy,GetColor(255,0,0),10);
		DrawLine(dx,dy,dx,0,GetColor(255,0,0),10);
		DrawLine(dx,0,0,0,GetColor(255,0,0),10);
	}
}
