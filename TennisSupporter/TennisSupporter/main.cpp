#include<iostream>
#include<sstream>
#include<cassert>
#include"DxLib.h"
#include<Kinect.h>
#include"input.h"

void ErrorCheck(HRESULT hresult,const char *msg)noexcept(false){
	if(hresult!=S_OK){
		throw(std::runtime_error(msg));
	}
}

int WINAPI WinMain(HINSTANCE,HINSTANCE,LPSTR,int){
	try{
		const Vector2D KinectSize(512,424);
		//dxライブラリの初期化
		//画面モードの設定(一応こんな感じ)
		SetGraphMode(KinectSize.x,KinectSize.y,16);
		//タイトルメニュー文字
		SetMainWindowText("TennisSupporter");
		//ウインドウサイズの変更
		SetWindowSizeExtendRate(1.0);
		//ウインドウサイズの変更をできるようにする
		SetWindowSizeChangeEnableFlag(FALSE);
		//アイコンの設定
		SetWindowIconID(101);


		if(ChangeWindowMode(TRUE) != 0) {
			throw(std::runtime_error("ChangeWindowMode(TRUE) failed."));
		}
		if(DxLib_Init() != 0) {
			throw(std::runtime_error("DxLib_Init() failed."));
		}
		if(SetDrawScreen(DX_SCREEN_BACK) != 0) {
			DxLib_End();
			throw(std::runtime_error("SetDrawScreen(DX_SCREEN_BACK) failed."));
		}

		//kinectの初期化
		//sensor
		IKinectSensor *pSensor=nullptr;
		ErrorCheck(GetDefaultKinectSensor(&pSensor),"You can't get Kinect Sensor.");
		ErrorCheck(pSensor->Open(),"You can't activate Kinect Sensor.");
		BOOLEAN isOpen=false;
		ErrorCheck(pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
		//source
		IDepthFrameSource *pDepthSource;
		ErrorCheck(pSensor->get_DepthFrameSource(&pDepthSource),"You can't get source.");
		//reader
		IDepthFrameReader *pDepthReader;
		ErrorCheck(pDepthSource->OpenReader(&pDepthReader),"You can't open reader.");
		//memory
		unsigned int bufferSize=KinectSize.x*KinectSize.y*sizeof(unsigned short);
		unsigned short *bufferMat=nullptr;//情報取得用
		unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//描画用
		if(drawMat==nullptr){
			throw(std::runtime_error("Memory is not satisfied."));
		}
		for(int i=0;i<KinectSize.x*KinectSize.y;i++){
			drawMat[i]=0;
		}
		

		//入力機構の初期化
		InitInputControler();

		//アプリケーション動作
		while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
			//ゲーム本体
			//キー情報更新
			input_update();
			//描画
			clsDx();
			const unsigned short *adress1=bufferMat;
			printfDx("%x\n",bufferMat);
			for(int y=0;y<KinectSize.y;y++){
				for(int x=0;x<KinectSize.x;x++){
					int color=drawMat[x+y*KinectSize.x];
					int c=color*255/8000;//値を0~255の範囲内に収める。drawmat内にある数値は500～8000で、mm単位での物体までの距離である
					if(c>255){
						//観測範囲を超えたところは黒く描画する
						c=0;
					}
					DrawPixel(x,y,GetColor(c,c,c));
				}
			}
			//計算処理
			try{
				IDepthFrame *pDepthFrame=nullptr;
				ErrorCheck(pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error");
				ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error");
				printfDx("success");
				//読み取りができたのでdrawMatを更新。鏡像なので左右反転させる。
				const unsigned short *adress2=bufferMat;
				if(bufferMat!=nullptr){
					for(int y=0;y<KinectSize.y;y++){
						for(int x=0;x<KinectSize.x;x++){
							drawMat[(KinectSize.x-x-1)+y*KinectSize.x]=bufferMat[x+y*KinectSize.x];
						}
					}
				}
				pDepthFrame->Release();//bufferMatの参照が終了したのでpDepthFrameを開放する
			} catch(const std::exception &e){
				printfDx(e.what());
			}
			//終了検出
			if(keyboard_get(KEY_INPUT_ESCAPE)>0){
				break;
			}
		}

		//終了処理
		if(drawMat!=nullptr){
			delete drawMat;
		}

		DeleteInputControler();//入力機構の解放

		pSensor->Close();
		pSensor->Release();

		DxLib_End();


		return 0;
	} catch(const std::exception &e){
		assert(e.what());
		return 1;
	}
}

