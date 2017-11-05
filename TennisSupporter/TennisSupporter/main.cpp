#include<cassert>
#include"DxLib.h"
#include<Kinect.h>
#include"input.h"

#include<set>
#include<vector>

void ErrorCheck(HRESULT hresult,const char *msg)noexcept(false){
	if(hresult!=S_OK){
		throw(std::runtime_error(msg));
	}
}

void DepthSimulate(bool objectcheck,Vector2D KinectSize)noexcept(false){
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
	unsigned short *bufferMat=nullptr;//情報取得用ポインタ（Kinectが情報取得のために確保したメモリにアクセスするので、毎フレームポインタの値が変わる）
	unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//直前に読み取ったdepth画像を保持する配列
	unsigned char *readMat=new unsigned char[KinectSize.x*KinectSize.y/8];//物体検出の際に既に物体があるかを調べたピクセルかを0(false)か1(true)で記憶して計算を効率化する。sizeof(char)=1byte=8bitは環境に依存しないのを利用し、(x,y)の判定はreadMat[(x+y*KinectSize.x)/8]の上から((x+y*KinectSize.x)%8)bit目(一番上を0bit目とする)に格納。
	if(drawMat==nullptr || readMat==nullptr){
		throw(std::runtime_error("Memory is not satisfied."));
	}
	for(int i=0;i<KinectSize.x*KinectSize.y;i++){
		drawMat[i]=0;
	}
	std::vector<std::set<int>> contoursVec={};//輪郭線一覧

	//アプリケーション動作
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//ゲーム本体
		//キー情報更新
		input_update();
		//描画
		clsDx();
		const unsigned short *adress1=bufferMat;
		printfDx("BufferMat:%x\ncontoursVec size:%d\n",bufferMat,contoursVec.size());
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				int color=drawMat[x+y*KinectSize.x];
				int c=color*255/8000;//値を0~255の範囲内に収める。drawmat内にある数値は500～8000で、mm単位での物体までの距離である
				DrawPixel(x,y,GetColor(c,c,c));
			}
		}
		//輪郭線描画
		for(const std::set<int> &set:contoursVec){
			for(const int &index:set){
				int x=index%KinectSize.x;
				int y=index/KinectSize.x;
				DrawPixel(x,y,GetColor(0,0,255));
			}
		}
		//計算処理
		//データの読み取り
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

		//物体検出
		//認識できなかったピクセル部分のdepthを0にする。
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				if(drawMat[x+y*KinectSize.x]>8000){
					drawMat[x+y*KinectSize.x]=0;
				}
			}
		}
		if(objectcheck){
			//readMat初期化
			for(int i=0;i<KinectSize.x*KinectSize.y/8;i++){
				readMat[i]=0;
			}
			//contoursVec初期化
			contoursVec.clear();
			//定数定義
			const int acceptdistance=300;//50mm以内のdepth距離の違いがあるピクセルに進んでいく
										 //物体が検出できたピクセルを検出し、物体認識をしていく
			for(int i=0;i<KinectSize.x*KinectSize.y;i++){
				bool insertflag=true;//次に調べる物体をcontoursVecに追加するかどうか
									 //まだ物体検出がされておらず、物体検出したピクセルを見つけたら
				if(drawMat[i]>0 && (readMat[i/8] & 1<<(7-i%8))==0){
					//そのピクセルから時計回りに距離が近いピクセルを探していくことで、物体の輪郭を求める
					int index=i;
					std::set<int> contours={};//輪郭を表すピクセルの集合
					int indextable[8][2]={{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};
					int finalJ=3;//検索するindextableが毎回のdo~whileループ内で同じ箇所から始まらないように（対称な点の次の点から始まるように）、直前でどのindextableを検索したかを記録しておく。初期値はぶっちゃけなんでもよい。
					do{
						//現在のx,y座標を求める
						int x=index%KinectSize.x,y=index/KinectSize.x;
						//周囲8ピクセルのどこにdepth距離の近いピクセルがあるかを検索してindexの更新
						for(int j=(finalJ+5)%8;j!=(finalJ+4)%8;j=(j+1)%8){
							int xx=x+indextable[j][0];
							int yy=y+indextable[j][1];
							//(xx,yy)のピクセルが存在しているかの判定
							if(xx>=0 && xx<KinectSize.x && yy>=0 && yy<KinectSize.y){
								//depth距離が近いかの判定
								int nextindex=xx+yy*KinectSize.x;//調べるピクセルに対応する配列番号
								int d1=drawMat[index];
								int d2=drawMat[nextindex];
								if(std::abs(drawMat[index]-drawMat[nextindex])<=acceptdistance){
									index=nextindex;
									finalJ=j;
									break;
								}
							}
						}
						//輪郭集合に情報を格納する
						contours.insert(index);
						//readMatを更新
						unsigned char byte=1<<(7-index%8);
						if((readMat[index/8] & byte)!=0){
							//次のピクセルが既にreadMatが0なら、そのピクセルは既に検出された別の物体に属しているので、これ以上調べても無駄になる。輪郭集合に入れない事も伝えつつ探索終了する。
							insertflag=false;
							break;
						} else{
							readMat[index/8]=readMat[index/8] | byte;
						}
					} while(index!=i);
					if(insertflag){
						//既に調べた物体でなければ
						if(contours.size()>8){
							//ある程度の大きさがあれば
							contoursVec.push_back(contours);
						}
					}
				}
			}
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
	if(readMat!=nullptr){
		delete readMat;
	}


	pSensor->Close();
	pSensor->Release();

}

void BodySimulate(Vector2D KinectSize){
	//kinectの初期化
	//bodyについて
	//sensor
	IKinectSensor *pSensor=nullptr;
	ErrorCheck(GetDefaultKinectSensor(&pSensor),"You can't get Kinect Sensor.");
	ErrorCheck(pSensor->Open(),"You can't activate Kinect Sensor.");
	BOOLEAN isOpen=false;
	ErrorCheck(pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
	//source
	IBodyFrameSource *pBodySource;
	ErrorCheck(pSensor->get_BodyFrameSource(&pBodySource),"You can't get source.");
	//reader
	IBodyFrameReader *pBodyReader;
	ErrorCheck(pBodySource->OpenReader(&pBodyReader),"You can't open reader.");
	

	IBodyFrame *pBodyFrame=nullptr;
	IBody *pBodies[6];
	for(size_t i=0;i<6;i++){
		pBodies[i]=nullptr;
	}

	//可視化のためにdepth画像も表示
	//source
	IDepthFrameSource *pDepthSource;
	ErrorCheck(pSensor->get_DepthFrameSource(&pDepthSource),"You can't get source.");
	//reader
	IDepthFrameReader *pDepthReader;
	ErrorCheck(pDepthSource->OpenReader(&pDepthReader),"You can't open reader.");
	//memory
	unsigned int bufferSize=KinectSize.x*KinectSize.y*sizeof(unsigned short);
	unsigned short *bufferMat=nullptr;//情報取得用ポインタ（Kinectが情報取得のために確保したメモリにアクセスするので、毎フレームポインタの値が変わる）
	unsigned short *drawMat=new unsigned short[KinectSize.x*KinectSize.y];//直前に読み取ったdepth画像を保持する配列
	unsigned char *readMat=new unsigned char[KinectSize.x*KinectSize.y/8];//物体検出の際に既に物体があるかを調べたピクセルかを0(false)か1(true)で記憶して計算を効率化する。sizeof(char)=1byte=8bitは環境に依存しないのを利用し、(x,y)の判定はreadMat[(x+y*KinectSize.x)/8]の上から((x+y*KinectSize.x)%8)bit目(一番上を0bit目とする)に格納。
	if(drawMat==nullptr || readMat==nullptr){
		throw(std::runtime_error("Memory is not satisfied."));
	}
	for(int i=0;i<KinectSize.x*KinectSize.y;i++){
		drawMat[i]=0;
	}


	//アプリケーション動作
	while(ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		//ゲーム本体
		//キー情報更新
		input_update();
		
		//描画
		clsDx();
		//depth画像描画
		for(int y=0;y<KinectSize.y;y++){
			for(int x=0;x<KinectSize.x;x++){
				int color=drawMat[x+y*KinectSize.x];
				int c=color*255/8000;//値を0~255の範囲内に収める。drawmat内にある数値は500～8000で、mm単位での物体までの距離である
				DrawPixel(x,y,GetColor(c,c,c));
			}
		}
		//複数あるbodyそれぞれに対して処理を行う
		for(auto pBody:pBodies){
			if(pBody==nullptr){
				continue;
			}
			try{
				BOOLEAN flag;
				ErrorCheck(pBody->get_IsTracked(&flag),"");
			} catch(const std::exception &e){
				continue;
			}
			//関節の取得
			Joint joints[JointType::JointType_Count];
			pBody->GetJoints(JointType::JointType_Count,joints);
			//各関節に対する処理
			for(Joint joint:joints){
				try{
					ICoordinateMapper *mapper;
					ErrorCheck(pSensor->get_CoordinateMapper(&mapper),"mapper failed\n");
					DepthSpacePoint point;//opencv系の座標。すなわちdxlibと同じ。
					mapper->MapCameraPointToDepthSpace(joint.Position,&point);
					DrawCircle(KinectSize.x-(int)point.X,(int)point.Y,5,GetColor(0,255,0),TRUE);//鏡像なので反転させて表示
				} catch(const std::exception &e){
					printfDx(e.what());
				}
			}
		}
		
		//情報更新
		//depth
		//データの読み取り
		printfDx("depth:\n");
		try{
			IDepthFrame *pDepthFrame=nullptr;
			ErrorCheck(pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error\n");
			ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error\n");
			printfDx("success\n");
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
		//body
		printfDx("body:\n");
		try{
			ErrorCheck(pBodyReader->AcquireLatestFrame(&pBodyFrame),"aquaire failed\n");//直近フレームのbodyデータの取得
			ErrorCheck(pBodyFrame->GetAndRefreshBodyData(6,pBodies),"access failed\n");//bodyデータをpBodiesに格納
			pBodyFrame->Release();//これ以降pBodyFrameは使わない
			printfDx("success\n");
		} catch(const std::exception &e){
			printfDx(e.what());
		}
		


		//終了検出
		if(keyboard_get(KEY_INPUT_ESCAPE)>0){
			break;
		}
	}

	//終了処理

	pSensor->Close();
	pSensor->Release();

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

		//入力機構の初期化
		InitInputControler();

		//実行
		//DepthSimulate(false,KinectSize);
		BodySimulate(KinectSize);

		//終了処理
		DeleteInputControler();//入力機構の解放
		DxLib_End();


		return 0;
	} catch(const std::exception &e){
		assert(e.what());
		return 1;
	}
}

