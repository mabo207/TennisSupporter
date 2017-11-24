#include"DepthKinectSensor.h"
#include"DxLib.h"

//--------------DepthKinectSensor--------------
DepthKinectSensor::DepthKinectSensor(Vector2D i_kinectSize,IKinectSensor *pSensor)
	:kinectSize(i_kinectSize)
{
	//描画情報の配列領域の確保
	m_drawMat=new unsigned short[kinectSize.x*kinectSize.y];//直前に読み取ったdepth画像を保持する配列
	for(int i=0;i<kinectSize.x*kinectSize.y;i++){
		m_drawMat[i]=0;
	}
	//depth画像のセンサーの初期化
	//source
	IDepthFrameSource *pDepthSource;
	ErrorCheck(pSensor->get_DepthFrameSource(&pDepthSource),"You can't get source.");
	//reader
	ErrorCheck(pDepthSource->OpenReader(&m_pDepthReader),"You can't open reader.");

}

DepthKinectSensor::~DepthKinectSensor(){
	if(m_drawMat!=nullptr){
		delete [] m_drawMat;
	}
}

int DepthKinectSensor::Update(){
	unsigned short *bufferMat=nullptr;//情報取得用ポインタ
	unsigned int bufferSize=kinectSize.x*kinectSize.y*sizeof(unsigned short);
	try{
		IDepthFrame *pDepthFrame=nullptr;
		ErrorCheck(m_pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error\n");
		ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error\n");
		printfDx("success\n");
		//読み取りができたのでm_drawMatを更新。鏡像なので左右反転させる。
		if(bufferMat!=nullptr){
			for(int y=0;y<kinectSize.y;y++){
				for(int x=0;x<kinectSize.x;x++){
					m_drawMat[(kinectSize.x-x-1)+y*kinectSize.x]=bufferMat[x+y*kinectSize.x];
				}
			}
		}
		pDepthFrame->Release();//bufferMatの参照が終了したのでpDepthFrameを開放する
	} catch(const std::exception &e){
		printfDx(e.what());
	}

	//特に終了時の情報伝達は無い。0を返す
	return 0;
}

void DepthKinectSensor::Draw(Vector2D depthPos)const{
	for(int y=0;y<kinectSize.y;y++){
		for(int x=0;x<kinectSize.x;x++){
			int color=m_drawMat[x+y*kinectSize.x];
			int c=color*255/8000;//値を0~255の範囲内に収める。drawmat内にある数値は500～8000で、mm単位での物体までの距離である
			Vector2D leftUpPos=depthPos-kinectSize/2;
			DrawPixel(leftUpPos.x+x,leftUpPos.y+y,GetColor(c,c,c));
		}
	}
}
