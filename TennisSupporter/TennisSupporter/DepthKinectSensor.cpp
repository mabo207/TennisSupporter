#include"DepthKinectSensor.h"
#include"DxLib.h"

//--------------DepthKinectSensor--------------
DepthKinectSensor::DepthKinectSensor(Vector2D i_kinectSize,IKinectSensor *pSensor)
	:kinectSize(i_kinectSize)
{
	//�`����̔z��̈�̊m��
	m_drawMat=new unsigned short[kinectSize.x*kinectSize.y];//���O�ɓǂݎ����depth�摜��ێ�����z��
	for(int i=0;i<kinectSize.x*kinectSize.y;i++){
		m_drawMat[i]=0;
	}
	//depth�摜�̃Z���T�[�̏�����
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
	unsigned short *bufferMat=nullptr;//���擾�p�|�C���^
	unsigned int bufferSize=kinectSize.x*kinectSize.y*sizeof(unsigned short);
	try{
		IDepthFrame *pDepthFrame=nullptr;
		ErrorCheck(m_pDepthReader->AcquireLatestFrame(&pDepthFrame),"acquire error\n");
		ErrorCheck(pDepthFrame->AccessUnderlyingBuffer(&bufferSize,&bufferMat),"access error\n");
		printfDx("success\n");
		//�ǂݎ�肪�ł����̂�m_drawMat���X�V�B�����Ȃ̂ō��E���]������B
		if(bufferMat!=nullptr){
			for(int y=0;y<kinectSize.y;y++){
				for(int x=0;x<kinectSize.x;x++){
					m_drawMat[(kinectSize.x-x-1)+y*kinectSize.x]=bufferMat[x+y*kinectSize.x];
				}
			}
		}
		pDepthFrame->Release();//bufferMat�̎Q�Ƃ��I�������̂�pDepthFrame���J������
	} catch(const std::exception &e){
		printfDx(e.what());
	}

	//���ɏI�����̏��`�B�͖����B0��Ԃ�
	return 0;
}

void DepthKinectSensor::Draw(Vector2D depthPos)const{
	for(int y=0;y<kinectSize.y;y++){
		for(int x=0;x<kinectSize.x;x++){
			int color=m_drawMat[x+y*kinectSize.x];
			int c=color*255/8000;//�l��0~255�͈͓̔��Ɏ��߂�Bdrawmat���ɂ��鐔�l��500�`8000�ŁAmm�P�ʂł̕��̂܂ł̋����ł���
			Vector2D leftUpPos=depthPos-kinectSize/2;
			DrawPixel(leftUpPos.x+x,leftUpPos.y+y,GetColor(c,c,c));
		}
	}
}
