#define _USE_MATH_DEFINES

#include<cmath>
#include"BodySimulator.h"
#include"DxLib.h"
#include"input.h"

//-----------------BodySimulator-----------------
const int BodySimulator::writeCountMax=30*30;
const int BodySimulator::captureFps=30;
const int BodySimulator::drawFps=60;
const Vector2D BodySimulator::kinectSize=Vector2D(512,424);
const Vector2D BodySimulator::graphPos=Vector2D(100,60);
const Vector2D BodySimulator::graphSize=Vector2D(BodySimulator::writeCountMax,360);

BodySimulator::BodySimulator()
	:m_fileWriteFlag(false),m_writeCount(0),m_mode(0),m_playFrame(0.0),m_playRate(1.0),m_dataMin(300),m_dataMax(-300),
	m_font(CreateFontToHandle("���C���I",12,1,-1)),m_playFlag(true)
{
	//�Z���T�[�̋N��
	m_pSensor=nullptr;
	ErrorCheck(GetDefaultKinectSensor(&m_pSensor),"You can't get Kinect Sensor.");
	ErrorCheck(m_pSensor->Open(),"You can't activate Kinect Sensor.");
	BOOLEAN isOpen=false;
	ErrorCheck(m_pSensor->get_IsOpen(&isOpen),"Kinect is not open.");
	//BodySensor�̋N��
	m_pBodyKinectSensor=std::shared_ptr<BodyKinectSensor>(new BodyKinectSensor(m_pSensor));
	//DepthKinectSensor�̋N��
	m_pDepthKinectSensor=std::shared_ptr<DepthKinectSensor>(new DepthKinectSensor(kinectSize,m_pSensor));
	//GraphDataBuilder�̋N��
	m_pGraphDataBuilder=std::shared_ptr<GraphDataBuilder>(new GraphDataBuilder(Vector2D(kinectSize.x*2,0)));
	//m_writeFile,m_readFile�́A�K�v�ɂȂ莟�揉��������B

}

BodySimulator::~BodySimulator(){
	m_pSensor->Close();
	m_pSensor->Release();

	m_writeFile.close();
	
	DeleteFontToHandle(m_font);
}

bool BodySimulator::ReadFile(const char *filename){
	std::ifstream readFile(filename);
	//�f�[�^��S�ēǂݍ��ނ��A�ʂ������̂ŏ�肭reserve���Ȃ���ǂݍ���
	m_playData.clear();
	m_playData.reserve(writeCountMax);//�ő�L�^�t���[�����ɂ��reserve

	std::vector<std::vector<BodyKinectSensor::JointPosition>> frameData;
	frameData.reserve(BodyKinectSensor::bodyNum);

	std::vector<BodyKinectSensor::JointPosition> bodyData;
	bodyData.reserve(JointType_Count);
	
	size_t bodyindex=0,jointindex=0;
	std::string parts="";
	parts.reserve(50);
	bool inBracketsFlag=false;//����"(~~)"�̒���ǂݎ���Ă��邩�ǂ���
	char ch='a';//�����l�̓e�L�g�[
	try{
		while(true){
			ch=readFile.get();
			if(ch==EOF){
				//�t�@�C���̖������B��
				break;
			} else if(ch=='\n'){
				//1�s�ǂݏI��鎞(==1�t���[���ǂݏI���)
				m_playData.push_back(frameData);
				frameData.clear();
				bodyData.clear();
				bodyindex=0;
				jointindex=0;
				parts="";
				inBracketsFlag=false;
			} else{
				if(inBracketsFlag){
					//()���ǂݎ�莞(=')'��T���Ă���)
					parts.push_back(ch);
					if(ch==')'){
						inBracketsFlag=!inBracketsFlag;
						//jointPositions�Ɋi�[
						if(bodyindex<BodyKinectSensor::bodyNum && jointindex<JointType_Count){
							bodyData.push_back(BodyKinectSensor::JointPosition(parts));
							//index�n�̍X�V
							jointindex++;
							if(jointindex>=JointType_Count){
								jointindex=0;
								bodyindex++;
								frameData.push_back(bodyData);
								bodyData.clear();
							}
						}
						parts.clear();
					}
				} else{
					//()�O�ǂݎ�莞(='('��T���Ă���)
					if(ch=='('){
						inBracketsFlag=!inBracketsFlag;
						parts.push_back(ch);
					}
				}
			}
		}
	}catch(const std::exception &e){
		//�������֘A�̃G���[�΍�
		printf(e.what());
		return false;
	}
	//�O���t�ɂ��Ẵf�[�^�ǂݎ��
	DataBuild();
	readFile.close();

	//�Đ��f�[�^�̏�����
	m_playFlag=true;

	return true;
}

void BodySimulator::DataBuild(){
	size_t playdatasize=m_playData.size();
	m_data.clear();
	m_data.reserve(playdatasize);
	for(size_t i=0;i<playdatasize;i++){
		double data=0.0;
		for(size_t j=0,bodynum=m_playData[i].size();j<bodynum;j++){
			bool flag=false;
			for(size_t k=0,jointnum=m_playData[i][j].size();k<jointnum;k++){
				if(!(m_playData[i][j][k]==BodyKinectSensor::JointPosition())){
					flag=true;
					break;
				}
			}
			if(flag){
				data=m_pGraphDataBuilder->CalData(m_playData[i][j]);
				break;
			}
		}
		m_data.push_back(data);
		if(i!=0){
			m_dataMin=std::fmin(m_dataMin,data);
			m_dataMax=std::fmax(m_dataMax,data);
		} else{
			m_dataMin=data;
			m_dataMax=data;
		}
	}
	m_playFrame=0.0;
}

int BodySimulator::CalReadIndex()const{
	return (int)(m_playFrame*captureFps/drawFps);
}

double BodySimulator::CalPlayFrame(int index)const{
	return ((double)index)*drawFps/captureFps;
}

bool BodySimulator::JudgeMouseInGraph()const{
	Vector2D relativeMouse=GetMousePointVector2D()-graphPos;
	return (relativeMouse.x>=0 && relativeMouse.x<=graphSize.x && relativeMouse.y>=0 && relativeMouse.y<=graphSize.y);
}

int BodySimulator::Update(){
	switch(m_mode){
		//�L�^���[�h
	case(0):
		printfDx("RecordingDataMode\n");
		//depth
		//�f�[�^�̓ǂݎ��

		//depth
		printfDx("depth:\n");
		m_pDepthKinectSensor->Update();

		//body
		printfDx("body:\n");
		m_pBodyKinectSensor->Update();
		//���̋L�^�����邩�̃t���O�̍X�V
		if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
			m_fileWriteFlag=!m_fileWriteFlag;
			if(m_fileWriteFlag){
				//�L�^�J�n���̓t�@�C�����J���Am_writeCount��0�ɂ���
				m_writeCount=0;
				m_writeFile.open("SaveData/"+to_string_0d(0,3)+".txt",std::ios_base::trunc);
				if(!m_writeFile){
					//�t�@�C�����J���Ȃ���΋L�^�J�n���Ȃ�
					m_fileWriteFlag=false;
				}
			} else{
				//�L�^�I�����̓t�@�C�������
				m_writeFile.close();
			}
		}
		//�t�@�C���o��
		printfDx("fileWriteFlag:\n");
		printfDx((m_fileWriteFlag && !(!m_writeFile)) ? "true\n":"false\n");
		if(m_fileWriteFlag){
			//�������݂�������
			m_writeCount++;
			if(m_writeCount>writeCountMax){
				m_fileWriteFlag=false;
			}
			//body�ʒu�̏o��
			m_pBodyKinectSensor->OutputJointPoitions(m_writeFile);
		}
		//�L�^�f�[�^�Đ����[�h�ւ̈ڍs����
		if(keyboard_get(KEY_INPUT_0)==1){
			if(ReadFile(("SaveData/"+to_string_0d(0,3)+".txt").c_str())){
				//�ǂݍ��ݐ������̂݁A�Đ����[�h��
				m_mode=1;
			}
		}
		break;
		//�Đ����[�h
	case(1):
		printfDx("PlayingDataMode\n");
		//�b���X�V
		if(mouse_get(MOUSE_INPUT_LEFT)>0 && JudgeMouseInGraph()){
			//�O���t���ō��N���b�N���A�Đ����Ԃ������ɍ��킹��
			m_playFrame=CalPlayFrame((GetMousePointVector2D().x-graphPos.x)*writeCountMax/graphSize.x);
		} else if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
			//Enter�L�[���͂Ő擪����Đ�
			m_playFrame=0.0;
		} else if(keyboard_get(KEY_INPUT_RSHIFT)==1){
			//�E�V�t�g�L�[���͂ōĐ���~�̐؂�ւ�
			m_playFlag=!m_playFlag;
		}
		//�C���[�W�Đ����
		int a=CalReadIndex();
		if(m_playFlag && a<(int)m_playData.size()){
			//�܂��f�[�^�I�[�܂ł����Ă��炸�A���Đ����[�h�ɂȂ��Ă��鎞�A�t���[�������X�V
			m_playFrame+=m_playRate;
		}
		int b=CalReadIndex();//���̒l��a�Ɉ�v���Ă��鎞�͓ǂݍ��݂͍s�킸�A�O�t���[���Ɠ����摜��`�悷��
		if(a!=b){
			if(b<(int)m_playData.size()){
				m_pBodyKinectSensor->Update(m_playData[b]);
			}
		}
		//���̓C���^�[�t�F�[�X
		if(m_pGraphDataBuilder->Update()==1){
			//m_dataFactory���X�V��������DataBuild()���g�p����
			DataBuild();
		}
		//��ʑJ��
		if(keyboard_get(KEY_INPUT_BACK)==1){
			//Back�L�[���͂ŋL�^���[�h�ɖ߂�
			m_mode=0;
		}
		break;
	}
	//�S���[�h���ʏ���
	//�Đ����[�h�ɂ�����Đ����x����
	if(keyboard_get(KEY_INPUT_Z)>0){
		//�x������
		m_playRate=std::fmax(0.1,m_playRate-0.05);
	} else if(keyboard_get(KEY_INPUT_X)>0){
		//1.0�{�ɖ߂�
		m_playRate=1.0;
	} else if(keyboard_get(KEY_INPUT_C)>0){
		//��������
		m_playRate=m_playRate+0.05;
	}
	printfDx("playRate:\n%f",m_playRate);


	//���ɏI�������͖����̂�0����ɕԂ�
	return 0;
}

void BodySimulator::Draw()const{
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	switch(m_mode){
	case(0):
		//�f�[�^�L�^��
		m_pDepthKinectSensor->Draw(depthPos);
		m_pBodyKinectSensor->Draw(m_pSensor,depthPos,kinectSize,xyPos,kinectSize,zyPos,kinectSize);
		break;
	case(1):
		//�Đ���
		m_pBodyKinectSensor->Draw(m_pSensor,Vector2D(-3000,-3000),kinectSize,xyPos,kinectSize,zyPos,kinectSize);//(depth�摜�ɑ΂���body�{�[���͕`�悵�Ȃ�)
		//�O���t�`��
		{
			//�܂���̕`��
			for(size_t i=0,datanum=m_data.size();i<datanum;i++){
				DrawPixel(graphPos.x+i,graphPos.y+(int)(graphSize.y/std::fmax(m_dataMax-m_dataMin,0.00001)*(m_dataMax-m_data[i])),GetColor(128,255,255));
			}
			//�Đ����Ԃ̕`��
			DrawLine(graphPos.x+CalReadIndex(),graphPos.y,graphPos.x+CalReadIndex(),graphPos.y+graphSize.y,GetColor(128,128,128),1);
			//���݂̒l�̕\��
			if(CalReadIndex()>=0 && CalReadIndex()<(int)m_data.size()){
				//�z��O�Q�Ƃ�����\��������̂Œe���B�z��O�Q�Ǝ��͕`�悵�Ȃ��B
				int dataY=graphPos.y+(int)(graphSize.y/std::fmax(m_dataMax-m_dataMin,0.00001)*(m_dataMax-m_data[CalReadIndex()]));
				DrawLine(graphPos.x,dataY,graphPos.x+graphSize.x,dataY,GetColor(128,128,128),1);
			}
			//data�ő�l�̕\��
			DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
			DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(m_dataMax),GetColor(255,255,255),m_font);
			//data�ŏ��l�̕\��
			DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
			DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(m_dataMin),GetColor(255,255,255),m_font);
			
			//�ǂݍ��݃f�[�^�C���^�[�t�F�[�X�̕`��
			m_pGraphDataBuilder->Draw();
		}
		break;
	}
	
}
