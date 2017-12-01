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
const std::string BodySimulator::sectionStr="##################";

BodySimulator::BodySimulator()
	:m_fileWriteFlag(false),m_writeCount(0),m_mode(0),m_playFrame(0.0),m_playRate(1.0),m_dataMin(300),m_dataMax(-300),
	m_font(CreateFontToHandle("���C���I",12,1,-1)),m_playFlag(true),m_beforeRClickFrame(0),m_startSectionIndex(0)
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

	//�t�@�C�������g���q�ȊO�ۑ�
	std::string fname(filename);
	size_t index=0;
	for(size_t size=fname.size();index<size;index++){
		if(fname[index]=='.'){
			break;
		}
	}
	m_playDataName.clear();
	m_playDataName.reserve(index);
	for(size_t i=0;i<index;i++){
		m_playDataName.push_back(fname[i]);
	}

	//�Đ��f�[�^�̏�����
	m_playFlag=true;
	m_section.clear();
	m_beforeRClickFrame=0;
	m_startSectionIndex=0;

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
	//�t���[�������������A�C���[�W���X�V
	m_playFrame=0.0;
	UpdateImage();
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

void BodySimulator::UpdateImage(){
	if(m_mode==1){
		int index=CalReadIndex();
		if(index>=0 && index<(int)m_playData.size()){
			m_pBodyKinectSensor->Update(m_playData[index]);
		}
	}
}

void BodySimulator::WriteSections(){
	//�ۑ���t�@�C�����쐬
	const std::string filename=m_playDataName+"_section.txt";
	//�t�@�C�����J��
	std::ofstream ofs(filename.c_str(),std::ios_base::trunc);
	if(!ofs){
		return;
	}
	//�t�@�C���ɏ����o��
	int playdatasize=(int)m_playData.size();
	for(const std::pair<int,int> &pair:m_section){
		//�J�n�Ɩ������Z�o
		int top=std::fmin(pair.first,pair.second);
		int bottom=std::fmax(pair.first,pair.second);
		//�����o��
		if(top>=0 && bottom<playdatasize){
			//��؂蕶����Ɖ��s�̏����o��
			ofs<<sectionStr<<std::endl;
			//1�t���[���������o��
			for(int i=top;i<=bottom;i++){
				m_pBodyKinectSensor->OutputJointPoitions(ofs,m_playData[i]);
			}
		}
	}
	ofs.close();

	//�����o���I����������悤�ɍĐ����n�߂�
	m_playFlag=true;
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
			//�O���t���ō��N���b�N���A�Đ����Ԃ������ɍ��킹�āA�C���[�W�X�V
			m_playFrame=CalPlayFrame((GetMousePointVector2D().x-graphPos.x)*writeCountMax/graphSize.x);
			UpdateImage();
		} else if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
			//Enter�L�[���͂Ő擪����Đ����A�C���[�W�X�V
			m_playFrame=0.0;
			UpdateImage();
		} else if(keyboard_get(KEY_INPUT_RSHIFT)==1){
			//�E�V�t�g�L�[���͂ōĐ���~�̐؂�ւ�
			m_playFlag=!m_playFlag;
		} else if(keyboard_get(KEY_INPUT_LEFT)==1){
			//���L�[���͂ōĐ��ʒu��߂��A�C���[�W�X�V
			m_playFrame-=m_playRate;
			UpdateImage();
		} else if(keyboard_get(KEY_INPUT_RIGHT)==1){
			//�E�L�[���͂ōĐ��ʒu��i�߁A�C���[�W�X�V
			m_playFrame+=m_playRate;
			UpdateImage();
		}
		//�C���[�W�Đ����
		if(mouse_get(MOUSE_INPUT_RIGHT)<=0){
			//�E�N���b�N��������Ă��Ȃ��ꍇ�͕��ʂɍĐ�
			int a=CalReadIndex();
			if(m_playFlag && a<(int)m_playData.size()){
				//�܂��f�[�^�I�[�܂ł����Ă��炸�A���Đ����[�h�ɂȂ��Ă��鎞�A�t���[�������X�V
				m_playFrame+=m_playRate;
			}
			int b=CalReadIndex();//���̒l��a�Ɉ�v���Ă��鎞�͓ǂݍ��݂͍s�킸�A�O�t���[���Ɠ����摜��`�悷��
			if(a!=b){
				UpdateImage();
			}
		} else{
			//�E�N���b�N��������Ă���ꍇ�́A�}�E�X�̈ʒu�ɏ]���ăC���[�W���X�V
			size_t index=(size_t)(std::fmin(m_playData.size()-1,(size_t)(std::fmax(0,GetMousePointVector2D().x-graphPos.x))));
			m_pBodyKinectSensor->Update(m_playData[index]);
		}
		//���̓C���^�[�t�F�[�X
		if(m_pGraphDataBuilder->Update()==1){
			//m_dataFactory���X�V��������DataBuild()���g�p����
			DataBuild();
		}
		//�O���t�f�[�^�؂��葀��
		int rframe=mouse_get(MOUSE_INPUT_RIGHT);
		if(rframe==1){
			//�E�N���b�N�J�n
			m_startSectionIndex=CalReadIndex();//�J�nindex�̕ۑ�
		} else if(rframe==0 && m_beforeRClickFrame>0){
			//�E�N���b�N�𗣂�������
			Vector2D mousePos=GetMousePointVector2D();
			if(JudgeMouseInGraph()){
				m_section.push_back(std::pair<int,int>(m_startSectionIndex,(mousePos-graphPos).x*writeCountMax/graphSize.x));//��Ԃ̕ۑ�
			}
		}
		m_beforeRClickFrame=rframe;
		//�Z�N�V�����f�[�^�o�͑���
		if(keyboard_get(KEY_INPUT_S)==10){
			WriteSections();
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
			//���ݑ��݂��Ă����Ԃ̕\��
			for(const std::pair<int,int> &section:m_section){
				DrawBox(graphPos.x+section.first,graphPos.y,graphPos.x+section.second,graphPos.y+graphSize.y,GetColor(128,128,255),TRUE);
			}
			//���ݍ���Ă����Ԃ̕\��
			if(mouse_get(MOUSE_INPUT_RIGHT)>1){
				DrawBox(graphPos.x+m_startSectionIndex,graphPos.y,GetMousePointVector2D().x,graphPos.y+graphSize.y,GetColor(255,255,0),TRUE);
			}

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
