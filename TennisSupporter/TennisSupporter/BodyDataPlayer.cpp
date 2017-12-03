#define _USE_MATH_DEFINES

#include<cmath>
#include"BodyDataPlayer.h"
#include"DxLib.h"
#include"input.h"

//-----------------BodyDataPlayer-----------------
const Vector2D BodyDataPlayer::graphPos=Vector2D(100,60);
const Vector2D BodyDataPlayer::graphSize=Vector2D(BodyDataPlayer::writeCountMax,360);
const std::string BodyDataPlayer::sectionStr="##################";

BodyDataPlayer::BodyDataPlayer(int font,const char *filename)
	:IBodySimulateScene(MODE::PLAYER),m_playFrame(0.0),m_playRate(1.0),
	m_font(font),m_playFlag(true),m_beforeRClickFrame(0),m_startSectionIndex(0),m_graphUnity(false)
{
	//BodyVirtualKinectSensor�̋N��
	m_graphSingleData.m_pBodyVirtualKinectSensor=std::shared_ptr<BodyVirtualKinectSensor>(new BodyVirtualKinectSensor());
	//GraphDataBuilder�̋N��
	m_pGraphDataBuilder=std::shared_ptr<GraphDataBuilder>(new GraphDataBuilder(Vector2D(kinectSize.x*2,0)));
	//�f�[�^�̓ǂݎ��
	ReadFile(filename);
}

BodyDataPlayer::~BodyDataPlayer(){}

bool BodyDataPlayer::ReadFile(const char *filename){
	std::ifstream readFile(filename);
	//�f�[�^��S�ēǂݍ��ނ��A�ʂ������̂ŏ�肭reserve���Ȃ���ǂݍ���
	m_graphSingleData.m_playData.clear();
	m_graphSingleData.m_playData.reserve(writeCountMax);//�ő�L�^�t���[�����ɂ��reserve

	std::vector<std::vector<IBodyKinectSensor::JointPosition>> frameData;
	frameData.reserve(IBodyKinectSensor::bodyNum);

	std::vector<IBodyKinectSensor::JointPosition> bodyData;
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
				m_graphSingleData.m_playData.push_back(frameData);
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
						if(bodyindex<IBodyKinectSensor::bodyNum && jointindex<JointType_Count){
							bodyData.push_back(IBodyKinectSensor::JointPosition(parts));
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
	} catch(const std::exception &e){
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

void BodyDataPlayer::DataBuild(){
	m_graphSingleData.DataBuild(m_pGraphDataBuilder);
	//�t���[�������������A�C���[�W���X�V
	m_playFrame=0.0;
	UpdateImage();
}

int BodyDataPlayer::CalReadIndex()const{
	return (int)(m_playFrame*captureFps/drawFps);
}

double BodyDataPlayer::CalPlayFrame(int index)const{
	return ((double)index)*drawFps/captureFps;
}

bool BodyDataPlayer::JudgeMouseInGraph()const{
	Vector2D relativeMouse=GetMousePointVector2D()-graphPos;
	return (relativeMouse.x>=0 && relativeMouse.x<=graphSize.x && relativeMouse.y>=0 && relativeMouse.y<=graphSize.y);
}

void BodyDataPlayer::UpdateImage(){
	m_graphSingleData.UpdateVirtualSensor(CalReadIndex());
}

void BodyDataPlayer::WriteSections(){
	//�ۑ���t�@�C�����쐬
	const std::string filename=m_playDataName+"_section.txt";
	//�t�@�C�����J��
	std::ofstream ofs(filename.c_str(),std::ios_base::trunc);
	if(!ofs){
		return;
	}
	//�t�@�C���ɏ����o��
	int playdatasize=(int)m_graphSingleData.m_playData.size();
	for(const std::pair<int,int> &pair:m_section){
		//�J�n�Ɩ������Z�o
		int top=(int)std::fmin(pair.first,pair.second);
		int bottom=(int)std::fmax(pair.first,pair.second);
		//�����o��
		if(top>=0 && bottom<playdatasize){
			//��؂蕶����Ɖ��s�̏����o��
			ofs<<sectionStr<<std::endl;
			//1�t���[���������o��
			for(int i=top;i<=bottom;i++){
				m_graphSingleData.m_pBodyVirtualKinectSensor->OutputJointPoitions(ofs,m_graphSingleData.m_playData[i]);
			}
		}
	}
	ofs.close();

	//�����o���I����������悤�ɍĐ����n�߂�
	m_playFlag=true;
}

int BodyDataPlayer::Update(){
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
		if(m_playFlag && a<(int)m_graphSingleData.m_playData.size()){
			//�܂��f�[�^�I�[�܂ł����Ă��炸�A���Đ����[�h�ɂȂ��Ă��鎞�A�t���[�������X�V
			m_playFrame+=m_playRate;
		}
		int b=CalReadIndex();//���̒l��a�Ɉ�v���Ă��鎞�͓ǂݍ��݂͍s�킸�A�O�t���[���Ɠ����摜��`�悷��
		if(a!=b){
			UpdateImage();
		}
	} else{
		//�E�N���b�N��������Ă���ꍇ�́A�}�E�X�̈ʒu�ɏ]���ăC���[�W���X�V
		size_t index=(size_t)(std::fmin(m_graphSingleData.m_playData.size()-1,(size_t)(std::fmax(0,GetMousePointVector2D().x-graphPos.x))));
		m_graphSingleData.UpdateVirtualSensor(index);
	}
	//���̓C���^�[�t�F�[�X
	if(m_pGraphDataBuilder->Update()==1){
		//m_dataFactory���X�V��������DataBuild()���g�p����
		DataBuild();
	}
	//�O���t�f�[�^�؂��葀��
	int rframe=mouse_get(MOUSE_INPUT_RIGHT);
	if(rframe==1 && !m_graphUnity){
		//�E�N���b�N�J�n
		m_startSectionIndex=CalReadIndex();//�J�nindex�̕ۑ�
	} else if(rframe==0 && m_beforeRClickFrame>0 && !m_graphUnity){
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
	//�O���t�̎���̐؂�ւ�
	if(keyboard_get(KEY_INPUT_U)==1){
		m_graphUnity=!m_graphUnity;
	}
	//�Đ����x����
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


	//��ʑJ��
	if(keyboard_get(KEY_INPUT_BACK)==1){
		//Back�L�[���͂ŋL�^���[�h�ɖ߂�
		return 1;
	}
	return 0;
}

void BodyDataPlayer::Draw()const{
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	//�Đ���
	m_graphSingleData.m_pBodyVirtualKinectSensor->Draw(nullptr,Vector2D(-3000,-3000),kinectSize,xyPos,kinectSize,zyPos,kinectSize);//(depth�摜�ɑ΂���body�{�[���͕`�悵�Ȃ�)
	//�O���t�`��
	if(m_graphUnity){
		//����(1px�ɂ�1�t���[���Ƃ���������ł���)
		//�ő�ŏ��l�̐ݒ�
		const double dataTop=m_pGraphDataBuilder->DataMax(),dataBottom=m_pGraphDataBuilder->DataMin();
		//1�t���[���ɑ΂���s�N�Z�����̌v�Z
		const double frameRateToPixel=((double)writeCountMax)/m_graphSingleData.m_data.size();
		//���ݑ��݂��Ă����Ԃ̕\��
		for(const std::pair<int,int> &section:m_section){
			DrawBox(graphPos.x+(int)(section.first*frameRateToPixel),graphPos.y,graphPos.x+(int)(section.second*frameRateToPixel),graphPos.y+graphSize.y,GetColor(128,128,255),TRUE);
		}
		//���ݍ���Ă����Ԃ̕\��
		if(mouse_get(MOUSE_INPUT_RIGHT)>1){
			DrawBox(graphPos.x+(int)(m_startSectionIndex*frameRateToPixel),graphPos.y,GetMousePointVector2D().x,graphPos.y+graphSize.y,GetColor(255,255,0),TRUE);
		}

		//�܂���̕`��
		for(size_t i=0,datanum=m_graphSingleData.m_data.size();i<datanum;i++){
			DrawCircle(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[i])),1,GetColor(128,255,255),TRUE);
		}
		for(size_t i=0,datanum=m_graphSingleData.m_data.size();i+1<datanum;i++){
			DrawLine(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[i])),graphPos.x+(int)((i+1)*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[i+1])),GetColor(128,128,255),1);
		}
		//�Đ����Ԃ̕`��
		DrawLine(graphPos.x+(int)(CalReadIndex()*frameRateToPixel),graphPos.y,graphPos.x+(int)(CalReadIndex()*frameRateToPixel),graphPos.y+graphSize.y,GetColor(128,128,128),1);
		//���݂̒l�̕\��
		if(CalReadIndex()>=0 && CalReadIndex()<(int)m_graphSingleData.m_data.size()){
			//�z��O�Q�Ƃ�����\��������̂Œe���B�z��O�Q�Ǝ��͕`�悵�Ȃ��B
			int dataY=graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphSingleData.m_data[CalReadIndex()]));
			DrawLine(graphPos.x,dataY,graphPos.x+graphSize.x,dataY,GetColor(128,128,128),1);
		}
		//data�ő�l�̕\��
		DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(dataTop),GetColor(255,255,255),m_font);
		//data�ŏ��l�̕\��
		DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(dataBottom),GetColor(255,255,255),m_font);
	} else{
		//�f�[�^�ˑ��̊
		//���ݑ��݂��Ă����Ԃ̕\��
		for(const std::pair<int,int> &section:m_section){
			DrawBox(graphPos.x+section.first,graphPos.y,graphPos.x+section.second,graphPos.y+graphSize.y,GetColor(128,128,255),TRUE);
		}
		//���ݍ���Ă����Ԃ̕\��
		if(mouse_get(MOUSE_INPUT_RIGHT)>1){
			DrawBox(graphPos.x+m_startSectionIndex,graphPos.y,GetMousePointVector2D().x,graphPos.y+graphSize.y,GetColor(255,255,0),TRUE);
		}

		//�܂���̕`��
		for(size_t i=0,datanum=m_graphSingleData.m_data.size();i<datanum;i++){
			DrawPixel(graphPos.x+i,graphPos.y+(int)(graphSize.y/std::fmax(m_graphSingleData.m_dataMax-m_graphSingleData.m_dataMin,0.00001)*(m_graphSingleData.m_dataMax-m_graphSingleData.m_data[i])),GetColor(128,255,255));
		}
		//�Đ����Ԃ̕`��
		DrawLine(graphPos.x+CalReadIndex(),graphPos.y,graphPos.x+CalReadIndex(),graphPos.y+graphSize.y,GetColor(128,128,128),1);
		//���݂̒l�̕\��
		if(CalReadIndex()>=0 && CalReadIndex()<(int)m_graphSingleData.m_data.size()){
			//�z��O�Q�Ƃ�����\��������̂Œe���B�z��O�Q�Ǝ��͕`�悵�Ȃ��B
			int dataY=graphPos.y+(int)(graphSize.y/std::fmax(m_graphSingleData.m_dataMax-m_graphSingleData.m_dataMin,0.00001)*(m_graphSingleData.m_dataMax-m_graphSingleData.m_data[CalReadIndex()]));
			DrawLine(graphPos.x,dataY,graphPos.x+graphSize.x,dataY,GetColor(128,128,128),1);
		}
		//data�ő�l�̕\��
		DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(m_graphSingleData.m_dataMax),GetColor(255,255,255),m_font);
		//data�ŏ��l�̕\��
		DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
		DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(m_graphSingleData.m_dataMin),GetColor(255,255,255),m_font);

	}
	//�ǂݍ��݃f�[�^�C���^�[�t�F�[�X�̕`��
	m_pGraphDataBuilder->Draw();
}

