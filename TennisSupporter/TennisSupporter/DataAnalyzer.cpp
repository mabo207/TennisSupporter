#define _USE_MATH_DEFINES

#include<cmath>
#include"DataAnalyzer.h"
#include"DxLib.h"
#include"input.h"

//-----------------DataAnalyzer-----------------
const Vector2D DataAnalyzer::graphPos=Vector2D(100,60);
const Vector2D DataAnalyzer::graphSize=Vector2D(DataAnalyzer::writeCountMax,360);
const std::string DataAnalyzer::sectionStr="##################";

DataAnalyzer::DataAnalyzer(int font,const char *filename)
	:IBodySimulateScene(MODE::ANALYZER,font),m_playFrame(0.0),m_playRate(1.0),
	m_playFlag(true),m_graphUnity(false),m_extend(1.0),m_dataAverage(0.0),m_widthUnity(false),m_dataSizeMax(1)
{
	//GraphDataBuilder�̋N��
	m_pGraphDataBuilder=std::shared_ptr<GraphDataBuilder>(new GraphDataBuilder(Vector2D(kinectSize.x*2+80,0),m_font));
	//�f�[�^�̓ǂݎ��
	ReadFile(filename);
}

DataAnalyzer::DataAnalyzer(int font,const char *filename,std::shared_ptr<GraphDataBuilder> pCopyDataBuilder)
	:IBodySimulateScene(MODE::ANALYZER,font),m_playFrame(0.0),m_playRate(1.0),
	m_playFlag(true),m_graphUnity(false),m_extend(1.0),m_dataAverage(0.0),m_widthUnity(false),m_dataSizeMax(1),m_pGraphDataBuilder(pCopyDataBuilder)
{
	//�f�[�^�̓ǂݎ��
	ReadFile(filename);
}

DataAnalyzer::~DataAnalyzer(){}

bool DataAnalyzer::ReadFile(const char *filename){
	std::ifstream readFile(filename);
	//�f�[�^��S�ēǂݍ��ނ��A�ʂ������̂ŏ�肭reserve���Ȃ���ǂݍ���
	GraphSingleData graphSingleData;
	
	graphSingleData.m_playData.clear();
	graphSingleData.m_playData.reserve(writeCountMax);//�ő�L�^�t���[�����ɂ��reserve

	std::vector<std::vector<IBodyKinectSensor::JointPosition>> frameData;
	frameData.reserve(IBodyKinectSensor::bodyNum);

	std::vector<IBodyKinectSensor::JointPosition> bodyData;
	bodyData.reserve(JointType_Count);

	size_t bodyindex=0,jointindex=0;
	std::string parts="";
	parts.reserve(50);
	bool inBracketsFlag=false;//����"(~~)"�̒���ǂݎ���Ă��邩�ǂ���
	bool sectionFlag=false;//����sectionStr��ǂݎ���Ă��邩�ǂ���
	char ch='a';//�����l�̓e�L�g�[
	try{
		while(true){
			ch=readFile.get();
			if(ch==EOF){
				//�t�@�C���̖������B��
				//���݂�graphSingleData��m_graphData�Ɋi�[
				if(!graphSingleData.m_playData.empty()){
					graphSingleData.m_pBodyVirtualKinectSensor=std::shared_ptr<BodyVirtualKinectSensor>(new BodyVirtualKinectSensor());
					m_graphData.push_back(graphSingleData);
				}
				graphSingleData.m_playData.clear();
				break;
			} else if(ch=='\n'){
				if(!sectionFlag){
					//1�t���[���ǂݏI���̎�
					graphSingleData.m_playData.push_back(frameData);
				} else{
					//1�l���ǂݏI���̎�(sectionStr�̍s��ǂݏI���)
					if(!graphSingleData.m_playData.empty()){
						graphSingleData.m_pBodyVirtualKinectSensor=std::shared_ptr<BodyVirtualKinectSensor>(new BodyVirtualKinectSensor());
						m_graphData.push_back(graphSingleData);
					}
					graphSingleData.m_playData.clear();
					sectionFlag=false;
				}
				//�ȉ����ʏ���
				frameData.clear();
				bodyData.clear();
				bodyindex=0;
				jointindex=0;
				parts="";
				inBracketsFlag=false;
			} else if(ch=='#'){
				//#��1�����ǂ񂾂�A���̍s�̂���ȍ~�̕����͓ǂݎ��Ȃ�
				sectionFlag=true;
			} else if(!sectionFlag){
				//sectionStr��ǂ�ł��Ȃ�
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

	//�Đ��f�[�^�̏�����
	m_playFlag=true;
	
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

	return true;
}

void DataAnalyzer::DataBuild(){
	for(GraphSingleData &gdata:m_graphData){
		gdata.DataBuild(m_pGraphDataBuilder);
	}
	//m_dataAverage��m_dataSizeMax�����߂�
	size_t size=0,memo=0;
	m_dataAverage=0;
	m_dataSizeMax=1;
	for(const GraphSingleData &gdata:m_graphData){
		for(const double &data:gdata.m_data){
			size++;
			m_dataAverage+=data;
		}
		m_dataSizeMax=(size_t)std::fmax(m_dataSizeMax,size-memo);
		memo=size;
	}
	m_dataAverage/=std::fmax(size,1);
	//�t���[�������������A�C���[�W���X�V
	m_playFrame=0.0;
	UpdateImage();
}

int DataAnalyzer::CalReadIndex()const{
	return (int)(m_playFrame*captureFps/drawFps);
}

double DataAnalyzer::CalPlayFrame(int index)const{
	return ((double)index)*drawFps/captureFps;
}

bool DataAnalyzer::JudgeMouseInGraph()const{
	Vector2D relativeMouse=GetMousePointVector2D()-graphPos;
	return (relativeMouse.x>=0 && relativeMouse.x<=graphSize.x && relativeMouse.y>=0 && relativeMouse.y<=graphSize.y);
}

void DataAnalyzer::UpdateImage(){
	UpdateImage(CalReadIndex());
}

void DataAnalyzer::UpdateImage(int index){
	for(GraphSingleData &gdata:m_graphData){
		gdata.UpdateVirtualSensor(index);
	}
}

void DataAnalyzer::OutputGraphData()const{
	//�t�@�C�����̍쐬
	std::string fname=m_playDataName+"_"+m_pGraphDataBuilder->GetFactoryType()+".csv";
	//�����o��
	OutputGraphData(fname.c_str());
}

void DataAnalyzer::OutputGraphData(const char *filename)const{
	//�����o���t�@�C�����I�[�v��
	if(JudgeFileExist(filename)){
		//����t�@�C���������ɑ��݂��Ă�����A���ɉ������Ȃ�

	} else{
		//���݂��Ă��Ȃ��Ȃ�
		std::ofstream writeFile(filename,std::ios_base::trunc);
		if(!writeFile){
			//�t�@�C�����J���̂����s������A���ɉ������Ȃ�

		} else{
			//����ɏ������݂��ł���
			for(const GraphSingleData &gdata:m_graphData){
				//1�s��1�̃f�[�^�n�񂸂����o��
				gdata.WriteGraphSingleData(writeFile);
			}
		}
		writeFile.close();
	}
}

void DataAnalyzer::InputToOutputFolder()const{
	//�ǂݎ�茳�A�����o����t�H���_�������K��
	const std::string inpDir="Input/",outDir="Output/";
	const std::string fName[]={
		"A-suburi",
		"A-serve",
		"B-suburi",
		"B-serve",
		"C-suburi",
		"C-serve",
		"D-suburi",
		"D-serve",
		"E-suburi",
		"E-serve",
		"F-suburi",
		"F-serve",
		"G-suburi",
		"G-serve"
	};
	//�����o��
	for(const std::string &fname:fName){
		DataAnalyzer d(-1,(inpDir+fname+".txt").c_str(),m_pGraphDataBuilder);
		d.OutputGraphData((outDir+fname+"_"+m_pGraphDataBuilder->GetFactoryType()+".csv").c_str());
	}
}

int DataAnalyzer::Update(){
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
		if(m_playFlag && a<writeCountMax){
			//�܂��f�[�^�I�[�܂ł����Ă��炸�A���Đ����[�h�ɂȂ��Ă��鎞�A�t���[�������X�V
			m_playFrame+=m_playRate;
		}
		int b=CalReadIndex();//���̒l��a�Ɉ�v���Ă��鎞�͓ǂݍ��݂͍s�킸�A�O�t���[���Ɠ����摜��`�悷��
		if(a!=b){
			UpdateImage();
		}
	} else{
		//�E�N���b�N��������Ă���ꍇ�́A�}�E�X�̈ʒu�ɏ]���ăC���[�W���X�V
		int index=GetMousePointVector2D().x-graphPos.x;
		UpdateImage(index);
	}
	//���̓C���^�[�t�F�[�X
	if(m_pGraphDataBuilder->Update()==1){
		//m_dataFactory���X�V��������DataBuild()���g�p����
		DataBuild();
	}
	//�O���t�̎���̐؂�ւ�
	if(keyboard_get(KEY_INPUT_U)==1){
		m_graphUnity=!m_graphUnity;
	}
	//�O���t�̉����ψꉻ�̐؂�ւ�
	if(keyboard_get(KEY_INPUT_I)==1){
		m_widthUnity=!m_widthUnity;
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
	//�O���t�g�嗦����
	if(keyboard_get(KEY_INPUT_Q)>0){
		//����������
		m_extend=std::fmax(0.1,m_extend-0.05);
	} else if(keyboard_get(KEY_INPUT_W)>0){
		//1.0�{�ɖ߂�
		m_extend=1.0;
	} else if(keyboard_get(KEY_INPUT_E)>0){
		//�傫������
		const int frame=keyboard_get(KEY_INPUT_E);
		double speed;
		if(frame<60){
			speed=0.05;
		} else{
			speed=0.002*frame;
		}
		m_extend=m_extend+speed;
	}
	//csv�o��
	if(keyboard_get(KEY_INPUT_S)==10){
		//���ݕ\�����Ă���O���t��csv�o��
		OutputGraphData();
	} else if(keyboard_get(KEY_INPUT_A)==20){
		//Input�ɂ���f�[�^�̃O���t����Output�ɏo��
		InputToOutputFolder();
	}

	//��ʑJ��
	if(keyboard_get(KEY_INPUT_BACK)==1){
		//Back�L�[���͂ŋL�^���[�h�ɖ߂�
		return 1;
	}
	return 0;
}

void DataAnalyzer::Draw()const{
	//�`��ʒu
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	
	//�^�l�Ԃ͕`�悵�Ȃ�
	//m_graphSingleData.m_pBodyVirtualKinectSensor->Draw(nullptr,Vector2D(-3000,-3000),kinectSize,xyPos,kinectSize,zyPos,kinectSize);//(depth�摜�ɑ΂���body�{�[���͕`�悵�Ȃ�)
	
	//�O���t�`��
	//����(1px�ɂ�1�t���[���Ƃ���������ł���)
	//�ő�ŏ��l�̐ݒ�
	double dataTop=m_pGraphDataBuilder->DataMax(),dataBottom=m_pGraphDataBuilder->DataMin();
	//�{������
	double dataCenter=(dataTop+dataBottom)/2;
	if(m_graphUnity){
		dataTop=dataCenter+(dataTop-dataCenter)/m_extend;
		dataBottom=dataCenter+(dataBottom-dataCenter)/m_extend;
	} else{
		dataTop=m_dataAverage+(dataTop-dataCenter)/m_extend;
		dataBottom=m_dataAverage+(dataBottom-dataCenter)/m_extend;
	}
	//�܂���̕`��
	for(size_t j=0,size=m_graphData.size();j<size;j++){
		//�O���t�̐F�̐ݒ�
		unsigned int color=GetColor(64*(j%4)+63,64*((j*3%16)/4)+63,64*((j*5%64)/16)+63);
		//�F�ꗗ�ɕ`��
		DrawBox(graphPos.x+graphSize.x+20,330+j*20,graphPos.x+graphSize.x+20+60,330+j*20+10,color,TRUE);
		//1�t���[���ɑ΂���s�N�Z�����̌v�Z
		double frameRateToPixel;
		if(m_widthUnity){
			frameRateToPixel=((double)writeCountMax)/m_graphData[j].m_data.size();
		} else{
			frameRateToPixel=((double)writeCountMax)/m_dataSizeMax;
		}
		//�܂���̕`��
		for(size_t i=0,datanum=m_graphData[j].m_data.size();i<datanum;i++){
			DrawCircle(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphData[j].m_data[i])),1,color,TRUE);
		}
		for(size_t i=0,datanum=m_graphData[j].m_data.size();i+1<datanum;i++){
			DrawLine(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphData[j].m_data[i])),graphPos.x+(int)((i+1)*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphData[j].m_data[i+1])),color,1);
		}
	}
	//data�ő�l�̕\��
	DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
	DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(dataTop),GetColor(255,255,255),m_font);
	//data�ŏ��l�̕\��
	DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
	DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(dataBottom),GetColor(255,255,255),m_font);

	//�ǂݍ��݃f�[�^�C���^�[�t�F�[�X�̕`��
	m_pGraphDataBuilder->Draw();
	//��������̕`��(�������̓e�L�g�[�A�ǂ����݂͂����Ȃ�)
	DrawStringNewLineToHandle(zyPos.x,zyPos.y,0,0,10000,10000,GetColor(255,255,255),m_font,GetFontSizeToHandle(m_font),
		"L click (on body) : set kind of graph\n"
		"U : convert height mode ( unity / normal )\n"
		"I : convert width mode ( unity / normal )\n"
		"Q : reduce extend rate\n"
		"W : reset extend rate\n"
		"E : add extend rate\n"
		"S : output graph\n"
		"A : output files of Input directory to OutputDirectory\n"
		"back : return photographer mode\n"
	);

	//�����o���f�[�^�t�@�C����
	DrawStringToHandle(200,800,(m_playDataName+"_"+m_pGraphDataBuilder->GetFactoryType()+".csv").c_str(),GetColor(255,255,255),m_font);
}


