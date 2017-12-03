#include"BodyKinectSensor.h"
#include"DxLib.h"

//-------------------BodyKinectSensor-------------------
BodyKinectSensor::BodyKinectSensor(IKinectSensor *pSensor){
	//source
	IBodyFrameSource *pBodySource;
	ErrorCheck(pSensor->get_BodyFrameSource(&pBodySource),"You can't get source.");
	//reader
	ErrorCheck(pBodySource->OpenReader(&m_pBodyReader),"You can't open reader.");
}

BodyKinectSensor::~BodyKinectSensor(){}

int BodyKinectSensor::Update(){
	//�ꎞ�ϐ��̗p��
	IBodyFrame *pBodyFrame=nullptr;
	IBody *pBodies[bodyNum];
	for(size_t i=0;i<bodyNum;i++){
		pBodies[i]=nullptr;
	}
	//�X�V���
	try{
		ErrorCheck(m_pBodyReader->AcquireLatestFrame(&pBodyFrame),"aquaire failed\n");//���߃t���[����body�f�[�^�̎擾
		ErrorCheck(pBodyFrame->GetAndRefreshBodyData(6,pBodies),"access failed\n");//body�f�[�^��pBodies�Ɋi�[
		pBodyFrame->Release();//����ȍ~pBodyFrame�͎g��Ȃ�
		printfDx("success\n");
		for(int j=0;j<bodyNum;j++){
			//�֐߂̎����W�ʒu��jointPositions�Ɋi�[
			Joint joints[JointType::JointType_Count];
			pBodies[j]->GetJoints(JointType::JointType_Count,joints);//�֐߈ʒu�̎����W�̎擾
			//jointPositions�Ɋ֐߂̎����W���i�[
			for(int i=0;i<JointType_Count;i++){
				m_jointPositions[j][i]=JointPosition(joints[i].Position);
			}
		}
	} catch(const std::exception &e){
		printfDx(e.what());
	}

	return 0;//�Ԃ��l�̓e�L�g�[
}

int BodyKinectSensor::Update(std::ifstream &readFile){
	size_t bodyindex=0,jointindex=0;
	std::string parts="";
	parts.reserve(50);
	bool inBracketsFlag=false;//����"(~~)"�̒���ǂݎ���Ă��邩�ǂ���
	char ch='a';//�����l�̓e�L�g�[
	while(true){
		ch=readFile.get();
		if(ch=='\n' || ch==EOF){
			//1�s�ǂݏI��邩�A�t�@�C���̖������B��
			break;
		} else{
			if(inBracketsFlag){
				//()���ǂݎ�莞(=')'��T���Ă���)
				parts.push_back(ch);
				if(ch==')'){
					inBracketsFlag=!inBracketsFlag;
					//jointPositions�Ɋi�[
					if(bodyindex<bodyNum && jointindex<JointType_Count){
						m_jointPositions[bodyindex][jointindex]=JointPosition(parts);
						//index�n�̍X�V
						jointindex++;
						if(jointindex>=JointType_Count){
							jointindex=0;
							bodyindex++;
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

	//�l��Ԃ��Bch��EOF���ǂ�����`����
	if(ch==EOF){
		return 1;//�f�[�^�ǂݍ��݂��I�����鎖��`����
	} else{
		return 0;
	}
}
