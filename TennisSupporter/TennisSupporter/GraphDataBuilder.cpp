#include"GraphDataBuilder.h"
#include"DxLib.h"
#include"input.h"

//---------------GraphDataBuilder::IDataFactory---------------
std::shared_ptr<GraphDataBuilder::IDataFactory> GraphDataBuilder::IDataFactory::CreateFactory(const std::vector<JointType> &input){
	//input�̌���3�������ǂ����ō�镨��ς���
	size_t size=input.size();
	if(size<1){
		return std::shared_ptr<IDataFactory>(nullptr);
	} else if(size<AngleDataFactory::indexNum){
		return std::shared_ptr<IDataFactory>(new PosDataFactory(input[0]));
	} else{
		return std::shared_ptr<IDataFactory>(new AngleDataFactory(input[0],input[1],input[2]));
	}
}

//---------------GraphDataBuilder::PosDataFactory---------------
GraphDataBuilder::PosDataFactory::PosDataFactory(JointType i_type):type(i_type){}

double GraphDataBuilder::PosDataFactory::ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const{
	return data[type].Z;
}

void GraphDataBuilder::PosDataFactory::Draw()const{
	const Vector2D v=relativeInputPos.find(type)->second;
	DrawCircle(v.x,v.y,circleSize,GetColor(255,128,0));
}

//---------------GraphDataBuilder::AngleDataFactory---------------
GraphDataBuilder::AngleDataFactory::AngleDataFactory(JointType point1,JointType point2,JointType point3)
	:type{point1,point2,point3}{}

double GraphDataBuilder::AngleDataFactory::ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const{
	return data[type[1]].CalculateAngle(data[type[0]],data[type[2]]);
}

void GraphDataBuilder::AngleDataFactory::Draw()const{
	Vector2D v[indexNum];
	//�h��Ԃ�
	for(size_t i=0;i<indexNum;i++){
		v[i]=relativeInputPos.find(type[i])->second;
		DrawCircle(v[i].x,v[i].y,circleSize,GetColor(0,128,255));
	}
	//��������
	for(size_t i=0;i<indexNum-1;i++){
		DrawLine(v[i].x,v[i].y,v[i+1].x,v[i+1].y,GetColor(0,128,255),1);
	}

}

//---------------GraphDataBuilder---------------
const std::map<JointType,Vector2D> GraphDataBuilder::relativeInputPos={
	std::pair<JointType,Vector2D>(JointType_SpineBase,Vector2D(75,150))
	,std::pair<JointType,Vector2D>(JointType_SpineMid,Vector2D(75,120))
	,std::pair<JointType,Vector2D>(JointType_Neck,Vector2D(75,70))
	,std::pair<JointType,Vector2D>(JointType_Head,Vector2D(75,40))
	,std::pair<JointType,Vector2D>(JointType_ShoulderLeft,Vector2D(55,90))
	,std::pair<JointType,Vector2D>(JointType_ElbowLeft,Vector2D(35,90))
	,std::pair<JointType,Vector2D>(JointType_WristLeft,Vector2D(15,90))
	//,std::pair<JointType,Vector2D>(JointType_HandLeft,Vector2D(75,150))
	,std::pair<JointType,Vector2D>(JointType_ShoulderRight,Vector2D(95,90))
	,std::pair<JointType,Vector2D>(JointType_ElbowRight,Vector2D(115,90))
	,std::pair<JointType,Vector2D>(JointType_WristRight,Vector2D(135,90))
	//,std::pair<JointType,Vector2D>(JointType_HandRight,Vector2D(75,150))
	,std::pair<JointType,Vector2D>(JointType_HipLeft,Vector2D(55,150))
	,std::pair<JointType,Vector2D>(JointType_KneeLeft,Vector2D(55,180))
	,std::pair<JointType,Vector2D>(JointType_AnkleLeft,Vector2D(55,210))
	//,std::pair<JointType,Vector2D>(JointType_FootLeft,Vector2D(55,210))
	,std::pair<JointType,Vector2D>(JointType_HipRight,Vector2D(95,150))
	,std::pair<JointType,Vector2D>(JointType_KneeRight,Vector2D(95,180))
	,std::pair<JointType,Vector2D>(JointType_AnkleRight,Vector2D(95,210))
	//,std::pair<JointType,Vector2D>(JointType_FootRight,Vector2D(55,150))
	,std::pair<JointType,Vector2D>(JointType_SpineShoulder,Vector2D(75,90))
	//,std::pair<JointType,Vector2D>(JointType_HandTipLeft,Vector2D(50,150))
	//,std::pair<JointType,Vector2D>(JointType_ThumbLeft,Vector2D(50,150))
	//,std::pair<JointType,Vector2D>(JointType_HandTipRight,Vector2D(50,150))
	//,std::pair<JointType,Vector2D>(JointType_ThumbRight,Vector2D(50,150))
};
const int GraphDataBuilder::circleSize=5;

GraphDataBuilder::GraphDataBuilder(Vector2D position)
	:m_position(position),m_dataFactory(IDataFactory::CreateFactory(std::vector<JointType>(JointType_SpineBase))),m_inpFrame(0){}

GraphDataBuilder::~GraphDataBuilder(){}

int GraphDataBuilder::Update(){
	const int mFrame=mouse_get(MOUSE_INPUT_LEFT);
	//�}�E�X�̓��͔���
	if(mFrame>0){
		//���݈ʒu�̊m�F
		Vector2D mousepos=GetMousePointVector2D();
		//m_input�ɒǉ�����JointType�����肷��
		JointType type=JointType_Count;//type�����̒l�̂܂܂Ȃ�m_input�ɒǉ����Ȃ�
		for(const std::pair<JointType,Vector2D> &pair:relativeInputPos){
			if((pair.second+m_position-mousepos).sqSize()<circleSize*circleSize){
				type=pair.first;
				break;
			}
		}
		//m_input�̖�����type�Ɉ�v�����A�Ȃ�����m_input�̒��g���ő�l(AngleDataFactory::indexNum)�𒴂��Ă��Ȃ��ꍇ�ǉ�����
		if(m_input.back()!=type && m_input.size()<AngleDataFactory::indexNum){
			m_input.push_back(type);
		}
	} else{
		if(m_inpFrame>0){
			//�����ꂽ�u�ԂȂ�m_dataFactory���X�V����
			m_dataFactory=IDataFactory::CreateFactory(m_input);
		}
		//�}�E�X���͂�����Ă��Ȃ��Ȃ�m_input�����
		m_input.clear();
	}

	//�O�t���[���ɂ�����t���[�����̍X�V
	m_inpFrame=mFrame;

	return 0;
}

void GraphDataBuilder::Draw()const{
	//�֐߂�S�Ē������~�ŕ`��
	for(const std::pair<JointType,Vector2D> &pair:relativeInputPos){
		DrawCircle(pair.second.x,pair.second.y,circleSize,GetColor(0,255,0),FALSE);
	}
	//born��`��
	const std::map<JointType,Vector2D>::const_iterator ite=relativeInputPos.end();
	for(const std::pair<JointType,JointType> &pair:BodyKinectSensor::bonePairs){
		//�S�Ă̊֐߂�`�悷��킯�ł͂Ȃ��̂ŘR�ꂪ����Bfind()��p����B
		const std::map<JointType,Vector2D>::const_iterator fit=relativeInputPos.find(pair.first),sit=relativeInputPos.find(pair.second);
		if(fit!=ite && sit!=ite){
			DrawLine(fit->second.x,fit->second.y,sit->second.x,sit->second.y,GetColor(0,255,0),1);
		}
	}
	//���ݑI������Ă���֐߂̉~��S�ēh��Ԃ�
	if(m_dataFactory.get()!=nullptr){
		m_dataFactory->Draw();
	}
}
