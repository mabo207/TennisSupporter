#define _USE_MATH_DEFINES
#include<cmath>
#include"GraphDataBuilder.h"
#include"DxLib.h"
#include"input.h"

//---------------GraphDataBuilder::IDataFactory---------------

//---------------GraphDataBuilder::PosDataFactory---------------
GraphDataBuilder::PosDataFactory::PosDataFactory(JointType i_type,double i_nVecX,double i_nVecY,double i_nVecZ):type(i_type),nVecX(i_nVecX),nVecY(i_nVecY),nVecZ(i_nVecZ){}

double GraphDataBuilder::PosDataFactory::ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const{
	//return data[type].Y;
	//�@���x�N�g����(a,b,c)�̕��ʂ̌��_��ʂ镽�ʂ̕�������ax+by+cz=0�B�_(X,Y,Z)�ƕ��ʂ̋�����|aX+bY+cZ|/��a^2+b^2+c^2�ŋ��߂���B
	return std::abs(nVecX*data[type].X+nVecY*data[type].Y+nVecZ*data[type].Z)/std::sqrt(std::pow(nVecX,2)+std::pow(nVecY,2)+std::pow(nVecZ,2));
}

double GraphDataBuilder::PosDataFactory::DataMax()const{
	return 4.5;
}

double GraphDataBuilder::PosDataFactory::DataMin()const{
	return -4.5;
}

void GraphDataBuilder::PosDataFactory::Draw(Vector2D pos)const{
	const Vector2D v=relativeInputPos.find(type)->second+pos;
	DrawCircle(v.x,v.y,circleSize,GetColor(255,128,0));
}

std::vector<JointType> GraphDataBuilder::PosDataFactory::IGetInput()const{
	return std::vector<JointType>{type};
}

//---------------GraphDataBuilder::AngleDataFactory---------------
GraphDataBuilder::AngleDataFactory::AngleDataFactory(JointType point1,JointType point2,JointType point3)
	:type{point1,point2,point3}{}

double GraphDataBuilder::AngleDataFactory::ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const{
	return data[type[1]].CalculateAngle(data[type[0]],data[type[2]])/M_PI*180;
}

double GraphDataBuilder::AngleDataFactory::DataMax()const{
	return 180.0;
}

double GraphDataBuilder::AngleDataFactory::DataMin()const{
	return 0.0;
}

void GraphDataBuilder::AngleDataFactory::Draw(Vector2D pos)const{
	Vector2D v[indexNum];
	//�h��Ԃ�
	for(size_t i=0;i<indexNum;i++){
		v[i]=relativeInputPos.find(type[i])->second+pos;
		DrawCircle(v[i].x,v[i].y,circleSize,GetColor(0,128,255));
	}
	//��������
	for(size_t i=0;i<indexNum-1;i++){
		DrawLine(v[i].x,v[i].y,v[i+1].x,v[i+1].y,GetColor(0,128,255),3);
	}

}

std::vector<JointType> GraphDataBuilder::AngleDataFactory::IGetInput()const{
	std::vector<JointType> v;
	v.reserve(indexNum);
	for(const JointType &j:type){
		v.push_back(j);
	}
	return v;
}

//---------------GraphDataBuilder---------------
const std::map<JointType,Vector2D> GraphDataBuilder::relativeInputPos={
	std::pair<JointType,Vector2D>(JointType_SpineBase,Vector2D(105,200))
	,std::pair<JointType,Vector2D>(JointType_SpineMid,Vector2D(105,160))
	,std::pair<JointType,Vector2D>(JointType_Neck,Vector2D(105,90))
	,std::pair<JointType,Vector2D>(JointType_Head,Vector2D(105,50))
	,std::pair<JointType,Vector2D>(JointType_ShoulderLeft,Vector2D(75,120))
	,std::pair<JointType,Vector2D>(JointType_ElbowLeft,Vector2D(45,120))
	,std::pair<JointType,Vector2D>(JointType_WristLeft,Vector2D(15,120))
	//,std::pair<JointType,Vector2D>(JointType_HandLeft,Vector2D(105,200))
	,std::pair<JointType,Vector2D>(JointType_ShoulderRight,Vector2D(135,120))
	,std::pair<JointType,Vector2D>(JointType_ElbowRight,Vector2D(165,120))
	,std::pair<JointType,Vector2D>(JointType_WristRight,Vector2D(195,120))
	//,std::pair<JointType,Vector2D>(JointType_HandRight,Vector2D(105,200))
	,std::pair<JointType,Vector2D>(JointType_HipLeft,Vector2D(75,200))
	,std::pair<JointType,Vector2D>(JointType_KneeLeft,Vector2D(75,240))
	,std::pair<JointType,Vector2D>(JointType_AnkleLeft,Vector2D(75,280))
	//,std::pair<JointType,Vector2D>(JointType_FootLeft,Vector2D(75,280))
	,std::pair<JointType,Vector2D>(JointType_HipRight,Vector2D(135,200))
	,std::pair<JointType,Vector2D>(JointType_KneeRight,Vector2D(135,240))
	,std::pair<JointType,Vector2D>(JointType_AnkleRight,Vector2D(135,280))
	//,std::pair<JointType,Vector2D>(JointType_FootRight,Vector2D(75,200))
	,std::pair<JointType,Vector2D>(JointType_SpineShoulder,Vector2D(105,120))
	//,std::pair<JointType,Vector2D>(JointType_HandTipLeft,Vector2D(50,200))
	//,std::pair<JointType,Vector2D>(JointType_ThumbLeft,Vector2D(50,200))
	//,std::pair<JointType,Vector2D>(JointType_HandTipRight,Vector2D(50,200))
	//,std::pair<JointType,Vector2D>(JointType_ThumbRight,Vector2D(50,200))
};
const Vector2D GraphDataBuilder::xzVectorBoxPos=Vector2D(10,320);
const Vector2D GraphDataBuilder::boxSize=Vector2D(200,200);
const Vector2D GraphDataBuilder::yVectorBoxPos=xzVectorBoxPos+Vector2D(GraphDataBuilder::boxSize.x,0);
const int GraphDataBuilder::boxCircleSize=50;
const int GraphDataBuilder::axisSize=70;
const Vector2D GraphDataBuilder::xzBoxCircleCenterPos=xzVectorBoxPos+boxSize/2;
const Vector2D GraphDataBuilder::xBoxPos=Vector2D((xzVectorBoxPos+boxSize/2).x+axisSize-squareSize/2,(xzVectorBoxPos+boxSize/2).y-squareSize/2);
const Vector2D GraphDataBuilder::zBoxPos=Vector2D((xzVectorBoxPos+boxSize/2).x-squareSize/2,(xzVectorBoxPos+boxSize/2).y-axisSize-squareSize/2);
const int GraphDataBuilder::circleSize=10;
const int GraphDataBuilder::squareSize=GraphDataBuilder::circleSize*2;

GraphDataBuilder::GraphDataBuilder(Vector2D position,int font)
	:m_position(position),m_inpFrame(0),m_font(font),m_xzAngle(0.0),m_xzOrY(false)
{
	//m_dataFactory�̏�����
	CreateFactory(std::vector<JointType>{JointType_SpineBase});
}

GraphDataBuilder::~GraphDataBuilder(){}

void GraphDataBuilder::CreateFactory(const std::vector<JointType> &input){
	//input�̌���3�������ǂ����ō�镨��ς���
	size_t size=input.size();
	if(size<1){
		m_dataFactory=std::shared_ptr<IDataFactory>(nullptr);
	} else if(size<AngleDataFactory::indexNum){
		//�@���x�N�g���̎Z�o
		double nVecX,nVecY,nVecZ;
		if(m_xzOrY){
			//xz�x�N�g��������ꍇ
			nVecX=std::cos(m_xzAngle);
			nVecY=0.0;
			nVecZ=std::sin(m_xzAngle);
		} else{
			//y�x�N�g��������ꍇ
			nVecX=0.0;
			nVecY=1.0;
			nVecZ=0.0;
		}
		//dataFactory�쐬
		m_dataFactory=std::shared_ptr<IDataFactory>(new PosDataFactory(input[0],nVecX,nVecY,nVecZ));
	} else{
		m_dataFactory=std::shared_ptr<IDataFactory>(new AngleDataFactory(input[0],input[1],input[2]));
	}
}

int GraphDataBuilder::Update(){
	int ret=0;
	const int mFrame=mouse_get(MOUSE_INPUT_LEFT);
	//�}�E�X�̓��͔���
	if(mFrame>0){
		//���݈ʒu�̊m�F
		Vector2D mousepos=GetMousePointVector2D();
		//�^�l�ԃC���^�[�t�F�[�X�ɂ��Ă̏���
		//m_input�ɒǉ�����JointType�����肷��
		JointType type=JointType_Count;//type�����̒l�̂܂܂Ȃ�m_input�ɒǉ����Ȃ�
		for(const std::pair<JointType,Vector2D> &pair:relativeInputPos){
			if((pair.second+m_position-mousepos).sqSize()<circleSize*circleSize){
				type=pair.first;
				break;
			}
		}
		//m_input�̖�����type�Ɉ�v�����A�Ȃ�����m_input�̒��g���ő�l(AngleDataFactory::indexNum)�𒴂��Ă��Ȃ��ꍇ�ǉ�����
		if(((m_input.size()==0) || (m_input.back()!=type && m_input.size()<AngleDataFactory::indexNum)) && type!=JointType_Count){
			m_input.push_back(type);
		}
		//�x�N�g���ݒ�C���^�[�t�F�[�X�ɂ��Ă̏���
		//xz��y�̂ǂ��炩��I�����Ă��邩�i����Ƃ��ύX�Ȃ����j�𔻒肵�ēK�p
		if((mousepos-(m_position+xzVectorBoxPos)).x>=0 && (mousepos-(m_position+xzVectorBoxPos)).y>=0 && (mousepos-(m_position+xzVectorBoxPos+boxSize)).x<=0 && (mousepos-(m_position+xzVectorBoxPos+boxSize)).y<=0){
			m_xzOrY=true;
			//���̏ꍇ�͊p�x�X�V���s��
			if((mousepos-(m_position+xBoxPos)).x>=0 && (mousepos-(m_position+xBoxPos)).y>=0 && (mousepos-(m_position+xBoxPos)).x<=squareSize && (mousepos-(m_position+xBoxPos)).y<=squareSize){
				//x���Ɉ�v������{�b�N�X�Ƀ}�E�X�����鎞
				m_xzAngle=0.0;
			} else if((mousepos-(m_position+zBoxPos)).x>=0 && (mousepos-(m_position+zBoxPos)).y>=0 && (mousepos-(m_position+zBoxPos)).x<=squareSize && (mousepos-(m_position+zBoxPos)).y<=squareSize){
				//z���Ɉ�v������{�b�N�X�Ƀ}�E�X�����鎞
				m_xzAngle=M_PI/2;
			} else{
				//������̏ꍇ�ł��Ȃ����͌v�Z�l�����̂܂ܓK�p����
				Vector2D v=mousepos-(m_position+xzBoxCircleCenterPos);//�~���S����}�E�X�Ɍ������x�N�g��
				v=Vector2D(v.x,-v.y);//�l�Ԃ����₷�����������ƂȂ�悤�ɕϊ�
				m_xzAngle=v.GetRadian();
			}
		} else if((mousepos-(m_position+yVectorBoxPos)).x>=0 && (mousepos-(m_position+yVectorBoxPos)).y>=0 && (mousepos-(m_position+yVectorBoxPos+boxSize)).x<=0 && (mousepos-(m_position+yVectorBoxPos+boxSize)).y<=0){
			m_xzOrY=false;
		}
	} else{
		if(m_inpFrame>0){
			//�����ꂽ�u�ԂȂ�m_dataFactory���X�V����
			if(!m_input.empty()){
				//�_�l�Ԃɓ��͂��������ꍇ�͂��̓��͂�p����
				CreateFactory(m_input);
			} else{
				//�_�l�Ԃɓ��͂��Ȃ��ꍇ�͌��݂̖_�l�ԓ��͂�]�p����
				CreateFactory(m_dataFactory->IGetInput());
			}
			ret=1;
		}
		//�}�E�X���͂�����Ă��Ȃ��Ȃ�m_input�����
		m_input.clear();
	}

	//�O�t���[���ɂ�����t���[�����̍X�V
	m_inpFrame=mFrame;

	return ret;
}

void GraphDataBuilder::Draw()const{
	//�^�l�ԃC���^�[�t�F�[�X�̕`��
	//�֐߂�S�Ē������~�ŕ`��
	for(const std::pair<JointType,Vector2D> &pair:relativeInputPos){
		const Vector2D v=m_position+pair.second;
		DrawCircle(v.x,v.y,circleSize,IBodyKinectSensor::GetJointColor(pair.first),FALSE);
	}
	//born��`��
	const std::map<JointType,Vector2D>::const_iterator ite=relativeInputPos.end();
	for(const std::pair<JointType,JointType> &pair:IBodyKinectSensor::bonePairs){
		//�S�Ă̊֐߂�`�悷��킯�ł͂Ȃ��̂ŘR�ꂪ����Bfind()��p����B
		const std::map<JointType,Vector2D>::const_iterator fit=relativeInputPos.find(pair.first),sit=relativeInputPos.find(pair.second);
		if(fit!=ite && sit!=ite){
			const Vector2D fv=m_position+fit->second,sv=m_position+sit->second;
			DrawLine(fv.x,fv.y,sv.x,sv.y,GetColor(0,255,0),1);
		}
	}
	//���ݑI������Ă���֐߂̉~��S�ēh��Ԃ�
	if(m_dataFactory.get()!=nullptr){
		m_dataFactory->Draw(m_position);
	}
	//m_input�Ƃ��đI������Ă���֐߂̉~��S�ēh��Ԃ��֐߂��Ȃ�
	std::vector<Vector2D> inpPos;
	//�h��Ԃ�
	for(size_t i=0,max=m_input.size();i<max;i++){
		inpPos.push_back(relativeInputPos.find(m_input[i])->second+m_position);
		DrawCircle(inpPos[i].x,inpPos[i].y,circleSize,GetColor(255,255,0));
	}
	//��������
	for(size_t i=0,max=m_input.size();i+1<max;i++){
		DrawLine(inpPos[i].x,inpPos[i].y,inpPos[i+1].x,inpPos[i+1].y,GetColor(255,0,0),1);
	}
	//�x�N�g���ݒ�C���^�[�t�F�[�X�̕`��
	//�g�ƍ��ږ��̐F��`
	//const unsigned int xzColor=GetColor(255,255,255),yColor=GetColor(255,255,255);
	unsigned int xzColor,yColor;
	//�I������Ă���������F��
	if(m_xzOrY){
		xzColor=GetColor(255,255,0);
		yColor=GetColor(255,255,255);
	} else{
		xzColor=GetColor(255,255,255);
		yColor=GetColor(255,255,0);
	}
	//��g
	DrawBox((m_position+xzVectorBoxPos).x,(m_position+xzVectorBoxPos).y,(m_position+xzVectorBoxPos+boxSize).x,(m_position+xzVectorBoxPos+boxSize).y
		,xzColor,FALSE);
	DrawBox((m_position+yVectorBoxPos).x,(m_position+yVectorBoxPos).y,(m_position+yVectorBoxPos+boxSize).x,(m_position+yVectorBoxPos+boxSize).y
		,yColor,FALSE);
	//���ږ�
	DrawBox((m_position+xzVectorBoxPos).x,(m_position+xzVectorBoxPos).y,(m_position+xzVectorBoxPos).x+squareSize,(m_position+xzVectorBoxPos).y+squareSize
		,xzColor,TRUE);
	DrawStringCenterBaseToHandle((m_position+xzVectorBoxPos).x+squareSize/2,(m_position+xzVectorBoxPos).y+squareSize/2,"xz"
		,GetInvertedColor(xzColor),m_font,true);
	DrawBox((m_position+yVectorBoxPos).x,(m_position+yVectorBoxPos).y,(m_position+yVectorBoxPos).x+squareSize,(m_position+yVectorBoxPos).y+squareSize
		,yColor,TRUE);
	DrawStringCenterBaseToHandle((m_position+yVectorBoxPos).x+squareSize/2,(m_position+yVectorBoxPos).y+squareSize/2,"y"
		,GetInvertedColor(yColor),m_font,true);
	//�~��
	const unsigned int standardColor=GetColor(0,255,0);
	DrawCircle((m_position+xzBoxCircleCenterPos).x,(m_position+xzBoxCircleCenterPos).y,boxCircleSize
		,standardColor,FALSE);
	//��
	DrawLine((m_position+xzBoxCircleCenterPos).x,(m_position+xzBoxCircleCenterPos).y,(m_position+xBoxPos).x+squareSize/2,(m_position+xBoxPos).y+squareSize/2
		,standardColor);//x��
	DrawLine((m_position+xzBoxCircleCenterPos).x,(m_position+xzBoxCircleCenterPos).y,(m_position+zBoxPos).x+squareSize/2,(m_position+zBoxPos).y+squareSize/2
		,standardColor);//z��
	//����
	DrawBox((m_position+xBoxPos).x,(m_position+xBoxPos).y,(m_position+xBoxPos).x+squareSize,(m_position+xBoxPos).y+squareSize
		,standardColor,TRUE);
	DrawStringCenterBaseToHandle((m_position+xBoxPos).x+squareSize/2,(m_position+xBoxPos).y+squareSize/2,"x"
		,GetInvertedColor(standardColor),m_font,true);
	DrawBox((m_position+zBoxPos).x,(m_position+zBoxPos).y,(m_position+zBoxPos).x+squareSize,(m_position+zBoxPos).y+squareSize
		,standardColor,TRUE);
	DrawStringCenterBaseToHandle((m_position+zBoxPos).x+squareSize/2,(m_position+zBoxPos).y+squareSize/2,"z"
		,GetInvertedColor(standardColor),m_font,true);
	//�p�x�~
//	const double angle=50.0/180*M_PI;
	const double angle=m_xzAngle;
	DrawCircle((m_position+xzBoxCircleCenterPos).x+(int)(boxCircleSize*std::cos(angle)),(m_position+xzBoxCircleCenterPos).y-(int)(boxCircleSize*std::sin(angle)),circleSize
		,GetInvertedColor(standardColor),TRUE);
	//���݂̕�������
	DrawLine((m_position+xzBoxCircleCenterPos).x,(m_position+xzBoxCircleCenterPos).y,(m_position+xzBoxCircleCenterPos).x+(int)(boxCircleSize*std::cos(angle)),(m_position+xzBoxCircleCenterPos).y-(int)(boxCircleSize*std::sin(angle))
		,GetInvertedColor(standardColor));
}

double GraphDataBuilder::CalData(const std::vector<IBodyKinectSensor::JointPosition> &playData)const{
	return m_dataFactory->ICalData(playData);
}

double GraphDataBuilder::DataMax()const{
	return m_dataFactory->DataMax();
}

double GraphDataBuilder::DataMin()const{
	return m_dataFactory->DataMin();
}
