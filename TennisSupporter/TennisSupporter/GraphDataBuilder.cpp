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

std::string GraphDataBuilder::PosDataFactory::IGetFactoryType()const{
	std::string str="pos_"+IBodyKinectSensor::jointName.find(type)->second;
	const double pal[3]={nVecX,nVecY,nVecZ};
	const std::string index[3]={"_X","_Y","_Z"};
	for(size_t j=0;j<3;j++){
		std::string s=std::to_string(pal[j]);
		//�����_��'-'�ɒu���B1�����ϊ��Ȃ̂Ń��N������B
		for(size_t i=0,size=s.size();i<size;i++){
			if(s[i]=='.'){
				s[i]='-';
			}
		}
		//�����p����index�Ƌ��ɏo��
		str+=(index[j]+s);
	}
	return str;
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

std::string GraphDataBuilder::AngleDataFactory::IGetFactoryType()const{
	std::string str="angle";
	for(const JointType &t:type){
		str+=("_"+IBodyKinectSensor::jointName.find(t)->second);
	}
	return str;
}

//---------------GraphDataBuilder::LengthDataFactory---------------
GraphDataBuilder::LengthDataFactory::LengthDataFactory(JointType point1,JointType point2,bool i_xFlag,bool i_yFlag,bool i_zFlag)
	:xFlag(i_xFlag),yFlag(i_yFlag),zFlag(i_zFlag),type{point1,point2}{}

double GraphDataBuilder::LengthDataFactory::ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const{
	const double dx=xFlag ? (double)(data[type[1]].X-data[type[0]].X) : 0.0;
	const double dy=yFlag ? (double)(data[type[1]].Y-data[type[0]].Y) : 0.0;
	const double dz=zFlag ? (double)(data[type[1]].Z-data[type[0]].Z) : 0.0;
	return std::sqrt(dx*dx+dy*dy+dz*dz);
}

double GraphDataBuilder::LengthDataFactory::DataMax()const{
	return 13.5;
}

double GraphDataBuilder::LengthDataFactory::DataMin()const{
	return 0.0;
}

void GraphDataBuilder::LengthDataFactory::Draw(Vector2D pos)const{
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

std::vector<JointType> GraphDataBuilder::LengthDataFactory::IGetInput()const{
	std::vector<JointType> v;
	v.reserve(indexNum);
	for(const JointType &j:type){
		v.push_back(j);
	}
	return v;
}

std::string GraphDataBuilder::LengthDataFactory::IGetFactoryType()const{
	std::string str="length";
	str+=(xFlag?"_x":"");
	str+=(yFlag?"_y":"");
	str+=(zFlag?"_z":"");
	for(const JointType &t:type){
		str+=("_"+IBodyKinectSensor::jointName.find(t)->second);
	}
	return str;
}

//---------------GraphDataBuilder::SlopeDataFactory---------------
GraphDataBuilder::SlopeDataFactory::SlopeDataFactory(JointType point1,JointType point2,ElementType i_dividedEle,ElementType i_divideEle)
	:divideEle(i_divideEle),dividedEle(i_dividedEle),type{point1,point2}{}

double GraphDataBuilder::SlopeDataFactory::CalculateDiff(ElementType ele,const std::vector<IBodyKinectSensor::JointPosition> &data)const{
	switch(ele){
	case(elX):
		return (double)(data[type[1]].X-data[type[0]].X);
		break;
	case(elY):
		return (double)(data[type[1]].Y-data[type[0]].Y);
		break;
	case(elZ):
		return (double)(data[type[1]].Z-data[type[0]].Z);
		break;
	}
	return 0.0;
}

std::string GraphDataBuilder::SlopeDataFactory::ElementToStr(ElementType ele)const{
	switch(ele){
	case(elX):
		return "x";
		break;
	case(elY):
		return "y";
		break;
	case(elZ):
		return "z";
		break;
	}
	return "";

}

double GraphDataBuilder::SlopeDataFactory::ICalData(const std::vector<IBodyKinectSensor::JointPosition> &data)const{
	double divided=CalculateDiff(dividedEle,data),divide=CalculateDiff(divideEle,data);
	if(divide!=0.0){
		//0���Z�łȂ���
		return divided/divide;
	} else{
		//0���Z��dataMax,0.0,dataMin��Ԃ�
		if(divided>0.0){
			return DataMax();
		} else if(divided==0.0){
			return 0.0;
		} else{
			return DataMin();
		}
	}
}

double GraphDataBuilder::SlopeDataFactory::DataMax()const{
	return 100.0;
}

double GraphDataBuilder::SlopeDataFactory::DataMin()const{
	return -100.0;
}

void GraphDataBuilder::SlopeDataFactory::Draw(Vector2D pos)const{
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

std::vector<JointType> GraphDataBuilder::SlopeDataFactory::IGetInput()const{
	std::vector<JointType> v;
	v.reserve(indexNum);
	for(const JointType &j:type){
		v.push_back(j);
	}
	return v;
}

std::string GraphDataBuilder::SlopeDataFactory::IGetFactoryType()const{
	std::string str="slope_"+ElementToStr(dividedEle)+"_divided_"+ElementToStr(divideEle);
	for(const JointType &t:type){
		str+=("_"+IBodyKinectSensor::jointName.find(t)->second);
	}
	return str;
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
const Vector2D GraphDataBuilder::lengthBoxPos=xzVectorBoxPos+Vector2D(0,boxCircleSize+boxSize.y);
const Vector2D GraphDataBuilder::twoPointBoxSize=boxSize/2;
const Vector2D GraphDataBuilder::tanBoxPos=lengthBoxPos+Vector2D(0,twoPointBoxSize.y);
const int GraphDataBuilder::circleSize=10;
const int GraphDataBuilder::squareSize=GraphDataBuilder::circleSize*2;

const std::string GraphDataBuilder::TanCalKind::str[END]={"z/x"};
const std::string GraphDataBuilder::TwoPointCalKind::str[END]={"length","tan"};

GraphDataBuilder::GraphDataBuilder(Vector2D position,int font)
	:m_position(position),m_inpFrame(0),m_font(font),m_xzAngle(0.0),m_xzOrY(false),m_updateDataFactoryFlag(false),
	m_twoPointCalKind(TwoPointCalKind::LENGTH),m_xLengthFlag(true),m_yLengthFlag(false),m_zLengthFlag(true),m_tanCalKind(TanCalKind::ZDIVX)
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
	} else if(size<SlopeDataFactory::indexNum){
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
	} else if(size<AngleDataFactory::indexNum){
		//2�_�I���C���^�[�t�F�[�X�̓���
		switch(m_twoPointCalKind){
		case(TwoPointCalKind::LENGTH):
			m_dataFactory=std::shared_ptr<IDataFactory>(new LengthDataFactory(input[0],input[1],m_xLengthFlag,m_yLengthFlag,m_zLengthFlag));
			break;
		case(TwoPointCalKind::TAN):
			switch(m_tanCalKind){
			case(TanCalKind::ZDIVX):
				m_dataFactory=std::shared_ptr<IDataFactory>(new SlopeDataFactory(input[0],input[1],SlopeDataFactory::elZ,SlopeDataFactory::elX));
				break;
			}
			break;
		}
	} else{
		//�p�x�v�Z��dataFactory�ō쐬
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
			//�v�f�̒ǉ�
			m_input.push_back(type);
			//�O���t�̍X�V���m�肷��̂Ńt���O�𗧂Ă�
			m_updateDataFactoryFlag=true;
		}
		//�x�N�g���ݒ�C���^�[�t�F�[�X�ɂ��Ă̏���
		//xz��y�̂ǂ��炩��I�����Ă��邩�i����Ƃ��ύX�Ȃ����j�𔻒肵�ēK�p
		if((mousepos-(m_position+xzVectorBoxPos)).x>=0 && (mousepos-(m_position+xzVectorBoxPos)).y>=0 && (mousepos-(m_position+xzVectorBoxPos+boxSize)).x<=0 && (mousepos-(m_position+xzVectorBoxPos+boxSize)).y<=0){
			m_xzOrY=true;
			//�O���t�̍X�V���m�肷��̂Ńt���O�𗧂Ă�
			m_updateDataFactoryFlag=true;
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
			//�O���t�̍X�V���m�肷��̂Ńt���O�𗧂Ă�
			m_updateDataFactoryFlag=true;
		}
		//2�_�I�������C���^�[�t�F�[�X�ɂ��Ă̏���
		if(mFrame==1){
			//���N���b�N�̏u�Ԃ̂ݔ�����s��
			//�����v�Z�C���^�[�t�F�[�X
			Vector2D lv=mousepos-(m_position+lengthBoxPos);//length vector
			int lc=lv.x/twoPointBoxSize.x;//length count
			if(lv.y>=0 && lv.y<=twoPointBoxSize.y && lc>=0 && lc<4){
				//�l�p���ɂ���Όv�Z���@�̐؂�ւ�
				m_twoPointCalKind=TwoPointCalKind::LENGTH;
				//�v�Z�Ώۂ�ONOFF�؂�ւ�
				switch(lc){
				case(1):
					m_xLengthFlag=!m_xLengthFlag;
					break;
				case(2):
					m_yLengthFlag=!m_yLengthFlag;
					break;
				case(3):
					m_zLengthFlag=!m_zLengthFlag;
					break;
				}
				//�O���t�̍X�V���m�肷��̂Ńt���O�𗧂Ă�
				m_updateDataFactoryFlag=true;
			}
			//tan�v�Z�C���^�[�t�F�[�X
			Vector2D tv=mousepos-(m_position+tanBoxPos);//tan vector
			int tc=tv.x/twoPointBoxSize.x;//tan count
			if(tv.y>=0 && tv.y<=twoPointBoxSize.y && tc>=0 && tc<4){
				//�l�p���ɂ���Όv�Z���@�̐؂�ւ�
				m_twoPointCalKind=TwoPointCalKind::TAN;
				//�v�Z�Ώۂ�ONOFF�؂�ւ�
				switch(lc){
				case(1):
					m_tanCalKind=TanCalKind::ZDIVX;
					break;
				}
				//�O���t�̍X�V���m�肷��̂Ńt���O�𗧂Ă�
				m_updateDataFactoryFlag=true;
			}

		}
	} else{
		if(m_inpFrame>0 && m_updateDataFactoryFlag){
			//�����ꂽ�u�Ԃ��A�O���t�̍X�V������Ă���Ȃ�m_dataFactory���X�V����
			if(!m_input.empty()){
				//�_�l�Ԃɓ��͂��������ꍇ�͂��̓��͂�p����
				CreateFactory(m_input);
			} else{
				//�_�l�Ԃɓ��͂��Ȃ��ꍇ�͌��݂̖_�l�ԓ��͂�]�p����
				CreateFactory(m_dataFactory->IGetInput());
			}
			ret=1;
		}
		//m_input�����
		m_input.clear();
		//���}�E�X�{�^���������Ă��Ȃ�����m_dataFactory�̍X�V�̃t���O��false�ɂ��Ă���
		m_updateDataFactoryFlag=false;
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

	//2�֐ߓ_���̃C���^�[�t�F�[�X�̕`��
	unsigned int lengthColor=GetColor(255,255,255),tanColor=GetColor(255,255,255),twoBoxBackColor=GetColor(96,96,96);
	const int lengthCount=3;
	const char lengthStr[][lengthCount]={"x","y","z"};
	const bool lengthFlag[lengthCount]={m_xLengthFlag,m_yLengthFlag,m_zLengthFlag};
	switch(m_twoPointCalKind){
	case(TwoPointCalKind::LENGTH):
		lengthColor=GetColor(255,255,0);
		break;
	case(TwoPointCalKind::TAN):
		tanColor=GetColor(255,255,0);
		break;
	}
	//���̍��ڗ�
	DrawBox((lengthBoxPos+m_position).x,(lengthBoxPos+m_position).y,(lengthBoxPos+m_position+twoPointBoxSize).x,(lengthBoxPos+m_position+twoPointBoxSize).y,lengthColor,FALSE);
	DrawStringCenterBaseToHandle((lengthBoxPos+m_position+twoPointBoxSize/2).x,(lengthBoxPos+m_position+twoPointBoxSize/2).y,"length",lengthColor,m_font,true);
	DrawBox((tanBoxPos+m_position).x,(tanBoxPos+m_position).y,(tanBoxPos+m_position+twoPointBoxSize).x,(tanBoxPos+m_position+twoPointBoxSize).y,tanColor,FALSE);
	DrawStringCenterBaseToHandle((tanBoxPos+m_position+twoPointBoxSize/2).x,(tanBoxPos+m_position+twoPointBoxSize/2).y,"tan",tanColor,m_font,true);
	//�����C���^�[�t�F�[�X�̕`��
	for(int i=0;i<lengthCount;i++){
		unsigned int inColor=lengthFlag[i] ? lengthColor : twoBoxBackColor;
		const Vector2D v1=lengthBoxPos+m_position+Vector2D(twoPointBoxSize.x*(i+1),0),v2=v1+twoPointBoxSize,v3=(v1+v2)/2;
		DrawBox(v1.x,v1.y,v2.x,v2.y,inColor,TRUE);//����
		DrawBox(v1.x,v1.y,v2.x,v2.y,lengthColor,FALSE);//�O��
		DrawStringCenterBaseToHandle(v3.x,v3.y,lengthStr[i],GetInvertedColor(inColor),m_font,true);//����
	}
	//tan�C���^�[�t�F�[�X�̕`��
	for(int i=0;i<TanCalKind::END;i++){
		unsigned int inColor=i==(m_tanCalKind) ? tanColor : twoBoxBackColor;
		const Vector2D v1=tanBoxPos+m_position+Vector2D(twoPointBoxSize.x*(i+1),0),v2=v1+twoPointBoxSize,v3=(v1+v2)/2;
		DrawBox(v1.x,v1.y,v2.x,v2.y,inColor,TRUE);//����
		DrawBox(v1.x,v1.y,v2.x,v2.y,tanColor,FALSE);//�O��
		DrawStringCenterBaseToHandle(v3.x,v3.y,TanCalKind::str[i].c_str(),GetInvertedColor(inColor),m_font,true);//����
	}
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

std::string GraphDataBuilder::GetFactoryType()const{
	return m_dataFactory->IGetFactoryType();
}
