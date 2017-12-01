#define _USE_MATH_DEFINES
#include<cmath>
#include"GraphDataBuilder.h"
#include"DxLib.h"
#include"input.h"

//---------------GraphDataBuilder::IDataFactory---------------
std::shared_ptr<GraphDataBuilder::IDataFactory> GraphDataBuilder::IDataFactory::CreateFactory(const std::vector<JointType> &input){
	//inputの個数が3個未満かどうかで作る物を変える
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

void GraphDataBuilder::PosDataFactory::Draw(Vector2D pos)const{
	const Vector2D v=relativeInputPos.find(type)->second+pos;
	DrawCircle(v.x,v.y,circleSize,GetColor(255,128,0));
}

//---------------GraphDataBuilder::AngleDataFactory---------------
GraphDataBuilder::AngleDataFactory::AngleDataFactory(JointType point1,JointType point2,JointType point3)
	:type{point1,point2,point3}{}

double GraphDataBuilder::AngleDataFactory::ICalData(const std::vector<BodyKinectSensor::JointPosition> &data)const{
	return data[type[1]].CalculateAngle(data[type[0]],data[type[2]])/M_PI*180;
}

void GraphDataBuilder::AngleDataFactory::Draw(Vector2D pos)const{
	Vector2D v[indexNum];
	//塗りつぶし
	for(size_t i=0;i<indexNum;i++){
		v[i]=relativeInputPos.find(type[i])->second+pos;
		DrawCircle(v[i].x,v[i].y,circleSize,GetColor(0,128,255));
	}
	//線を引く
	for(size_t i=0;i<indexNum-1;i++){
		DrawLine(v[i].x,v[i].y,v[i+1].x,v[i+1].y,GetColor(0,128,255),3);
	}

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
const int GraphDataBuilder::circleSize=10;

GraphDataBuilder::GraphDataBuilder(Vector2D position)
	:m_position(position),m_dataFactory(IDataFactory::CreateFactory(std::vector<JointType>{JointType_SpineBase})),m_inpFrame(0){}

GraphDataBuilder::~GraphDataBuilder(){}

int GraphDataBuilder::Update(){
	int ret=0;
	const int mFrame=mouse_get(MOUSE_INPUT_LEFT);
	//マウスの入力判定
	if(mFrame>0){
		//現在位置の確認
		Vector2D mousepos=GetMousePointVector2D();
		//m_inputに追加するJointTypeを決定する
		JointType type=JointType_Count;//typeがこの値のままならm_inputに追加しない
		for(const std::pair<JointType,Vector2D> &pair:relativeInputPos){
			if((pair.second+m_position-mousepos).sqSize()<circleSize*circleSize){
				type=pair.first;
				break;
			}
		}
		//m_inputの末尾がtypeに一致せず、なおかつm_inputの中身が最大値(AngleDataFactory::indexNum)を超えていない場合追加する
		if(((m_input.size()==0) || (m_input.back()!=type && m_input.size()<AngleDataFactory::indexNum)) && type!=JointType_Count){
			m_input.push_back(type);
		}
	} else{
		if(m_inpFrame>0 && !m_input.empty()){
			//離された瞬間ならm_dataFactoryを更新する
			m_dataFactory=IDataFactory::CreateFactory(m_input);
			ret=1;
		}
		//マウス入力がされていないならm_inputを空に
		m_input.clear();
	}

	//前フレームにおけるフレーム数の更新
	m_inpFrame=mFrame;

	return ret;
}

void GraphDataBuilder::Draw()const{
	//関節を全て中抜き円で描画
	for(const std::pair<JointType,Vector2D> &pair:relativeInputPos){
		const Vector2D v=m_position+pair.second;
		DrawCircle(v.x,v.y,circleSize,GetColor(0,255,0),FALSE);
	}
	//bornを描画
	const std::map<JointType,Vector2D>::const_iterator ite=relativeInputPos.end();
	for(const std::pair<JointType,JointType> &pair:BodyKinectSensor::bonePairs){
		//全ての関節を描画するわけではないので漏れがある。find()を用いる。
		const std::map<JointType,Vector2D>::const_iterator fit=relativeInputPos.find(pair.first),sit=relativeInputPos.find(pair.second);
		if(fit!=ite && sit!=ite){
			const Vector2D fv=m_position+fit->second,sv=m_position+sit->second;
			DrawLine(fv.x,fv.y,sv.x,sv.y,GetColor(0,255,0),1);
		}
	}
	//現在選択されている関節の円を全て塗りつぶす
	if(m_dataFactory.get()!=nullptr){
		m_dataFactory->Draw(m_position);
	}
	//m_inputとして選択されている関節の円を全て塗りつぶし関節もつなぐ
	std::vector<Vector2D> inpPos;
	//塗りつぶし
	for(size_t i=0,max=m_input.size();i<max;i++){
		inpPos.push_back(relativeInputPos.find(m_input[i])->second+m_position);
		DrawCircle(inpPos[i].x,inpPos[i].y,circleSize,GetColor(255,255,0));
	}
	//線を引く
	for(size_t i=0,max=m_input.size();i+1<max;i++){
		DrawLine(inpPos[i].x,inpPos[i].y,inpPos[i+1].x,inpPos[i+1].y,GetColor(255,0,0),1);
	}

}

double GraphDataBuilder::CalData(const std::vector<BodyKinectSensor::JointPosition> &playData)const{
	return m_dataFactory->ICalData(playData);
}
