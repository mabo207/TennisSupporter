#include"GraphSingleData.h"

//--------------GraphSingleData--------------
GraphSingleData::GraphSingleData():m_dataMin(300),m_dataMax(-300){}

GraphSingleData::~GraphSingleData(){}

void GraphSingleData::DataBuild(std::shared_ptr<const GraphDataBuilder> pGraphDataBuilder){
	size_t playdatasize=m_playData.size();
	m_data.clear();
	m_data.reserve(playdatasize);
	for(size_t i=0;i<playdatasize;i++){
		double data=0.0;
		for(size_t j=0,bodynum=m_playData[i].size();j<bodynum;j++){
			bool flag=false;
			for(size_t k=0,jointnum=m_playData[i][j].size();k<jointnum;k++){
				if(!(m_playData[i][j][k]==IBodyKinectSensor::JointPosition())){
					flag=true;
					break;
				}
			}
			if(flag){
				data=pGraphDataBuilder->CalData(m_playData[i][j]);
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
}

void GraphSingleData::UpdateVirtualSensor(const int index){
	if(index>=0 && index<(int)m_playData.size()){
		m_pBodyVirtualKinectSensor->Update(m_playData[index]);
	}
}

void GraphSingleData::WriteGraphSingleData(std::ofstream &writeFile)const{
	//m_dataを1行に出力する。
	if(!writeFile.is_open()){
		//出力できない場合はreturn
		return;
	}
	for(std::vector<double>::const_iterator it=m_data.begin(),itb=m_data.begin(),ite=m_data.end();it!=ite;it++){
		//","の出力
		if(it!=itb){
			//先頭イテレータのみ出力しない
			writeFile<<",";
		}
		//数値の出力
		writeFile<<(*it);
	}
	//全て出力したら改行出力
	writeFile<<std::endl;
}
