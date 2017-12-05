#include"BodyVirtualKinectSensor.h"

//-------------------BodyVirtualKinectSensor-------------------
BodyVirtualKinectSensor::BodyVirtualKinectSensor():IBodyKinectSensor(){}

BodyVirtualKinectSensor::~BodyVirtualKinectSensor(){}

int BodyVirtualKinectSensor::Update(const std::vector<std::vector<JointPosition>> &frameData){
	for(size_t i=0,topsize=frameData.size();i<bodyNum;i++){
		//配列の大きさを記録
		size_t secondsize=0;
		if(i<topsize){
			secondsize=frameData[i].size();
		}
		//格納
		for(size_t j=0;j<JointType_Count;j++){
			if(i<topsize && j<secondsize){
				m_jointPositions[i][j]=frameData[i][j];
			} else{
				//frameDataの配列外参照が起こる時はゴミデータを格納
				m_jointPositions[i][j]=JointPosition();
			}
		}
	}
	return 0;
}

void BodyVirtualKinectSensor::OutputJointPoitions(std::ofstream &writeFile,const std::vector<std::vector<JointPosition>> &frameData)const{
	if(!(!writeFile)){
		//body位置の出力
		//形式は1行につき、1フレームでのjointPositions[i][j]の各要素を(X,Y,Z)という形式にして出力。
		for(size_t i=0,topsize=frameData.size();i<bodyNum;i++){
			//配列の大きさを記録
			size_t secondsize=0;
			if(i<topsize){
				secondsize=frameData[i].size();
			}
			//格納
			for(size_t j=0;j<JointType_Count;j++){
				if(i<topsize && j<secondsize){
					writeFile<<frameData[i][j].GetString();
				} else{
					//frameDataの配列外参照が起こる時はゴミデータを格納
					writeFile<<JointPosition().GetString();
				}
			}
		}
		writeFile<<std::endl;//1フレーム内の全ての出力が終了したので改行を出力
	}

}

