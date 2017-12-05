#include"BodyVirtualKinectSensor.h"

//-------------------BodyVirtualKinectSensor-------------------
BodyVirtualKinectSensor::BodyVirtualKinectSensor():IBodyKinectSensor(){}

BodyVirtualKinectSensor::~BodyVirtualKinectSensor(){}

int BodyVirtualKinectSensor::Update(const std::vector<std::vector<JointPosition>> &frameData){
	for(size_t i=0,topsize=frameData.size();i<bodyNum;i++){
		//�z��̑傫�����L�^
		size_t secondsize=0;
		if(i<topsize){
			secondsize=frameData[i].size();
		}
		//�i�[
		for(size_t j=0;j<JointType_Count;j++){
			if(i<topsize && j<secondsize){
				m_jointPositions[i][j]=frameData[i][j];
			} else{
				//frameData�̔z��O�Q�Ƃ��N���鎞�̓S�~�f�[�^���i�[
				m_jointPositions[i][j]=JointPosition();
			}
		}
	}
	return 0;
}

void BodyVirtualKinectSensor::OutputJointPoitions(std::ofstream &writeFile,const std::vector<std::vector<JointPosition>> &frameData)const{
	if(!(!writeFile)){
		//body�ʒu�̏o��
		//�`����1�s�ɂ��A1�t���[���ł�jointPositions[i][j]�̊e�v�f��(X,Y,Z)�Ƃ����`���ɂ��ďo�́B
		for(size_t i=0,topsize=frameData.size();i<bodyNum;i++){
			//�z��̑傫�����L�^
			size_t secondsize=0;
			if(i<topsize){
				secondsize=frameData[i].size();
			}
			//�i�[
			for(size_t j=0;j<JointType_Count;j++){
				if(i<topsize && j<secondsize){
					writeFile<<frameData[i][j].GetString();
				} else{
					//frameData�̔z��O�Q�Ƃ��N���鎞�̓S�~�f�[�^���i�[
					writeFile<<JointPosition().GetString();
				}
			}
		}
		writeFile<<std::endl;//1�t���[�����̑S�Ă̏o�͂��I�������̂ŉ��s���o��
	}

}

