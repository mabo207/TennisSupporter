#include"KinectTools.h"

//�G���[�`�F�b�N���s���֐�
void ErrorCheck(HRESULT hresult,const char *msg)noexcept(false){
	if(hresult!=S_OK){
		throw(std::runtime_error(msg));
	}
}

