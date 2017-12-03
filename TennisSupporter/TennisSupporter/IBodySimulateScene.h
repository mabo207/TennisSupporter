#ifndef DEF_BODYSIMULATESCENE_H
#define DEF_BODYSIMULATESCENE_H

#include"KinectTools.h"

//�V�~�����[�^��ŋ��ʂ��ėp����萔�ƃN���X�`�����`����
class IBodySimulateScene{
	//�񋓑́E�^
public:
	//�p����̃N���X���ǂ̂悤�ȃN���X�����`����񋓑�
	struct MODE{
		enum TYPE{
			PHOTOGRAPHER
			,PLAYER
			,END
		};
		static const TYPE link(int num);
	};

	//�萔
public:
	static const int writeCountMax;//����ȏ�̎��Ԃ̓f�[�^���������܂Ȃ��悤�ɂ���
	static const int captureFps;//�B�e�f�[�^��fps
	static const int drawFps;//�`�掞��fps
	static const Vector2D kinectSize;//KinectV2�̎擾�\�ȉ摜�T�C�Y�ibody,depth�j

	//�ϐ�
protected:
	const MODE::TYPE m_type;

	//�֐�
public:
	IBodySimulateScene(MODE::TYPE type):m_type(type){}
	virtual ~IBodySimulateScene(){}
	virtual int Update()=0;
	virtual void Draw()const=0;
	MODE::TYPE GetType()const{
		return m_type;
	}
};

#endif // !DEF_BODYSIMULATECONSTPALS_H
#pragma once
