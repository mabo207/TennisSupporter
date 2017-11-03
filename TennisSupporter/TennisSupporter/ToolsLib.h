#ifndef DEF_TOOLSLIB_H
#define DEF_TOOLSLIB_H

//�C���N���[�h
#include<string>
#include<math.h>

//��ʓI�ɗp���邱�Ƃ��ł���֗��֐��E�\���̂������ɏ���
//�ʒu�ɂ��Ă̍\����
class Vector2D{
public:
	//�ϐ�
	int x, y;
	//�֐�
	Vector2D(): x(0),y(0){}//����̃R���X�g���N�^
	Vector2D(int i_x, int i_y) : x(i_x), y(i_y) {}
	const Vector2D operator+(const Vector2D &otherobj)const {
		return Vector2D(x+otherobj.x,y+otherobj.y);
	}
	const Vector2D operator-(const Vector2D &otherobj)const {
		return Vector2D(x - otherobj.x, y - otherobj.y);
	}
	//�萔�{
	const Vector2D operator*(double aMag)const{
		return Vector2D((int)(x*aMag), (int)(y*aMag));
	}
	const Vector2D operator/(double aMag)const {
		return Vector2D((int)(x /aMag), (int)(y/aMag));
	}
	int dot(const Vector2D &otherobj)const {
		return x*otherobj.x + y*otherobj.y;
	}
	//this cross otherobj��Ԃ��܂��B
	int cross(const Vector2D &otherobj)const {
		return x*otherobj.y - otherobj.x*y;
	}
	double size()const {
		return sqrt(x*x + y*y);
	}
	//�T�C�Y�̓���Ԃ�
	int sqSize()const {
		return x*x + y*y;
	}
	Vector2D norm()const {
		double siz = size();
		if(siz!=0.0){
			return Vector2D((int)(x / siz),(int)(y / siz));
		}
		return Vector2D(0,0);
	}
	//�p�x��Ԃ�(�P�ʂ̓��W�A��)
	double GetRadian()const;
	//���v���ɉ�]���������̃x�N�g����Ԃ�(�p�x�̒P�ʂ̓��W�A��)
	Vector2D turn(double radian)const;
};

//�}�E�X�̈ʒu��Vector2D�^�ŕԂ��֐�
Vector2D GetMousePointVector2D();



//�`��֘A
//��ʑS�̂�`��͈͂ɂ���
int SetDrawAllArea();

//������`��B\n�ŉ��s������B�܂��E�[�܂ōs��������s����B
//�Ōオ\0�ŏI���Ȃ������񂾂ƃt���[�Y�܂��̓I�[�o�[�t���[���N����
int DrawStringNewLineToHandle(const int strX,const int strY,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const char *str);

int DrawStringNewLineToHandle(const int strX,const int strY,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const std::string &str);

//��̕�����`��̕����ŁA�`��͂����ɕK�v��Y���W�̕��̂݋��߂�
int GetStringHeightNewLineToHandle(const int maxDX,const int font,const char *str);

int GetStringHeightNewLineToHandle(const int maxDX,const int font,const std::string str);

//�g��`��B�ʒu�w��ł͂Ȃ��傫���w��Ŋg�嗦���w��B
int DrawExtendGraphSizeAssign(int x,int y,int dx,int dy,int GrHandle,int TransFlag);

//�����̕`��ʒu���w�肵��������`��
int DrawStringCenterBaseToHandle(const int centerx,const int centery,const char *str,unsigned int color,int fonthandle,bool yposcenterbaseflag,unsigned int EdgeColor=0U,int VerticalFlag=0);

//�E�񂹂̕�����`��
int DrawStringRightJustifiedToHandle(int x,int y,const std::string &str,int color,int handle,unsigned int edgeColor=0U,int verticalFlag=0);

//int��string�ϊ��̍ۂɁA0�l�߂��s���悤�ɂ���
std::string to_string_0d(int pal,unsigned int length);

//���l�̕ω���l�X�Ȏ��ŊǗ�����N���X
class Easing{
	//�񋓑�
public:
	enum TYPE{
		TYPE_IN,
		TYPE_OUT,
		TYPE_INOUT
	};
	enum FUNCTION{
		FUNCTION_LINER,
		FUNCTION_EXPO
	};
	//�ϐ�
protected:
	int flame,maxflame;//�t���[�����̊Ǘ�
	int x,startx,endx;//���lx�̊Ǘ�
	TYPE type;//�ω��`��
	FUNCTION function;//�g�p����֐�
	double degree;//�ω��x����
	//�֐�
public:
	Easing(int i_x=0,int i_maxflame=0,TYPE i_type=TYPE_IN,FUNCTION i_function=FUNCTION_LINER,double i_degree=0.0);
	virtual ~Easing(){}//�f�X�g���N�^
	virtual void Update();//�ʒu�X�V
	void SetTarget(int i_endx,bool initflame);//�ڕW�ʒu�����߂�
	void EnforceEnd();//�����I�ɓ����ɂ���
	void Retry();//��������Z�b�g���Ă�蒼��
	void Retry(int i_startx);//��������Z�b�g���Ă�蒼���B�X�^�[�g�ʒu���ς���
	int GetX()const{
		return x;
	}
	int GetstartX()const{
		return startx;
	}
	int GetendX()const{
		return endx;
	}
	int GetFlame()const{
		return flame;
	}
	virtual int GetMaxFlame()const{
		return maxflame;
	}
	FUNCTION GetFunction()const{
		return function;
	}
	TYPE GetType()const{
		return type;
	}
	double GetDegree()const{
		return degree;
	}
	void SetMaxFlame(int flame,bool targetinitflag);
	virtual bool GetEndFlag()const;//���삪�I�����Ă��邩�𔻒肷��
};

//�ʒu��F�X�Ȏ��ŊǗ�����N���X
class PositionControl{
	//�񋓑�
public:
	//�ϐ�
protected:
	Easing x,y;
	//�֐�
public:
	PositionControl(int i_x=0,int i_y=0,int i_maxflame=0,Easing::TYPE i_type=Easing::TYPE_IN,Easing::FUNCTION i_function=Easing::FUNCTION_LINER,double i_degree=0.0)
		:x(i_x,i_maxflame,i_type,i_function,i_degree),y(i_y,i_maxflame,i_type,i_function,i_degree){}//�ʒu�̏������i�ŏ��̂݁j
	virtual ~PositionControl(){}//�f�X�g���N�^
	virtual void Update();//�ʒu�X�V
	void SetTarget(int i_endx,int i_endy,bool initflame);//�ڕW�ʒu�����߂�
	void EnforceEnd();//�����I�ɓ����ɂ���
	void Retry();//��������Z�b�g���Ă�蒼��
	void Retry(int i_startx,int i_starty);//��������Z�b�g���Ă�蒼���B�X�^�[�g�ʒu���ς���
	Easing GetEasingX()const{
		return x;
	}
	Easing GetEasingY()const{
		return y;
	}
	int GetX()const{
		return x.GetX();
	}
	int GetstartX()const{
		return x.GetstartX();
	}
	int GetendX()const{
		return x.GetendX();
	}
	int GetY()const{
		return y.GetX();
	}
	int GetstartY()const{
		return y.GetstartX();
	}
	int GetendY()const{
		return y.GetendX();
	}
	int GetFlame()const{
		return x.GetFlame();
	}
	virtual int GetMaxFlame()const{
		return x.GetMaxFlame();
	}
	Easing::FUNCTION GetFunction()const{
		return x.GetFunction();
	}
	Easing::TYPE GetType()const{
		return x.GetType();
	}
	double GetDegree()const{
		return x.GetDegree();
	}
	void SetMaxFlame(int flame,bool targetinitflag);
	virtual bool GetEndFlag()const;//���삪�I�����Ă��邩�𔻒肷��
};

//�傫�����������ׂĕ\������ʒu���v�Z����N���X
class LiningupScalingMechanism{
	//�^�E�񋓑�
public:
	enum DIRECTION{
		UP,
		LEFT,
		RIGHT,
		UNDER
	};
	//�萔

	//�ϐ�
protected:
	DIRECTION fixedside;//�ǂ̕ӂ����킹�邩
	int startx,starty;//�J�n�ʒu
	PositionControl size;//�g�債�Ă��镨�̒���

						 //�֐�
public:
	LiningupScalingMechanism(int x,int y,DIRECTION side,PositionControl initsize);
	~LiningupScalingMechanism();
	void SetScaling(int startdx,int startdy,int enddx,int enddy);//�傫�������肷��
	void Update();
	void Retry();
	void EnforceEnd();
	int GetX(int n,int expandingn,int reducingn)const;
	int GetY(int n,int expandingn,int reducingn)const;
	int GetNormalSizeX()const;
	int GetNormalSizeY()const;
	int GetExpandingSizeX()const;
	int GetExpandingSizeY()const;
	int GetReducingSizeX()const;
	int GetReducingSizeY()const;
};

//�t���[���𐔂��邽�߂̃N���X
class Timer{
	//�萔
protected:
	const int fps;//Flame per Second�B1�ȏ�̒l������B

	//�ϐ�
protected:
	int counter;//�t���[�����𐔂���BUpdate�̂��т�1�����邾���B
	int startTimer,endTimer;//��r�ΏہBtimer�܂Ő�����A�Ƃ����ꍇ���w�ǁB

	//�֐�
public:
	Timer(int i_fps);
	~Timer();
	int GetProcessCounter(bool secondFlag)const;//startTimer���琔�����o�ߎ��Ԃ�Ԃ��Bflame�P�ʂ��b�P�ʂŕԂ����I�ׂ�B
	int GetLeftCounter(bool secondFlag)const;//endTimer�܂Ŏc��ǂ̂��炢���邩��Ԃ��Bflame�P�ʂ��b�P�ʂŕԂ����I�ׂ�B
	bool JudgeEnd()const;//counter��endTimer�𒴂������ǂ����𔻒肷��
	bool SetTimer(int timeLength,bool secondFlag);//�^�C�}�[�̐ݒ������Bflame�P�ʂ��b�P�ʂŐݒ肷�邩�I�ׂ�B
	void Update();
	void EnforceEnd();
};

#endif // !DEF_TOOLSLIB_H
#pragma once
