#include<vector>
#include"ToolsLib.h"
#include"DxLib.h"
#include<fstream>
#include<iostream>

#define INTMAXINDEX 12//int�^�̍ő包��+2('-'��'\0'�̂��߂�+2)(int�^�̕�����)


//Vector2D�ɂ��Ă̊֐�
double Vector2D::GetRadian()const{
	return std::atan2((double)y,(double)x);
}

Vector2D Vector2D::turn(double radian)const{
	double cos=std::cos(radian),sin=std::sin(radian);
	return Vector2D((int)(x*cos-y*sin),(int)(x*sin+y*cos));	
}

Vector2D GetMousePointVector2D(){
	int x,y;
	GetMousePoint(&x,&y);
	return Vector2D((int)x,(int)y);
}


//�`��֘A
//��ʑS�̂�`��͈͂ɂ���
int SetDrawAllArea(){
	int dx,dy;
	GetScreenState(&dx,&dy,0);

	return SetDrawArea(0,0,dx,dy);
}

//������`��B\n�ŉ��s������B�܂��E�[�܂ōs��������s����B
//�Ōオ\0�ŏI���Ȃ������񂾂ƃt���[�Y�܂��̓I�[�o�[�t���[���N����
int DrawStringNewLineToHandle(const int strX,const int strY,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const char *str){
	//�`��\�͈͂�ݒ�
	SetDrawArea(printableX-1,printableY-1,printableX+maxDX+1,printableY+maxDY+1);//�ׂ��������ŕ������؂�邱�Ƃ�����̂ŗ]�T����������
																				 //������̒������i�[
	int strlength=strlen(str)+1;
	//���ꂼ��̕������ǂꂾ���`�敝���g�p���邩���v�Z����
	int *indexlength=new int[strlength];
	for(int i=0;i<strlength;){
		//i�͌��ݒ��ׂĂ��镶���̈ʒu�������Ă���
		char *partstr;
		if(str[i]>=0 || i+1>=strlength){//i�����ڂ�1�o�C�g�����Ȃ�
			partstr=new char[2];
			partstr[0]=str[i];
			partstr[1]='\0';//���ꂪ�Ȃ��ƕ�����������������̒��������߂Ă��܂�
			indexlength[i]=GetDrawStringWidthToHandle(partstr,2,Font);
			delete[] partstr;
			i+=1;
		} else{//2�o�C�g�����Ȃ�
			partstr=new char[3];
			partstr[0]=str[i];
			partstr[1]=str[i+1];
			partstr[2]='\0';
			indexlength[i]=0;//2�o�C�g�����̏ꍇ��2�o�C�g�ڂɂ̂ݒ������i�[����B
			indexlength[i+1]=GetDrawStringWidthToHandle(partstr,2,Font);
			delete[] partstr;
			i+=2;
		}
	}
	int linenum=0;//�s��(0�s�ځA1�s�ځE�E�E)
	int totalstrlen=0;//i�����ڂ܂ł̕`���̒����̍��v
	int totalstrdrawlen=0;//���܂łɕ`�悵���̕�����̕`���̒����̍��v
	int nowstartindex=0;//���̍s�̕`��̍ŏ��̕����̔z��ԍ�
	for(int i=0;i<strlength;i++){
		totalstrlen+=indexlength[i];
		//i�܂ł̕����������
		if(totalstrlen>maxDX+totalstrdrawlen || str[i]=='\n'){
			//������(i�܂�)�����ɃI�[�o�[����܂��͂����ŉ��s����Ȃ�
			//���̍s�̕�����(endnum�܂�)�𐶐�
			totalstrdrawlen=totalstrlen-indexlength[i];//�`��͂���̂ł��炩���ߍX�V
			int endnum=i;//���̍s�̕�����̍Ō�̕����̒���̔z��ԍ�
			if(endnum>0 && str[endnum-1]<0 && indexlength[endnum-1]==0){//���O�̔z��ԍ��̒��g�����āA2�o�C�g�����Ɣ��f������
				endnum--;//�Ō�̕������ЂƂO�Ɏ����Ă���
				totalstrlen-=indexlength[endnum];//�ς��������̒����������Ă���
			}
			char *STR=new char[endnum-nowstartindex+1];//�����ɏI�[���������邽�߂ɗv�f��1��������
			for(int s=0;s<endnum-nowstartindex;s++){
				STR[s]=str[s+nowstartindex];
			}
			STR[endnum-nowstartindex]='\0';
			//������̕`��
			if(strY+FontSize*(linenum+1)>printableY && strY+FontSize*linenum<printableY+maxDY){//�͈͊O�Ȃ�`�悵�Ȃ�
				DrawStringToHandle(strX,strY+FontSize*linenum,STR,Color,Font);
			}
			//��������̍X�V
			linenum++;
			nowstartindex=endnum;
			if(str[endnum]=='\n'){
				//���s�������݂Đ�ɐi�񂾏ꍇ
				nowstartindex++;//���̍s�͉��s��������X�^�[�g���Ȃ�
				totalstrdrawlen+=indexlength[endnum];//���s�������̒����͒ǉ�����ĂȂ��̂Œǉ�����
			}
			//if���I������
			delete[] STR;
			STR=NULL;
			continue;
		}
		if(i>=strlength-1){//�z�񂪍Ō�܂ŒB������
						   //�c��̕�����̕`��
			const char *pstr=&str[nowstartindex];
			if(strY+FontSize*(linenum+1)>printableY && strY+FontSize*linenum<printableY+maxDY){//�͈͊O�Ȃ�`�悵�Ȃ�
				DrawStringToHandle(strX,strY+FontSize*linenum,pstr,Color,Font);
			}
			//���[�v�𔲂���x��

		}
	}
	SetDrawAllArea();//�`��͈͐ݒ�̉���

					 //�I������
	delete[] indexlength;
	//return 0;
	return (linenum+1)*FontSize;
}

int DrawStringNewLineToHandle(const int X,const int Y,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const std::string &str){
	//�`��
	int returnnum=DrawStringNewLineToHandle(X,Y,printableX,printableY,maxDX,maxDY,Color,Font,FontSize,str.c_str());
	
	return returnnum;
}

//��̕�����`��̕����ŁA�`��͂����ɕK�v��Y���W�̕��̂݋��߂�
int GetStringHeightNewLineToHandle(const int maxDX,const int font,const char *str){
	//�ϐ�������
	int strlength=strlen(str)+1;//str�̕����񐔂��i�[
	int *indexlength=new int[strlength];//str�̊e������X�������߂�
	for(int i=0;i<strlength;){
		//�estr�𒲂ׂāA������̒������i�[����
		if(str[i]<0 && i+1<strlength){//str[i]��2�o�C�g������str[i+1]���z��O�Q�ƂłȂ����
			char *ch=new char[3];
			ch[0]=str[i];
			ch[1]=str[i+1];
			ch[2]='\0';
			indexlength[i]=GetDrawStringWidthToHandle(ch,2,font);
			indexlength[i+1]=0;
			delete[] ch;
			i+=2;
		} else{//1�o�C�g�����̏ꍇ
			char *ch=new char[2];
			ch[0]=str[i];
			ch[1]='\0';
			indexlength[i]=GetDrawStringWidthToHandle(ch,1,font);
			delete[] ch;
			i++;
		}
	}
	int fontsize=GetFontSizeToHandle(font);//�t�H���g�̑傫�����i�[
	int nowlinenum=1;//���ݑ��݂��Ă���s��
	int nowabovelineprintwidth=0;//���݂̍s����ɕ`�悵�Ă��镶�����X��
	int nowprintwidth=0;//���ݕ`�悵�Ă��镶�����X��
						//�s���v�Z
	for(int i=0;i<strlength;i++){
		if(nowprintwidth-nowabovelineprintwidth+indexlength[i]<=maxDX && str[i]!='\n'){
			//���̕������`�悵�Ă��O�ɂ͂ݏo���Ȃ��ꍇ�A�X�ɉ��s�����łȂ��ꍇ
			nowprintwidth+=indexlength[i];//����������ǉ�
		} else{
			//�͂ݏo���ꍇ�A�܂��͉��s�����𔭌������ꍇ
			nowabovelineprintwidth=nowprintwidth;//���݂܂ł̕�����̕����L�^
			nowlinenum++;//�s���̃J�E���g���P���₷
			nowprintwidth+=indexlength[i];//����������ǉ�
		}
	}
	//�I������
	delete[] indexlength;
	return nowlinenum*fontsize;
}

int GetStringHeightNewLineToHandle(const int maxDX,const int font,const std::string str){
	//string�^�����H
	char *c_str=new char[str.size()];
	//strcpy(c_str,str.c_str());
	for(unsigned int i=0;i<str.size();i++){
		c_str[i]=str[i];
	}
	//�v�Z
	int returnnum=GetStringHeightNewLineToHandle(maxDX,font,c_str);
	//�I������
	delete[] c_str;
	c_str=NULL;

	return returnnum;
}

//�g��`��B�ʒu�w��ł͂Ȃ��傫���w��Ŋg�嗦���w��B
int DrawExtendGraphSizeAssign(int x,int y,int dx,int dy,int GrHandle,int TransFlag){
	return DrawExtendGraph(x,y,x+dx,y+dy,GrHandle,TransFlag);
}

//���������̕�����`��
int DrawStringCenterBaseToHandle(const int centerx,const int centery,const char *str,unsigned int color,int fonthandle,bool yposcenterbaseflag,unsigned int EdgeColor,int VerticalFlag){
	if(yposcenterbaseflag){
		return DrawStringToHandle(centerx-GetDrawStringWidthToHandle(str,strlen(str),fonthandle)/2,centery-GetFontSizeToHandle(fonthandle)/2
			,str,color,fonthandle,EdgeColor,VerticalFlag);
	} else{
		return DrawStringToHandle(centerx-GetDrawStringWidthToHandle(str,strlen(str),fonthandle)/2,centery,str,color,fonthandle,EdgeColor,VerticalFlag);
	}
}

//�E�����̕�����`��
int DrawStringRightJustifiedToHandle(int x,int y,const std::string &str,int color,int handle,unsigned int edgeColor,int verticalFlag){
	const char *const pc=str.c_str();
	int dx=GetDrawStringWidthToHandle(pc,str.size(),handle,verticalFlag);
	return DrawStringToHandle(x-dx,y,pc,color,handle,edgeColor,verticalFlag);
}

//int��string�ϊ��̍ۂɁA0�l�߂��s���悤�ɂ���
std::string to_string_0d(int pal,unsigned int length){
	std::string str;
	str+=std::to_string(pal);
	str.reserve(length);
	for(unsigned int i=str.size();i<length;i++){
		str="0"+str;
	}
	return str;
}

//�t�@�C�������݂��邩�ǂ����𒲂ׂ�B���s�t�@�C���̑��݂���f�B���N�g����ɑ��݂��邩�𒲂ׂ�B
bool JudgeFileExist(const std::string &str){
	//�u�t�@�C�����J����v���u�t�@�C�������݂���v�Ƃ����l�����B�Z�L�����e�B�ʂ̖�肪����B
	std::ifstream ifs(str);
	bool flag=ifs.is_open();
	ifs.close();
	return flag;
}

//���]�F�̃J���[�R�[�h���擾����
unsigned int GetInvertedColor(unsigned int color){
	int r,g,b;
	GetColor2(color,&r,&g,&b);
	return GetColor(255-r,255-g,255-b);
}

//���l�ω���l�X�Ȏ��ŊǗ�����N���X
//---Easing---
Easing::Easing(int i_x,int i_maxflame,TYPE i_type,FUNCTION i_function,double i_degree)
	:flame(0),maxflame(i_maxflame),x(i_x),startx(i_x),endx(i_x),type(i_type),function(i_function),degree(i_degree){}

void Easing::SetTarget(int i_endx,bool initflame){
	startx=x;
	if(initflame){
		//initflame��true�̎��̂�flame��0��
		flame=0;
	}
	endx=i_endx;
}

void Easing::Update(){
	double ft;//��������
	if(!GetEndFlag()){
		if(maxflame>0){
			switch(function){
			case(FUNCTION_LINER):
				ft=1.0*flame/maxflame;
				break;
			case(FUNCTION_EXPO):
				if(type==TYPE_IN){
					ft=pow(2.0,degree*(1.0*flame/maxflame-1.0));
				} else if(type==TYPE_OUT){
					ft=1.0-pow(2.0,-degree*flame/maxflame);
				} else if(type==TYPE_INOUT){
					if(flame<maxflame/2){
						ft=pow(2.0,degree*(flame*2.0/maxflame-1.0))/2.0;
					} else{
						ft=1.0-pow(2.0,-degree*(flame*2.0/maxflame-1.0))/2.0;
					}
				}
				break;
			}
		} else{
			ft=1.0;
		}
		x=startx+(int)((endx-startx)*ft);
		flame++;
	} else{
		x=endx;
	}
}

void Easing::EnforceEnd(){
	flame=maxflame;
	Update();
}

void Easing::Retry(){
	x=startx;
	flame=0;
}

void Easing::Retry(int i_startx){
	startx=i_startx;
	Retry();
}

void Easing::SetMaxFlame(int flame,bool targetinitflag){
	maxflame=flame;
	if(targetinitflag){
		Retry(x);
	}else{
		flame=min(flame,maxflame);
	}
}

bool Easing::GetEndFlag()const{
	return (flame>=maxflame);
}


//�ʒu��F�X�Ȏ��ŊǗ�����N���X
//---PositionControl---
void PositionControl::SetTarget(int i_endx,int i_endy,bool initflame){
	x.SetTarget(i_endx,initflame);
	y.SetTarget(i_endy,initflame);
}

void PositionControl::Update(){
	x.Update();
	y.Update();
}

void PositionControl::EnforceEnd(){
	x.EnforceEnd();
	y.EnforceEnd();
}

void PositionControl::Retry(){
	x.Retry();
	y.Retry();
}

void PositionControl::Retry(int i_startx,int i_starty){
	x.Retry(i_startx);
	y.Retry(i_starty);
}

void PositionControl::SetMaxFlame(int flame,bool targetinitflag){
	x.SetMaxFlame(flame,targetinitflag);
	y.SetMaxFlame(flame,targetinitflag);
}

bool PositionControl::GetEndFlag()const{
	//x��y�������t���[���Ǘ��Ȃ̂ŁAy��GetEndFlag��x��GetEndFlag������
	return x.GetEndFlag();
}

//�傫�����������ׂĕ\������ʒu���v�Z����N���X
LiningupScalingMechanism::LiningupScalingMechanism(int x,int y,DIRECTION side,PositionControl initsize)
	:fixedside(side),startx(x),starty(y),size(initsize){}

LiningupScalingMechanism::~LiningupScalingMechanism(){}

void LiningupScalingMechanism::Update(){
	size.Update();
}

void LiningupScalingMechanism::SetScaling(int startdx,int startdy,int enddx,int enddy){
	size=PositionControl(startdx,startdy,size.GetMaxFlame(),size.GetType()
		,size.GetFunction(),size.GetDegree());
	size.SetTarget(enddx,enddy,true);
}

void LiningupScalingMechanism::EnforceEnd(){
	size.EnforceEnd();
}

void LiningupScalingMechanism::Retry(){
	size.Retry();
}

int LiningupScalingMechanism::GetX(int n,int expandingn,int reducingn)const{
	int x=startx;
	switch(fixedside){
	case(UP):
		//��ӂ𑵂��ĉ����тȂ̂ŁAn�Ԗڂ܂őS�đ������킹��
	case(UNDER):
		//���ӂ𑵂������тȂ̂ŁAn�Ԗڂ܂ő������킹��
		for(int i=0;i<n;i++){
			if(i==expandingn){
				x+=GetExpandingSizeX();
			} else if(i==reducingn){
				x+=GetReducingSizeX();
			} else{
				x+=GetNormalSizeX();
			}
		}
		break;
	case(LEFT):
		//���ӂ𑵂��ďc���тȂ̂ŁA���̂܂ܕԂ�
		break;
	case(RIGHT):
		//�E�ӂ𑵂��ďc���тȂ̂ŁAn�Ԗڂ̉����������ĕԂ�
		if(n==expandingn){
			x-=GetExpandingSizeX();
		} else if(n==reducingn){
			x-=GetReducingSizeX();
		} else{
			x-=GetNormalSizeX();
		}
		break;
	default:
		break;
	}
	return x;
}

int LiningupScalingMechanism::GetY(int n,int expandingn,int reducingn)const{
	int y=starty;
	switch(fixedside){
	case(LEFT):
		//���ӂ𑵂��ĉ����тȂ̂ŁAn�Ԗڂ܂őS�đ������킹��
	case(RIGHT):
		//�E�ӂ𑵂������тȂ̂ŁAn�Ԗڂ܂ő������킹��
		for(int i=0;i<n;i++){
			if(i==expandingn){
				y+=GetExpandingSizeY();
			} else if(i==reducingn){
				y+=GetReducingSizeY();
			} else{
				y+=GetNormalSizeY();
			}
		}
		break;
	case(UP):
		//��ӂ𑵂��ďc���тȂ̂ŁA���̂܂ܕԂ�
		break;
	case(UNDER):
		//���ӂ𑵂��ďc���тȂ̂ŁAn�Ԗڂ̉����������ĕԂ�
		if(n==expandingn){
			y-=GetExpandingSizeY();
		} else if(n==reducingn){
			y-=GetReducingSizeY();
		} else{
			y-=GetNormalSizeY();
		}
		break;
	default:
		break;
	}
	return y;
}

int LiningupScalingMechanism::GetNormalSizeX()const{
	return size.GetstartX();
}

int LiningupScalingMechanism::GetNormalSizeY()const{
	return size.GetstartY();
}

int LiningupScalingMechanism::GetExpandingSizeX()const{
	return size.GetX();
}

int LiningupScalingMechanism::GetExpandingSizeY()const{
	return size.GetY();
}

int LiningupScalingMechanism::GetReducingSizeX()const{
	return size.GetendX()+size.GetstartX()-size.GetX();
}

int LiningupScalingMechanism::GetReducingSizeY()const{
	return size.GetendY()+size.GetstartY()-size.GetY();
}

//�t���[�����𐔂��邽�߂̃N���XTimer
Timer::Timer(int i_fps)
	:fps(max(1,i_fps)),counter(0),startTimer(0),endTimer(0){}

Timer::~Timer(){}

int Timer::GetProcessCounter(bool secondFlag)const{
	if(secondFlag){
		//�b�P�ʂŕԂ�
		return (counter-startTimer)/fps;
	}else{
		//flame�P�ʂŕԂ�
		return counter-startTimer;
	}
}

int Timer::GetLeftCounter(bool secondFlag)const{
	if(secondFlag){
		//�b�P�ʂŕԂ�
		return (endTimer-startTimer)/fps-GetProcessCounter(true);
	}else{
		//flame�P�ʂŕԂ�
		return endTimer-counter;
	}
}

bool Timer::JudgeEnd()const{
	return counter>=endTimer;
}

bool Timer::SetTimer(int timeLength,bool secondFlag){
	startTimer=counter;
	if(secondFlag){
		//�b�P�ʂŐݒ�
		endTimer=startTimer+timeLength*fps;
	} else{
		//flame�P�ʂŐݒ�
		endTimer=startTimer+timeLength;
	}
	return true;//���s���Ȃ��̂�true��Ԃ��B
}

void Timer::Update(){
	counter++;
}

void Timer::EnforceEnd(){
	counter=endTimer;
}



