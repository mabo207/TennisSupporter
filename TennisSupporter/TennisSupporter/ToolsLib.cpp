#include<vector>
#include"ToolsLib.h"
#include"DxLib.h"
#include<fstream>
#include<iostream>

#define INTMAXINDEX 12//int型の最大桁数+2('-'と'\0'のための+2)(int型の文字数)


//Vector2Dについての関数
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


//描画関連
//画面全体を描画範囲にする
int SetDrawAllArea(){
	int dx,dy;
	GetScreenState(&dx,&dy,0);

	return SetDrawArea(0,0,dx,dy);
}

//文字列描画。\nで改行させる。また右端まで行ったら改行する。
//最後が\0で終わらない文字列だとフリーズまたはオーバーフローが起こる
int DrawStringNewLineToHandle(const int strX,const int strY,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const char *str){
	//描画可能範囲を設定
	SetDrawArea(printableX-1,printableY-1,printableX+maxDX+1,printableY+maxDY+1);//細かい部分で文字が切れることがあるので余裕を持たせる
																				 //文字列の長さを格納
	int strlength=strlen(str)+1;
	//それぞれの文字がどれだけ描画幅を使用するかを計算する
	int *indexlength=new int[strlength];
	for(int i=0;i<strlength;){
		//iは現在調べている文字の位置を示している
		char *partstr;
		if(str[i]>=0 || i+1>=strlength){//i文字目が1バイト文字なら
			partstr=new char[2];
			partstr[0]=str[i];
			partstr[1]='\0';//これがないと文字化けした文字列の長さを求めてしまう
			indexlength[i]=GetDrawStringWidthToHandle(partstr,2,Font);
			delete[] partstr;
			i+=1;
		} else{//2バイト文字なら
			partstr=new char[3];
			partstr[0]=str[i];
			partstr[1]=str[i+1];
			partstr[2]='\0';
			indexlength[i]=0;//2バイト文字の場合は2バイト目にのみ長さを格納する。
			indexlength[i+1]=GetDrawStringWidthToHandle(partstr,2,Font);
			delete[] partstr;
			i+=2;
		}
	}
	int linenum=0;//行数(0行目、1行目・・・)
	int totalstrlen=0;//i文字目までの描画上の長さの合計
	int totalstrdrawlen=0;//今までに描画したの文字列の描画上の長さの合計
	int nowstartindex=0;//今の行の描画の最初の文字の配列番号
	for(int i=0;i<strlength;i++){
		totalstrlen+=indexlength[i];
		//iまでの文字列を見る
		if(totalstrlen>maxDX+totalstrdrawlen || str[i]=='\n'){
			//文字列(iまで)が横にオーバーするまたはここで改行するなら
			//この行の文字列(endnumまで)を生成
			totalstrdrawlen=totalstrlen-indexlength[i];//描画はするのであらかじめ更新
			int endnum=i;//この行の文字列の最後の文字の直後の配列番号
			if(endnum>0 && str[endnum-1]<0 && indexlength[endnum-1]==0){//直前の配列番号の中身を見て、2バイト文字と判断したら
				endnum--;//最後の文字をひとつ前に持っていく
				totalstrlen-=indexlength[endnum];//変えた部分の長さも引いておく
			}
			char *STR=new char[endnum-nowstartindex+1];//末尾に終端文字を入れるために要素を1多くする
			for(int s=0;s<endnum-nowstartindex;s++){
				STR[s]=str[s+nowstartindex];
			}
			STR[endnum-nowstartindex]='\0';
			//文字列の描画
			if(strY+FontSize*(linenum+1)>printableY && strY+FontSize*linenum<printableY+maxDY){//範囲外なら描画しない
				DrawStringToHandle(strX,strY+FontSize*linenum,STR,Color,Font);
			}
			//文字列情報の更新
			linenum++;
			nowstartindex=endnum;
			if(str[endnum]=='\n'){
				//改行文字をみて先に進んだ場合
				nowstartindex++;//次の行は改行文字からスタートしない
				totalstrdrawlen+=indexlength[endnum];//改行文字分の長さは追加されてないので追加する
			}
			//if文終了処理
			delete[] STR;
			STR=NULL;
			continue;
		}
		if(i>=strlength-1){//配列が最後まで達したら
						   //残りの文字列の描画
			const char *pstr=&str[nowstartindex];
			if(strY+FontSize*(linenum+1)>printableY && strY+FontSize*linenum<printableY+maxDY){//範囲外なら描画しない
				DrawStringToHandle(strX,strY+FontSize*linenum,pstr,Color,Font);
			}
			//ループを抜ける支持

		}
	}
	SetDrawAllArea();//描画範囲設定の解除

					 //終了処理
	delete[] indexlength;
	//return 0;
	return (linenum+1)*FontSize;
}

int DrawStringNewLineToHandle(const int X,const int Y,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const std::string &str){
	//描画
	int returnnum=DrawStringNewLineToHandle(X,Y,printableX,printableY,maxDX,maxDY,Color,Font,FontSize,str.c_str());
	
	return returnnum;
}

//上の文字列描画の方式で、描画はせずに必要なY座標の幅のみ求める
int GetStringHeightNewLineToHandle(const int maxDX,const int font,const char *str){
	//変数初期化
	int strlength=strlen(str)+1;//strの文字列数を格納
	int *indexlength=new int[strlength];//strの各文字のX幅を求める
	for(int i=0;i<strlength;){
		//各strを調べて、文字列の長さを格納する
		if(str[i]<0 && i+1<strlength){//str[i]が2バイト文字でstr[i+1]が配列外参照でなければ
			char *ch=new char[3];
			ch[0]=str[i];
			ch[1]=str[i+1];
			ch[2]='\0';
			indexlength[i]=GetDrawStringWidthToHandle(ch,2,font);
			indexlength[i+1]=0;
			delete[] ch;
			i+=2;
		} else{//1バイト文字の場合
			char *ch=new char[2];
			ch[0]=str[i];
			ch[1]='\0';
			indexlength[i]=GetDrawStringWidthToHandle(ch,1,font);
			delete[] ch;
			i++;
		}
	}
	int fontsize=GetFontSizeToHandle(font);//フォントの大きさを格納
	int nowlinenum=1;//現在存在している行数
	int nowabovelineprintwidth=0;//現在の行より上に描画している文字列のX幅
	int nowprintwidth=0;//現在描画している文字列のX幅
						//行数計算
	for(int i=0;i<strlength;i++){
		if(nowprintwidth-nowabovelineprintwidth+indexlength[i]<=maxDX && str[i]!='\n'){
			//次の文字列を描画しても外にはみ出さない場合、更に改行文字でない場合
			nowprintwidth+=indexlength[i];//書く文字を追加
		} else{
			//はみ出す場合、または改行文字を発見した場合
			nowabovelineprintwidth=nowprintwidth;//現在までの文字列の幅を記録
			nowlinenum++;//行数のカウントを１つ増やす
			nowprintwidth+=indexlength[i];//書く文字を追加
		}
	}
	//終了処理
	delete[] indexlength;
	return nowlinenum*fontsize;
}

int GetStringHeightNewLineToHandle(const int maxDX,const int font,const std::string str){
	//string型を加工
	char *c_str=new char[str.size()];
	//strcpy(c_str,str.c_str());
	for(unsigned int i=0;i<str.size();i++){
		c_str[i]=str[i];
	}
	//計算
	int returnnum=GetStringHeightNewLineToHandle(maxDX,font,c_str);
	//終了処理
	delete[] c_str;
	c_str=NULL;

	return returnnum;
}

//拡大描画。位置指定ではなく大きさ指定で拡大率を指定。
int DrawExtendGraphSizeAssign(int x,int y,int dx,int dy,int GrHandle,int TransFlag){
	return DrawExtendGraph(x,y,x+dx,y+dy,GrHandle,TransFlag);
}

//中央揃えの文字列描画
int DrawStringCenterBaseToHandle(const int centerx,const int centery,const char *str,unsigned int color,int fonthandle,bool yposcenterbaseflag,unsigned int EdgeColor,int VerticalFlag){
	if(yposcenterbaseflag){
		return DrawStringToHandle(centerx-GetDrawStringWidthToHandle(str,strlen(str),fonthandle)/2,centery-GetFontSizeToHandle(fonthandle)/2
			,str,color,fonthandle,EdgeColor,VerticalFlag);
	} else{
		return DrawStringToHandle(centerx-GetDrawStringWidthToHandle(str,strlen(str),fonthandle)/2,centery,str,color,fonthandle,EdgeColor,VerticalFlag);
	}
}

//右揃えの文字列描画
int DrawStringRightJustifiedToHandle(int x,int y,const std::string &str,int color,int handle,unsigned int edgeColor,int verticalFlag){
	const char *const pc=str.c_str();
	int dx=GetDrawStringWidthToHandle(pc,str.size(),handle,verticalFlag);
	return DrawStringToHandle(x-dx,y,pc,color,handle,edgeColor,verticalFlag);
}

//int→string変換の際に、0詰めを行うようにする
std::string to_string_0d(int pal,unsigned int length){
	std::string str;
	str+=std::to_string(pal);
	str.reserve(length);
	for(unsigned int i=str.size();i<length;i++){
		str="0"+str;
	}
	return str;
}

//ファイルが存在するかどうかを調べる。実行ファイルの存在するディレクトリ上に存在するかを調べる。
bool JudgeFileExist(const std::string &str){
	//「ファイルが開ける」→「ファイルが存在する」という考え方。セキュリティ面の問題がある。
	std::ifstream ifs(str);
	bool flag=ifs.is_open();
	ifs.close();
	return flag;
}

//反転色のカラーコードを取得する
unsigned int GetInvertedColor(unsigned int color){
	int r,g,b;
	GetColor2(color,&r,&g,&b);
	return GetColor(255-r,255-g,255-b);
}

//数値変化を様々な式で管理するクラス
//---Easing---
Easing::Easing(int i_x,int i_maxflame,TYPE i_type,FUNCTION i_function,double i_degree)
	:flame(0),maxflame(i_maxflame),x(i_x),startx(i_x),endx(i_x),type(i_type),function(i_function),degree(i_degree){}

void Easing::SetTarget(int i_endx,bool initflame){
	startx=x;
	if(initflame){
		//initflameがtrueの時のみflameを0に
		flame=0;
	}
	endx=i_endx;
}

void Easing::Update(){
	double ft;//増加割合
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


//位置を色々な式で管理するクラス
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
	//xもyも同じフレーム管理なので、yのGetEndFlagもxのGetEndFlagも同じ
	return x.GetEndFlag();
}

//大きさ調整しつつ並べて表示する位置を計算するクラス
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
		//上辺を揃えて横並びなので、n番目まで全て足し合わせる
	case(UNDER):
		//下辺を揃え横並びなので、n番目まで足し合わせる
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
		//左辺を揃えて縦並びなので、そのまま返す
		break;
	case(RIGHT):
		//右辺を揃えて縦並びなので、n番目の横幅を引いて返す
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
		//左辺を揃えて横並びなので、n番目まで全て足し合わせる
	case(RIGHT):
		//右辺を揃え横並びなので、n番目まで足し合わせる
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
		//上辺を揃えて縦並びなので、そのまま返す
		break;
	case(UNDER):
		//下辺を揃えて縦並びなので、n番目の横幅を引いて返す
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

//フレーム数を数えるためのクラスTimer
Timer::Timer(int i_fps)
	:fps(max(1,i_fps)),counter(0),startTimer(0),endTimer(0){}

Timer::~Timer(){}

int Timer::GetProcessCounter(bool secondFlag)const{
	if(secondFlag){
		//秒単位で返す
		return (counter-startTimer)/fps;
	}else{
		//flame単位で返す
		return counter-startTimer;
	}
}

int Timer::GetLeftCounter(bool secondFlag)const{
	if(secondFlag){
		//秒単位で返す
		return (endTimer-startTimer)/fps-GetProcessCounter(true);
	}else{
		//flame単位で返す
		return endTimer-counter;
	}
}

bool Timer::JudgeEnd()const{
	return counter>=endTimer;
}

bool Timer::SetTimer(int timeLength,bool secondFlag){
	startTimer=counter;
	if(secondFlag){
		//秒単位で設定
		endTimer=startTimer+timeLength*fps;
	} else{
		//flame単位で設定
		endTimer=startTimer+timeLength;
	}
	return true;//失敗しないのでtrueを返す。
}

void Timer::Update(){
	counter++;
}

void Timer::EnforceEnd(){
	counter=endTimer;
}



