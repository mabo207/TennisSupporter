#ifndef DEF_TOOLSLIB_H
#define DEF_TOOLSLIB_H

//インクルード
#include<string>
#include<math.h>

//一般的に用いることができる便利関数・構造体をここに書く
//位置についての構造体
class Vector2D{
public:
	//変数
	int x, y;
	//関数
	Vector2D(): x(0),y(0){}//既定のコンストラクタ
	Vector2D(int i_x, int i_y) : x(i_x), y(i_y) {}
	const Vector2D operator+(const Vector2D &otherobj)const {
		return Vector2D(x+otherobj.x,y+otherobj.y);
	}
	const Vector2D operator-(const Vector2D &otherobj)const {
		return Vector2D(x - otherobj.x, y - otherobj.y);
	}
	//定数倍
	const Vector2D operator*(double aMag)const{
		return Vector2D((int)(x*aMag), (int)(y*aMag));
	}
	const Vector2D operator/(double aMag)const {
		return Vector2D((int)(x /aMag), (int)(y/aMag));
	}
	int dot(const Vector2D &otherobj)const {
		return x*otherobj.x + y*otherobj.y;
	}
	//this cross otherobjを返します。
	int cross(const Vector2D &otherobj)const {
		return x*otherobj.y - otherobj.x*y;
	}
	double size()const {
		return sqrt(x*x + y*y);
	}
	//サイズの二乗を返す
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
	//角度を返す(単位はラジアン)
	double GetRadian()const;
	//時計回りに回転させた時のベクトルを返す(角度の単位はラジアン)
	Vector2D turn(double radian)const;
};

//マウスの位置をVector2D型で返す関数
Vector2D GetMousePointVector2D();



//描画関連
//画面全体を描画範囲にする
int SetDrawAllArea();

//文字列描画。\nで改行させる。また右端まで行ったら改行する。
//最後が\0で終わらない文字列だとフリーズまたはオーバーフローが起こる
int DrawStringNewLineToHandle(const int strX,const int strY,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const char *str);

int DrawStringNewLineToHandle(const int strX,const int strY,const int printableX,const int printableY,const int maxDX,const int maxDY,const int Color,const int Font,const int FontSize,const std::string &str);

//上の文字列描画の方式で、描画はせずに必要なY座標の幅のみ求める
int GetStringHeightNewLineToHandle(const int maxDX,const int font,const char *str);

int GetStringHeightNewLineToHandle(const int maxDX,const int font,const std::string str);

//拡大描画。位置指定ではなく大きさ指定で拡大率を指定。
int DrawExtendGraphSizeAssign(int x,int y,int dx,int dy,int GrHandle,int TransFlag);

//中央の描画位置を指定した文字列描画
int DrawStringCenterBaseToHandle(const int centerx,const int centery,const char *str,unsigned int color,int fonthandle,bool yposcenterbaseflag,unsigned int EdgeColor=0U,int VerticalFlag=0);

//右寄せの文字列描画
int DrawStringRightJustifiedToHandle(int x,int y,const std::string &str,int color,int handle,unsigned int edgeColor=0U,int verticalFlag=0);

//int→string変換の際に、0詰めを行うようにする
std::string to_string_0d(int pal,unsigned int length);

//数値の変化を様々な式で管理するクラス
class Easing{
	//列挙体
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
	//変数
protected:
	int flame,maxflame;//フレーム数の管理
	int x,startx,endx;//数値xの管理
	TYPE type;//変化形式
	FUNCTION function;//使用する関数
	double degree;//変化度合い
	//関数
public:
	Easing(int i_x=0,int i_maxflame=0,TYPE i_type=TYPE_IN,FUNCTION i_function=FUNCTION_LINER,double i_degree=0.0);
	virtual ~Easing(){}//デストラクタ
	virtual void Update();//位置更新
	void SetTarget(int i_endx,bool initflame);//目標位置を決める
	void EnforceEnd();//強制的に動作後にする
	void Retry();//動作をリセットしてやり直す
	void Retry(int i_startx);//動作をリセットしてやり直す。スタート位置も変える
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
	virtual bool GetEndFlag()const;//動作が終了しているかを判定する
};

//位置を色々な式で管理するクラス
class PositionControl{
	//列挙体
public:
	//変数
protected:
	Easing x,y;
	//関数
public:
	PositionControl(int i_x=0,int i_y=0,int i_maxflame=0,Easing::TYPE i_type=Easing::TYPE_IN,Easing::FUNCTION i_function=Easing::FUNCTION_LINER,double i_degree=0.0)
		:x(i_x,i_maxflame,i_type,i_function,i_degree),y(i_y,i_maxflame,i_type,i_function,i_degree){}//位置の初期化（最初のみ）
	virtual ~PositionControl(){}//デストラクタ
	virtual void Update();//位置更新
	void SetTarget(int i_endx,int i_endy,bool initflame);//目標位置を決める
	void EnforceEnd();//強制的に動作後にする
	void Retry();//動作をリセットしてやり直す
	void Retry(int i_startx,int i_starty);//動作をリセットしてやり直す。スタート位置も変える
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
	virtual bool GetEndFlag()const;//動作が終了しているかを判定する
};

//大きさ調整しつつ並べて表示する位置を計算するクラス
class LiningupScalingMechanism{
	//型・列挙体
public:
	enum DIRECTION{
		UP,
		LEFT,
		RIGHT,
		UNDER
	};
	//定数

	//変数
protected:
	DIRECTION fixedside;//どの辺を合わせるか
	int startx,starty;//開始位置
	PositionControl size;//拡大している物の調整

						 //関数
public:
	LiningupScalingMechanism(int x,int y,DIRECTION side,PositionControl initsize);
	~LiningupScalingMechanism();
	void SetScaling(int startdx,int startdy,int enddx,int enddy);//大きさを決定する
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

//フレームを数えるためのクラス
class Timer{
	//定数
protected:
	const int fps;//Flame per Second。1以上の値が入る。

	//変数
protected:
	int counter;//フレーム数を数える。Updateのたびに1増えるだけ。
	int startTimer,endTimer;//比較対象。timerまで数える、という場合が殆ど。

	//関数
public:
	Timer(int i_fps);
	~Timer();
	int GetProcessCounter(bool secondFlag)const;//startTimerから数えた経過時間を返す。flame単位か秒単位で返すか選べる。
	int GetLeftCounter(bool secondFlag)const;//endTimerまで残りどのくらいあるかを返す。flame単位か秒単位で返すか選べる。
	bool JudgeEnd()const;//counterがendTimerを超えたかどうかを判定する
	bool SetTimer(int timeLength,bool secondFlag);//タイマーの設定をする。flame単位か秒単位で設定するか選べる。
	void Update();
	void EnforceEnd();
};

#endif // !DEF_TOOLSLIB_H
#pragma once
