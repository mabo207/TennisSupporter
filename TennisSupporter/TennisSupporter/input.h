#ifndef DEF_INPUT_H
#define DEF_INPUT_H


#include"ToolsLib.h"
#include<set>
#include<string>

//キーボード関連
int input_update();

int keyboard_get(int KeyCode);

int mouse_get(int MouseCode);

Vector2D analogjoypad_get(int InputType);

void input_erase();//入力情報を全て消す(どのボタンも入力されてないことにする)

void keyboard_COMinput(int KeyCode);//ボタンを押されたことにする

//入力関連
void InitInputControler();

void DeleteInputControler();

class InputControler{
	//型
protected:
	//ゲームパッドのボタンとキーボードの対応をする構造体
	struct GamepadKeyboardMap{
		int keyboard;
		int padbutton;
		bool operator<(const GamepadKeyboardMap &otherobj)const;
		bool operator==(const GamepadKeyboardMap &otherobj)const;
		GamepadKeyboardMap(int i_keyboard,int i_padbutton)
			:keyboard(i_keyboard),padbutton(i_padbutton){}
		~GamepadKeyboardMap(){}
	};
	//ゲームパッドのジョイパッドの入力状態とキーボードの対応をする構造体
	struct AnalogJoypadKeyboardMap{
		int keyboard;//対応キーボード入力
		double center;//中心の角度
		double okerror;//許容誤差角度幅
		double sizemin;//必要な入力の大きさ
		bool operator<(const AnalogJoypadKeyboardMap &otherobj)const;
		bool operator==(const AnalogJoypadKeyboardMap &otherobj)const;
		AnalogJoypadKeyboardMap(int i_keyboard,double i_center,double i_okerror,double i_sizemin)
			:keyboard(i_keyboard),center(i_center),okerror(i_okerror),sizemin(i_sizemin){}
		~AnalogJoypadKeyboardMap(){}
		bool JudgeInput()const;
	};

	//定数
	static const std::string InitFileName;
	static const int KeyNum=256;//キーボードの入力キー数
	static const int MouseButtonNum=8;//マウスの入力ボタン数

	//変数
	int m_keyboardFlame[KeyNum];//各キーボードが入力されたフレーム数
	int m_mouseFlame[MouseButtonNum];//各マウスのボタンが入力されたフレーム数
	std::set<GamepadKeyboardMap> m_connectmap;//ゲームパッドとキーボードの対応表
	std::set<AnalogJoypadKeyboardMap> m_stickmap;//アナログスティックとキーボードの対応表

	//関数
protected:

public:
	InputControler();
	~InputControler();
	int Update();
	int Get(int KeyCode);
	int MouseGet(int MouseCode);
	void InitInput();
	void COMinput(int KeyCode);
	void AddConnectMap(int KeyCode,int PadButton);
	void MapSaving();
};

//リアルタイムでの半角文字列入力をサポートするクラス。keyboard_get()を用いる。
class InputSingleCharStringControler{
	//型・列挙体

	//定数
	static const int inputBreakFlame;//キーを押しっぱなしにした時、このフレーム数の間だけ入力を防ぐ

	//変数
protected:
	std::string m_string;//入力文字列
	const std::string m_banString;//入力禁止文字の一覧
	const size_t m_maxLen;//文字列の長さの上限。0なら制限なし。末尾の'\0'を含める。
	bool m_inputFlag;//入力中かのフラグ。初期値true

	//関数
protected:
	char InputCharString()const;//入力された文字を返す。何も入力されてないなら'\0'を返す。

public:
	InputSingleCharStringControler(const std::string &banString,size_t maxLen=0);
	~InputSingleCharStringControler();
	std::string GetString()const{
		return m_string;
	}
	const char *GetStringCStr()const{
		return m_string.c_str();
	}
	bool GetInputFlag()const{
		return m_inputFlag;
	}
	void Update();//更新。Enterキーの入力で入力が終了しm_inputFlagがfalseとなる。

};

#endif