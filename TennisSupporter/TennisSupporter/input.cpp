#define _USE_MATH_DEFINES
#include<iostream>
#include"DxLib.h"
#include"input.h"

//入力関連
static InputControler *inputControler;

int input_update(){
	return inputControler->Update();
}

//キーボード関連
int keyboard_get(int KeyCode){//KeyCodeはキーボードの名前
	return inputControler->Get(KeyCode);
}

//マウス関連
int mouse_get(int MouseCode){
	return inputControler->MouseGet(MouseCode);
}

//アナログジョイパッド関連
Vector2D analogjoypad_get(int InputType){
	int x,y;
	GetJoypadAnalogInput(&x,&y,InputType);
	return Vector2D((float)x,(float)y);
}

//入力情報を全て消す(どのボタンも入力されてないことにする)
void input_erase(){
	inputControler->InitInput();
}

//ボタンを押されたことにする
void keyboard_COMinput(int KeyCode){
	inputControler->COMinput(KeyCode);
}

//入力関連
void InitInputControler(){
	inputControler=new InputControler();
}

void DeleteInputControler(){
	if(inputControler!=NULL){
		delete inputControler;
		inputControler=NULL;
	}
}

const std::string InputControler::InitFileName="Database/savedata/InputEdit.csv";

bool InputControler::GamepadKeyboardMap::operator<(const GamepadKeyboardMap &otherobj)const{
	if(keyboard < otherobj.keyboard){
		return true;
	}else if(keyboard > otherobj.keyboard){
		return false;
	}
	return (padbutton < otherobj.padbutton);
}

bool InputControler::GamepadKeyboardMap::operator==(const GamepadKeyboardMap &otherobj)const{
	return (this->keyboard == otherobj.keyboard);
}

bool InputControler::AnalogJoypadKeyboardMap::operator<(const AnalogJoypadKeyboardMap &otherobj)const{
	//keyboardの大きさで順序を決める。
	if(keyboard < otherobj.keyboard){
		return true;
	} else if(keyboard > otherobj.keyboard){
		return false;
	}
	//以下テキトー。矛盾だけ生じないようにしている。
	if(center < otherobj.center){
		return true;
	} else if(center > otherobj.center){
		return false;
	}
	if(okerror < otherobj.okerror){
		return true;
	} else if(okerror > otherobj.okerror){
		return false;
	}
	return(sizemin < otherobj.sizemin);
}

bool InputControler::AnalogJoypadKeyboardMap::operator==(const AnalogJoypadKeyboardMap &otherobj)const{
	return (keyboard==otherobj.keyboard);
}

bool InputControler::AnalogJoypadKeyboardMap::JudgeInput()const{
	Vector2D v=analogjoypad_get(DX_INPUT_PAD1);
	//十字キーによるvの補正
	if(GetJoypadNum()>0){
		if(keyboard_get(KEY_INPUT_UP)>0){
			v.y-=1000;
		}
		if(keyboard_get(KEY_INPUT_LEFT)>0){
			v.x-=1000;
		}
		if(keyboard_get(KEY_INPUT_RIGHT)>0){
			v.x+=1000;
		}
		if(keyboard_get(KEY_INPUT_DOWN)>0){
			v.y+=1000;
		}
	}
	//角度の認識
	double radian=v.GetRadian();
	double size=(double)v.size();
if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
		int poi=0;
	}


	return (std::fmin(std::abs(radian-center),M_PI*2-std::abs(radian-center))<okerror && sizemin<=size);
}

InputControler::InputControler(){
	for(int i=0;i<KeyNum;i++){
		m_keyboardFlame[i]=0;
	}
	for(int i=0;i<MouseButtonNum;i++){
		m_mouseFlame[i]=0;
	}
	
	//ジョイパッドボタンとキーボードの対応表
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_NUMPADENTER,PAD_INPUT_4));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_BACK,PAD_INPUT_3));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_Q,PAD_INPUT_5));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_R,PAD_INPUT_6));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_1,PAD_INPUT_7));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_4,PAD_INPUT_8));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_UP,PAD_INPUT_UP));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_LEFT,PAD_INPUT_LEFT));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_RIGHT,PAD_INPUT_RIGHT));
	m_connectmap.insert(GamepadKeyboardMap(KEY_INPUT_DOWN,PAD_INPUT_DOWN));

	
	//ジョイパッドのアナログスティックとキーボードの対応表
	//(centerに誤差を与えておくことで上下キー入力がdoubleの計算誤差によって左上左下に対応して不可逆入力になるという歪な事態を避ける)
	SetJoypadDeadZone(DX_INPUT_PAD1,0.0);
	m_stickmap.insert(AnalogJoypadKeyboardMap(KEY_INPUT_W,-M_PI*2/3+0.0001,M_PI/6,1000));
	m_stickmap.insert(AnalogJoypadKeyboardMap(KEY_INPUT_E,-M_PI/3+0.0001,M_PI/6,1000));
	m_stickmap.insert(AnalogJoypadKeyboardMap(KEY_INPUT_D,0.0001,M_PI/6,1000));
	m_stickmap.insert(AnalogJoypadKeyboardMap(KEY_INPUT_X,M_PI/3+0.0001,M_PI/6,1000));
	m_stickmap.insert(AnalogJoypadKeyboardMap(KEY_INPUT_Z,M_PI*2/3+0.0001,M_PI/6,1000));
	m_stickmap.insert(AnalogJoypadKeyboardMap(KEY_INPUT_A,M_PI+0.0001,M_PI/6,1000));
}

InputControler::~InputControler(){}

int InputControler::Update(){
	//キーボードの更新
	char tmpKey[256];
	GetHitKeyStateAll(tmpKey);//tmpKeyだとtmpKeyの先頭のアドレスを示す
	int tmpPad=0;//ゲームパッドのボタンの入力
	if(GetJoypadNum()>0){
		tmpPad=GetJoypadInputState(DX_INPUT_PAD1);//ジョイパッドが１つでも入力されていれば、１つ目のジョイパッドのボタン入力を受け付ける
	}
	std::set<GamepadKeyboardMap>::iterator cit=m_connectmap.begin(),cite=m_connectmap.end();
	std::set<AnalogJoypadKeyboardMap>::iterator sit=m_stickmap.begin(),site=m_stickmap.end();
	bool cflag,sflag;
	for(int i=0;i<KeyNum;i++){
		cflag=((cit!=cite) && (cit->keyboard)==i);//キーボード入力に対応するパッドボタンが存在すればtrueとなる。
		sflag=((sit!=site) && (sit->keyboard)==i);//キーボード入力に対応するジョイパッドアナログスティックの入力方法が存在すればtrueとなる。
		if(tmpKey[i]!=0 || (cflag && (tmpPad & cit->padbutton)!=0) || (sflag && sit->JudgeInput())){
			m_keyboardFlame[i]++;
		}else{
			m_keyboardFlame[i]=0;
		}
		if(cflag){
			cit++;
		}
		if(sflag){
			sit++;
		}
	}
	//マウスの更新
	int mouseinput=GetMouseInput();
	for(int i=0;i<MouseButtonNum;i++){
		if((mouseinput>>i) & 0x01){
			m_mouseFlame[i]++;
		}else{
			m_mouseFlame[i]=0;
		}
	}
	return 0;
}

int InputControler::Get(int KeyCode){
	if(KeyCode>=0 && KeyCode<KeyNum){
		return m_keyboardFlame[KeyCode];
	}
	return 0;
}

int InputControler::MouseGet(int MouseCode){
	//MouseCodeの下からx個目のbitが1ならばm_mouseFlame[x-1]に入力フレーム数が格納されている
	//つまりn回右シフトしたところ1が見えたならばm_mouseFlame[n]を返してあげれば良い
	//ここでは、最下位bitが1のものを1bitずつ左シフトしていき、それとのAND演算によって何bit目に1があるかを検出する
	int bit=0x01;
	for(int i=0;i<MouseButtonNum;i++){
		if(MouseCode & (bit<<i)){
			return m_mouseFlame[i];
		}
	}
	return 0;
}

void InputControler::InitInput(){
	for(int i=0;i<KeyNum;i++){
		m_keyboardFlame[i]=0;
	}
	for(int i=0;i<MouseButtonNum;i++){
		m_mouseFlame[i]=0;
	}
}

void InputControler::COMinput(int KeyCode){
	if(KeyCode>=0 && KeyCode<KeyNum){
		m_keyboardFlame[KeyCode]++;
	}
}

void InputControler::AddConnectMap(int KeyCode,int PadButton){
	m_connectmap.insert(GamepadKeyboardMap(KeyCode,PadButton));
}

void InputControler::MapSaving(){

}

//リアルタイム半角文字列入力のサポート
const int InputSingleCharStringControler::inputBreakFlame=30;

InputSingleCharStringControler::InputSingleCharStringControler(const std::string &banString,size_t maxLen)
	:m_string(""),m_banString(banString),m_maxLen(maxLen),m_inputFlag(true)
{
	//計算量削減のために、文字数制限が決まっている場合は予めメモリ確保をする。
	if(maxLen>0){
		m_string.reserve(maxLen);
	}
}

InputSingleCharStringControler::~InputSingleCharStringControler(){}

char InputSingleCharStringControler::InputCharString()const{
	//スーパーベタ実装。Shift入りの入力を確認する。
	bool shiftFlag=(keyboard_get(KEY_INPUT_LSHIFT)>0 || keyboard_get(KEY_INPUT_RSHIFT)>0);
	auto f=[](int key)->bool{int t=keyboard_get(key);return (t==1 || t>=inputBreakFlame);};
	if(f(KEY_INPUT_TAB)){
		if(!shiftFlag){
			return '\t';
		} else{
			return '\t';
		}
	} else if(f(KEY_INPUT_SPACE)){
		if(!shiftFlag){
			return ' ';
		} else{
			return ' ';
		}
	} else if(f(KEY_INPUT_MINUS)){
		if(!shiftFlag){
			return '-';
		} else{
			return '=';
		}
	} else if(f(KEY_INPUT_YEN)){
		if(!shiftFlag){
			return '\\';
		} else{
			return '|';
		}
	} else if(f(KEY_INPUT_PREVTRACK)){
		if(!shiftFlag){
			return '^';
		} else{
			return '~';
		}
	} else if(f(KEY_INPUT_PERIOD)){
		if(!shiftFlag){
			return '.';
		} else{
			return '>';
		}
	} else if(f(KEY_INPUT_SLASH)){
		if(!shiftFlag){
			return '/';
		} else{
			return '?';
		}
	} else if(f(KEY_INPUT_SEMICOLON)){
		if(!shiftFlag){
			return ';';
		} else{
			return '+';
		}
	} else if(f(KEY_INPUT_COLON)){
		if(!shiftFlag){
			return ':';
		} else{
			return '*';
		}
	} else if(f(KEY_INPUT_LBRACKET)){
		if(!shiftFlag){
			return '[';
		} else{
			return '{';
		}
	} else if(f(KEY_INPUT_RBRACKET)){
		if(!shiftFlag){
			return ']';
		} else{
			return '}';
		}
	} else if(f(KEY_INPUT_AT)){
		if(!shiftFlag){
			return '@';
		} else{
			return '`';
		}
	} else if(f(KEY_INPUT_BACKSLASH)){
		if(!shiftFlag){
			return '\\';
		} else{
			return '_';
		}
	} else if(f(KEY_INPUT_COMMA)){
		if(!shiftFlag){
			return ',';
		} else{
			return '<';
		}
	} else if(f(KEY_INPUT_0)){
		if(!shiftFlag){
			return '0';
		} else{
			return '\0';
		}
	} else if(f(KEY_INPUT_1)){
		if(!shiftFlag){
			return '1';
		} else{
			return '!';
		}
	} else if(f(KEY_INPUT_2)){
		if(!shiftFlag){
			return '2';
		} else{
			return '"';
		}
	} else if(f(KEY_INPUT_3)){
		if(!shiftFlag){
			return '3';
		} else{
			return '#';
		}
	} else if(f(KEY_INPUT_4)){
		if(!shiftFlag){
			return '4';
		} else{
			return '$';
		}
	} else if(f(KEY_INPUT_5)){
		if(!shiftFlag){
			return '5';
		} else{
			return '%';
		}
	} else if(f(KEY_INPUT_6)){
		if(!shiftFlag){
			return '6';
		} else{
			return '&';
		}
	} else if(f(KEY_INPUT_7)){
		if(!shiftFlag){
			return '7';
		} else{
			return '\'';
		}
	} else if(f(KEY_INPUT_8)){
		if(!shiftFlag){
			return '8';
		} else{
			return '(';
		}
	} else if(f(KEY_INPUT_9)){
		if(!shiftFlag){
			return '9';
		} else{
			return ')';
		}
	} else if(f(KEY_INPUT_A)){
		if(!shiftFlag){
			return 'a';
		} else{
			return 'A';
		}
	} else if(f(KEY_INPUT_B)){
		if(!shiftFlag){
			return 'b';
		} else{
			return 'B';
		}
	} else if(f(KEY_INPUT_C)){
		if(!shiftFlag){
			return 'c';
		} else{
			return 'C';
		}
	} else if(f(KEY_INPUT_D)){
		if(!shiftFlag){
			return 'd';
		} else{
			return 'D';
		}
	} else if(f(KEY_INPUT_E)){
		if(!shiftFlag){
			return 'e';
		} else{
			return 'E';
		}
	} else if(f(KEY_INPUT_F)){
		if(!shiftFlag){
			return 'f';
		} else{
			return 'F';
		}
	} else if(f(KEY_INPUT_G)){
		if(!shiftFlag){
			return 'g';
		} else{
			return 'G';
		}
	} else if(f(KEY_INPUT_H)){
		if(!shiftFlag){
			return 'h';
		} else{
			return 'H';
		}
	} else if(f(KEY_INPUT_I)){
		if(!shiftFlag){
			return 'i';
		} else{
			return 'I';
		}
	} else if(f(KEY_INPUT_J)){
		if(!shiftFlag){
			return 'j';
		} else{
			return 'J';
		}
	} else if(f(KEY_INPUT_K)){
		if(!shiftFlag){
			return 'k';
		} else{
			return 'K';
		}
	} else if(f(KEY_INPUT_L)){
		if(!shiftFlag){
			return 'l';
		} else{
			return 'L';
		}
	} else if(f(KEY_INPUT_M)){
		if(!shiftFlag){
			return 'm';
		} else{
			return 'M';
		}
	} else if(f(KEY_INPUT_N)){
		if(!shiftFlag){
			return 'n';
		} else{
			return 'N';
		}
	} else if(f(KEY_INPUT_O)){
		if(!shiftFlag){
			return 'o';
		} else{
			return 'O';
		}
	} else if(f(KEY_INPUT_P)){
		if(!shiftFlag){
			return 'p';
		} else{
			return 'P';
		}
	} else if(f(KEY_INPUT_Q)){
		if(!shiftFlag){
			return 'q';
		} else{
			return 'Q';
		}
	} else if(f(KEY_INPUT_R)){
		if(!shiftFlag){
			return 'r';
		} else{
			return 'R';
		}
	} else if(f(KEY_INPUT_S)){
		if(!shiftFlag){
			return 's';
		} else{
			return 'S';
		}
	} else if(f(KEY_INPUT_T)){
		if(!shiftFlag){
			return 't';
		} else{
			return 'T';
		}
	} else if(f(KEY_INPUT_U)){
		if(!shiftFlag){
			return 'u';
		} else{
			return 'U';
		}
	} else if(f(KEY_INPUT_V)){
		if(!shiftFlag){
			return 'v';
		} else{
			return 'V';
		}
	} else if(f(KEY_INPUT_W)){
		if(!shiftFlag){
			return 'w';
		} else{
			return 'W';
		}
	} else if(f(KEY_INPUT_X)){
		if(!shiftFlag){
			return 'x';
		} else{
			return 'X';
		}
	} else if(f(KEY_INPUT_Y)){
		if(!shiftFlag){
			return 'y';
		} else{
			return 'Y';
		}
	} else if(f(KEY_INPUT_Z)){
		if(!shiftFlag){
			return 'z';
		} else{
			return 'Z';
		}
	}
	/*
	if(f(KEY_INPUT_)){
		if(!shiftFlag){
			return '';
		} else{
			return '';
		}
	} else 
	//*/
	return '\0';
}

void InputSingleCharStringControler::Update(){
	if(m_inputFlag){
		//入力中なら
		char c=InputCharString();//文字列取得
		if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
			//Enterキーが押された場合
			m_inputFlag=false;//入力終了
		} else if((keyboard_get(KEY_INPUT_BACK)==1 || keyboard_get(KEY_INPUT_BACK)>inputBreakFlame) && !m_string.empty()){
			//backキーが押された場合(30フレーム以降は連続入力)
			m_string.pop_back();//1文字削除
		} else{
			//その他の入力
			if(c!='\0'){
				//無入力ではない
				if(m_maxLen==0 || m_string.size()+1<m_maxLen){
					//文字数制限をクリアしている
					if(m_banString.find(c)==std::string::npos){
						//禁止文字列の入力でもない
						m_string.push_back(c);
					}
				}
			}
		}
	}
}
