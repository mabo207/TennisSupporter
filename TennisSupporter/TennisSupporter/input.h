#ifndef DEF_INPUT_H
#define DEF_INPUT_H


#include"ToolsLib.h"
#include<set>
#include<string>

//�L�[�{�[�h�֘A
int input_update();

int keyboard_get(int KeyCode);

int mouse_get(int MouseCode);

Vector2D analogjoypad_get(int InputType);

void input_erase();//���͏���S�ď���(�ǂ̃{�^�������͂���ĂȂ����Ƃɂ���)

void keyboard_COMinput(int KeyCode);//�{�^���������ꂽ���Ƃɂ���

//���͊֘A
void InitInputControler();

void DeleteInputControler();

class InputControler{
	//�^
protected:
	//�Q�[���p�b�h�̃{�^���ƃL�[�{�[�h�̑Ή�������\����
	struct GamepadKeyboardMap{
		int keyboard;
		int padbutton;
		bool operator<(const GamepadKeyboardMap &otherobj)const;
		bool operator==(const GamepadKeyboardMap &otherobj)const;
		GamepadKeyboardMap(int i_keyboard,int i_padbutton)
			:keyboard(i_keyboard),padbutton(i_padbutton){}
		~GamepadKeyboardMap(){}
	};
	//�Q�[���p�b�h�̃W���C�p�b�h�̓��͏�ԂƃL�[�{�[�h�̑Ή�������\����
	struct AnalogJoypadKeyboardMap{
		int keyboard;//�Ή��L�[�{�[�h����
		double center;//���S�̊p�x
		double okerror;//���e�덷�p�x��
		double sizemin;//�K�v�ȓ��͂̑傫��
		bool operator<(const AnalogJoypadKeyboardMap &otherobj)const;
		bool operator==(const AnalogJoypadKeyboardMap &otherobj)const;
		AnalogJoypadKeyboardMap(int i_keyboard,double i_center,double i_okerror,double i_sizemin)
			:keyboard(i_keyboard),center(i_center),okerror(i_okerror),sizemin(i_sizemin){}
		~AnalogJoypadKeyboardMap(){}
		bool JudgeInput()const;
	};

	//�萔
	static const std::string InitFileName;
	static const int KeyNum=256;//�L�[�{�[�h�̓��̓L�[��
	static const int MouseButtonNum=8;//�}�E�X�̓��̓{�^����

	//�ϐ�
	int m_keyboardFlame[KeyNum];//�e�L�[�{�[�h�����͂��ꂽ�t���[����
	int m_mouseFlame[MouseButtonNum];//�e�}�E�X�̃{�^�������͂��ꂽ�t���[����
	std::set<GamepadKeyboardMap> m_connectmap;//�Q�[���p�b�h�ƃL�[�{�[�h�̑Ή��\
	std::set<AnalogJoypadKeyboardMap> m_stickmap;//�A�i���O�X�e�B�b�N�ƃL�[�{�[�h�̑Ή��\

	//�֐�
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

//���A���^�C���ł̔��p��������͂��T�|�[�g����N���X�Bkeyboard_get()��p����B
class InputSingleCharStringControler{
	//�^�E�񋓑�

	//�萔
	static const int inputBreakFlame;//�L�[���������ςȂ��ɂ������A���̃t���[�����̊Ԃ������͂�h��

	//�ϐ�
protected:
	std::string m_string;//���͕�����
	const std::string m_banString;//���͋֎~�����̈ꗗ
	const size_t m_maxLen;//������̒����̏���B0�Ȃ琧���Ȃ��B������'\0'���܂߂�B
	bool m_inputFlag;//���͒����̃t���O�B�����ltrue

	//�֐�
protected:
	char InputCharString()const;//���͂��ꂽ������Ԃ��B�������͂���ĂȂ��Ȃ�'\0'��Ԃ��B

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
	void Update();//�X�V�BEnter�L�[�̓��͂œ��͂��I����m_inputFlag��false�ƂȂ�B

};

#endif