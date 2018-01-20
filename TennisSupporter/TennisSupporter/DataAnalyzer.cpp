#define _USE_MATH_DEFINES

#include<cmath>
#include"DataAnalyzer.h"
#include"DxLib.h"
#include"input.h"

//-----------------DataAnalyzer-----------------
const Vector2D DataAnalyzer::graphPos=Vector2D(100,60);
const Vector2D DataAnalyzer::graphSize=Vector2D(DataAnalyzer::writeCountMax,360);
const std::string DataAnalyzer::sectionStr="##################";

DataAnalyzer::DataAnalyzer(int font,const char *filename)
	:IBodySimulateScene(MODE::ANALYZER,font),m_playFrame(0.0),m_playRate(1.0),
	m_playFlag(true),m_graphUnity(false),m_extend(1.0),m_dataAverage(0.0),m_widthUnity(false),m_dataSizeMax(1)
{
	//GraphDataBuilderの起動
	m_pGraphDataBuilder=std::shared_ptr<GraphDataBuilder>(new GraphDataBuilder(Vector2D(kinectSize.x*2+80,0),m_font));
	//データの読み取り
	ReadFile(filename);
}

DataAnalyzer::DataAnalyzer(int font,const char *filename,std::shared_ptr<GraphDataBuilder> pCopyDataBuilder)
	:IBodySimulateScene(MODE::ANALYZER,font),m_playFrame(0.0),m_playRate(1.0),
	m_playFlag(true),m_graphUnity(false),m_extend(1.0),m_dataAverage(0.0),m_widthUnity(false),m_dataSizeMax(1),m_pGraphDataBuilder(pCopyDataBuilder)
{
	//データの読み取り
	ReadFile(filename);
}

DataAnalyzer::~DataAnalyzer(){}

bool DataAnalyzer::ReadFile(const char *filename){
	std::ifstream readFile(filename);
	//データを全て読み込むが、量が多いので上手くreserveしながら読み込む
	GraphSingleData graphSingleData;
	
	graphSingleData.m_playData.clear();
	graphSingleData.m_playData.reserve(writeCountMax);//最大記録フレーム数によりreserve

	std::vector<std::vector<IBodyKinectSensor::JointPosition>> frameData;
	frameData.reserve(IBodyKinectSensor::bodyNum);

	std::vector<IBodyKinectSensor::JointPosition> bodyData;
	bodyData.reserve(JointType_Count);

	size_t bodyindex=0,jointindex=0;
	std::string parts="";
	parts.reserve(50);
	bool inBracketsFlag=false;//現在"(~~)"の中を読み取っているかどうか
	bool sectionFlag=false;//現在sectionStrを読み取っているかどうか
	char ch='a';//初期値はテキトー
	try{
		while(true){
			ch=readFile.get();
			if(ch==EOF){
				//ファイルの末尾到達時
				//現在のgraphSingleDataをm_graphDataに格納
				if(!graphSingleData.m_playData.empty()){
					graphSingleData.m_pBodyVirtualKinectSensor=std::shared_ptr<BodyVirtualKinectSensor>(new BodyVirtualKinectSensor());
					m_graphData.push_back(graphSingleData);
				}
				graphSingleData.m_playData.clear();
				break;
			} else if(ch=='\n'){
				if(!sectionFlag){
					//1フレーム読み終わりの時
					graphSingleData.m_playData.push_back(frameData);
				} else{
					//1人分読み終わりの時(sectionStrの行を読み終わる)
					if(!graphSingleData.m_playData.empty()){
						graphSingleData.m_pBodyVirtualKinectSensor=std::shared_ptr<BodyVirtualKinectSensor>(new BodyVirtualKinectSensor());
						m_graphData.push_back(graphSingleData);
					}
					graphSingleData.m_playData.clear();
					sectionFlag=false;
				}
				//以下共通処理
				frameData.clear();
				bodyData.clear();
				bodyindex=0;
				jointindex=0;
				parts="";
				inBracketsFlag=false;
			} else if(ch=='#'){
				//#を1文字読んだら、その行のそれ以降の部分は読み取らない
				sectionFlag=true;
			} else if(!sectionFlag){
				//sectionStrを読んでいない
				if(inBracketsFlag){
					//()内読み取り時(=')'を探している)
					parts.push_back(ch);
					if(ch==')'){
						inBracketsFlag=!inBracketsFlag;
						//jointPositionsに格納
						if(bodyindex<IBodyKinectSensor::bodyNum && jointindex<JointType_Count){
							bodyData.push_back(IBodyKinectSensor::JointPosition(parts));
							//index系の更新
							jointindex++;
							if(jointindex>=JointType_Count){
								jointindex=0;
								bodyindex++;
								frameData.push_back(bodyData);
								bodyData.clear();
							}
						}
						parts.clear();
					}
				} else{
					//()外読み取り時(='('を探している)
					if(ch=='('){
						inBracketsFlag=!inBracketsFlag;
						parts.push_back(ch);
					}
				}
			}
		}
	} catch(const std::exception &e){
		//メモリ関連のエラー対策
		printf(e.what());
		return false;
	}
	//グラフについてのデータ読み取り
	DataBuild();
	readFile.close();

	//再生データの初期化
	m_playFlag=true;
	
	//ファイル名を拡張子以外保存
	std::string fname(filename);
	size_t index=0;
	for(size_t size=fname.size();index<size;index++){
		if(fname[index]=='.'){
			break;
		}
	}
	m_playDataName.clear();
	m_playDataName.reserve(index);
	for(size_t i=0;i<index;i++){
		m_playDataName.push_back(fname[i]);
	}

	return true;
}

void DataAnalyzer::DataBuild(){
	for(GraphSingleData &gdata:m_graphData){
		gdata.DataBuild(m_pGraphDataBuilder);
	}
	//m_dataAverageとm_dataSizeMaxを求める
	size_t size=0,memo=0;
	m_dataAverage=0;
	m_dataSizeMax=1;
	for(const GraphSingleData &gdata:m_graphData){
		for(const double &data:gdata.m_data){
			size++;
			m_dataAverage+=data;
		}
		m_dataSizeMax=(size_t)std::fmax(m_dataSizeMax,size-memo);
		memo=size;
	}
	m_dataAverage/=std::fmax(size,1);
	//フレーム数を初期化、イメージも更新
	m_playFrame=0.0;
	UpdateImage();
}

int DataAnalyzer::CalReadIndex()const{
	return (int)(m_playFrame*captureFps/drawFps);
}

double DataAnalyzer::CalPlayFrame(int index)const{
	return ((double)index)*drawFps/captureFps;
}

bool DataAnalyzer::JudgeMouseInGraph()const{
	Vector2D relativeMouse=GetMousePointVector2D()-graphPos;
	return (relativeMouse.x>=0 && relativeMouse.x<=graphSize.x && relativeMouse.y>=0 && relativeMouse.y<=graphSize.y);
}

void DataAnalyzer::UpdateImage(){
	UpdateImage(CalReadIndex());
}

void DataAnalyzer::UpdateImage(int index){
	for(GraphSingleData &gdata:m_graphData){
		gdata.UpdateVirtualSensor(index);
	}
}

void DataAnalyzer::OutputGraphData()const{
	//ファイル名の作成
	std::string fname=m_playDataName+"_"+m_pGraphDataBuilder->GetFactoryType()+".csv";
	//書き出し
	OutputGraphData(fname.c_str());
}

void DataAnalyzer::OutputGraphData(const char *filename)const{
	//書き出しファイルをオープン
	if(JudgeFileExist(filename)){
		//同一ファイル名が既に存在していたら、特に何もしない

	} else{
		//存在していないなら
		std::ofstream writeFile(filename,std::ios_base::trunc);
		if(!writeFile){
			//ファイルを開くのを失敗したら、特に何もしない

		} else{
			//正常に書き込みができる
			for(const GraphSingleData &gdata:m_graphData){
				//1行に1つのデータ系列ずつ書き出し
				gdata.WriteGraphSingleData(writeFile);
			}
		}
		writeFile.close();
	}
}

void DataAnalyzer::InputToOutputFolder()const{
	//読み取り元、書き出し先フォルダ名生成規則
	const std::string inpDir="Input/",outDir="Output/";
	const std::string fName[]={
		"A-suburi",
		"A-serve",
		"B-suburi",
		"B-serve",
		"C-suburi",
		"C-serve",
		"D-suburi",
		"D-serve",
		"E-suburi",
		"E-serve",
		"F-suburi",
		"F-serve",
		"G-suburi",
		"G-serve"
	};
	//書き出し
	for(const std::string &fname:fName){
		DataAnalyzer d(-1,(inpDir+fname+".txt").c_str(),m_pGraphDataBuilder);
		d.OutputGraphData((outDir+fname+"_"+m_pGraphDataBuilder->GetFactoryType()+".csv").c_str());
	}
}

int DataAnalyzer::Update(){
	printfDx("PlayingDataMode\n");
	//秒数更新
	if(mouse_get(MOUSE_INPUT_LEFT)>0 && JudgeMouseInGraph()){
		//グラフ内で左クリック時、再生時間をそこに合わせて、イメージ更新
		m_playFrame=CalPlayFrame((GetMousePointVector2D().x-graphPos.x)*writeCountMax/graphSize.x);
		UpdateImage();
	} else if(keyboard_get(KEY_INPUT_NUMPADENTER)==1){
		//Enterキー入力で先頭から再生し、イメージ更新
		m_playFrame=0.0;
		UpdateImage();
	} else if(keyboard_get(KEY_INPUT_RSHIFT)==1){
		//右シフトキー入力で再生停止の切り替え
		m_playFlag=!m_playFlag;
	} else if(keyboard_get(KEY_INPUT_LEFT)==1){
		//左キー入力で再生位置を戻し、イメージ更新
		m_playFrame-=m_playRate;
		UpdateImage();
	} else if(keyboard_get(KEY_INPUT_RIGHT)==1){
		//右キー入力で再生位置を進め、イメージ更新
		m_playFrame+=m_playRate;
		UpdateImage();
	}
	//イメージ再生画面
	if(mouse_get(MOUSE_INPUT_RIGHT)<=0){
		//右クリックが押されていない場合は普通に再生
		int a=CalReadIndex();
		if(m_playFlag && a<writeCountMax){
			//まだデータ終端までいっておらず、かつ再生モードになっている時、フレーム数を更新
			m_playFrame+=m_playRate;
		}
		int b=CalReadIndex();//この値がaに一致している時は読み込みは行わず、前フレームと同じ画像を描画する
		if(a!=b){
			UpdateImage();
		}
	} else{
		//右クリックが押されている場合は、マウスの位置に従ってイメージを更新
		int index=GetMousePointVector2D().x-graphPos.x;
		UpdateImage(index);
	}
	//入力インターフェース
	if(m_pGraphDataBuilder->Update()==1){
		//m_dataFactoryを更新した時はDataBuild()を使用する
		DataBuild();
	}
	//グラフの軸基準の切り替え
	if(keyboard_get(KEY_INPUT_U)==1){
		m_graphUnity=!m_graphUnity;
	}
	//グラフの横軸均一化の切り替え
	if(keyboard_get(KEY_INPUT_I)==1){
		m_widthUnity=!m_widthUnity;
	}
	//再生速度調整
	if(keyboard_get(KEY_INPUT_Z)>0){
		//遅くする
		m_playRate=std::fmax(0.1,m_playRate-0.05);
	} else if(keyboard_get(KEY_INPUT_X)>0){
		//1.0倍に戻す
		m_playRate=1.0;
	} else if(keyboard_get(KEY_INPUT_C)>0){
		//速くする
		m_playRate=m_playRate+0.05;
	}
	printfDx("playRate:\n%f",m_playRate);
	//グラフ拡大率調整
	if(keyboard_get(KEY_INPUT_Q)>0){
		//小さくする
		m_extend=std::fmax(0.1,m_extend-0.05);
	} else if(keyboard_get(KEY_INPUT_W)>0){
		//1.0倍に戻す
		m_extend=1.0;
	} else if(keyboard_get(KEY_INPUT_E)>0){
		//大きくする
		const int frame=keyboard_get(KEY_INPUT_E);
		double speed;
		if(frame<60){
			speed=0.05;
		} else{
			speed=0.002*frame;
		}
		m_extend=m_extend+speed;
	}
	//csv出力
	if(keyboard_get(KEY_INPUT_S)==10){
		//現在表示しているグラフをcsv出力
		OutputGraphData();
	} else if(keyboard_get(KEY_INPUT_A)==20){
		//Inputにあるデータのグラフ化をOutputに出力
		InputToOutputFolder();
	}

	//場面遷移
	if(keyboard_get(KEY_INPUT_BACK)==1){
		//Backキー入力で記録モードに戻る
		return 1;
	}
	return 0;
}

void DataAnalyzer::Draw()const{
	//描画位置
	Vector2D depthPos(kinectSize/2)
		,xyPos(kinectSize.x/2,kinectSize.y*3/2)
		,zyPos(kinectSize*3/2);
	
	//某人間は描画しない
	//m_graphSingleData.m_pBodyVirtualKinectSensor->Draw(nullptr,Vector2D(-3000,-3000),kinectSize,xyPos,kinectSize,zyPos,kinectSize);//(depth画像に対するbodyボーンは描画しない)
	
	//グラフ描画
	//統一基準(1pxにつき1フレームという基準が消滅する)
	//最大最小値の設定
	double dataTop=m_pGraphDataBuilder->DataMax(),dataBottom=m_pGraphDataBuilder->DataMin();
	//倍率調整
	double dataCenter=(dataTop+dataBottom)/2;
	if(m_graphUnity){
		dataTop=dataCenter+(dataTop-dataCenter)/m_extend;
		dataBottom=dataCenter+(dataBottom-dataCenter)/m_extend;
	} else{
		dataTop=m_dataAverage+(dataTop-dataCenter)/m_extend;
		dataBottom=m_dataAverage+(dataBottom-dataCenter)/m_extend;
	}
	//折れ線の描画
	for(size_t j=0,size=m_graphData.size();j<size;j++){
		//グラフの色の設定
		unsigned int color=GetColor(64*(j%4)+63,64*((j*3%16)/4)+63,64*((j*5%64)/16)+63);
		//色一覧に描画
		DrawBox(graphPos.x+graphSize.x+20,330+j*20,graphPos.x+graphSize.x+20+60,330+j*20+10,color,TRUE);
		//1フレームに対するピクセル数の計算
		double frameRateToPixel;
		if(m_widthUnity){
			frameRateToPixel=((double)writeCountMax)/m_graphData[j].m_data.size();
		} else{
			frameRateToPixel=((double)writeCountMax)/m_dataSizeMax;
		}
		//折れ線の描画
		for(size_t i=0,datanum=m_graphData[j].m_data.size();i<datanum;i++){
			DrawCircle(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphData[j].m_data[i])),1,color,TRUE);
		}
		for(size_t i=0,datanum=m_graphData[j].m_data.size();i+1<datanum;i++){
			DrawLine(graphPos.x+(int)(i*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphData[j].m_data[i])),graphPos.x+(int)((i+1)*frameRateToPixel),graphPos.y+(int)(graphSize.y/std::fmax(dataTop-dataBottom,0.00001)*(dataTop-m_graphData[j].m_data[i+1])),color,1);
		}
	}
	//data最大値の表示
	DrawLine(graphPos.x,graphPos.y,graphPos.x+graphSize.x,graphPos.y,GetColor(128,128,128),1);
	DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y,std::to_string(dataTop),GetColor(255,255,255),m_font);
	//data最小値の表示
	DrawLine(graphPos.x,graphPos.y+graphSize.y,graphPos.x+graphSize.x,graphPos.y+graphSize.y,GetColor(128,128,128),1);
	DrawStringRightJustifiedToHandle(graphPos.x-5,graphPos.y+graphSize.y,std::to_string(dataBottom),GetColor(255,255,255),m_font);

	//読み込みデータインターフェースの描画
	m_pGraphDataBuilder->Draw();
	//操作説明の描画(横幅等はテキトー、どうせはみださない)
	DrawStringNewLineToHandle(zyPos.x,zyPos.y,0,0,10000,10000,GetColor(255,255,255),m_font,GetFontSizeToHandle(m_font),
		"L click (on body) : set kind of graph\n"
		"U : convert height mode ( unity / normal )\n"
		"I : convert width mode ( unity / normal )\n"
		"Q : reduce extend rate\n"
		"W : reset extend rate\n"
		"E : add extend rate\n"
		"S : output graph\n"
		"A : output files of Input directory to OutputDirectory\n"
		"back : return photographer mode\n"
	);

	//書き出しデータファイル名
	DrawStringToHandle(200,800,(m_playDataName+"_"+m_pGraphDataBuilder->GetFactoryType()+".csv").c_str(),GetColor(255,255,255),m_font);
}


