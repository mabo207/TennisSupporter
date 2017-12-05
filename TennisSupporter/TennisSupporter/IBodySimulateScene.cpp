#include"IBodySimulateScene.h"

//----------------------IBodySimulateScene----------------------
const int IBodySimulateScene::writeCountMax=30*30;
const int IBodySimulateScene::captureFps=30;
const int IBodySimulateScene::drawFps=60;
const Vector2D IBodySimulateScene::kinectSize=Vector2D(512,424);

const IBodySimulateScene::MODE::TYPE IBodySimulateScene::MODE::link(int num){
	if(num>=0 || num<END){
		return static_cast<TYPE>(num);
	}
	return END;
}
