#ifndef DEF_KINECTTOOLS_H
#define DEF_KINECTTOOLS_H

#include<Kinect.h>
#include"ToolsLib.h"
//Kinectアプリケーション内で用いる便利な関数群
//ToolsLib.hのラッパー
//エラーチェックを行う関数
void ErrorCheck(HRESULT hresult,const char *msg)noexcept(false);

#endif // !DEF_KINECTTOOLS_H
#pragma once
