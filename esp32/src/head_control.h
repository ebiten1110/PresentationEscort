#ifndef HEAD_CONTROL_H
#define HEAD_CONTROL_H

#include <Arduino.h>

// 頭サーボ制御の初期化
void initHeadControl();

// 横軸：左右
void headLeft();
void headRight();

// 縦軸：上下
void headUp();
void headDown();

// 横軸・縦軸を中央に戻す
void headCenter();

// 任意角度に動かす
void setHeadYawAngle(double angle);
void setHeadPitchAngle(double angle);

// 現在角度を返す
double getHeadYawAngle();
double getHeadPitchAngle();

#endif