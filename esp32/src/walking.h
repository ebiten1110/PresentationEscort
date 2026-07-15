#ifndef WALKING_H
#define WALKING_H

#include <Arduino.h>

// 歩行制御の初期化
void initWalking();

// loop() 内で毎回呼ぶ
void updateWalking();

// 直立姿勢
void stand();

// 指定回数だけ前進する
void walkForwardSteps(int steps);

// 1回だけ前進する
void walkForwardOnce();

// 停止モーションへ移行する
void stopWalking();

// 左旋回
void turnLeft();

// 右旋回
void turnRight();

// 歩行・旋回・停止モーション中かどうか
bool isWalkingBusy();

// 現在の歩行状態名を返す
const char* getWalkingStateName();

#endif