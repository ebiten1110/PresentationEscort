#ifndef MOTION_PLAYER_H
#define MOTION_PLAYER_H

#include <Arduino.h>
#include "motion_library.h"

// モーション再生の初期化
void initMotionPlayer();

// 指定したモーションを再生開始する
void playMotion(const MotionData& motion);

// loop() 内で毎回呼ぶ
// 再生中のモーションを少しずつ進める
void updateMotionPlayer();

// モーション再生中かどうか
bool isMotionPlaying();

// 現在のモーションを停止する
void stopMotion();

// 現在再生中のモーション名を返す
const char* getCurrentMotionName();

#endif