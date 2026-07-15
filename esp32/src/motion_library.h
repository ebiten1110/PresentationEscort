#ifndef MOTION_LIBRARY_H
#define MOTION_LIBRARY_H

#include <Arduino.h>

// =====================================================
// MotionStep
// wait     : ひとつ前のステップ開始から何ms後に開始するか
// ch       : 動かすサーボのPCA9685チャンネル
// angle    : 目標角度
// duration : その角度まで何msかけて動かすか
// =====================================================
struct MotionStep {
  int wait;
  int ch;
  double angle;
  int duration;
};

// =====================================================
// MotionData
// name   : モーション名
// steps  : MotionStep配列
// length : ステップ数
// =====================================================
struct MotionData {
  const char* name;
  const MotionStep* steps;
  int length;
};

// モーション取得関数
const MotionData& getStandMotion();
const MotionData& getStartWalkMotion();
const MotionData& getForwardWalkMotion();
const MotionData& getStopWalkMotion();
const MotionData& getTurnLeftMotion();
const MotionData& getTurnRightMotion();

#endif