#ifndef MOTION_LIBRARY_H
#define MOTION_LIBRARY_H

#include <Arduino.h>

// =====================================================
// MotionStep
// wait     : ひとつ前のステップ開始から何ms後に開始するか
// ch       : PCA9685チャンネル
// angle    : 目標角度
// duration : 目標角度までの移動時間
// =====================================================

struct MotionStep {
  int wait;
  int ch;
  double angle;
  int duration;
};

// =====================================================
// MotionData
// =====================================================

struct MotionData {
  const char* name;
  const MotionStep* steps;
  int length;
};

// 通常動作
const MotionData& getStandMotion();
const MotionData& getStartWalkMotion();
const MotionData& getForwardWalkMotion();
const MotionData& getStopWalkMotion();

// 手動用V7旋回
const MotionData& getTurnLeftMotion();
const MotionData& getTurnRightMotion();

// Raspberry Pi自動追従用・小刻み旋回
const MotionData& getTrackLeftMotion();
const MotionData& getTrackRightMotion();

#endif
