#include "motion_library.h"
#include "config.h"

// =====================================================
// 直立姿勢
// =====================================================
static const MotionStep standSteps[] = {
  { 0, CH_LEFT_HIP,   0, 300 },
  { 0, CH_LEFT_LEG,   0, 300 },
  { 0, CH_RIGHT_HIP,  0, 300 },
  { 0, CH_RIGHT_LEG,  0, 300 }
};

static const MotionData standMotion = {
  "STAND",
  standSteps,
  sizeof(standSteps) / sizeof(MotionStep)
};

// =====================================================
// 歩き始め：左足を一歩出す動き
// 学校資料の startMove をベースにした安全寄り設定
// =====================================================
static const MotionStep startWalkSteps[] = {
  // 体を傾けて左足を浮かせる
  { 0,   CH_RIGHT_LEG, -12, 300 },
  { 0,   CH_LEFT_LEG,  -12, 300 },

  // 腰を回して左足を前に出す
  { 800, CH_LEFT_HIP,  -20, 400 },
  { 0,   CH_RIGHT_HIP, -20, 400 },

  // 足を下ろす
  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 }
};

static const MotionData startWalkMotion = {
  "START_WALK",
  startWalkSteps,
  sizeof(startWalkSteps) / sizeof(MotionStep)
};

// =====================================================
// 前進：右足→左足を交互に出す動き
// =====================================================
static const MotionStep forwardWalkSteps[] = {
  // 右足側を出す準備
  { 800, CH_RIGHT_LEG, 12, 300 },
  { 0,   CH_LEFT_LEG,  12, 300 },

  // 腰を前方向へ
  { 800, CH_LEFT_HIP,  20, 400 },
  { 0,   CH_RIGHT_HIP, 20, 400 },

  // 足を下ろす
  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 },

  // 左足側を出す準備
  { 800, CH_RIGHT_LEG, -12, 300 },
  { 0,   CH_LEFT_LEG,  -12, 300 },

  // 腰を逆方向へ
  { 800, CH_LEFT_HIP,  -20, 400 },
  { 0,   CH_RIGHT_HIP, -20, 400 },

  // 足を下ろす
  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 }
};

static const MotionData forwardWalkMotion = {
  "FORWARD_WALK",
  forwardWalkSteps,
  sizeof(forwardWalkSteps) / sizeof(MotionStep)
};

// =====================================================
// 停止：足を揃えて直立姿勢へ戻す
// =====================================================
static const MotionStep stopWalkSteps[] = {
  // 足を軽く持ち上げる
  { 0,   CH_RIGHT_LEG, 12, 300 },
  { 0,   CH_LEFT_LEG,  12, 300 },

  // 腰を中央へ戻す
  { 800, CH_LEFT_HIP,  0, 300 },
  { 0,   CH_RIGHT_HIP, 0, 300 },

  // 足を下ろす
  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 },

  // 完全に直立
  { 500, CH_LEFT_HIP,   0, 300 },
  { 0,   CH_LEFT_LEG,   0, 300 },
  { 0,   CH_RIGHT_HIP,  0, 300 },
  { 0,   CH_RIGHT_LEG,  0, 300 }
};

static const MotionData stopWalkMotion = {
  "STOP_WALK",
  stopWalkSteps,
  sizeof(stopWalkSteps) / sizeof(MotionStep)
};

// =====================================================
// 左旋回
// 最初は小さめ角度。倒れそうなら角度をさらに小さくする。
// =====================================================
static const MotionStep turnLeftSteps[] = {
  // 軽く体重移動
  { 0,   CH_RIGHT_LEG, -10, 300 },
  { 0,   CH_LEFT_LEG,  -10, 300 },

  // 腰を左旋回方向へ
  { 700, CH_LEFT_HIP,  -15, 350 },
  { 0,   CH_RIGHT_HIP,  15, 350 },

  // 足を戻す
  { 700, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 },

  // 腰を中央へ
  { 500, CH_LEFT_HIP,  0, 300 },
  { 0,   CH_RIGHT_HIP, 0, 300 }
};

static const MotionData turnLeftMotion = {
  "TURN_LEFT",
  turnLeftSteps,
  sizeof(turnLeftSteps) / sizeof(MotionStep)
};

// =====================================================
// 右旋回
// =====================================================
static const MotionStep turnRightSteps[] = {
  // 軽く体重移動
  { 0,   CH_RIGHT_LEG, 10, 300 },
  { 0,   CH_LEFT_LEG,  10, 300 },

  // 腰を右旋回方向へ
  { 700, CH_LEFT_HIP,   15, 350 },
  { 0,   CH_RIGHT_HIP, -15, 350 },

  // 足を戻す
  { 700, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 },

  // 腰を中央へ
  { 500, CH_LEFT_HIP,  0, 300 },
  { 0,   CH_RIGHT_HIP, 0, 300 }
};

static const MotionData turnRightMotion = {
  "TURN_RIGHT",
  turnRightSteps,
  sizeof(turnRightSteps) / sizeof(MotionStep)
};

// =====================================================
// getter
// =====================================================
const MotionData& getStandMotion() {
  return standMotion;
}

const MotionData& getStartWalkMotion() {
  return startWalkMotion;
}

const MotionData& getForwardWalkMotion() {
  return forwardWalkMotion;
}

const MotionData& getStopWalkMotion() {
  return stopWalkMotion;
}

const MotionData& getTurnLeftMotion() {
  return turnLeftMotion;
}

const MotionData& getTurnRightMotion() {
  return turnRightMotion;
}