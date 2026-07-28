#include "motion_library.h"
#include "config.h"


// =====================================================
// 共通設定
// =====================================================

static constexpr double NEUTRAL_ANGLE = 0.0;


// =====================================================
// 二段階ピボット旋回 V7
// =====================================================
//
// 旋回後に足裏の輪ゴムが床へ引っかかり、脚と腰が
// 正位置へ戻りにくい問題への対策。
//
// 重要:
// 最初に浮かせた足を上げたまま両腰を0度へ戻すと、
// 浮かせた足も元の位置へ戻り、旋回量が消えやすい。
//
// そこで次の順番にする。
//
// 1. 最初の軸足へ体重を乗せる。
// 2. 反対側の足を浮かせて旋回位置へ送る。
// 3. 送った足を一度着地させる。
// 4. 今度は着地した足へ体重を移す。
// 5. 最初の軸足を浮かせる。
// 6. 最初の軸足が浮いた状態で両腰を0度へ戻す。
// 7. 最初の軸足を下ろして直立する。
//
// これにより、輪ゴムを床へ擦りながら腰を戻さずに済む。
// LEFT/RIGHTの命令入れ替えは行わない。
// =====================================================


// =====================================================
// 左旋回の調整値
// =====================================================
//
// 最初:
//   軸足   = 左
//   浮かす = 右
//
// 腰を戻す段階:
//   軸足   = 右
//   浮かす = 左
// =====================================================

static constexpr double LEFT_FIRST_SHIFT_ANGLE = 18.0;

static constexpr double LEFT_SUPPORT_HIP_ANGLE = 5.0;
static constexpr double LEFT_SWING_HIP_ANGLE = -38.0;

// 右足を着地した後、右足へ体重を移して左足を浮かせる。
static constexpr double LEFT_RECENTER_SHIFT_ANGLE = -12.0;


// =====================================================
// 右旋回の調整値
// =====================================================
//
// 最初:
//   軸足   = 右
//   浮かす = 左
//
// 腰を戻す段階:
//   軸足   = 左
//   浮かす = 右
// =====================================================

static constexpr double RIGHT_FIRST_SHIFT_ANGLE = -12.0;

static constexpr double RIGHT_SWING_HIP_ANGLE = 24.0;
static constexpr double RIGHT_SUPPORT_HIP_ANGLE = -7.0;

// 左足を着地した後、左足へ体重を移して右足を浮かせる。
// V6の+12度では右足が上がり切らなかったため、
// +14度から+22度へ二段階で持ち上げる。
static constexpr double RIGHT_RECENTER_SHIFT_STAGE1_ANGLE = 14.0;
static constexpr double RIGHT_RECENTER_SHIFT_STAGE2_ANGLE = 22.0;

// 腰を戻した後、右足をいきなり下ろさず、
// 一度+8度まで下げてから0度へ戻す。
static constexpr double RIGHT_RECENTER_PRELOWER_ANGLE = 8.0;


// =====================================================
// 動作時間
// =====================================================

static constexpr int CENTER_DURATION_MS = 400;
static constexpr int FIRST_SHIFT_DURATION_MS = 550;
static constexpr int SWING_DURATION_MS = 650;
static constexpr int FIRST_LOWER_DURATION_MS = 550;
static constexpr int RECENTER_SHIFT_DURATION_MS = 550;

// RIGHTの復帰用。右足を確実に浮かせるための2段階目。
static constexpr int RIGHT_RECENTER_BOOST_DURATION_MS = 500;

static constexpr int HIP_RECENTER_DURATION_MS = 750;

// 腰復帰後、足を段階的に下ろす。
static constexpr int RIGHT_PRELOWER_DURATION_MS = 450;
static constexpr int FINAL_LOWER_DURATION_MS = 600;
static constexpr int FINAL_HOLD_DURATION_MS = 400;


// =====================================================
// 次段階までの待ち時間
//
// MotionPlayerではwaitが直前ステップの開始から数えられる。
// 前段階のdurationより大きな値にする。
// =====================================================

static constexpr int WAIT_AFTER_CENTER_MS = 600;
static constexpr int WAIT_AFTER_FIRST_SHIFT_MS = 850;
static constexpr int WAIT_AFTER_SWING_MS = 950;
static constexpr int WAIT_AFTER_FIRST_LOWER_MS = 850;
static constexpr int WAIT_AFTER_RECENTER_SHIFT_MS = 850;

// RIGHTの+22度までの持ち上げ完了と保持。
static constexpr int WAIT_AFTER_RIGHT_RECENTER_BOOST_MS = 800;

static constexpr int WAIT_AFTER_HIP_RECENTER_MS = 1050;

// RIGHTの+8度までの下降完了と保持。
static constexpr int WAIT_AFTER_RIGHT_PRELOWER_MS = 700;

static constexpr int WAIT_AFTER_FINAL_LOWER_MS = 850;


// =====================================================
// 直立姿勢
// =====================================================

static const MotionStep standSteps[] = {
  { 0, CH_LEFT_HIP,  0, 400 },
  { 0, CH_LEFT_LEG,  0, 400 },
  { 0, CH_RIGHT_HIP, 0, 400 },
  { 0, CH_RIGHT_LEG, 0, 400 }
};

static const MotionData standMotion = {
  "STAND",
  standSteps,
  sizeof(standSteps) / sizeof(MotionStep)
};


// =====================================================
// 歩き始め
// =====================================================

static const MotionStep startWalkSteps[] = {
  { 0,   CH_RIGHT_LEG, -12, 300 },
  { 0,   CH_LEFT_LEG,  -12, 300 },

  { 800, CH_LEFT_HIP,  -20, 400 },
  { 0,   CH_RIGHT_HIP, -20, 400 },

  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 }
};

static const MotionData startWalkMotion = {
  "START_WALK",
  startWalkSteps,
  sizeof(startWalkSteps) / sizeof(MotionStep)
};


// =====================================================
// 前進
// =====================================================

static const MotionStep forwardWalkSteps[] = {
  { 800, CH_RIGHT_LEG, 12, 300 },
  { 0,   CH_LEFT_LEG,  12, 300 },

  { 800, CH_LEFT_HIP,  20, 400 },
  { 0,   CH_RIGHT_HIP, 20, 400 },

  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 },

  { 800, CH_RIGHT_LEG, -12, 300 },
  { 0,   CH_LEFT_LEG,  -12, 300 },

  { 800, CH_LEFT_HIP,  -20, 400 },
  { 0,   CH_RIGHT_HIP, -20, 400 },

  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 }
};

static const MotionData forwardWalkMotion = {
  "FORWARD_WALK",
  forwardWalkSteps,
  sizeof(forwardWalkSteps) / sizeof(MotionStep)
};


// =====================================================
// 停止
// =====================================================

static const MotionStep stopWalkSteps[] = {
  { 0,   CH_RIGHT_LEG, 12, 300 },
  { 0,   CH_LEFT_LEG,  12, 300 },

  { 800, CH_LEFT_HIP,  0, 300 },
  { 0,   CH_RIGHT_HIP, 0, 300 },

  { 800, CH_RIGHT_LEG, 0, 300 },
  { 0,   CH_LEFT_LEG,  0, 300 },

  { 500, CH_LEFT_HIP,   0, 400 },
  { 0,   CH_LEFT_LEG,   0, 400 },
  { 0,   CH_RIGHT_HIP,  0, 400 },
  { 0,   CH_RIGHT_LEG,  0, 400 }
};

static const MotionData stopWalkMotion = {
  "STOP_WALK",
  stopWalkSteps,
  sizeof(stopWalkSteps) / sizeof(MotionStep)
};


// =====================================================
// 左旋回 V6
//
// 1. 左足を軸に右足を送る。
// 2. 右足を着地。
// 3. 右足へ体重を移して左足を浮かせる。
// 4. 左足が浮いた状態で腰を0度へ戻す。
// 5. 左足を下ろす。
// =====================================================

static const MotionStep turnLeftSteps[] = {
  // Phase 0: 中央姿勢。
  { 0, CH_LEFT_HIP,  NEUTRAL_ANGLE, CENTER_DURATION_MS },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, CENTER_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, CENTER_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, CENTER_DURATION_MS },

  // Phase 1: 左足へ体重を移して右足を浮かせる。
  {
    WAIT_AFTER_CENTER_MS,
    CH_LEFT_LEG,
    LEFT_FIRST_SHIFT_ANGLE,
    FIRST_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    LEFT_FIRST_SHIFT_ANGLE,
    FIRST_SHIFT_DURATION_MS
  },

  // Phase 2: 左足を軸に右足を左旋回位置へ送る。
  {
    WAIT_AFTER_FIRST_SHIFT_MS,
    CH_LEFT_HIP,
    LEFT_SUPPORT_HIP_ANGLE,
    SWING_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    LEFT_SWING_HIP_ANGLE,
    SWING_DURATION_MS
  },

  // Phase 3: 右足を新しい位置へ着地させる。
  {
    WAIT_AFTER_SWING_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    FIRST_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    FIRST_LOWER_DURATION_MS
  },

  // Phase 4: 右足へ体重を移し、左足を浮かせる。
  {
    WAIT_AFTER_FIRST_LOWER_MS,
    CH_LEFT_LEG,
    LEFT_RECENTER_SHIFT_ANGLE,
    RECENTER_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    LEFT_RECENTER_SHIFT_ANGLE,
    RECENTER_SHIFT_DURATION_MS
  },

  // Phase 5: 左足が浮いたまま、両腰を中央へ戻す。
  {
    WAIT_AFTER_RECENTER_SHIFT_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    HIP_RECENTER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    HIP_RECENTER_DURATION_MS
  },

  // Phase 6: 左足をゆっくり下ろす。
  {
    WAIT_AFTER_HIP_RECENTER_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    FINAL_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    FINAL_LOWER_DURATION_MS
  },

  // Phase 7: 0度を再指示して保持する。
  {
    WAIT_AFTER_FINAL_LOWER_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  },
  {
    0,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  }
};

static const MotionData turnLeftMotion = {
  "TURN_LEFT_V7_TWO_STAGE_RECENTER",
  turnLeftSteps,
  sizeof(turnLeftSteps) / sizeof(MotionStep)
};


// =====================================================
// 右旋回 V6
//
// 1. 右足を軸に左足を送る。
// 2. 左足を着地。
// 3. 左足へ体重を移して右足を浮かせる。
// 4. 右足が浮いた状態で腰を0度へ戻す。
// 5. 右足を下ろす。
// =====================================================

static const MotionStep turnRightSteps[] = {
  // Phase 0: 中央姿勢。
  { 0, CH_LEFT_HIP,  NEUTRAL_ANGLE, CENTER_DURATION_MS },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, CENTER_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, CENTER_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, CENTER_DURATION_MS },

  // Phase 1: 右足へ体重を移して左足を浮かせる。
  {
    WAIT_AFTER_CENTER_MS,
    CH_LEFT_LEG,
    RIGHT_FIRST_SHIFT_ANGLE,
    FIRST_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    RIGHT_FIRST_SHIFT_ANGLE,
    FIRST_SHIFT_DURATION_MS
  },

  // Phase 2: 右足を軸に左足を右旋回位置へ送る。
  {
    WAIT_AFTER_FIRST_SHIFT_MS,
    CH_LEFT_HIP,
    RIGHT_SWING_HIP_ANGLE,
    SWING_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    RIGHT_SUPPORT_HIP_ANGLE,
    SWING_DURATION_MS
  },

  // Phase 3: 左足を新しい位置へ着地させる。
  {
    WAIT_AFTER_SWING_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    FIRST_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    FIRST_LOWER_DURATION_MS
  },

  // Phase 4: 左足へ体重を移し、右足を浮かせ始める。
  {
    WAIT_AFTER_FIRST_LOWER_MS,
    CH_LEFT_LEG,
    RIGHT_RECENTER_SHIFT_STAGE1_ANGLE,
    RECENTER_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    RIGHT_RECENTER_SHIFT_STAGE1_ANGLE,
    RECENTER_SHIFT_DURATION_MS
  },

  // Phase 5: 右足の持ち上げを強化する。
  {
    WAIT_AFTER_RECENTER_SHIFT_MS,
    CH_LEFT_LEG,
    RIGHT_RECENTER_SHIFT_STAGE2_ANGLE,
    RIGHT_RECENTER_BOOST_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    RIGHT_RECENTER_SHIFT_STAGE2_ANGLE,
    RIGHT_RECENTER_BOOST_DURATION_MS
  },

  // Phase 6: 右足が十分に浮いた状態で両腰を中央へ戻す。
  {
    WAIT_AFTER_RIGHT_RECENTER_BOOST_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    HIP_RECENTER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    HIP_RECENTER_DURATION_MS
  },

  // Phase 7: 腰が中央へ戻った後、右足を途中まで下ろす。
  {
    WAIT_AFTER_HIP_RECENTER_MS,
    CH_LEFT_LEG,
    RIGHT_RECENTER_PRELOWER_ANGLE,
    RIGHT_PRELOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    RIGHT_RECENTER_PRELOWER_ANGLE,
    RIGHT_PRELOWER_DURATION_MS
  },

  // Phase 8: 右足をゆっくり0度へ下ろす。
  {
    WAIT_AFTER_RIGHT_PRELOWER_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    FINAL_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    FINAL_LOWER_DURATION_MS
  },

  // Phase 9: 0度を再指示して保持する。
  {
    WAIT_AFTER_FINAL_LOWER_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  },
  {
    0,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    FINAL_HOLD_DURATION_MS
  }
};

static const MotionData turnRightMotion = {
  "TURN_RIGHT_V7_RIGHT_FOOT_LIFT_BOOST",
  turnRightSteps,
  sizeof(turnRightSteps) / sizeof(MotionStep)
};



// =====================================================
// 自動追従専用・小刻み旋回 V8
// =====================================================
//
// 手動LEFT / RIGHTはV7の旋回量を維持する。
// Raspberry Piの自動追従ではTRACK_LEFT / TRACK_RIGHTを使う。
//
// 小刻み旋回は1回あたりの旋回量を小さくし、
// 顔が中央へ入った時点でSTOPを受け取ると、walking.cpp側で
// モーションを中断してSTANDへ移行できる。
// =====================================================


// -----------------------------------------------------
// 小刻み左旋回
// -----------------------------------------------------

static constexpr double TRACK_LEFT_FIRST_SHIFT_ANGLE = 10.0;
static constexpr double TRACK_LEFT_SUPPORT_HIP_ANGLE = 2.0;
static constexpr double TRACK_LEFT_SWING_HIP_ANGLE = -12.0;
static constexpr double TRACK_LEFT_RECENTER_SHIFT_ANGLE = -8.0;


// -----------------------------------------------------
// 小刻み右旋回
// -----------------------------------------------------

static constexpr double TRACK_RIGHT_FIRST_SHIFT_ANGLE = -8.0;
static constexpr double TRACK_RIGHT_SWING_HIP_ANGLE = 9.0;
static constexpr double TRACK_RIGHT_SUPPORT_HIP_ANGLE = -3.0;

// 右足を確実に浮かせて腰を戻すため、二段階で持ち上げる。
static constexpr double TRACK_RIGHT_RECENTER_STAGE1_ANGLE = 14.0;
static constexpr double TRACK_RIGHT_RECENTER_STAGE2_ANGLE = 18.0;
static constexpr double TRACK_RIGHT_PRELOWER_ANGLE = 6.0;


// -----------------------------------------------------
// 小刻み旋回の時間
// -----------------------------------------------------

static constexpr int TRACK_CENTER_DURATION_MS = 200;
static constexpr int TRACK_SHIFT_DURATION_MS = 280;
static constexpr int TRACK_SWING_DURATION_MS = 320;
static constexpr int TRACK_LOWER_DURATION_MS = 280;
static constexpr int TRACK_RECENTER_SHIFT_DURATION_MS = 300;
static constexpr int TRACK_RECENTER_BOOST_DURATION_MS = 280;
static constexpr int TRACK_HIP_RECENTER_DURATION_MS = 380;
static constexpr int TRACK_PRELOWER_DURATION_MS = 260;
static constexpr int TRACK_FINAL_LOWER_DURATION_MS = 320;
static constexpr int TRACK_HOLD_DURATION_MS = 220;

// waitは直前段階のdurationより長くする。
static constexpr int TRACK_WAIT_AFTER_CENTER_MS = 300;
static constexpr int TRACK_WAIT_AFTER_SHIFT_MS = 420;
static constexpr int TRACK_WAIT_AFTER_SWING_MS = 460;
static constexpr int TRACK_WAIT_AFTER_LOWER_MS = 400;
static constexpr int TRACK_WAIT_AFTER_RECENTER_SHIFT_MS = 430;
static constexpr int TRACK_WAIT_AFTER_RECENTER_BOOST_MS = 410;
static constexpr int TRACK_WAIT_AFTER_HIP_RECENTER_MS = 520;
static constexpr int TRACK_WAIT_AFTER_PRELOWER_MS = 380;
static constexpr int TRACK_WAIT_AFTER_FINAL_LOWER_MS = 450;


// =====================================================
// 自動追従用・左へ小刻み旋回
// =====================================================

static const MotionStep trackLeftSteps[] = {
  // Phase 0: 中央姿勢。
  { 0, CH_LEFT_HIP,  NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },

  // Phase 1: 左足を軸にし、右足を少し浮かせる。
  {
    TRACK_WAIT_AFTER_CENTER_MS,
    CH_LEFT_LEG,
    TRACK_LEFT_FIRST_SHIFT_ANGLE,
    TRACK_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    TRACK_LEFT_FIRST_SHIFT_ANGLE,
    TRACK_SHIFT_DURATION_MS
  },

  // Phase 2: 右足側を小さく送る。
  {
    TRACK_WAIT_AFTER_SHIFT_MS,
    CH_LEFT_HIP,
    TRACK_LEFT_SUPPORT_HIP_ANGLE,
    TRACK_SWING_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    TRACK_LEFT_SWING_HIP_ANGLE,
    TRACK_SWING_DURATION_MS
  },

  // Phase 3: 右足を着地。
  {
    TRACK_WAIT_AFTER_SWING_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    TRACK_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    TRACK_LOWER_DURATION_MS
  },

  // Phase 4: 右足へ体重を移して左足を浮かせる。
  {
    TRACK_WAIT_AFTER_LOWER_MS,
    CH_LEFT_LEG,
    TRACK_LEFT_RECENTER_SHIFT_ANGLE,
    TRACK_RECENTER_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    TRACK_LEFT_RECENTER_SHIFT_ANGLE,
    TRACK_RECENTER_SHIFT_DURATION_MS
  },

  // Phase 5: 左足を浮かせたまま腰を中央へ戻す。
  {
    TRACK_WAIT_AFTER_RECENTER_SHIFT_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    TRACK_HIP_RECENTER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    TRACK_HIP_RECENTER_DURATION_MS
  },

  // Phase 6: 左足を下ろす。
  {
    TRACK_WAIT_AFTER_HIP_RECENTER_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    TRACK_FINAL_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    TRACK_FINAL_LOWER_DURATION_MS
  },

  // Phase 7: 中央姿勢を保持。
  {
    TRACK_WAIT_AFTER_FINAL_LOWER_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    TRACK_HOLD_DURATION_MS
  },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, TRACK_HOLD_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, TRACK_HOLD_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, TRACK_HOLD_DURATION_MS }
};

static const MotionData trackLeftMotion = {
  "TRACK_LEFT_V8_MICRO",
  trackLeftSteps,
  sizeof(trackLeftSteps) / sizeof(MotionStep)
};


// =====================================================
// 自動追従用・右へ小刻み旋回
// =====================================================

static const MotionStep trackRightSteps[] = {
  // Phase 0: 中央姿勢。
  { 0, CH_LEFT_HIP,  NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, TRACK_CENTER_DURATION_MS },

  // Phase 1: 右足を軸にし、左足を少し浮かせる。
  {
    TRACK_WAIT_AFTER_CENTER_MS,
    CH_LEFT_LEG,
    TRACK_RIGHT_FIRST_SHIFT_ANGLE,
    TRACK_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    TRACK_RIGHT_FIRST_SHIFT_ANGLE,
    TRACK_SHIFT_DURATION_MS
  },

  // Phase 2: 左足側を小さく送る。
  {
    TRACK_WAIT_AFTER_SHIFT_MS,
    CH_LEFT_HIP,
    TRACK_RIGHT_SWING_HIP_ANGLE,
    TRACK_SWING_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    TRACK_RIGHT_SUPPORT_HIP_ANGLE,
    TRACK_SWING_DURATION_MS
  },

  // Phase 3: 左足を着地。
  {
    TRACK_WAIT_AFTER_SWING_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    TRACK_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    TRACK_LOWER_DURATION_MS
  },

  // Phase 4: 左足へ体重を移して右足を持ち上げる。
  {
    TRACK_WAIT_AFTER_LOWER_MS,
    CH_LEFT_LEG,
    TRACK_RIGHT_RECENTER_STAGE1_ANGLE,
    TRACK_RECENTER_SHIFT_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    TRACK_RIGHT_RECENTER_STAGE1_ANGLE,
    TRACK_RECENTER_SHIFT_DURATION_MS
  },

  // Phase 5: 右足の持ち上げを追加。
  {
    TRACK_WAIT_AFTER_RECENTER_SHIFT_MS,
    CH_LEFT_LEG,
    TRACK_RIGHT_RECENTER_STAGE2_ANGLE,
    TRACK_RECENTER_BOOST_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    TRACK_RIGHT_RECENTER_STAGE2_ANGLE,
    TRACK_RECENTER_BOOST_DURATION_MS
  },

  // Phase 6: 右足を浮かせたまま腰を中央へ戻す。
  {
    TRACK_WAIT_AFTER_RECENTER_BOOST_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    TRACK_HIP_RECENTER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    TRACK_HIP_RECENTER_DURATION_MS
  },

  // Phase 7: 右足を途中まで下ろす。
  {
    TRACK_WAIT_AFTER_HIP_RECENTER_MS,
    CH_LEFT_LEG,
    TRACK_RIGHT_PRELOWER_ANGLE,
    TRACK_PRELOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    TRACK_RIGHT_PRELOWER_ANGLE,
    TRACK_PRELOWER_DURATION_MS
  },

  // Phase 8: 右足を0度へ下ろす。
  {
    TRACK_WAIT_AFTER_PRELOWER_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    TRACK_FINAL_LOWER_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    TRACK_FINAL_LOWER_DURATION_MS
  },

  // Phase 9: 中央姿勢を保持。
  {
    TRACK_WAIT_AFTER_FINAL_LOWER_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    TRACK_HOLD_DURATION_MS
  },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, TRACK_HOLD_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, TRACK_HOLD_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, TRACK_HOLD_DURATION_MS }
};

static const MotionData trackRightMotion = {
  "TRACK_RIGHT_V8_MICRO",
  trackRightSteps,
  sizeof(trackRightSteps) / sizeof(MotionStep)
};


// =====================================================
// 距離追従用・小刻み前進／後退 V8.3
// =====================================================
//
// TRACK_FORWARD / TRACK_BACKWARDは、距離確認用の短い1パルス。
// 連続歩行にはせず、1回終了するたびにRaspberry Pi側で
// 顔の大きさを再判定する。
//
// 後退は前進に対して腰サーボの振り方向を反転している。
// 初回は必ずロボットを手で支え、実機方向を確認する。
// =====================================================

static constexpr double DISTANCE_LEG_SHIFT_ANGLE = 8.0;
static constexpr double DISTANCE_HIP_SWING_ANGLE = 11.0;

static constexpr int DISTANCE_CENTER_DURATION_MS = 250;
static constexpr int DISTANCE_LEG_DURATION_MS = 300;
static constexpr int DISTANCE_HIP_DURATION_MS = 350;
static constexpr int DISTANCE_RETURN_DURATION_MS = 350;
static constexpr int DISTANCE_HOLD_DURATION_MS = 250;

static constexpr int DISTANCE_WAIT_AFTER_CENTER_MS = 350;
static constexpr int DISTANCE_WAIT_AFTER_LEG_MS = 450;
static constexpr int DISTANCE_WAIT_AFTER_HIP_MS = 500;
static constexpr int DISTANCE_WAIT_AFTER_RETURN_MS = 500;


// -----------------------------------------------------
// 小刻み前進
// -----------------------------------------------------

static const MotionStep trackForwardSteps[] = {
  // Phase 0: 中央姿勢。
  { 0, CH_LEFT_HIP,  NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },

  // Phase 1: 一方向へ体重移動。
  {
    DISTANCE_WAIT_AFTER_CENTER_MS,
    CH_LEFT_LEG,
    -DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    -DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },

  // Phase 2: 腰を前進方向へ小さく振る。
  {
    DISTANCE_WAIT_AFTER_LEG_MS,
    CH_LEFT_HIP,
    -DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    -DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },

  // Phase 3: 脚を中央へ戻す。
  {
    DISTANCE_WAIT_AFTER_HIP_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },

  // Phase 4: 反対方向へ体重移動。
  {
    DISTANCE_WAIT_AFTER_RETURN_MS,
    CH_LEFT_LEG,
    DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },

  // Phase 5: 反対側の前進ストローク。
  {
    DISTANCE_WAIT_AFTER_LEG_MS,
    CH_LEFT_HIP,
    DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },

  // Phase 6: 全軸を中央へ戻す。
  {
    DISTANCE_WAIT_AFTER_HIP_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    DISTANCE_WAIT_AFTER_RETURN_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },

  // Phase 7: 中央姿勢を保持。
  {
    DISTANCE_WAIT_AFTER_RETURN_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    DISTANCE_HOLD_DURATION_MS
  },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, DISTANCE_HOLD_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, DISTANCE_HOLD_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, DISTANCE_HOLD_DURATION_MS }
};

static const MotionData trackForwardMotion = {
  "TRACK_FORWARD_V8_3_MICRO",
  trackForwardSteps,
  sizeof(trackForwardSteps) / sizeof(MotionStep)
};


// -----------------------------------------------------
// 小刻み後退
// -----------------------------------------------------

static const MotionStep trackBackwardSteps[] = {
  // Phase 0: 中央姿勢。
  { 0, CH_LEFT_HIP,  NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, DISTANCE_CENTER_DURATION_MS },

  // Phase 1: 一方向へ体重移動。
  {
    DISTANCE_WAIT_AFTER_CENTER_MS,
    CH_LEFT_LEG,
    -DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    -DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },

  // Phase 2: 前進とは逆方向へ腰を振る。
  {
    DISTANCE_WAIT_AFTER_LEG_MS,
    CH_LEFT_HIP,
    DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },

  // Phase 3: 脚を中央へ戻す。
  {
    DISTANCE_WAIT_AFTER_HIP_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },

  // Phase 4: 反対方向へ体重移動。
  {
    DISTANCE_WAIT_AFTER_RETURN_MS,
    CH_LEFT_LEG,
    DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    DISTANCE_LEG_SHIFT_ANGLE,
    DISTANCE_LEG_DURATION_MS
  },

  // Phase 5: 反対側の後退ストローク。
  {
    DISTANCE_WAIT_AFTER_LEG_MS,
    CH_LEFT_HIP,
    -DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    -DISTANCE_HIP_SWING_ANGLE,
    DISTANCE_HIP_DURATION_MS
  },

  // Phase 6: 全軸を中央へ戻す。
  {
    DISTANCE_WAIT_AFTER_HIP_MS,
    CH_LEFT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    0,
    CH_RIGHT_LEG,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    DISTANCE_WAIT_AFTER_RETURN_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },
  {
    0,
    CH_RIGHT_HIP,
    NEUTRAL_ANGLE,
    DISTANCE_RETURN_DURATION_MS
  },

  // Phase 7: 中央姿勢を保持。
  {
    DISTANCE_WAIT_AFTER_RETURN_MS,
    CH_LEFT_HIP,
    NEUTRAL_ANGLE,
    DISTANCE_HOLD_DURATION_MS
  },
  { 0, CH_RIGHT_HIP, NEUTRAL_ANGLE, DISTANCE_HOLD_DURATION_MS },
  { 0, CH_LEFT_LEG,  NEUTRAL_ANGLE, DISTANCE_HOLD_DURATION_MS },
  { 0, CH_RIGHT_LEG, NEUTRAL_ANGLE, DISTANCE_HOLD_DURATION_MS }
};

static const MotionData trackBackwardMotion = {
  "TRACK_BACKWARD_V8_3_MICRO",
  trackBackwardSteps,
  sizeof(trackBackwardSteps) / sizeof(MotionStep)
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

const MotionData& getTrackLeftMotion() {
  return trackLeftMotion;
}

const MotionData& getTrackRightMotion() {
  return trackRightMotion;
}

const MotionData& getTrackForwardMotion() {
  return trackForwardMotion;
}

const MotionData& getTrackBackwardMotion() {
  return trackBackwardMotion;
}