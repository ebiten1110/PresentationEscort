#include "walking.h"

#include "motion_player.h"
#include "motion_library.h"


// ============================================================
// 歩行状態
// ============================================================

enum WalkingState {
  WALK_IDLE,
  WALK_STAND,
  WALK_START,
  WALK_FORWARD,
  WALK_STOP,
  WALK_TURN_LEFT,
  WALK_TURN_RIGHT
};

static WalkingState walkingState = WALK_IDLE;

// 前進の残り回数
static int remainingForwardSteps = 0;

// 停止要求
static bool stopRequested = false;


// ============================================================
// 旋回方向調整
// ============================================================

// シリアルモニターからLEFTを直接送っても右へ旋回する場合は、
// trueへ変更する。
// Python側の映像反転が原因の場合はfalseのままにする。
static const bool INVERT_TURN_MOTION = false;


// ============================================================
// 内部関数
// ============================================================

static void setWalkingState(
  WalkingState state
) {
  walkingState = state;

  Serial.print("[Walking] State: ");
  Serial.println(getWalkingStateName());
}


static void playStandMotion() {
  setWalkingState(WALK_STAND);
  playMotion(getStandMotion());
}


static void playStartWalkMotion() {
  setWalkingState(WALK_START);
  playMotion(getStartWalkMotion());
}


static void playForwardWalkMotion() {
  setWalkingState(WALK_FORWARD);
  playMotion(getForwardWalkMotion());
}


static void playStopWalkMotion() {
  setWalkingState(WALK_STOP);
  playMotion(getStopWalkMotion());
}


static void playTurnLeftMotion() {
  setWalkingState(WALK_TURN_LEFT);

  if (INVERT_TURN_MOTION) {
    playMotion(getTurnRightMotion());
  }
  else {
    playMotion(getTurnLeftMotion());
  }
}


static void playTurnRightMotion() {
  setWalkingState(WALK_TURN_RIGHT);

  if (INVERT_TURN_MOTION) {
    playMotion(getTurnLeftMotion());
  }
  else {
    playMotion(getTurnRightMotion());
  }
}


// ============================================================
// 公開関数
// ============================================================

void initWalking() {
  walkingState = WALK_IDLE;
  remainingForwardSteps = 0;
  stopRequested = false;

  Serial.println("[Walking] Initialized.");
}


void updateWalking() {
  // MotionPlayerはここで1回だけ更新する。
  updateMotionPlayer();

  if (isMotionPlaying()) {
    return;
  }

  switch (walkingState) {
    case WALK_STAND:
      setWalkingState(WALK_IDLE);
      break;

    case WALK_START:
      if (stopRequested) {
        playStopWalkMotion();
      }
      else if (remainingForwardSteps > 0) {
        remainingForwardSteps--;
        playForwardWalkMotion();
      }
      else {
        playStopWalkMotion();
      }
      break;

    case WALK_FORWARD:
      if (stopRequested) {
        playStopWalkMotion();
      }
      else if (remainingForwardSteps > 0) {
        remainingForwardSteps--;
        playForwardWalkMotion();
      }
      else {
        playStopWalkMotion();
      }
      break;

    case WALK_STOP:
      stopRequested = false;
      remainingForwardSteps = 0;
      setWalkingState(WALK_IDLE);
      break;

    case WALK_TURN_LEFT:
      setWalkingState(WALK_IDLE);
      break;

    case WALK_TURN_RIGHT:
      setWalkingState(WALK_IDLE);
      break;

    case WALK_IDLE:
    default:
      break;
  }
}


void stand() {
  if (isMotionPlaying()) {
    Serial.println(
      "[Walking] Busy. Cannot stand now."
    );
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  playStandMotion();
}


void walkForwardSteps(int steps) {
  if (steps <= 0) {
    Serial.println(
      "[Walking] steps must be greater than 0."
    );
    return;
  }

  if (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  ) {
    Serial.println(
      "[Walking] Busy. Cannot start walking."
    );
    return;
  }

  stopRequested = false;

  // START_WALKが1歩目に相当する。
  remainingForwardSteps = steps - 1;

  Serial.print(
    "[Walking] Walk forward steps: "
  );
  Serial.println(steps);

  playStartWalkMotion();
}


void walkForwardOnce() {
  walkForwardSteps(1);
}


void stopWalking() {
  Serial.println("[Walking] Stop requested.");

  // すでに停止中・待機中なら、
  // 長いSTOP_WALKモーションを新しく開始しない。
  if (
    !isMotionPlaying()
    && walkingState == WALK_IDLE
  ) {
    stopRequested = false;
    remainingForwardSteps = 0;

    Serial.println(
      "[Walking] Already idle."
    );
    return;
  }

  // 再生中なら、現在モーション終了後に停止する。
  if (isMotionPlaying()) {
    stopRequested = true;
    remainingForwardSteps = 0;
    return;
  }

  // IDLE以外でモーションが止まっている場合。
  stopRequested = true;
  remainingForwardSteps = 0;
  playStopWalkMotion();
}


void turnLeft() {
  if (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  ) {
    Serial.println(
      "[Walking] Busy. Cannot turn left."
    );
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  playTurnLeftMotion();
}


void turnRight() {
  if (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  ) {
    Serial.println(
      "[Walking] Busy. Cannot turn right."
    );
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  playTurnRightMotion();
}


bool isWalkingBusy() {
  return (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  );
}


const char* getWalkingStateName() {
  switch (walkingState) {
    case WALK_IDLE:
      return "IDLE";

    case WALK_STAND:
      return "STAND";

    case WALK_START:
      return "START_WALK";

    case WALK_FORWARD:
      return "FORWARD_WALK";

    case WALK_STOP:
      return "STOP_WALK";

    case WALK_TURN_LEFT:
      return "TURN_LEFT";

    case WALK_TURN_RIGHT:
      return "TURN_RIGHT";

    default:
      return "UNKNOWN";
  }
}
