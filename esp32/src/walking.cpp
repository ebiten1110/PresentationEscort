#include "walking.h"

#include "motion_player.h"
#include "motion_library.h"

// =====================================================
// 歩行状態
// =====================================================

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

// =====================================================
// 内部関数
// =====================================================

static void setWalkingState(WalkingState state) {
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
  playMotion(getTurnLeftMotion());
}

static void playTurnRightMotion() {
  setWalkingState(WALK_TURN_RIGHT);
  playMotion(getTurnRightMotion());
}

// =====================================================
// 公開関数
// =====================================================

void initWalking() {
  walkingState = WALK_IDLE;
  remainingForwardSteps = 0;
  stopRequested = false;

  Serial.println("[Walking] Initialized.");
}

void updateWalking() {
  // まずMotionPlayerを進める
  updateMotionPlayer();

  // モーション再生中なら何もしない
  if (isMotionPlaying()) {
    return;
  }

  // モーションが終わった後の状態遷移
  switch (walkingState) {
    case WALK_STAND:
      setWalkingState(WALK_IDLE);
      break;

    case WALK_START:
      if (stopRequested) {
        playStopWalkMotion();
      } else if (remainingForwardSteps > 0) {
        remainingForwardSteps--;
        playForwardWalkMotion();
      } else {
        playStopWalkMotion();
      }
      break;

    case WALK_FORWARD:
      if (stopRequested) {
        playStopWalkMotion();
      } else if (remainingForwardSteps > 0) {
        remainingForwardSteps--;
        playForwardWalkMotion();
      } else {
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
    Serial.println("[Walking] Busy. Cannot stand now.");
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  playStandMotion();
}

void walkForwardSteps(int steps) {
  if (steps <= 0) {
    Serial.println("[Walking] steps must be greater than 0.");
    return;
  }

  if (isMotionPlaying() || walkingState != WALK_IDLE) {
    Serial.println("[Walking] Busy. Cannot start walking.");
    return;
  }

  stopRequested = false;

  // START_WALK が1歩目に相当するので、
  // 残り歩数は steps - 1 にする
  remainingForwardSteps = steps - 1;

  Serial.print("[Walking] Walk forward steps: ");
  Serial.println(steps);

  playStartWalkMotion();
}

void walkForwardOnce() {
  walkForwardSteps(1);
}

void stopWalking() {
  Serial.println("[Walking] Stop requested.");

  // 再生中なら、今のモーションが終わった後にSTOPへ行く
  if (isMotionPlaying()) {
    stopRequested = true;
    remainingForwardSteps = 0;
    return;
  }

  // 止まっているなら停止モーションだけ再生
  if (walkingState == WALK_IDLE) {
    playStopWalkMotion();
    return;
  }

  stopRequested = true;
  remainingForwardSteps = 0;
}

void turnLeft() {
  if (isMotionPlaying() || walkingState != WALK_IDLE) {
    Serial.println("[Walking] Busy. Cannot turn left.");
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  playTurnLeftMotion();
}

void turnRight() {
  if (isMotionPlaying() || walkingState != WALK_IDLE) {
    Serial.println("[Walking] Busy. Cannot turn right.");
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  playTurnRightMotion();
}

bool isWalkingBusy() {
  return isMotionPlaying() || walkingState != WALK_IDLE;
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