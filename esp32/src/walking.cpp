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
  WALK_TURN_RIGHT,
  WALK_TRACK_LEFT,
  WALK_TRACK_RIGHT
};

static WalkingState walkingState = WALK_IDLE;

static int remainingForwardSteps = 0;
static bool stopRequested = false;


// =====================================================
// 内部関数
// =====================================================

static void setWalkingState(
  WalkingState state
) {
  walkingState = state;

  Serial.print("[Walking] State: ");
  Serial.println(getWalkingStateName());
}


static bool isTrackingTurnState() {
  return (
    walkingState == WALK_TRACK_LEFT
    || walkingState == WALK_TRACK_RIGHT
  );
}


static void playStandMotion() {
  setWalkingState(WALK_STAND);
  Serial.println("[Walking] Motion=STAND");
  playMotion(getStandMotion());
}


static void playStartWalkMotion() {
  setWalkingState(WALK_START);
  Serial.println("[Walking] Motion=START_WALK");
  playMotion(getStartWalkMotion());
}


static void playForwardWalkMotion() {
  setWalkingState(WALK_FORWARD);
  Serial.println("[Walking] Motion=FORWARD_WALK");
  playMotion(getForwardWalkMotion());
}


static void playStopWalkMotion() {
  setWalkingState(WALK_STOP);
  Serial.println("[Walking] Motion=STOP_WALK");
  playMotion(getStopWalkMotion());
}


static void playTurnLeftMotion() {
  setWalkingState(WALK_TURN_LEFT);

  Serial.println("[Walking] Direction=LEFT");
  Serial.println("[Walking] TurnMode=MANUAL_V7");
  Serial.println(
    "[Walking] Motion="
    "TURN_LEFT_V7_TWO_STAGE_RECENTER"
  );

  playMotion(getTurnLeftMotion());
}


static void playTurnRightMotion() {
  setWalkingState(WALK_TURN_RIGHT);

  Serial.println("[Walking] Direction=RIGHT");
  Serial.println("[Walking] TurnMode=MANUAL_V7");
  Serial.println(
    "[Walking] Motion="
    "TURN_RIGHT_V7_RIGHT_FOOT_LIFT_BOOST"
  );

  playMotion(getTurnRightMotion());
}


static void playTrackLeftMotion() {
  setWalkingState(WALK_TRACK_LEFT);

  Serial.println("[Walking] Direction=LEFT");
  Serial.println("[Walking] TurnMode=AUTO_MICRO_V8");
  Serial.println("[Walking] Interruptible=YES");
  Serial.println(
    "[Walking] Motion=TRACK_LEFT_V8_MICRO"
  );

  playMotion(getTrackLeftMotion());
}


static void playTrackRightMotion() {
  setWalkingState(WALK_TRACK_RIGHT);

  Serial.println("[Walking] Direction=RIGHT");
  Serial.println("[Walking] TurnMode=AUTO_MICRO_V8");
  Serial.println("[Walking] Interruptible=YES");
  Serial.println(
    "[Walking] Motion=TRACK_RIGHT_V8_MICRO"
  );

  playMotion(getTrackRightMotion());
}


// =====================================================
// 公開関数
// =====================================================

void initWalking() {
  walkingState = WALK_IDLE;
  remainingForwardSteps = 0;
  stopRequested = false;

  Serial.println("[Walking] Initialized.");
  Serial.println(
    "[Walking] ManualTurnVersion="
    "V7_RIGHT_FOOT_LIFT_BOOST"
  );
  Serial.println(
    "[Walking] AutoTurnVersion="
    "V8_MICRO_CENTER_STOP"
  );
  Serial.println(
    "[Walking] CommandInversion=DISABLED"
  );
}


void updateWalking() {
  // MotionPlayerはここでだけ進める。
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
      Serial.println("[Walking] LEFT turn finished.");
      setWalkingState(WALK_IDLE);
      break;

    case WALK_TURN_RIGHT:
      Serial.println("[Walking] RIGHT turn finished.");
      setWalkingState(WALK_IDLE);
      break;

    case WALK_TRACK_LEFT:
      Serial.println(
        "[Walking] TRACK_LEFT pulse finished."
      );
      setWalkingState(WALK_IDLE);
      break;

    case WALK_TRACK_RIGHT:
      Serial.println(
        "[Walking] TRACK_RIGHT pulse finished."
      );
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

  // 自動追従の小刻み旋回だけは、顔が中央へ来た時点で
  // モーションを直ちに止め、現在角度からSTANDへ戻す。
  if (
    isMotionPlaying()
    && isTrackingTurnState()
  ) {
    Serial.println(
      "[Walking] Immediate auto-track stop."
    );

    stopMotion();

    stopRequested = false;
    remainingForwardSteps = 0;

    playStandMotion();
    return;
  }

  // 手動V7旋回や歩行は転倒防止のため途中中断しない。
  if (isMotionPlaying()) {
    stopRequested = true;
    remainingForwardSteps = 0;
    return;
  }

  // 待機状態では不要な停止モーションを開始しない。
  if (walkingState == WALK_IDLE) {
    stopRequested = false;
    remainingForwardSteps = 0;

    Serial.println("[Walking] Already idle.");
    return;
  }

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

  Serial.println("[Walking] turnLeft() accepted.");
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

  Serial.println("[Walking] turnRight() accepted.");
  playTurnRightMotion();
}


void trackLeft() {
  if (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  ) {
    Serial.println(
      "[Walking] Busy. Cannot track left."
    );
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  Serial.println("[Walking] trackLeft() accepted.");
  playTrackLeftMotion();
}


void trackRight() {
  if (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  ) {
    Serial.println(
      "[Walking] Busy. Cannot track right."
    );
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;

  Serial.println("[Walking] trackRight() accepted.");
  playTrackRightMotion();
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

    case WALK_TRACK_LEFT:
      return "TRACK_LEFT";

    case WALK_TRACK_RIGHT:
      return "TRACK_RIGHT";

    default:
      return "UNKNOWN";
  }
}
