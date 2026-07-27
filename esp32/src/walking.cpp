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
  Serial.println("[Walking] TurnMethod=TWO_STAGE_RECENTER");

  Serial.println("[Walking] PhaseA.SupportFoot=LEFT");
  Serial.println("[Walking] PhaseA.LiftedFoot=RIGHT");

  Serial.println("[Walking] PhaseB.SupportFoot=RIGHT");
  Serial.println("[Walking] PhaseB.LiftedFoot=LEFT");

  Serial.println(
    "[Walking] RecenterHipsWhileFootRaised=YES"
  );

  Serial.println(
    "[Walking] Motion="
    "TURN_LEFT_V7_TWO_STAGE_RECENTER"
  );

  playMotion(getTurnLeftMotion());
}


static void playTurnRightMotion() {
  setWalkingState(WALK_TURN_RIGHT);

  Serial.println("[Walking] Direction=RIGHT");
  Serial.println("[Walking] TurnMethod=TWO_STAGE_RECENTER");

  Serial.println("[Walking] PhaseA.SupportFoot=RIGHT");
  Serial.println("[Walking] PhaseA.LiftedFoot=LEFT");

  Serial.println("[Walking] PhaseB.SupportFoot=LEFT");
  Serial.println("[Walking] PhaseB.LiftedFoot=RIGHT");

  Serial.println(
    "[Walking] RecenterHipsWhileFootRaised=YES"
  );

  Serial.println(
    "[Walking] RightRecenterLift=14_TO_22_DEG"
  );

  Serial.println(
    "[Walking] RightFootLowering=22_TO_8_TO_0_DEG"
  );

  Serial.println(
    "[Walking] Motion="
    "TURN_RIGHT_V7_RIGHT_FOOT_LIFT_BOOST"
  );

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

  Serial.println(
    "[Walking] TurnMotionVersion="
    "V7_RIGHT_FOOT_LIFT_BOOST"
  );

  Serial.println(
    "[Walking] CommandInversion=DISABLED"
  );
}


void updateWalking() {
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

  if (
    !isMotionPlaying()
    && walkingState == WALK_IDLE
  ) {
    stopRequested = false;
    remainingForwardSteps = 0;
    Serial.println("[Walking] Already idle.");
    return;
  }

  if (isMotionPlaying()) {
    stopRequested = true;
    remainingForwardSteps = 0;
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