#include "walking.h"

#include "motion_player.h"
#include "motion_library.h"


enum WalkingState {
  WALK_IDLE,
  WALK_STAND,
  WALK_START,
  WALK_FORWARD,
  WALK_STOP,
  WALK_TURN_LEFT,
  WALK_TURN_RIGHT,
  WALK_TRACK_LEFT,
  WALK_TRACK_RIGHT,
  WALK_TRACK_FORWARD,
  WALK_TRACK_BACKWARD
};

static WalkingState walkingState = WALK_IDLE;

static int remainingForwardSteps = 0;
static bool stopRequested = false;


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


static bool isDistancePulseState() {
  return (
    walkingState == WALK_TRACK_FORWARD
    || walkingState == WALK_TRACK_BACKWARD
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
  Serial.println("[Walking] Interruptible=IMMEDIATE");
  Serial.println(
    "[Walking] Motion=TRACK_LEFT_V8_MICRO"
  );
  playMotion(getTrackLeftMotion());
}


static void playTrackRightMotion() {
  setWalkingState(WALK_TRACK_RIGHT);
  Serial.println("[Walking] Direction=RIGHT");
  Serial.println("[Walking] TurnMode=AUTO_MICRO_V8");
  Serial.println("[Walking] Interruptible=IMMEDIATE");
  Serial.println(
    "[Walking] Motion=TRACK_RIGHT_V8_MICRO"
  );
  playMotion(getTrackRightMotion());
}


static void playTrackForwardMotion() {
  setWalkingState(WALK_TRACK_FORWARD);
  Serial.println("[Walking] Direction=FORWARD");
  Serial.println("[Walking] DistanceMode=AUTO_MICRO_V8_3");
  Serial.println("[Walking] Interruptible=AFTER_CURRENT_PULSE");
  Serial.println(
    "[Walking] Motion=TRACK_FORWARD_V8_3_MICRO"
  );
  playMotion(getTrackForwardMotion());
}


static void playTrackBackwardMotion() {
  setWalkingState(WALK_TRACK_BACKWARD);
  Serial.println("[Walking] Direction=BACKWARD");
  Serial.println("[Walking] DistanceMode=AUTO_MICRO_V8_3");
  Serial.println("[Walking] Interruptible=AFTER_CURRENT_PULSE");
  Serial.println(
    "[Walking] Motion=TRACK_BACKWARD_V8_3_MICRO"
  );
  playMotion(getTrackBackwardMotion());
}


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
    "[Walking] DistanceMoveVersion="
    "V8_3_MICRO_FORWARD_BACKWARD"
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
      stopRequested = false;
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

    case WALK_TRACK_FORWARD:
      Serial.println(
        "[Walking] TRACK_FORWARD pulse finished."
      );
      stopRequested = false;
      playStandMotion();
      break;

    case WALK_TRACK_BACKWARD:
      Serial.println(
        "[Walking] TRACK_BACKWARD pulse finished."
      );
      stopRequested = false;
      playStandMotion();
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

  // 左右の自動旋回は、中央到達時に直ちに止める。
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

  // 距離移動は転倒を避けるため、現在の短い1パルスを
  // 最後まで終えてからSTANDへ戻す。
  if (
    isMotionPlaying()
    && isDistancePulseState()
  ) {
    Serial.println(
      "[Walking] Distance stop queued."
    );

    stopRequested = true;
    remainingForwardSteps = 0;
    return;
  }

  if (isMotionPlaying()) {
    stopRequested = true;
    remainingForwardSteps = 0;
    return;
  }

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


void trackForward() {
  if (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  ) {
    Serial.println(
      "[Walking] Busy. Cannot track forward."
    );
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;
  Serial.println("[Walking] trackForward() accepted.");
  playTrackForwardMotion();
}


void trackBackward() {
  if (
    isMotionPlaying()
    || walkingState != WALK_IDLE
  ) {
    Serial.println(
      "[Walking] Busy. Cannot track backward."
    );
    return;
  }

  stopRequested = false;
  remainingForwardSteps = 0;
  Serial.println("[Walking] trackBackward() accepted.");
  playTrackBackwardMotion();
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

    case WALK_TRACK_FORWARD:
      return "TRACK_FORWARD";

    case WALK_TRACK_BACKWARD:
      return "TRACK_BACKWARD";

    default:
      return "UNKNOWN";
  }
}
