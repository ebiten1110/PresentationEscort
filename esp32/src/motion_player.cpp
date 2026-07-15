#include "motion_player.h"

#include "servo_control.h"
#include "config.h"

// =====================================================
// 内部状態
// =====================================================

struct ServoState {
  double nowAngle;
  double fromAngle;
  double toAngle;
  int duration;
  int elapsed;
  bool moving;
};

static ServoState servoStates[16];

static const MotionData* currentMotion = nullptr;
static int stepIndex = 0;
static int motionElapsed = 0;
static bool playing = false;

static const int UPDATE_INTERVAL_MS = 10;
static unsigned long lastUpdateTime = 0;

// =====================================================
// 内部関数
// =====================================================

static double clampServoAngle(double angle) {
  if (angle < SERVO_MIN_ANGLE) {
    return SERVO_MIN_ANGLE;
  }

  if (angle > SERVO_MAX_ANGLE) {
    return SERVO_MAX_ANGLE;
  }

  return angle;
}

static void startStep(const MotionStep& step) {
  if (step.ch < 0 || step.ch > 15) {
    Serial.print("[MotionPlayer] Invalid channel: ");
    Serial.println(step.ch);
    return;
  }

  servoStates[step.ch].fromAngle = servoStates[step.ch].nowAngle;
  servoStates[step.ch].toAngle = clampServoAngle(step.angle);
  servoStates[step.ch].duration = max(step.duration, UPDATE_INTERVAL_MS);
  servoStates[step.ch].elapsed = 0;
  servoStates[step.ch].moving = true;

  Serial.print("[MotionPlayer] Step ");
  Serial.print(stepIndex);
  Serial.print(" CH=");
  Serial.print(step.ch);
  Serial.print(" from=");
  Serial.print(servoStates[step.ch].fromAngle);
  Serial.print(" to=");
  Serial.print(servoStates[step.ch].toAngle);
  Serial.print(" duration=");
  Serial.println(servoStates[step.ch].duration);
}

static void updateServoStates() {
  for (int ch = 0; ch < 16; ch++) {
    if (!servoStates[ch].moving) {
      continue;
    }

    ServoState& state = servoStates[ch];

    state.elapsed += UPDATE_INTERVAL_MS;

    if (state.elapsed >= state.duration) {
      state.nowAngle = state.toAngle;
      state.moving = false;
      setServoAngle(ch, state.nowAngle);
      continue;
    }

    double progress = (double)state.elapsed / (double)state.duration;
    double nextAngle =
      state.fromAngle + (state.toAngle - state.fromAngle) * progress;

    state.nowAngle = nextAngle;
    setServoAngle(ch, state.nowAngle);
  }
}

static bool anyServoMoving() {
  for (int ch = 0; ch < 16; ch++) {
    if (servoStates[ch].moving) {
      return true;
    }
  }

  return false;
}

// =====================================================
// 公開関数
// =====================================================

void initMotionPlayer() {
  for (int ch = 0; ch < 16; ch++) {
    servoStates[ch].nowAngle = 0.0;
    servoStates[ch].fromAngle = 0.0;
    servoStates[ch].toAngle = 0.0;
    servoStates[ch].duration = 0;
    servoStates[ch].elapsed = 0;
    servoStates[ch].moving = false;
  }

  currentMotion = nullptr;
  stepIndex = 0;
  motionElapsed = 0;
  playing = false;
  lastUpdateTime = millis();

  Serial.println("[MotionPlayer] Initialized.");
}

void playMotion(const MotionData& motion) {
  currentMotion = &motion;
  stepIndex = 0;
  motionElapsed = 0;
  playing = true;
  lastUpdateTime = millis();

  Serial.print("[MotionPlayer] Play: ");
  Serial.print(motion.name);
  Serial.print(" steps=");
  Serial.println(motion.length);
}

void updateMotionPlayer() {
  if (!playing || currentMotion == nullptr) {
    return;
  }

  unsigned long now = millis();

  if (now - lastUpdateTime < UPDATE_INTERVAL_MS) {
    return;
  }

  lastUpdateTime = now;

  // 開始すべきステップを探す
  while (
    stepIndex < currentMotion->length &&
    currentMotion->steps[stepIndex].wait <= motionElapsed
  ) {
    startStep(currentMotion->steps[stepIndex]);

    // 学校資料と同じ考え方：
    // ステップ開始後、次のステップのwaitを数えるためにelapsedを0へ戻す
    motionElapsed = 0;
    stepIndex++;
  }

  updateServoStates();

  motionElapsed += UPDATE_INTERVAL_MS;

  // すべてのステップを開始済み、かつ全サーボが止まったら完了
  if (stepIndex >= currentMotion->length && !anyServoMoving()) {
    Serial.print("[MotionPlayer] Finished: ");
    Serial.println(currentMotion->name);

    playing = false;
    currentMotion = nullptr;
  }
}

bool isMotionPlaying() {
  return playing;
}

void stopMotion() {
  playing = false;
  currentMotion = nullptr;
  stepIndex = 0;
  motionElapsed = 0;

  for (int ch = 0; ch < 16; ch++) {
    servoStates[ch].moving = false;
  }

  Serial.println("[MotionPlayer] Stopped.");
}

const char* getCurrentMotionName() {
  if (currentMotion == nullptr) {
    return "NONE";
  }

  return currentMotion->name;
}