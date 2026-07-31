#include <Arduino.h>

#include "head_control.h"
#include "servo_control.h"
#include "config.h"


// ============================================================
// 設定
// ============================================================
//
// 既存config.hには、次の名前が#defineとして定義されている:
//
//   HEAD_SMOOTH_STEP_ANGLE
//   HEAD_UPDATE_INTERVAL_MS
//
// #defineされた名前と同じ変数名を宣言すると、プリプロセッサが
// 変数名を数値へ置換してコンパイルエラーになる。
//
// V8.8.1では二重ループ制御専用の名前へ変更し、衝突を防ぐ。
// config.hから値を調整したい場合は、次の2つを追加できる:
//
//   #define DUAL_LOOP_HEAD_SMOOTH_STEP_ANGLE 1.0f
//   #define DUAL_LOOP_HEAD_UPDATE_INTERVAL_MS 30UL
// ============================================================

#ifndef HEAD_YAW_MIN_ANGLE
#define HEAD_YAW_MIN_ANGLE -20.0f
#endif

#ifndef HEAD_YAW_MAX_ANGLE
#define HEAD_YAW_MAX_ANGLE 20.0f
#endif

#ifndef HEAD_PITCH_MIN_ANGLE
#define HEAD_PITCH_MIN_ANGLE -12.0f
#endif

#ifndef HEAD_PITCH_MAX_ANGLE
#define HEAD_PITCH_MAX_ANGLE 12.0f
#endif

#ifndef HEAD_YAW_NUDGE_ANGLE
#define HEAD_YAW_NUDGE_ANGLE 1.0f
#endif

#ifndef HEAD_PITCH_NUDGE_ANGLE
#define HEAD_PITCH_NUDGE_ANGLE 1.0f
#endif

#ifndef HEAD_YAW_LEFT_DIRECTION
#define HEAD_YAW_LEFT_DIRECTION -1.0f
#endif

#ifndef HEAD_YAW_RIGHT_DIRECTION
#define HEAD_YAW_RIGHT_DIRECTION 1.0f
#endif

#ifndef HEAD_PITCH_UP_DIRECTION
#define HEAD_PITCH_UP_DIRECTION -1.0f
#endif

#ifndef HEAD_PITCH_DOWN_DIRECTION
#define HEAD_PITCH_DOWN_DIRECTION 1.0f
#endif

#ifndef STAND_ANGLE_HEAD_YAW
#define STAND_ANGLE_HEAD_YAW 0.0f
#endif

#ifndef STAND_ANGLE_HEAD_PITCH
#define STAND_ANGLE_HEAD_PITCH 0.0f
#endif

#ifndef DUAL_LOOP_HEAD_SMOOTH_STEP_ANGLE
#define DUAL_LOOP_HEAD_SMOOTH_STEP_ANGLE 1.0f
#endif

#ifndef DUAL_LOOP_HEAD_UPDATE_INTERVAL_MS
#define DUAL_LOOP_HEAD_UPDATE_INTERVAL_MS 30UL
#endif

#ifndef DUAL_LOOP_HEAD_STATUS_INTERVAL_MS
#define DUAL_LOOP_HEAD_STATUS_INTERVAL_MS 150UL
#endif


// ============================================================
// 状態
// ============================================================

static float currentYawAngle = STAND_ANGLE_HEAD_YAW;
static float targetYawAngle = STAND_ANGLE_HEAD_YAW;

static float currentPitchAngle = STAND_ANGLE_HEAD_PITCH;
static float targetPitchAngle = STAND_ANGLE_HEAD_PITCH;

static unsigned long lastHeadUpdateTime = 0;
static unsigned long lastStatusTime = 0;


// ============================================================
// 内部関数
// ============================================================

static float clampAngle(
  float value,
  float minimum,
  float maximum
) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}


static float moveToward(
  float currentValue,
  float targetValue
) {
  const float stepAngle =
    static_cast<float>(
      DUAL_LOOP_HEAD_SMOOTH_STEP_ANGLE
    );

  if (currentValue < targetValue) {
    currentValue += stepAngle;

    if (currentValue > targetValue) {
      currentValue = targetValue;
    }
  }
  else if (currentValue > targetValue) {
    currentValue -= stepAngle;

    if (currentValue < targetValue) {
      currentValue = targetValue;
    }
  }

  return currentValue;
}


// ============================================================
// 状態表示
// ============================================================

void printHeadStatus() {
  Serial.print("[HeadState] currentYaw=");
  Serial.print(currentYawAngle, 1);

  Serial.print(" targetYaw=");
  Serial.print(targetYawAngle, 1);

  Serial.print(" currentPitch=");
  Serial.print(currentPitchAngle, 1);

  Serial.print(" targetPitch=");
  Serial.print(targetPitchAngle, 1);

  Serial.print(" moving=");
  Serial.println(
    isHeadMoving() ? "YES" : "NO"
  );

  lastStatusTime = millis();
}


// ============================================================
// 初期化・更新
// ============================================================

void initHeadControl() {
  currentYawAngle = STAND_ANGLE_HEAD_YAW;
  targetYawAngle = STAND_ANGLE_HEAD_YAW;

  currentPitchAngle = STAND_ANGLE_HEAD_PITCH;
  targetPitchAngle = STAND_ANGLE_HEAD_PITCH;

  setHeadYaw(currentYawAngle);
  setHeadPitch(currentPitchAngle);

  lastHeadUpdateTime = millis();
  lastStatusTime = millis();

  Serial.print(
    "[HeadControl] stepAngle="
  );
  Serial.print(
    static_cast<float>(
      DUAL_LOOP_HEAD_SMOOTH_STEP_ANGLE
    ),
    1
  );

  Serial.print(" updateMs=");
  Serial.println(
    static_cast<unsigned long>(
      DUAL_LOOP_HEAD_UPDATE_INTERVAL_MS
    )
  );

  printHeadStatus();
}


void updateHeadControl() {
  const unsigned long now = millis();

  if (
    now - lastHeadUpdateTime
    < static_cast<unsigned long>(
      DUAL_LOOP_HEAD_UPDATE_INTERVAL_MS
    )
  ) {
    return;
  }

  lastHeadUpdateTime = now;

  const float previousYaw =
    currentYawAngle;

  const float previousPitch =
    currentPitchAngle;

  currentYawAngle = moveToward(
    currentYawAngle,
    targetYawAngle
  );

  currentPitchAngle = moveToward(
    currentPitchAngle,
    targetPitchAngle
  );

  if (
    currentYawAngle
    != previousYaw
  ) {
    setHeadYaw(currentYawAngle);
  }

  if (
    currentPitchAngle
    != previousPitch
  ) {
    setHeadPitch(currentPitchAngle);
  }

  const bool moved = (
    currentYawAngle != previousYaw
    || currentPitchAngle != previousPitch
  );

  if (
    moved
    && (
      now - lastStatusTime
      >= static_cast<unsigned long>(
        DUAL_LOOP_HEAD_STATUS_INTERVAL_MS
      )
    )
  ) {
    printHeadStatus();
  }
}


// ============================================================
// 絶対角度設定
// ============================================================

void setHeadYawTarget(
  float yawAngle
) {
  targetYawAngle = clampAngle(
    yawAngle,
    HEAD_YAW_MIN_ANGLE,
    HEAD_YAW_MAX_ANGLE
  );
}


void setHeadPitchTarget(
  float pitchAngle
) {
  targetPitchAngle = clampAngle(
    pitchAngle,
    HEAD_PITCH_MIN_ANGLE,
    HEAD_PITCH_MAX_ANGLE
  );
}


void setHeadPoseTarget(
  float yawAngle,
  float pitchAngle
) {
  targetYawAngle = clampAngle(
    yawAngle,
    HEAD_YAW_MIN_ANGLE,
    HEAD_YAW_MAX_ANGLE
  );

  targetPitchAngle = clampAngle(
    pitchAngle,
    HEAD_PITCH_MIN_ANGLE,
    HEAD_PITCH_MAX_ANGLE
  );
}


// ============================================================
// 従来コマンド互換
// ============================================================

void headLeft() {
  setHeadYawTarget(
    targetYawAngle
    + HEAD_YAW_NUDGE_ANGLE
      * HEAD_YAW_LEFT_DIRECTION
  );
}


void headRight() {
  setHeadYawTarget(
    targetYawAngle
    + HEAD_YAW_NUDGE_ANGLE
      * HEAD_YAW_RIGHT_DIRECTION
  );
}


void headUp() {
  setHeadPitchTarget(
    targetPitchAngle
    + HEAD_PITCH_NUDGE_ANGLE
      * HEAD_PITCH_UP_DIRECTION
  );
}


void headDown() {
  setHeadPitchTarget(
    targetPitchAngle
    + HEAD_PITCH_NUDGE_ANGLE
      * HEAD_PITCH_DOWN_DIRECTION
  );
}


void headCenter() {
  setHeadPoseTarget(
    STAND_ANGLE_HEAD_YAW,
    STAND_ANGLE_HEAD_PITCH
  );
}


// ============================================================
// 状態取得
// ============================================================

float getCurrentHeadYaw() {
  return currentYawAngle;
}


float getCurrentHeadPitch() {
  return currentPitchAngle;
}


float getTargetHeadYaw() {
  return targetYawAngle;
}


float getTargetHeadPitch() {
  return targetPitchAngle;
}


bool isHeadMoving() {
  return (
    currentYawAngle != targetYawAngle
    || currentPitchAngle != targetPitchAngle
  );
}