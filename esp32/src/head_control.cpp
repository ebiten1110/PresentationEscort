#include <Arduino.h>

#include "head_control.h"
#include "servo_control.h"
#include "config.h"


// ============================================================
// 現在角度・目標角度
// ============================================================

static float currentYawAngle =
  STAND_ANGLE_HEAD_YAW;

static float targetYawAngle =
  STAND_ANGLE_HEAD_YAW;

static float currentPitchAngle =
  STAND_ANGLE_HEAD_PITCH;

static float targetPitchAngle =
  STAND_ANGLE_HEAD_PITCH;

static unsigned long lastHeadUpdateTime = 0;


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
  if (currentValue < targetValue) {
    currentValue += HEAD_SMOOTH_STEP_ANGLE;

    if (currentValue > targetValue) {
      currentValue = targetValue;
    }
  }
  else if (currentValue > targetValue) {
    currentValue -= HEAD_SMOOTH_STEP_ANGLE;

    if (currentValue < targetValue) {
      currentValue = targetValue;
    }
  }

  return currentValue;
}


static void printHeadState(
  const char* action
) {
  Serial.print("[HeadControl] ");
  Serial.print(action);

  Serial.print(" currentYaw=");
  Serial.print(currentYawAngle, 1);

  Serial.print(" targetYaw=");
  Serial.print(targetYawAngle, 1);

  Serial.print(" currentPitch=");
  Serial.print(currentPitchAngle, 1);

  Serial.print(" targetPitch=");
  Serial.println(targetPitchAngle, 1);
}


// ============================================================
// 初期化
// ============================================================

void initHeadControl() {
  currentYawAngle =
    STAND_ANGLE_HEAD_YAW;

  targetYawAngle =
    STAND_ANGLE_HEAD_YAW;

  currentPitchAngle =
    STAND_ANGLE_HEAD_PITCH;

  targetPitchAngle =
    STAND_ANGLE_HEAD_PITCH;

  setHeadYaw(currentYawAngle);
  setHeadPitch(currentPitchAngle);

  lastHeadUpdateTime = millis();

  printHeadState("INITIALIZED");
}


// ============================================================
// 1度ずつ実角度を目標へ近づける
// ============================================================

void updateHeadControl() {
  const unsigned long now = millis();

  if (
    now - lastHeadUpdateTime
    < HEAD_UPDATE_INTERVAL_MS
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

  if (currentYawAngle != previousYaw) {
    setHeadYaw(currentYawAngle);

    if (HEAD_APPLIED_ANGLE_LOG) {
      Serial.print(
        "[HeadControl] Applied yaw="
      );
      Serial.println(
        currentYawAngle,
        1
      );
    }
  }

  if (currentPitchAngle != previousPitch) {
    setHeadPitch(currentPitchAngle);

    if (HEAD_APPLIED_ANGLE_LOG) {
      Serial.print(
        "[HeadControl] Applied pitch="
      );
      Serial.println(
        currentPitchAngle,
        1
      );
    }
  }
}


// ============================================================
// 左右：コマンド1回につき目標を1度変更
// ============================================================

void headLeft() {
  targetYawAngle += (
    HEAD_YAW_NUDGE_ANGLE
    * HEAD_YAW_LEFT_DIRECTION
  );

  targetYawAngle = clampAngle(
    targetYawAngle,
    HEAD_YAW_MIN_ANGLE,
    HEAD_YAW_MAX_ANGLE
  );

  printHeadState("HEAD_LEFT_1_DEG");
}


void headRight() {
  targetYawAngle += (
    HEAD_YAW_NUDGE_ANGLE
    * HEAD_YAW_RIGHT_DIRECTION
  );

  targetYawAngle = clampAngle(
    targetYawAngle,
    HEAD_YAW_MIN_ANGLE,
    HEAD_YAW_MAX_ANGLE
  );

  printHeadState("HEAD_RIGHT_1_DEG");
}


// ============================================================
// 上下：コマンド1回につき目標を1度変更
// ============================================================

void headUp() {
  targetPitchAngle += (
    HEAD_PITCH_NUDGE_ANGLE
    * HEAD_PITCH_UP_DIRECTION
  );

  targetPitchAngle = clampAngle(
    targetPitchAngle,
    HEAD_PITCH_MIN_ANGLE,
    HEAD_PITCH_MAX_ANGLE
  );

  printHeadState("HEAD_UP_1_DEG");
}


void headDown() {
  targetPitchAngle += (
    HEAD_PITCH_NUDGE_ANGLE
    * HEAD_PITCH_DOWN_DIRECTION
  );

  targetPitchAngle = clampAngle(
    targetPitchAngle,
    HEAD_PITCH_MIN_ANGLE,
    HEAD_PITCH_MAX_ANGLE
  );

  printHeadState("HEAD_DOWN_1_DEG");
}


// ============================================================
// 1度ずつ中央へ戻す
// ============================================================

void headCenter() {
  targetYawAngle =
    STAND_ANGLE_HEAD_YAW;

  targetPitchAngle =
    STAND_ANGLE_HEAD_PITCH;

  printHeadState("HEAD_CENTER_SMOOTH");
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