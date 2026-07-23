#include <Arduino.h>

#include "config.h"
#include "servo_control.h"
#include "head_control.h"


// ============================================================
// 現在角度と目標角度
// ============================================================

static float currentYawAngle = HEAD_YAW_CENTER_ANGLE;
static float targetYawAngle = HEAD_YAW_CENTER_ANGLE;

static float currentPitchAngle = HEAD_PITCH_CENTER_ANGLE;
static float targetPitchAngle = HEAD_PITCH_CENTER_ANGLE;

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
    float targetValue,
    float step
) {
    if (currentValue < targetValue) {
        currentValue += step;

        if (currentValue > targetValue) {
            currentValue = targetValue;
        }
    }
    else if (currentValue > targetValue) {
        currentValue -= step;

        if (currentValue < targetValue) {
            currentValue = targetValue;
        }
    }

    return currentValue;
}


// ============================================================
// 初期化
// ============================================================

void initHeadControl() {
    currentYawAngle = HEAD_YAW_CENTER_ANGLE;
    targetYawAngle = HEAD_YAW_CENTER_ANGLE;

    currentPitchAngle = HEAD_PITCH_CENTER_ANGLE;
    targetPitchAngle = HEAD_PITCH_CENTER_ANGLE;

    setServoAngle(
        HEAD_YAW_CHANNEL,
        currentYawAngle
    );

    setServoAngle(
        HEAD_PITCH_CHANNEL,
        currentPitchAngle
    );

    lastHeadUpdateTime = millis();

    Serial.println("[Head] Initialized");
}


// ============================================================
// 滑らか移動
// ============================================================

void updateHeadControl() {
    unsigned long now = millis();

    if (
        now - lastHeadUpdateTime
        < HEAD_UPDATE_INTERVAL_MS
    ) {
        return;
    }

    lastHeadUpdateTime = now;

    float previousYaw = currentYawAngle;
    float previousPitch = currentPitchAngle;

    currentYawAngle = moveToward(
        currentYawAngle,
        targetYawAngle,
        HEAD_SMOOTH_STEP_ANGLE
    );

    currentPitchAngle = moveToward(
        currentPitchAngle,
        targetPitchAngle,
        HEAD_SMOOTH_STEP_ANGLE
    );

    // 角度が変わった場合だけPCA9685へ送信する
    if (currentYawAngle != previousYaw) {
        setServoAngle(
            HEAD_YAW_CHANNEL,
            currentYawAngle
        );
    }

    if (currentPitchAngle != previousPitch) {
        setServoAngle(
            HEAD_PITCH_CHANNEL,
            currentPitchAngle
        );
    }
}


// ============================================================
// 左右
// ============================================================

void headLeft() {
    targetYawAngle = HEAD_YAW_MIN_ANGLE;

    Serial.print("[Head] Target yaw: ");
    Serial.println(targetYawAngle);
}


void headRight() {
    targetYawAngle = HEAD_YAW_MAX_ANGLE;

    Serial.print("[Head] Target yaw: ");
    Serial.println(targetYawAngle);
}


// ============================================================
// 上下
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

    Serial.print("[Head] Nudge UP, target pitch: ");
    Serial.println(targetPitchAngle);
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

    Serial.print("[Head] Nudge DOWN, target pitch: ");
    Serial.println(targetPitchAngle);
}


// ============================================================
// 中央
// ============================================================

void headCenter() {
    targetYawAngle = HEAD_YAW_CENTER_ANGLE;
    targetPitchAngle = HEAD_PITCH_CENTER_ANGLE;

    Serial.println("[Head] Moving slowly to center");
}


// ============================================================
// デバッグ
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