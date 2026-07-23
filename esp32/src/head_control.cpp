#include <Arduino.h>

#include "head_control.h"
#include "servo_control.h"


// ============================================================
// 頭サーボ設定
// ============================================================

// PCA9685チャンネル
// 水平サーボ
static const uint8_t HEAD_YAW_CHANNEL = 2;

// 上下サーボ
static const uint8_t HEAD_PITCH_CHANNEL = 3;


// 中央角度
static const int HEAD_YAW_CENTER_ANGLE = 0;
static const int HEAD_PITCH_CENTER_ANGLE = 0;


// 安全な可動範囲
static const int HEAD_YAW_MIN_ANGLE = -20;
static const int HEAD_YAW_MAX_ANGLE = 20;

static const int HEAD_PITCH_MIN_ANGLE = -12;
static const int HEAD_PITCH_MAX_ANGLE = 12;


// コマンドを1回受け取ったときに変える目標角度
static const int HEAD_YAW_NUDGE_ANGLE = 2;
static const int HEAD_PITCH_NUDGE_ANGLE = 1;


// updateHeadControl() 1回の有効更新で動かす角度
static const int HEAD_SMOOTH_STEP_ANGLE = 1;


// サーボ角度を更新する間隔
// 50msなら、最大で1秒に20度動く。
static const unsigned long HEAD_UPDATE_INTERVAL_MS = 50;


// 実機で上下が逆の場合は、次の符号を入れ替える。
static const int HEAD_PITCH_UP_DIRECTION = -1;
static const int HEAD_PITCH_DOWN_DIRECTION = 1;


// 実機で左右が逆の場合は、次の符号を入れ替える。
static const int HEAD_YAW_LEFT_DIRECTION = -1;
static const int HEAD_YAW_RIGHT_DIRECTION = 1;


// ============================================================
// 状態
// ============================================================

static int currentYawAngle = HEAD_YAW_CENTER_ANGLE;
static int targetYawAngle = HEAD_YAW_CENTER_ANGLE;

static int currentPitchAngle = HEAD_PITCH_CENTER_ANGLE;
static int targetPitchAngle = HEAD_PITCH_CENTER_ANGLE;

static unsigned long lastHeadUpdateTime = 0;


// ============================================================
// 内部関数
// ============================================================

static int clampAngle(
    int value,
    int minimum,
    int maximum
) {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}


static int moveToward(
    int currentValue,
    int targetValue,
    int step
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


static void printTargetAngles(
    const char* action
) {
    Serial.print("[Head] ");
    Serial.print(action);

    Serial.print(" targetYaw=");
    Serial.print(targetYawAngle);

    Serial.print(" targetPitch=");
    Serial.println(targetPitchAngle);
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

    Serial.println(
        "[Head] Initialized at center"
    );
}


// ============================================================
// 滑らか移動
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

    const int previousYaw = currentYawAngle;
    const int previousPitch = currentPitchAngle;

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

    // 角度が変化したときだけPCA9685へ送信する。
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
    targetYawAngle += (
        HEAD_YAW_NUDGE_ANGLE
        * HEAD_YAW_LEFT_DIRECTION
    );

    targetYawAngle = clampAngle(
        targetYawAngle,
        HEAD_YAW_MIN_ANGLE,
        HEAD_YAW_MAX_ANGLE
    );

    printTargetAngles("NUDGE_LEFT");
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

    printTargetAngles("NUDGE_RIGHT");
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

    printTargetAngles("NUDGE_UP");
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

    printTargetAngles("NUDGE_DOWN");
}


// ============================================================
// 中央
// ============================================================

void headCenter() {
    targetYawAngle = HEAD_YAW_CENTER_ANGLE;
    targetPitchAngle = HEAD_PITCH_CENTER_ANGLE;

    printTargetAngles("SMOOTH_CENTER");
}


// ============================================================
// 状態取得
// ============================================================

int getCurrentHeadYaw() {
    return currentYawAngle;
}


int getCurrentHeadPitch() {
    return currentPitchAngle;
}


int getTargetHeadYaw() {
    return targetYawAngle;
}


int getTargetHeadPitch() {
    return targetPitchAngle;
}


bool isHeadMoving() {
    return (
        currentYawAngle != targetYawAngle
        || currentPitchAngle != targetPitchAngle
    );
}