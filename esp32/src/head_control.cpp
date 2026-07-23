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


// ============================================================
// 1回のコマンドで動かす目標角度
// ============================================================

// HEAD_LEFT / HEAD_RIGHTを1回受信するごとに1度変更
static const int HEAD_YAW_NUDGE_ANGLE = 1;

// HEAD_UP / HEAD_DOWNを1回受信するごとに1度変更
static const int HEAD_PITCH_NUDGE_ANGLE = 1;


// ============================================================
// 滑らかなサーボ移動
// ============================================================

// 1回の更新で実際のサーボ角度を1度だけ変更
static const int HEAD_SMOOTH_STEP_ANGLE = 1;

// 1度動かす間隔
// 50msなら、最大で1秒に20度動く
static const unsigned long HEAD_UPDATE_INTERVAL_MS = 50;


// ============================================================
// サーボ取り付け方向
// ============================================================

// 実機で上下が逆の場合は、符号を入れ替える
static const int HEAD_PITCH_UP_DIRECTION = -1;
static const int HEAD_PITCH_DOWN_DIRECTION = 1;

// 実機で左右が逆の場合は、符号を入れ替える
static const int HEAD_YAW_LEFT_DIRECTION = -1;
static const int HEAD_YAW_RIGHT_DIRECTION = 1;


// ============================================================
// 現在角度・目標角度
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


static int moveTowardByOneDegree(
    int currentValue,
    int targetValue
) {
    if (currentValue < targetValue) {
        return currentValue + 1;
    }

    if (currentValue > targetValue) {
        return currentValue - 1;
    }

    return currentValue;
}


static void printTargetAngles(
    const char* action
) {
    Serial.print("[Head] ");
    Serial.print(action);

    Serial.print(" currentYaw=");
    Serial.print(currentYawAngle);

    Serial.print(" targetYaw=");
    Serial.print(targetYawAngle);

    Serial.print(" currentPitch=");
    Serial.print(currentPitchAngle);

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
// 1度ずつ滑らかに移動
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

    // 目標角度へ1度ずつ近づける
    currentYawAngle = moveTowardByOneDegree(
        currentYawAngle,
        targetYawAngle
    );

    currentPitchAngle = moveTowardByOneDegree(
        currentPitchAngle,
        targetPitchAngle
    );

    // 角度が変わった場合だけPCA9685へ送信
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
// 左右：1回の受信につき目標角度を1度変更
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

    printTargetAngles("NUDGE_LEFT_1_DEG");
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

    printTargetAngles("NUDGE_RIGHT_1_DEG");
}


// ============================================================
// 上下：1回の受信につき目標角度を1度変更
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

    printTargetAngles("NUDGE_UP_1_DEG");
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

    printTargetAngles("NUDGE_DOWN_1_DEG");
}


// ============================================================
// 中央：目標を0度にして、1度ずつ中央へ戻す
// ============================================================

void headCenter() {
    targetYawAngle = HEAD_YAW_CENTER_ANGLE;
    targetPitchAngle = HEAD_PITCH_CENTER_ANGLE;

    printTargetAngles("SMOOTH_CENTER_1_DEG");
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