#include <Arduino.h>

#include "servo_control.h"
#include "motion_player.h"
#include "walking.h"
#include "serial_receive.h"
#include "head_control.h"
#include "light_control.h"


// ============================================================
// 初期化
// ============================================================

void setup() {
    Serial.begin(115200);

    // シリアルモニター開始待ち。
    delay(300);

    Serial.println();
    Serial.println(
        "========================================"
    );
    Serial.println(
        "Presentation Escort ESP32"
    );
    Serial.println(
        "Smooth Head Control"
    );
    Serial.println(
        "========================================"
    );

    // PCA9685とサーボ制御
    initServoControl();

    // モーション再生機能
    initMotionPlayer();

    // 二足歩行制御
    initWalking();

    // 頭サーボを中央位置で初期化
    initHeadControl();

    // 白色LED制御
    initLightControl();

    // Raspberry Pi／シリアルモニターからの
    // コマンド受信機能
    initSerialReceive();

    Serial.println("[Main] Setup complete");
    Serial.println(
        "[Main] Commands: "
        "LEFT RIGHT FORWARD STOP "
        "HEAD_UP HEAD_DOWN HEAD_CENTER"
    );
}


// ============================================================
// メインループ
// ============================================================

void loop() {
    // Raspberry Piからのコマンドを受信する。
    // HEAD_UP / HEAD_DOWNはhead_control.cpp内で
    // 目標角度を少しだけ変更する。
    updateSerialReceive();

    // 歩行する方向や状態を更新する。
    updateWalking();

    // 現在実行中の歩行モーションを更新する。
    updateMotionPlayer();

    // 頭サーボを目標角度へ少しずつ近づける。
    // この関数を高頻度で呼ぶ必要があるため、
    // loop()内に長いdelay()を置かないこと。
    updateHeadControl();

    // ESP32のWatchdogや他処理へ実行時間を渡す。
    delay(1);
}