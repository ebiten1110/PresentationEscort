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

  delay(300);

  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    "Presentation Escort ESP32"
  );
  Serial.println(
    "1 Degree Smooth Head Control"
  );
  Serial.println(
    "========================================"
  );

  initServoControl();
  initMotionPlayer();
  initWalking();
  initHeadControl();
  initLightControl();
  initSerialReceive();

  Serial.println("[Main] Setup complete");

  Serial.println(
    "[Main] Commands: "
    "LEFT RIGHT FORWARD STOP "
    "HEAD_LEFT HEAD_RIGHT "
    "HEAD_UP HEAD_DOWN HEAD_CENTER "
    "LIGHT_ON LIGHT_OFF LIGHT_TOGGLE"
  );
}


// ============================================================
// メインループ
// ============================================================

void loop() {
  // Raspberry Pi／シリアルモニターから命令を受信
  updateSerialReceive();

  // updateWalking()の内部でupdateMotionPlayer()も呼ばれる。
  // main.cppから重複して呼ばない。
  updateWalking();

  // 頭を目標角度へ1度ずつ動かす。
  updateHeadControl();

  // 長いdelayは入れない。
  delay(1);
}
