#include <Arduino.h>

#include "servo_control.h"
#include "motion_player.h"
#include "walking.h"
#include "serial_receive.h"
#include "head_control.h"
#include "light_control.h"


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
    "V8 Auto Follow Micro Turn"
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
}


void loop() {
  // シリアル受信
  updateSerialReceive();

  // updateWalking()内部でMotionPlayerも1回だけ更新する。
  updateWalking();

  // 頭サーボ
  updateHeadControl();

  delay(1);
}
