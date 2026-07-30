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
    "V8.4C GPIO RGB LED x3"
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
  updateSerialReceive();

  // updateWalking()内部でMotionPlayerも更新する。
  updateWalking();

  updateHeadControl();

  // 赤点滅をdelayなしで進める。
  updateLightControl();

  delay(1);
}
