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
  Serial.println("========================================");
  Serial.println("Presentation Escort ESP32");
  Serial.println("V8.8 DUAL LOOP CAMERA BODY");
  Serial.println("========================================");

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
  updateWalking();
  updateHeadControl();
  updateLightControl();
  delay(1);
}
