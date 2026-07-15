#include <Arduino.h>

#include "config.h"
#include "servo_control.h"
#include "motion_player.h"
#include "walking.h"
#include "serial_receive.h"
#include "head_control.h"
#include "light_control.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println("PresentationEscort Phase 4-2 - Light Control");

  initServoControl();
  initMotionPlayer();
  initWalking();
  initHeadControl();
  initLightControl();
  initSerialReceive();

  stand();

  Serial.println("Ready. Type HELP.");
}

void loop() {
  updateWalking();
  updateSerialReceive();
}