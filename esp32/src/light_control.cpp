#include "light_control.h"

#include "config.h"

static bool lightState = false;

void initLightControl() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lightState = false;

  Serial.println("[LightControl] Initialized.");
}

void lightOn() {
  digitalWrite(LED_PIN, HIGH);
  lightState = true;

  Serial.println("[LightControl] LIGHT_ON");
}

void lightOff() {
  digitalWrite(LED_PIN, LOW);
  lightState = false;

  Serial.println("[LightControl] LIGHT_OFF");
}

void lightToggle() {
  if (lightState) {
    lightOff();
  } else {
    lightOn();
  }
}

bool isLightOn() {
  return lightState;
}