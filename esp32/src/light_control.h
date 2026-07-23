#ifndef LIGHT_CONTROL_H
#define LIGHT_CONTROL_H

#include <Arduino.h>


// ============================================================
// 白色LED制御
// ============================================================

void initLightControl();

void lightOn();
void lightOff();
void lightToggle();

// 既存コードとの互換用別名
void toggleLight();
void turnLightOn();
void turnLightOff();

void setLight(bool enabled);

bool isLightOn();
bool getLightState();


#endif