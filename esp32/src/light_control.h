#ifndef LIGHT_CONTROL_H
#define LIGHT_CONTROL_H

#include <Arduino.h>

// ライト制御の初期化
void initLightControl();

// ライトON
void lightOn();

// ライトOFF
void lightOff();

// ライト切り替え
void lightToggle();

// 現在ライトが点灯しているか
bool isLightOn();

#endif