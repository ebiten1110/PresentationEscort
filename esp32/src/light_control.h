#ifndef LIGHT_CONTROL_H
#define LIGHT_CONTROL_H

#include <Arduino.h>


// ============================================================
// 4本足RGB LEDの表示モード
// ============================================================

enum class LightMode : uint8_t {
  OFF,
  WHITE,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  PURPLE,
  ERROR_BLINK
};


// ============================================================
// 初期化・更新
// ============================================================

void initLightControl();

// ERROR_BLINKをdelayなしで進める。
// loop()から毎回呼び出す。
void updateLightControl();


// ============================================================
// 色指定
// ============================================================

void setLightMode(LightMode mode);

void lightOff();
void lightWhite();
void lightRed();
void lightGreen();
void lightBlue();
void lightYellow();
void lightPurple();
void lightError();


// ============================================================
// 既存LIGHT_ON等との互換
// ============================================================

// LIGHT_ONは白色点灯として扱う。
void lightOn();
void lightToggle();

void toggleLight();
void turnLightOn();
void turnLightOff();

void setLight(bool enabled);


// ============================================================
// 状態取得
// ============================================================

bool isLightOn();
bool getLightState();

LightMode getLightMode();
const char* getLightModeName();

#endif
