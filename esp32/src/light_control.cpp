#include <Arduino.h>

#include "light_control.h"


// ============================================================
// 4本足RGB LED ×3 設定
// ============================================================
//
// 3個とも同じ色で同期点灯するため、
// 同じ色の端子をそれぞれ抵抗経由で同じGPIOへ接続する。
//
// LED1 R -- 330Ω --+
// LED2 R -- 330Ω --+--> GPIO25
// LED3 R -- 330Ω --+
//
// LED1 G -- 330Ω --+
// LED2 G -- 330Ω --+--> GPIO26
// LED3 G -- 330Ω --+
//
// LED1 B -- 330Ω --+
// LED2 B -- 330Ω --+--> GPIO27
//
// 抵抗はLEDごと・色ごとに必要。合計9本。
// ============================================================

static constexpr uint8_t RGB_RED_PIN = 25;
static constexpr uint8_t RGB_GREEN_PIN = 26;
static constexpr uint8_t RGB_BLUE_PIN = 27;


// ============================================================
// LED種類
// ============================================================
//
// 共通カソード:
//   共通端子をGNDへ接続。
//   RGB_COMMON_ANODE = false
//
// 共通アノード:
//   共通端子を3.3Vへ接続。
//   RGB_COMMON_ANODE = true
//
// 授業資料のLEDは共通カソード型。
// ============================================================

static constexpr bool RGB_COMMON_ANODE = false;


// ============================================================
// 点滅設定
// ============================================================

static constexpr unsigned long ERROR_BLINK_INTERVAL_MS = 300;


// ============================================================
// 状態
// ============================================================

static LightMode currentMode = LightMode::OFF;

static bool errorBlinkVisible = false;
static unsigned long lastBlinkTimeMs = 0;


// ============================================================
// 内部関数
// ============================================================

static void writeColorChannel(
  uint8_t pin,
  bool enabled
) {
  bool outputHigh = enabled;

  if (RGB_COMMON_ANODE) {
    outputHigh = !enabled;
  }

  digitalWrite(
    pin,
    outputHigh ? HIGH : LOW
  );
}


static void showColor(
  bool red,
  bool green,
  bool blue
) {
  writeColorChannel(
    RGB_RED_PIN,
    red
  );

  writeColorChannel(
    RGB_GREEN_PIN,
    green
  );

  writeColorChannel(
    RGB_BLUE_PIN,
    blue
  );
}


static void applyCurrentMode() {
  switch (currentMode) {
    case LightMode::OFF:
      showColor(false, false, false);
      break;

    case LightMode::WHITE:
      showColor(true, true, true);
      break;

    case LightMode::RED:
      showColor(true, false, false);
      break;

    case LightMode::GREEN:
      showColor(false, true, false);
      break;

    case LightMode::BLUE:
      showColor(false, false, true);
      break;

    case LightMode::YELLOW:
      showColor(true, true, false);
      break;

    case LightMode::PURPLE:
      showColor(true, false, true);
      break;

    case LightMode::ERROR_BLINK:
      showColor(
        errorBlinkVisible,
        false,
        false
      );
      break;

    default:
      showColor(false, false, false);
      break;
  }

  Serial.print("[LightControl] mode=");
  Serial.println(getLightModeName());
}


// ============================================================
// 初期化・更新
// ============================================================

void initLightControl() {
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  currentMode = LightMode::OFF;
  errorBlinkVisible = false;
  lastBlinkTimeMs = millis();

  applyCurrentMode();

  Serial.println(
    "[LightControl] 4-pin common-cathode RGB LED x3 initialized."
  );

  Serial.print(
    "[LightControl] pins R/G/B="
  );
  Serial.print(RGB_RED_PIN);
  Serial.print("/");
  Serial.print(RGB_GREEN_PIN);
  Serial.print("/");
  Serial.println(RGB_BLUE_PIN);

  Serial.print(
    "[LightControl] common=CATHODE fixed, detected="
  );
  Serial.println(
    RGB_COMMON_ANODE
      ? "ANODE"
      : "CATHODE"
  );
}


void updateLightControl() {
  if (
    currentMode
    != LightMode::ERROR_BLINK
  ) {
    return;
  }

  const unsigned long nowMs = millis();

  if (
    nowMs - lastBlinkTimeMs
    < ERROR_BLINK_INTERVAL_MS
  ) {
    return;
  }

  lastBlinkTimeMs = nowMs;
  errorBlinkVisible = !errorBlinkVisible;

  applyCurrentMode();
}


// ============================================================
// モード設定
// ============================================================

void setLightMode(LightMode mode) {
  if (currentMode == mode) {
    return;
  }

  currentMode = mode;

  if (
    currentMode
    == LightMode::ERROR_BLINK
  ) {
    errorBlinkVisible = true;
    lastBlinkTimeMs = millis();
  }
  else {
    errorBlinkVisible = false;
  }

  applyCurrentMode();
}


void lightOff() {
  setLightMode(LightMode::OFF);
}


void lightWhite() {
  setLightMode(LightMode::WHITE);
}


void lightRed() {
  setLightMode(LightMode::RED);
}


void lightGreen() {
  setLightMode(LightMode::GREEN);
}


void lightBlue() {
  setLightMode(LightMode::BLUE);
}


void lightYellow() {
  setLightMode(LightMode::YELLOW);
}


void lightPurple() {
  setLightMode(LightMode::PURPLE);
}


void lightError() {
  setLightMode(LightMode::ERROR_BLINK);
}


// ============================================================
// 既存コードとの互換
// ============================================================

void lightOn() {
  lightWhite();
}


void lightToggle() {
  if (isLightOn()) {
    lightOff();
  }
  else {
    lightWhite();
  }
}


void toggleLight() {
  lightToggle();
}


void turnLightOn() {
  lightOn();
}


void turnLightOff() {
  lightOff();
}


void setLight(bool enabled) {
  if (enabled) {
    lightWhite();
  }
  else {
    lightOff();
  }
}


// ============================================================
// 状態取得
// ============================================================

bool isLightOn() {
  return currentMode != LightMode::OFF;
}


bool getLightState() {
  return isLightOn();
}


LightMode getLightMode() {
  return currentMode;
}


const char* getLightModeName() {
  switch (currentMode) {
    case LightMode::OFF:
      return "OFF";

    case LightMode::WHITE:
      return "WHITE";

    case LightMode::RED:
      return "RED";

    case LightMode::GREEN:
      return "GREEN";

    case LightMode::BLUE:
      return "BLUE";

    case LightMode::YELLOW:
      return "YELLOW";

    case LightMode::PURPLE:
      return "PURPLE";

    case LightMode::ERROR_BLINK:
      return "ERROR_BLINK";

    default:
      return "UNKNOWN";
  }
}
