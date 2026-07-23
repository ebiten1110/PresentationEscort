#include <Arduino.h>

#include "light_control.h"


// ============================================================
// LED設定
// ============================================================

// 白色LEDを接続しているESP32のGPIO
static const uint8_t LIGHT_PIN = 25;

// 現在の点灯状態
static bool lightEnabled = false;


// ============================================================
// 内部関数
// ============================================================

static void applyLightState() {
    digitalWrite(
        LIGHT_PIN,
        lightEnabled ? HIGH : LOW
    );

    Serial.print("[LightControl] state=");
    Serial.println(
        lightEnabled ? "ON" : "OFF"
    );
}


// ============================================================
// 初期化
// ============================================================

void initLightControl() {
    pinMode(
        LIGHT_PIN,
        OUTPUT
    );

    lightEnabled = false;
    applyLightState();

    Serial.println(
        "[LightControl] Initialized"
    );
}


// ============================================================
// 点灯・消灯
// ============================================================

void lightOn() {
    lightEnabled = true;
    applyLightState();
}


void lightOff() {
    lightEnabled = false;
    applyLightState();
}


void lightToggle() {
    lightEnabled = !lightEnabled;
    applyLightState();
}


// ============================================================
// 既存コードとの互換用関数
// ============================================================

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
    lightEnabled = enabled;
    applyLightState();
}


// ============================================================
// 状態取得
// ============================================================

bool isLightOn() {
    return lightEnabled;
}


bool getLightState() {
    return lightEnabled;
}