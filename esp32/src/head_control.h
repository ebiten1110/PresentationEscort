#ifndef HEAD_CONTROL_H
#define HEAD_CONTROL_H

#include <Arduino.h>


// ============================================================
// 初期化・定期更新
// ============================================================

// 頭サーボを中央位置で初期化する。
void initHeadControl();

// loop()から毎回呼ぶ。
// 現在角度を目標角度へ1度ずつ近づける。
void updateHeadControl();


// ============================================================
// 頭の操作
// ============================================================

// 1回呼ぶごとに目標角度を1度変更する。
void headLeft();
void headRight();
void headUp();
void headDown();

// 目標角度を中央へ設定する。
// 実際のサーボはupdateHeadControl()で1度ずつ中央へ戻る。
void headCenter();


// ============================================================
// 状態取得
// 戻り値をfloatで統一する。
// ============================================================

float getCurrentHeadYaw();
float getCurrentHeadPitch();

float getTargetHeadYaw();
float getTargetHeadPitch();

bool isHeadMoving();


#endif