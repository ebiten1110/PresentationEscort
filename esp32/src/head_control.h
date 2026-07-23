#ifndef HEAD_CONTROL_H
#define HEAD_CONTROL_H

#include <Arduino.h>

// ============================================================
// 初期化・定期更新
// ============================================================

// 頭サーボを中央位置で初期化する。
void initHeadControl();

// loop()から毎回呼ぶ。
// delay()を使わず、現在角度を目標角度へ徐々に近づける。
void updateHeadControl();


// ============================================================
// コマンド
// ============================================================

// 1回呼ばれるたび、目標角度を少しだけ左へ動かす。
void headLeft();

// 1回呼ばれるたび、目標角度を少しだけ右へ動かす。
void headRight();

// 1回呼ばれるたび、目標角度を少しだけ上へ動かす。
void headUp();

// 1回呼ばれるたび、目標角度を少しだけ下へ動かす。
void headDown();

// 目標角度を中央へ設定する。
// 実際のサーボはupdateHeadControl()によって徐々に中央へ戻る。
void headCenter();


// ============================================================
// 状態取得
// ============================================================

int getCurrentHeadYaw();
int getCurrentHeadPitch();

int getTargetHeadYaw();
int getTargetHeadPitch();

bool isHeadMoving();

#endif