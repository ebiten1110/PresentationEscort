#ifndef COMMAND_H
#define COMMAND_H

// =====================================================
// PresentationEscort - Command定義
// PC / Raspberry Pi からESP32へ送る文字列コマンド
// =====================================================

// 歩行・姿勢
#define CMD_STAND       "STAND"
#define CMD_FORWARD     "FORWARD"
#define CMD_LEFT        "LEFT"
#define CMD_RIGHT       "RIGHT"
#define CMD_STOP        "STOP"

// ライト制御用：Phase 4以降で使用
#define CMD_LIGHT_ON    "LIGHT_ON"
#define CMD_LIGHT_OFF   "LIGHT_OFF"
#define CMD_LIGHT_TOGGLE "LIGHT_TOGGLE"

// モード切替用：Raspberry Pi連携後に使用
#define CMD_FOLLOW      "FOLLOW"
#define CMD_FIX         "FIX"
#define CMD_MANUAL      "MANUAL"

// 頭サーボ用：後で使用
#define CMD_HEAD_LEFT   "HEAD_LEFT"
#define CMD_HEAD_RIGHT  "HEAD_RIGHT"
#define CMD_HEAD_CENTER "HEAD_CENTER"
#define CMD_HEAD_UP     "HEAD_UP"
#define CMD_HEAD_DOWN   "HEAD_DOWN"
// デバッグ用
#define CMD_HELP        "HELP"
#define CMD_STATUS      "STATUS"

#endif