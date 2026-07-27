#ifndef COMMAND_H
#define COMMAND_H

// 歩行・姿勢
#define CMD_STAND        "STAND"
#define CMD_FORWARD      "FORWARD"
#define CMD_LEFT         "LEFT"
#define CMD_RIGHT        "RIGHT"
#define CMD_STOP         "STOP"

// Raspberry Pi自動追従専用の小刻み旋回
#define CMD_TRACK_LEFT   "TRACK_LEFT"
#define CMD_TRACK_RIGHT  "TRACK_RIGHT"

// ライト
#define CMD_LIGHT_ON     "LIGHT_ON"
#define CMD_LIGHT_OFF    "LIGHT_OFF"
#define CMD_LIGHT_TOGGLE "LIGHT_TOGGLE"

// モード
#define CMD_FOLLOW       "FOLLOW"
#define CMD_FIX          "FIX"
#define CMD_MANUAL       "MANUAL"

// 頭
#define CMD_HEAD_LEFT    "HEAD_LEFT"
#define CMD_HEAD_RIGHT   "HEAD_RIGHT"
#define CMD_HEAD_CENTER  "HEAD_CENTER"
#define CMD_HEAD_UP      "HEAD_UP"
#define CMD_HEAD_DOWN    "HEAD_DOWN"

// デバッグ
#define CMD_HELP         "HELP"
#define CMD_STATUS       "STATUS"

#endif
