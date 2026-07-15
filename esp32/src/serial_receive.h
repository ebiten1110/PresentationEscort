#ifndef SERIAL_RECEIVE_H
#define SERIAL_RECEIVE_H

#include <Arduino.h>

// シリアル受信機能の初期化
void initSerialReceive();

// loop() 内で毎回呼ぶ
void updateSerialReceive();

// 最後に受け取ったコマンド文字列を返す
String getLastCommand();

// ヘルプ表示
void printCommandHelp();

#endif