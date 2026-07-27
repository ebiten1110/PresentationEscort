#ifndef WALKING_H
#define WALKING_H

#include <Arduino.h>

void initWalking();
void updateWalking();

void stand();

void walkForwardSteps(int steps);
void walkForwardOnce();

void stopWalking();

// 手動用の大きい旋回
void turnLeft();
void turnRight();

// 自動追従用の小刻み旋回
void trackLeft();
void trackRight();

bool isWalkingBusy();
const char* getWalkingStateName();

#endif
