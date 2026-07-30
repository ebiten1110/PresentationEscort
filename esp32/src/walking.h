#ifndef WALKING_H
#define WALKING_H

#include <Arduino.h>

void initWalking();
void updateWalking();

void stand();

void walkForwardSteps(int steps);
void walkForwardOnce();

void stopWalking();

void turnLeft();
void turnRight();

void trackLeft();
void trackRight();

void trackForward();
void trackBackward();

bool isWalkingBusy();
const char* getWalkingStateName();

#endif
