#ifndef HEAD_CONTROL_H
#define HEAD_CONTROL_H

#include <Arduino.h>

void initHeadControl();
void updateHeadControl();

void headLeft();
void headRight();
void headUp();
void headDown();
void headCenter();

void setHeadYawTarget(float yawAngle);
void setHeadPitchTarget(float pitchAngle);
void setHeadPoseTarget(float yawAngle, float pitchAngle);
void printHeadStatus();

float getCurrentHeadYaw();
float getCurrentHeadPitch();
float getTargetHeadYaw();
float getTargetHeadPitch();
bool isHeadMoving();

#endif
