#include "head_control.h"

#include "config.h"
#include "servo_control.h"

static double currentHeadYawAngle = HEAD_YAW_CENTER_ANGLE;
static double currentHeadPitchAngle = HEAD_PITCH_CENTER_ANGLE;

void initHeadControl() {
  currentHeadYawAngle = HEAD_YAW_CENTER_ANGLE;
  currentHeadPitchAngle = HEAD_PITCH_CENTER_ANGLE;

  setServoAngle(CH_HEAD_YAW, currentHeadYawAngle);
  setServoAngle(CH_HEAD_PITCH, currentHeadPitchAngle);

  Serial.println("[HeadControl] Initialized.");
}

void setHeadYawAngle(double angle) {
  currentHeadYawAngle = angle;
  setServoAngle(CH_HEAD_YAW, currentHeadYawAngle);

  Serial.print("[HeadControl] Yaw angle: ");
  Serial.println(currentHeadYawAngle);
}

void setHeadPitchAngle(double angle) {
  currentHeadPitchAngle = angle;
  setServoAngle(CH_HEAD_PITCH, currentHeadPitchAngle);

  Serial.print("[HeadControl] Pitch angle: ");
  Serial.println(currentHeadPitchAngle);
}

void headLeft() {
  Serial.println("[HeadControl] HEAD_LEFT");
  setHeadYawAngle(HEAD_YAW_LEFT_ANGLE);
}

void headRight() {
  Serial.println("[HeadControl] HEAD_RIGHT");
  setHeadYawAngle(HEAD_YAW_RIGHT_ANGLE);
}

void headUp() {
  Serial.println("[HeadControl] HEAD_UP");
  setHeadPitchAngle(HEAD_PITCH_UP_ANGLE);
}

void headDown() {
  Serial.println("[HeadControl] HEAD_DOWN");
  setHeadPitchAngle(HEAD_PITCH_DOWN_ANGLE);
}

void headCenter() {
  Serial.println("[HeadControl] HEAD_CENTER");

  setHeadYawAngle(HEAD_YAW_CENTER_ANGLE);
  setHeadPitchAngle(HEAD_PITCH_CENTER_ANGLE);
}

double getHeadYawAngle() {
  return currentHeadYawAngle;
}

double getHeadPitchAngle() {
  return currentHeadPitchAngle;
}