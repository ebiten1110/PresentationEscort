#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>


// ============================================================
// PCA9685・サーボ初期化
// ============================================================

void initServoControl();


// ============================================================
// 基本サーボ制御
// ============================================================

void setServoAngle(
  int ch,
  double angle
);


// ============================================================
// 下半身
// ============================================================

void setLegPose(
  double leftHip,
  double leftLeg,
  double rightHip,
  double rightLeg
);

void centerLegServos();


// ============================================================
// 頭
// ============================================================

void setHeadYaw(double angle);
void setHeadPitch(double angle);

void centerHeadServos();

// 以前のコードとの互換用。
void centerHeadServo();


// ============================================================
// 全サーボ
// ============================================================

void centerAllServos();

void stopServo(int ch);
void stopAllServos();


#endif