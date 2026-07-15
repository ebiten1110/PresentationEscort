#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

// PCA9685とサーボ制御の初期化
void initServoControl();

// 指定チャンネルのサーボを指定角度へ動かす
void setServoAngle(int ch, double angle);

// 4つの下半身サーボを0度にする
void centerLegServos();

// 頭サーボを0度にする
void centerHeadServo();

// 全サーボを初期姿勢にする
void centerAllServos();

// 下半身4サーボをまとめて動かす
void setLegPose(
  double leftHip,
  double leftLeg,
  double rightHip,
  double rightLeg
);

// 頭サーボを動かす
void setHeadYaw(double angle);

// サーボ出力を停止する
void stopServo(int ch);

// 全サーボ出力を停止する
void stopAllServos();

#endif