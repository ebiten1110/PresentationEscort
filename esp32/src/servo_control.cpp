#include "servo_control.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "config.h"

// PCA9685インスタンス
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// 角度を安全範囲に制限する
static double clampAngle(double angle) {
  if (angle < SERVO_MIN_ANGLE) {
    return SERVO_MIN_ANGLE;
  }

  if (angle > SERVO_MAX_ANGLE) {
    return SERVO_MAX_ANGLE;
  }

  return angle;
}

// 角度をPCA9685のPWM値に変換する
static int angleToDuty(double angle) {
  angle = clampAngle(angle);

  double dutyRatio =
    DUTY_MIN + ((angle + 90.0) / 180.0) * (DUTY_MAX - DUTY_MIN);

  int duty = (int)(dutyRatio * 4096.0);

  if (duty < 0) {
    duty = 0;
  }

  if (duty > 4095) {
    duty = 4095;
  }

  return duty;
}

// PCA9685とサーボ制御の初期化
void initServoControl() {
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(SERVO_PWM_FREQ);

  delay(10);

  centerAllServos();
}

// 指定チャンネルのサーボを指定角度へ動かす
void setServoAngle(int ch, double angle) {
  if (ch < 0 || ch > 15) {
    Serial.print("[Servo] Invalid channel: ");
    Serial.println(ch);
    return;
  }

  double correctedAngle = angle + angleBias[ch];
  correctedAngle = clampAngle(correctedAngle);

  int duty = angleToDuty(correctedAngle);

  pwm.setPWM(ch, 0, duty);

  Serial.print("[Servo] CH=");
  Serial.print(ch);
  Serial.print(" angle=");
  Serial.print(angle);
  Serial.print(" bias=");
  Serial.print(angleBias[ch]);
  Serial.print(" corrected=");
  Serial.print(correctedAngle);
  Serial.print(" duty=");
  Serial.println(duty);
}

// 下半身4サーボをまとめて動かす
void setLegPose(
  double leftHip,
  double leftLeg,
  double rightHip,
  double rightLeg
) {
  setServoAngle(CH_LEFT_HIP, leftHip);
  setServoAngle(CH_LEFT_LEG, leftLeg);
  setServoAngle(CH_RIGHT_HIP, rightHip);
  setServoAngle(CH_RIGHT_LEG, rightLeg);
}

// 4つの下半身サーボを0度にする
void centerLegServos() {
  setLegPose(
    STAND_ANGLE_LEFT_HIP,
    STAND_ANGLE_LEFT_LEG,
    STAND_ANGLE_RIGHT_HIP,
    STAND_ANGLE_RIGHT_LEG
  );
}

// 頭サーボを動かす
void setHeadYaw(double angle) {
  setServoAngle(CH_HEAD_YAW, angle);
}

// 頭サーボを0度にする
void centerHeadServo() {
  setHeadYaw(STAND_ANGLE_HEAD_YAW);
}

// 全サーボを初期姿勢にする
void centerAllServos() {
  centerLegServos();
  centerHeadServo();
}

// 指定サーボのPWM出力を停止する
void stopServo(int ch) {
  if (ch < 0 || ch > 15) {
    Serial.print("[Servo] Invalid stop channel: ");
    Serial.println(ch);
    return;
  }

  pwm.setPWM(ch, 0, 0);

  Serial.print("[Servo] Stop CH=");
  Serial.println(ch);
}

// 全サーボ出力を停止する
void stopAllServos() {
  for (int ch = 0; ch < 16; ch++) {
    stopServo(ch);
  }
}