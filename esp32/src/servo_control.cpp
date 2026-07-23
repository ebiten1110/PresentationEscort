#include "servo_control.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "config.h"


// ============================================================
// PCA9685
// ============================================================

Adafruit_PWMServoDriver pwm = (
  Adafruit_PWMServoDriver(PCA9685_ADDRESS)
);


// ============================================================
// 内部関数
// ============================================================

static double clampAngle(double angle) {
  if (angle < SERVO_MIN_ANGLE) {
    return SERVO_MIN_ANGLE;
  }

  if (angle > SERVO_MAX_ANGLE) {
    return SERVO_MAX_ANGLE;
  }

  return angle;
}


static int angleToDuty(double angle) {
  angle = clampAngle(angle);

  const double dutyRatio =
    DUTY_MIN
    + (
      (angle + 90.0)
      / 180.0
    )
    * (
      DUTY_MAX
      - DUTY_MIN
    );

  int duty = static_cast<int>(
    dutyRatio * 4096.0
  );

  if (duty < 0) {
    duty = 0;
  }

  if (duty > 4095) {
    duty = 4095;
  }

  return duty;
}


// ============================================================
// 初期化
// ============================================================

void initServoControl() {
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(SERVO_PWM_FREQ);

  delay(10);

  centerAllServos();

  Serial.println("[Servo] Initialized.");
}


// ============================================================
// 基本サーボ制御
// ============================================================

void setServoAngle(
  int ch,
  double angle
) {
  if (ch < 0 || ch > 15) {
    Serial.print("[Servo] Invalid channel: ");
    Serial.println(ch);
    return;
  }

  const double correctedAngle = clampAngle(
    angle + angleBias[ch]
  );

  const int duty = angleToDuty(
    correctedAngle
  );

  pwm.setPWM(
    ch,
    0,
    duty
  );

  if (SERVO_DEBUG_LOG) {
    Serial.print("[Servo] CH=");
    Serial.print(ch);

    Serial.print(" angle=");
    Serial.print(angle, 2);

    Serial.print(" bias=");
    Serial.print(angleBias[ch], 2);

    Serial.print(" corrected=");
    Serial.print(correctedAngle, 2);

    Serial.print(" duty=");
    Serial.println(duty);
  }
}


// ============================================================
// 下半身
// ============================================================

void setLegPose(
  double leftHip,
  double leftLeg,
  double rightHip,
  double rightLeg
) {
  setServoAngle(
    CH_LEFT_HIP,
    leftHip
  );

  setServoAngle(
    CH_LEFT_LEG,
    leftLeg
  );

  setServoAngle(
    CH_RIGHT_HIP,
    rightHip
  );

  setServoAngle(
    CH_RIGHT_LEG,
    rightLeg
  );
}


void centerLegServos() {
  setLegPose(
    STAND_ANGLE_LEFT_HIP,
    STAND_ANGLE_LEFT_LEG,
    STAND_ANGLE_RIGHT_HIP,
    STAND_ANGLE_RIGHT_LEG
  );
}


// ============================================================
// 頭
// ============================================================

void setHeadYaw(double angle) {
  setServoAngle(
    CH_HEAD_YAW,
    angle
  );
}


void setHeadPitch(double angle) {
  setServoAngle(
    CH_HEAD_PITCH,
    angle
  );
}


void centerHeadServos() {
  setHeadYaw(
    STAND_ANGLE_HEAD_YAW
  );

  setHeadPitch(
    STAND_ANGLE_HEAD_PITCH
  );
}


void centerHeadServo() {
  centerHeadServos();
}


// ============================================================
// 全サーボ
// ============================================================

void centerAllServos() {
  centerLegServos();
  centerHeadServos();
}


// ============================================================
// PWM出力停止
// ============================================================

void stopServo(int ch) {
  if (ch < 0 || ch > 15) {
    Serial.print(
      "[Servo] Invalid stop channel: "
    );
    Serial.println(ch);
    return;
  }

  pwm.setPWM(
    ch,
    0,
    0
  );

  if (SERVO_DEBUG_LOG) {
    Serial.print("[Servo] Stop CH=");
    Serial.println(ch);
  }
}


void stopAllServos() {
  for (int ch = 0; ch < 16; ch++) {
    stopServo(ch);
  }
}