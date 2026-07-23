#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
// PresentationEscort - ESP32 設定ファイル
// Phase 1: PCA9685 + 4サーボ原点調整用
// =====================================================

// ---------- シリアル通信 ----------
#define SERIAL_BAUD 115200

// ---------- PCA9685設定 ----------
#define PCA9685_ADDRESS 0x40
#define SERVO_PWM_FREQ 50

// ---------- サーボPWM設定 ----------
// 学校資料ベース
// サーボの動きが狭い/広すぎる場合はここを微調整
#define DUTY_MIN 0.022
#define DUTY_MAX 0.130

// ---------- サーボチャンネル設定 ----------
// L1: 左腰
// L2: 左足
// R1: 右腰
// R2: 右足
#define CH_LEFT_HIP   1   // L1 左腰
#define CH_LEFT_LEG   0   // L2 左足
#define CH_RIGHT_HIP  15  // R1 右腰
#define CH_RIGHT_LEG  14  // R2 右足


// ---------- サーボ角度制限 ----------
#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE  90

// ---------- 原点補正 angleBias ----------
// まずは全部 0.0 で開始。
// ロボットがまっすぐ立たない場合、該当チャンネルだけ ±1.0 ずつ調整する。
// 例：左足が外側に傾く → CH_LEFT_LEG の値を 1.0 / -1.0 で試す。
static double angleBias[16] = {
  0.0, 0.0, 0.0, 0.0,  // ch 0 - 3
  0.0, 0.0, 0.0, 0.0,  // ch 4 - 7
  0.0, 0.0, 0.0, 0.0,  // ch 8 - 11
  0.0, 0.0, 0.0, 0.0   // ch 12 - 15
};

// ---------- 初期姿勢 ----------
#define STAND_ANGLE_LEFT_HIP    0
#define STAND_ANGLE_LEFT_LEG    0
#define STAND_ANGLE_RIGHT_HIP   0
#define STAND_ANGLE_RIGHT_LEG   0
#define STAND_ANGLE_HEAD_YAW    0

// ---------- LED設定 ----------
// Phase 1では未使用。
// 後で白色LEDをGPIO制御するときに使う。
#define LED_PIN 25

// ---------- トグルスイッチ設定 ----------
// Phase 1では未使用。
// 後で手動/追従モード切替に使う。
#define SWITCH_PIN 26

// ============================================================
// 頭サーボの滑らか制御
// ============================================================

// PCA9685の接続チャンネル
#define HEAD_YAW_CHANNEL       2
#define HEAD_PITCH_CHANNEL     3

// 中央角度
#define HEAD_YAW_CENTER_ANGLE      0.0f
#define HEAD_PITCH_CENTER_ANGLE    0.0f

// 安全な最大可動範囲
#define HEAD_YAW_MIN_ANGLE       -20.0f
#define HEAD_YAW_MAX_ANGLE        20.0f

#define HEAD_PITCH_MIN_ANGLE     -12.0f
#define HEAD_PITCH_MAX_ANGLE      12.0f

// HEAD_UP / HEAD_DOWNを1回受け取ったときに
// 目標角度を何度変更するか
#define HEAD_PITCH_NUDGE_ANGLE     1.0f

// サーボを1回の更新で何度動かすか
#define HEAD_SMOOTH_STEP_ANGLE     0.5f

// サーボ角度を更新する間隔
#define HEAD_UPDATE_INTERVAL_MS    30

// 実機で上下が逆だった場合は符号を入れ替える
#define HEAD_PITCH_UP_DIRECTION   -1.0f
#define HEAD_PITCH_DOWN_DIRECTION  1.0f

#endif