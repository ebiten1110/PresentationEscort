#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
// PresentationEscort - ESP32 設定ファイル
// PCA9685 + 二足歩行 + 頭サーボ + LED
// =====================================================


// =====================================================
// シリアル通信
// =====================================================

#define SERIAL_BAUD 115200


// =====================================================
// PCA9685
// =====================================================

#define PCA9685_ADDRESS 0x40
#define SERVO_PWM_FREQ 50


// =====================================================
// サーボPWM
// =====================================================

// 学校資料を基準にしたPWM範囲。
// 実機のサーボ可動域が狭い、または広すぎる場合に調整する。
#define DUTY_MIN 0.022
#define DUTY_MAX 0.130


// =====================================================
// PCA9685チャンネル
// =====================================================

// 下半身
#define CH_LEFT_HIP   1
#define CH_LEFT_LEG   0
#define CH_RIGHT_HIP  15
#define CH_RIGHT_LEG  14

// 頭
//
// platformio.iniのbuild_flagsで同名マクロが定義されていても
// 再定義エラー・警告にならないよう#ifndefで保護する。
#ifndef CH_HEAD_YAW
#define CH_HEAD_YAW 2
#endif

#ifndef CH_HEAD_PITCH
#define CH_HEAD_PITCH 3
#endif

// 以前のコードとの互換用別名
#ifndef HEAD_YAW_CHANNEL
#define HEAD_YAW_CHANNEL CH_HEAD_YAW
#endif

#ifndef HEAD_PITCH_CHANNEL
#define HEAD_PITCH_CHANNEL CH_HEAD_PITCH
#endif


// =====================================================
// サーボ角度制限
// =====================================================

#define SERVO_MIN_ANGLE -90.0
#define SERVO_MAX_ANGLE  90.0


// =====================================================
// 原点補正
// =====================================================

// ロボットがまっすぐ立たない場合や、頭の中央がずれる場合は、
// 該当チャンネルの値を1.0度ずつ調整する。
//
// ch2 = 頭左右
// ch3 = 頭上下
static double angleBias[16] = {
  0.0, 0.0, 0.0, 0.0,  // ch 0 - 3
  0.0, 0.0, 0.0, 0.0,  // ch 4 - 7
  0.0, 0.0, 0.0, 0.0,  // ch 8 - 11
  0.0, 0.0, 0.0, 0.0   // ch 12 - 15
};


// =====================================================
// 初期姿勢
// =====================================================

#define STAND_ANGLE_LEFT_HIP    0.0
#define STAND_ANGLE_LEFT_LEG    0.0
#define STAND_ANGLE_RIGHT_HIP   0.0
#define STAND_ANGLE_RIGHT_LEG   0.0

#define STAND_ANGLE_HEAD_YAW    0.0
#define STAND_ANGLE_HEAD_PITCH  0.0


// =====================================================
// 頭サーボの1度追従制御
// =====================================================

// 安全な可動範囲
#define HEAD_YAW_MIN_ANGLE      -20.0f
#define HEAD_YAW_MAX_ANGLE       20.0f

#define HEAD_PITCH_MIN_ANGLE    -12.0f
#define HEAD_PITCH_MAX_ANGLE     12.0f

// コマンド1回につき、目標角度を1度変更する。
#define HEAD_YAW_NUDGE_ANGLE      1.0f
#define HEAD_PITCH_NUDGE_ANGLE    1.0f

// 実際のサーボ角度も1回につき1度だけ変更する。
#define HEAD_SMOOTH_STEP_ANGLE    1.0f

// 1度動かす間隔。
// 100msなら最大で1秒に10度動く。
#define HEAD_UPDATE_INTERVAL_MS 100UL

// サーボ取り付け方向。
// 実機で動作方向が逆の場合は、該当する2つの符号を入れ替える。
#define HEAD_YAW_LEFT_DIRECTION    -1.0f
#define HEAD_YAW_RIGHT_DIRECTION    1.0f

#define HEAD_PITCH_UP_DIRECTION    -1.0f
#define HEAD_PITCH_DOWN_DIRECTION   1.0f

// 1度ごとの適用ログ。
// trueにすると動作確認しやすいが、追従中は通信量が増える。
#define HEAD_APPLIED_ANGLE_LOG false

// 全サーボの詳細ログ。
// 歩行中に大量出力されるため、通常はfalse。
#define SERVO_DEBUG_LOG false


// =====================================================
// LED
// =====================================================

#define LED_PIN 25


// =====================================================
// トグルスイッチ
// =====================================================

#define SWITCH_PIN 26


#endif