# Phase 6 — ESP32独立通信ウォッチドッグ（2026-10-20）

## 制御仕様

| 項目 | 内容 |
|---|---|
| 初期状態 | DISARMED、モーター・頭部追従命令拒否 |
| ARM | SAFETY_ARMで明示的に許可、IDLE時のみ受理 |
| Heartbeat | PiからSAFETY_PINGを約150ms間隔で送信 |
| ESP32 watchdog | 750msを超えるとLATCHED、stopWalking()を要求 |
| STOP | いつでも受理、DISARMEDへ。LATCHEDは解除不可 |
| ESTOP | SAFETY_ESTOPでLATCHEDへ |
| RESET | SAFETY_RESETでLATCHED→DISARMEDのみ、再ARMは別途 |
| 確認 | SAFETY_STATUS → [Safety] State: ARMED/DISARMED/LATCHED |

## ファイル

- esp32/src/safety_watchdog.h — Arduino非依存の通信監視状態機械
- esp32/src/safety_runtime.cpp/.h — 現行ESP32モーション／頭部停止関数との連携
- esp32/src/main.cpp / serial_receive.cpp — 受信前にタイムアウト評価、すべての動作命令を許可ゲートへ通す
- experiments/pc_gpu/esp32_safety_link.py — Piで明示的な安全ARM応答と鮮度の解析
- experiments/pc_gpu/motion_gateway.py — Pi watchdogスレッドがPINGを送信・安全ゲート管理
- experiments/pc_gpu/esp32_feedback_poller.py — STATUSの戻りを安全ゲートへ配送
- ESP32の物理的旋回マッピング・モーション波形・サーボ角度は変更していない

## 運用上の重大な注意点

**新ESP32版を書き込むと旧V8.8.3のPiからの動作命令は拒否される。** 必ずPi/ESP32双方の対応版とバックアップを準備し、同時に段階的に移行すること。実機への書き込みはユーザーが検証可能になるまで禁止。

旧ESP32はSAFETY_ARM受諾応答を返さないため、新PiはARMを許可しない。安全ゲートを迂回して使わない。

STOPは転倒を防ぐ保証ではない。既存の前後移動TRACK_FORWARD/BACKWARDは現在の動作パルス完了後に停止。Pi側では前後移動を初期OFFにしたまま。

ESP32のloop()が停止するようなESP32自身のハング、電源断、サーボ暴走をこのソフトウェアwatchdogだけでは救済できない。独立した物理電源遮断を検討すること。

起動時initServoControl()がcenterAllServos()を呼ぶ既存仕様は残るので、サーボ電源は外してソフトウェアの初期接続確認を行うこと。

USBシリアルはPiだけが書き込める信頼済みローカル接続前提で、ネットワーク認証は別途実装。

## 検証

- こちらで実施：C++ネイティブwatchdog状態遷移、PlatformIO ESP32ビルド、Pythonユニットテスト・モックUART
- 後日に保留：ESP32フラッシュ、実UART ACK/PIO通信、サーボ駆動、Pi切断時の姿勢・停止遅延、STOPの実機応答、実電源遮断
- ソフトウェアが通っても実機安全性や28日の完成を保証しない

## 未完成

- 実機配備ランチャー、認証済みAUTO/MANUAL UI、実機への書き込みと配線照合
- ESP32からの確実なモーション受理ACK（今は既存Walkingログを使用）
- 実機の距離・上下制御の再キャリブレーション
