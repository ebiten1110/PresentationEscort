# PC GPU person-detection experiment

Production V8.8.3を変更せず、Pi USBカメラ映像をPC/GPUへ送り、人物検出とBBox座標変換を行う開発用モジュール。

構成: USB Camera → Raspberry Pi → JPEG/TCP → PC GPU → DetectionResult/JSON → Raspberry Pi

## 実装一覧
- pc_detector_server.py: YOLOによる人物検出
- person_tracker.py: 位置/サイズの連続性ガード（本人識別ではない）
- pi_stream_client.py: Pi→PC映像送信と結果受信
- person_target_adapter.py: PersonBBoxから頭部付近のAimと誤差を計算
- pi_tracking_dry_run.py: 座標とSafetyManager状態を表示（ESP32にはアクセスしない）
- safety_manager.py: SAFE / AUTO / MANUAL / ESTOP論理状態とコマンド許可制御
- check_pc_environment.py: PC/GPU環境チェック

接続手順は SETUP_WINDOWS_PI.md を参照。

## テスト実行方針（10/20更新）
- **ソフトウェア単体テストとGitHub CIは、アシスタントが実行して結果を記録する。**
- **Pi/ESP32の実機検証はユーザーが再開を指示するまで延期する。**
- 詳細: ../../docs/TEST_PLAN_2026-10-20.md
- 既存実測: ../../docs/TEST_RESULTS_2026-10-19.md

## 安全上の制約
- SafetyManagerは動作コマンドを論理的に拒否するだけであり、ESP32へSTOPを送らない。
- ESP32側の独立watchdog、非常停止、認証されたMANUAL操作経路は今後実装する。
- 既存ESP32の前後パルスはSTOPが届いてもそのパルス終了後の停止となる。
- 初期状態で前後移動はallow_translation=Falseとし、実機校正まで無効。
- この段階でのソフトウェアテスト合格は、実機安全性を保証しない。
