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

## Pi側のSTOP送信ゲート（2026-10-20追加）

- `motion_gateway.py`: SAFE/AUTO/MANUAL/ESTOP状態に従うコマンド送信ゲート。GPU応答待ちとは独立のwatchdogスレッド（標準監視頻度50ms）。
- `serial_command_sink.py`: 許可されたモーター命令のみESP32の115200bps USBシリアルへ送信。実ポートは**オブジェクト生成時のみ開く**。
- `pi_safe_stop_probe.py`: Pi→ESP32のSTOPのみ確認する小ツール。デフォルトは**完全なメモリDry-run**。
- `pi_tracking_dry_run.py`: MotionGateway/MemoryCommandSinkを使用するので、これまで同様**ESP32へのアクセスはゼロ**。

### STOPのみのソフトウェアDry-run
```bash
python3 experiments/pc_gpu/pi_safe_stop_probe.py
```

### 実機用の予約手順（ユーザーの実機テスト再開指示まで実行しない）
サーボの外部電源をOFFにし、現在のESP32ファームウェアとUSBポート名を確認後、明示した場合に限り:
```bash
python3 experiments/pc_gpu/pi_safe_stop_probe.py --enable-serial --port /dev/ttyUSB0
```
注意：ポートOPEN時ESP32が再起動し、サーボの初期姿勢指令が走るコードがある。**外部サーボ電源を必ず切る。** 出力側の電気的状態やSTOPの実際の効果はまだ未確認。

### 重要な制約
- Pi側watchdogは**Pythonプロセスが動いている間だけ**機能する。PiフリーズやUSB抜去でSTOPできるとは限らない。ESP32側の独立watchdogが今後必要。
- ESP32 `stopWalking()` は前後TRACKを現在の短いパルス終了まで止めないことがあるため、物理的緊急停止と等価ではない。
- 停止コマンドはシリアルwriteが受け付けられたことまでしか検査できず、ESP32側の受理ACKはまだない。
- **自律モーションを有効にするエントリーポイントはまだ作っていない。** モーター稼働前にhead/body二重ループの統合と検証が必要。
- MANUALはAI推論リンクが途絶えても動作可能とする設計だが、将来UIには認証と別経路の操作デッドマンが必要。
