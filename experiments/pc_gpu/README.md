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

## Phase 4: V8.8.3相当の二重ループ接続（Dry-runのみ）

- `v883_person_dual_loop.py`: V8.8.3の頭部Yaw/Pitch、画像座標系、物理確認済み本体旋回方向・しきい値を移植したプランナー。
- `pi_tracking_dry_run.py`: PC/GPU→Pi人物座標→V8.8.3相当のHead/Body計画まで算出する。**実機角度の読み戻しが未接続なので本体旋回は常にBLOCKED**。
- `tests/test_v883_person_dual_loop.py`: オフラインで左右符号、画角による誤差、角度制限、確認フレーム、Busy interlock、フィードバック要件を検証。
- `tests/test_remote_loop_integration.py`: 模擬YOLO検出→頭部/体プランナー→PiMotionGateway（メモリシンク）の結合検証。
- 設計上の制約: [../../docs/PHASE4_V883_REMOTE_CONTROL_ADAPTER.md](../../docs/PHASE4_V883_REMOTE_CONTROL_ADAPTER.md)

実機には命令を出さない。元V8.8.3の起動スクリプトは変更していない。

## Phase 5: ESP32テレメトリと頭部制御ゲート（ソフトウェア検証）

- `esp32_telemetry.py`: 既存の`[HeadState]`、`Walking State:`、`[Walking] State:`、`Busy:`を解析。状態が古ければ不許可
- `esp32_feedback_poller.py`: 同じシリアルポートに対し`STATUS`を定期照会し、読み取り結果をプランナーへ渡す。**インスタンス作成だけではシリアルポートを開かない**
- `serial_command_sink.py`: `HEAD_POSE_SET`（Yaw±20°、Pitch±12°）、`STATUS`、`HEAD_STATUS`を追加。非同期読み取りは`read_available`
- `motion_gateway.py`: AUTO + 有効人物検出状態のみ頭部角度命令を認可。ESTOP/SAFEでは頭部を動かさない
- `remote_person_controller.py`: 検出→目標計算→新しいESP32状態→安全ゲートをまとめた接続部。**`enable_actuation=False`が初期値**
- テスト：`tests/test_esp32_telemetry.py`、`tests/test_phase5_head_gate.py`、`tests/test_esp32_feedback_poller.py`

注意：以前のESP32ファームウェアは引き続き未変更です。ここでのSTATUS/HEAD_STATEは**実機の新しいファームウェアを書き込まず既存の出力を利用**します。ただし、現在実機に入っているファームウェアがGitHub版と一致する保証はありません。

`enable_actuation=True`は実機安全停止とシリアル読み戻しの検証前に有効化しないこと。ESP32独立watchdog・Piクラッシュ時の停止はまだ実装・検証していません。実機では安全に利用できるよう、別のハードウェア停止装置も必要です。

## Phase 6 — ESP32独立通信ウォッチドッグ

- ESP32の新ファームウェアは起動時DISARMED、750ms通信途絶でLATCHED。
- PiはARM_ACCEPTEDとState: ARMEDの確認後にのみAUTO/MANUALへ移行し、監視スレッドが定期PING。
- 片側だけ更新すると旧版は走行できない。実機のフラッシュは未実施。
- 詳細: ../../docs/PHASE6_ESP32_WATCHDOG.md

## Phase 7：Piオペレーター操作画面（初期は実機出力OFF）

- 起動: `python3 experiments/pc_gpu/pi_operator_console.py`
- PC↔Pi人物検出を接続したDry-run起動: `python3 experiments/pc_gpu/pi_operator_console.py --pc-host <GPU_PC_IP>`
- Pi上の待受は **127.0.0.1:8766限定**。LAN全体に公開しない
- PCからは `ssh -L 8766:127.0.0.1:8766 pi@<PI_IP>` でトンネルを作り、ブラウザで `http://127.0.0.1:8766/` を開く
- Piターミナルに表示されるランダムなトークンを画面に入力（ブラウザ内だけに保持）
- AUTO／MANUAL切替・左右Hold操作・STOP・ESTOP・RESET・GPU/人物/RTT/最後の安全命令を表示
- マウス/タッチ操作の終了・画面移動・ブラウザ非表示でSTOP要求を送る。さらにPi側MANUALデッドマン350msで停止
- **操作コマンドはMemoryCommandSinkにのみ記録**。SerialCommandSinkを作成しないためESP32は操作されない
- **前後移動はこのUIで提供しない**。対象距離が未校正であるため
- CLIトークン／ローカルSSHトンネルのみ。公開HTTP・VPNなしLAN配信・ポート開放は禁止
- APIテスト: `tests/test_operator_console.py` をGitHub CIで実施。実機UIの表示・タッチ操作は後日

Phase 7設計: [../../docs/PHASE7_OPERATOR_CONSOLE.md](../../docs/PHASE7_OPERATOR_CONSOLE.md)


## Phase 8: 配備前点検（実機モーター出力は引き続き無効）

- デフォルトはデバイスを一切使わない静的整合性監査：python3 experiments/pc_gpu/pi_deployment_launcher.py
- コンソール起動：python3 experiments/pc_gpu/pi_deployment_launcher.py --stage console
- PiのWebカメラ→PC検出を接続：上記に --pc-host <PC IPv4> を追加（実機サーボは無関係）
- ESP32のファームウェア診断はサーボ電源OFF、実機検証再開後に限り使用：--stage diagnostic --serial-port <ESP32シリアル> --servo-power-off-confirmed
- SAFETY_VERSIONの返信は V1 / TimeoutMs=750 と DISARMED の両方がそろった場合のみ互換と判定する
- **走行や頭部を動かす起動モードはまだ作らない**。Arduino/C++のwatchdogテスト成功と実機停止安全性を混同しない
- 詳細：docs/PHASE8_DEPLOYMENT_PRECHECK.md

## Phase 9：単一プロセス統合シミュレーション

- `pi_integrated_simulation.py`: UI/API、GPU検出受信、V8.8.3人物追従計算、Pi安全ゲート、`VirtualEsp32`状態フィードバックを同一Piプロセスで統合
- `virtual_esp32.py`: ESP32 V1 ACK、750ms watchdog、Busy/Head状態、旋回/STOPを**メモリ上だけで**再現
- 起動: `python3 experiments/pc_gpu/pi_deployment_launcher.py --stage integrated`
- 実PC/GPUへ接続: `--stage integrated --pc-host <Windows PCのIP>`（ESP32物理デバイスは使わない）
- 操作画面はPhase7と共通、ファームウェア状態には**VIRTUAL**と表示
- `--serial-port` と同時使用不可、ハードウェア動作ステージなし
- 設計: [../../docs/PHASE9_INTEGRATED_SIMULATION.md](../../docs/PHASE9_INTEGRATED_SIMULATION.md)

## Phase 10：バージョン照合・ソース/履歴バックアップ・実機判定

- 基準: `docs/PHASE10_CODE_BASELINE.json`（Phase 9の重要15ファイルをGit blob SHA-1で固定）
- Piで確認: `python3 experiments/pc_gpu/phase10_release.py check`
- ソース/履歴保存: `python3 experiments/pc_gpu/phase10_release.py backup --dest ~/PresentationEscort-backups`
- 保存後のSHA-256確認: `python3 experiments/pc_gpu/phase10_release.py verify --backup-dir <保存先の実フォルダ名>`
- 実機の合格表: `docs/PHASE10_HARDWARE_GATE_TEMPLATE.json`
- オフライン判定: `python3 experiments/pc_gpu/phase10_gate.py --report docs/PHASE10_HARDWARE_GATE_TEMPLATE.json` （初期NO-GO）
- Git履歴とソースのバックアップは**現在ESP32に焼かれているファームウェアを保存しない**
- 既存本番Pi V8.8.3、実ESP32、GitHub mainは変更せず、motor出力は初期OFFのまま
- 手順とGO/NO-GO条件: [../../docs/PHASE10_RELEASE_READINESS.md](../../docs/PHASE10_RELEASE_READINESS.md)

## Phase 11: 発表者ロック（人の明示選択・模擬ESP32のみ）

- `pc_detector_server.py`が新V2 protocolでYOLO検出の人物候補すべてを送信（PCもPiも開発版へ更新）
- `pi_presenter_lock_demo.py`は実機シリアルを使わず、VirtualEsp32だけに追従指令を送信
- 起動例: `python3 experiments/pc_gpu/pi_presenter_lock_demo.py --pc-host 10.17.6.97`
- ブラウザの**発表者ロック**で現物の位置に合う候補を手動選択。位置は左右・上下％で表示される。人の本人認証ではない。
- 大きな移動・混雑・交差・フレーム欠落・ロスト・通信断はAUTOを止め、次の追従には再選択と明示ARMが必要
- **V1サーバーや旧版の単一人物結果では安全なAUTOに進めない**
- Phase10の不変ソフトウェアベースラインはそのまま。実機複数人AUTOは引き続きNO-GO。
- 設計・制限: [../../docs/PHASE11_PRESENTER_LOCK.md](../../docs/PHASE11_PRESENTER_LOCK.md)
