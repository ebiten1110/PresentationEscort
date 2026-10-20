# Phase 5 — ESP32 Head/BodyテレメトリとPi安全制御の結合

2026-10-20 / 実機未使用

## 既存ESP32ソースから確認したプロトコル

- `HEAD_STATUS`、`STATUS`を送ると`[HeadState] currentYaw=... targetYaw=... currentPitch=... targetPitch=... moving=YES/NO`が返る
- `STATUS`には `Walking State: IDLE`、`Busy: NO` が含まれる
- 旋回や歩行開始・終了時には `[Walking] State: TRACK_LEFT/RIGHT/IDLE`などが返る
- `HEAD_POSE_SET:<yaw>,<pitch>`は従来コードに実装済み。変更・上書きせずに利用する
- `STOP`は従来プログラムの制限（距離パルスを即時停止しない）を受ける

## Phase 5の変更

```text
Pi USB camera -> PC YOLO -> Pi PersonTarget
                                 |
                                 v
                       V883PersonDualLoop <---- ESP32 STATUS/HEAD_STATE
                                 |               ^
                                 v               |
                        PiMotionGateway    Esp32FeedbackPoller
                         /             \            |
                  Head safety gate   Body gate       |
                         \             /            |
                          SerialCommandSink ---------+
                                 |
                                ESP32
```

- Pi側のみ新規ソフトウェアを作成。従来Productionは維持
- `Esp32Telemetry`でUARTテキストを有限バッファで解析、head TTL 0.45s、walking/busy TTL 0.40s。異常値は破棄
- Bodyは新鮮なHeadYawに加えて、walking IDLEとBusy:NOが両方新鮮なときだけプランナーから提案可能
- HEAD_POSE_SETはAUTO状態かつ新鮮な検出のときだけPiGateway経由で送信可能。ESP32角度情報の受信も運用側で確認してから送る
- モーター送信を行わない設計がデフォルト。ユーザーが実機を扱えるまで有効化しない
- MANUAL制御はAI通信が切れても独立可能な論理設計だが、認証済みの操作UIは別途実装必要

## 安全上の既知課題

- 現行ESP32はACK付きの信頼できるコマンド受理通知を定義しておらず、STATUSとログから推定している
- Piプロセス停止・USB抜去・Piフリーズ時、Pi側STOPがESP32に届く保証はない
- ESP32側の独立通信watchdogはまだ実装していない（次工程として検討）
- 全身BBoxからの距離・Pitch調整は実機再校正が必要
- 現行ロジックは真のReIDや本人識別ではない
- 新しい`RemotePersonController`は実機ランチャーには組み込んでいない。動作中のV8.8.3へコードを上書きしない

## ソフトウェア検証

`python -m unittest discover -s experiments/pc_gpu/tests -v`

既存テスト＋テレメトリ分割/破損/タイムアウト/状態切替/頭部許可/コマンド範囲/疑似UART結合をGitHub Actionsで実行。
実機テストはユーザー指示まで保留。
