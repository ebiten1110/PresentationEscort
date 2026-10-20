# Phase 8 — 配備前点検・旧版復旧手順（2026-10-20）

## 現在の実装範囲

**実機モーターを動かすランチャーはまだ提供しません。** 物理非常停止、サーボ可動域、電源定格、ファームウェア一致を確認できていないためです。提供範囲はオフライン点検、モーターOFF操作画面、サーボ外部電源OFF時のESP32診断まで。

変更箇所：
- experiments/pc_gpu/pi_deployment_launcher.py: audit / console / diagnostic の3段階
- experiments/pc_gpu/firmware_preflight.py: ESP32安全プロトコルV1およびDISARMED検証
- esp32/src/safety_runtime.cpp: SAFETY_VERSIONへの応答（新ファームウェア。実機未書込）
- serial_command_sink.py: SAFETY_VERSIONコマンド
- 旧V8.8.3は維持。本番モーターONフラグはない

## 1. デバイスを開かないオフライン点検

Piの開発用クローンで以下を実行します。

    cd ~/PresentationEscort-demo
    git fetch origin
    git switch feature/pc-gpu-follow-2026-10-28
    git pull --ff-only
    python3 experiments/pc_gpu/pi_deployment_launcher.py

標準のauditモードはソースの安全機構を静的に検査します。ESP32/カメラ/LANは使いません。静的検査は実機安全性を保証しません。

## 2. 操作画面のDry-run

    python3 experiments/pc_gpu/pi_deployment_launcher.py --stage console

Pi→GPU PCの人物検出まで動かす場合:

    python3 experiments/pc_gpu/pi_deployment_launcher.py --stage console --pc-host 10.17.6.97

PCのIPv4は前回の値なので、必要ならWindowsの ipconfig で再確認してください。

別のWindows PowerShellでSSHトンネル:

    ssh -L 8766:127.0.0.1:8766 pi@pyiot014.local

Windowsのブラウザで http://127.0.0.1:8766/ を開き、Piコンソールに表示されたトークンを入力します。モーターには**一切接続しません**。

## 3. 将来のESP32診断（現時点では実機テスト保留）

先に必ず外部サーボ電源および駆動電源をOFFにし、ロボットを支持台に固定。USBの接続・OPENでESP32が再起動し、現行コードがサーボ初期位置を指令する場合があります。配線とPCA9685のVCC/V+、ボードの型番を照合してください。

Pi/PCのほかのシリアルプログラムを止めた状態でのみ:

    python3 experiments/pc_gpu/pi_deployment_launcher.py \
      --stage diagnostic \
      --serial-port /dev/ttyUSB0 \
      --servo-power-off-confirmed

ポート名は実機に合わせて変更。診断はUSB開通後2.5秒待機し、SAFETY_VERSIONとSAFETY_STATUSだけ送信します。ARM、移動、PING、サーボ角度命令は送信しません。

判定：
- PASS: PROTOCOL_V1_DISARMED — **プロトコル互換のみ確認**、物理的安全性は別
- BLOCKED: FIRMWARE_VERSION_UNCONFIRMED — 旧版または返信なし、動作不可
- BLOCKED: FIRMWARE_PROTOCOL_MISMATCH — 期待したV1と異なる、動作不可
- BLOCKED: FIRMWARE_NOT_DISARMED — ARMED/LATCHED状態、動作不可
- UART異常 — エラーで終了、動作不可

## 4. ロールバック・旧版の保護

- Piの旧 ~/PresentationEscort を保存し、開発用 ~/PresentationEscort-demo と区別する
- mainのV8.8.3および元ESP32ソースは未変更。新ESP32ファームウェアは開発ブランチのみ
- **実機に入っている旧ファームウェアがGitHubのコードと一致する保証はない**ため、書込前に版・ボード構成・モーション・サーボ校正・元の書込イメージのバックアップを確認
- 対応する旧ファームウェアのバイナリを保有していない場合、確実な元通りへの復旧は保証できない
- 新ESP32版は起動時DISARMED。旧Piソフトのみで走行させられないので、Pi/ESP32を片側だけ更新してはいけない
- トラブル時はサーボ外部電源を安全に切断、新Piプロセスを止め、旧ソースと対応する保存済みイメージから復旧する
- ソフトウェアSTOPだけで転倒や物理危険を防げるとは限らない。独立した電源停止を準備する

## 5. 実機確認項目（すべて未実施）

| ID | 内容 |
|---|---|
| P1 | 配線・ESP32の現在版・サーボ供給電圧・機械原点 |
| P2 | SAFETY_VERSION + DISARMED、旧版との互換差 |
| P3 | 同一Pi USBシリアルでハートビート/Head/Busy読み戻し |
| P4 | サーボ電源OFFでSTOP/ESTOP状態遷移 |
| P5 | 支持台上でYaw/Pitch制限、意図せぬ動きなし |
| P6 | 本体左右方向・USB抜去時STOP・姿勢 |
| P7 | 誤検出・複数人・対象ロスト・Wi-Fi断 |
| P8 | 展示用5分間連続デモ |

実機完成の判定は上記の実機試験を終えるまで保留。

## 次工程

単一UART所有、プロトコル互換性確認、Head/Body/STOP安全ゲート、GPU通信、操作UIを一つにまとめた実機用ランチャーを開発する。走行許可は現物安全対策が整ってから追加する。
