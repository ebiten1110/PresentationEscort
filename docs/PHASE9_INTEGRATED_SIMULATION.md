# Phase 9 — GPU人物検出・安全ゲート・模擬ESP32の一体結合

2026-10-20。実機USBシリアル・サーボ・ESP32フラッシュは実施しない。

## 単一プロセスの構成

```text
Piカメラ --- JPEG/TCP ---> Windows RTX4060/YOLO
                               |
                               v
                       DetectionResult
                               |
Pi内: 操作コンソール ----> PiMotionGateway (独立50ms監視)
                               |
                       RemotePersonController
                               |
                       V883PersonDualLoop
                               |
                          VirtualEsp32
                     (模擬ACK / STATUS / Head / Walking / STOP)
                               |
                      Esp32FeedbackPoller
                               |
                       PiMotionGateway
```

- `virtual_esp32.py`: シリアルプロトコルの模擬送受信。ARM/ACK/PING/750msロスト/頭角度/旋回/STOP/RESETをメモリ上で再現
- `pi_integrated_simulation.py`: カメラGPU Worker / UI / PiMotionGateway / V8.8.3プランナー / VirtualEsp32 / 独立テレメトリ受信スレッドを統合
- Phase 7と同じローカルHTTPサーバー・Bearerトークン・SSHトンネル。画面では `VIRTUAL:ARMED` のように模擬データと分かる表示を使用
- 新規 `--stage integrated` は**ESP32やUSBを一切開かない**。`--serial-port`、`--stage hardware` には対応しない
- Piの安全ゲート `request_head_pose()` 実行後も頭部だけが動作中という状態を保持し、人物ロスト・モード終了・通信断時にSTOPを送るように改善
- AUTO/MANUALは模擬ESP32のARM_ACCEPTED + ARMEDを受け取った後にのみ切り替わる。UIはハンドシェイク中を「応答待ち」と表示
- 前進・後退は禁止。USB切断模擬時にはESTOPをラッチし、勝手な再ARMをしない

## 使い方

オフライン監査（初期値）:

```bash
python3 experiments/pc_gpu/pi_deployment_launcher.py
```

統合シミュレーション（カメラ・ネットワークも開かない）:

```bash
python3 experiments/pc_gpu/pi_deployment_launcher.py --stage integrated
```

実Piカメラ→PC RTX4060の人物検出も利用する場合（ただし模擬ESP32のみ）:

```bash
python3 experiments/pc_gpu/pi_deployment_launcher.py --stage integrated --pc-host 10.17.6.97
```

PC IPは前回の接続例。現在の環境に合わせて指定してください。

PiのローカルWebコンソールへはWindows端末からSSHトンネルで接続:

```powershell
ssh -L 8766:127.0.0.1:8766 pi@pyiot014.local
```

ブラウザで http://127.0.0.1:8766/ を開き、Pi側端末に表示されるトークンで認証。

**重要:** GUIで実行した左右旋回やESTOPはVirtualEsp32の内部状態とメモリログを変化させるだけです。ロボットのモーターは動きません。表示される成功は実機動作の成功ではありません。

## ソフトウェアテスト

- HTTP API認証とオリジン/Host制限を保持
- 初期SAFE／FW DISARMED／人物不在でAUTO拒否
- 映像人物位置→追従計算→Gateway→模擬HEAD_POSE_SET/TRACK_LEFT/RIGHT
- ファームウェアACK待ち／750ms watchdog／デッドマン／STOP
- **頭部追従のみ実行中の対象ロスト→STOP発行**
- 模擬UART切断→ESTOP、ESTOP解除後もSAFE→明示ARM必須
- AUTOとMANUALの切替、Pi通信断時のAUTO停止とMANUAL独立性
- `--stage integrated` がリアルUSBシリアルを開かないことを確認

## 実機の重要な未確認事項

- 現物ファームウェアがV1対応かどうかは未確認
- 人物検出の遅延、PiのCPU/Wi-Fi負荷、ESP32実シリアルSTATUSの実行タイミング
- 歩行ルーチンの実際の停止時間、サーボ機械負荷、転倒リスク
- 実機独立電源遮断、現物E-STOP、配線・電源校正、旧版リカバリー
- **新ESP32を書き込んで旧Piプログラムを使うことはできない**。実機確認まではフラッシュ禁止

## 次の工程

完全版デモのための実機安全手順と起動条件を再確認し、対応Pi・ESP32ファームウェアの配備計画を固める。実機出力は安全確認後に明示的に段階開放する。
