# Phase 17 — 見失った側への1回だけの探索旋回・展示前起動手順

対象: `feature/pc-gpu-follow-2026-10-28`。PC (YOLO pose) → Pi (USBカメラ、安全制御) → ESP32 (PWM/サーボ)。

## 今回追加した動作

1. 赤=STOP/SAFE、黄色=操作画面でジェスチャー待機中、白=ESP32のwatchdogがARMED。**白は人物の身元確認を意味しない。**
2. 両手を頭上に連続4フレーム→ロック。ロック成立後、3フレームの正常な検出とESP32からのARM確認で追従開始。
3. 人物がカメラ中心から少しずれたら首Yawで補正（実機用は校正済み±3°、Pitchは初期0°）。顔/肩の照準点、カメラ中央、矢印、誤差px、首／本体モードを画面へ表示。
4. 大きな横ずれでは、既存の新鮮なESP32頭部/IDLEフィードバックを条件に、左右の本体マイクロ旋回。
5. **横方向に寄ったロック対象を完全に見失い、次の有効なカメラフレームには誰も映っていない場合だけ**、直前の首と本体向きから決めた側へ**最大1回**のTRACK_LEFT/RIGHTマイクロ旋回を許可。探索窓は最大1.5秒。ESP32には連続回転指令を送らない。
6. 同じ幾何学的対象が3フレーム連続で戻ればAUTOに復帰。混雑・交差・フレーム異常・シリアル異常・STOP/ESTOP・通信断・探索時間切れなら止まり、新たなジェスチャーを要する。人物の身元認証ではない。
7. **前進/後退は未校正・無効**。障害物検知も未統合であり、フリーフロアでの自律追従は許可しない。

### 安全上の重要事項

- 実機を頑丈な支持台に固定し、脚と首の動作範囲を空け、サーボ外部電源を切れる担当者をつける。STOP/ESTOPはソフトウェア指令であり、**物理的なサーボV+遮断ではない**。
- 実機の方向、サーボPWMと中立点、GPIO25=R/26=G/27=Bの共通カソードRGB配線と抵抗、LEDの実際の発光を確認する。GPIO26は`SWITCH_PIN`との潜在的競合に注意。
- 未校正の実機に`--body-turn-calibrated`等のフラグを「動かしたいから」という理由で追加しない。ソフトウェアテスト合格は現物の機械的安全を保証しない。
- この機能追加は**PiのPython制御が主**。ESP32の状態LEDとウォッチドッグは同じブランチの既存ファームにあるため、ESP32が旧版なら書き込みが必要。

## STEP 1 — WindowsでESP32を書き込み

PC・Piの既存ロボットプログラムは先にCtrl+Cで停止。**サーボ外部電源をOFF**にし、ESP32のUSBだけWindowsへ接続。PowerShell:

```powershell
cd "$HOME\PresentationEscort-demo"
git status --short
git pull --ff-only origin feature/pc-gpu-follow-2026-10-28
& ".\.venv\Scripts\python.exe" -m pip install platformio
& ".\.venv\Scripts\python.exe" -m platformio device list
```

`COM3`など、ESP32に対応するポートを確認（例はそのまま使わない）。ビルド→書き込み:

```powershell
& ".\.venv\Scripts\python.exe" -m platformio run -d esp32 -e esp32dev
& ".\.venv\Scripts\python.exe" -m platformio run -d esp32 -e esp32dev -t upload --upload-port COM3
```

失敗したら、シリアルモニターやPi側のポートを閉じ、USBケーブルを確認。転送速度が不安定なら`esp32/platformio.ini`の`upload_speed=921600`を`115200`へ調整してから再試行。初回のPlatformIOライブラリ取得にはネット接続が必要。

監視は必要な場合だけ（開いたままPiに接続しない）:

```powershell
& ".\.venv\Scripts\python.exe" -m platformio device monitor --port COM3 --baud 115200
```

`[Safety] Protocol=V1 TimeoutMs=750`、`[Safety] State: DISARMED`が期待値。確認後Ctrl+Cで閉じる。

## STEP 2 — PC GPUサーバーとSSHトンネル

ESP32をWindowsから取り外し、PiのUSBへ接続。PiのIPは毎回確認すること。

Windows PowerShell **窓1**:

```powershell
cd "$HOME\PresentationEscort-demo"
git pull --ff-only origin feature/pc-gpu-follow-2026-10-28
& ".\.venv\Scripts\python.exe" experiments/pc_gpu/check_pc_environment.py
& ".\.venv\Scripts\python.exe" experiments/pc_gpu/pc_detector_preview_server.py --device 0
```

`CUDA available: False`ならCPU版PyTorchの可能性がある。無理にCUDA `0`で進めず原因を確認。CPU検証は`--device cpu`に変更できるが、往復遅延が500msを超えると安全側で止まる。

Windows PowerShell **窓2**（Piの最新IPに差し替え）:

```powershell
ssh -o ExitOnForwardFailure=yes -N -R 127.0.0.1:8765:127.0.0.1:8765 -L 127.0.0.1:8766:127.0.0.1:8766 pi@10.17.70.68
```

この窓は接続中、何も表示されないことがある。**閉じずに維持**。前回IP `10.17.70.68` は現在のIPを保証しない。

## STEP 3 — Piで最新コードと実ESP32を起動

Piの別ターミナルで:

```bash
cd ~/PresentationEscort-demo
git status --short
git pull --ff-only origin feature/pc-gpu-follow-2026-10-28
python3 -m unittest discover -s experiments/pc_gpu/tests -p 'test_phase17_bounded_search.py' -v
ls -l /dev/serial/by-id/
```

`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` は以前の機器の例。実際の表示で置き換える。ESP32シリアルポートを他のプログラムと共有しない。

**まずはモニター専用（サーボ外部電源OFF、機械動作なし）:**

```bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-off-confirmed --pc-host 127.0.0.1
```

モニター確認後Ctrl+Cで停止。次は**支持台・中立・首Yaw±3°・左右旋回・物理遮断を実機で確認した場合に限り**、外部サーボ電源をONにして:

```bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --enable-body-turns --body-turn-calibrated \
  --bench-secured --neutral-calibrated --physical-cutoff-tested
```

今までの左右向きが実機と逆であればSTOP後に`--legacy-horizontal-direction`を付けて比較。コマンド名そのものの反転やESP32の左右モーション書き換えはしない。

## STEP 4 — Windowsの表示と確認

1. Windowsブラウザで <http://127.0.0.1:8766/>。Pi窓に表示された**今回の新しいトークン**を入力。
2. `実ESP32 / DISARMED`、LED**赤**を確認。モニター専用では動作の有効化はしない。
3. 実機試験条件を満たす場合に限り「ジェスチャー追従を待機」をON → LED**黄色**。両手を頭上へ数フレーム保ち、AUTO/ESP32 ARMED → **白**。
4. 首が小さい左右の動きを追い、カメラ映像の中央十字、顔/肩のオレンジ点、中心へ向かう矢印、dx/dyに整合するか確認。画面端で本体旋回を確認。
5. **支持台の上でのみ**、横端の対象が画面外へ消えるように移動。画面に`SEARCHING`、向き、1 TURN MAXが出て、**追加旋回は1回**になること。短時間内に同じ側へ戻ると、3フレームを確認して復帰すること。
6. 1.5秒の時間切れ、他人の映り込み、通信遮断、STOP/ESTOPで必ず停止するか確認。危険なら即時に**物理電源遮断**。

終了: UI STOP → Pi Ctrl+C → PC GPU Ctrl+C → SSHトンネル終了 → サーボ外部電源OFF。

## よくあるエラー

| 症状 | 確認 |
|---|---|
| ESP32書き込みが`Failed to connect` | COMポート、USBケーブル、シリアルモニター終了、アップロード速度 |
| UIに接続できない | SSHの`-L 8766`、Piプログラム、ブラウザURL、トークン |
| GPUカメラが出ない | USBカメラの`/dev/video*`、PC GPU 8765、SSHの`-R`、Windowsファイアウォール |
| LEDが常時赤 | ファームが旧版、待機ON未設定、Pi接続異常、ESP32がDISARMED/FAULT |
| LEDが黄色へ変わらない | GUIで待機ON、GPIOの共通極・抵抗、ESP32の実ファーム、RGB配線競合 |
| LEDが白へ変わらない | 両手挙上4フレーム、人物ロック、ESP32 SAFETY_ARM ACKとウォッチドッグ、UARTエラー |
| 首が動かない | 外部サーボ電源・校正・安全フラグ・`HEAD_POSE_SET`、新鮮なHeadState |
| 人物が消えても探索しない | 画面中央でロスト・他人が画面内・頭が±3°未到達・ESP32がBusy・古い検出の場合は意図的に禁止 |
| 前後へ動かない | **仕様どおり**。距離と床面障害物の校正が未完了 |

ソフトウェアテストはVirtualEsp32のみ。**現物でのフラッシュ、LEDの発光、旋回方向、転倒の有無はこのリポジトリから確認できない。**
