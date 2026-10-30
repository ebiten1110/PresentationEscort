# Phase 13: Presentation Escort 全体結合テスト（2026-10-27）

対象: `feature/pc-gpu-follow-2026-10-28`。

**安全境界**
- **A: ソフトウェア全体テスト**はVirtualEsp32限定。ESP32へのUSB接続なし、モーター絶対に動かさない。
- **B: カメラ+実ESP32状態照合**は外部サーボV+を物理的にOFF。ESP32の `SAFETY_STATUS` を受信するがARMしない。モーター動作はしない。
- **C: 首Yawの限定試験**は安全支持台、実際の中立角、物理遮断の効果、可動域が個別に実機確認されてから使用。脚部・本体旋回は現行の実機連携では禁止。
- **未承認**: 展示会で人がいる場所の自動追従・歩行、Pitch・車輪・全機構連動。テストPASSだけで許可にはならない。

## 1. ソフトウェア全体の1コマンド試験（まずPiで実施）

```bash
cd ~/PresentationEscort-demo
git pull --ff-only
python3 experiments/pc_gpu/pi_showtime_smoke.py
```

**期待値:** `FULL-STACK SOFTWARE SMOKE: PASS (VIRTUAL MOTORS ONLY)`。
内部ではV2人物検出結果→発表者選択→AUTOのACK→頭Yaw指令→対象消失STOP→自動再ARM禁止→ESTOP保持を通す。
実際のPC/GPU映像ではなく模擬フレーム。実機サーボは動かない。

## 2. カメラを使う統合動作（全モーター仮想）

**Windows PowerShell①：**

```powershell
cd "$HOME\PresentationEscort-demo"
git pull --ff-only
& ".\.venv\Scripts\python.exe" experiments/pc_gpu/pc_detector_preview_server.py --device 0
```

**Windows PowerShell②：** PiのIPv4を確認して置換する。以前は `10.17.70.68` 。

```powershell
ssh -o ExitOnForwardFailure=yes -N -R 127.0.0.1:8765:127.0.0.1:8765 -L 127.0.0.1:8766:127.0.0.1:8766 pi@10.17.70.68
```

**Pi（既にSSH接続している別端末から）：**

```bash
cd ~/PresentationEscort-demo
git pull --ff-only
python3 experiments/pc_gpu/pi_presenter_lock_demo.py --pc-host 127.0.0.1
```

PCブラウザーで `http://127.0.0.1:8766/` を開き、Pi側端末に表示された操作者トークンを入力。
Windows YOLOプレビューで確認した演者の画面位置に一致する人物候補を**SAFE状態で手動選択**する。正常なV2フレームが3回続いた後、操作画面でAUTOを明示操作。
首追従・停止指令は `VirtualEsp32` にだけ送られる。右左のシミュレーション、選択解除、STOP、ESTOP、見失いを確認する。
**初回CUDA推論が遅くてもCameraInferenceWorkerは30秒まで待機する（Pi側安全判定は独立）。**

## 3. モーター電源OFFで、実ESP32の安全通信を組み込む

PiのUSBへESP32を接続し、**サーボ電源OFF**。Pi上で `ls -l /dev/serial/by-id/` で実ポートを確認。
シリアルモニター・診断ツールなど、他のソフトは終了すること。

上述のWindowsプレビュー・SSHトンネルは起動したまま、Piで `pi_presenter_lock_demo.py` を終了してから：

```bash
cd ~/PresentationEscort-demo
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-off-confirmed \
  --pc-host 127.0.0.1
```

ブラウザーは同じ `http://127.0.0.1:8766/` 。状態欄が仮想ではなく実ESP32のDISARMEDを示すか確認。
このモードは **ARM/AUTO/MANUALを拒否**。人物検出・人物ロック・UI状態と実ESP32の通信を一プロセスで監視する。
新しいファームウェアがESP32へ書き込まれたことは確認済みだが、**物理的なPWM無出力と独立電源遮断の効果は未検証**。

## 4. サーボ実機試験の条件

Cモードの機能は実装済み。ただし利用条件は以下すべてを物理的に確認したときだけ：
- 重力落下・転倒のない頑丈な支持台、可動部への接触防止
- サーボの中立位置と実際の可動範囲の確認
- 電源スイッチが本当にサーボ外部V+を遮断し、非常停止時に有効かを検証
- 照準の誤選択や急な首振りに備え監督者が独立スイッチを保持
- 頭部以外のサーボの駆動がソフト上で拒否されることを確認

許可したとしても、この入口は**Yaw ±3°・Pitch 0°固定・脚/本体命令拒否**。安全を保証しないため展示環境での運転は禁止。
物理的な安全が確認できない場合はBまでで停止。

## 5. 終了

Pi側 `Ctrl+C` → Windowsプレビュー `Q`/`Esc` → Windows SSH `Ctrl+C`。
サーボ電源OFFのまま片付け。プレビューとESP32モニターで別々にUSBシリアルを開かない。
