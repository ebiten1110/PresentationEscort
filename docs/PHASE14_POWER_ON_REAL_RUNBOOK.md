# Phase 14 — サーボ外部電源ONでの Presentation Escort 統合運用

対象ブランチ: `feature/pc-gpu-follow-2026-10-28` (2026-10-28)

## 何を実装したか

- PiのUSBカメラ→Windows RTX GPU YOLO→PiのV2発表者ロック→Pi安全ゲート→USBシリアル実ESP32
- 通常の実ESP32ソフトウェア許可は **首Yaw ±3°のみ、Pitch 0°、脚・本体・前後移動禁止**。オプションで事前校正済みの体幹左右旋回パルスを最大1～3回まで許可可能（初期値2回、別途確認必須）。**前後移動・連続歩行・障害物回避は未統合・未実機認証**
- **サーボ電源ONを申告して起動**できる。電源ONだけではESP32をARMしない。対象選択+操作者AUTOの明示操作が必要
- 対象消失、YOLO停止、USB通信異常時のソフトウェアSTOP/ESTOPとESP32 watchdog 750ms（物理電源遮断ではない）
- UIは仮想ESP32/実ESP32と許可された範囲を区別。実電源はセンサで検出できず**ユーザー申告値**
- 自動テストにはVirtualEsp32によるARM, HEAD_POSE_SET, STOP, コマンド拒否, USB断, 範囲制限、CLI許可条件を含める
- **ユーザーの手動試験結果なしに「実サーボの安全検証済み」とはしない**

## 一括起動 — Windows 2窓、Pi 1窓

サーボ外部電源ONが利用者の今回の運用条件。ロボットが頑丈な支持台に固定され、
頭部と脚部が引っ掛からず、物理遮断スイッチがすぐ使える状況でのみ進む。
通電で首の保持力が落ちたり、USB接続時にESP32が再起動したりする可能性もある。

### Windows PowerShell 1（GPU YOLOサーバー）

```powershell
cd "$HOME\PresentationEscort-demo"
git pull --ff-only
& ".\.venv\Scripts\python.exe" experiments/pc_gpu/pc_detector_preview_server.py --device 0
```

### Windows PowerShell 2（SSHリバース＋ローカルフォワード）

PiのIPv4は毎回確認。以下の `10.17.70.68` は前回記録であり、現時点でのIPを保証しない。

```powershell
ssh -o ExitOnForwardFailure=yes -N -R 127.0.0.1:8765:127.0.0.1:8765 -L 127.0.0.1:8766:127.0.0.1:8766 pi@10.17.70.68
```

### Pi（専有USBシリアル/カメラ）

同じポートをシリアルモニターや他アプリが開いていないこと。前回の `pi_presenter_lock_demo.py` を実行中なら **Ctrl+C** で終了。

```bash
cd ~/PresentationEscort-demo
git pull --ff-only
python3 -m unittest discover -s experiments/pc_gpu/tests -p 'test_phase14_power_on_integration.py' -v
ls -l /dev/serial/by-id/
```

Python側のテストはモックのみで、実モーター動作を起こさない。

サーボ外部電源が実際にONであること、以下の安全前提が実物で成立していることを確認したうえで開始：

```bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on \
  --pc-host 127.0.0.1 \
  --enable-head-test \
  --bench-secured \
  --neutral-calibrated \
  --physical-cutoff-tested
```

フラグは実機確認済みという申告であり、アプリが検査したことにはならない。
`--servo-power-on` だけで起動する場合はモニター専用で、AUTO/実機駆動は禁止。

Windowsブラウザー `http://127.0.0.1:8766/` を開き、新しいPiコンソールのトークンで認証。
画面に `実ESP32`・`DISARMED` が表示されることを確認。
人物候補を明示選択して `TRACKING` にし、操作者が `AUTO` を押すことで初めてARMを開始。
頭部Yawの少量移動を見ながら、対象消失でSTOP/SAFEに戻ることを確認。
**首Yaw出力が±3°でも実サーボの物理角度が小さく動く保証はない**（中立角の機械差やPWM校正が前提）。

終了はまず操作者STOP、プログラム停止 `Ctrl+C`、ESP32安全状態確認。非常時はソフトSTOPではなく**物理電源遮断**を実施。
通電したままUSBを差し替える必要がある場合は、静電気/ショートや不意な姿勢変化にも注意。

### 体幹旋回も含む限定的な全体系統合試験（任意）

上記の標準コマンドは首Yawのみ。事前に**左右旋回パルス自体が実機で校正済み**であり、頑丈な支持台・周囲安全・独立物理電源遮断が検証済みの場合、起動時に追加で次を指定できる：

```bash
  --enable-body-turns --body-turn-calibrated --max-body-turns 2
```

**自動ロック選択のみで旋回は始まらない。** TRACKING確立後の明示的AUTOが必要。
トラッキング状態とESP32のHeadState/Busy/Walkingの新鮮なフィードバックを使う。
プログラムの1回の起動につき体幹旋回パルスは最大2回、原点復帰の保証なし。
脚の前後移動は常に禁止。物理機構の停止距離・停止時の姿勢・転倒は自動テストできない。
ステージなど人が近い場所では使わない。

## 自動テスト / 実機テストの境界

| 項目 | ソフトテスト | 手動/実機検証が必須 |
|---|---|---|
| V2人物ロック + AUTO条件 | 対応 | 人物取り違え、遮蔽、通行人 |
| UART命令の禁止とYaw角度クランプ、旋回パルス予算 | 対応 | 初回通電時のサーボ姿勢、PWM・可動域、旋回角度と転倒 |
| 対象消失 STOP | 対応 | 現物が停止するか |
| USB通信断ESTOP | 対応 | 実機の750msタイムアウトと残留トルク |
| サーボ電源 | 申告の表示のみ | OFF/ONの状態と遮断性能 |
| 体幹左右旋回（最大1〜3パルス） | 明示的オプション実装、模擬検証 | 実機では未承認 |
| 前後歩行・障害物回避 | **未実装/無効** | 未承認 |

### 手動確認を一括で報告するフォーマット

```text
1. 支持台・可動部・周囲安全: OK / NG
2. 中立姿勢・Yaw +/-3°で干渉なし: OK / NG
3. 物理電源遮断が有効: OK / NG
4. Pi USBでESP32認識、シリアル独占: OK / NG
5. UI: 実ESP32 / DISARMED / ON申告表示: OK / NG
6. TRACKING → AUTO → 首Yawのみ動いた: OK / NG
7. 見失い/ネットワーク断 → STOP/SAFE: OK / NG
8. ESTOP → 再ARM拒否、明示復帰のみ: OK / NG
9. 首Yaw専用時は脚・体幹・Pitchに動作なし: OK / NG
10. 体幹旋回を使用した場合は左右旋回最大2回・前後禁止: OK / NG / 未実施
11. 気付いた異音・引っ掛かり・不安定さ: なし / 詳細
```
