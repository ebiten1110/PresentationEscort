# 展示前統合修正：遠距離感度・速度・自動再キャリブレーション

## 最新の実機フィードバック

- 上下サーボは実際に動いた。CH3の配線問題とは断定しない。
- 足裏にゴムを巻くと旋回のグリップが改善した。前版で本体旋回方向を反転したため、今回はESP32ファームと脚の歩行振幅は変更しない。
- 遠くの人物に対する首Yawの振幅が大きく、速い移動に追いつけない。
- 診断(4)ではカメラ実効約5.2fps。首の指令速度を上げるだけでは映像の遅延は解決しない。

## 実装と限界

1. `--distance-adaptive-head`：人物の画面占有サイズ（バウンディングボックス幅と高さ）から近さの**相対指標**を算出。遠い（画面内で小さい）人物にはYaw指令の追加補正を最大65%弱める。サイズの揺らぎを平滑化し、既存の±3°制限を保持。カメラ焦点距離や実身長を測定していないため、**距離のメートル値は算出できない**。人物が屈む、横を向く、隠れると推定値が動く。
2. `--responsive-follow`：カメラ追従フィルタの時間定数を80msから45msに下げ、映像上の急な移動へ速く反応。`--head-speed-deg-per-second` / `--pitch-speed-deg-per-second` は別々の上限として残り、実機ファーム内部の動作速度まで保証しない。
3. `--auto-recalibrate-brief-loss`：既にAUTOで単独人物を追従しており、通信・ESP32頭角・本体IDLEが正常なときだけ、突然1フレーム人物が消えた場合に短時間（最長1.5秒）**動かず待つ**。唯一の同一らしい人物が戻り、既存SafetyManagerの3連続フレーム確認を通れば、そのAUTOセッションで視覚ロックを復帰。相対Pitch基準も再取得。人混み・長時間消失・通信切断・明示STOP/ESTOP後は勝手にARMせず、ジェスチャーからやり直す。顔認証・本人識別ではない。
4. 低FPSで小さく見える単独人物のフレーム間移動に対し、1フレームにつき画面幅・高さの10%までの限定的なbbox位置差を許容。高信頼の単独検出、直前も単独検出した場合のみ。人混みでは従来の厳しい判定を優先。

## 3ステップ一括実機確認

1. Piで更新：`cd ~/PresentationEscort-demo` → `git pull --ff-only origin feature/pc-gpu-follow-2026-10-28`。
2. 従来のPC GPU推論とSSHトンネルが起動している状態で、Piで以下を実行。安全関連フラグは過去の実機確認が引き続き有効な設置条件でだけ使う。

```bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --enable-body-turns --body-turn-calibrated \
  --pitch-calibrated --pitch-min-deg -2 --pitch-max-deg 2 \
  --relative-pitch-reference --reverse-body-turn-direction \
  --distance-adaptive-head --responsive-follow --auto-recalibrate-brief-loss \
  --head-speed-deg-per-second 3.5 --pitch-speed-deg-per-second 2.5 \
  --turn-stable-frames 3 --turn-settle-seconds 0.7 \
  --turn-edge-fraction 0.15 \
  --camera-width 640 --camera-height 480 --camera-fps 30 \
  --camera-mjpeg --bench-secured --neutral-calibrated \
  --physical-cutoff-tested --capture-diagnostics
```

3. 同じセッションで「両手挙げ → 遠くで左右に移動 → 近くで左右に移動 → しゃがむ → ほんの一瞬だけ画面外 → 戻る」を実施。画面の `距離感度`（小さい人物ほど小さい）、実効FPS、旋回方向、`REACQUIRED_3_FRAMES` を確認。停止したら自動で再ARMしないことも確認し、必要なら診断ZIPを共有する。

## 速度の次の課題

- 5.2fpsでは新しい画像が約190msに1枚しか届かない。高速追従は遅延に上限があるため、画面の実測FPSと `camera_stage_timing_ms` を優先して評価する。
- 実効FPSが大きく増えなければ、別途カメラ解像度・MJPEGの実際のFourCC・YOLO推論処理時間を診断し、画質やジェスチャー精度と交換条件で軽量化する。
- もっと大きく首を動かすにはサーボ可動範囲の**実機再校正**が必要。コードだけで現在の±3°/±2°を拡張しない。
