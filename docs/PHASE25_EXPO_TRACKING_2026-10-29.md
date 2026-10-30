# 2026-10-29 展示前修正: 遠距離の首振り・人物ロック・映像遅延

## 診断ZIP (5) に基づく事実

- 600フレーム中TRACKINGは79、SAFEは524。自動復帰を使える時間より長く人物が映らなくなっていた。
- seq760で2人のbboxが近づいたが、元の顔は(426,126)→(423,122)と安定。別人の顔は167px離れていたのに、全身bbox同士の近さだけでロックを解除。
- seq877→878では顔x379→214と1フレームで大きく移動。旧閾値でロックを解除。
- TRACKINGの79フレーム中69フレームはYaw角の目標が-3°付近。本体旋回指令はわずか2回。
- カメラ実測7.1FPS、平均GPU推論35ms、Pi→PCの往復約92ms。プレビュー画像の描画・JPEG再圧縮も毎フレーム同期実行されていた。

## 修正

1. 人物ロック: bboxが近接しても顔位置が安定し別人の顔と十分離れていれば続行。単独人物・高信頼・部分重なりがある場合に、速い横移動を許容。しゃがみ→立ち上がり時の縦長bbox変化も処理。両手挙げの検出が1フレーム途切れても、連続動作なら待機黄色からやり直しやすくした。顔認証ではないので、顔まであいまいな場合は従来どおり停止。
2. 遠距離の首: 人物が小さく見えるときはYawの軟らかい角度上限を約±1.1°へ縮小。相対的に遠いときはより早く本体旋回へ分担し、±3°に張り付いて画面外へ出るのを軽減。
3. 通信の速度: PCへの推論は全フレームを維持し、Piのプレビュー再JPEG生成だけ3フレームに1回へ。送信JPEGを画質70にする起動引数を使用。実効FPSの改善は実機で測定する。
4. カメラ位置と方位: カメラ画像xのずれ + 首Yawフィードバックから対象の推定方位角を計算する既存のbody_bearing_degで本体旋回を決める。首角と単眼画像だけで並進距離をメートル単位で確定することは不可能。必要なら深度センサーか既知サイズの物体と焦点距離の校正が必要。走行/並進の自動指令は今回追加していない。

## 3ステップ統合テスト

1. Piで cd ~/PresentationEscort-demo の後、git pull --ff-only origin feature/pc-gpu-follow-2026-10-28 を実行。
2. PC側のGPU推論とSSHトンネルを起動したまま、Piで以下のコマンドを実行。安全確認の申告は従来の設置条件が継続しているときだけ有効。

~~~bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --enable-body-turns --body-turn-calibrated \
  --pitch-calibrated --pitch-min-deg -2 --pitch-max-deg 2 \
  --relative-pitch-reference --reverse-body-turn-direction \
  --distance-adaptive-head --responsive-follow --auto-recalibrate-brief-loss \
  --head-speed-deg-per-second 3.5 --pitch-speed-deg-per-second 2.5 \
  --turn-stable-frames 3 --turn-settle-seconds 0.7 --turn-edge-fraction 0.15 \
  --camera-width 640 --camera-height 480 --camera-fps 30 \
  --camera-mjpeg --jpeg-quality 70 --preview-every-n-frames 3 \
  --bench-secured --neutral-calibrated --physical-cutoff-tested \
  --capture-diagnostics
~~~

3. 遠距離→近距離→素早い横移動→しゃがみ→立ち上がり→別人が画面端に映る→一瞬の遮蔽を1回の追従で確認。ロボットが旋回後に直立待機することも見る。STOP/ESTOP後の自動ARMはしない。解消しなければ新しい診断ZIPで顔位置・Yaw角・本体指令・実効FPSを比較する。

## 重要な制約

- 見た目の顔座標だけでは個人識別を保証できない。あいまいな人混みは自動復帰させない。
- 映像FPSの上限、実機の首可動域、旋回時のゴム摩擦などの制約は物理テストが必要。
- 上下CH3サーボとESP32の脚モーションは変更しない。
