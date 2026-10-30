# 2026-10-29 展示前の統合修正: Pitch/本体旋回/人物再認識

## 実機で確認された事実

診断ZIP (4) は600フレーム、人物検出577、人物ロック487、旋回指令13回（右10・左3）、Pitchの内部報告が -2.0〜-1.9°、600フレームすべてで上下可動域端。UARTは`PWM ch=3 angle=-2.0 duty=306 i2c=ACK`と報告した。これは基板I2C通信とPWM**指令**の確認であり、上下サーボ軸の動作を証明しない。

ユーザーの実機観測：上下サーボの動きは見えず、床上で左右の脚は動くが方向は変わらなかった。その後、足裏へゴムを巻くと方向が変わるようになったものの、人物が動いた方向と逆に旋回した。

**結果を踏まえて、ESP32の旋回モーションを強くする変更はしない。** ゴムによってグリップが変わった後の歩行振幅増加は転倒し得る。ESP32ファーム更新は今回は不要。

## ソフトの変更

1. `--reverse-body-turn-direction`: **本体への `TRACK_LEFT` / `TRACK_RIGHT` だけ**を入れ替える。首Yawやカメラの画像を反転させない。見失い時の限定探索方向にも適用する。指定しなければ以前の挙動を維持。
2. `--relative-pitch-reference`: 追従対象を初めてロックした**画像上の顔の高さ**を上下基準線とし、そこからの移動を中立0°に対する絶対Pitch目標へ変換する。画面上部に顔が常に映り、画面中央を目指すと -2°で張り付いていた状況を回避。首の機械的原点のキャリブレーションではなく、カメラ画像上の相対基準。上下モーターの許可域は従来どおり ±2°、速度は設定どおり制限。
3. 人物検出の縦長bboxが同一フレーム間で激変しても、唯一の候補で顔座標が安定しているならロックを保持。一時的な未検出後、唯一の人物が大きく重なって再出現した場合は**視覚的再認識のみ**行い、STOP後の自動ARMはしない。別人が近くにいるときは従来どおり停止・再登録する。

## 一括実機テスト: 3ステップ以内

1. Piで`git pull --ff-only origin feature/pc-gpu-follow-2026-10-28`。
2. これまでのPC GPU推論とSSHトンネルを起動したまま、Piで以下を実行。`--bench-secured`などは既に実際に確認済みの設置条件でのみ指定する。

```bash
cd ~/PresentationEscort-demo
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --enable-body-turns --body-turn-calibrated \
  --pitch-calibrated --pitch-min-deg -2 --pitch-max-deg 2 \
  --relative-pitch-reference --reverse-body-turn-direction \
  --head-speed-deg-per-second 2.0 \
  --pitch-speed-deg-per-second 1.5 \
  --turn-stable-frames 3 --turn-settle-seconds 0.7 \
  --turn-edge-fraction 0.15 \
  --camera-width 640 --camera-height 480 --camera-fps 30 \
  --camera-mjpeg --bench-secured --neutral-calibrated \
  --physical-cutoff-tested --capture-diagnostics
```

3. 一度の追従セッションで初回両手挙げ→上下移動→左右大移動→一時視野外→戻る。**Pitch目標・CH3 dutyが実際に変わるか**、本体旋回方向が修正されたか、`VISUAL_REACQUIRED_GESTURE_REQUIRED`になるかを確認し、STOPしてZIPを保存する。物理的に予期しない動きなら即停止。

### CH3が変化しても上下が動かない場合

ソフトウェアでは修復不能な可能性。基板に印字された`CH3`とユーザーの「左端ピンブロック・4番目」の位置を突き合わせる（ブロックの向きだけでは番号を断定できない）。上下サーボのGND/V+/信号線の向き、サーボ給電、ホーン固定、ケーブル、機械的干渉を確認する。**±2°以外へ勝手に拡張したり、信号ピンを推測で入れ替えたりしない。**

### 仕様上の限界

- 顔の初期高さに合わせて追従するため、実際のカメラ画面中央へ完全に顔を運ぶものではない。±2°の狭い物理許可範囲では、画面中央への大きな上下補正は不可能。
- 顔の位置とbboxの重なりで同じ人物「らしい」と判定しているが、顔認証やReIDではない。再ロックは本人保証でなく、STOP/ESTOP/通信切断からの再ARMは人の明示操作に委ねる。
- 実機の物理動作はこのリポジトリのオフラインテストだけでは証明されない。
