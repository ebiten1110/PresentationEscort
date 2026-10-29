# Phase 19: 低速な顔中心追従（Piのみ更新、ESP32再書き込み不要）

## 変更点と安全上の制限

- V8.8.3で実機調整した横方向の符号をPi実機ランナーの初期設定へ戻す。物理的に逆なら停止してから `--reverse-horizontal-direction` で比較。従来の `--legacy-horizontal-direction` は互換フラグとして受け付ける。
- 首のYaw/Pitch目標角度の**指令値**を初期設定で最大 **3°/秒** に制限。10 Hz程度の小さな角度指令へ分割し、顔座標を平滑化する。カメラが止まっても次の目標角度が飛躍しないよう1ステップを最大0.2秒分に制限。
- 姿勢モデルの鼻座標で顔を狙い、検出不能なら肩中央・人物bbox上部の順で代替。**顔認証や身元同定ではない**。人物ロック・4フレームのジェスチャー・異常時停止を継続。
- 首の左右は既存の **±3°** を維持。上下Pitchは**初期0°**。前バージョンのコードには±12°設定・PCA9685 CH2=Yaw/CH3=Pitchがあるが、現在の機体の機械的安全限界をプログラムが測定しているわけではない。
- 首だけでは追い切れず、ESP32から**新鮮な首角度**と**本体IDLE**が返り、かつ首が実際に限界付近へ達した場合だけ本体旋回候補を許す。走行・前後移動は引き続き禁止。見失い探索も既存の一回限定・時間制限を維持。
- SAFE中に計算した未送信の首目標はAUTO開始時へ持ち越さない。鮮度のあるESP32角度から安全な最初の目標を作り、許可範囲外の角度が報告されたらESTOP。
- ESP32のモーター安全管理・LED制御・WATCHDOG・安全ゲートは変更していない。これは**ソフトウェア試験済みの変更**であり、実機の速度・配線・軸方向・安全性の検証ではない。

## Raspberry Pi の更新

実機操作プログラムを `Ctrl+C` で停止し、動力電源をOFFにして、GitHubの開発ブランチを取得する。ローカルに未保存の変更がある場合、`git pull`前に確認し、上書きしない。

```bash
cd ~/PresentationEscort-demo
git status --short
git branch --show-current
git pull --ff-only origin feature/pc-gpu-follow-2026-10-28
git log -1 --oneline
```

**初回は本体旋回を有効にしない。** 機体を固定し、ニュートラル・外部遮断・首の可動範囲を確認したうえで次の首のみの試験を行う。

```bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --head-speed-deg-per-second 3 \
  --bench-secured --neutral-calibrated --physical-cutoff-tested
```

新旧の左右方向は `--reverse-horizontal-direction` で**停止後の別試験**として切り替えられる。動作中に配線・GPIO・サーボ方向を変更しない。

## 上下Pitchの段階的有効化

前バージョンで上下軸が実際に動いたという記録と、**現在の機体で確認済みの範囲**が一致するときだけ使用する。少しでも原点・配線・干渉に不安があれば上下0°のまま。以下は機械的に `-2..+2°` が確認済みの場合の**小さな試験例**で、機体の保証範囲を意味しない。

```bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --head-speed-deg-per-second 3 \
  --pitch-calibrated --pitch-min-deg -2 --pitch-max-deg 2 \
  --bench-secured --neutral-calibrated --physical-cutoff-tested
```

姿勢推定が鼻を見つけると上下左右の偏差で目標角度を更新するが、物理的な上下の向きは必ず固定した機体で観察。逆・干渉・急変・通信異常があれば即時物理遮断し、それ以上ARMしない。

## 確認する表示

1. ESP32 DISARMED / Pi SAFEでジェスチャー待機ON → LED表示は黄色（実配線・LED色は未測定）。
2. 両手を頭上に保持 → 4フレーム確定した人物だけがロックされ、許可条件がすべて成立すればAUTO/ARMED。LED表示は白。
3. カメラ映像の `FACE` / `SHOULDERS` / `UPPER BODY` と中央の矢印、`dx/dy` を確認。AUTOでは `YAW/PITCH` 目標角度も表示。
4. 左右・上下それぞれ、小さな顔移動をさせ、追いかける方向と速度を確認。体幹旋回は無効のまま。
5. 追従対象が消えた・別の人物が近づいた・通信が切れた場合は動作を止め、再ジェスチャーと安全認証なしに復帰しない。

## RGB LED 1個だけ違う色の場合

3個は同じGPIO（R=25、G=26、B=27）を共有する設計で、それぞれの色端子には個別の電流制限抵抗が必要。LED一つだけ色が違う場合は**動力もESP32も電源OFF**にして、RGB端子の接続、共通端子（共通カソード想定）、抵抗、破損を確認。現行ヘッダには `SWITCH_PIN=26` も定義されており、実配線で緑LEDと競合させないこと。コード上のLED希望色は電気的な点灯確認ではない。
