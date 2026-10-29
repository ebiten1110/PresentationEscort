# Phase 15: ジェスチャー選択・人物/顔追従・左右符号補正

## 実装範囲
- YOLO26n-pose（Windows RTX GPU）から、人物bbox + COCO 17キーポイントを取得
- 両手首が頭上にあるジェスチャーを連続4フレーム確認すると対象を選択。複数人同時挙手・曖昧な重なりでは選択不可
- 信頼できる鼻（顔）を優先し、顔不可なら肩中心、最後に人物bbox上部を狙い点にする
- 操作コンソールで一度「ジェスチャー追従を待機」を有効化後、ジェスチャーの成立でAUTO ARMを要求。起動時、接続断後、STOP/ESTOP後は待機無効
- ロックが失われるとSTOP/SAFE。単に人物が再び映るだけで再ARMしない。次の新たなジェスチャーでのみ再ARM要求
- 首Yaw・本体方位の横方向誤差の符号をユーザーの実機観察に基づき反転。必要なら --legacy-horizontal-direction で旧設定に戻せる
- ESP32ファームウェアや物理サーボ動作は今回変更しない。旋回回数のセッション制限はない

### 機材・試験の前提
サーボ電源、脚/首の校正、可動範囲、機体固定、障害物除去、物理電源遮断が確認できる状態だけで実機試験を実施すること。
STOP/ESTOPはサーボ V+を遮断しない。人物ロックは顔認証ではなく位置連続性に依存し、混雑・遮蔽時は誤追従防止を優先して停止する。AUTOにしていない間もカメラの人物解析は行う。

## 起動
Windows PowerShell 1（PC GPU。新しい姿勢モデルを初回ダウンロードする場合がある）:

```powershell
cd "$HOME\PresentationEscort-demo"
git pull --ff-only
& ".\.venv\Scripts\python.exe" experiments/pc_gpu/pc_detector_preview_server.py --device 0
```

Windows PowerShell 2: 前回通りPiの最新IPでSSHトンネル接続。
```powershell
ssh -o ExitOnForwardFailure=yes -N -R 127.0.0.1:8765:127.0.0.1:8765 -L 127.0.0.1:8766:127.0.0.1:8766 pi@10.17.70.68
```

Pi: 以前の実機プログラムは Ctrl+C で停止し、GitHubの最新版へ更新する。
```bash
cd ~/PresentationEscort-demo
git pull --ff-only
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --enable-body-turns --body-turn-calibrated \
  --bench-secured --neutral-calibrated --physical-cutoff-tested
```

**上記フラグは物理的に確認できた場合だけ指定すること。**
首Pitchは未校正であればゼロのまま。顔の画面内での上下位置を計算しても上下サーボ駆動は実施しない。Pitchを有効化する際は実機校正した上下限（機械限界/配線を含む）を確認して
`--pitch-calibrated --pitch-min-deg <下限> --pitch-max-deg <上限>`
を追加する。

Windowsで http://127.0.0.1:8766/ を開き、Piターミナルに表示された新しいトークンで接続する。

## 実機検証チェック
1. まずGUIでSAFE/DISARMED、映像が正常（人が右に移動するとプレビューで対応する座標が動く）を確認
2. 「ジェスチャー追従を待機」を押す。**この時点でESP32へARMは送信されない**
3. 左右の動きを試す前に、ロボットを固定した状態で1人が両手を頭上へ保持する。LOCKED→AUTO/ARMEDに切り替わるかを見る
4. 目視で低速の首Yawおよび本体の左右方向を検証。もし方向がまだ逆なら**STOP後**、`--legacy-horizontal-direction` で起動して比較する。ファームウェアのTRACK_LEFT/RIGHTマッピングは変更していない
5. カメラから短時間外れる→SAFE/STOP。再登場だけでは再ARMしない。再びジェスチャーするとARMするか確認
6. STOP、ESTOP、通信断ではジェスチャー待機解除または動作禁止になることを確認。異常動作では物理電源遮断
7. 顔が中央へ向くかを確認。上下のサーボはPitch未校正なら動かない

## 制限
- これはジェスチャーに基づく **操作対象の選択** であり、人物の身元を特定・認証しない
- YOLO姿勢モデルのキーポイントと信頼度が取得できない場合は意図的にジェスチャー認識を行わない
- 視野角・設置距離はカメラ解像度とは別。被写体が大きすぎる場合、適切な配置または広角カメラが必要
- ソフトウェアのCI成功は実機の向き・転倒しないこと・PWMの校正を証明しない
