# Phase 21: カメラ映像の非表示・LATCHED切り分けと診断ZIP

## 実機画面で確認できたこと（2026-10-29）

- GPU側は「接続中」、往復遅延82.4ms、カメラ欄640×480なのに、ブラウザには壊れた画像アイコンが表示されていた。
- ブラウザのContent-Security-Policyが`default-src 'none'`で、映像表示用の`blob:` URLを許可していなかった。この不整合を`img-src 'self' blob:`だけ許可する形で修正した。認証・操作APIのアクセス制御は変更していない。
- `ESP32 LATCHED`かつ`[Safety] FAULT=REMOTE_ESTOP`。これはESP32が遠隔非常停止を受け取ったという記録で、**なぜその指令が必要になったか**までは画面だけでは確定できない。映像表示不具合と非常停止を別々に調べる。
- 安全確定前にESTOPを何度も解除したりARMしたりしない。STOP/ESTOPはサーボ動力電源を物理的に遮断しない。

## 変更点

1. JPEGプレビューの`blob:`表示を最小限のCSP許可で修正。画像読み込み中に旧URLを即時破棄しない。
2. 映像プレビューが未生成／HTTPエラーなら画面に理由を表示。詳細診断に「画像生成からの時間」を追加。
3. 実機ランナー起動時の任意フラグ`--capture-diagnostics`でRAMに最近の推論結果と画像を保存する。デフォルトは**記録OFF**。
4. オペレータートークンで認証した管理画面から「診断データを保存（画像＋判定 ZIP）」を押すと次がダウンロードされる。
    - `status.json`: ESP32状態、LATCHED/ESTOP、通信・カメラ、Yaw/Pitch、直近の判断
    - `decisions.jsonl`: 最近最大600フレームの人物候補、ジェスチャー、顔狙点、追従結果、RTT・GPU推論時間
    - `frames/*.jpg`: 最近最大48枚の枠・中央線付きJPEG画面。最大およそ12秒分（約0.25秒間隔）
    - `README.txt`: アーカイブの説明とプライバシー注意
5. ZIPはPi内のRAMから生成し、自動保存・外部送信・自動ARMはしない。最大枚数・JPEGサイズ・イベント数を制限する。オペレータートークンは含めない。

## 安全な最初の手順：サーボ動力電源OFFで映像と判定だけを見る

現在の実機が`LATCHED`なので、まず物理的にサーボ動力電源を切って確認する。ESP32 USB側の状態確認は続けられる。

```bash
cd ~/PresentationEscort-demo
git status --short
git branch --show-current
git pull --ff-only origin feature/pc-gpu-follow-2026-10-28
```

未保存の変更がある場合は`git pull`を進めず確認する。次のランナーはモーターARMを行わない。

```bash
python3 experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-off-confirmed \
  --pc-host 127.0.0.1 \
  --camera-fps 30 \
  --capture-diagnostics
```

PC側のGPU推論サーバーとSSHトンネルは、従来の構成で起動しておく。ブラウザの操作画面をCtrl+Shift+Rで再読み込みし、表示された新しいトークンで接続する。

1. 画面左の映像が表示されるか。
2. GPU推論時間・実測FPSが表示されるか。
3. 画面右の詳細診断でカメラプレビューが「画像生成からn ms」になるか。
4. 安全状態が`LATCHED`のままでも診断ZIPボタンを押して保存できるか。
5. 保存した`PresentationEscort-diagnostics.zip`をChatGPTに添付する。**人物の顔が映り込む可能性があるため第三者には公開しない。**

映像がなお表示されなければ、保存したZIPとPC側YOLOサーバーの直近の標準出力を比較する。何も映らずZIP内に写真も無い場合、GPU未接続、Piカメラ未取得、JPEG生成失敗のどこかを追加で切り分ける。

## 注意：この修正でLATCHEDが自動解除されるわけではない

起動・通信・ESP32ファームのリセット条件は既存の安全仕様のまま。`FAULT=REMOTE_ESTOP`はESP32への遠隔非常停止が記録されたことを示す。原因（操作、タイムアウト、通信エラー等）をログで確認し、物理遮断・停止条件・ESP32の状態を確認した後にだけ、正式な復旧手順を別途判断する。診断中の再ARMは行わない。

## 2026-10-29 診断ZIPで判明した追加の不整合と修正

- 600件・約129秒の判定履歴では実機報告 `Yaw=0.8° / Pitch=2.0°` が一貫して新鮮で、ファーム状態は常時 `LATCHED`、報告された故障は `REMOTE_ESTOP`。人物検出は560/600件で、196件に顔向けの狙点があった。UARTエラーとモーター送信は0件。
- Piの `BoundedSearchController.process()` はモーター無効の **SERIAL_PROBE_ONLY** であっても、サーボ上下角度が設定 `(0,0)` と不一致なら毎フレーム `SAFETY_ESTOP` を送っていた。診断モードでこの角度を単に観察しても緊急停止を指示しないよう修正した。
- **AUTO中**に上下左右角度が校正済みの範囲外になれば、従来どおり `SAFETY_ESTOP` で即座に止める。AUTO開始前に**既に分かっている**範囲外角度があれば、ESP32をLATCHEDにする代わりに `HEAD_FEEDBACK_NOT_CALIBRATED` を返してARMを拒否する。
- ESP32からの首角度が未受信の間は従来のARMプロトコルの扱いを維持し、実際の首姿勢指令は新鮮なESP32テレメトリを要するゲートが別途守る。
- コンソールの「首角度」に校正範囲警告を追加し、ZIPにも `head_feedback_error` と `head_pitch_approved_range` を含める。
- **今回のソフト修正は既に記録されているLATCHEDを解除しない。** 物理電源の遮断を確認し、再起動後に新たなREMOTE_ESTOPが送られていないことをオフラインで検証してから、実機の安全復帰手順を判断する。診断モードにはARM/RESET手段を追加していない。
