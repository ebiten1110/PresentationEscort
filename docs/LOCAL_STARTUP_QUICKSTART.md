# Presentation Escort 開発用・起動/停止手順
最終更新: 2026-10-27 / 対象: `feature/pc-gpu-follow-2026-10-28`

> **現時点の実機確認範囲は人物認識プレビュー、Pi↔PC、Pi↔ESP32のプロトコル診断まで。**
> 本手順は**モーターを動かさない**。既存のGPU推論と映像プレビューを起動する手順。
> **サーボ外部V+はOFF**。電源遮断スイッチの存在だけでは電気・機械的安全を証明しない。

## 端末の役割と起動順

- **Windows PowerShell ①**：YOLOサーバー（画面付き）。
- **Windows PowerShell ②**：WindowsからPiにSSH接続し、ポート8765の逆向きトンネルを保持。
- **Piの端末**（または別のWindows PowerShellからのPi SSH）：USBカメラのストリーミングクライアント。
- Piからの接続先 `127.0.0.1:8765` は、SSHトンネルを通ってWindowsの `127.0.0.1:8765` に接続する。
- 実機には自動的に電源を投入しない。ESP32のUART診断は別の独立手順。

### 0. Raspberry PiのIP確認（変更があり得る）

Pi端末で `hostname -I` を実行し、現在のIPv4アドレスを控える。
以前成功したIPは `10.17.70.68` だが、学校ネットワークで変わる場合がある。
Pi端末にアクセスできなければ、Windowsから `ssh pi@<Piの現在のIP>` で接続する。

### 1. Windows PowerShell ①：YOLO画面付きサーバー

```powershell
cd "$HOME\PresentationEscort-demo"
git status --short
git pull --ff-only
& ".\.venv\Scripts\python.exe" experiments/pc_gpu/pc_detector_preview_server.py --device 0
```

正常例：`[Model] ready`, `[Preview] listening on 127.0.0.1:8765`。
ウィンドウが開いて「Waiting for Pi camera...」と出ても、接続前なら正常。
前述の `git pull` が競合などで失敗したら続行せず状態を確認する。
通常のサーバー `pc_detector_server.py` とプレビューサーバーを**同時起動しない**（同じ8765番ポートを使う）。

### 2. Windows PowerShell ②：SSH逆トンネル

```powershell
ssh -o ExitOnForwardFailure=yes -N -R 127.0.0.1:8765:127.0.0.1:8765 pi@10.17.70.68
```

**IPはSTEP 0で確認したPi IPv4に置換。** このPowerShellは閉じない。
パスワードや鍵認証後、何も表示されず入力待ちの状態になることがあるが正常。
既に同じトンネルが動いている場合、二重起動しない。

### 3. Raspberry Pi端末：カメラ映像を送信

```bash
cd ~/PresentationEscort-demo
git status --short
git pull --ff-only
python3 experiments/pc_gpu/pi_stream_client.py \
  --host 127.0.0.1 --port 8765 \
  --width 320 --height 240 \
  --jpeg-quality 60 --timeout 30
```

正常例：`[Client] connected to 127.0.0.1:8765`、`[Result] found=True ... infer=...ms rtt=...ms`。
人物がいない/認識できないフレームの `found=False` は必ずしも通信エラーではない。
Windowsのプレビュー上に緑色の人物枠が表示される。
**このコマンドはESP32へ接続せず、モーター指令を出さない。**

### 4. 終了手順

1. Pi側ストリーミングを `Ctrl+C` で停止。
2. Windows YOLOプレビューを `Q` / `Esc` または `Ctrl+C` で停止。
3. Windows SSHトンネルを `Ctrl+C` で停止。
4. サーボ外部電源はOFFのまま。

## オプション：ESP32の版・DISARMEDのみを検証（動作禁止）

外部サーボ電源を**物理的にOFF**にして実際の接続と使用中ポートを確認した後、Pi上で実施する。

```bash
cd ~/PresentationEscort-demo
python3 experiments/pc_gpu/pi_deployment_launcher.py \
  --stage diagnostic \
  --serial-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --servo-power-off-confirmed
```

以前の実機結果：`PASS: PROTOCOL_V1_DISARMED`。これはモーター停止やPWMの安全性の証明ではない。
**2026-10-27開発ブランチではESP32起動時のPWMを抑えるコード変更を追加したが、実機再書込み・ビルド・新ファームウェア動作確認は未実施。**
実機書込みやサーボ通電は、別途安全手順に基づく単独試験まで行わない。

## よくある状況

- `Connection refused`：プレビューサーバー未起動、SSHトンネル切断、Pi接続先を確認。
- `TimeoutError`：Windowsの推論ログを確認。初回のCUDAウォームアップで過去約11.8秒の事例あり、30秒設定を利用。
- `[Result] found=False`：視野内に人がいない、照明・遮蔽・構図などで未検出。プレビュー映像で確認。
- `Address already in use`：別のYOLOサーバーや旧SSHトンネルが8765を使用している可能性。
- カメラ警告 `GStreamer`：映像取得・検出ログが続いていれば、警告だけで失敗とは限らない。
- PiとPCのIPはネットワーク変更により変わる。ここに記載した学校内IPは恒久設定ではない。

## 将来の実機モーター制御について

現在の起動コマンドは**あくまで人物検出試験**。Pi側の `pi_deployment_launcher.py` は意図的に実機運転モードを提供しない。ESP32のSTOP/ESTOP、物理的電源遮断、支持台、首Yaw/Pitch可動範囲、脚部の転倒リスク、人物ロックの誤切替を実機で検証するまでサーボ自動追従へ移行しない。
