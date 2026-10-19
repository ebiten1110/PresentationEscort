# PC GPU ↔ Raspberry Pi 接続手順（10/20 工程）

**重要:** 現時点の実装は人物検出と通信の確認専用です。`follow_dual_loop_v8_8.py` およびESP32は変更・起動しません。

## 0. 準備

- PC（Windows、NVIDIA GPUの場合はCUDA推論を選択可）
- Raspberry Pi 3 A+、USBカメラ
- PCとPiを**同じ信頼できるLAN**へ接続
- Windows側はPython 3.10以降を推奨
- この実験用TCPポートは認証・暗号化をまだ提供しません。**インターネットへ公開しない**でください。

## 1. Windows PC（PowerShell）

元の作業フォルダを変更しないよう、新しい場所に実験用コピーを作成します。

```powershell
git clone --branch feature/pc-gpu-follow-2026-10-28 https://github.com/ebiten1110/PresentationEscort.git PresentationEscort-demo
cd PresentationEscort-demo
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r experiments/pc_gpu/requirements-pc.txt
.\.venv\Scripts\python.exe experiments/pc_gpu/check_pc_environment.py
```

### CUDA判定

診断結果に `CUDA available: True` およびGPU名が出れば、まずGPU推論を試します。
NVIDIA GPUが搭載されていても `False` の場合、PyTorchがCPU版の可能性があります。CUDAに対応したPyTorchの導入コマンドは公式サイト <https://pytorch.org/get-started/locally/> でGPU/OSに合わせて確認してください。モデル初回起動時は重みの取得にネット接続が必要な場合があります。

### PC側サーバー起動

CUDAが使えるPC:
```powershell
.\.venv\Scripts\python.exe experiments/pc_gpu/pc_detector_server.py --device 0
```

CPUで通信確認だけ行う場合:
```powershell
.\.venv\Scripts\python.exe experiments/pc_gpu/pc_detector_server.py --device cpu
```

`[Server] listening on 0.0.0.0:8765` を確認します。

Windowsのファイアウォールでアクセス許可を求められた場合、**プライベートネットワークだけ**許可します。パブリックネットワークやルーターのポート開放は不要です。

別のPowerShellで `ipconfig` を実行し、同じLANに属するPCのIPv4アドレス（例 `192.168.x.x`）をメモします。IPは環境ごとに異なるため決め打ちしません。

## 2. Raspberry Pi（ターミナル）

Raspberry Pi OSのパッケージ経由でOpenCVを導入すると、Pi上でのpipビルドを避けやすくなります。

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy git
git clone --branch feature/pc-gpu-follow-2026-10-28 https://github.com/ebiten1110/PresentationEscort.git PresentationEscort-demo
cd PresentationEscort-demo
python3 -c 'import cv2, numpy; print("OpenCV", cv2.__version__)'
```

まずカメラが接続されているか `ls /dev/video*` で確認し、接続されていればPCアドレスを指定してテストします。

```bash
python3 experiments/pc_gpu/pi_stream_client.py --host <PCのIPv4アドレス>
```

もしPiの映像取得が重ければ（Pi 3 A+はメモリが少ないため）、初期検証は次の設定も試してください。

```bash
python3 experiments/pc_gpu/pi_stream_client.py --host <PCのIPv4アドレス> --width 320 --height 240 --jpeg-quality 60
```

`<PCのIPv4アドレス>` は実際の `ipconfig` の値に置き換えてください。

## 3. 成功ログ

PC: `found=True` および `infer=...ms` が表示される。人がいない場合は `found=False`。

Pi: `found=... center=(x,y) size=(w x h) infer=...ms rtt=...ms` が連続表示される。

`rtt` はカメラ取得・JPEG圧縮・送受信・推論・返却・展開を含む往復時間です。ネットワークだけの遅延ではありません。

## 4. エラー対応

| 症状 | 最初に確認する点 |
| --- | --- |
| `ModuleNotFoundError: ultralytics` | PCの `.venv` にrequirementsを導入したか |
| `CUDA available: False` | GPU種類、NVIDIAドライバ、PyTorch CUDA版 |
| `ConnectionRefusedError` | PCサーバー起動、PC IPv4、8765番、ファイアウォール |
| `TimeoutError` | Wi-Fi / LAN、ポート、PC推論負荷 |
| `camera open failed` | `/dev/video*`、カメラ接続、`--camera-index` |
| RTTが高い | 320x240、JPEG品質60、PiのCPU負荷 |
| PCサーバーが落ちる | PCのエラー全文、メモリ/GPU使用量 |

## 5. 安全上の約束

今回のテストは**ESP32へ歩行・首動作命令を送信しません**。実機モーションに接続する前に、対象ロスト/通信断時の停止ロジックと手動緊急停止を設計・テストします。

## 6. オフラインの通信回帰テスト

以下はYOLO・カメラ・GPUなしで実行できます。

```bash
python -m unittest discover -s experiments/pc_gpu/tests -v
```
