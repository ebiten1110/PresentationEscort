# PC GPU person-detection experiment

Production V8.8.3を変更せず、Raspberry PiのUSBカメラ映像をPCへ送り、PC GPUで人物を検出する実験です。

## 構成

```text
USB Camera -> Raspberry Pi -> JPEG/TCP -> PC GPU(YOLO26) -> DetectionResult/JSON -> Raspberry Pi
```

現段階では**ロボットへコマンドを送りません**。まず End-to-End latency と人物検出安定性を測定します。

- **Windows PCとPiの具体的なセットアップ:** [SETUP_WINDOWS_PI.md](SETUP_WINDOWS_PI.md)
- **PCのGPU環境診断:** `python experiments/pc_gpu/check_pc_environment.py`
- **通信回帰テスト:** `python -m unittest discover -s experiments/pc_gpu/tests -v`

## PC側起動

```powershell
.\.venv\Scripts\python.exe experiments/pc_gpu/pc_detector_server.py --device 0
```

NVIDIA CUDAが使用できなければ、CPUでの確認に限り `--device cpu` を指定します。`yolo26n.pt` の初回取得にはネット接続が必要な場合があります。

## Pi側起動

```bash
python3 experiments/pc_gpu/pi_stream_client.py --host <PCのIPv4アドレス>
```

Piの負荷が高ければ `--width 320 --height 240 --jpeg-quality 60` も試してください。

## 最初の測定項目

- Personが正面・横向き・歩行中に継続検出されること
- `infer=...ms` と `rtt=...ms` の値
- 人物がいない場合 `found=False`
- PC停止・LAN切断時にクライアントがタイムアウトすること

## 初期合格目安（仮）

- 推論時間：50ms未満
- End-to-End RTT：120ms未満
- 展示距離で人物ロストがHaarより少ないこと

数値は実機測定で調整します。**これらは実測結果ではありません。**
