# PC GPU person-detection experiment

Production V8.8.3を変更せず、Raspberry PiのUSBカメラ映像をPCへ送り、PC GPUで人物検出する実験です。

## 構成

```text
USB Camera -> Raspberry Pi -> JPEG/TCP -> PC GPU(YOLO26) -> DetectionResult/JSON -> Raspberry Pi
```

現段階では**ロボットへコマンドを送りません**。まず End-to-End latency と人物検出安定性を測定します。

## PC

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r experiments/pc_gpu/requirements-pc.txt
python experiments/pc_gpu/pc_detector_server.py --device 0
```

NVIDIA CUDA環境で `--device 0` を使用します。GPUが使用できない検証では `--device cpu` に変更できます。

## Raspberry Pi

```bash
python3 experiments/pc_gpu/pi_stream_client.py --host <PC_IP_ADDRESS>
```

## 初回確認項目

- Personが正面・横向き・歩行中に継続検出されること
- `infer=...ms` が表示されること
- Pi側で `rtt=...ms` が表示されること
- 人物がいない場合 `found=False` になること
- PC停止・LAN切断時にクライアントがタイムアウトすること

## 合格目安（初期）

- 推論: 50ms未満を目標
- End-to-End RTT: 120ms未満を目標
- 展示距離で人物ロストがHaarより明確に少ないこと

数値は実機測定後に確定します。
