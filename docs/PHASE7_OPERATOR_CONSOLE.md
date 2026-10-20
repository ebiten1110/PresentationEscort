# Phase 7 — ローカル操作コンソール／GPU検出Dry-run

2026-10-20。**ESP32へのモーター出力なし、GUIと安全APIのソフトウェア実装のみ**。

## 実装

Piの標準ライブラリ `ThreadingHTTPServer` と既存のPiMotionGateway/RemotePersonControllerを使用。追加のWebフレームワーク不要。

### 画面

- SAFE/AUTO/MANUAL/ESTOP 状態表示
- PC接続・人物検出・RTT・推論時間・安全状態
- AUTO／MANUAL明示切替、STOP、ESTOP、RESET
- LEFT/RIGHTを押している間だけ120ms間隔でMANUAL要求（Pi側350msデッドマンで独立保護）
- 通信断、タブ非表示、ポインタ離脱時のSTOP要求
- **前後移動は禁止**、距離キャリブレーションを行うまでUIにも出さない

### 安全と認証

- Webサーバーは **127.0.0.1限定**。`0.0.0.0`やLAN公開は拒否
- 毎起動時に暗号学的乱数で32-byteオペレータートークンを生成。画面の入力欄からAuthorization Bearer headerで送る
- API認証必須、Hostヘッダー制限、別OriginのPOST拒否、JSON形式と1024byte以下の制限
- TokenをURLに含めず、Cookie・LocalStorage等へ保存しない。ブラウザJS変数内のみ
- no-store / CSP / X-Frame-Options / nosniff
- **MemoryCommandSink固定**。SerialCommandSinkをインスタンス化しない、実機を操作する起動フラグも提供しない
- STOPの物理的安全性は未確認であり、既存ESP32の独立watchdogと物理停止テストが別途必要

## 起動手順

### 1. Raspberry Piで開発版を取得

```bash
cd ~/PresentationEscort-demo
git pull --ff-only
python3 experiments/pc_gpu/pi_operator_console.py
```

上記はPiカメラもGPU検出も起動しない、コンソール操作だけの完全Dry-run。

PiからGPU PCへカメラ映像と人物検出を送る場合:

```bash
python3 experiments/pc_gpu/pi_operator_console.py --pc-host 10.17.6.97
```

PCのYOLOサーバーが8765/TCPで起動している必要がある。PCのIPは以前の一例であり、変更されている場合は `ipconfig` で確認すること。

### 2. Windows PowerShellからSSHトンネル

```powershell
ssh -L 8766:127.0.0.1:8766 pi@pyiot014.local
```

SSH名が利用できない場合はPiのIPを使う。PowerShellのトンネルを維持したままWindowsブラウザで `http://127.0.0.1:8766/` を開く。

Piコンソールの起動ログに表示されたランダムなトークンをページで入力→「接続」。

## 既存Phase 6との関係

- GUIとSTATUSはすべてPi上の安全ゲートを呼び出す。STOP/ESTOPをGPU処理ループの外側から操作できる
- GPU接続断ではAUTO状態をSAFEへ戻す。MANUALは別のoperatorチャンネルの継続操作中だけデッドマン監視で許可
- 新ESP32の安全ハンドシェイクはまだこのコンソールの実機出力と結合していない。GUIシミュレーションのAUTO状態を物理的追従完成と扱わない
- 実機サーボ制御を有効化するには、GPIO/電源照合・ファームウェア版一致・Pi単一シリアル所有・ACK確認・物理STOP検証が必要

## テスト

こちらで実行可能: HTTP/API認証、Origin/Host検証、SAFE初期値、未検出AUTO拒否、左右MANUAL/STOP、前後移動拒否、ESTOP解除、デッドマン、シミュレーション検出連携、静的配線禁止、プログラム構文検査。

後日ユーザー実機: Raspberry Pi 3A+でUI起動とブラウザ接続、Wi-Fi復帰、Piカメラ映像、実GPU推論の安定性、ESTOPの物理作用。

## 次工程

- 実機用ランチャー（適切な安全確認後、明示有効化）
- GPIO/supplyチェックとESP32ファームウェア移行手順
- 10/28に実機のモーション、ACK、非常停止、5分間展示動作を判定
