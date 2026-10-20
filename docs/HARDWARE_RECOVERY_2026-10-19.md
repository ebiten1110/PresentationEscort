# 実機接続復元・ファームウェア確認（2026-10-19）

**このページはGitHub上の過去のソースから復元した情報です。実機の現在の配線・ESP32に書き込まれているファームウェアが一致するとは限りません。**

## 0. 先に安全確認

- 外部サーボ電源を切り、バッテリーを取り外す。サーボ電源を切ったままESP32をUSB接続して、シリアルログのみ読む。
- ESP32の `initServoControl()` は起動時 `centerAllServos()` を実行する。サーボ電源が入ったままだと予期しない動作が起き得る。
- 最初の段階では `flash` / `pio run -t upload` などでESP32を上書きしない。
- 元配線が分からない場合は電源の＋/−とPCA9685の `VCC`・`V+` を目視確認してから接続する。GPIOからサーボに電力供給しない。
- 改造配線・電池ケース・DC-DC変換器の仕様はGitHubだけでは特定できない。写真と電源の電圧ラベルを確認する。

## 1. GitHubソース上の接続

### Raspberry Pi 3 A+ -> ESP32

- Raspberry Pi 3 A+ の単一USB-A端子に、ESP32のUSBシリアル端子を接続する。
- USBカメラもUSBなので、**両方使う場合はUSBハブ（電力が不足する場合はセルフパワー型）**が必要。
- Piとの通信は115200bps、以前の`config.py`は `/dev/ttyUSB0` を指定しているが、`/dev/ttyACM0` や`/dev/serial/by-id/`等の実ポートを確認する。
- PiとPC間の通信はWi-Fi/LAN。現行GPU実験ではカメラ映像 Pi -> PC、人物検出結果 PC -> Pi、制御命令 Pi -> ESP32。

### ESP32 -> PCA9685

ESP32 Arduino `Wire.begin()` の標準ピンはGPIO21(SDA)、GPIO22(SCL)。特別な `Wire.setPins()` 呼び出しは現在の `servo_control.cpp` にはない。

| ESP32 | PCA9685 | 役割 |
|---|---|---|
| GPIO21 | SDA | I2C data |
| GPIO22 | SCL | I2C clock |
| 3V3 | VCC | PCA9685 logic supply (3.3V) |
| GND | GND | 共有GND |
| 別途外部電源+ | V+ | サーボの仕様に合う電源（一般例5～6V） |

`VCC` と `V+` は異なる。サーボ電源の実際の定格を必ず確認する。

I2C address `0x40`、PWM `50Hz`。

### PCA9685 -> サーボ（ESP32 `include/config.h`）

| CH | 用途 |
|---|---|
| 0 | 左足 |
| 1 | 左腰 |
| 2 | 頭Yaw |
| 3 | 頭Pitch |
| 14 | 右足 |
| 15 | 右腰 |

サーボの信号線／＋／GNDの向きは基板のシルク印刷で確認する。色だけでは断定しない。

### LED

`light_control.cpp`の設定:
- GPIO25 -> 赤（各LEDに330Ω抵抗）
- GPIO26 -> 緑（各LEDに330Ω抵抗）
- GPIO27 -> 青（各LEDに330Ω抵抗）
- 共通カソード（共通GND）を想定し、3つのLEDを同期点灯
- `config.h`の `SWITCH_PIN=26` は**GPIO26（緑LED）と衝突**しているが、現行のコードがこのスイッチピンを利用しているかは別途確認する。配線変更禁止、要調査。

## 2. ESP32に今何が入っているか（上書きなし）

1. 外部サーボ電源を切る（必要ならサーボ用V+を切り離す）。
2. ESP32を**PCへUSB直結**（この時点ではPiへのUSB接続は外す）。
3. Windowsのデバイスマネージャー > ポート(COMとLPT)でCOM番号を確認。
4. PCの既存仮想環境で `python -m pip install pyserial` （未導入なら）。
5. シリアルモニター:

```powershell
.\.venv\Scripts\python.exe -m serial.tools.miniterm COM5 115200
```

`COM5`は例。実際のCOM番号を指定。起動ログが見えなければESP32のRESETボタンで再起動する（サーボ外部電源OFF）。

以前のGitHubファームウェアは以下のように表示する:

```text
Presentation Escort ESP32
V8.8 DUAL LOOP CAMERA BODY
[Main] Setup complete
```

その後、読取専用の `STATUS` を送るとファームウェア実装上は `Walking State`、`RGB Light`、`Control` 等が表示される。ヘルプは `HELP`。**前後左右のコマンドや自動追従はこの時点で送らない。**

注: UARTからファームウェアの元ソースコードをそのまま復元できるとは限らない。現在のソースと一致すると確認するまでは書き込みしない。

## 3. Raspberry Piの現在状態の調査（変更なし）

Piの電源は安定した5V/2.5Aを目安に専用給電する。電源が不十分なUSBハブではカメラとシリアル接続が不安定になりうる。

SSHのユーザー名・ホスト名は環境に依存する。接続例:

```powershell
ssh <Piのユーザー名>@raspberrypi.local
```

接続できない場合は、Piの画面/ルーターの端末一覧などからIPを確認。

Piシェルで以下の読取のみ実行:

```bash
hostname
hostname -I
cat /etc/os-release
python3 --version
lsusb
ls -l /dev/video* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
ls -la ~/PresentationEscort 2>/dev/null
git -C ~/PresentationEscort status --short 2>/dev/null
git -C ~/PresentationEscort log -1 --oneline 2>/dev/null
```

**注**: `docs/起動.txt` は古く `run_v8_7_with_log.sh` を参照しているが、Production候補はV8.8.3で `run_v8_8_with_log.sh`。過去の起動手順をそのまま実行しない。

## 4. 未確認事項

- 実機基板のバージョン、PCA9685電源電圧/許容電流
- USBカメラとESP32の実際の接続、Pi上のSSHアカウント
- ESP32に現在書き込まれているファームウェア（シリアル起動ログで識別）
- PiのローカルコードとGitHubの違い
- 起動時に意図しない動作がないこと
- サーボ出力の故障/摩耗/取り付け角度

## 5. 復旧チェック順序

1. 実機の写真（ESP32、PCA9685、電源、Pi端子）で配線照合。
2. PC直結・サーボ電源OFFでESP32起動ログ収集。
3. Piにログインし、OSとUSBデバイス／Gitバージョンを調査。
4. 変更前のコード/設定/ログをバックアップ。
5. GPUのCPU版PyTorchをCUDA版へ移行し、PC単体推論を確認。
6. Pi -> PC映像送信、PC -> Pi検出結果（dry-run）を確認。
7. 安全停止を実装・テストした後にサーボ電源を復帰し、最初は頭部のみ動作確認。
8. 頭部・旋回・距離の順に実機試験。

従来のProduction `V8.8.3` について、**左右旋回マッピングを安易に反転しない**。
