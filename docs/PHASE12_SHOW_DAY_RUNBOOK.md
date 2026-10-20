# Phase 12 — 10月28日 当日準備・オフライン診断・異常時の復旧

作成日：2026-10-20｜対象：PresentationEscort 開発ブランチ。**実機ESP32・サーボ・カメラへの操作はこのPhase12からは行わない。**

## 現時点の完成判定

- Phase11まで：PC人物検出V2、発表者手動ロック、Pi安全ゲート、模擬ESP32、操作UIのテスト（274件）完了
- Phase12：オフライン再点検・模擬ESP32の重要シナリオ自動検査・証跡保存を追加
- しかし **Pi3A+での性能、現在ESP32実ファームウェア、配線、物理停止、サーボ、ロスト時の転倒防止は未検証**
- 本番用実機ランチャーは未提供。実機フラッシュや自動モーター有効化はしない
- **複数人のいる展示環境でのAUTOはNO-GO**。Phase11はBBox継続追跡で本人認証/ReIDではない

## 1. 当日より前に行う作業（実機が使えない間も可能）

### GitHubとソース保全

Pi開発用クローン（`~/PresentationEscort-demo`）で、手元の未コミット変更がないかまず確認。**重要な未コミット変更は消さない。**

```bash
cd ~/PresentationEscort-demo
git status --short
git fetch origin
git switch feature/pc-gpu-follow-2026-10-28
git pull --ff-only
python3 experiments/pc_gpu/phase10_release.py check
```

`check` の成功には重要15ファイルのハッシュ一致、開発履歴、Git作業ツリーの清浄状態が必要。GitHub Actionsが合格してもPiのSDカードにバックアップがあることは意味しない。

Piが安全に利用可能になったら、ソース・Git履歴のバックアップ（ESP32の実フラッシュは含まない）を作成：

```bash
python3 experiments/pc_gpu/phase10_release.py backup --dest ~/PresentationEscort-backups
python3 experiments/pc_gpu/phase10_release.py verify --backup-dir <実際に作成された保存先>
```

USBメモリ等の別媒体にコピーし、そこでverifyを再実行。旧フォルダを削除しない。

### Phase12の安全なオフラインリハーサル

```bash
python3 experiments/pc_gpu/phase12_rehearsal.py --output-dir ~/PresentationEscort-reports
```

出力は2ファイル：`PresentationEscort-preflight-<UTC>.json` と `.json.sha256`。ロボット、PC/GPU、ESP32 USB、サーボを開かない。Gitハッシュ、ソフトウェア安全条件、VirtualEsp32の停止・再ARM、人物の再選択を確認する。

出力の意味：

- `offline_rehearsal_passed=true`：**このクローンのソフトウェア側検証が通った**。実物確認ではない
- `readiness_evidence_complete=false`：ソースバックアップ、実機合格、あるいはコードなどに不足がある
- `motor_activation_permitted=false`：**常にfalse**。JSONを改変しても実機の動作を有効化する仕組みはない
- `blockers`：現在未完了の検査事項。手元のPiでソースバックアップ未実行なら通常不足として表示

既存バックアップ・現地チェック表も読み込む場合：

```bash
python3 experiments/pc_gpu/phase12_rehearsal.py \
  --backup-dir <保存済みバックアップフォルダ> \
  --hardware-report <人が記入した実機チェック表のコピー> \
  --output-dir ~/PresentationEscort-reports
```

チェック表にPASSと書いた事実だけでは、物理的な安全性は証明されない。観測記録と2人による現物確認が別途必要。

## 2. 当日の現物確認順序（ユーザーが実機でのみ実施）

| 優先度 | 手順 | 合格しなければ |
|---|---|---|
| 1 | 物理機構、Pi/ESP32、PCA9685、サーボ供給電源、USBハブの点検 | 通電・書込せず中止 |
| 2 | 現ESP32の機種、書込済みFW、校正値、元版復旧可能なバイナリ・方法を確認 | 新FWへの上書き禁止 |
| 3 | 非常時にソフトウェアと独立して安全に動力を切れる手段を確認 | サーボ駆動禁止 |
| 4 | 外部サーボ電源OFF、実機が安全な姿勢、PiとESP32のUSB接続を準備 | 書込・起動中止 |
| 5 | `--stage diagnostic --serial-port ... --servo-power-off-confirmed` でプロトコルV1とDISARMEDを照合（**対応新FWを書込済みのときだけ**） | 不一致なら実機運転中止 |
| 6 | 安全支持台上で単一Yaw/Pitch、左右方向、STOP、ESTOP、モーション終了時間、物理遮断を実際に観察 | 少しでも異常なら即動力遮断 |
| 7 | Pi/GPU/USB断、人物ロスト、他人物の接近、MANUALデッドマン、再ARMの禁止を確認 | AUTO/MANUAL運転禁止 |
| 8 | 全12項目の現地記録・確認者2人、監督下の5分展示リハーサル | 本番走行禁止 |

※ 現実には利用可能な制御構成・本番ランチャーが未完成であり、上記6〜8を**現行Phase12コードだけで実施できるわけではない**。安全条件が満たされない場合は静的展示または映像・シミュレーション展示へ切り替える。

## 3. 3つの展示モードを区別

- **モードA：ソフトウェア紹介** — 操作Web UI、PC YOLO人物検出、VirtualESP32、STOPの挙動を画面表示。実機モーターOFF。現時点で準備が進んだ構成
- **モードB：静的ロボット展示** — ロボットを動かさず、構造・センサー構成・デモ映像を説明。物理的な倒れ込み防止と通電安全は現場責任者が確認
- **モードC：実機動作** — 現状はNO-GO。回路、バックアップ、実機STOP、本人誤追従、5分リハーサルを含む現地試験と適切な運転用ランチャーが必要

## 4. 異常時の共通対応

ネットワークが切れる、CPU負荷が過大、サーボが想定外に動く、人物が切り替わる、PCとPiで制御モードが一致しない、ESP32状態が取れない、ロボットが傾く等の場合：

1. 人を近づけず、周囲に知らせ、危険のない範囲で**独立した物理動力遮断**を優先。安全な距離から操作
2. ソフトウェアSTOP/ESTOPは補助であり、サーボ電源断と同じ意味ではない
3. 原因が不明な状態でAUTOを再ARMせず、実機を安全な支持台で確認。ESP32のファームウェアを場当たり的に再書込しない
4. PC YOLO、Pi、ESP32のログ・時刻・電源・FW識別・STOP応答を保存。不要な個人情報・顔動画は収集しない
5. 復旧が間に合わない場合はモードA/Bへ切り替え。本番走行を無理に継続しない

## 5. 成果物と継続課題

- `experiments/pc_gpu/phase12_rehearsal.py` — Git安全監査、VirtualESP32シナリオ、チェック表の状態を統合してJSONレポートを作る
- `experiments/pc_gpu/tests/test_phase12_rehearsal.py` — 操作結果、強制NO-GO、バックアップ不在、レポートSHA-256、実機不使用の回帰テスト
- `docs/PHASE10_HARDWARE_GATE_TEMPLATE.json` — 既存12項目。値は実際の現場確認までは`NOT_RUN`を維持
- `docs/PHASE10_RELEASE_READINESS.md` — 旧ソース復旧・ハードウェア調査の手順
- 未解決：**現ESP32実フラッシュの保全、実Piシリアル、動力遮断、物理テスト、Pi用実機安全ランチャー、本人認証不要でも誤追従を許さない運用方法**

重要：このレポートは、実機を安全に動かせるという証明書でも、運転開始を許可するものでもない。
