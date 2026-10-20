# Phase 10：2026-10-28 実機移行・復旧の準備

記録日：2026-10-20。現時点では実機のモーター出力を許可しない。

## 1. ソフトウェア基準を固定

- ソフトウェア基準コミット：e0435b3f953f57cdd768173e8f171eb8db215376（Phase 9のCI成功後）
- 開発ブランチ：feature/pc-gpu-follow-2026-10-28
- docs/PHASE10_CODE_BASELINE.json に重要15ファイルのGit blob SHA-1を記録。V8.8.3原本・ESP32安全通信・Piのシミュレータ・ランチャーを含む
- Gitコミット番号とGit blob SHA-1は変更検出のための照合値。悪意ある改竄への署名・認証ではない
- これは **Phase9ソフトウェア基準** であり、実ESP32の書込済みプログラムや最終配備バイナリが同じという保証はない

## 2. Piでの事前検査（デバイスは一切開かない）

    cd ~/PresentationEscort-demo
    git fetch origin
    git switch feature/pc-gpu-follow-2026-10-28
    git pull --ff-only
    git status --short
    python3 experiments/pc_gpu/phase10_release.py check

成功条件：pinned_files_match=true / baseline_reachable=true / working_tree_dirty=false / ready_for_source_backup=true
hardware_approved=false は正常。実機の合格判定はまだないため。
不一致の場合、git reset --hardでローカルの変更を消さないこと。理由を調査し、ユーザーが必要なコードを保全する。

## 3. Gitソースと履歴の保存（ユーザーのPiで後日実施）

    python3 experiments/pc_gpu/phase10_release.py backup --dest ~/PresentationEscort-backups

バックアップの新規専用フォルダに生成するもの：
- source.tar.gz：その時点のGit HEADに記録済みのソースのみ
- history.bundle：オフライン復旧できるGit履歴
- backup.json：日時・ソースコミット・各ファイルのSHA-256

確認：

    python3 experiments/pc_gpu/phase10_release.py verify --backup-dir ~/PresentationEscort-backups/<生成されたフォルダ名>

verified=true を確認し、可能なら別媒体にもコピーして再検証する。バックアップはGit未保存の変更・OSの設定・Python依存ライブラリ・ESP32の実フラッシュイメージを含まない。

## 4. ソース復旧（ESP32実機への再書込は含まない）

1. まず動力系電源を安全に切り、実機制御プロセスを止める
2. 上記verifyが通るバックアップを確認し、既存フォルダを削除せず別名で復元

    git clone ~/PresentationEscort-backups/<生成されたフォルダ名>/history.bundle ~/PresentationEscort-restore
    cd ~/PresentationEscort-restore
    git log -1 --oneline

3. ログのコミットとbackup.jsonのsource_commitが一致するか確認
4. Python/OS/デバイスアクセスやPi設定は別途復元が必要
5. **実ESP32の旧バイナリは別管理**。ボード型番、現在のバイナリ、フラッシュ容量・書込み設定、サーボ校正が確認できない場合は書き換えない
6. 新ESP32のSAFETY_ARM/PING版と旧Pi V8.8.3はそのままでは互換性がないため、片側だけ差し替えて動かさない

## 5. 2026-10-28の実機GO/NO-GO判定

テンプレートは docs/PHASE10_HARDWARE_GATE_TEMPLATE.json。現時点の全項目NOT_RUNが正しい状態。

    python3 experiments/pc_gpu/phase10_gate.py --report docs/PHASE10_HARDWARE_GATE_TEMPLATE.json

初期状態はNO-GO。実機チェック12項目（P01〜P12）:

| ID | 現地で実際に確認すること |
|---|---|
| P01 | 配線・サーボ電圧・極性・電源分離 |
| P02 | 現在書込済みESP32版の実際の復旧手段が存在する |
| P03 | PiとESP32のV1プロトコルが一致し初期DISARMED |
| P04 | ソフトウェアから独立した動力遮断手段 |
| P05 | 通電・再起動時に不意動作しない |
| P06 | 支持台上で頭Yaw±20°・Pitch±12°が機構限界内 |
| P07 | 支持台上の本体左右旋回方向が一致 |
| P08 | STOP/ESTOPと電源遮断の実効果と停止姿勢 |
| P09 | Piフリーズ・USB抜去・GPU通信断の停止と復帰禁止 |
| P10 | 発表者ロスト・複数人で別の人を勝手に追わない |
| P11 | オペレーター認証・MANUALとデッドマン |
| P12 | 監視下の5分間展示リハーサル |

**P10の注意**：現在のGPUモデルは信頼度の高いpersonを選択するだけで、発表者の本人ロック・複数人物の誤追従防止が完成していない。複数人がいる展示でAUTOは解禁しないこと。
テンプレートには二人の実機観測者が記録し、12項目PASSと観測内容を満たす必要がある。この判定プログラムは人の自己申告を検査するだけで、モーターONにする機能を持たない。

## 6. 現物に残る重要な確認点

- GitHub上の旧設定には GPIO26（SWITCH_PIN とRGB緑LED）競合疑い。配線と現ESP32ファームウェアが一致するか照合するまで同時利用しない
- Raspberry Pi 3 A+のUSB-Aは1口。カメラとESP32の同時接続にはUSBハブが必要。サーボはUSB経由で給電しない
- STOPは前後パルスを即中断しない既存制約があり、実機の停止時間・転倒リスクが未評価。前後移動は初期禁止
- ESP32自身のmain loopフリーズや電源断はPi STOPでは救済できない。物理的緊急停止装置の代替とみなさない
- 起動時の中心位置へのサーボ指令にも注意。診断時は必ず外部サーボ電源をOFF

## 7. 実施/未実施を混同しない

- Phase9の221件合格は**模擬ESP32上のソフトウェアテスト**
- Phase10のソース監査・バックアップツールのCI合格は**ユーザーのPiにバックアップを作成したことではない**
- 現在のESP32フラッシュ内容・配線・電源・実機停止はまだ未確認
- 2026-10-28に上記を実機検証してからGO/NO-GO判定。未検証ならNO-GO。
