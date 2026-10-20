# ソフトウェア検証結果 — 2026-10-20

## 実施方針
ユーザーの指示により**アシスタント側で実行可能なテストは実施**し、Pi・ESP32・サーボ実機が必要な試験は後日に延期。

## 実際に実施した内容

- 対象commit: `9eba97ad6193563867ab76123b2e71750ea7edb2`
- 実行環境: GitHub Actions `ubuntu-latest` / Python 3.11.17
- 実行コマンド: `python -m unittest discover -s experiments/pc_gpu/tests -v`
- テスト結果: **45 tests / OK / failures 0**
- Actions run 1: https://github.com/ebiten1110/PresentationEscort/actions/runs/39346534490
- Actions run 2: https://github.com/ebiten1110/PresentationEscort/actions/runs/39346540041

| 分野 | 実施 | 合格 | 不合格 |
| --- | ---: | ---: | ---: |
| TCP/JSON通信プロトコル | 8 | 8 | 0 |
| 人物BBox/追従座標変換 | 12 | 12 | 0 |
| 人物位置連続性ロック | 10 | 10 | 0 |
| SafetyManager（論理ゲート） | 15 | 15 | 0 |
| **合計** | **45** | **45** | **0** |

## 今回追加したコード

- `experiments/pc_gpu/safety_manager.py`: SAFE/AUTO/MANUAL/ESTOP、明示ARM、対象ロスト、遅延破棄、通信監視、MANUALデッドマン、前後移動初期無効
- `experiments/pc_gpu/tests/test_safety_manager.py`: 上記のユニットテスト15件
- `experiments/pc_gpu/pi_tracking_dry_run.py`: 推論結果に応じたSafetyManager状態を表示（ESP32アクセスなし）

## 実機テストは未実施

Raspberry Pi 3 A+の現在環境、ESP32の実際の書込版、サーボの配線、PCからの実データ接続、STOPの物理動作、ウォッチドッグ実装、AUTO/MANUAL UIの実機連携はこのソフトウェア検証の対象外。

**注意:** `SafetyManager` は論理的なコマンド許可判定を提供するだけであり、ESP32へのSTOP送信やPiクラッシュ時の停止を保証するわけではない。ESP32の距離パルスはSTOP命令後も短い現パルスの完了を待つ設計。デモ実機の安全性の合格判定をしてはいけない。

後日テストの詳細: `docs/TEST_PLAN_2026-10-20.md`
