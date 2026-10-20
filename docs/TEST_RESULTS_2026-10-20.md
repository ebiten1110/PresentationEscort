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

## 追加検証：Pi Motion Gateway + STOP送信（10月20日）

- 検証したコードcommit: `4c7d5d202a7494dfdcf32387299d5aebdf0e72e5`
- GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39348388795
- Ubuntu runner / Python 3.11.17、`compileall` 成功
- ユニットテスト: **72件実施・72件合格・失敗0**
- コマンド: `python -m unittest discover -s experiments/pc_gpu/tests -v`

追加実装:
- `motion_gateway.py`: メイン推論ループのrecv待ちと別スレッドでタイムアウトを監視し、モーション命令がある状態で失効すればSTOPへ遷移。起動/終了時のSTOPも実装
- `serial_command_sink.py`: 115200bpsシリアル、許可コマンドのみ、書込時間上限、ショートライトや抜去を例外化。**このテストではモックシリアルだけを使用**
- `pi_safe_stop_probe.py`: 既定でメモリ上にSTOPを送る。実シリアルポートを開くには`--enable-serial`が必要（実機未実施）
- `pi_tracking_dry_run.py`: Gatewayとメモリシンクで監視状態を表示するがESP32にはアクセスしない
- MANUALはPC/GPU検出リンクに依存しないよう論理的に分離し、350msデッドマンを保持
- 実シリアルシンクが接続された場合、watchdogスレッドが動いていなければARMもモーション許可も拒否

実行中に見つかった課題と修正:
- 最初のGitHub Actions実行は **69件中2件失敗**。2件は「拒否命令は何も送らない」というテスト期待値に対し、実装が安全側としてSTOP送信していたことが原因。STOPを残して期待値を修正。追加テストと監視強制の改修を経て72件すべて成功。

### 未検証（重要）

- ESP32実機へのSTOPコマンド受信と実際の運動停止
- Pi電源断、OSフリーズ、USB抜去時の安全停止。**ESP32側の独立監視機能が必要**
- ESP32のSTOPは前後距離パルスを即時中断しないため、物理的緊急停止と同等ではない
- STOPをESP32が受理したことを示すACK応答経路は未実装
- CPU競合や実際のシリアル遅延を伴う実機タイミングの検証
- Raspberry Pi実機で新しいgatewayコードの実行確認
- V8.8.3二重ループに新ゲートを接続する作業と実機走行

前記の「45件合格」は同日の**以前の状態**の記録。上記の**72件合格が現時点の最新版コードに対応する結果**。
