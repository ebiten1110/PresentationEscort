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

## Phase 4：V8.8.3相当のリモート人物追従計算（2026-10-20）

- 現行プランナー・結合検証commit: `e97ed5388d135058ac2c7d65422bebffb85b5f40`
- GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39349542489
- GitHub CI（Linux/Python 3.11.17）にて`compileall`完了、**95 tests / OK / 0 failures**
- 以前の72件に対し、V8.8.3相当計画17件 + 人物検出→安全ゲート統合6件を追加

検証した動作:
- 320x240→V8.8.3基準640x480換算したHead Yaw/Pitch制御
- Yaw±20°, Pitch±12°, デッドゾーン、最大角度増分、更新間隔
- 物理確認済みカメラ座標の符号とBody旋回方向（負→TRACK_LEFT、正→TRACK_RIGHT）
- 体Yawフィードバックの鮮度なし/ESP32 Busy時はBodyコマンド不生成
- 2フレーム継続確認、ESP32完了通知後のクールダウン、一回限りの動作候補
- 検出枠の上下欠けが疑われるときはPitchを保持
- 人物ロスト、GPU処理停止、未ARM状態はPiMotionGatewayでSTOPまたは動作拒否
- 前後移動コマンドは人物検出BBoxから一切生成しない

実機確認保留:
- 実サーボの現在Yaw/Pitchのシリアル読み戻し
- ESP32 Busy/IDLEとモーション完了ACKの実機検証
- 角度候補HEAD_POSE_SETを、SAFE/ESTOPを通す実シリアル出力へ接続する作業
- ESP32独立watchdogと物理停止
- 実機の動作方向、推論遅延下での旋回・頭部追従、距離校正

**注**: V8.8.3の原本は未変更。新プランナーは数式と物理校正パラメータを移植した独立実装であり、元のHeadPoseControllerを直接呼んでいない。モーター稼働を解禁した状態ではない。

## Phase 5：ESP32 Telemetry / Head Safety Gate

- GitHub commit: `962db8e983f15c1bc31d989e60b4c5fb21f1c4bb`
- GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39350597767
- CI: Python 3.11、`python -m compileall -q experiments/pc_gpu`成功、**132/132 unit tests成功**
- 今回追加: ESP32テレメトリの解析、UART状態監視/TTL、頭部命令の範囲とARM制限、Head/Body疑似統合

### 具体的に検証したこと

- `[HeadState]`形式のYaw/Pitch読み取り、シリアル分割/ノイズ/不正数値の拒否
- `Walking State: IDLE`と`Busy: NO`の両方が新鮮な場合だけIDLE扱い
- 0.45秒以内の頭部角度、0.40秒以内の歩行状態・Busyのみ採用
- ESP32がBusy/状態不明なら本体旋回を計画しても動作しない
- SAFE/MANUAL/ESTOP、ロスト時、頭部コマンドを拒否
- AUTO + 新鮮な検出 + 角度範囲でHEAD_POSE_SETを許可
- 偽UART/Memory sinkでSTATUS照会、HEAD角度、TRACK左右、STOPをソフトウェア検証

### テスト中の修正

- 最初のCIは132件中1件失敗。テストデータに先頭空白があり、コード側の空白除去で正しく受理されるため、実際の不正データとして`ZZ[HeadState]...`を用いるテストに訂正し、**132件全合格**

### 保留

- 実Pi/ESP32の現行ファームウェア確認と配線・電源安全確認
- ESP32からの通信断ウォッチドッグ（ESP32単体で通信断をSTOPする機能は未実装）
- STOPのESP32受信ACKの信頼性、頭部サーボの実際の動き・方向
- 実機制御のランチャー、有効化UI、手動認証経路の確認
- 展示用フル走行テスト

**注意：Phase 5の合格はソフトウェアのみ。実機サーボへは送信していない。**

## Phase 6 — ESP32独立通信ウォッチドッグ（2026-10-20）

- 対象GitHub commit: 8e45f0a93a3917a40b149a8f1cc5d86f79a1a65c
- Python構文チェック＋ソフトウェア単体テスト: **150 / 150合格**
- Python GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39353199423
- ESP32ファームウェアのPlatformIO **ビルド成功（書き込みなし）**
- C++純粋ウォッチドッグ状態機械のネイティブテスト **成功**
- ESP32 GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39352815051
- 確認内容：初期DISARMED、明示ARM、150ms PING、750msタイムアウトでLATCHED、遅延PING拒否、STOP/ESTOP/RESET後の再ARM必須、Arduino millisオーバーフロー、Pi側ACK待ちと異常時コマンド遮断。
- 追加異常解除テスト2件で、ラッチ状態からの明示リセット要求、ESP32のDISARMED応答後だけPiのESTOP解除、および勝手に追従を再開しないことを検証。
- テスト中の修正：仮想時刻と実時刻の比較が混在していたテストを統一し、150件全合格。
- ESP32物理モーションの向き・サーボの取り付け角・配線・電源・停止タイミングは**実機未検証**。
- 重要：新ESP32ファームウェアでは旧V8.8.3のPi側プログラムが動作をARMできない。バージョン同時移行が必須。新ESP32コードはGitHubの開発ブランチだけに反映し、実機への書込は実施していない。

## Phase 7 — Piローカル操作コンソール（2026-10-20）

- 対象コード：`pi_operator_console.py`、`operator_console.html/css/js`、`tests/test_operator_console.py`
- 実行方式：Piの127.0.0.1限定・SSHトンネルアクセス、毎起動ランダムトークン認証。
- **Python構文確認と単体/HTTP APIテスト171件すべて合格**（追加操作画面検証21件）。
- GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39355774838
- 初回実行は171件中1件失敗：テストが画面内`Dry-run`を大文字`DRY`で比較していたため。期待文言を修正し全171件合格。
- 検証対象：未認証拒否、Host/Origin制約、JSONサイズ/型、初期SAFE、AUTO起動要件、MANUAL左右とデッドマン350ms、STOP、ESTOPラッチ/解除、前後移動拒否、GPU検出→AUTO判定連携。
- Raspberry PiのWeb操作画面は**実機モーターへアクセスする経路を持たない（MemoryCommandSink固定）**ため、このテストはロボット駆動の合格判定ではない。
- 既存`pi_tracking_dry_run.py`の終了処理にあった未定義変数`safety`を`gateway`へ修正。ESP32へのアクセスは引き続きなし。

実機で未実施：Pi上のWeb起動、SSHトンネル・モバイルブラウザ操作感、Piカメラ/RTX4060持続送受信、ESP32の物理停止、発表者追従、手動モーション、安全復帰。

## Phase 8 — 段階的起動とESP32バージョン事前照合

- 最新コード: feature/pc-gpu-follow-2026-10-28（commit a28789c2f67c9494c8bde00bab8b1002185b58aa）
- GitHub Actions Python: https://github.com/ebiten1110/PresentationEscort/actions/runs/39357631575
- **Python構文確認＋194 tests / OK / failures 0**
- ESP32 C++状態機械・PlatformIO firmwareビルド: https://github.com/ebiten1110/PresentationEscort/actions/runs/39357504263 — 両ジョブ成功、物理書込なし
- Phase8新規: firmware_preflight.py、pi_deployment_launcher.py、tests/test_firmware_preflight.py、SAFETY_VERSION応答、Linux Serial排他OPEN
- デフォルト起動は静的auditのみで、カメラ、USB、サーボを開かない
- コンソールはMemoryCommandSink固定。ESP32へのモーター指令なし
- 診断モードは外部サーボ電源OFFの明示確認とシリアルポート名指定を要求。SAFETY_VERSION/SAFETY_STATUSのみ送信
- 旧ファームウェア/バージョン不一致/ARMED/LATCHED/状態未確認では診断不合格。実機運転は許可しない
- シリアル返信が分割された場合、バージョンとDISARMEDの両方を受信するまで判定保留とする修正を実施
- Pi上のUSBシリアル所有を排他モードとして要求（ソフトウェアテスト成功、実機未確認）

制限：バージョン診断もESP32実機では実行していない。物理的な電源・サーボの安全性は不明。実モーターを動かす統合ランチャーはまだ公開していない。今後は単一UART所有・検出/頭部/体/STOP状態と操作UIの統合が必要。

## Phase 9 — PC/GPU人物追従 + Pi安全ゲート + Virtual ESP32 + 操作画面

- 対象commit: `06809f5f53f25ba745a424ac55487336bc66fb32`
- GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39358876863
- Python構文確認、**221件のソフトウェア単体・統合テスト全件成功（失敗0）**
- 追加: `virtual_esp32.py`、`pi_integrated_simulation.py`、`tests/test_phase9_integrated_simulation.py`、ランチャー `--stage integrated`。
- `--stage integrated` はモーター指令を `VirtualEsp32` にのみ送り、実ESP32のUSB/シリアルには絶対接続しない。
- 単一プロセスで既存 `CameraInferenceWorker` / PiMotionGateway / Esp32FeedbackPoller / RemotePersonController / Web UI を統合。
- 検証: 模擬ARM_ACCEPTEDとState: ARMED、750msファームウェアwatchdog、MANUALデッドマン、左右旋回、HEAD_POSE_SET、STOP、ESTOPと手動RESET→SAFE。
- 追加修正: PiMotionGatewayが頭部のみ制御していた場合でも対象ロスト・解除時にSTOPを送信。
- 追加修正: UARTに古いDISARMEDが残り新しいARM要求より前に読まれても、ARM_ACCEPTEDの到着を待つ（ACKなしならタイムアウト拒否）。
- 初回Phase9試験ではARMが古いDISARMEDで取り消される問題を含む失敗があった。修正後、ACK順序を含む221件すべて合格。
- 本検証は**模擬ESP32を利用したソフトウェア試験**。実機ESP32やPi3A+上のパフォーマンス、サーボ、物理停止、電源、転倒防止は未検証。

実機使用時はFirmware V1、STOP/ACK、通信断、配線、サーボ、支持台での停止を先に確かめ、直結出力はデフォルトOFFのまま段階開放する。

## Phase 10 — ソフトウェア基準固定／オフラインバックアップと実機GO/NO-GO（2026-10-20）

- 対象コミット: `f17a63734c9470d00b95b48e36cc1c74b66f38ed`
- GitHub Actions: https://github.com/ebiten1110/PresentationEscort/actions/runs/39360021994
- GitHub CIにて重要15ファイルのGit blob SHA照合＋Git履歴の祖先確認に成功
- Python構文確認およびユニット・結合テストは **242件全合格・失敗0**
- Phase10新規21件はGit archive/bundle保存・検証、未コミット/ハッシュ不一致/別履歴拒否、バックアップ改竄のSHA256検出、GO/NO-GO判定テンプレの受理・拒否を検証
- `docs/PHASE10_CODE_BASELINE.json` はPhase9ソフトウェア基準（コミット`e0435b3f953f57cdd768173e8f171eb8db215376`）の変更検知基準。
- ソースバックアップは実Gitクローンの追跡対象と履歴だけを保存。**ユーザーのPiに保存した実績ではない**。ESP32の現物ファームウェアも保存しない。
- 実機GO/NO-GOテンプレの初期値は全項目`NOT_RUN`。物理STOP・配線・電源・発表者ロック・観客のいる環境でのAUTO動作は未検証。
- 実機運動の解禁・PiやESP32への書込・本番バイナリの指定・GitHub mainへの統合は行っていない。

Phase10運用手順: `docs/PHASE10_RELEASE_READINESS.md`
