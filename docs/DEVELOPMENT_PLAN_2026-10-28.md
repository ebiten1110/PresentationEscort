# Presentation Escort 開発計画 — 実機確認期限 2026-10-28

## 完成条件

10月28日の実機確認時点で、最低限以下を満たす。

1. PC/GPUで人物を安定検出できる
2. PC -> Raspberry PiへDetectionResultを低遅延で返せる
3. 既存V8.8.3二重ループへ人物BBoxを入力できる
4. 左右追従・頭部追従が実機で動く
5. 距離追従は安全に有効/無効を切り替えられる
6. 対象ロスト時にSTOPする
7. PC/Pi通信断時にSTOPする
8. AUTO / MANUALを切り替えられる
9. 手動でSTOP / 前後左右を操作できる
10. 展示前にHaar版V8.8.3へ即時ロールバックできる

## 日程

### 10/19 — PC GPU基盤
- リポジトリ整理ブランチを基準化
- YOLO26 person detection server
- Pi camera streaming client
- DetectionResult JSON protocol
- 構文/プロトコル単体確認

**終了条件:** PCとPiをネットワーク接続すれば人物BBoxとRTTを測れる状態。

### 10/20 — PC/Pi実機通信
- PC GPU環境セットアップ
- Pi -> PC映像送信
- GPU人物検出
- FPS / inference / RTT計測
- JPEG quality / resolution調整

**終了条件:** 連続人物検出が安定し、End-to-End latencyの実測値を取得。

### 10/21 — Detection Adapter
- PC結果をV8.8.3のFaceResult相当へ変換
- Haar検出部分と制御部分を分離
- `remote_person` と `haar_face` を切替可能にする
- ロボット駆動はまだOFFで確認

**終了条件:** V8.8.3の制御計算がPC人物BBoxを入力として動く。

### 10/22 — 頭部・左右追従
- HEAD Yaw/Pitch実機確認
- Body Outer Loop接続
- NEGATIVE -> TRACK_LEFT / POSITIVE -> TRACK_RIGHTを維持
- 旋回中ロスト挙動確認

**終了条件:** 人が左右に歩いても頭部+本体が同方向へ追う。

### 10/23 — 距離追従
- Person BBox向け距離指標を調整
- FORWARD / BACKWARDを安全ゲート付きで有効化
- 最大連続パルス制限

**終了条件:** 左右正面が揃った時だけ前後移動する。

### 10/24 — Safety
- PC通信heartbeat
- タイムアウトSTOP
- Detection lost STOP
- Emergency STOP
- AUTO/MANUALモード状態機械

**終了条件:** AI/通信が壊れてもロボットが勝手に歩き続けない。

### 10/25 — Remote Override
- PCからSTOP / LEFT / RIGHT / FORWARD / BACKWARD
- AUTO / MANUAL切替
- 状態表示

**終了条件:** 自動追従失敗時に遠隔から即座に回収できる。

### 10/26 — 展示統合試験
- 5分連続追従
- 横向き・前後移動・一時遮蔽
- 複数人環境
- LAN一時切断
- ログ収集

**終了条件:** 展示シナリオを通しで完走。

### 10/27 — バグ修正・凍結
- 重大バグのみ修正
- パラメータ固定
- production candidate tag/commit作成
- 起動手順更新
- ロールバック手順確認

**終了条件:** コード変更を原則停止し、展示版を固定。

### 10/28 — 実機確認
- 起動チェック
- AUTO追従
- MANUAL Override
- Emergency STOP
- ロスト/通信断STOP
- 予備のV8.8.3起動確認

## 28日まで後回しにする機能

- 顔認証による本人特定
- 複数ジェスチャー
- 自動探索旋回
- 高度なReID
- UI装飾

Pose/Gestureは基幹機能が安定した場合のみ追加する。最優先は「人物追従が止まらず、安全に人間が介入できること」。
