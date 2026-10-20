# Phase 4 — PC人物検出 → V8.8.3追従計画（10/20）

## 今回追加したもの

- `experiments/pc_gpu/v883_person_dual_loop.py`：既存 `raspberry_pi/follow_dual_loop_v8_8.py` の頭部内側ループ・体外側ループの定数と演算を、ハードウェアなしで使える独立プランナーへ移植
- `experiments/pc_gpu/pi_tracking_dry_run.py`：PC/GPU→Pi検出→人物Aim→V8.8.3相当の頭部・本体プランナーを結合。ESP32への命令送信はしない
- `experiments/pc_gpu/tests/test_v883_person_dual_loop.py`：方向・ゲイン・デッドゾーン・上下制限・対象消失・旋回確認フレーム・busy/idleインターロック・ESP32頭角テレメトリ・前後移動禁止・原本定数との一致をソフトウェアテスト

## 制御の流れ

```text
Pi USBカメラ ──JPEG/TCP──▶ PC RTX4060 YOLO
                              │ DetectionResult
                              ▼
Pi person_target_adapter ──▶ V883PersonDualLoop (計画のみ)
                              │ HEAD_POSE_SET候補
                              │ TRACK_LEFT/RIGHT候補
                              ▼
                         PiMotionGateway (SAFE/ARM/STOP監視)
                              │
                         SerialCommandSink (別工程で接続予定)
                              ▼
                            ESP32
```

## 実機校正の保持

- 顔位置ベースのデッドゾーンを、320x240映像からV8.8.3基準の640x480ピクセルへ正規化
- Yaw±20°, Pitch±12°, X/Y deadzone 14/12px, gain 0.055/0.045, max yaw/pitch step 2.5/2.0deg
- カメラX→ロボットYaw符号 -1.0
- **body bearing = ESP32から確認した物理カメラYaw + 画像残差deg**
- **負bearing→TRACK_LEFT / 正bearing→TRACK_RIGHT**（既存の実機調整を変更しない）
- 本体旋回開始9deg、旋回後継続4.5deg、停止判断3deg
- 通常は2フレーム連続確認、ESP32の完了イベント後の継続追従は1フレーム
- 以前のV8.8.3はHEAD_POSE_SET、HEAD_STATUS、モーション完了イベントを使用。新プランナーもこのフィードバックを前提にする

## 安全インターロック

- **頭Yawの実機フィードバックがない場合、BODYコマンドは必ずNone**。仮のYaw目標から導いたbearingと旋回方向は表示だけ
- **ESP32がidle確認されない場合、BODYコマンドはNone**
- 旋回候補生成後は`motion_pending`を立て、`motion_completed()`の明示受理まで次の旋回を発行しない
- PersonBBoxが画面の90%以上を占める場合はPitchを保持（上半身/頭部位置をBBox比率だけで誤決定しない）
- 人物BBoxから距離追従を行わない（前後移動は安全ゲートでも初期OFF）
- 人物ロスト時に新しい頭/本体コマンドを生成しない
- AUTO有効化・MANUAL UI・ESP32実シリアル書込はこのDry-runには含まない

## 現時点の制限

この実装は**V8.8.3元ソースの数学的制御部分を移植した独立プランナー**であり、既存`HeadPoseController`クラスや`ESP32Serial`クラスをそのまま呼び出してはいない。V8.8.3のProductionコードは変更していない。

実機でHEAD_POSE_SETを送信する段階では、サーボへの角度コマンドもSafetyManagerによるモード制限・コマンド制限を通さなければならない。将来のESP32側独立watchdog・STATUS/IDLE・HEAD_STATUSの読み戻しが必要。

特に`pi_tracking_dry_run.py`の`turnSuggestion`は可視化用の演算値で、実機への`bodyCommand`ではない。実機Yawフィードバックがないので`bodyCommand=BLOCKED`が正常。
