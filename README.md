# Presentation Escort

発表者をカメラで検出し、頭部・ロボット本体・照明を自動追従させる Presentation Escort の開発リポジトリです。

## Production baseline

現在の実機確認済み基準は Raspberry Pi 側の **V8.8.3 dual-loop control** です。
ファイル名は互換性維持のため `raspberry_pi/follow_dual_loop_v8_8.py` のまま保持します。

- Raspberry Pi: `raspberry_pi/follow_dual_loop_v8_8.py`
- Raspberry Pi config: `raspberry_pi/config.py`
- Launcher: `raspberry_pi/run_v8_8_with_log.sh`
- ESP32 firmware: `esp32/`

## Safety rule

左右旋回は実機調整済みです。命令名だけを見て LEFT / RIGHT を入れ替えないでください。
V8.8.3 の基準は `NEGATIVE -> TRACK_LEFT`, `POSITIVE -> TRACK_RIGHT` です。

## Repository layout

詳細は [`docs/PROJECT_STRUCTURE.md`](docs/PROJECT_STRUCTURE.md) を参照してください。

今後の検出器比較は `experiments/` 以下で行い、production の二重ループ制御とは分離します。
