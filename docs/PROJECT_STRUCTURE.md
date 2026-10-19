# Project Structure

この構成は、実機確認済み V8.8.3 を保護しながら、旧版と今後の検出器実験を分離するためのものです。

```text
PresentationEscort/
├─ README.md
├─ .gitignore
├─ docs/
│  ├─ 起動.txt
│  ├─ PROJECT_STRUCTURE.md
│  ├─ reference/
│  │  └─ ロボット開発入門(2026)#1_企画書フォーマット.docx
│  └─ archive/
│     └─ legacy/
│        ├─ フォルダ構成.txt
│        ├─ プログラムリスト.txt
│        ├─ ロードマップ.txt
│        └─ 現在タスク.txt
├─ raspberry_pi/
│  ├─ follow_dual_loop_v8_8.py   # Production: V8.8.3
│  ├─ config.py                   # Production config
│  ├─ run_v8_8_with_log.sh        # Production launcher
│  ├─ camera_test.py
│  ├─ follow_test.py
│  ├─ advanced_follow_test.py
│  ├─ follow_turn_stability_test.py
│  ├─ serial_sender.py
│  ├─ camera_monitor.html
│  └─ archive/
│     ├─ versions/
│     │  ├─ follow_distance_rgb_gpio_v8_4c.py
│     │  ├─ follow_8.6.py
│     │  ├─ follow_8.7.py
│     │  └─ run_v8_7_with_log.sh
│     ├─ legacy/
│     │  ├─ advanced_follow_loop.py
│     │  └─ follow_loop.py
│     ├─ config_notes/
│     │  ├─ config_v8_5_note.py
│     │  └─ config_v8_8_additions.py
│     └─ unused/
│        └─ main.py
├─ esp32/                         # Production ESP32 / PlatformIO project
└─ experiments/
   ├─ roboflow/
   └─ pc_gpu/
```

## Production protection

以下は整理目的で移動・改名しません。

- `raspberry_pi/follow_dual_loop_v8_8.py`
- `raspberry_pi/config.py`
- `raspberry_pi/run_v8_8_with_log.sh`
- `esp32/src/`
- `esp32/include/config.h`
- `esp32/platformio.ini`

特に `esp32/src/walking.cpp` と `esp32/src/motion_library.cpp` は実機調整済みモーション資産として扱います。

## Development flow

1. Production baseline を保存
2. `experiments/roboflow/` で Raspberry Pi 単体推論を測定
3. `experiments/pc_gpu/` で PC GPU 推論を測定
4. FPS / 推論時間 / 通信遅延 / 検出率を比較
5. 最良方式だけを DetectionResult 相当の共通出力へ接続
6. Production の二重ループ制御へ統合
