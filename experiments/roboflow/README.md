# Roboflow / Raspberry Pi experiments

Production codeを変更せず、Raspberry Pi 3 A+ 上で人物・顔検出器の性能を測定するための実験領域です。

最低限記録する項目:
- FPS
- 1フレーム推論時間
- 正面 / 横顔 / 移動中の検出率
- CPU負荷
- End-to-End追従遅延

結果が十分であることを確認するまで、`raspberry_pi/follow_dual_loop_v8_8.py` へ直接統合しません。
