# PC GPU inference experiments

Raspberry Pi から映像をPCへ送り、PC GPUで検出した結果を返す構成を検証する領域です。

最低限記録する項目:
- 推論FPS
- 推論時間
- Pi -> PC 映像転送遅延
- PC -> Pi 検出結果返却遅延
- End-to-End遅延
- ネットワーク切断時の安全停止

GPU単体速度ではなく、End-to-Endの追従性能で Raspberry Pi 単体方式と比較します。
