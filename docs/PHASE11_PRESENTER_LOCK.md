# Phase 11 — オペレーター明示選択・人物ロック／誤切替防止

2026-10-20 / PCのYOLO + Raspberry PiのVirtualESP32 **ソフトウェア上のみ**。

## 何を変更したか

- `detection_protocol.py`: `protocol_version`、`frame_seq`、`people`を追加。旧V1クライアントとの互換は保持するが、Phase11 AUTOではV2以外を拒否。
- `pc_detector_server.py`: YOLOのperson全候補（座標、信頼度）をV2形式でPiへ送信。旧`found/center`も互換のため残す。Pi側の安全ロックは旧サーバーの最高信頼度/継続トラッカー結果を採用しない。
- `presenter_lock.py`: 手動ロックを明示的に選択。次フレーム以降、選択BBoxと位置・サイズの変化が整合する一人だけ許可。ロック消失・近接人物・複数候補・再接続・フレーム欠落・リプレイ・V1応答を即失効して新たな手動選択を要求。
- `pi_presenter_lock_demo.py`: Phase9の統合Runtimeを継承し、**PiMotionGatewayに渡す前**に発表者ロックのフィルターを適用。ESP32はVirtualEsp32固定、実機のシリアルやモーターとは接続しない。
- `pi_operator_console.py`: 認証済み`POST /api/target`で候補を選択／解除。SAFEモード以外では選択できない。旧Consoleでは409を返す。
- `operator_console.html/js`: 人物候補の相対位置（左右何%、上下何%）と信頼度、ロック状態を表示。操作者が現物/画像を目で確認して候補を選択する。

## 起動

PC/GPU側：`python experiments/pc_gpu/pc_detector_server.py --device 0`（RTX 4060 CUDAが動作する環境、サーバー用PCのシェル）。新V2コードをPCで更新してから起動する。

Pi側は初期motor OFFの模擬実行：

```bash
cd ~/PresentationEscort-demo
git pull --ff-only
python3 experiments/pc_gpu/pi_presenter_lock_demo.py --pc-host 10.17.6.97
```

Windows PowerShellでSSHトンネル：`ssh -L 8766:127.0.0.1:8766 pi@pyiot014.local`。ブラウザから`http://127.0.0.1:8766/`、Piに表示されたトークンを入力。GPU PCのIPは以前の例で、変化している場合はipconfigで確認。

Pi操作画面で検出人物の候補を見て位置を目視確認して1人選択。次の3つの正常検出を経てAUTO準備が可能。AUTOボタンは別途明示操作。候補の画面順は人物のIDではない。

**人物の枠だけで本人を識別することはできない。** 候補表示にはカメラ映像そのものは含まれず、現場の人物と位置を操作者が照合するための位置%だけを提供する。

## Fail-closed（安全側に停止）

- 操作者の選択がない状態で勝手に人物を取得しない。自動の追従対象切替を禁止
- 検出枠の大きなジャンプ・重なり・極端なサイズ変化・フレーム番号再利用・フレーム飛び・通信断時はロック破棄＋STOP
- 一度破棄したら対象の人が再び出現しても追従を自動再開しない。操作者が再選択し、AUTOを再ARMする
- 追従中は距離による前進・後退を一切生成しない
- 初期・終了時のシリアルアクセスなし。VirtualESP32でのシミュレーション結果は現物ESP32の停止性能の証明ではない

## 残る根本的制限とNO-GO

幾何学的位置と大きさによる継続追跡は、人物が完全に重なって入れ替わる、同じ場所に別の人が入る、急な動きが発生するケースで本人の取り違えを原理的に防ぎきれない。ReID・顔認証・複数カメラ・有人専任操作などの独立確認が必要になる。

**P10（複数人物の誤追従）実機テストは依然NOT_RUN/NO-GO**。展示会場に複数人がいる状態でAUTO運転を許可していない。人が近付いたら即時停止できるだけでは「本人特定済み」ではない。実機では物理停止、サーボ電源、通信断、支持台上の動作確認を先に行う。

## 既存Phase10不変基準との関係

`docs/PHASE10_CODE_BASELINE.json`はPhase9の成功コミットと重要15ファイルを固定した比較基準。Phase11は新規モジュール、PC検出と操作UIを追加/改修し、Pin対象であるV8.8.3、ESP32、Phase9の統合プランナー/ランチャー/VirtualEsp32の内容を変更しない。基準照合が失敗した場合はCIで検出する。

## ソフトウェア検証

`python3 -m unittest discover -s experiments/pc_gpu/tests -v`：手動選択、信頼度入替、2人交差、対象ロスト→STOP、再接続、V1/リプレイ/フレーム欠落、HTTP認証、VirtualESP32 AUTO/MANUAL独立の回帰テスト。

