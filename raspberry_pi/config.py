CAMERA_MODE = "usb"
IP_CAMERA_URL = "http://192.168.x.x:8080/video"
CAMERA_INDEX = 0

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1.0

# ============================================================
# Phase 5.9 追従ロジック調整
# ============================================================

# ---------- 判定基準 ----------

# 顔の中心X座標が、画面中央から何px以内なら「中央」とみなすか
# 小さくするとLEFT / RIGHTが出やすくなる
CENTER_TOLERANCE_X = 45

# 顔の中心Y座標が、画面中央から何px以内ならHEAD_CENTERとするか
CENTER_TOLERANCE_Y = 50

# カメラ取り付け位置のずれ補正
# 正の値：論理上の中心を右へ移動し、LEFTを出やすくする
# 負の値：論理上の中心を左へ移動し、RIGHTを出やすくする
CAMERA_CENTER_OFFSET_X = 0
CAMERA_CENTER_OFFSET_Y = 0

# 左右・上下判定のヒステリシス幅
# 判定境界付近で命令が往復するのを防ぐ
X_HYSTERESIS = 15
Y_HYSTERESIS = 15

# 顔の検出幅による距離判定
# 顔幅がこの値より小さい場合はFAR
FACE_FAR_WIDTH = 140

# 顔幅がこの値より大きい場合はNEAR
FACE_NEAR_WIDTH = 230

# 距離判定のヒステリシス幅
DISTANCE_HYSTERESIS = 12


# ---------- 判定確定 ----------

# LEFT / RIGHT / FORWARDがこの回数連続したら確定する
MOVE_CONFIRM_FRAMES = 2

# STOPがこの回数連続したら確定する
# NEARや顔ロスト時の安全停止は、この値を待たず即時確定する
STOP_CONFIRM_FRAMES = 2

# 頭コマンドがこの回数連続したら確定する
HEAD_CONFIRM_FRAMES = 2

# 顔をこの回数連続で見失ったらSTOPする
ADVANCED_NO_FACE_STOP_THRESHOLD = 3


# ---------- コマンド送信 ----------

# 同じコマンドを毎ループ送らず、変化時または再送間隔経過時に送る
ADVANCED_SEND_ONLY_ON_CHANGE = True

# LEFT / RIGHT / FORWARDを再送する間隔
# ESP32側の旋回・前進が一回動作の場合、同じ命令を定期的に再送する
MOVE_REPEAT_INTERVAL = 1.2

SEND_MOVE_COMMAND = True
SEND_HEAD_COMMAND = True
SEND_COMMAND_TO_ESP32 = True

# ロボットが判定と逆方向へ動く場合はTrue
INVERT_HORIZONTAL_COMMANDS = False


# ---------- ループ ----------

# 追従判定の実行間隔
ADVANCED_FOLLOW_LOOP_INTERVAL = 0.35

# Noneなら手動停止まで無限実行
# 最初の確認では30程度を推奨
ADVANCED_FOLLOW_LOOP_MAX_COUNT = 30


# ---------- 画像・検出 ----------

ADVANCED_FOLLOW_LOOP_IMAGE_PATH = "advanced_follow_loop_latest.jpg"

# SDカードへの書き込み負荷を下げる
SAVE_RESULT_IMAGE = True
SAVE_RESULT_EVERY_N_LOOPS = 5

# 暗めの画像に対してコントラストを補正する
ENABLE_HISTOGRAM_EQUALIZATION = True