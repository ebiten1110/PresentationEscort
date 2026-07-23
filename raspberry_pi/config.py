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
# 既存の同名設定がある場合は、重複させず置き換える
# ============================================================

# カメラ映像が鏡像の場合のみTrue
MIRROR_CAMERA_IMAGE = False

# 顔の中心判定
CENTER_TOLERANCE_X = 35
CENTER_TOLERANCE_Y = 35

# カメラ取り付け位置の補正
CAMERA_CENTER_OFFSET_X = 0
CAMERA_CENTER_OFFSET_Y = 0

# 境界付近で判定が往復するのを防ぐ
X_HYSTERESIS = 8
Y_HYSTERESIS = 8

# 顔幅による距離判定
FACE_FAR_WIDTH = 140
FACE_NEAR_WIDTH = 230
DISTANCE_HYSTERESIS = 8

# 判定確定回数
MOVE_CONFIRM_FRAMES = 1
STOP_CONFIRM_FRAMES = 1
HEAD_CONFIRM_FRAMES = 1

# 顔を連続で見失った場合の停止判定
ADVANCED_NO_FACE_STOP_THRESHOLD = 2

# コマンド送信
ADVANCED_SEND_ONLY_ON_CHANGE = True
MOVE_REPEAT_INTERVAL = 0.6
SEND_MOVE_COMMAND = True
SEND_HEAD_COMMAND = True
SEND_COMMAND_TO_ESP32 = True

# LEFT / RIGHTが実機と逆の場合のみTrue
INVERT_HORIZONTAL_COMMANDS = False

# ループ
ADVANCED_FOLLOW_LOOP_INTERVAL = 0.15
ADVANCED_FOLLOW_LOOP_MAX_COUNT = None

# 結果保存
ADVANCED_FOLLOW_LOOP_IMAGE_PATH = "advanced_follow_loop_latest.jpg"
SAVE_RESULT_IMAGE = True
SAVE_RESULT_EVERY_N_LOOPS = 5

# 暗所向けコントラスト補正
ENABLE_HISTOGRAM_EQUALIZATION = True

SAVE_RESULT_IMAGE = True

# 1なら毎ループ保存
# Raspberry Pi 3 A+では、まず2がおすすめ
SAVE_RESULT_EVERY_N_LOOPS = 2

ADVANCED_FOLLOW_LOOP_IMAGE_PATH = "advanced_follow_loop_latest.jpg"