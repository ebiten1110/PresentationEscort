# PresentationEscort Raspberry Pi Config

# "usb" または "ip"
CAMERA_MODE = "usb"

# スマホIPカメラ用
IP_CAMERA_URL = "http://192.168.x.x:8080/video"

# USB Webカメラ用
CAMERA_INDEX = 0

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

SHOW_WINDOW = False

SAVE_TEST_IMAGE = True
SAVE_IMAGE_PATH = "camera_test.jpg"

# 顔検出結果の保存先
FACE_DETECT_IMAGE_PATH = "face_detect_test.jpg"

# 顔位置判定結果の保存先
FACE_POSITION_IMAGE_PATH = "face_position_test.jpg"

# 中央判定の許容範囲
CENTER_TOLERANCE_X = 80

# ESP32 Serial
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1.0

# Phase 5.5 Follow Test
FOLLOW_TEST_IMAGE_PATH = "follow_test.jpg"

# 顔が見つからなかったときに送る命令
NO_FACE_COMMAND = "STOP"

# 連続送信前の安全設定
SEND_COMMAND_TO_ESP32 = True

# Phase 5.6 Follow Loop
FOLLOW_LOOP_IMAGE_PATH = "follow_loop_latest.jpg"

# 追従ループ間隔 秒
FOLLOW_LOOP_INTERVAL = 1.0

# 最大ループ回数
# None にすると無限ループ
FOLLOW_LOOP_MAX_COUNT = 20

# 同じコマンドを連続送信しない
SEND_ONLY_ON_CHANGE = True

# 顔が見つからない連続回数がこの数を超えたらSTOP送信
NO_FACE_STOP_THRESHOLD = 5

# Phase 5.7 Advanced Follow

# 左右の中央許容範囲
CENTER_TOLERANCE_X = 80

# 上下の中央許容範囲
CENTER_TOLERANCE_Y = 60

# 顔サイズによる距離判定
# 顔の幅がこれより小さいなら遠い
FACE_FAR_WIDTH = 80

# 顔の幅がこれより大きいなら近い
FACE_NEAR_WIDTH = 180

# 距離判定結果画像
ADVANCED_FOLLOW_IMAGE_PATH = "advanced_follow_test.jpg"
