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