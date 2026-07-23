"""
Presentation Escort
Raspberry Pi側 共通設定

Phase 5.9:
・追従判定の高速化
・左右／上下／距離判定のヒステリシス
・頭サーボへの小刻みなHEAD_UP / HEAD_DOWN再送
・顔を一時的に見失っても、すぐに首を中央へ戻さない
"""

# ============================================================
# カメラ
# ============================================================

# "usb" または "ip"
CAMERA_MODE = "usb"

# スマートフォンのIPカメラを使う場合
IP_CAMERA_URL = "http://192.168.1.100:8080/video"

# USBカメラ番号
CAMERA_INDEX = 0

# 取得解像度
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Raspberry PiはGUIなしで使うためFalse
SHOW_WINDOW = False

# カメラ映像が鏡像になっている場合だけTrue
MIRROR_CAMERA_IMAGE = False


# ============================================================
# 画像保存
# ============================================================

SAVE_TEST_IMAGE = True
SAVE_IMAGE_PATH = "camera_test.jpg"
FACE_DETECT_IMAGE_PATH = "face_detect_test.jpg"
FACE_POSITION_IMAGE_PATH = "face_position_test.jpg"
FOLLOW_TEST_IMAGE_PATH = "follow_test.jpg"
FOLLOW_LOOP_IMAGE_PATH = "follow_loop_latest.jpg"
ADVANCED_FOLLOW_IMAGE_PATH = "advanced_follow_test.jpg"

ADVANCED_FOLLOW_LOOP_IMAGE_PATH = (
    "advanced_follow_loop_latest.jpg"
)

# 追従画像の保存
SAVE_RESULT_IMAGE = True

# 1なら毎ループ、2なら2ループごと
# Raspberry Pi 3 A+では2～5を推奨
SAVE_RESULT_EVERY_N_LOOPS = 2


# ============================================================
# 顔検出
# ============================================================

HAAR_CASCADE_PATH = (
    "/usr/share/opencv4/haarcascades/"
    "haarcascade_frontalface_default.xml"
)

# 暗めの画像に対してコントラスト補正を行う
ENABLE_HISTOGRAM_EQUALIZATION = True


# ============================================================
# 画面中央・位置判定
# ============================================================

# 顔の中心が、この範囲内なら左右中央とみなす
CENTER_TOLERANCE_X = 35

# 顔の中心が、この範囲内なら上下中央とみなす
CENTER_TOLERANCE_Y = 35

# カメラ取り付け位置の補正
CAMERA_CENTER_OFFSET_X = 0
CAMERA_CENTER_OFFSET_Y = 0

# 判定境界付近でLEFT/CENTERなどが往復するのを防ぐ
X_HYSTERESIS = 8
Y_HYSTERESIS = 8


# ============================================================
# 距離判定
# ============================================================

# 顔幅が小さい場合はFAR
FACE_FAR_WIDTH = 140

# 顔幅が大きい場合はNEAR
FACE_NEAR_WIDTH = 230

# FAR/GOOD/NEARの境界で判定が往復するのを防ぐ
DISTANCE_HYSTERESIS = 8


# ============================================================
# 判定確定
# ============================================================

# 反応を速くするため1回で確定
MOVE_CONFIRM_FRAMES = 1
STOP_CONFIRM_FRAMES = 1
HEAD_CONFIRM_FRAMES = 1


# ============================================================
# 顔を見失った場合
# ============================================================

# この回数見失ったら歩行だけSTOP
# 首は現在角度を維持し、顔の再検出を待つ
NO_FACE_MOVE_STOP_THRESHOLD = 2

# この回数見失ったら首もHEAD_CENTERへ戻す
# 首が動いた直後の一時的な顔ロスト対策として長めにする
NO_FACE_HEAD_CENTER_THRESHOLD = 8

# 過去コードとの互換用
ADVANCED_NO_FACE_STOP_THRESHOLD = NO_FACE_MOVE_STOP_THRESHOLD


# ============================================================
# Raspberry Pi → ESP32 シリアル通信
# ============================================================

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1.0

SEND_COMMAND_TO_ESP32 = True
SEND_MOVE_COMMAND = True
SEND_HEAD_COMMAND = True

# 基本はコマンド変化時のみ送信
ADVANCED_SEND_ONLY_ON_CHANGE = True

# LEFT / RIGHT / FORWARDの定期再送間隔
MOVE_REPEAT_INTERVAL = 0.60

# 顔が上下中央に入るまで、
# HEAD_UP / HEAD_DOWNをこの間隔で小刻みに再送する
HEAD_REPEAT_INTERVAL = 0.20

# 実機のLEFT/RIGHTがPython判定と逆の場合のみTrue
INVERT_HORIZONTAL_COMMANDS = False


# ============================================================
# ループ
# ============================================================

FOLLOW_LOOP_INTERVAL = 1.0
FOLLOW_LOOP_MAX_COUNT = 20

# Phase 5.9
ADVANCED_FOLLOW_LOOP_INTERVAL = 0.15

# NoneならCtrl+Cまで無限実行
ADVANCED_FOLLOW_LOOP_MAX_COUNT = None


# ============================================================
# 旧テスト用設定
# ============================================================

CENTER_TOLERANCE_X_OLD = 80
NO_FACE_COMMAND = "STOP"

FOLLOW_LOOP_INTERVAL_OLD = 1.0
FOLLOW_LOOP_MAX_COUNT_OLD = 20
SEND_ONLY_ON_CHANGE = True
NO_FACE_STOP_THRESHOLD = 2

CENTER_TOLERANCE_Y_OLD = 60
FACE_FAR_WIDTH_OLD = 90
FACE_NEAR_WIDTH_OLD = 180