# PresentationEscort Raspberry Pi Config

# スマホIPカメラを使う
USE_IP_CAMERA = True

# スマホIPカメラアプリに表示されたURLに変更する
IP_CAMERA_URL = "http://192.168.x.x:8080/video"

# USBカメラ用。今回は使わない
CAMERA_INDEX = 0

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

SHOW_WINDOW = True