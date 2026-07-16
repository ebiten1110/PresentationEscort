import cv2
from config import (
    CAMERA_MODE,
    IP_CAMERA_URL,
    CAMERA_INDEX,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    FACE_POSITION_IMAGE_PATH,
    CENTER_TOLERANCE_X,
)


CASCADE_PATH = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"


def open_camera():
    """
    config.py の CAMERA_MODE に応じてカメラを開く
    """
    if CAMERA_MODE == "ip":
        print("[Camera] Mode: IP Camera")
        print(f"[Camera] URL : {IP_CAMERA_URL}")
        return cv2.VideoCapture(IP_CAMERA_URL)

    if CAMERA_MODE == "usb":
        print("[Camera] Mode: USB Camera")
        print(f"[Camera] Index: {CAMERA_INDEX}")
        return cv2.VideoCapture(CAMERA_INDEX)

    print(f"[Camera] ERROR: Unknown CAMERA_MODE: {CAMERA_MODE}")
    print('[Camera] CAMERA_MODE must be "usb" or "ip"')
    return None


def capture_frame():
    """
    カメラから1枚画像を取得する
    """
    cap = open_camera()

    if cap is None:
        return None

    if not cap.isOpened():
        print("[Camera] ERROR: カメラを開けませんでした")
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[Camera] Capturing frame...")

    # 起動直後の乱れたフレームを捨てる
    for _ in range(5):
        cap.read()

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("[Camera] ERROR: フレームを取得できませんでした")
        return None

    height, width = frame.shape[:2]
    print(f"[Camera] Frame size: {width} x {height}")

    return frame


def load_face_cascade():
    """
    顔検出用Haar Cascadeを読み込む
    """
    face_cascade = cv2.CascadeClassifier(CASCADE_PATH)

    if face_cascade.empty():
        print("[FaceDetect] ERROR: Haar Cascade を読み込めませんでした")
        print(f"[FaceDetect] Path: {CASCADE_PATH}")
        return None

    print(f"[FaceDetect] Cascade loaded: {CASCADE_PATH}")
    return face_cascade


def detect_faces(frame):
    """
    顔を検出する
    """
    face_cascade = load_face_cascade()

    if face_cascade is None:
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
    )

    return faces


def select_main_face(faces):
    """
    複数顔が検出された場合、一番大きい顔をメインターゲットにする
    """
    if len(faces) == 0:
        return None

    main_face = max(faces, key=lambda face: face[2] * face[3])
    return main_face


def decide_command(face_center_x, screen_center_x):
    """
    顔のX座標と画面中央から、移動命令を決める
    """
    diff_x = face_center_x - screen_center_x

    if diff_x < -CENTER_TOLERANCE_X:
        command = "LEFT"
    elif diff_x > CENTER_TOLERANCE_X:
        command = "RIGHT"
    else:
        command = "STOP"

    return command, diff_x


def draw_result(frame, faces, main_face, command, diff_x):
    """
    判定結果を画像に描画する
    """
    height, width = frame.shape[:2]
    screen_center_x = width // 2
    screen_center_y = height // 2

    # 画面中央線
    cv2.line(frame, (screen_center_x, 0), (screen_center_x, height), (0, 255, 0), 1)
    cv2.line(frame, (0, screen_center_y), (width, screen_center_y), (0, 255, 0), 1)
    cv2.circle(frame, (screen_center_x, screen_center_y), 5, (0, 0, 255), -1)

    # STOP判定範囲
    left_limit = screen_center_x - CENTER_TOLERANCE_X
    right_limit = screen_center_x + CENTER_TOLERANCE_X

    cv2.line(frame, (left_limit, 0), (left_limit, height), (0, 255, 255), 1)
    cv2.line(frame, (right_limit, 0), (right_limit, height), (0, 255, 255), 1)

    # 全検出顔を薄く描画
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (80, 80, 80), 1)

    if main_face is not None:
        x, y, w, h = main_face
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        # メイン顔の枠
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # 顔中心
        cv2.circle(frame, (face_center_x, face_center_y), 6, (0, 255, 255), -1)

        cv2.putText(
            frame,
            f"Face Center: ({face_center_x}, {face_center_y})",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            frame,
            f"Diff X: {diff_x}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            frame,
            f"Command: {command}",
            (10, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
        )

    else:
        cv2.putText(
            frame,
            "No face detected",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

        cv2.putText(
            frame,
            "Command: STOP",
            (10, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
        )

    return frame


def main():
    print("=== Presentation Escort Phase 5.3 ===")
    print("Face Position Command Test")

    frame = capture_frame()

    if frame is None:
        return

    height, width = frame.shape[:2]
    screen_center_x = width // 2

    faces = detect_faces(frame)
    print(f"[FaceDetect] Detected faces: {len(faces)}")

    main_face = select_main_face(faces)

    if main_face is None:
        command = "STOP"
        diff_x = 0
        print("[Control] No face detected.")
        print("[Control] Command: STOP")
    else:
        x, y, w, h = main_face
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        command, diff_x = decide_command(face_center_x, screen_center_x)

        print("[Control] Main face selected.")
        print(f"  Box         : x={x}, y={y}, w={w}, h={h}")
        print(f"  Face Center : ({face_center_x}, {face_center_y})")
        print(f"  Screen X    : {screen_center_x}")
        print(f"  Diff X      : {diff_x}")
        print(f"  Command     : {command}")

    result_frame = draw_result(frame, faces, main_face, command, diff_x)

    cv2.imwrite(FACE_POSITION_IMAGE_PATH, result_frame)
    print(f"[Control] Saved image: {FACE_POSITION_IMAGE_PATH}")
    print("[Control] Test finished.")


if __name__ == "__main__":
    main()