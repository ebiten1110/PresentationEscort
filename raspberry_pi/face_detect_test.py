import cv2
from config import (
    CAMERA_MODE,
    IP_CAMERA_URL,
    CAMERA_INDEX,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    FACE_DETECT_IMAGE_PATH,
)


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


def draw_center_guides(frame):
    """
    画面中央の十字線を描画する
    """
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    cv2.line(frame, (center_x, 0), (center_x, height), (0, 255, 0), 1)
    cv2.line(frame, (0, center_y), (width, center_y), 0, 1)
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    cv2.putText(
        frame,
        f"Screen Center: ({center_x}, {center_y})",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    return frame


def detect_faces(frame):
    """
    OpenCV標準のHaar Cascadeで顔を検出する
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)

    if face_cascade.empty():
        print("[FaceDetect] ERROR: Haar Cascade を読み込めませんでした")
        print(f"[FaceDetect] Path: {cascade_path}")
        return []

    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
    )

    return faces


def draw_faces(frame, faces):
    """
    検出した顔に枠と中心点を描画する
    """
    if len(faces) == 0:
        print("[FaceDetect] No face detected.")
        cv2.putText(
            frame,
            "No face detected",
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        return frame

    print(f"[FaceDetect] Detected faces: {len(faces)}")

    height, width = frame.shape[:2]
    screen_center_x = width // 2

    for i, (x, y, w, h) in enumerate(faces):
        face_center_x = x + w // 2
        face_center_y = y + h // 2
        diff_x = face_center_x - screen_center_x

        print(f"[FaceDetect] Face {i + 1}")
        print(f"  Box    : x={x}, y={y}, w={w}, h={h}")
        print(f"  Center : ({face_center_x}, {face_center_y})")
        print(f"  Diff X : {diff_x}")

        # 顔の枠
        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (255, 0, 0),
            2,
        )

        # 顔の中心点
        cv2.circle(
            frame,
            (face_center_x, face_center_y),
            5,
            (0, 255, 255),
            -1,
        )

        # 顔情報テキスト
        cv2.putText(
            frame,
            f"Face {i + 1}: ({face_center_x}, {face_center_y})",
            (x, max(y - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )

    return frame


def main():
    print("=== Presentation Escort Phase 5.2 ===")
    print("Face Detect Test")

    cap = open_camera()

    if cap is None:
        return

    if not cap.isOpened():
        print("[Camera] ERROR: カメラを開けませんでした")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[Camera] Capturing frame...")

    # 起動直後の暗い/乱れたフレームを避けるため数枚捨てる
    for _ in range(5):
        cap.read()

    ret, frame = cap.read()

    if not ret:
        print("[Camera] ERROR: フレームを取得できませんでした")
        cap.release()
        return

    cap.release()

    height, width = frame.shape[:2]
    print(f"[Camera] Frame size: {width} x {height}")

    frame = draw_center_guides(frame)

    faces = detect_faces(frame)
    frame = draw_faces(frame, faces)

    cv2.imwrite(FACE_DETECT_IMAGE_PATH, frame)
    print(f"[FaceDetect] Saved image: {FACE_DETECT_IMAGE_PATH}")
    print("[FaceDetect] Test finished.")


if __name__ == "__main__":
    main()