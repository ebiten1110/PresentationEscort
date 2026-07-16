import time
import cv2
import serial

from config import (
    CAMERA_MODE,
    IP_CAMERA_URL,
    CAMERA_INDEX,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    CENTER_TOLERANCE_X,
    FOLLOW_TEST_IMAGE_PATH,
    NO_FACE_COMMAND,
    SEND_COMMAND_TO_ESP32,
    SERIAL_PORT,
    SERIAL_BAUD,
    SERIAL_TIMEOUT,
)


CASCADE_PATH = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"


def open_camera():
    if CAMERA_MODE == "ip":
        print("[Camera] Mode: IP Camera")
        print(f"[Camera] URL : {IP_CAMERA_URL}")
        return cv2.VideoCapture(IP_CAMERA_URL)

    if CAMERA_MODE == "usb":
        print("[Camera] Mode: USB Camera")
        print(f"[Camera] Index: {CAMERA_INDEX}")
        return cv2.VideoCapture(CAMERA_INDEX)

    print(f"[Camera] ERROR: Unknown CAMERA_MODE: {CAMERA_MODE}")
    return None


def capture_frame():
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
    face_cascade = cv2.CascadeClassifier(CASCADE_PATH)

    if face_cascade.empty():
        print("[FaceDetect] ERROR: Haar Cascade を読み込めませんでした")
        print(f"[FaceDetect] Path: {CASCADE_PATH}")
        return None

    print(f"[FaceDetect] Cascade loaded: {CASCADE_PATH}")
    return face_cascade


def detect_faces(frame):
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
    if len(faces) == 0:
        return None

    # 一番大きい顔をターゲットにする
    return max(faces, key=lambda face: face[2] * face[3])


def decide_command(face_center_x, screen_center_x):
    diff_x = face_center_x - screen_center_x

    if diff_x < -CENTER_TOLERANCE_X:
        command = "LEFT"
    elif diff_x > CENTER_TOLERANCE_X:
        command = "RIGHT"
    else:
        command = "STOP"

    return command, diff_x


def send_command_to_esp32(command):
    if not SEND_COMMAND_TO_ESP32:
        print("[Serial] SEND_COMMAND_TO_ESP32 is False")
        print(f"[Serial] Skip sending: {command}")
        return

    print(f"[Serial] Opening {SERIAL_PORT} at {SERIAL_BAUD} bps")

    try:
        with serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=SERIAL_TIMEOUT,
        ) as ser:
            # ESP32はシリアル接続時にリセットされることがあるので待つ
            time.sleep(2.0)

            message = command.strip().upper() + "\n"
            ser.write(message.encode("utf-8"))
            ser.flush()

            print(f"[Serial] Sent: {command}")

            # ESP32側のSerial.printlnを少し読む
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if ser.in_waiting > 0:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        print(f"[ESP32] {line}")

    except serial.SerialException as e:
        print("[Serial] ERROR: SerialException")
        print(e)
        print("SERIAL_PORT が合っているか確認してください")
        print("例: /dev/ttyUSB0 または /dev/ttyACM0")


def draw_result(frame, faces, main_face, command, diff_x):
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

    # 検出された全顔
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (80, 80, 80), 1)

    if main_face is not None:
        x, y, w, h = main_face
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
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
    else:
        cv2.putText(
            frame,
            "No face detected",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

    cv2.putText(
        frame,
        f"Command: {command}",
        (10, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 255),
        2,
    )

    return frame


def main():
    print("=== Presentation Escort Phase 5.5 ===")
    print("Face Position -> ESP32 Serial Test")

    frame = capture_frame()

    if frame is None:
        print("[Main] Capture failed.")
        return

    height, width = frame.shape[:2]
    screen_center_x = width // 2

    faces = detect_faces(frame)
    print(f"[FaceDetect] Detected faces: {len(faces)}")

    main_face = select_main_face(faces)

    if main_face is None:
        command = NO_FACE_COMMAND
        diff_x = 0
        print("[Control] No face detected.")
        print(f"[Control] Command: {command}")
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

    cv2.imwrite(FOLLOW_TEST_IMAGE_PATH, result_frame)
    print(f"[Main] Saved image: {FOLLOW_TEST_IMAGE_PATH}")

    send_command_to_esp32(command)

    print("[Main] Test finished.")


if __name__ == "__main__":
    main()