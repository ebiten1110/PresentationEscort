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
    NO_FACE_COMMAND,
    SEND_COMMAND_TO_ESP32,
    SERIAL_PORT,
    SERIAL_BAUD,
    SERIAL_TIMEOUT,
    FOLLOW_LOOP_IMAGE_PATH,
    FOLLOW_LOOP_INTERVAL,
    FOLLOW_LOOP_MAX_COUNT,
    SEND_ONLY_ON_CHANGE,
    NO_FACE_STOP_THRESHOLD,
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


def load_face_cascade():
    face_cascade = cv2.CascadeClassifier(CASCADE_PATH)

    if face_cascade.empty():
        print("[FaceDetect] ERROR: Haar Cascade を読み込めませんでした")
        print(f"[FaceDetect] Path: {CASCADE_PATH}")
        return None

    print(f"[FaceDetect] Cascade loaded: {CASCADE_PATH}")
    return face_cascade


def detect_faces(frame, face_cascade):
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


class ESP32Serial:
    def __init__(self):
        self.ser = None

    def open(self):
        if not SEND_COMMAND_TO_ESP32:
            print("[Serial] SEND_COMMAND_TO_ESP32 is False")
            return

        print(f"[Serial] Opening {SERIAL_PORT} at {SERIAL_BAUD} bps")

        self.ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=SERIAL_TIMEOUT,
        )

        # ESP32はシリアル接続時にリセットされることがあるので待つ
        time.sleep(2.0)
        print("[Serial] Opened")

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("[Serial] Closed")

    def send(self, command):
        if not SEND_COMMAND_TO_ESP32:
            print(f"[Serial] Skip sending: {command}")
            return False

        if self.ser is None or not self.ser.is_open:
            print("[Serial] ERROR: Serial port is not open")
            return False

        message = command.strip().upper() + "\n"
        self.ser.write(message.encode("utf-8"))
        self.ser.flush()

        print(f"[Serial] Sent: {command}")
        return True

    def read_available(self, duration=0.2):
        if self.ser is None or not self.ser.is_open:
            return

        start_time = time.time()

        while time.time() - start_time < duration:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(f"[ESP32] {line}")


def draw_result(frame, faces, main_face, command, diff_x, loop_count):
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

    cv2.putText(
        frame,
        f"Loop: {loop_count}",
        (10, 135),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return frame


def main():
    print("=== Presentation Escort Phase 5.6 ===")
    print("Continuous Follow Loop")

    face_cascade = load_face_cascade()
    if face_cascade is None:
        return

    cap = open_camera()
    if cap is None:
        return

    if not cap.isOpened():
        print("[Camera] ERROR: カメラを開けませんでした")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # 起動直後の乱れたフレームを捨てる
    for _ in range(5):
        cap.read()

    esp32 = ESP32Serial()

    last_command = None
    no_face_count = 0
    loop_count = 0

    try:
        esp32.open()

        print("[Loop] Start")
        print("[Loop] Press Ctrl + C to stop.")

        while True:
            if FOLLOW_LOOP_MAX_COUNT is not None and loop_count >= FOLLOW_LOOP_MAX_COUNT:
                print("[Loop] Max count reached.")
                break

            loop_count += 1

            ret, frame = cap.read()

            if not ret:
                print("[Camera] ERROR: フレームを取得できませんでした")
                time.sleep(FOLLOW_LOOP_INTERVAL)
                continue

            height, width = frame.shape[:2]
            screen_center_x = width // 2

            faces = detect_faces(frame, face_cascade)
            main_face = select_main_face(faces)

            if main_face is None:
                no_face_count += 1
                diff_x = 0

                if no_face_count >= NO_FACE_STOP_THRESHOLD:
                    command = NO_FACE_COMMAND
                else:
                    command = last_command if last_command is not None else NO_FACE_COMMAND

                print(
                    f"[Loop {loop_count}] No face. "
                    f"no_face_count={no_face_count}, command={command}"
                )

            else:
                no_face_count = 0

                x, y, w, h = main_face
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                command, diff_x = decide_command(face_center_x, screen_center_x)

                print(
                    f"[Loop {loop_count}] "
                    f"faces={len(faces)}, "
                    f"center=({face_center_x}, {face_center_y}), "
                    f"diff_x={diff_x}, "
                    f"command={command}"
                )

            should_send = True

            if SEND_ONLY_ON_CHANGE and command == last_command:
                should_send = False
                print(f"[Loop {loop_count}] Command unchanged. Skip send.")

            if should_send:
                esp32.send(command)
                esp32.read_available(duration=0.2)
                last_command = command

            result_frame = draw_result(
                frame,
                faces,
                main_face,
                command,
                diff_x,
                loop_count,
            )

            # 最新結果だけ上書き保存
            cv2.imwrite(FOLLOW_LOOP_IMAGE_PATH, result_frame)

            time.sleep(FOLLOW_LOOP_INTERVAL)

    except KeyboardInterrupt:
        print()
        print("[Loop] Stopped by user.")

    except serial.SerialException as e:
        print("[Serial] ERROR: SerialException")
        print(e)
        print("SERIAL_PORT が合っているか確認してください")

    finally:
        print("[Loop] Sending STOP for safety.")
        try:
            esp32.send("STOP")
        except Exception:
            pass

        esp32.close()
        cap.release()
        print("[Loop] Finished.")


if __name__ == "__main__":
    main()