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
    CENTER_TOLERANCE_Y,
    FACE_FAR_WIDTH,
    FACE_NEAR_WIDTH,
    SERIAL_PORT,
    SERIAL_BAUD,
    SERIAL_TIMEOUT,
    SEND_COMMAND_TO_ESP32,
    ADVANCED_FOLLOW_LOOP_IMAGE_PATH,
    ADVANCED_FOLLOW_LOOP_INTERVAL,
    ADVANCED_FOLLOW_LOOP_MAX_COUNT,
    ADVANCED_SEND_ONLY_ON_CHANGE,
    ADVANCED_NO_FACE_STOP_THRESHOLD,
    SEND_HEAD_COMMAND,
    SEND_MOVE_COMMAND,
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
    print('[Camera] CAMERA_MODE must be "usb" or "ip"')
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

    # 一番大きい顔をメインターゲットにする
    return max(faces, key=lambda face: face[2] * face[3])


def decide_advanced_commands(face_center_x, face_center_y, face_w, screen_center_x, screen_center_y):
    diff_x = face_center_x - screen_center_x
    diff_y = face_center_y - screen_center_y

    # 左右方向
    if diff_x < -CENTER_TOLERANCE_X:
        move_command = "LEFT"
    elif diff_x > CENTER_TOLERANCE_X:
        move_command = "RIGHT"
    else:
        move_command = "STOP"

    # 上下方向
    # 画像では上がY小、下がY大
    if diff_y < -CENTER_TOLERANCE_Y:
        head_command = "HEAD_UP"
    elif diff_y > CENTER_TOLERANCE_Y:
        head_command = "HEAD_DOWN"
    else:
        head_command = "HEAD_CENTER"

    # 距離方向
    # 顔が小さい = 遠い
    # 顔が大きい = 近い
    if face_w < FACE_FAR_WIDTH:
        distance_state = "FAR"
        distance_command = "FORWARD"
    elif face_w > FACE_NEAR_WIDTH:
        distance_state = "NEAR"
        distance_command = "STOP"
    else:
        distance_state = "GOOD"
        distance_command = "STOP"

    # 歩行命令の優先順位
    # 1. 左右にズレているなら LEFT / RIGHT
    # 2. 左右が中央で遠いなら FORWARD
    # 3. それ以外は STOP
    if move_command in ["LEFT", "RIGHT"]:
        final_move_command = move_command
    else:
        final_move_command = distance_command

    return {
        "diff_x": diff_x,
        "diff_y": diff_y,
        "move_command": move_command,
        "head_command": head_command,
        "distance_state": distance_state,
        "distance_command": distance_command,
        "final_move_command": final_move_command,
    }


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

    def read_available(self, duration=0.15):
        if self.ser is None or not self.ser.is_open:
            return

        start_time = time.time()

        while time.time() - start_time < duration:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(f"[ESP32] {line}")


def draw_result(frame, faces, main_face, result, loop_count):
    height, width = frame.shape[:2]
    screen_center_x = width // 2
    screen_center_y = height // 2

    # 画面中央線
    cv2.line(frame, (screen_center_x, 0), (screen_center_x, height), (0, 255, 0), 1)
    cv2.line(frame, (0, screen_center_y), (width, screen_center_y), (0, 255, 0), 1)
    cv2.circle(frame, (screen_center_x, screen_center_y), 5, (0, 0, 255), -1)

    # 左右STOP範囲
    left_x = screen_center_x - CENTER_TOLERANCE_X
    right_x = screen_center_x + CENTER_TOLERANCE_X
    cv2.line(frame, (left_x, 0), (left_x, height), (0, 255, 255), 1)
    cv2.line(frame, (right_x, 0), (right_x, height), (0, 255, 255), 1)

    # 上下HEAD_CENTER範囲
    upper_y = screen_center_y - CENTER_TOLERANCE_Y
    lower_y = screen_center_y + CENTER_TOLERANCE_Y
    cv2.line(frame, (0, upper_y), (width, upper_y), (255, 255, 0), 1)
    cv2.line(frame, (0, lower_y), (width, lower_y), (255, 255, 0), 1)

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
            f"Face Size: w={w}, h={h}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            frame,
            f"Diff: x={result['diff_x']}, y={result['diff_y']}",
            (10, 90),
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
        f"Move: {result['final_move_command']}",
        (10, 130),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )

    cv2.putText(
        frame,
        f"Head: {result['head_command']}",
        (10, 170),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Distance: {result['distance_state']}",
        (10, 210),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Loop: {loop_count}",
        (10, 250),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return frame


def main():
    print("=== Presentation Escort Phase 5.8 ===")
    print("Advanced Follow Loop")
    print("Move: LEFT / RIGHT / FORWARD / STOP")
    print("Head: HEAD_UP / HEAD_DOWN / HEAD_CENTER")

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

    last_move_command = None
    last_head_command = None
    no_face_count = 0
    loop_count = 0

    try:
        esp32.open()

        print("[Loop] Start")
        print("[Loop] Press Ctrl + C to stop.")

        while True:
            if (
                ADVANCED_FOLLOW_LOOP_MAX_COUNT is not None
                and loop_count >= ADVANCED_FOLLOW_LOOP_MAX_COUNT
            ):
                print("[Loop] Max count reached.")
                break

            loop_count += 1

            ret, frame = cap.read()

            if not ret:
                print("[Camera] ERROR: フレームを取得できませんでした")
                time.sleep(ADVANCED_FOLLOW_LOOP_INTERVAL)
                continue

            height, width = frame.shape[:2]
            screen_center_x = width // 2
            screen_center_y = height // 2

            faces = detect_faces(frame, face_cascade)
            main_face = select_main_face(faces)

            if main_face is None:
                no_face_count += 1

                if no_face_count >= ADVANCED_NO_FACE_STOP_THRESHOLD:
                    result = {
                        "diff_x": 0,
                        "diff_y": 0,
                        "move_command": "STOP",
                        "head_command": "HEAD_CENTER",
                        "distance_state": "UNKNOWN",
                        "distance_command": "STOP",
                        "final_move_command": "STOP",
                    }
                else:
                    result = {
                        "diff_x": 0,
                        "diff_y": 0,
                        "move_command": last_move_command or "STOP",
                        "head_command": last_head_command or "HEAD_CENTER",
                        "distance_state": "UNKNOWN",
                        "distance_command": "STOP",
                        "final_move_command": last_move_command or "STOP",
                    }

                print(
                    f"[Loop {loop_count}] No face. "
                    f"no_face_count={no_face_count}, "
                    f"move={result['final_move_command']}, "
                    f"head={result['head_command']}"
                )

            else:
                no_face_count = 0

                x, y, w, h = main_face
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                result = decide_advanced_commands(
                    face_center_x,
                    face_center_y,
                    w,
                    screen_center_x,
                    screen_center_y,
                )

                print(
                    f"[Loop {loop_count}] "
                    f"faces={len(faces)}, "
                    f"center=({face_center_x}, {face_center_y}), "
                    f"size=({w}, {h}), "
                    f"diff=({result['diff_x']}, {result['diff_y']}), "
                    f"move={result['final_move_command']}, "
                    f"head={result['head_command']}, "
                    f"distance={result['distance_state']}"
                )

            move_command = result["final_move_command"]
            head_command = result["head_command"]

            # 頭コマンド送信
            if SEND_HEAD_COMMAND:
                should_send_head = True

                if ADVANCED_SEND_ONLY_ON_CHANGE and head_command == last_head_command:
                    should_send_head = False

                if should_send_head:
                    esp32.send(head_command)
                    esp32.read_available(duration=0.15)
                    last_head_command = head_command
                    time.sleep(0.1)

            # 歩行コマンド送信
            if SEND_MOVE_COMMAND:
                should_send_move = True

                if ADVANCED_SEND_ONLY_ON_CHANGE and move_command == last_move_command:
                    should_send_move = False

                if should_send_move:
                    esp32.send(move_command)
                    esp32.read_available(duration=0.15)
                    last_move_command = move_command

            result_frame = draw_result(
                frame,
                faces,
                main_face,
                result,
                loop_count,
            )

            cv2.imwrite(ADVANCED_FOLLOW_LOOP_IMAGE_PATH, result_frame)

            time.sleep(ADVANCED_FOLLOW_LOOP_INTERVAL)

    except KeyboardInterrupt:
        print()
        print("[Loop] Stopped by user.")

    except serial.SerialException as e:
        print("[Serial] ERROR: SerialException")
        print(e)
        print("SERIAL_PORT が合っているか確認してください")

    finally:
        print("[Loop] Sending STOP and HEAD_CENTER for safety.")
        try:
            esp32.send("STOP")
            time.sleep(0.1)
            esp32.send("HEAD_CENTER")
        except Exception:
            pass

        esp32.close()
        cap.release()
        print("[Loop] Finished.")


if __name__ == "__main__":
    main()