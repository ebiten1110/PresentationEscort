import cv2

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
    ADVANCED_FOLLOW_IMAGE_PATH,
)


CASCADE_PATH = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"


def open_camera():
    """
    USBカメラ / IPカメラのどちらかを開く
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
    カメラから1枚取得する
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
    Haar Cascadeを読み込む
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
    複数顔がある場合、一番大きい顔をターゲットにする
    """
    if len(faces) == 0:
        return None

    return max(faces, key=lambda face: face[2] * face[3])


def decide_advanced_commands(face_center_x, face_center_y, face_w, screen_center_x, screen_center_y):
    """
    顔の位置と大きさから、移動・頭・距離の命令を決める
    """
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

    # 実際に歩行へ使う最終移動命令
    # 優先順位：
    # 1. 左右にズレているなら LEFT / RIGHT
    # 2. 中央にいて遠いなら FORWARD
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


def draw_result(frame, faces, main_face, result):
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

    if main_face is None:
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
            "Final Move: STOP",
            (10, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        cv2.putText(
            frame,
            "Head: HEAD_CENTER",
            (10, 115),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        return frame

    x, y, w, h = main_face
    face_center_x = x + w // 2
    face_center_y = y + h // 2

    # メイン顔
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

    cv2.putText(
        frame,
        f"Move: {result['move_command']}",
        (10, 125),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Head: {result['head_command']}",
        (10, 160),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Distance: {result['distance_state']} -> {result['distance_command']}",
        (10, 195),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Final Move: {result['final_move_command']}",
        (10, 235),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
    )

    return frame


def main():
    print("=== Presentation Escort Phase 5.7 ===")
    print("Advanced Follow Test")
    print("X: LEFT / RIGHT / STOP")
    print("Y: HEAD_UP / HEAD_DOWN / HEAD_CENTER")
    print("Distance: FORWARD / STOP")

    frame = capture_frame()

    if frame is None:
        print("[Main] Capture failed.")
        return

    height, width = frame.shape[:2]
    screen_center_x = width // 2
    screen_center_y = height // 2

    faces = detect_faces(frame)
    print(f"[FaceDetect] Detected faces: {len(faces)}")

    main_face = select_main_face(faces)

    if main_face is None:
        print("[Control] No face detected.")
        print("[Control] Final Move Command: STOP")
        print("[Control] Head Command      : HEAD_CENTER")

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

        print("[Control] Main face selected.")
        print(f"  Box                : x={x}, y={y}, w={w}, h={h}")
        print(f"  Face Center        : ({face_center_x}, {face_center_y})")
        print(f"  Screen Center      : ({screen_center_x}, {screen_center_y})")
        print(f"  Diff X             : {result['diff_x']}")
        print(f"  Diff Y             : {result['diff_y']}")
        print(f"  Move Command       : {result['move_command']}")
        print(f"  Head Command       : {result['head_command']}")
        print(f"  Distance State     : {result['distance_state']}")
        print(f"  Distance Command   : {result['distance_command']}")
        print(f"  Final Move Command : {result['final_move_command']}")

    result_frame = draw_result(frame, faces, main_face, result)

    cv2.imwrite(ADVANCED_FOLLOW_IMAGE_PATH, result_frame)
    print(f"[Main] Saved image: {ADVANCED_FOLLOW_IMAGE_PATH}")
    print("[Main] Test finished.")


if __name__ == "__main__":
    main()