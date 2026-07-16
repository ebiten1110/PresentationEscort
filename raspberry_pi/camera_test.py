import cv2
from config import (
    CAMERA_MODE,
    IP_CAMERA_URL,
    CAMERA_INDEX,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    SHOW_WINDOW,
)


def open_camera():
    """
    CAMERA_MODE に応じてカメラを開く
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
    print('[Camera] CAMERA_MODE must be "ip" or "usb"')
    return None


def draw_center_guides(frame):
    """
    画面中央に十字線と中心座標を描画する
    """
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    cv2.line(frame, (center_x, 0), (center_x, height), (0, 255, 0), 1)
    cv2.line(frame, (0, center_y), (width, center_y), (0, 255, 0), 1)
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    cv2.putText(
        frame,
        f"Center: ({center_x}, {center_y})",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return frame


def main():
    cap = open_camera()

    if cap is None:
        return

    if not cap.isOpened():
        print("[Camera] ERROR: カメラを開けませんでした")

        if CAMERA_MODE == "ip":
            print("[Camera] IPカメラのURLが正しいか確認してください")
            print("[Camera] スマホとRaspberry Piが同じWi-Fiか確認してください")
            print("[Camera] 例: http://スマホのIP:8080/video")

        if CAMERA_MODE == "usb":
            print("[Camera] USBカメラが認識されているか確認してください")
            print("[Camera] v4l2-ctl --list-devices で確認してください")
            print("[Camera] CAMERA_INDEX を 0, 1, 2 に変えて試してください")

        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[Camera] Camera test started.")
    print("[Camera] Press 'q' to quit.")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("[Camera] ERROR: フレームを取得できませんでした")
            break

        frame = draw_center_guides(frame)

        if SHOW_WINDOW:
            cv2.imshow("PresentationEscort Camera Test", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        else:
            print("[Camera] Frame captured.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()