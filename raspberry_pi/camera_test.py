import cv2
from config import (
    CAMERA_MODE,
    IP_CAMERA_URL,
    CAMERA_INDEX,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    SHOW_WINDOW,
    SAVE_TEST_IMAGE,
    SAVE_IMAGE_PATH,
)


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


def draw_center_guides(frame):
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
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("[Camera] Camera test started.")

    ret, frame = cap.read()

    if not ret:
        print("[Camera] ERROR: フレームを取得できませんでした")
        cap.release()
        return

    frame = draw_center_guides(frame)

    if SHOW_WINDOW:
        print("[Camera] Show window mode")
        print("[Camera] Press 'q' to quit.")

        while True:
            ret, frame = cap.read()

            if not ret:
                print("[Camera] ERROR: フレームを取得できませんでした")
                break

            frame = draw_center_guides(frame)
            cv2.imshow("PresentationEscort Camera Test", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()

    else:
        print("[Camera] Headless mode")

        if SAVE_TEST_IMAGE:
            cv2.imwrite(SAVE_IMAGE_PATH, frame)
            print(f"[Camera] Saved image: {SAVE_IMAGE_PATH}")

        height, width = frame.shape[:2]
        print(f"[Camera] Frame size: {width} x {height}")
        print("[Camera] Capture OK")

    cap.release()


if __name__ == "__main__":
    main()