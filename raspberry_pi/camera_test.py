import cv2
from config import CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, CENTER_X, CENTER_Y


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("ERROR: カメラを開けませんでした")
        print("CAMERA_INDEX を 0, 1, 2 に変えて試してください")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("Camera test started.")
    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("ERROR: フレームを取得できませんでした")
            break

        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2

        # 画面中心の線
        cv2.line(frame, (center_x, 0), (center_x, height), (0, 255, 0), 1)
        cv2.line(frame, (0, center_y), (width, center_y), (0, 255, 0), 1)

        # 中心点
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

        cv2.imshow("PresentationEscort Camera Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()