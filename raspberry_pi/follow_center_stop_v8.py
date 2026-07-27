"""
Presentation Escort
自動追従 V8

機能:
- 顔の左右方向へTRACK_LEFT / TRACK_RIGHTで小刻みに旋回
- 顔が中央へ到達した瞬間にSTOPを送る
- ESP32側は自動追従旋回だけ即時中断し、STANDへ戻る
- 顔の上下位置を判定し、HEAD_UP / HEAD_DOWNで追従
- 顔検出中はLIGHT_ON、顔ロスト時はLIGHT_OFF
- 顔検出を320px幅へ縮小して高速化

実行:
    python3 follow_center_stop_v8.py

終了:
    Ctrl+C
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import serial

import config as cfg


# ============================================================
# config.pyから読み込む設定
# ============================================================

CAMERA_MODE = getattr(cfg, "CAMERA_MODE", "usb")
IP_CAMERA_URL = getattr(cfg, "IP_CAMERA_URL", "")
CAMERA_INDEX = getattr(cfg, "CAMERA_INDEX", 0)

FRAME_WIDTH = getattr(cfg, "FRAME_WIDTH", 640)
FRAME_HEIGHT = getattr(cfg, "FRAME_HEIGHT", 480)

MIRROR_CAMERA_IMAGE = getattr(
    cfg,
    "MIRROR_CAMERA_IMAGE",
    False,
)

CAMERA_CENTER_OFFSET_X = getattr(
    cfg,
    "CAMERA_CENTER_OFFSET_X",
    0,
)
CAMERA_CENTER_OFFSET_Y = getattr(
    cfg,
    "CAMERA_CENTER_OFFSET_Y",
    0,
)

SERIAL_PORT = getattr(
    cfg,
    "SERIAL_PORT",
    "/dev/ttyUSB0",
)
SERIAL_BAUD = getattr(
    cfg,
    "SERIAL_BAUD",
    115200,
)

CASCADE_PATH = getattr(
    cfg,
    "HAAR_CASCADE_PATH",
    (
        "/usr/share/opencv4/haarcascades/"
        "haarcascade_frontalface_default.xml"
    ),
)

ENABLE_HISTOGRAM_EQUALIZATION = getattr(
    cfg,
    "ENABLE_HISTOGRAM_EQUALIZATION",
    True,
)


# ============================================================
# 自動追従設定
# ============================================================

# 左右方向
# この範囲を超えたら小刻み旋回を開始する。
BODY_TURN_START_X = 85

# 顔がこの範囲へ入ったらSTOPを送る。
BODY_STOP_X = 28

# 旋回開始判定の連続フレーム数。
BODY_START_CONFIRM_FRAMES = 2

# 中央到達・通過判定の連続フレーム数。
BODY_STOP_CONFIRM_FRAMES = 2

# 小刻み旋回終了後の再判定待ち。
BODY_RESTART_COOLDOWN_SECONDS = 0.35

# 上下方向
HEAD_START_Y = 42
HEAD_STOP_Y = 18

# HEAD_UP / HEAD_DOWNを再送する間隔。
HEAD_COMMAND_INTERVAL_SECONDS = 0.22

# 顔ロスト
NO_FACE_BODY_STOP_FRAMES = 2
NO_FACE_LIGHT_OFF_FRAMES = 3
NO_FACE_HEAD_CENTER_FRAMES = 8

# 顔検出を高速化するための幅。
DETECTION_WIDTH = 320

LOOP_INTERVAL_SECONDS = 0.04

RESULT_IMAGE_PATH = Path(
    "follow_center_stop_latest.jpg"
)
SAVE_IMAGE_EVERY_N_LOOPS = 4

PRINT_ALL_ESP32_LINES = False


# ============================================================
# データ
# ============================================================

@dataclass
class FaceResult:
    x: int
    y: int
    width: int
    height: int
    center_x: int
    center_y: int
    diff_x: int
    diff_y: int


# ============================================================
# カメラ・顔検出
# ============================================================

def open_camera() -> cv2.VideoCapture:
    if CAMERA_MODE == "usb":
        print(
            f"[Camera] USB index={CAMERA_INDEX}"
        )
        cap = cv2.VideoCapture(CAMERA_INDEX)

    elif CAMERA_MODE == "ip":
        print(
            f"[Camera] IP url={IP_CAMERA_URL}"
        )
        cap = cv2.VideoCapture(IP_CAMERA_URL)

    else:
        raise ValueError(
            f"CAMERA_MODEが不正です: {CAMERA_MODE}"
        )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        FRAME_WIDTH,
    )
    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        FRAME_HEIGHT,
    )
    cap.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1,
    )

    return cap


def load_cascade() -> cv2.CascadeClassifier:
    cascade = cv2.CascadeClassifier(
        CASCADE_PATH
    )

    if cascade.empty():
        raise RuntimeError(
            "Haar Cascadeを読み込めません: "
            f"{CASCADE_PATH}"
        )

    return cascade


def detect_main_face(
    frame,
    cascade: cv2.CascadeClassifier,
) -> Tuple[Optional[FaceResult], list]:
    frame_height, frame_width = frame.shape[:2]

    scale = (
        DETECTION_WIDTH
        / float(frame_width)
    )

    detection_height = max(
        1,
        int(frame_height * scale),
    )

    small = cv2.resize(
        frame,
        (
            DETECTION_WIDTH,
            detection_height,
        ),
        interpolation=cv2.INTER_AREA,
    )

    gray = cv2.cvtColor(
        small,
        cv2.COLOR_BGR2GRAY,
    )

    if ENABLE_HISTOGRAM_EQUALIZATION:
        gray = cv2.equalizeHist(gray)

    small_faces = cascade.detectMultiScale(
        gray,
        scaleFactor=1.15,
        minNeighbors=4,
        minSize=(24, 24),
    )

    if len(small_faces) == 0:
        return None, []

    sx, sy, sw, sh = max(
        small_faces,
        key=lambda item: item[2] * item[3],
    )

    inverse_scale = 1.0 / scale

    x = int(sx * inverse_scale)
    y = int(sy * inverse_scale)
    width = int(sw * inverse_scale)
    height = int(sh * inverse_scale)

    center_x = x + width // 2
    center_y = y + height // 2

    target_center_x = (
        frame_width // 2
        + CAMERA_CENTER_OFFSET_X
    )
    target_center_y = (
        frame_height // 2
        + CAMERA_CENTER_OFFSET_Y
    )

    face = FaceResult(
        x=x,
        y=y,
        width=width,
        height=height,
        center_x=center_x,
        center_y=center_y,
        diff_x=center_x - target_center_x,
        diff_y=center_y - target_center_y,
    )

    return face, [(x, y, width, height)]


# ============================================================
# ESP32シリアル
# ============================================================

class ESP32Serial:
    IMPORTANT_PREFIXES = (
        "[Walking]",
        "[MotionPlayer] Play:",
        "[MotionPlayer] Finished:",
        "[SerialReceive]",
        "Walking State:",
        "Brownout",
        "Guru Meditation",
    )

    def __init__(self) -> None:
        self.ser: Optional[serial.Serial] = None
        self.buffer = ""

        self.walking_state = "UNKNOWN"
        self.busy = False

        self.active_track: Optional[str] = None
        self.stop_sent_for_track = False

        self.last_idle_time = time.monotonic()
        self.fault = False

    def open(self) -> None:
        print(
            f"[Serial] Open {SERIAL_PORT} "
            f"baud={SERIAL_BAUD}"
        )

        self.ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=0,
            write_timeout=1.0,
        )

        time.sleep(2.0)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        print("[Serial] Connected")

        self.send("LIGHT_OFF")
        self.send("STATUS")

        # STATUS応答を少し待つ。
        wait_until = time.monotonic() + 0.8

        while time.monotonic() < wait_until:
            self.poll()
            time.sleep(0.02)

    def close(self) -> None:
        if (
            self.ser is not None
            and self.ser.is_open
        ):
            self.ser.close()
            print("[Serial] Closed")

    def send(self, command: str) -> bool:
        if (
            self.ser is None
            or not self.ser.is_open
        ):
            return False

        message = (
            command.strip().upper()
            + "\n"
        )

        try:
            self.ser.write(
                message.encode("utf-8")
            )
            self.ser.flush()

        except serial.SerialException as error:
            print(
                f"[Serial] ERROR: {error}"
            )
            self.fault = True
            return False

        print(f"[Serial] Sent: {command}")
        return True

    def send_track(self, command: str) -> bool:
        if self.busy or self.fault:
            return False

        if command not in (
            "TRACK_LEFT",
            "TRACK_RIGHT",
        ):
            raise ValueError(command)

        if not self.send(command):
            return False

        self.busy = True
        self.active_track = command
        self.stop_sent_for_track = False

        print(
            f"[Body] START {command}"
        )
        return True

    def send_center_stop(self, reason: str) -> bool:
        if self.stop_sent_for_track:
            return False

        if self.active_track is None:
            return False

        if not self.send("STOP"):
            return False

        self.stop_sent_for_track = True

        print(
            "[Body] CENTER STOP "
            f"active={self.active_track} "
            f"reason={reason}"
        )

        return True

    def poll(self) -> None:
        if (
            self.ser is None
            or not self.ser.is_open
        ):
            return

        waiting = self.ser.in_waiting

        if waiting <= 0:
            return

        data = self.ser.read(waiting)

        self.buffer += data.decode(
            "utf-8",
            errors="ignore",
        )

        while "\n" in self.buffer:
            line, self.buffer = (
                self.buffer.split("\n", 1)
            )

            line = line.strip("\r ")

            if line:
                self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        if (
            PRINT_ALL_ESP32_LINES
            or line.startswith(
                self.IMPORTANT_PREFIXES
            )
        ):
            print(f"[ESP32] {line}")

        state = None

        if "[Walking] State:" in line:
            state = (
                line.split(
                    "[Walking] State:",
                    1,
                )[1]
                .strip()
                .upper()
            )

        elif line.startswith("Walking State:"):
            state = (
                line.split(
                    "Walking State:",
                    1,
                )[1]
                .strip()
                .upper()
            )

        if state is not None:
            self.walking_state = state

            if state == "IDLE":
                self.busy = False
                self.active_track = None
                self.stop_sent_for_track = False
                self.last_idle_time = (
                    time.monotonic()
                )

                print("[Body] IDLE")
            else:
                self.busy = True

        if (
            "Brownout" in line
            or "Guru Meditation" in line
        ):
            self.fault = True

    def ready_for_new_track(self) -> bool:
        if self.busy or self.fault:
            return False

        return (
            time.monotonic()
            - self.last_idle_time
            >= BODY_RESTART_COOLDOWN_SECONDS
        )


# ============================================================
# 補助制御
# ============================================================

class RepeatedCommand:
    def __init__(self) -> None:
        self.last_command: Optional[str] = None
        self.last_sent_time = 0.0

    def send_if_due(
        self,
        esp32: ESP32Serial,
        command: Optional[str],
        interval: float,
    ) -> bool:
        if command is None:
            self.last_command = None
            return False

        now = time.monotonic()

        command_changed = (
            command != self.last_command
        )

        interval_elapsed = (
            now - self.last_sent_time
            >= interval
        )

        if not (
            command_changed
            or interval_elapsed
        ):
            return False

        if not esp32.send(command):
            return False

        self.last_command = command
        self.last_sent_time = now
        return True


# ============================================================
# 描画
# ============================================================

def draw_result(
    frame,
    face: Optional[FaceResult],
    esp32: ESP32Serial,
    horizontal: str,
    vertical: str,
    light_on: bool,
) -> None:
    height, width = frame.shape[:2]

    center_x = (
        width // 2
        + CAMERA_CENTER_OFFSET_X
    )
    center_y = (
        height // 2
        + CAMERA_CENTER_OFFSET_Y
    )

    cv2.line(
        frame,
        (center_x, 0),
        (center_x, height),
        (0, 255, 0),
        1,
    )
    cv2.line(
        frame,
        (0, center_y),
        (width, center_y),
        (0, 255, 0),
        1,
    )

    for offset in (
        -BODY_STOP_X,
        BODY_STOP_X,
    ):
        cv2.line(
            frame,
            (center_x + offset, 0),
            (center_x + offset, height),
            (0, 255, 255),
            1,
        )

    for offset in (
        -HEAD_STOP_Y,
        HEAD_STOP_Y,
    ):
        cv2.line(
            frame,
            (0, center_y + offset),
            (width, center_y + offset),
            (255, 255, 0),
            1,
        )

    if face is not None:
        cv2.rectangle(
            frame,
            (face.x, face.y),
            (
                face.x + face.width,
                face.y + face.height,
            ),
            (255, 0, 0),
            2,
        )

        cv2.circle(
            frame,
            (
                face.center_x,
                face.center_y,
            ),
            5,
            (0, 255, 255),
            -1,
        )

        cv2.putText(
            frame,
            (
                f"diff=({face.diff_x},"
                f"{face.diff_y})"
            ),
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

    cv2.putText(
        frame,
        (
            f"X={horizontal} "
            f"Y={vertical}"
        ),
        (10, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        (
            f"ESP32={esp32.walking_state} "
            f"active={esp32.active_track}"
        ),
        (10, 85),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Light={'ON' if light_on else 'OFF'}",
        (10, 115),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (255, 255, 255),
        2,
    )


# ============================================================
# メイン
# ============================================================

def main() -> None:
    print(
        "=============================================="
    )
    print(
        "Presentation Escort Auto Follow V8"
    )
    print(
        "Micro turn + center stop + Y tracking + light"
    )
    print(
        "=============================================="
    )

    cascade = load_cascade()
    cap = open_camera()

    if not cap.isOpened():
        raise RuntimeError(
            "カメラを開けませんでした。"
        )

    # 起動直後の画像を捨てる。
    for _ in range(6):
        cap.read()

    esp32 = ESP32Serial()

    head_sender = RepeatedCommand()

    light_on = False
    no_face_count = 0

    body_start_candidate: Optional[str] = None
    body_start_count = 0
    body_stop_count = 0

    loop_count = 0

    try:
        esp32.open()

        while True:
            loop_started = time.monotonic()
            loop_count += 1

            esp32.poll()

            ret, frame = cap.read()

            if not ret:
                raise RuntimeError(
                    "カメラ画像を取得できません。"
                )

            if MIRROR_CAMERA_IMAGE:
                frame = cv2.flip(frame, 1)

            face, _ = detect_main_face(
                frame,
                cascade,
            )

            horizontal_state = "NO_FACE"
            vertical_state = "NO_FACE"

            if face is None:
                no_face_count += 1

                body_start_candidate = None
                body_start_count = 0
                body_stop_count = 0

                # 顔を見失った状態で旋回中なら早めに止める。
                if (
                    no_face_count
                    >= NO_FACE_BODY_STOP_FRAMES
                ):
                    esp32.send_center_stop(
                        "FACE_LOST"
                    )

                if (
                    light_on
                    and no_face_count
                    >= NO_FACE_LIGHT_OFF_FRAMES
                ):
                    if esp32.send("LIGHT_OFF"):
                        light_on = False

                if (
                    no_face_count
                    == NO_FACE_HEAD_CENTER_FRAMES
                ):
                    esp32.send("HEAD_CENTER")
                    head_sender.last_command = None

                print(
                    f"[Loop {loop_count:04d}] "
                    f"NO_FACE count={no_face_count} "
                    f"state={esp32.walking_state}"
                )

            else:
                no_face_count = 0

                # 顔検出中はライトを点灯。
                if not light_on:
                    if esp32.send("LIGHT_ON"):
                        light_on = True

                # ------------------------------------------------
                # 上下追従
                # ------------------------------------------------
                if face.diff_y <= -HEAD_START_Y:
                    vertical_state = "UP"
                    head_command = "HEAD_UP"

                elif face.diff_y >= HEAD_START_Y:
                    vertical_state = "DOWN"
                    head_command = "HEAD_DOWN"

                elif abs(face.diff_y) <= HEAD_STOP_Y:
                    vertical_state = "CENTER"
                    head_command = None

                else:
                    vertical_state = "HOLD"
                    head_command = None

                head_sender.send_if_due(
                    esp32,
                    head_command,
                    HEAD_COMMAND_INTERVAL_SECONDS,
                )

                # ------------------------------------------------
                # 左右追従
                # ------------------------------------------------
                if esp32.active_track == "TRACK_LEFT":
                    horizontal_state = "TRACKING_LEFT"

                    # 左へ旋回中、顔が中央へ入るか通過したらSTOP。
                    reached_center = (
                        face.diff_x >= -BODY_STOP_X
                    )

                    if reached_center:
                        body_stop_count += 1
                    else:
                        body_stop_count = 0

                    if (
                        body_stop_count
                        >= BODY_STOP_CONFIRM_FRAMES
                    ):
                        esp32.send_center_stop(
                            (
                                "LEFT_REACHED_CENTER "
                                f"diffX={face.diff_x}"
                            )
                        )

                elif esp32.active_track == "TRACK_RIGHT":
                    horizontal_state = "TRACKING_RIGHT"

                    # 右へ旋回中、顔が中央へ入るか通過したらSTOP。
                    reached_center = (
                        face.diff_x <= BODY_STOP_X
                    )

                    if reached_center:
                        body_stop_count += 1
                    else:
                        body_stop_count = 0

                    if (
                        body_stop_count
                        >= BODY_STOP_CONFIRM_FRAMES
                    ):
                        esp32.send_center_stop(
                            (
                                "RIGHT_REACHED_CENTER "
                                f"diffX={face.diff_x}"
                            )
                        )

                elif esp32.busy:
                    horizontal_state = (
                        esp32.walking_state
                    )

                    body_start_candidate = None
                    body_start_count = 0
                    body_stop_count = 0

                else:
                    body_stop_count = 0

                    if abs(face.diff_x) <= BODY_STOP_X:
                        horizontal_state = "CENTER"

                        body_start_candidate = None
                        body_start_count = 0

                    elif face.diff_x <= -BODY_TURN_START_X:
                        horizontal_state = "LEFT"
                        candidate = "TRACK_LEFT"

                        if (
                            candidate
                            == body_start_candidate
                        ):
                            body_start_count += 1
                        else:
                            body_start_candidate = candidate
                            body_start_count = 1

                    elif face.diff_x >= BODY_TURN_START_X:
                        horizontal_state = "RIGHT"
                        candidate = "TRACK_RIGHT"

                        if (
                            candidate
                            == body_start_candidate
                        ):
                            body_start_count += 1
                        else:
                            body_start_candidate = candidate
                            body_start_count = 1

                    else:
                        horizontal_state = "HOLD"

                        body_start_candidate = None
                        body_start_count = 0

                    if (
                        body_start_candidate is not None
                        and body_start_count
                        >= BODY_START_CONFIRM_FRAMES
                        and esp32.ready_for_new_track()
                    ):
                        if esp32.send_track(
                            body_start_candidate
                        ):
                            body_start_candidate = None
                            body_start_count = 0

                print(
                    f"[Loop {loop_count:04d}] "
                    f"diff=({face.diff_x},"
                    f"{face.diff_y}) "
                    f"X={horizontal_state} "
                    f"Y={vertical_state} "
                    f"state={esp32.walking_state} "
                    f"active={esp32.active_track} "
                    f"light={'ON' if light_on else 'OFF'}"
                )

            if (
                loop_count
                % SAVE_IMAGE_EVERY_N_LOOPS
                == 0
            ):
                draw_result(
                    frame,
                    face,
                    esp32,
                    horizontal_state,
                    vertical_state,
                    light_on,
                )

                cv2.imwrite(
                    str(RESULT_IMAGE_PATH),
                    frame,
                )

            if esp32.fault:
                print(
                    "[Safety] ESP32 fault detected."
                )
                break

            elapsed = (
                time.monotonic()
                - loop_started
            )

            sleep_time = (
                LOOP_INTERVAL_SECONDS
                - elapsed
            )

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print()
        print("[Main] Ctrl+C")

    except Exception as error:
        print(
            "[Main] ERROR: "
            f"{type(error).__name__}: {error}"
        )

    finally:
        try:
            esp32.send("STOP")
            time.sleep(0.15)
            esp32.send("HEAD_CENTER")
            esp32.send("LIGHT_OFF")

        except Exception as error:
            print(
                "[Safety] 終了処理失敗: "
                f"{error}"
            )

        esp32.close()
        cap.release()

        print("[Main] Finished.")


if __name__ == "__main__":
    main()
