"""
Presentation Escort 自動追従 V8.3

機能:
- 顔の左右位置に合わせてTRACK_LEFT / TRACK_RIGHT
- 顔の上下位置に合わせてHEAD_UP / HEAD_DOWN
- 顔検出中はライト点灯
- 顔の高さ比率でFAR / GOOD / NEARを判定
- FARならTRACK_FORWARDを1パルス
- NEARならTRACK_BACKWARDを1パルス
- GOODなら前後移動しない
- 前後移動は顔が左右中央付近にある場合だけ実行
- 1パルス終了ごとに距離を再判定

実行:
    python3 follow_distance_move_v8_3.py
"""

from __future__ import annotations

import os
import shutil
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional, Tuple

import cv2
import serial

import config as cfg


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

INVERT_HORIZONTAL_TRACKING = getattr(
    cfg,
    "INVERT_HORIZONTAL_TRACKING",
    True,
)

# TRACK_FORWARD / TRACK_BACKWARDの実機方向が逆の場合にTrue。
INVERT_DISTANCE_MOVEMENT = getattr(
    cfg,
    "INVERT_DISTANCE_MOVEMENT",
    False,
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
# 左右・上下
# ============================================================

BODY_TURN_START_X = getattr(
    cfg,
    "BODY_TURN_START_X",
    85,
)
BODY_STOP_X = getattr(
    cfg,
    "BODY_STOP_X",
    28,
)

BODY_START_CONFIRM_FRAMES = 2
BODY_STOP_CONFIRM_FRAMES = 2
BODY_RESTART_COOLDOWN_SECONDS = 0.40

HEAD_START_Y = 42
HEAD_STOP_Y = 18
HEAD_COMMAND_INTERVAL_SECONDS = 0.22


# ============================================================
# 距離移動
# ============================================================

# 実行ログに合わせ、FARを検出可能な範囲へ変更。
DISTANCE_FAR_ENTER_RATIO = getattr(
    cfg,
    "DISTANCE_FAR_ENTER_RATIO",
    0.24,
)
DISTANCE_FAR_EXIT_RATIO = getattr(
    cfg,
    "DISTANCE_FAR_EXIT_RATIO",
    0.26,
)

DISTANCE_NEAR_ENTER_RATIO = getattr(
    cfg,
    "DISTANCE_NEAR_ENTER_RATIO",
    0.34,
)
DISTANCE_NEAR_EXIT_RATIO = getattr(
    cfg,
    "DISTANCE_NEAR_EXIT_RATIO",
    0.31,
)

DISTANCE_SMOOTHING_FRAMES = getattr(
    cfg,
    "DISTANCE_SMOOTHING_FRAMES",
    5,
)

DISTANCE_CONFIRM_FRAMES = getattr(
    cfg,
    "DISTANCE_CONFIRM_FRAMES",
    3,
)

DISTANCE_MOVEMENT_ENABLED = getattr(
    cfg,
    "DISTANCE_MOVEMENT_ENABLED",
    True,
)

# 顔が左右中央から外れている場合、前後移動より旋回を優先。
DISTANCE_MOVE_MAX_X = getattr(
    cfg,
    "DISTANCE_MOVE_MAX_X",
    55,
)

DISTANCE_RESTART_COOLDOWN_SECONDS = getattr(
    cfg,
    "DISTANCE_RESTART_COOLDOWN_SECONDS",
    0.55,
)

# テスト中の暴走防止。
# GOODを一度確認するまでに行える連続パルス数。
MAX_CONSECUTIVE_DISTANCE_PULSES = getattr(
    cfg,
    "MAX_CONSECUTIVE_DISTANCE_PULSES",
    4,
)


# ============================================================
# 顔ロスト・カメラ
# ============================================================

NO_FACE_BODY_STOP_FRAMES = 2
NO_FACE_LIGHT_OFF_FRAMES = 8
NO_FACE_HEAD_CENTER_FRAMES = 8
NO_FACE_DISTANCE_RESET_FRAMES = 5

DETECTION_WIDTH = 320
LOOP_INTERVAL_SECONDS = 0.04

RESULT_IMAGE_PATH = Path(
    "follow_distance_move_latest.jpg"
)
SAVE_IMAGE_EVERY_N_LOOPS = 4

SHOW_PREVIEW_WINDOW_REQUESTED = getattr(
    cfg,
    "SHOW_PREVIEW_WINDOW",
    False,
)
PREVIEW_WINDOW_NAME = "Presentation Escort V8.3"

PRINT_ALL_ESP32_LINES = False


def can_use_preview_window() -> bool:
    if not SHOW_PREVIEW_WINDOW_REQUESTED:
        print("[Preview] Disabled by config.")
        return False

    display = os.environ.get("DISPLAY", "").strip()
    wayland_display = os.environ.get(
        "WAYLAND_DISPLAY",
        "",
    ).strip()

    if display:
        xdpyinfo = shutil.which("xdpyinfo")

        if xdpyinfo is None:
            print(
                "[Preview] Disabled: "
                "xdpyinfo is not installed."
            )
            return False

        try:
            result = subprocess.run(
                [xdpyinfo, "-display", display],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0,
                check=False,
            )
        except (
            OSError,
            subprocess.TimeoutExpired,
        ):
            return False

        if result.returncode == 0:
            print(
                f"[Preview] Enabled: DISPLAY={display}"
            )
            return True

        print(
            "[Preview] Disabled: "
            f"cannot connect to DISPLAY={display}"
        )
        return False

    runtime_dir = os.environ.get(
        "XDG_RUNTIME_DIR",
        "",
    ).strip()

    if wayland_display and runtime_dir:
        socket_path = (
            Path(runtime_dir)
            / wayland_display
        )

        if socket_path.exists():
            print(
                "[Preview] Enabled: "
                f"Wayland={wayland_display}"
            )
            return True

    print(
        "[Preview] Disabled: "
        "usable display not found."
    )
    return False


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

    faces = cascade.detectMultiScale(
        gray,
        scaleFactor=1.15,
        minNeighbors=4,
        minSize=(24, 24),
    )

    if len(faces) == 0:
        return None, []

    sx, sy, sw, sh = max(
        faces,
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

    return (
        FaceResult(
            x=x,
            y=y,
            width=width,
            height=height,
            center_x=center_x,
            center_y=center_y,
            diff_x=center_x - target_center_x,
            diff_y=center_y - target_center_y,
        ),
        [(x, y, width, height)],
    )


def get_horizontal_control_error(
    raw_diff_x: int,
) -> int:
    if INVERT_HORIZONTAL_TRACKING:
        return -raw_diff_x

    return raw_diff_x


class DistanceEstimator:
    def __init__(self) -> None:
        self.history: Deque[float] = deque(
            maxlen=max(
                1,
                int(DISTANCE_SMOOTHING_FRAMES),
            )
        )
        self.state = "UNKNOWN"
        self.ratio: Optional[float] = None

    def reset(self) -> None:
        self.history.clear()
        self.state = "UNKNOWN"
        self.ratio = None

    def update(
        self,
        face_height: int,
        frame_height: int,
    ) -> Tuple[str, float, bool]:
        raw_ratio = (
            float(face_height)
            / float(frame_height)
        )

        self.history.append(raw_ratio)

        ratio = (
            sum(self.history)
            / len(self.history)
        )

        previous = self.state
        self.ratio = ratio

        if self.state == "FAR":
            if ratio >= DISTANCE_FAR_EXIT_RATIO:
                self.state = (
                    "NEAR"
                    if ratio > DISTANCE_NEAR_ENTER_RATIO
                    else "GOOD"
                )

        elif self.state == "NEAR":
            if ratio <= DISTANCE_NEAR_EXIT_RATIO:
                self.state = (
                    "FAR"
                    if ratio < DISTANCE_FAR_ENTER_RATIO
                    else "GOOD"
                )

        else:
            if ratio < DISTANCE_FAR_ENTER_RATIO:
                self.state = "FAR"
            elif ratio > DISTANCE_NEAR_ENTER_RATIO:
                self.state = "NEAR"
            else:
                self.state = "GOOD"

        return (
            self.state,
            ratio,
            self.state != previous,
        )


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
        self.active_body: Optional[str] = None
        self.stop_sent = False

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

        limit = time.monotonic() + 0.8

        while time.monotonic() < limit:
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

        try:
            self.ser.write(
                (
                    command.strip().upper()
                    + "\n"
                ).encode("utf-8")
            )
            self.ser.flush()

        except serial.SerialException as error:
            print(f"[Serial] ERROR: {error}")
            self.fault = True
            return False

        print(f"[Serial] Sent: {command}")
        return True

    def send_body(self, command: str) -> bool:
        allowed = {
            "TRACK_LEFT",
            "TRACK_RIGHT",
            "TRACK_FORWARD",
            "TRACK_BACKWARD",
        }

        if command not in allowed:
            raise ValueError(command)

        if self.busy or self.fault:
            return False

        if not self.send(command):
            return False

        self.busy = True
        self.active_body = command
        self.stop_sent = False

        print(f"[Body] START {command}")
        return True

    def send_stop(self, reason: str) -> bool:
        if self.active_body is None:
            return False

        if self.stop_sent:
            return False

        if not self.send("STOP"):
            return False

        self.stop_sent = True

        print(
            "[Body] STOP "
            f"active={self.active_body} "
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
                self.active_body = None
                self.stop_sent = False
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

    def ready_for_body_command(
        self,
        cooldown: float,
    ) -> bool:
        if self.busy or self.fault:
            return False

        return (
            time.monotonic()
            - self.last_idle_time
            >= cooldown
        )


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

        if (
            command == self.last_command
            and now - self.last_sent_time < interval
        ):
            return False

        if not esp32.send(command):
            return False

        self.last_command = command
        self.last_sent_time = now
        return True


def map_distance_command(
    distance_state: str,
) -> Optional[str]:
    command: Optional[str]

    if distance_state == "FAR":
        command = "TRACK_FORWARD"
    elif distance_state == "NEAR":
        command = "TRACK_BACKWARD"
    else:
        return None

    if INVERT_DISTANCE_MOVEMENT:
        if command == "TRACK_FORWARD":
            return "TRACK_BACKWARD"
        return "TRACK_FORWARD"

    return command


def distance_action_label(
    distance_state: str,
) -> str:
    command = map_distance_command(
        distance_state
    )

    if command is None:
        return "HOLD"

    if not DISTANCE_MOVEMENT_ENABLED:
        return f"{command}_DISABLED"

    return command


def draw_result(
    frame,
    face: Optional[FaceResult],
    esp32: ESP32Serial,
    horizontal_state: str,
    vertical_state: str,
    distance_state: str,
    distance_ratio: Optional[float],
    distance_action: str,
    light_on: bool,
    pulse_count: int,
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

        control_x = get_horizontal_control_error(
            face.diff_x
        )

        cv2.putText(
            frame,
            (
                f"rawX={face.diff_x} "
                f"ctrlX={control_x} "
                f"diffY={face.diff_y}"
            ),
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            2,
        )

    ratio_text = (
        "---"
        if distance_ratio is None
        else f"{distance_ratio:.3f}"
    )

    lines = (
        f"X={horizontal_state} Y={vertical_state}",
        (
            f"Distance={distance_state} "
            f"Ratio={ratio_text}"
        ),
        (
            f"Action={distance_action} "
            f"Pulse={pulse_count}/"
            f"{MAX_CONSECUTIVE_DISTANCE_PULSES}"
        ),
        (
            f"ESP32={esp32.walking_state} "
            f"active={esp32.active_body}"
        ),
        (
            f"Light={'ON' if light_on else 'OFF'} "
            f"Move={DISTANCE_MOVEMENT_ENABLED}"
        ),
    )

    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (10, 55 + index * 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            2,
        )


def main() -> None:
    print(
        "=============================================="
    )
    print(
        "Presentation Escort Auto Follow V8.3"
    )
    print(
        "Distance movement test"
    )
    print(
        "=============================================="
    )

    print(
        "[DistanceConfig] "
        f"FAR={DISTANCE_FAR_ENTER_RATIO:.3f}/"
        f"{DISTANCE_FAR_EXIT_RATIO:.3f} "
        f"NEAR={DISTANCE_NEAR_ENTER_RATIO:.3f}/"
        f"{DISTANCE_NEAR_EXIT_RATIO:.3f}"
    )
    print(
        "[DistanceConfig] "
        f"MOVEMENT={DISTANCE_MOVEMENT_ENABLED} "
        f"INVERT={INVERT_DISTANCE_MOVEMENT} "
        f"MAX_X={DISTANCE_MOVE_MAX_X} "
        f"MAX_PULSES="
        f"{MAX_CONSECUTIVE_DISTANCE_PULSES}"
    )

    preview_enabled = can_use_preview_window()

    cascade = load_cascade()
    cap = open_camera()

    if not cap.isOpened():
        raise RuntimeError(
            "カメラを開けませんでした。"
        )

    for _ in range(6):
        cap.read()

    esp32 = ESP32Serial()
    head_sender = RepeatedCommand()
    distance_estimator = DistanceEstimator()

    light_on = False
    no_face_count = 0

    body_start_candidate: Optional[str] = None
    body_start_count = 0
    body_stop_count = 0

    distance_candidate: Optional[str] = None
    distance_candidate_count = 0
    consecutive_distance_pulses = 0
    last_distance_command: Optional[str] = None

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
            distance_state = "NO_FACE"
            distance_ratio: Optional[float] = None
            distance_action = "NONE"

            if face is None:
                no_face_count += 1

                body_start_candidate = None
                body_start_count = 0
                body_stop_count = 0

                distance_candidate = None
                distance_candidate_count = 0

                if (
                    no_face_count
                    >= NO_FACE_BODY_STOP_FRAMES
                ):
                    esp32.send_stop("FACE_LOST")

                if (
                    no_face_count
                    >= NO_FACE_DISTANCE_RESET_FRAMES
                ):
                    distance_estimator.reset()

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
                    f"active={esp32.active_body}"
                )

            else:
                no_face_count = 0

                (
                    distance_state,
                    distance_ratio,
                    distance_changed,
                ) = distance_estimator.update(
                    face.height,
                    frame.shape[0],
                )

                distance_action = (
                    distance_action_label(
                        distance_state
                    )
                )

                if distance_changed:
                    print(
                        "[Distance] "
                        f"State={distance_state} "
                        f"FaceH={face.height}px "
                        f"Ratio={distance_ratio:.3f} "
                        f"Action={distance_action}"
                    )

                if distance_state == "GOOD":
                    consecutive_distance_pulses = 0
                    last_distance_command = None

                if not light_on:
                    if esp32.send("LIGHT_ON"):
                        light_on = True

                # 上下
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

                control_x = (
                    get_horizontal_control_error(
                        face.diff_x
                    )
                )

                # --------------------------------------------
                # 現在動作中の停止判定
                # --------------------------------------------

                if esp32.active_body == "TRACK_LEFT":
                    horizontal_state = "TRACKING_LEFT"

                    if control_x >= -BODY_STOP_X:
                        body_stop_count += 1
                    else:
                        body_stop_count = 0

                    if (
                        body_stop_count
                        >= BODY_STOP_CONFIRM_FRAMES
                    ):
                        esp32.send_stop(
                            (
                                "LEFT_CENTER "
                                f"ctrlX={control_x}"
                            )
                        )

                elif esp32.active_body == "TRACK_RIGHT":
                    horizontal_state = "TRACKING_RIGHT"

                    if control_x <= BODY_STOP_X:
                        body_stop_count += 1
                    else:
                        body_stop_count = 0

                    if (
                        body_stop_count
                        >= BODY_STOP_CONFIRM_FRAMES
                    ):
                        esp32.send_stop(
                            (
                                "RIGHT_CENTER "
                                f"ctrlX={control_x}"
                            )
                        )

                elif esp32.active_body in (
                    "TRACK_FORWARD",
                    "TRACK_BACKWARD",
                ):
                    horizontal_state = "DISTANCE_MOVING"

                    # 途中で横へ大きく外れた、GOODになった、
                    # または逆方向が必要になった場合は停止予約。
                    expected = esp32.active_body
                    requested = map_distance_command(
                        distance_state
                    )

                    if abs(control_x) > DISTANCE_MOVE_MAX_X:
                        esp32.send_stop(
                            (
                                "HORIZONTAL_DRIFT "
                                f"ctrlX={control_x}"
                            )
                        )

                    elif requested != expected:
                        esp32.send_stop(
                            (
                                "DISTANCE_CHANGED "
                                f"state={distance_state}"
                            )
                        )

                elif esp32.busy:
                    horizontal_state = (
                        esp32.walking_state
                    )

                # --------------------------------------------
                # IDLE時の次動作
                # --------------------------------------------

                else:
                    body_stop_count = 0

                    if abs(control_x) <= BODY_STOP_X:
                        horizontal_state = "CENTER"

                    elif control_x <= -BODY_TURN_START_X:
                        horizontal_state = "LEFT"

                    elif control_x >= BODY_TURN_START_X:
                        horizontal_state = "RIGHT"

                    else:
                        horizontal_state = "HOLD"

                    turn_candidate: Optional[str] = None

                    if control_x <= -BODY_TURN_START_X:
                        turn_candidate = "TRACK_LEFT"
                    elif control_x >= BODY_TURN_START_X:
                        turn_candidate = "TRACK_RIGHT"

                    if turn_candidate is not None:
                        distance_candidate = None
                        distance_candidate_count = 0

                        if (
                            turn_candidate
                            == body_start_candidate
                        ):
                            body_start_count += 1
                        else:
                            body_start_candidate = (
                                turn_candidate
                            )
                            body_start_count = 1

                        if (
                            body_start_count
                            >= BODY_START_CONFIRM_FRAMES
                            and esp32.ready_for_body_command(
                                BODY_RESTART_COOLDOWN_SECONDS
                            )
                        ):
                            if esp32.send_body(
                                turn_candidate
                            ):
                                body_start_candidate = None
                                body_start_count = 0

                    else:
                        body_start_candidate = None
                        body_start_count = 0

                        requested_distance_command = (
                            map_distance_command(
                                distance_state
                            )
                        )

                        can_move_distance = (
                            DISTANCE_MOVEMENT_ENABLED
                            and requested_distance_command
                            is not None
                            and abs(control_x)
                            <= DISTANCE_MOVE_MAX_X
                        )

                        if can_move_distance:
                            if (
                                requested_distance_command
                                != last_distance_command
                            ):
                                consecutive_distance_pulses = 0
                                last_distance_command = (
                                    requested_distance_command
                                )

                            if (
                                requested_distance_command
                                == distance_candidate
                            ):
                                distance_candidate_count += 1
                            else:
                                distance_candidate = (
                                    requested_distance_command
                                )
                                distance_candidate_count = 1

                            pulse_limit_ok = (
                                consecutive_distance_pulses
                                < MAX_CONSECUTIVE_DISTANCE_PULSES
                            )

                            if (
                                distance_candidate_count
                                >= DISTANCE_CONFIRM_FRAMES
                                and pulse_limit_ok
                                and esp32.ready_for_body_command(
                                    DISTANCE_RESTART_COOLDOWN_SECONDS
                                )
                            ):
                                if esp32.send_body(
                                    requested_distance_command
                                ):
                                    consecutive_distance_pulses += 1
                                    distance_candidate = None
                                    distance_candidate_count = 0

                            elif not pulse_limit_ok:
                                distance_action = (
                                    "SAFETY_PULSE_LIMIT"
                                )

                        else:
                            distance_candidate = None
                            distance_candidate_count = 0

                print(
                    f"[Loop {loop_count:04d}] "
                    f"rawX={face.diff_x} "
                    f"ctrlX={control_x} "
                    f"diffY={face.diff_y} "
                    f"FaceH={face.height}px "
                    f"Distance={distance_state} "
                    f"Ratio={distance_ratio:.3f} "
                    f"Action={distance_action} "
                    f"X={horizontal_state} "
                    f"Y={vertical_state} "
                    f"Pulse={consecutive_distance_pulses}/"
                    f"{MAX_CONSECUTIVE_DISTANCE_PULSES} "
                    f"state={esp32.walking_state} "
                    f"active={esp32.active_body}"
                )

            if (
                loop_count
                % SAVE_IMAGE_EVERY_N_LOOPS
                == 0
            ):
                output_frame = frame.copy()

                draw_result(
                    output_frame,
                    face,
                    esp32,
                    horizontal_state,
                    vertical_state,
                    distance_state,
                    distance_ratio,
                    distance_action,
                    light_on,
                    consecutive_distance_pulses,
                )

                cv2.imwrite(
                    str(RESULT_IMAGE_PATH),
                    output_frame,
                )

            if preview_enabled:
                preview_frame = frame.copy()

                draw_result(
                    preview_frame,
                    face,
                    esp32,
                    horizontal_state,
                    vertical_state,
                    distance_state,
                    distance_ratio,
                    distance_action,
                    light_on,
                    consecutive_distance_pulses,
                )

                cv2.imshow(
                    PREVIEW_WINDOW_NAME,
                    preview_frame,
                )

                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), 27):
                    break

            if esp32.fault:
                print("[Safety] ESP32 fault detected.")
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

        if preview_enabled:
            cv2.destroyAllWindows()

        print("[Main] Finished.")


if __name__ == "__main__":
    main()