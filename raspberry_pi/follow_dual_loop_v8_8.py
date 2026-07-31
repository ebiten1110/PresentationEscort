"""
Presentation Escort 自動追従 V8.8.2

機能:
- 顔の左右位置に合わせてTRACK_LEFT / TRACK_RIGHT
- 顔の上下位置に合わせてHEAD_UP / HEAD_DOWN
- 顔検出中はRGB LED 3個を白色点灯
- 静止中の短い顔ロストは1.0秒保持
- 旋回・前進・後退中は白色を維持し、動作後に再検出を待つ
- Haar検出をCLAHE・ROI再検出・代替カスケードで補助
- Haarが外れた短時間はLucas-Kanade光学フローで位置追跡
- 顔の高さ比率でFAR / GOOD / NEARを判定
- FARならTRACK_FORWARDを1パルス
- NEARならTRACK_BACKWARDを1パルス
- GOODなら前後移動しない
- 前後移動は顔が左右中央付近にある場合だけ実行
- 1パルス終了ごとに距離を再判定

実行:
    python3 follow_dual_loop_v8_8_2.py
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
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

# V8.5では実機の左右方向を修正するためFalseへ固定。
# 既存config.pyにTrueが残っていても、この値を使用する。
INVERT_HORIZONTAL_TRACKING = False

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
# 二重ループ制御
# ============================================================

# 内側ループ：顔の画像誤差からカメラのパン・チルト角度を更新する。
HEAD_YAW_MIN_DEG = -20.0
HEAD_YAW_MAX_DEG = 20.0
HEAD_PITCH_MIN_DEG = -12.0
HEAD_PITCH_MAX_DEG = 12.0

HEAD_X_DEADZONE_PX = 14
HEAD_Y_DEADZONE_PX = 12
HEAD_YAW_GAIN_DEG_PER_PX = 0.055
HEAD_PITCH_GAIN_DEG_PER_PX = 0.045
HEAD_MAX_YAW_STEP_DEG = 2.5
HEAD_MAX_PITCH_STEP_DEG = 2.0
HEAD_POSE_COMMAND_INTERVAL_SECONDS = 0.055
HEAD_STATUS_INTERVAL_SECONDS = 0.15
HEAD_COMMAND_EPSILON_DEG = 0.25

# ============================================================
# 水平方向の符号
# ============================================================
#
# USBカメラはロボット側から人物を見ているため、
# 人物が自分から見て右へ移動すると、ロボット座標では左方向になる。
#
# V8.8では画像X誤差をそのままyawへ加算していたため、
# 人物が右へ移動したときにカメラも物理的な右へ向いていた。
#
# V8.8.2では画像Xからロボットyawへの符号を1か所へ統一し、
# カメラ内側ループと体外側ループの残差計算の両方に適用する。
#
# 実機設定:
#   画面右の顔  -> 負yaw
#   画面左の顔  -> 正yaw
#
# 既存config.pyのHEAD_YAW_IMAGE_DIRECTIONは使用しない。
# 方向を変更する場合は、新しい設定名だけを変更する。
CAMERA_X_TO_ROBOT_YAW_DIRECTION = float(
    getattr(
        cfg,
        "CAMERA_X_TO_ROBOT_YAW_DIRECTION",
        -1.0,
    )
)

HEAD_PITCH_IMAGE_DIRECTION = float(
    getattr(cfg, "HEAD_PITCH_IMAGE_DIRECTION", 1.0)
)

# 顔の残留画像誤差を概算角度へ変換するための水平画角。
CAMERA_HORIZONTAL_FOV_DEG = float(
    getattr(cfg, "CAMERA_HORIZONTAL_FOV_DEG", 60.0)
)

# 外側ループ：カメラの左右角度＋画像内残差から体の方向誤差を求める。
BODY_BEARING_START_DEG = 9.0
BODY_BEARING_CONTINUE_DEG = 4.5
BODY_BEARING_STOP_DEG = 3.0
BODY_TURN_CONFIRM_FRAMES = 2
BODY_TURN_CONTINUE_CONFIRM_FRAMES = 1
BODY_RESTART_COOLDOWN_SECONDS = 0.18
BODY_POST_TURN_LOCKOUT_SECONDS = 0.55
BODY_TURN_CONTINUE_WINDOW_SECONDS = 2.50

# 現在の実機では、負方向のカメラ角に対してTRACK_RIGHT、
# 正方向に対してTRACK_LEFTを送ると人物側へ旋回する。
BODY_NEGATIVE_BEARING_COMMAND = "TRACK_RIGHT"
BODY_POSITIVE_BEARING_COMMAND = "TRACK_LEFT"

# 前後移動はカメラと顔が正面へ戻った後だけ許可する。
DISTANCE_HEAD_YAW_MAX_DEG = 4.0
DISTANCE_BODY_BEARING_MAX_DEG = 4.0
DISTANCE_FACE_X_MAX_PX = 24
DISTANCE_ALIGNMENT_CONFIRM_FRAMES = 4
DISTANCE_MOVE_ABORT_BEARING_DEG = 9.0

# 上下は体を動かさず、内側ループのカメラチルトだけで追従する。

# V8.7互換の表示用定数。二重ループ本体では角度しきい値を使用する。
BODY_TURN_START_X = 82
BODY_STOP_X = 36
SWAP_TRACK_COMMANDS = True


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

# 7フレーム平均は安定する一方で反応が遅かったため、
# V8.6では5フレームへ戻す。
DISTANCE_SMOOTHING_FRAMES = 5

DISTANCE_CONFIRM_FRAMES = 2

DISTANCE_MOVEMENT_ENABLED = getattr(
    cfg,
    "DISTANCE_MOVEMENT_ENABLED",
    True,
)

# 前後移動を開始する条件はDISTANCE_START_MAX_Xで厳しくし、
# 開始後の中断条件は少し広くする（開始・中断のヒステリシス）。
DISTANCE_MOVE_MAX_X = 105

DISTANCE_RESTART_COOLDOWN_SECONDS = 0.30

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

# 静止中の短い検出落ちは保持する。
IDLE_FACE_LOST_GRACE_SECONDS = 1.20

# 旋回・前進・後退中は、モーションブラーや画角変化で検出が
# 一時的に切れやすいため、動作中は赤へ切り替えない。
# ただし安全のため、完全ロストが長時間続いた場合は停止する。
MOVING_FACE_LOST_HARD_TIMEOUT_SECONDS = 5.00
POST_MOTION_REACQUIRE_SECONDS = 1.50

# 長時間ロスト後に首を中央へ戻す。
HEAD_CENTER_AFTER_FACE_LOST = True

# 顔検出は320pxより少し高い解像度で行う。
DETECTION_WIDTH = 400

# Haar Cascadeの検出補助。
ALT_CASCADE_PATH = (
    "/usr/share/opencv4/haarcascades/"
    "haarcascade_frontalface_alt2.xml"
)
PROFILE_CASCADE_PATH = (
    "/usr/share/opencv4/haarcascades/"
    "haarcascade_profileface.xml"
)
PRIMARY_SCALE_FACTOR = 1.08
PRIMARY_MIN_NEIGHBORS = 4
FALLBACK_SCALE_FACTOR = 1.05
FALLBACK_MIN_NEIGHBORS = 3
DETECTION_MIN_SIZE = (24, 24)
ROI_EXPANSION = 1.80

# CLAHEは局所コントラストを補正する。
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID = (8, 8)

# Haarが一時的に外れたとき、顔領域内の特徴点を
# Lucas-Kanade法で短時間追跡する。
OPTICAL_FLOW_MAX_SECONDS = 1.20
OPTICAL_FLOW_MIN_POINTS = 5
OPTICAL_FLOW_MAX_SHIFT_PER_FRAME = 90.0
LOOP_INTERVAL_SECONDS = 0.02

RESULT_IMAGE_PATH = Path(
    "follow_dual_loop_latest.jpg"
)
# JPEG保存は処理時間を使うため頻度を下げる。
SAVE_IMAGE_EVERY_N_LOOPS = 20

SHOW_PREVIEW_WINDOW_REQUESTED = getattr(
    cfg,
    "SHOW_PREVIEW_WINDOW",
    False,
)
PREVIEW_WINDOW_NAME = "Presentation Escort V8.8.2"

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

    # 対応しているカメラでは取得間隔を短縮する。
    cap.set(
        cv2.CAP_PROP_FPS,
        30,
    )

    return cap



@dataclass
class FaceDetectors:
    primary: cv2.CascadeClassifier
    alternate: Optional[cv2.CascadeClassifier]
    profile: Optional[cv2.CascadeClassifier]
    clahe: cv2.CLAHE


def load_cascades() -> FaceDetectors:
    primary = cv2.CascadeClassifier(
        CASCADE_PATH
    )

    if primary.empty():
        raise RuntimeError(
            "Haar Cascadeを読み込めません: "
            f"{CASCADE_PATH}"
        )

    alternate: Optional[
        cv2.CascadeClassifier
    ] = None

    if Path(ALT_CASCADE_PATH).exists():
        candidate = cv2.CascadeClassifier(
            ALT_CASCADE_PATH
        )

        if not candidate.empty():
            alternate = candidate
            print(
                "[Detection] Alternate cascade loaded: "
                f"{ALT_CASCADE_PATH}"
            )
        else:
            print(
                "[Detection] WARNING: "
                "alternate cascade is empty."
            )
    else:
        print(
            "[Detection] WARNING: "
            "alternate cascade not found: "
            f"{ALT_CASCADE_PATH}"
        )

    profile: Optional[
        cv2.CascadeClassifier
    ] = None

    if Path(PROFILE_CASCADE_PATH).exists():
        candidate = cv2.CascadeClassifier(
            PROFILE_CASCADE_PATH
        )

        if not candidate.empty():
            profile = candidate
            print(
                "[Detection] Profile cascade loaded: "
                f"{PROFILE_CASCADE_PATH}"
            )
        else:
            print(
                "[Detection] WARNING: "
                "profile cascade is empty."
            )
    else:
        print(
            "[Detection] WARNING: "
            "profile cascade not found: "
            f"{PROFILE_CASCADE_PATH}"
        )

    clahe = cv2.createCLAHE(
        clipLimit=CLAHE_CLIP_LIMIT,
        tileGridSize=CLAHE_TILE_GRID,
    )

    return FaceDetectors(
        primary=primary,
        alternate=alternate,
        profile=profile,
        clahe=clahe,
    )


def build_face_result(
    x: int,
    y: int,
    width: int,
    height: int,
    frame_width: int,
    frame_height: int,
) -> FaceResult:
    width = max(
        1,
        min(width, frame_width),
    )
    height = max(
        1,
        min(height, frame_height),
    )

    x = max(
        0,
        min(x, frame_width - width),
    )
    y = max(
        0,
        min(y, frame_height - height),
    )

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

    return FaceResult(
        x=x,
        y=y,
        width=width,
        height=height,
        center_x=center_x,
        center_y=center_y,
        diff_x=center_x - target_center_x,
        diff_y=center_y - target_center_y,
    )


def _largest_detection(
    detections,
) -> Optional[Tuple[int, int, int, int]]:
    if len(detections) == 0:
        return None

    item = max(
        detections,
        key=lambda value: value[2] * value[3],
    )

    return tuple(int(value) for value in item)


def _detect_in_image(
    image,
    cascade: cv2.CascadeClassifier,
    *,
    scale_factor: float,
    min_neighbors: int,
):
    return cascade.detectMultiScale(
        image,
        scaleFactor=scale_factor,
        minNeighbors=min_neighbors,
        minSize=DETECTION_MIN_SIZE,
    )


def detect_main_face(
    frame,
    detectors: FaceDetectors,
    last_face: Optional[FaceResult],
) -> Tuple[Optional[FaceResult], list, str]:
    """
    検出順:
    1. 前回顔の周辺ROIを高感度で再検出
    2. CLAHE画像を標準設定で全体検出
    3. 元グレー画像を高感度で全体検出
    4. alternate cascadeで全体検出
    """

    frame_height, frame_width = frame.shape[:2]

    target_detection_width = min(
        DETECTION_WIDTH,
        frame_width,
    )

    scale = (
        target_detection_width
        / float(frame_width)
    )

    detection_height = max(
        1,
        int(frame_height * scale),
    )

    small = cv2.resize(
        frame,
        (
            target_detection_width,
            detection_height,
        ),
        interpolation=cv2.INTER_AREA,
    )

    gray_raw = cv2.cvtColor(
        small,
        cv2.COLOR_BGR2GRAY,
    )

    if ENABLE_HISTOGRAM_EQUALIZATION:
        gray_prepared = detectors.clahe.apply(
            gray_raw
        )
    else:
        gray_prepared = gray_raw

    inverse_scale = 1.0 / scale

    def convert_detection(
        detection: Tuple[int, int, int, int],
        offset_x: int = 0,
        offset_y: int = 0,
    ) -> FaceResult:
        sx, sy, sw, sh = detection

        x = int(
            (sx + offset_x)
            * inverse_scale
        )
        y = int(
            (sy + offset_y)
            * inverse_scale
        )
        width = int(sw * inverse_scale)
        height = int(sh * inverse_scale)

        return build_face_result(
            x,
            y,
            width,
            height,
            frame_width,
            frame_height,
        )

    # 前回位置の周辺を先に探す。
    if last_face is not None:
        prev_x = int(last_face.x * scale)
        prev_y = int(last_face.y * scale)
        prev_w = max(
            1,
            int(last_face.width * scale),
        )
        prev_h = max(
            1,
            int(last_face.height * scale),
        )

        expanded_w = int(
            prev_w * ROI_EXPANSION
        )
        expanded_h = int(
            prev_h * ROI_EXPANSION
        )

        roi_x = max(
            0,
            prev_x
            - (expanded_w - prev_w) // 2,
        )
        roi_y = max(
            0,
            prev_y
            - (expanded_h - prev_h) // 2,
        )
        roi_right = min(
            target_detection_width,
            roi_x + expanded_w,
        )
        roi_bottom = min(
            detection_height,
            roi_y + expanded_h,
        )

        if (
            roi_right - roi_x
            >= DETECTION_MIN_SIZE[0]
            and roi_bottom - roi_y
            >= DETECTION_MIN_SIZE[1]
        ):
            roi_image = gray_prepared[
                roi_y:roi_bottom,
                roi_x:roi_right,
            ]

            roi_faces = _detect_in_image(
                roi_image,
                detectors.primary,
                scale_factor=FALLBACK_SCALE_FACTOR,
                min_neighbors=FALLBACK_MIN_NEIGHBORS,
            )

            detection = _largest_detection(
                roi_faces
            )

            if detection is not None:
                face = convert_detection(
                    detection,
                    roi_x,
                    roi_y,
                )
                return (
                    face,
                    [
                        (
                            face.x,
                            face.y,
                            face.width,
                            face.height,
                        )
                    ],
                    "DETECT_ROI",
                )

    primary_faces = _detect_in_image(
        gray_prepared,
        detectors.primary,
        scale_factor=PRIMARY_SCALE_FACTOR,
        min_neighbors=PRIMARY_MIN_NEIGHBORS,
    )

    detection = _largest_detection(
        primary_faces
    )

    if detection is not None:
        face = convert_detection(detection)

        return (
            face,
            [
                (
                    face.x,
                    face.y,
                    face.width,
                    face.height,
                )
            ],
            "DETECT_PRIMARY",
        )

    # CLAHEが合わない照明条件向けに、元グレー画像でも試す。
    raw_faces = _detect_in_image(
        gray_raw,
        detectors.primary,
        scale_factor=FALLBACK_SCALE_FACTOR,
        min_neighbors=FALLBACK_MIN_NEIGHBORS,
    )

    detection = _largest_detection(raw_faces)

    if detection is not None:
        face = convert_detection(detection)

        return (
            face,
            [
                (
                    face.x,
                    face.y,
                    face.width,
                    face.height,
                )
            ],
            "DETECT_RAW",
        )

    if detectors.alternate is not None:
        alternate_faces = _detect_in_image(
            gray_prepared,
            detectors.alternate,
            scale_factor=FALLBACK_SCALE_FACTOR,
            min_neighbors=FALLBACK_MIN_NEIGHBORS,
        )

        detection = _largest_detection(
            alternate_faces
        )

        if detection is not None:
            face = convert_detection(
                detection
            )

            return (
                face,
                [
                    (
                        face.x,
                        face.y,
                        face.width,
                        face.height,
                    )
                ],
                "DETECT_ALT",
            )

    if detectors.profile is not None:
        profile_faces = _detect_in_image(
            gray_prepared,
            detectors.profile,
            scale_factor=FALLBACK_SCALE_FACTOR,
            min_neighbors=FALLBACK_MIN_NEIGHBORS,
        )

        detection = _largest_detection(
            profile_faces
        )

        if detection is not None:
            face = convert_detection(
                detection
            )

            return (
                face,
                [
                    (
                        face.x,
                        face.y,
                        face.width,
                        face.height,
                    )
                ],
                "DETECT_PROFILE",
            )

        flipped = cv2.flip(
            gray_prepared,
            1,
        )

        flipped_faces = _detect_in_image(
            flipped,
            detectors.profile,
            scale_factor=FALLBACK_SCALE_FACTOR,
            min_neighbors=FALLBACK_MIN_NEIGHBORS,
        )

        detection = _largest_detection(
            flipped_faces
        )

        if detection is not None:
            sx, sy, sw, sh = detection
            remapped_x = (
                target_detection_width
                - (sx + sw)
            )

            face = convert_detection(
                (
                    remapped_x,
                    sy,
                    sw,
                    sh,
                )
            )

            return (
                face,
                [
                    (
                        face.x,
                        face.y,
                        face.width,
                        face.height,
                    )
                ],
                "DETECT_PROFILE_FLIPPED",
            )

    return None, [], "NONE"


class OpticalFlowFaceTracker:
    """
    Haar検出が一時的に外れたとき、
    顔領域内の特徴点をLucas-Kanade法で追跡する。
    """

    def __init__(self) -> None:
        self.previous_gray = None
        self.points = None
        self.face: Optional[FaceResult] = None
        self.last_detector_time: Optional[
            float
        ] = None

    def reset(self) -> None:
        self.previous_gray = None
        self.points = None
        self.face = None
        self.last_detector_time = None

    def _find_points(
        self,
        gray,
        face: FaceResult,
    ):
        mask = np.zeros_like(gray)

        margin_x = max(
            2,
            int(face.width * 0.08),
        )
        margin_y = max(
            2,
            int(face.height * 0.08),
        )

        left = max(0, face.x + margin_x)
        top = max(0, face.y + margin_y)
        right = min(
            gray.shape[1],
            face.x + face.width - margin_x,
        )
        bottom = min(
            gray.shape[0],
            face.y + face.height - margin_y,
        )

        if right <= left or bottom <= top:
            return None

        mask[top:bottom, left:right] = 255

        return cv2.goodFeaturesToTrack(
            gray,
            mask=mask,
            maxCorners=45,
            qualityLevel=0.01,
            minDistance=4,
            blockSize=7,
        )

    def update(
        self,
        gray,
        detected_face: Optional[FaceResult],
        detector_source: str,
        now: float,
    ) -> Tuple[Optional[FaceResult], str]:
        if detected_face is not None:
            self.face = detected_face
            self.last_detector_time = now
            self.points = self._find_points(
                gray,
                detected_face,
            )
            self.previous_gray = gray.copy()

            return (
                detected_face,
                detector_source,
            )

        if (
            self.last_detector_time is None
            or now - self.last_detector_time
            > OPTICAL_FLOW_MAX_SECONDS
            or self.previous_gray is None
            or self.points is None
            or self.face is None
        ):
            self.previous_gray = gray.copy()
            return None, "NONE"

        next_points, status, errors = (
            cv2.calcOpticalFlowPyrLK(
                self.previous_gray,
                gray,
                self.points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(
                    cv2.TERM_CRITERIA_EPS
                    | cv2.TERM_CRITERIA_COUNT,
                    30,
                    0.01,
                ),
            )
        )

        if (
            next_points is None
            or status is None
        ):
            self.previous_gray = gray.copy()
            self.points = None
            return None, "NONE"

        valid = status.reshape(-1) == 1

        if errors is not None:
            error_values = errors.reshape(-1)
            valid = valid & (error_values < 45.0)

        old_good = self.points.reshape(
            -1,
            2,
        )[valid]
        new_good = next_points.reshape(
            -1,
            2,
        )[valid]

        if (
            len(new_good)
            < OPTICAL_FLOW_MIN_POINTS
        ):
            self.previous_gray = gray.copy()
            self.points = None
            return None, "NONE"

        deltas = new_good - old_good
        median_delta = np.median(
            deltas,
            axis=0,
        )

        dx = float(median_delta[0])
        dy = float(median_delta[1])

        shift_magnitude = float(
            np.hypot(dx, dy)
        )

        if (
            not np.isfinite(shift_magnitude)
            or shift_magnitude
            > OPTICAL_FLOW_MAX_SHIFT_PER_FRAME
        ):
            self.previous_gray = gray.copy()
            self.points = None
            return None, "NONE"

        tracked = build_face_result(
            int(round(self.face.x + dx)),
            int(round(self.face.y + dy)),
            self.face.width,
            self.face.height,
            gray.shape[1],
            gray.shape[0],
        )

        self.face = tracked
        self.points = new_good.reshape(
            -1,
            1,
            2,
        ).astype(np.float32)
        self.previous_gray = gray.copy()

        if len(new_good) < 10:
            refreshed = self._find_points(
                gray,
                tracked,
            )

            if refreshed is not None:
                self.points = refreshed

        return tracked, "FLOW"


def get_horizontal_control_error(
    raw_diff_x: int,
) -> int:
    if INVERT_HORIZONTAL_TRACKING:
        return -raw_diff_x

    return raw_diff_x


def get_track_command_for_error(
    control_x: int,
    threshold: int = BODY_TURN_START_X,
) -> Optional[str]:
    """
    顔の左右位置から、実際に送る旋回命令を決める。

    control_x < 0:
        顔が画面左側。

    control_x > 0:
        顔が画面右側。

    実機ではTRACK_LEFT / TRACK_RIGHTの物理旋回方向が
    命令名と逆だったため、SWAP_TRACK_COMMANDS=Trueで
    最終的な送信命令だけを入れ替える。
    """

    if control_x <= -threshold:
        logical_command = "TRACK_LEFT"

    elif control_x >= threshold:
        logical_command = "TRACK_RIGHT"

    else:
        return None

    if not SWAP_TRACK_COMMANDS:
        return logical_command

    if logical_command == "TRACK_LEFT":
        return "TRACK_RIGHT"

    return "TRACK_LEFT"


def is_reliable_distance_source(
    face_source: str,
) -> bool:
    return face_source in {
        "DETECT_ROI",
        "DETECT_PRIMARY",
        "DETECT_RAW",
        "DETECT_ALT",
    }


def get_face_side_label(
    control_x: int,
) -> str:
    if control_x <= -BODY_TURN_START_X:
        return "FACE_LEFT"

    if control_x >= BODY_TURN_START_X:
        return "FACE_RIGHT"

    if abs(control_x) <= BODY_STOP_X:
        return "CENTER"

    return "HOLD"


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
        "[HeadState]",
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
        self.last_completed_body: Optional[str] = None
        self.last_completed_time = 0.0
        self.fault = False

        self.head_current_yaw = 0.0
        self.head_target_yaw = 0.0
        self.head_current_pitch = 0.0
        self.head_target_pitch = 0.0
        self.head_state_time = 0.0

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

        self.send("RGB_OFF")
        self.send("HEAD_CENTER")
        self.send("HEAD_STATUS")
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

    def send(
        self,
        command: str,
        quiet: bool = False,
    ) -> bool:
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

        if not quiet:
            print(f"[Serial] Sent: {command}")
        return True

    def send_head_pose(
        self,
        yaw_deg: float,
        pitch_deg: float,
    ) -> bool:
        command = (
            f"HEAD_POSE_SET:{yaw_deg:.1f},"
            f"{pitch_deg:.1f}"
        )
        return self.send(command, quiet=True)

    def request_head_status(self) -> bool:
        return self.send("HEAD_STATUS", quiet=True)

    def get_head_yaw_estimate(
        self,
        fallback_target: float,
    ) -> float:
        if time.monotonic() - self.head_state_time <= 0.45:
            return self.head_current_yaw
        return fallback_target

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

        head_match = re.search(
            r"\[HeadState\]\s+"
            r"currentYaw=([-+]?\d+(?:\.\d+)?)\s+"
            r"targetYaw=([-+]?\d+(?:\.\d+)?)\s+"
            r"currentPitch=([-+]?\d+(?:\.\d+)?)\s+"
            r"targetPitch=([-+]?\d+(?:\.\d+)?)",
            line,
        )

        if head_match is not None:
            self.head_current_yaw = float(
                head_match.group(1)
            )
            self.head_target_yaw = float(
                head_match.group(2)
            )
            self.head_current_pitch = float(
                head_match.group(3)
            )
            self.head_target_pitch = float(
                head_match.group(4)
            )
            self.head_state_time = time.monotonic()

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
                completed_body = self.active_body

                if completed_body is not None:
                    self.last_completed_body = (
                        completed_body
                    )
                    self.last_completed_time = (
                        time.monotonic()
                    )

                    print(
                        "[Body] COMPLETE "
                        f"{completed_body}"
                    )

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


class RGBStatusController:
    """
    同じ色コマンドを毎フレーム再送しないための制御。
    """

    def __init__(self) -> None:
        self.current_command: Optional[str] = None

    def set(
        self,
        esp32: ESP32Serial,
        command: str,
    ) -> bool:
        if command == self.current_command:
            return False

        if not esp32.send(command):
            return False

        self.current_command = command

        print(
            f"[RGB] Mode={command}"
        )
        return True


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



class HeadPoseController:
    """顔の画像誤差をカメラの絶対パン・チルト角度へ変換する。"""

    def __init__(self) -> None:
        self.yaw_target = 0.0
        self.pitch_target = 0.0
        self.last_sent_yaw: Optional[float] = None
        self.last_sent_pitch: Optional[float] = None
        self.last_pose_send_time = 0.0
        self.last_status_request_time = 0.0
        self.last_log_time = 0.0

    @staticmethod
    def _clamp(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _step_from_error(
        error_px: int,
        deadzone_px: int,
        gain: float,
        max_step: float,
    ) -> float:
        if abs(error_px) <= deadzone_px:
            return 0.0

        effective_error = (
            error_px - deadzone_px
            if error_px > 0
            else error_px + deadzone_px
        )

        return max(
            -max_step,
            min(max_step, effective_error * gain),
        )

    def update_from_face(
        self,
        face: FaceResult,
        esp32: ESP32Serial,
        now: float,
    ) -> bool:
        yaw_step = self._step_from_error(
            face.diff_x,
            HEAD_X_DEADZONE_PX,
            HEAD_YAW_GAIN_DEG_PER_PX,
            HEAD_MAX_YAW_STEP_DEG,
        ) * CAMERA_X_TO_ROBOT_YAW_DIRECTION

        pitch_step = self._step_from_error(
            face.diff_y,
            HEAD_Y_DEADZONE_PX,
            HEAD_PITCH_GAIN_DEG_PER_PX,
            HEAD_MAX_PITCH_STEP_DEG,
        ) * HEAD_PITCH_IMAGE_DIRECTION

        self.yaw_target = self._clamp(
            self.yaw_target + yaw_step,
            HEAD_YAW_MIN_DEG,
            HEAD_YAW_MAX_DEG,
        )
        self.pitch_target = self._clamp(
            self.pitch_target + pitch_step,
            HEAD_PITCH_MIN_DEG,
            HEAD_PITCH_MAX_DEG,
        )

        return self.send_if_due(esp32, now)

    def send_if_due(
        self,
        esp32: ESP32Serial,
        now: float,
        force: bool = False,
    ) -> bool:
        changed = (
            self.last_sent_yaw is None
            or self.last_sent_pitch is None
            or abs(self.yaw_target - self.last_sent_yaw)
            >= HEAD_COMMAND_EPSILON_DEG
            or abs(self.pitch_target - self.last_sent_pitch)
            >= HEAD_COMMAND_EPSILON_DEG
        )

        if not changed and not force:
            return False

        if (
            not force
            and now - self.last_pose_send_time
            < HEAD_POSE_COMMAND_INTERVAL_SECONDS
        ):
            return False

        if not esp32.send_head_pose(
            self.yaw_target,
            self.pitch_target,
        ):
            return False

        self.last_sent_yaw = self.yaw_target
        self.last_sent_pitch = self.pitch_target
        self.last_pose_send_time = now

        if now - self.last_log_time >= 0.25:
            print(
                "[HeadInnerLoop] "
                f"targetYaw={self.yaw_target:.1f} "
                f"targetPitch={self.pitch_target:.1f} "
                f"xToYawSign="
                f"{CAMERA_X_TO_ROBOT_YAW_DIRECTION:+.1f}"
            )
            self.last_log_time = now
        return True

    def request_status_if_due(
        self,
        esp32: ESP32Serial,
        now: float,
    ) -> None:
        if (
            now - self.last_status_request_time
            < HEAD_STATUS_INTERVAL_SECONDS
        ):
            return

        if esp32.request_head_status():
            self.last_status_request_time = now

    def center(
        self,
        esp32: ESP32Serial,
        now: float,
    ) -> None:
        self.yaw_target = 0.0
        self.pitch_target = 0.0
        self.send_if_due(
            esp32,
            now,
            force=True,
        )


def residual_x_to_degrees(
    diff_x: int,
    frame_width: int,
) -> float:
    if frame_width <= 0:
        return 0.0

    return (
        float(diff_x)
        / float(frame_width)
        * CAMERA_HORIZONTAL_FOV_DEG
    )


def body_command_from_bearing(
    bearing_deg: float,
) -> Optional[str]:
    if bearing_deg < 0.0:
        return BODY_NEGATIVE_BEARING_COMMAND

    if bearing_deg > 0.0:
        return BODY_POSITIVE_BEARING_COMMAND

    return None

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
    rgb_mode: str,
    pulse_count: int,
    face_source: str,
    camera_yaw: float,
    camera_pitch: float,
    body_bearing: float,
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
        f"FaceSource={face_source}",
        (
            f"CamYaw={camera_yaw:.1f} "
            f"CamPitch={camera_pitch:.1f} "
            f"Bearing={body_bearing:.1f}"
        ),
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
            f"RGB={rgb_mode} "
            f"Move={DISTANCE_MOVEMENT_ENABLED}"
        ),
    )

    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (10, 55 + index * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            2,
        )


def main() -> None:
    print("==============================================")
    print("Presentation Escort Auto Follow V8.8.2")
    print("Dual-loop camera/body visual servo")
    print("==============================================")

    print(
        "[HeadInnerLoopConfig] "
        f"X_DEADZONE={HEAD_X_DEADZONE_PX}px "
        f"Y_DEADZONE={HEAD_Y_DEADZONE_PX}px "
        f"YAW_RANGE={HEAD_YAW_MIN_DEG:.0f}..{HEAD_YAW_MAX_DEG:.0f} "
        f"PITCH_RANGE={HEAD_PITCH_MIN_DEG:.0f}..{HEAD_PITCH_MAX_DEG:.0f} "
        f"X_TO_YAW_SIGN={CAMERA_X_TO_ROBOT_YAW_DIRECTION:+.1f}"
    )
    print(
        "[BodyOuterLoopConfig] "
        f"START={BODY_BEARING_START_DEG:.1f}deg "
        f"CONTINUE={BODY_BEARING_CONTINUE_DEG:.1f}deg "
        f"STOP={BODY_BEARING_STOP_DEG:.1f}deg "
        f"NEG={BODY_NEGATIVE_BEARING_COMMAND} "
        f"POS={BODY_POSITIVE_BEARING_COMMAND}"
    )
    print(
        "[DistanceGate] "
        f"HEAD_YAW<={DISTANCE_HEAD_YAW_MAX_DEG:.1f}deg "
        f"BEARING<={DISTANCE_BODY_BEARING_MAX_DEG:.1f}deg "
        f"FACE_X<={DISTANCE_FACE_X_MAX_PX}px"
    )

    preview_enabled = can_use_preview_window()
    detectors = load_cascades()
    cap = open_camera()

    if not cap.isOpened():
        raise RuntimeError("カメラを開けませんでした。")

    for _ in range(6):
        cap.read()

    esp32 = ESP32Serial()
    rgb_status = RGBStatusController()
    head_controller = HeadPoseController()
    distance_estimator = DistanceEstimator()
    face_tracker = OpticalFlowFaceTracker()

    rgb_mode = "RGB_OFF"
    last_face: Optional[FaceResult] = None
    last_face_seen_time: Optional[float] = None
    has_detected_face = False
    long_loss_handled = False

    body_candidate: Optional[str] = None
    body_candidate_count = 0
    distance_candidate: Optional[str] = None
    distance_candidate_count = 0
    distance_alignment_count = 0
    consecutive_distance_pulses = 0
    last_distance_command: Optional[str] = None

    last_distance_state = "UNKNOWN"
    last_distance_ratio: Optional[float] = None
    last_distance_action = "NONE"
    last_body_bearing = 0.0
    last_camera_yaw = 0.0
    last_camera_pitch = 0.0
    loop_count = 0

    try:
        esp32.open()
        head_controller.center(esp32, time.monotonic())

        if rgb_status.set(esp32, "RGB_RED"):
            rgb_mode = "RGB_RED"

        while True:
            loop_started = time.monotonic()
            now = loop_started
            loop_count += 1

            esp32.poll()
            head_controller.request_status_if_due(
                esp32,
                now,
            )

            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("カメラ画像を取得できません。")

            if MIRROR_CAMERA_IMAGE:
                frame = cv2.flip(frame, 1)

            detected_face, _, detector_source = detect_main_face(
                frame,
                detectors,
                last_face,
            )
            gray_full = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY,
            )
            live_face, face_source = face_tracker.update(
                gray_full,
                detected_face,
                detector_source,
                now,
            )

            display_face = live_face
            horizontal_state = "NO_FACE"
            vertical_state = "NO_FACE"
            distance_state = last_distance_state
            distance_ratio = last_distance_ratio
            distance_action = last_distance_action

            camera_yaw = esp32.get_head_yaw_estimate(
                head_controller.yaw_target
            )
            camera_pitch = (
                esp32.head_current_pitch
                if time.monotonic() - esp32.head_state_time <= 0.45
                else head_controller.pitch_target
            )
            body_bearing = last_body_bearing

            if live_face is not None:
                face = live_face
                last_face = face
                last_face_seen_time = now
                has_detected_face = True
                long_loss_handled = False

                if rgb_status.set(esp32, "RGB_WHITE"):
                    rgb_mode = "RGB_WHITE"

                # 内側ループ：体の状態に関係なく、カメラで顔を中央へ追う。
                head_controller.update_from_face(
                    face,
                    esp32,
                    now,
                )

                camera_yaw = esp32.get_head_yaw_estimate(
                    head_controller.yaw_target
                )
                camera_pitch = (
                    esp32.head_current_pitch
                    if time.monotonic() - esp32.head_state_time <= 0.45
                    else head_controller.pitch_target
                )

                # カメラyawと画像残差を同じロボット座標系へそろえる。
                # ここへ同じ符号を適用しないと、カメラ角と残差が
                # 打ち消し合い、体が誤方向へ旋回する。
                residual_deg = (
                    residual_x_to_degrees(
                        face.diff_x,
                        frame.shape[1],
                    )
                    * CAMERA_X_TO_ROBOT_YAW_DIRECTION
                )
                body_bearing = camera_yaw + residual_deg

                last_camera_yaw = camera_yaw
                last_camera_pitch = camera_pitch
                last_body_bearing = body_bearing

                horizontal_state = (
                    "BODY_LEFT"
                    if body_bearing < -BODY_BEARING_STOP_DEG
                    else "BODY_RIGHT"
                    if body_bearing > BODY_BEARING_STOP_DEG
                    else "BODY_ALIGNED"
                )
                vertical_state = (
                    "CAMERA_UP"
                    if face.diff_y < -HEAD_Y_DEADZONE_PX
                    else "CAMERA_DOWN"
                    if face.diff_y > HEAD_Y_DEADZONE_PX
                    else "CAMERA_CENTER"
                )

                turning_active = esp32.active_body in {
                    "TRACK_LEFT",
                    "TRACK_RIGHT",
                }
                distance_active = esp32.active_body in {
                    "TRACK_FORWARD",
                    "TRACK_BACKWARD",
                }

                # 体の旋回中は距離値を更新しない。
                aligned_for_distance = (
                    abs(camera_yaw) <= DISTANCE_HEAD_YAW_MAX_DEG
                    and abs(body_bearing) <= DISTANCE_BODY_BEARING_MAX_DEG
                    and abs(face.diff_x) <= DISTANCE_FACE_X_MAX_PX
                    and is_reliable_distance_source(face_source)
                )

                if aligned_for_distance and not turning_active:
                    (
                        distance_state,
                        distance_ratio,
                        distance_changed,
                    ) = distance_estimator.update(
                        face.height,
                        frame.shape[0],
                    )
                else:
                    distance_changed = False
                    distance_state = distance_estimator.state
                    distance_ratio = distance_estimator.ratio

                if distance_changed:
                    print(
                        "[Distance] "
                        f"State={distance_state} "
                        f"Ratio={distance_ratio:.3f}"
                    )

                if distance_state == "GOOD":
                    consecutive_distance_pulses = 0
                    last_distance_command = None

                # 外側ループ：現在の1パルス中はカメラ追従だけを継続。
                if turning_active:
                    distance_action = "BODY_TURNING_CAMERA_TRACK"
                    body_candidate = None
                    body_candidate_count = 0
                    distance_candidate = None
                    distance_candidate_count = 0
                    distance_alignment_count = 0

                elif distance_active:
                    distance_action = "DISTANCE_MOVING_CAMERA_TRACK"
                    body_candidate = None
                    body_candidate_count = 0
                    distance_alignment_count = 0

                    if (
                        abs(body_bearing)
                        > DISTANCE_MOVE_ABORT_BEARING_DEG
                    ):
                        esp32.send_stop(
                            "CAMERA_YAW_REQUIRES_REALIGN "
                            f"bearing={body_bearing:.1f}"
                        )

                elif esp32.busy:
                    distance_action = "WAIT_BODY_IDLE"
                    distance_alignment_count = 0

                else:
                    recent_turn = (
                        esp32.last_completed_body
                        in {"TRACK_LEFT", "TRACK_RIGHT"}
                        and now - esp32.last_completed_time
                        <= BODY_TURN_CONTINUE_WINDOW_SECONDS
                    )
                    turn_threshold = (
                        BODY_BEARING_CONTINUE_DEG
                        if recent_turn
                        else BODY_BEARING_START_DEG
                    )
                    required_frames = (
                        BODY_TURN_CONTINUE_CONFIRM_FRAMES
                        if recent_turn
                        else BODY_TURN_CONFIRM_FRAMES
                    )

                    desired_body_command = None
                    if abs(body_bearing) >= turn_threshold:
                        desired_body_command = body_command_from_bearing(
                            body_bearing
                        )

                    if desired_body_command is not None:
                        distance_action = "CAMERA_YAW_BODY_TURN"
                        distance_candidate = None
                        distance_candidate_count = 0
                        distance_alignment_count = 0

                        if body_candidate == desired_body_command:
                            body_candidate_count += 1
                        else:
                            body_candidate = desired_body_command
                            body_candidate_count = 1

                        if (
                            body_candidate_count >= required_frames
                            and esp32.ready_for_body_command(
                                BODY_RESTART_COOLDOWN_SECONDS
                            )
                        ):
                            print(
                                "[BodyOuterLoop] "
                                f"cameraYaw={camera_yaw:.1f} "
                                f"residual={residual_deg:.1f} "
                                f"bearing={body_bearing:.1f} "
                                f"send={desired_body_command}"
                            )
                            if esp32.send_body(desired_body_command):
                                body_candidate = None
                                body_candidate_count = 0

                    else:
                        body_candidate = None
                        body_candidate_count = 0

                        turn_lockout = (
                            esp32.last_completed_body
                            in {"TRACK_LEFT", "TRACK_RIGHT"}
                            and now - esp32.last_completed_time
                            < BODY_POST_TURN_LOCKOUT_SECONDS
                        )

                        if aligned_for_distance:
                            distance_alignment_count += 1
                        else:
                            distance_alignment_count = 0

                        requested_distance = map_distance_command(
                            distance_state
                        )
                        can_move_distance = (
                            DISTANCE_MOVEMENT_ENABLED
                            and requested_distance is not None
                            and aligned_for_distance
                            and not turn_lockout
                            and distance_alignment_count
                            >= DISTANCE_ALIGNMENT_CONFIRM_FRAMES
                        )

                        if turn_lockout:
                            distance_action = "POST_TURN_CAMERA_SETTLE"
                            distance_candidate = None
                            distance_candidate_count = 0
                        elif not aligned_for_distance:
                            distance_action = "WAIT_CAMERA_FORWARD"
                            distance_candidate = None
                            distance_candidate_count = 0
                        elif can_move_distance:
                            if requested_distance != last_distance_command:
                                consecutive_distance_pulses = 0
                                last_distance_command = requested_distance

                            if distance_candidate == requested_distance:
                                distance_candidate_count += 1
                            else:
                                distance_candidate = requested_distance
                                distance_candidate_count = 1

                            if (
                                distance_candidate_count
                                >= DISTANCE_CONFIRM_FRAMES
                                and consecutive_distance_pulses
                                < MAX_CONSECUTIVE_DISTANCE_PULSES
                                and esp32.ready_for_body_command(
                                    DISTANCE_RESTART_COOLDOWN_SECONDS
                                )
                            ):
                                if esp32.send_body(requested_distance):
                                    consecutive_distance_pulses += 1
                                    distance_candidate = None
                                    distance_candidate_count = 0
                                    distance_alignment_count = 0
                        else:
                            distance_action = distance_action_label(
                                distance_state
                            )
                            distance_candidate = None
                            distance_candidate_count = 0

                last_distance_state = distance_state
                last_distance_ratio = distance_ratio
                last_distance_action = distance_action

                print(
                    f"[Loop {loop_count:04d}] "
                    f"Face={face_source} "
                    f"diffX={face.diff_x} diffY={face.diff_y} "
                    f"camYaw={camera_yaw:.1f} "
                    f"camTarget={head_controller.yaw_target:.1f} "
                    f"residual={residual_deg:.1f} "
                    f"bearing={body_bearing:.1f} "
                    f"Distance={distance_state} "
                    f"Action={distance_action} "
                    f"active={esp32.active_body} "
                    f"RGB={rgb_mode}"
                )

            else:
                if last_face_seen_time is None:
                    face_lost_seconds = float("inf")
                else:
                    face_lost_seconds = now - last_face_seen_time

                motion_active = (
                    esp32.active_body is not None
                    or esp32.busy
                )
                post_motion_protected = (
                    esp32.last_completed_body is not None
                    and now - esp32.last_completed_time
                    <= POST_MOTION_REACQUIRE_SECONDS
                )
                hard_timeout = (
                    motion_active
                    and face_lost_seconds
                    > MOVING_FACE_LOST_HARD_TIMEOUT_SECONDS
                )
                grace_active = (
                    has_detected_face
                    and not hard_timeout
                    and (
                        face_lost_seconds
                        <= IDLE_FACE_LOST_GRACE_SECONDS
                        or motion_active
                        or post_motion_protected
                    )
                )

                camera_yaw = esp32.get_head_yaw_estimate(
                    head_controller.yaw_target
                )
                camera_pitch = (
                    esp32.head_current_pitch
                    if time.monotonic() - esp32.head_state_time <= 0.45
                    else head_controller.pitch_target
                )
                body_bearing = last_body_bearing

                if grace_active:
                    face_source = (
                        "MOTION_GRACE"
                        if motion_active or post_motion_protected
                        else "GRACE"
                    )
                    display_face = last_face
                    horizontal_state = "HOLD_CAMERA_POSE"
                    vertical_state = "HOLD_CAMERA_POSE"
                    distance_action = "NO_NEW_COMMAND_REACQUIRE"
                    body_candidate = None
                    body_candidate_count = 0
                    distance_candidate = None
                    distance_candidate_count = 0
                    distance_alignment_count = 0

                    if rgb_status.set(esp32, "RGB_WHITE"):
                        rgb_mode = "RGB_WHITE"

                    print(
                        f"[Loop {loop_count:04d}] "
                        f"Face={face_source} "
                        f"lost={face_lost_seconds:.3f}s "
                        f"camYaw={camera_yaw:.1f} "
                        f"motion={motion_active} "
                        f"RGB={rgb_mode}"
                    )
                else:
                    face_source = "LOST"
                    display_face = None
                    horizontal_state = "NO_FACE"
                    vertical_state = "NO_FACE"
                    distance_action = "STOP_AND_RED"

                    if rgb_status.set(esp32, "RGB_RED"):
                        rgb_mode = "RGB_RED"

                    esp32.send_stop(
                        f"FACE_LOST_TIMEOUT {face_lost_seconds:.3f}s"
                    )

                    if not long_loss_handled:
                        body_candidate = None
                        body_candidate_count = 0
                        distance_candidate = None
                        distance_candidate_count = 0
                        distance_alignment_count = 0
                        consecutive_distance_pulses = 0
                        last_distance_command = None
                        distance_estimator.reset()
                        face_tracker.reset()
                        last_face = None
                        last_face_seen_time = None
                        has_detected_face = False
                        head_controller.center(esp32, now)
                        long_loss_handled = True
                        print(
                            "[FaceLost] Tracking reset and head centered."
                        )

                    print(
                        f"[Loop {loop_count:04d}] "
                        f"Face=LOST lost={face_lost_seconds:.3f}s "
                        f"Action=STOP_AND_RED RGB={rgb_mode}"
                    )

            if loop_count % SAVE_IMAGE_EVERY_N_LOOPS == 0:
                output_frame = frame.copy()
                draw_result(
                    output_frame,
                    display_face,
                    esp32,
                    horizontal_state,
                    vertical_state,
                    distance_state,
                    distance_ratio,
                    distance_action,
                    rgb_mode,
                    consecutive_distance_pulses,
                    face_source,
                    camera_yaw,
                    camera_pitch,
                    body_bearing,
                )
                cv2.imwrite(str(RESULT_IMAGE_PATH), output_frame)

            if preview_enabled:
                preview_frame = frame.copy()
                draw_result(
                    preview_frame,
                    display_face,
                    esp32,
                    horizontal_state,
                    vertical_state,
                    distance_state,
                    distance_ratio,
                    distance_action,
                    rgb_mode,
                    consecutive_distance_pulses,
                    face_source,
                    camera_yaw,
                    camera_pitch,
                    body_bearing,
                )
                cv2.imshow(PREVIEW_WINDOW_NAME, preview_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            if esp32.fault:
                rgb_status.set(esp32, "RGB_RED")
                print("[Safety] ESP32 fault detected.")
                break

            elapsed = time.monotonic() - loop_started
            sleep_time = LOOP_INTERVAL_SECONDS - elapsed
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
            esp32.send("RGB_OFF")
        except Exception as error:
            print(f"[Safety] 終了処理失敗: {error}")

        esp32.close()
        cap.release()
        if preview_enabled:
            cv2.destroyAllWindows()
        print("[Main] Finished.")


if __name__ == "__main__":
    main()