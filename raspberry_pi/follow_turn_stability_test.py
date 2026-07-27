"""
Presentation Escort
Raspberry Pi 連続旋回・安定性確認プログラム

目的:
- ESP32へLEFT / RIGHTを一度だけ送る
- ESP32の [Walking] State: IDLE を確認するまで次を送らない
- 旋回終了後に姿勢安定時間を取る
- 新しいカメラ画像で再判定してから次の旋回を行う
- この段階ではFORWARDと頭サーボを動かさない

実行:
    python3 follow_turn_stability_test.py

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
# 既存設定の読み込み
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

CAMERA_CENTER_OFFSET_X = getattr(
    cfg,
    "CAMERA_CENTER_OFFSET_X",
    0,
)


# ============================================================
# このテスト専用の安全設定
# ============================================================

# 顔が中央からこの値以上ずれたら旋回候補にする。
# 既存の35pxよりかなり広くし、小さな揺れでは旋回しない。
TURN_ENTER_X = 110

# 一度LEFT/RIGHTへ入った後、この範囲まで中央へ戻れば解除。
TURN_EXIT_X = 55

# 同じ方向が連続したフレーム数だけ続いたら確定。
TURN_CONFIRM_FRAMES = 4

# 旋回終了後、機体と輪ゴムが落ち着くまで待つ。
POST_TURN_SETTLE_SECONDS = 1.20

# 旋回終了後、新しい顔位置をこの回数確認してから次を許可。
POST_TURN_RECHECK_FRAMES = 5

# 顔を見失った場合に判定をリセットする回数。
NO_FACE_RESET_FRAMES = 3

# IDLEが返らない場合の安全タイムアウト。
# タイムアウト後は自動旋回を停止し、再起動を要求する。
MOTION_TIMEOUT_SECONDS = 15.0

# 中央を一度も確認せずに連続できる最大旋回回数。
MAX_TURNS_WITHOUT_CENTER = 3

# 画像処理周期。
LOOP_INTERVAL_SECONDS = 0.15

# 確認画像。
RESULT_IMAGE_PATH = Path(
    "turn_stability_latest.jpg"
)
SAVE_IMAGE_EVERY_N_LOOPS = 5

# ESP32の全ログを表示するとサーボ角度ログが大量に出る。
# Falseでは重要行だけ表示する。
PRINT_ALL_ESP32_LINES = False


# ============================================================
# データ構造
# ============================================================

@dataclass
class FaceResult:
    center_x: int
    center_y: int
    width: int
    height: int
    diff_x: int


# ============================================================
# 顔検出
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


def load_face_cascade() -> cv2.CascadeClassifier:
    cascade = cv2.CascadeClassifier(CASCADE_PATH)

    if cascade.empty():
        raise RuntimeError(
            "Haar Cascadeを読み込めません: "
            f"{CASCADE_PATH}"
        )

    print(
        f"[FaceDetect] Cascade={CASCADE_PATH}"
    )
    return cascade


def detect_main_face(
    frame,
    cascade: cv2.CascadeClassifier,
) -> Tuple[Optional[FaceResult], tuple]:
    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY,
    )

    if ENABLE_HISTOGRAM_EQUALIZATION:
        gray = cv2.equalizeHist(gray)

    faces = cascade.detectMultiScale(
        gray,
        scaleFactor=1.08,
        minNeighbors=4,
        minSize=(24, 24),
    )

    if len(faces) == 0:
        return None, faces

    x, y, w, h = max(
        faces,
        key=lambda face: face[2] * face[3],
    )

    frame_center_x = (
        frame.shape[1] // 2
        + CAMERA_CENTER_OFFSET_X
    )

    center_x = x + w // 2
    center_y = y + h // 2

    return (
        FaceResult(
            center_x=center_x,
            center_y=center_y,
            width=w,
            height=h,
            diff_x=center_x - frame_center_x,
        ),
        faces,
    )


# ============================================================
# 左右判定
# ============================================================

class HorizontalTurnController:
    def __init__(self) -> None:
        self.state = "CENTER"

    def reset(self) -> None:
        self.state = "CENTER"

    def update(self, diff_x: int) -> str:
        if self.state == "LEFT":
            if diff_x >= -TURN_EXIT_X:
                self.state = "CENTER"

        elif self.state == "RIGHT":
            if diff_x <= TURN_EXIT_X:
                self.state = "CENTER"

        else:
            if diff_x <= -TURN_ENTER_X:
                self.state = "LEFT"

            elif diff_x >= TURN_ENTER_X:
                self.state = "RIGHT"

        return self.state


class TurnStabilizer:
    def __init__(self) -> None:
        self.pending: Optional[str] = None
        self.count = 0

    def reset(self) -> None:
        self.pending = None
        self.count = 0

    def update(
        self,
        candidate: str,
    ) -> Tuple[Optional[str], int]:
        if candidate == "CENTER":
            self.reset()
            return None, 0

        if candidate == self.pending:
            self.count += 1
        else:
            self.pending = candidate
            self.count = 1

        if self.count >= TURN_CONFIRM_FRAMES:
            confirmed = self.pending
            self.reset()
            return confirmed, TURN_CONFIRM_FRAMES

        return None, self.count


# ============================================================
# ESP32通信
# ============================================================

class ESP32Serial:
    IMPORTANT_PREFIXES = (
        "[Walking]",
        "[MotionPlayer] Play:",
        "[MotionPlayer] Finished:",
        "[SerialReceive]",
        "Brownout",
        "Guru Meditation",
    )

    def __init__(self) -> None:
        self.ser: Optional[serial.Serial] = None
        self.rx_text = ""

        self.busy = False
        self.walking_state = "UNKNOWN"
        self.active_command: Optional[str] = None

        self.motion_started_at: Optional[float] = None
        self.idle_since = time.monotonic()

        self.just_became_idle = False
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

        # シリアル接続時のESP32再起動待ち。
        time.sleep(2.0)

        # 接続前の途中データを捨てる。
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.rx_text = ""

        print("[Serial] Connected")

        # 初期状態を明確にする。
        self.send_raw("STOP")
        time.sleep(0.2)
        self.send_raw("STATUS")

    def close(self) -> None:
        if (
            self.ser is not None
            and self.ser.is_open
        ):
            self.ser.close()
            print("[Serial] Closed")

    def send_raw(self, command: str) -> bool:
        if (
            self.ser is None
            or not self.ser.is_open
        ):
            print(
                "[Serial] ERROR: "
                "ポートが開いていません。"
            )
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

        except serial.SerialTimeoutException:
            print(
                "[Serial] ERROR: Write timeout"
            )
            self.fault = True
            return False

        print(f"[Serial] Sent: {command}")
        return True

    def send_turn(self, command: str) -> bool:
        command = command.upper()

        if command not in ("LEFT", "RIGHT"):
            raise ValueError(
                f"旋回以外の命令です: {command}"
            )

        if self.fault:
            print(
                "[Safety] fault中のため送信しません。"
            )
            return False

        if self.busy:
            print(
                "[Safety] ESP32が動作中のため"
                f"{command}を保留します。"
            )
            return False

        if not self.send_raw(command):
            return False

        # ESP32からログが返る前でも、送信直後からロックする。
        self.busy = True
        self.active_command = command
        self.motion_started_at = time.monotonic()
        self.just_became_idle = False

        print(
            f"[MotionGate] LOCK command={command}"
        )
        return True

    def _handle_line(self, line: str) -> None:
        if (
            PRINT_ALL_ESP32_LINES
            or line.startswith(
                self.IMPORTANT_PREFIXES
            )
            or "Walking State:" in line
        ):
            print(f"[ESP32] {line}")

        if "[Walking] State:" in line:
            state = (
                line.split(
                    "[Walking] State:",
                    1,
                )[1]
                .strip()
                .upper()
            )
            self._set_walking_state(state)

        elif line.startswith("Walking State:"):
            state = (
                line.split(
                    "Walking State:",
                    1,
                )[1]
                .strip()
                .upper()
            )
            self._set_walking_state(state)

        if "[MotionPlayer] Play:" in line:
            self.busy = True

            if self.motion_started_at is None:
                self.motion_started_at = (
                    time.monotonic()
                )

        if (
            "Brownout" in line
            or "Guru Meditation" in line
        ):
            self.fault = True
            print(
                "[Safety] ESP32異常ログを検出。"
                "自動旋回を停止します。"
            )

    def _set_walking_state(
        self,
        state: str,
    ) -> None:
        previous_state = self.walking_state
        self.walking_state = state

        if state == "IDLE":
            was_busy = self.busy

            self.busy = False
            self.active_command = None
            self.motion_started_at = None
            self.idle_since = time.monotonic()

            if was_busy or previous_state != "IDLE":
                self.just_became_idle = True

                print(
                    "[MotionGate] UNLOCK "
                    "ESP32 State=IDLE"
                )

        else:
            self.busy = True

            if self.motion_started_at is None:
                self.motion_started_at = (
                    time.monotonic()
                )

    def poll(self) -> None:
        if (
            self.ser is None
            or not self.ser.is_open
        ):
            return

        waiting = self.ser.in_waiting

        if waiting > 0:
            data = self.ser.read(waiting)

            self.rx_text += data.decode(
                "utf-8",
                errors="ignore",
            )

            while "\n" in self.rx_text:
                line, self.rx_text = (
                    self.rx_text.split("\n", 1)
                )

                line = line.strip("\r ")

                if line:
                    self._handle_line(line)

        self._check_timeout()

    def _check_timeout(self) -> None:
        if (
            not self.busy
            or self.motion_started_at is None
            or self.fault
        ):
            return

        elapsed = (
            time.monotonic()
            - self.motion_started_at
        )

        if elapsed < MOTION_TIMEOUT_SECONDS:
            return

        self.fault = True

        print(
            "[Safety] ERROR: "
            f"動作開始から{elapsed:.1f}秒経過しても"
            "IDLEが返りません。"
        )
        print(
            "[Safety] STOPを送信し、"
            "自動旋回を停止します。"
        )

        self.send_raw("STOP")

    def consume_idle_event(self) -> bool:
        if not self.just_became_idle:
            return False

        self.just_became_idle = False
        return True

    def ready_after_settle(self) -> bool:
        if self.busy or self.fault:
            return False

        return (
            time.monotonic()
            - self.idle_since
            >= POST_TURN_SETTLE_SECONDS
        )


# ============================================================
# 描画
# ============================================================

def draw_result(
    frame,
    faces,
    face: Optional[FaceResult],
    controller_state: str,
    pending_count: int,
    esp32: ESP32Serial,
    consecutive_turns: int,
) -> None:
    height, width = frame.shape[:2]

    center_x = (
        width // 2
        + CAMERA_CENTER_OFFSET_X
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
        (center_x - TURN_ENTER_X, 0),
        (center_x - TURN_ENTER_X, height),
        (0, 0, 255),
        1,
    )

    cv2.line(
        frame,
        (center_x + TURN_ENTER_X, 0),
        (center_x + TURN_ENTER_X, height),
        (0, 0, 255),
        1,
    )

    for x, y, w, h in faces:
        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (100, 100, 100),
            1,
        )

    if face is not None:
        x1 = face.center_x - face.width // 2
        y1 = face.center_y - face.height // 2

        cv2.rectangle(
            frame,
            (x1, y1),
            (
                x1 + face.width,
                y1 + face.height,
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
            f"diffX={face.diff_x}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

    cv2.putText(
        frame,
        (
            f"candidate={controller_state} "
            f"confirm={pending_count}/"
            f"{TURN_CONFIRM_FRAMES}"
        ),
        (10, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        (
            f"ESP32={esp32.walking_state} "
            f"busy={esp32.busy}"
        ),
        (10, 85),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        (
            f"turnsWithoutCenter="
            f"{consecutive_turns}/"
            f"{MAX_TURNS_WITHOUT_CENTER}"
        ),
        (10, 115),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
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
        "Presentation Escort"
    )
    print(
        "Raspberry Pi Turn Stability Test"
    )
    print(
        "=============================================="
    )
    print(
        f"[Config] mirror={MIRROR_CAMERA_IMAGE}"
    )
    print(
        f"[Config] enterX={TURN_ENTER_X}, "
        f"exitX={TURN_EXIT_X}"
    )
    print(
        f"[Config] confirmFrames="
        f"{TURN_CONFIRM_FRAMES}"
    )
    print(
        f"[Config] postTurnSettle="
        f"{POST_TURN_SETTLE_SECONDS:.2f}s"
    )
    print(
        "[Config] FORWARD=DISABLED, "
        "HEAD=DISABLED"
    )

    cascade = load_face_cascade()
    cap = open_camera()

    if not cap.isOpened():
        raise RuntimeError(
            "カメラを開けませんでした。"
        )

    # 起動直後の不安定な画像を捨てる。
    for _ in range(8):
        cap.read()

    controller = HorizontalTurnController()
    stabilizer = TurnStabilizer()
    esp32 = ESP32Serial()

    loop_count = 0
    no_face_count = 0

    # 旋回完了後に新しい画像を何回確認したか。
    fresh_frames_after_turn = (
        POST_TURN_RECHECK_FRAMES
    )

    consecutive_turns = 0
    center_confirm_count = 0

    try:
        esp32.open()

        print("[Loop] Start")
        print("[Loop] Ctrl+Cで停止します。")

        while True:
            loop_started = time.monotonic()
            loop_count += 1

            # 命令を送った時だけでなく毎ループ受信する。
            esp32.poll()

            if esp32.consume_idle_event():
                stabilizer.reset()
                controller.reset()

                fresh_frames_after_turn = 0

                print(
                    "[Safety] 旋回完了。"
                    "姿勢安定後に新しい画像で再判定します。"
                )

            ret, frame = cap.read()

            if not ret:
                raise RuntimeError(
                    "カメラ画像を取得できません。"
                )

            if MIRROR_CAMERA_IMAGE:
                frame = cv2.flip(frame, 1)

            face, faces = detect_main_face(
                frame,
                cascade,
            )

            pending_count = 0
            candidate = controller.state

            if face is None:
                no_face_count += 1

                if no_face_count >= NO_FACE_RESET_FRAMES:
                    controller.reset()
                    stabilizer.reset()
                    candidate = "CENTER"

                print(
                    f"[Loop {loop_count:04d}] "
                    f"NO FACE {no_face_count}/"
                    f"{NO_FACE_RESET_FRAMES} | "
                    f"ESP32={esp32.walking_state} "
                    f"busy={esp32.busy}"
                )

            else:
                no_face_count = 0

                candidate = controller.update(
                    face.diff_x
                )

                # 中央を数フレーム確認したら、
                # 連続旋回回数の制限を解除する。
                if candidate == "CENTER":
                    center_confirm_count += 1

                    if (
                        center_confirm_count
                        >= TURN_CONFIRM_FRAMES
                    ):
                        if consecutive_turns > 0:
                            print(
                                "[Safety] 顔の中央復帰を確認。"
                                "連続旋回カウンタをリセットします。"
                            )

                        consecutive_turns = 0
                        center_confirm_count = (
                            TURN_CONFIRM_FRAMES
                        )

                else:
                    center_confirm_count = 0

                # 動作中もカメラ処理は続けるが、
                # 次の命令の確定処理は行わない。
                can_evaluate_turn = (
                    not esp32.busy
                    and not esp32.fault
                    and esp32.ready_after_settle()
                    and fresh_frames_after_turn
                    >= POST_TURN_RECHECK_FRAMES
                )

                if (
                    not esp32.busy
                    and fresh_frames_after_turn
                    < POST_TURN_RECHECK_FRAMES
                ):
                    fresh_frames_after_turn += 1

                confirmed_turn = None

                if can_evaluate_turn:
                    (
                        confirmed_turn,
                        pending_count,
                    ) = stabilizer.update(candidate)
                else:
                    stabilizer.reset()

                print(
                    f"[Loop {loop_count:04d}] "
                    f"faceX={face.center_x} "
                    f"diffX={face.diff_x} | "
                    f"candidate={candidate} "
                    f"pending={pending_count}/"
                    f"{TURN_CONFIRM_FRAMES} | "
                    f"ESP32={esp32.walking_state} "
                    f"busy={esp32.busy} | "
                    f"fresh={fresh_frames_after_turn}/"
                    f"{POST_TURN_RECHECK_FRAMES} | "
                    f"turns={consecutive_turns}/"
                    f"{MAX_TURNS_WITHOUT_CENTER}"
                )

                if confirmed_turn is not None:
                    if (
                        consecutive_turns
                        >= MAX_TURNS_WITHOUT_CENTER
                    ):
                        print(
                            "[Safety] 中央を確認しないまま"
                            "連続旋回上限へ到達しました。"
                        )
                        print(
                            "[Safety] 顔が中央へ戻るまで"
                            "追加旋回を行いません。"
                        )
                        stabilizer.reset()

                    elif esp32.send_turn(
                        confirmed_turn
                    ):
                        consecutive_turns += 1
                        fresh_frames_after_turn = 0
                        stabilizer.reset()

            if (
                loop_count
                % SAVE_IMAGE_EVERY_N_LOOPS
                == 0
            ):
                draw_result(
                    frame,
                    faces,
                    face,
                    candidate,
                    pending_count,
                    esp32,
                    consecutive_turns,
                )

                if not cv2.imwrite(
                    str(RESULT_IMAGE_PATH),
                    frame,
                ):
                    print(
                        "[Image] WARN: "
                        f"保存できません: "
                        f"{RESULT_IMAGE_PATH}"
                    )

            if esp32.fault:
                print(
                    "[Safety] fault状態です。"
                    "プログラムを終了します。"
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
        print(
            "[Loop] ユーザー操作で停止しました。"
        )

    except serial.SerialException as error:
        print(
            "[Serial] ERROR: "
            f"{error}"
        )

    except Exception as error:
        print(
            "[Main] ERROR: "
            f"{type(error).__name__}: {error}"
        )

    finally:
        print(
            "[Safety] STOPを送信して終了します。"
        )

        try:
            esp32.send_raw("STOP")
            time.sleep(0.2)

        except Exception as error:
            print(
                "[Safety] STOP送信失敗: "
                f"{error}"
            )

        esp32.close()
        cap.release()

        print("[Main] Finished.")


if __name__ == "__main__":
    main()