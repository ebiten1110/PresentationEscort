import time

import cv2
import serial

import config as cfg


# ============================================================
# 設定値
# config.pyに値がない場合は、右側の初期値を使用する
# ============================================================

# カメラ
CAMERA_MODE = getattr(cfg, "CAMERA_MODE", "usb")
IP_CAMERA_URL = getattr(cfg, "IP_CAMERA_URL", "")
CAMERA_INDEX = getattr(cfg, "CAMERA_INDEX", 0)
FRAME_WIDTH = getattr(cfg, "FRAME_WIDTH", 640)
FRAME_HEIGHT = getattr(cfg, "FRAME_HEIGHT", 480)
MIRROR_CAMERA_IMAGE = getattr(cfg, "MIRROR_CAMERA_IMAGE", False)

# シリアル
SERIAL_PORT = getattr(cfg, "SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUD = getattr(cfg, "SERIAL_BAUD", 115200)
SERIAL_TIMEOUT = getattr(cfg, "SERIAL_TIMEOUT", 1.0)
SEND_COMMAND_TO_ESP32 = getattr(cfg, "SEND_COMMAND_TO_ESP32", True)

# 位置・距離判定
CENTER_TOLERANCE_X = getattr(cfg, "CENTER_TOLERANCE_X", 35)
CENTER_TOLERANCE_Y = getattr(cfg, "CENTER_TOLERANCE_Y", 35)
CAMERA_CENTER_OFFSET_X = getattr(cfg, "CAMERA_CENTER_OFFSET_X", 0)
CAMERA_CENTER_OFFSET_Y = getattr(cfg, "CAMERA_CENTER_OFFSET_Y", 0)
X_HYSTERESIS = getattr(cfg, "X_HYSTERESIS", 8)
Y_HYSTERESIS = getattr(cfg, "Y_HYSTERESIS", 8)
FACE_FAR_WIDTH = getattr(cfg, "FACE_FAR_WIDTH", 140)
FACE_NEAR_WIDTH = getattr(cfg, "FACE_NEAR_WIDTH", 230)
DISTANCE_HYSTERESIS = getattr(cfg, "DISTANCE_HYSTERESIS", 8)

# 判定確定
MOVE_CONFIRM_FRAMES = getattr(cfg, "MOVE_CONFIRM_FRAMES", 1)
STOP_CONFIRM_FRAMES = getattr(cfg, "STOP_CONFIRM_FRAMES", 1)
HEAD_CONFIRM_FRAMES = getattr(cfg, "HEAD_CONFIRM_FRAMES", 1)
NO_FACE_STOP_THRESHOLD = getattr(
    cfg,
    "ADVANCED_NO_FACE_STOP_THRESHOLD",
    2,
)

# コマンド送信
SEND_ONLY_ON_CHANGE = getattr(
    cfg,
    "ADVANCED_SEND_ONLY_ON_CHANGE",
    True,
)
MOVE_REPEAT_INTERVAL = getattr(cfg, "MOVE_REPEAT_INTERVAL", 0.6)
SEND_MOVE_COMMAND = getattr(cfg, "SEND_MOVE_COMMAND", True)
SEND_HEAD_COMMAND = getattr(cfg, "SEND_HEAD_COMMAND", True)
INVERT_HORIZONTAL_COMMANDS = getattr(
    cfg,
    "INVERT_HORIZONTAL_COMMANDS",
    False,
)

# ループ
FOLLOW_LOOP_INTERVAL = getattr(
    cfg,
    "ADVANCED_FOLLOW_LOOP_INTERVAL",
    0.15,
)
FOLLOW_LOOP_MAX_COUNT = getattr(
    cfg,
    "ADVANCED_FOLLOW_LOOP_MAX_COUNT",
    None,
)

# 保存・検出
RESULT_IMAGE_PATH = getattr(
    cfg,
    "ADVANCED_FOLLOW_LOOP_IMAGE_PATH",
    "advanced_follow_loop_latest.jpg",
)
SAVE_RESULT_IMAGE = getattr(cfg, "SAVE_RESULT_IMAGE", True)
SAVE_RESULT_EVERY_N_LOOPS = max(
    1,
    getattr(cfg, "SAVE_RESULT_EVERY_N_LOOPS", 5),
)
ENABLE_HISTOGRAM_EQUALIZATION = getattr(
    cfg,
    "ENABLE_HISTOGRAM_EQUALIZATION",
    True,
)

CASCADE_PATH = getattr(
    cfg,
    "HAAR_CASCADE_PATH",
    (
        "/usr/share/opencv4/haarcascades/"
        "haarcascade_frontalface_default.xml"
    ),
)


# ============================================================
# カメラ
# ============================================================

def open_camera():
    """設定に応じてUSBカメラまたはIPカメラを開く。"""

    if CAMERA_MODE == "usb":
        print(f"[Camera] Mode=USB, index={CAMERA_INDEX}")
        cap = cv2.VideoCapture(CAMERA_INDEX)

    elif CAMERA_MODE == "ip":
        print(f"[Camera] Mode=IP, url={IP_CAMERA_URL}")
        cap = cv2.VideoCapture(IP_CAMERA_URL)

    else:
        print(f"[Camera] ERROR: 不明なCAMERA_MODEです: {CAMERA_MODE}")
        print('[Camera] "usb" または "ip" を指定してください。')
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # 対応カメラのみ反映される。
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    return cap


# ============================================================
# 顔検出
# ============================================================

def load_face_cascade():
    """OpenCV Haar Cascadeを読み込む。"""

    face_cascade = cv2.CascadeClassifier(CASCADE_PATH)

    if face_cascade.empty():
        print("[FaceDetect] ERROR: Cascadeを読み込めませんでした。")
        print(f"[FaceDetect] Path={CASCADE_PATH}")
        return None

    print(f"[FaceDetect] Cascade loaded: {CASCADE_PATH}")
    return face_cascade


def detect_faces(frame, face_cascade):
    """画像から顔を検出する。"""

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 暗めの環境でコントラストを改善する。
    if ENABLE_HISTOGRAM_EQUALIZATION:
        gray = cv2.equalizeHist(gray)

    return face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.08,
        minNeighbors=4,
        minSize=(24, 24),
    )


def select_main_face(faces):
    """複数顔がある場合、最も大きい顔を追従対象にする。"""

    if len(faces) == 0:
        return None

    return max(faces, key=lambda face: face[2] * face[3])


# ============================================================
# ヒステリシス付き判定
# ============================================================

class HorizontalController:
    """LEFT / CENTER / RIGHTをヒステリシス付きで判定する。"""

    def __init__(self):
        self.state = "CENTER"

    def reset(self):
        self.state = "CENTER"

    def update(self, diff_x):
        outer = CENTER_TOLERANCE_X + X_HYSTERESIS
        inner = max(0, CENTER_TOLERANCE_X - X_HYSTERESIS)

        if self.state == "LEFT":
            # LEFTへ入った後は、十分中央へ戻るまで維持する。
            if diff_x >= -inner:
                self.state = "CENTER"

        elif self.state == "RIGHT":
            if diff_x <= inner:
                self.state = "CENTER"

        else:
            # CENTERから左右へ移るときは外側の境界を使う。
            if diff_x <= -outer:
                self.state = "LEFT"
            elif diff_x >= outer:
                self.state = "RIGHT"

        return self.state


class VerticalController:
    """UP / CENTER / DOWNをヒステリシス付きで判定する。"""

    def __init__(self):
        self.state = "CENTER"

    def reset(self):
        self.state = "CENTER"

    def update(self, diff_y):
        outer = CENTER_TOLERANCE_Y + Y_HYSTERESIS
        inner = max(0, CENTER_TOLERANCE_Y - Y_HYSTERESIS)

        if self.state == "UP":
            if diff_y >= -inner:
                self.state = "CENTER"

        elif self.state == "DOWN":
            if diff_y <= inner:
                self.state = "CENTER"

        else:
            # 画像では上ほどY座標が小さい。
            if diff_y <= -outer:
                self.state = "UP"
            elif diff_y >= outer:
                self.state = "DOWN"

        return self.state


class DistanceController:
    """顔幅からFAR / GOOD / NEARをヒステリシス付きで判定する。"""

    def __init__(self):
        self.state = "GOOD"

    def reset(self):
        self.state = "GOOD"

    def update(self, face_width):
        far_lower = FACE_FAR_WIDTH - DISTANCE_HYSTERESIS
        far_upper = FACE_FAR_WIDTH + DISTANCE_HYSTERESIS

        near_lower = FACE_NEAR_WIDTH - DISTANCE_HYSTERESIS
        near_upper = FACE_NEAR_WIDTH + DISTANCE_HYSTERESIS

        if self.state == "FAR":
            if face_width >= near_upper:
                self.state = "NEAR"
            elif face_width >= far_upper:
                self.state = "GOOD"

        elif self.state == "NEAR":
            if face_width <= far_lower:
                self.state = "FAR"
            elif face_width <= near_lower:
                self.state = "GOOD"

        else:
            if face_width <= far_lower:
                self.state = "FAR"
            elif face_width >= near_upper:
                self.state = "NEAR"

        return self.state


# ============================================================
# 連続判定によるコマンド確定
# ============================================================

class CommandStabilizer:
    """同じ候補が指定回数続いた場合にコマンドを確定する。"""

    def __init__(self, initial_command):
        self.confirmed = initial_command
        self.pending = None
        self.pending_count = 0

    def force(self, command):
        """安全停止など、待たずにコマンドを確定する。"""

        changed = command != self.confirmed
        self.confirmed = command
        self.pending = None
        self.pending_count = 0
        return self.confirmed, changed, 0

    def update(self, candidate, required_frames):
        """候補が必要回数続いた場合に確定する。"""

        required_frames = max(1, required_frames)

        if candidate == self.confirmed:
            self.pending = None
            self.pending_count = 0
            return self.confirmed, False, 0

        if candidate == self.pending:
            self.pending_count += 1
        else:
            self.pending = candidate
            self.pending_count = 1

        if self.pending_count >= required_frames:
            self.confirmed = candidate
            self.pending = None
            self.pending_count = 0
            return self.confirmed, True, required_frames

        return self.confirmed, False, self.pending_count


# ============================================================
# コマンド決定
# ============================================================

def horizontal_state_to_command(horizontal_state):
    """左右状態をESP32コマンドへ変換する。"""

    if horizontal_state == "LEFT":
        command = "LEFT"
    elif horizontal_state == "RIGHT":
        command = "RIGHT"
    else:
        return "STOP"

    if INVERT_HORIZONTAL_COMMANDS:
        return "RIGHT" if command == "LEFT" else "LEFT"

    return command


def vertical_state_to_command(
    vertical_state,
    current_head_command,
):
    """
    顔が上下へずれた場合だけ頭を動かす。

    CENTERへ入った場合はHEAD_CENTERへ戻さず、
    現在の頭位置を維持する。
    """

    if vertical_state == "UP":
        return "HEAD_UP"

    if vertical_state == "DOWN":
        return "HEAD_DOWN"

    return current_head_command


def decide_move_candidate(
    horizontal_state,
    distance_state,
    current_confirmed_command,
):
    """
    歩行コマンド候補を決定する。

    優先順位:
    1. NEARなら即STOP
    2. 左右にずれていればLEFT / RIGHT
    3. 中央かつFARならFORWARD
    4. 中央かつGOODならSTOP
    """

    if distance_state == "NEAR":
        return "STOP", "顔が近すぎるため安全停止"

    if horizontal_state in ("LEFT", "RIGHT"):
        command = horizontal_state_to_command(horizontal_state)
        return (
            command,
            f"顔が中央から{horizontal_state}方向へずれている",
        )

    if distance_state == "FAR":
        # 旋回から前進へ直接切り替えず、一度STOPを挟む。
        if current_confirmed_command in ("LEFT", "RIGHT"):
            return "STOP", "旋回完了後、前進前に一度停止する"

        return "FORWARD", "顔が左右中央かつ遠いため前進"

    return "STOP", "顔が左右中央かつ適正距離のため停止"


# ============================================================
# ESP32シリアル通信
# ============================================================

class ESP32Serial:
    def __init__(self):
        self.ser = None

    def open(self):
        if not SEND_COMMAND_TO_ESP32:
            print("[Serial] DRY RUN: ESP32へは送信しません。")
            return

        print(
            f"[Serial] Opening port={SERIAL_PORT}, "
            f"baud={SERIAL_BAUD}"
        )

        self.ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=SERIAL_TIMEOUT,
        )

        # ポートを開くとESP32が再起動する場合がある。
        time.sleep(2.0)
        print("[Serial] Opened")
        self.read_available(0.5)

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("[Serial] Closed")

    def send(self, command):
        if not SEND_COMMAND_TO_ESP32:
            print(f"[Serial] Skip: {command}")
            return False

        if self.ser is None or not self.ser.is_open:
            print("[Serial] ERROR: ポートが開かれていません。")
            return False

        message = command.strip().upper() + "\n"
        self.ser.write(message.encode("utf-8"))
        self.ser.flush()

        print(f"[Serial] Sent: {command}")
        return True

    def read_available(self, duration=0.08):
        if self.ser is None or not self.ser.is_open:
            return

        start_time = time.monotonic()

        while time.monotonic() - start_time < duration:
            if self.ser.in_waiting > 0:
                line = (
                    self.ser.readline()
                    .decode("utf-8", errors="ignore")
                    .strip()
                )

                if line:
                    print(f"[ESP32] {line}")


# ============================================================
# 結果画像
# ============================================================

def draw_result(
    frame,
    faces,
    main_face,
    result,
    loop_count,
):
    """最新の検出・判定結果を画像へ描画する。"""

    height, width = frame.shape[:2]
    center_x = width // 2 + CAMERA_CENTER_OFFSET_X
    center_y = height // 2 + CAMERA_CENTER_OFFSET_Y

    # 論理上の中心線
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

    # 中央判定範囲
    cv2.line(
        frame,
        (center_x - CENTER_TOLERANCE_X, 0),
        (center_x - CENTER_TOLERANCE_X, height),
        (0, 255, 255),
        1,
    )
    cv2.line(
        frame,
        (center_x + CENTER_TOLERANCE_X, 0),
        (center_x + CENTER_TOLERANCE_X, height),
        (0, 255, 255),
        1,
    )
    cv2.line(
        frame,
        (0, center_y - CENTER_TOLERANCE_Y),
        (width, center_y - CENTER_TOLERANCE_Y),
        (255, 255, 0),
        1,
    )
    cv2.line(
        frame,
        (0, center_y + CENTER_TOLERANCE_Y),
        (width, center_y + CENTER_TOLERANCE_Y),
        (255, 255, 0),
        1,
    )

    # 全検出顔
    for x, y, w, h in faces:
        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (90, 90, 90),
            1,
        )

    if main_face is not None:
        x, y, w, h = main_face
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (255, 0, 0),
            2,
        )
        cv2.circle(
            frame,
            (face_center_x, face_center_y),
            6,
            (0, 255, 255),
            -1,
        )

        cv2.putText(
            frame,
            f"Face=({face_center_x},{face_center_y}) size={w}x{h}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Diff=({result['diff_x']},{result['diff_y']})",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
    else:
        cv2.putText(
            frame,
            "No face detected",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    cv2.putText(
        frame,
        (
            f"State X={result['horizontal_state']} "
            f"Y={result['vertical_state']} "
            f"D={result['distance_state']}"
        ),
        (10, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        (
            f"Candidate={result['candidate_move']} "
            f"Confirmed={result['confirmed_move']}"
        ),
        (10, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"Head={result['confirmed_head']}",
        (10, 140),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"Loop={loop_count}",
        (10, 170),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    return frame


# ============================================================
# メイン
# ============================================================

def main():
    print("================================================")
    print("Presentation Escort Phase 5.9")
    print("Stable Advanced Follow Loop")
    print("================================================")
    print(
        f"[Config] mirror={MIRROR_CAMERA_IMAGE}, "
        f"invertHorizontal={INVERT_HORIZONTAL_COMMANDS}"
    )
    print(
        f"[Config] tolerance=({CENTER_TOLERANCE_X},"
        f"{CENTER_TOLERANCE_Y}), "
        f"hysteresis=({X_HYSTERESIS},{Y_HYSTERESIS})"
    )
    print(
        f"[Config] distance FAR<{FACE_FAR_WIDTH}, "
        f"NEAR>{FACE_NEAR_WIDTH}"
    )

    if MIRROR_CAMERA_IMAGE and INVERT_HORIZONTAL_COMMANDS:
        print(
            "[Config] WARN: 映像反転とコマンド反転が両方有効です。"
            "二重反転になっていないか確認してください。"
        )

    face_cascade = load_face_cascade()
    if face_cascade is None:
        return

    cap = open_camera()
    if cap is None or not cap.isOpened():
        print("[Camera] ERROR: カメラを開けませんでした。")
        return

    # 起動直後の不安定なフレームを捨てる。
    for _ in range(5):
        cap.read()

    horizontal_controller = HorizontalController()
    vertical_controller = VerticalController()
    distance_controller = DistanceController()

    move_stabilizer = CommandStabilizer("STOP")
    head_stabilizer = CommandStabilizer("HEAD_CENTER")

    esp32 = ESP32Serial()

    loop_count = 0
    no_face_count = 0

    last_sent_move = None
    last_sent_head = None
    last_move_sent_time = 0.0

    try:
        esp32.open()

        # 起動時の安全状態
        if SEND_MOVE_COMMAND:
            esp32.send("STOP")
            last_sent_move = "STOP"
            last_move_sent_time = time.monotonic()

        if SEND_HEAD_COMMAND:
            esp32.send("HEAD_CENTER")
            last_sent_head = "HEAD_CENTER"

        print("[Loop] Start")
        print("[Loop] Ctrl+Cで停止します。")

        while True:
            loop_started = time.monotonic()

            if (
                FOLLOW_LOOP_MAX_COUNT is not None
                and loop_count >= FOLLOW_LOOP_MAX_COUNT
            ):
                print("[Loop] 最大ループ回数へ到達しました。")
                break

            loop_count += 1

            ret, frame = cap.read()

            if not ret:
                print("[Camera] ERROR: フレーム取得に失敗しました。")
                break

            if MIRROR_CAMERA_IMAGE:
                frame = cv2.flip(frame, 1)

            height, width = frame.shape[:2]
            screen_center_x = (
                width // 2 + CAMERA_CENTER_OFFSET_X
            )
            screen_center_y = (
                height // 2 + CAMERA_CENTER_OFFSET_Y
            )

            faces = detect_faces(frame, face_cascade)
            main_face = select_main_face(faces)

            # 描画・ログ用の初期値
            diff_x = 0
            diff_y = 0

            horizontal_state = horizontal_controller.state
            vertical_state = vertical_controller.state
            distance_state = distance_controller.state

            candidate_move = move_stabilizer.confirmed
            candidate_head = head_stabilizer.confirmed

            move_pending_count = 0
            head_pending_count = 0

            reason = ""
            allow_move_repeat = False

            if main_face is None:
                # -----------------------------------------------
                # 顔を見失った場合
                # -----------------------------------------------
                no_face_count += 1

                if no_face_count >= NO_FACE_STOP_THRESHOLD:
                    confirmed_move, _, _ = move_stabilizer.force(
                        "STOP"
                    )
                    confirmed_head, _, _ = head_stabilizer.force(
                        "HEAD_CENTER"
                    )

                    horizontal_controller.reset()
                    vertical_controller.reset()
                    distance_controller.reset()

                    horizontal_state = "CENTER"
                    vertical_state = "CENTER"
                    distance_state = "UNKNOWN"
                    candidate_move = "STOP"
                    candidate_head = "HEAD_CENTER"

                    reason = (
                        f"顔ロストが{no_face_count}回続いたため安全停止"
                    )
                else:
                    # 猶予中は以前の状態を維持する。
                    # 移動コマンドの再送は行わない。
                    confirmed_move = move_stabilizer.confirmed
                    confirmed_head = head_stabilizer.confirmed
                    distance_state = "UNKNOWN"

                    reason = (
                        f"顔ロスト猶予中 "
                        f"{no_face_count}/{NO_FACE_STOP_THRESHOLD}"
                    )

                print(
                    f"[Loop {loop_count:03d}] "
                    f"NO FACE | "
                    f"move={confirmed_move} | "
                    f"head={confirmed_head} | "
                    f"reason={reason}"
                )

            else:
                # -----------------------------------------------
                # 顔を検出した場合
                # -----------------------------------------------
                no_face_count = 0

                x, y, w, h = main_face
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                diff_x = face_center_x - screen_center_x
                diff_y = face_center_y - screen_center_y

                horizontal_state = horizontal_controller.update(
                    diff_x
                )
                vertical_state = vertical_controller.update(
                    diff_y
                )
                distance_state = distance_controller.update(w)

                candidate_move, reason = decide_move_candidate(
                    horizontal_state,
                    distance_state,
                    move_stabilizer.confirmed,
                )

                # 顔が上下中央に入ってもHEAD_CENTERへ戻さず、
                # 現在の頭位置を維持する。
                candidate_head = vertical_state_to_command(
                    vertical_state,
                    head_stabilizer.confirmed,
                )

                # NEARは確認回数を待たず即時STOP。
                if distance_state == "NEAR":
                    confirmed_move, _, _ = move_stabilizer.force(
                        "STOP"
                    )
                    move_pending_count = 0
                else:
                    required_move_frames = (
                        STOP_CONFIRM_FRAMES
                        if candidate_move == "STOP"
                        else MOVE_CONFIRM_FRAMES
                    )

                    (
                        confirmed_move,
                        _,
                        move_pending_count,
                    ) = move_stabilizer.update(
                        candidate_move,
                        required_move_frames,
                    )

                (
                    confirmed_head,
                    _,
                    head_pending_count,
                ) = head_stabilizer.update(
                    candidate_head,
                    HEAD_CONFIRM_FRAMES,
                )

                allow_move_repeat = True

                print(
                    f"[Loop {loop_count:03d}] "
                    f"face=({face_center_x},{face_center_y}) "
                    f"size=({w},{h}) "
                    f"diff=({diff_x},{diff_y}) | "
                    f"X={horizontal_state} "
                    f"Y={vertical_state} "
                    f"D={distance_state} | "
                    f"candidate={candidate_move} "
                    f"pending={move_pending_count} "
                    f"confirmed={confirmed_move} | "
                    f"headCandidate={candidate_head} "
                    f"headPending={head_pending_count} "
                    f"head={confirmed_head} | "
                    f"reason={reason}"
                )

            now = time.monotonic()
            sent_any_command = False

            # -----------------------------------------------
            # 頭コマンド送信
            # -----------------------------------------------
            if SEND_HEAD_COMMAND:
                should_send_head = False

                if not SEND_ONLY_ON_CHANGE:
                    should_send_head = True
                elif confirmed_head != last_sent_head:
                    should_send_head = True

                if should_send_head:
                    if esp32.send(confirmed_head):
                        last_sent_head = confirmed_head
                        sent_any_command = True

            # -----------------------------------------------
            # 歩行コマンド送信
            # -----------------------------------------------
            if SEND_MOVE_COMMAND:
                should_send_move = False

                if confirmed_move != last_sent_move:
                    # コマンドが変わった場合
                    should_send_move = True

                elif (
                    allow_move_repeat
                    and confirmed_move
                    in ("LEFT", "RIGHT", "FORWARD")
                    and now - last_move_sent_time
                    >= MOVE_REPEAT_INTERVAL
                ):
                    # ESP32側が一回モーションの場合、
                    # 同じ移動命令を定期的に再送する。
                    should_send_move = True

                elif not SEND_ONLY_ON_CHANGE:
                    should_send_move = True

                if should_send_move:
                    if esp32.send(confirmed_move):
                        last_sent_move = confirmed_move
                        last_move_sent_time = now
                        sent_any_command = True

            if sent_any_command:
                esp32.read_available(0.08)

            result = {
                "diff_x": diff_x,
                "diff_y": diff_y,
                "horizontal_state": horizontal_state,
                "vertical_state": vertical_state,
                "distance_state": distance_state,
                "candidate_move": candidate_move,
                "confirmed_move": move_stabilizer.confirmed,
                "confirmed_head": head_stabilizer.confirmed,
            }

            if (
                SAVE_RESULT_IMAGE
                and loop_count % SAVE_RESULT_EVERY_N_LOOPS == 0
            ):
                result_frame = draw_result(
                    frame.copy(),
                    faces,
                    main_face,
                    result,
                    loop_count,
                )

                saved = cv2.imwrite(
                    RESULT_IMAGE_PATH,
                    result_frame,
                )

                if not saved:
                    print(
                        f"[Image] WARN: 保存に失敗しました: "
                        f"{RESULT_IMAGE_PATH}"
                    )

            # -----------------------------------------------
            # 処理時間表示とループ周期調整
            # -----------------------------------------------
            elapsed = time.monotonic() - loop_started
            processing_ms = elapsed * 1000

            print(
                f"[Performance] loop={processing_ms:.1f}ms "
                f"target={FOLLOW_LOOP_INTERVAL * 1000:.0f}ms"
            )

            sleep_time = FOLLOW_LOOP_INTERVAL - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print()
        print("[Loop] ユーザー操作で停止しました。")

    except serial.SerialException as error:
        print("[Serial] ERROR: シリアル通信に失敗しました。")
        print(error)

    except Exception as error:
        print("[Main] ERROR: 予期しないエラーが発生しました。")
        print(type(error).__name__, error)

    finally:
        print("[Safety] STOPとHEAD_CENTERを送信します。")

        try:
            if SEND_MOVE_COMMAND:
                esp32.send("STOP")
                time.sleep(0.1)

            if SEND_HEAD_COMMAND:
                esp32.send("HEAD_CENTER")

        except Exception as error:
            print(f"[Safety] 安全終了時の送信失敗: {error}")

        esp32.close()
        cap.release()
        print("[Main] Finished.")


if __name__ == "__main__":
    main()