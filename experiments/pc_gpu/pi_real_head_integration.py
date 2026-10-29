"""Controlled Pi camera -> PC YOLO -> presenter lock -> REAL ESP32 bridge.

Default: read-only UART monitor (servo power may be ON; no software ARM).
Opt-in, supervised: head yaw <=3 degrees. Pitch zero unless calibrated.
Body tracking turns require separate opt-in and calibration, with no session turn-count cap.
No translation or automatic ARM on startup/re-connection.
Power/physical ESTOP status is operator-declared, not measured by software.
"""
from __future__ import annotations

import argparse
import math
import re
import secrets
import time
from dataclasses import replace
from typing import Optional

from esp32_feedback_poller import Esp32FeedbackPoller
from motion_gateway import PiMotionGateway
from safety_manager import ControlDecision
from pi_operator_console import CameraInferenceWorker, OperatorRuntime, OperatorServer
from pi_presenter_lock_demo import LockedSimulationRuntime
from presenter_lock import PresenterLock
from remote_person_controller import FollowCycle, RemotePersonController
from person_target_adapter import detection_to_person_target
from safety_manager import Mode
from serial_command_sink import SerialCommandSink
from v883_person_dual_loop import V883PersonDualLoop

_POSE = re.compile(r"^HEAD_POSE_SET:([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?)$")


class HeadOnlyPlanner(V883PersonDualLoop):
    """Limit the planner's internal state, optionally using calibrated pitch."""

    def __init__(self, *, pitch_min: float = 0.0, pitch_max: float = 0.0,
                 invert_horizontal: bool = False):
        super().__init__()
        self.pitch_min = pitch_min
        self.pitch_max = pitch_max
        self.invert_horizontal = invert_horizontal

    def step(self, target, *args, **kwargs):
        if self.invert_horizontal:
            target = replace(target, error_x=-target.error_x)
        plan = super().step(target, *args, **kwargs)
        self.yaw_target_deg = max(-3.0, min(3.0, self.yaw_target_deg))
        self.pitch_target_deg = max(self.pitch_min, min(self.pitch_max, self.pitch_target_deg))
        self.body_candidate = None
        self.body_candidate_count = 0
        self.motion_pending = False
        if plan.head_pose_command is not None:
            self.last_sent_yaw = self.yaw_target_deg
            self.last_sent_pitch = self.pitch_target_deg
            head_command = f"HEAD_POSE_SET:{self.yaw_target_deg:.1f},{self.pitch_target_deg:.1f}"
        else:
            head_command = None
        return replace(plan, body_command=None, suggested_body_turn=None,
                       yaw_target_deg=self.yaw_target_deg,
                       pitch_target_deg=self.pitch_target_deg,
                       head_pose_command=head_command)


class HeadBodyTurnPlanner(V883PersonDualLoop):
    """Measured-feedback turns with visual confirmation, no session turn quota."""

    def __init__(self, *, pitch_min: float = 0.0, pitch_max: float = 0.0,
                 stable_frames: int = 4, settle_s: float = 1.0,
                 edge_fraction: float = 0.15,
                 invert_horizontal: bool = False):
        super().__init__()
        self.pitch_min = pitch_min
        self.pitch_max = pitch_max
        self.stable_frames = stable_frames
        self.settle_s = settle_s
        self.edge_fraction = edge_fraction
        self.invert_horizontal = invert_horizontal
        self._turn_proposals = 0
        self._proposal_direction = None

    def motion_completed(self, now: float, command: str) -> None:
        super().motion_completed(now, command)
        self._turn_proposals = 0
        self._proposal_direction = None

    def step(self, target, *, now, **kwargs):
        if self.invert_horizontal:
            target = replace(target, error_x=-target.error_x)
        plan = super().step(target, now=now, **kwargs)
        self.yaw_target_deg = max(-3.0, min(3.0, self.yaw_target_deg))
        self.pitch_target_deg = max(self.pitch_min, min(self.pitch_max, self.pitch_target_deg))
        head_command = None
        if plan.head_pose_command is not None:
            self.last_sent_yaw = self.yaw_target_deg
            self.last_sent_pitch = self.pitch_target_deg
            head_command = f"HEAD_POSE_SET:{self.yaw_target_deg:.1f},{self.pitch_target_deg:.1f}"
        body_command = plan.body_command
        reason = plan.reason
        if body_command is not None:
            normalized_error = abs(target.error_x) / float(target.frame_width)
            settled = (self.last_turn_completed_at is None or
                       now - self.last_turn_completed_at >= self.settle_s)
            # Head servo handles smaller movements. Only after yaw reaches
            # its calibrated boundary and the face approaches the edge do
            # we request a feedback-gated body turn while vision is still valid.
            head_near_limit = abs(self.yaw_target_deg) >= 2.5
            if (normalized_error < self.edge_fraction or
                    not head_near_limit or not settled):
                reason = "BODY_WAIT_TARGET_EDGE_OR_SETTLING"
                body_command = None
                self._turn_proposals = 0
                self._proposal_direction = None
            else:
                if body_command == self._proposal_direction:
                    self._turn_proposals += 1
                else:
                    self._proposal_direction = body_command
                    self._turn_proposals = 1
                if self._turn_proposals < self.stable_frames:
                    reason = "BODY_WAIT_STABLE_DIRECTION"
                    body_command = None
                else:
                    self._turn_proposals = 0
                    self._proposal_direction = None
            if body_command is None:
                # The base planner assumes a proposal was sent; clear that
                # flag when our stabilizer withheld it. No motion was issued.
                self.motion_pending = False
        elif not kwargs.get("head_feedback_fresh", False):
            self._turn_proposals = 0
            self._proposal_direction = None
        return replace(plan, yaw_target_deg=self.yaw_target_deg,
                       pitch_target_deg=self.pitch_target_deg,
                       head_pose_command=head_command, body_command=body_command,
                       reason=reason)


class HeadOnlySerialSink:
    requires_watchdog = True
    requires_firmware_safety = True

    def __init__(self, port: str, *, head_enabled: bool,
                 body_enabled: bool = False, pitch_min: float = 0.,
                 pitch_max: float = 0.):
        self._inner = SerialCommandSink(port)
        self.head_enabled = head_enabled
        self.body_enabled = body_enabled
        self.pitch_min = pitch_min
        self.pitch_max = pitch_max
        self._serial_partial = ""
        self.last_safety_event: Optional[str] = None
        self.last_servo_diagnostic: Optional[str] = None

    def send(self, command: str) -> None:
        cmd = command.strip().upper()
        if cmd.startswith("TRACK_") or cmd in {"LEFT", "RIGHT", "FORWARD", "BACKWARD", "STAND"}:
            if not (self.body_enabled and cmd in {"TRACK_LEFT", "TRACK_RIGHT"}
):
                raise ValueError("only TRACK_LEFT / TRACK_RIGHT are enabled; translation blocked")
        pose = _POSE.fullmatch(cmd)
        if pose is not None:
            yaw, pitch = float(pose[1]), float(pose[2])
            if (not self.head_enabled or not math.isfinite(yaw)
                    or not math.isfinite(pitch) or abs(yaw) > 3.0
                    or not self.pitch_min - .001 <= pitch <= self.pitch_max + .001):
                raise ValueError("head motion beyond calibrated head-only test scope")
        elif cmd.startswith("HEAD_"):
            # HEAD_STATUS read-only is allowed; all other head variants blocked.
            if cmd != "HEAD_STATUS":
                raise ValueError("unapproved head command")
        elif cmd == "SAFETY_ARM" and not self.head_enabled:
            raise ValueError("ARM is forbidden in UART probe mode")
        self._inner.send(cmd)

    def read_available(self, *, max_bytes: int = 2048) -> bytes:
        raw = self._inner.read_available(max_bytes=max_bytes)
        if raw:
            incoming = self._serial_partial + raw.decode("ascii", errors="replace")
            *lines, self._serial_partial = incoming.split("\n")
            self._serial_partial = self._serial_partial[-256:]
            for line in lines:
                line = line.strip()
                # Store ESP32's actual diagnostic line, not a guess inferred
                # from presenter detection or an old ARM-success message.
                if line.startswith("[ServoDiag]"):
                    self.last_servo_diagnostic = line[:180]
                if line.startswith("[Safety] FAULT=") or line in (
                    "[Safety] ARM_REJECTED",
                    "[Safety] RESET_REJECTED",
                ) or line.startswith("[Safety] MOTION_REJECTED="):
                    self.last_safety_event = line[:180]
        return raw

    def close(self) -> None:
        self._inner.close()


class BoundedSearchGateway(PiMotionGateway):
    """Real-runner-only search permit; baseline Phase10 gateway unchanged."""

    def request_recovery_turn(self, command: str, *,
                              now: Optional[float] = None) -> ControlDecision:
        with self._lock:
            self._tick_locked(now)
            if (self._transport_fault or self._closed or
                    (self._requires_watchdog and
                     not (self._thread and self._thread.is_alive())) or
                    (self._firmware_safety is not None and
                     not self._firmware_safety.fresh_armed(self.safety._now(now)))):
                return ControlDecision(False, "STOP", "SEARCH_LINK_NOT_READY",
                                       self.safety.mode)
            decision = self.safety.authorize_recovery_turn(command, now=now)
            if not decision.approved:
                return decision
            try:
                self.sink.send(decision.command)
                self._last_command = decision.command
                self._motion_pending = True
                return decision
            except Exception:
                self._transport_fault = True
                self.safety.emergency_stop()
                self._stop_locked("SEARCH_SEND_FAILED", force=True)
                return ControlDecision(False, "STOP", "SEARCH_SEND_FAILED", Mode.ESTOP)


class BoundedSearchController(RemotePersonController):
    """No planner output until three matched frames restore visual tracking."""

    def process(self, result, *, rtt_ms: float,
                now: Optional[float] = None) -> FollowCycle:
        if not self.gateway.safety.recovering:
            return super().process(result, rtt_ms=rtt_ms, now=now)
        current = time.monotonic() if now is None else now
        valid = detection_to_person_target(result) is not None and not result.error
        self.gateway.observe_detection(valid, rtt_ms, now=current)
        mode = self.gateway.status(now=current).safety.mode
        if not valid or self.gateway.safety.recovering or mode != Mode.AUTO:
            return FollowCycle(mode, valid, self.planner.target_lost(),
                               False, False, "SEARCH_REACQUISITION_CONFIRMING")
        return super().process(result, rtt_ms=rtt_ms, now=current)


class LockedRealRuntime(LockedSimulationRuntime):
    """Reuse real V2 presenter-lock logic and safety gateway without a virtual MCU."""

    def __init__(self, serial_port: str, *, head_enabled: bool = False,
                 power_declared: str = "UNKNOWN", body_enabled: bool = False,
                 pitch_bounds: tuple[float, float] = (0.0, 0.0),
                 turn_stable_frames: int = 4,
                 turn_settle_seconds: float = 1.0,
                 turn_edge_fraction: float = .15,
                 invert_horizontal: bool = True) -> None:
        if body_enabled and not head_enabled:
            raise ValueError("body turns require head control opt-in")
        super().__init__()
        # Gesture selects the actual person; never silently select a passerby.
        self.presenter = PresenterLock(gesture_enabled=True,
                                       gesture_confirm_frames=4,
                                       transient_recovery_s=1.5)
        self.gesture_control_enabled = False
        self._gesture_handled_event = 0
        self._gesture_pending_event = 0
        self._gesture_pending_since = 0.0
        self.invert_horizontal = invert_horizontal
        self._preview_jpeg: Optional[bytes] = None
        self._preview_resolution: Optional[str] = None
        self.turn_edge_fraction = turn_edge_fraction
        self._last_camera_aim: Optional[tuple[int, int]] = None
        self._last_camera_aim_at = 0.0
        # Superclass has not started any threads or issued any commands.
        self.sink.close()
        self.sink = HeadOnlySerialSink(
            serial_port, head_enabled=head_enabled, body_enabled=body_enabled,
            pitch_min=pitch_bounds[0], pitch_max=pitch_bounds[1])
        self.gateway = BoundedSearchGateway(self.sink)
        self.controller = BoundedSearchController(
            self.gateway,
            planner=(HeadBodyTurnPlanner(
                pitch_min=pitch_bounds[0], pitch_max=pitch_bounds[1],
                stable_frames=turn_stable_frames,
                settle_s=turn_settle_seconds,
                edge_fraction=turn_edge_fraction,
                invert_horizontal=invert_horizontal
            ) if body_enabled else HeadOnlyPlanner(
                pitch_min=pitch_bounds[0], pitch_max=pitch_bounds[1],
                invert_horizontal=invert_horizontal)),
            enable_actuation=head_enabled,
        )
        self._feedback = Esp32FeedbackPoller(self.sink, self.controller)
        self.head_enabled = head_enabled
        self.body_enabled = body_enabled
        self.pitch_bounds = pitch_bounds
        self.power_declared = power_declared
        self._last_event = "REAL_UART_READY_HEAD_DISABLED" if not head_enabled else "REAL_HEAD_TEST_READY"
        self._uart_poll_error: Optional[str] = None
        self._arm_wait_since: Optional[float] = None
        self._last_arm_error: Optional[str] = None
        self._last_led_color: Optional[str] = None
        self._led_sent_at = 0.0
        self._search_direction: Optional[str] = None
        self._search_started_at = 0.0
        self._search_result = "IDLE"

    def _search_candidate(self, detection, rtt_ms: float, now: float) -> Optional[str]:
        """Only a genuinely empty, sequential frame after a side-edge lock."""
        if (not self.body_enabled or not self.gesture_control_enabled
                or self.gateway.status().safety.mode != Mode.AUTO
                or self.gateway.firmware_safety_status() != "ARMED"
                or self.presenter.state != "TRACKING"
                or self.presenter._locked is None
                or self.presenter._frame_time is None
                or now - self.presenter._frame_time > .45
                or detection.error or detection.protocol_version != 2
                or type(detection.frame_seq) is not int
                or detection.frame_seq != self.presenter._last_seq + 1
                or detection.people != []
                or not math.isfinite(rtt_ms) or not 0 <= rtt_ms <= 500
                or self.presenter._frame_size != (detection.frame_width,
                                                  detection.frame_height)):
            return None
        previous = getattr(self, "_last_cycle", None)
        if (previous is None or not previous.target_valid
                or previous.plan.suggested_body_turn not in ("TRACK_LEFT", "TRACK_RIGHT")
                or abs(previous.plan.yaw_target_deg) < 2.5):
            return None
        width = detection.frame_width
        if abs(self.presenter._locked.cx - width / 2) < width * self.turn_edge_fraction:
            return None
        telemetry = self.controller.telemetry.snapshot(now=now)
        if (not telemetry.head_fresh or not telemetry.body_idle_confirmed
                or self.controller._pending_turn is not None
                or self.controller.planner.motion_pending):
            return None
        return previous.plan.suggested_body_turn

    def publish_preview(self, frame, detection) -> None:
        """Render a target-to-center vector, action zone and last seen bearing."""
        import cv2
        now = time.monotonic()
        with self._guard:
            view = self.presenter.display(now=now)
            selected = view.get("selected_index") if view["lock_state"] == "TRACKING" else None
            boxes = list(self.presenter.candidates)
            state = view["lock_state"]
            follow = getattr(self, "_last_cycle", None)
            drive_mode = self.gateway.status().safety.mode
            last_aim = getattr(self, "_last_camera_aim", None)
            last_seen_at = getattr(self, "_last_camera_aim_at", 0.)
        img = frame.copy()
        height, width = img.shape[:2]
        cx, cy = width // 2, height // 2
        # Visible boundary for preferring small head motions over body turns.
        turn_threshold_px = int(width * self.turn_edge_fraction)
        left = max(0, cx - turn_threshold_px)
        right = min(width - 1, cx + turn_threshold_px)
        cv2.line(img, (left, 0), (left, height), (130, 140, 160), 1, cv2.LINE_AA)
        cv2.line(img, (right, 0), (right, height), (130, 140, 160), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 20, (245, 245, 245), 2, cv2.LINE_AA)
        cv2.line(img, (cx-24, cy), (cx+24, cy), (245, 245, 245), 2)
        cv2.line(img, (cx, cy-24), (cx, cy+24), (245, 245, 245), 2)
        cv2.putText(img, "CAMERA CENTER", (max(2, cx-72), min(height-4, cy+38)),
                    cv2.FONT_HERSHEY_SIMPLEX, .46, (245,245,245), 1, cv2.LINE_AA)
        for idx, box in enumerate(boxes):
            active = idx == selected
            gesturing = idx in detection.gesture_indices
            color = (80, 222, 102) if active else ((0, 220, 255) if gesturing else (140, 146, 165))
            cv2.rectangle(img, (box.x1, box.y1), (box.x2, box.y2), color, 3 if active else 1)
            label = ("LOCKED" if active else "HANDS UP" if gesturing else "OTHER") + f" {idx+1}"
            cv2.putText(img, label, (max(1, box.x1), max(18, box.y1-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2, cv2.LINE_AA)
        if selected is not None and 0 <= selected < len(boxes):
            box = boxes[selected]
            aim_x = round(box.cx)
            aim_y = round(box.y1 + .22 * box.height)
            source = "UPPER BODY"
            if selected < len(detection.aim_points):
                aim = detection.aim_points[selected]
                if (type(aim.get("x")) is int and type(aim.get("y")) is int
                        and box.x1 <= aim["x"] <= box.x2
                        and box.y1 <= aim["y"] <= box.y2):
                    aim_x, aim_y = aim["x"], aim["y"]
                    source = str(aim.get("source", "FACE")).upper()
            dx, dy = aim_x - cx, aim_y - cy
            with self._guard:
                self._last_camera_aim = (aim_x, aim_y)
                self._last_camera_aim_at = now
            cv2.circle(img, (aim_x, aim_y), 9, (25, 172, 255), 3, cv2.LINE_AA)
            cv2.arrowedLine(img, (aim_x, aim_y), (cx, cy),
                            (35, 185, 255), 2, cv2.LINE_AA, tipLength=.09)
            # Display what the controller can actually do, not simply a
            # geometric zone. SAFE must never be labelled as moving.
            if self.gateway.safety.recovering:
                zone = "SEARCH: REACQUIRE 3 FRAMES"
            elif drive_mode != Mode.AUTO:
                zone = "SAFE / NO MOTION"
            elif follow is not None and follow.body_sent:
                zone = "BODY TURN SENT"
            elif follow is not None and follow.plan.reason == "WAIT_ESP32_IDLE_CONFIRMATION":
                zone = "BODY TURN IN PROGRESS"
            elif (abs(dx) >= turn_threshold_px and follow is not None
                  and abs(follow.plan.yaw_target_deg) >= 2.5):
                zone = "BODY TURN PREPARING"
            elif abs(dx) < width*.035 and abs(dy) < height*.06:
                zone = "CENTERED"
            else:
                zone = "HEAD AIM"
            screen_side = "RIGHT" if dx > 0 else "LEFT" if dx < 0 else "CENTER"
            heading = f"{zone} | {source} | SCREEN {screen_side} | dx:{dx:+d}px dy:{dy:+d}px"
        elif state in ("LOST", "TEMPORARILY_LOST") and last_aim is not None and now-last_seen_at < 3.:
            lx, ly = last_aim
            direction = "RIGHT" if lx > cx else "LEFT" if lx < cx else "CENTER"
            heading = (f"SEARCHING {self._search_direction} / 1 TURN MAX - LAST SEEN {direction}"
                       if self.gateway.safety.recovering else
                       f"TARGET LOST / STOP - LAST SEEN {direction}")
            cv2.circle(img, (lx, ly), 13, (40, 65, 250), 2, cv2.LINE_AA)
            cv2.arrowedLine(img, (lx, ly), (cx, cy), (35,185,255), 2,
                            cv2.LINE_AA, tipLength=.09)
        else:
            heading = "WAIT: HANDS OVER HEAD / NO TARGET"
        # Always annotate the decision at the top of the camera image.
        strip = img.copy()
        cv2.rectangle(strip, (0, 0), (width, 33), (12, 19, 31), -1)
        cv2.addWeighted(strip, .70, img, .30, 0, img)
        cv2.putText(img, heading, (9, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    max(.35, min(.58, width / 1200.)), (235,245,255), 2, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            with self._guard:
                self._preview_jpeg = encoded.tobytes()
                self._preview_resolution = f"{width}x{height}"

    def get_preview(self) -> Optional[bytes]:
        with self._guard:
            return self._preview_jpeg

    def set_camera_state(self, active: bool, connected: bool) -> None:
        with self._guard:
            super().set_camera_state(active, connected)
            if not connected:
                self._preview_jpeg = None
                self._preview_resolution = None
                # New PC connection must not inherit old operator gesture
                # authorization. An operator must re-enable standby.
                self.gesture_control_enabled = False
                self._gesture_pending_event = 0
                self._search_direction = None
                self._search_result = "LINK_LOST"

    def _update_status_led(self) -> None:
        """Synchronize the Pi gesture-wait LED lease; firmware owns WHITE.

        The watchdog-backed ESP32 updates actual WHITE/RED state independently.
        No LED command can arm a motor. A lost UART stops yellow lease renewal.
        """
        now = time.monotonic()
        mode = self.gateway.status().safety.mode
        firmware = self.gateway.firmware_safety_status()
        if mode == Mode.AUTO and firmware == "ARMED":
            self._last_led_color = "WHITE"
            return
        desired = ("YELLOW" if self.gesture_control_enabled and
                   mode == Mode.SAFE and firmware == "DISARMED"
                   else "RED")
        if (desired != self._last_led_color or
                (desired == "YELLOW" and now - self._led_sent_at >= .35)):
            self.sink.send("RGB_" + desired)
            self._last_led_color = desired
            self._led_sent_at = now

    def _poll_loop(self) -> None:
        """Real UART reader: persist errors beyond the next presenter event."""
        while not self._exit.wait(0.025):
            try:
                with self._guard:
                    self._feedback.poll()
                    # The Pi watchdog may have timed out the search without
                    # any further camera frames. Never silently reacquire.
                    gs = self.gateway.status().safety
                    if (gs.reason == "SEARCH_TIMEOUT"
                            and self._search_direction is not None):
                        self.presenter.clear("SEARCH_FAILED_NEW_GESTURE_REQUIRED")
                        self._search_result = "TIMED_OUT"
                        self._search_direction = None
                    self._update_status_led()
                    if self._reset_requested:
                        self.gateway.reset_emergency_stop()
                        if self.gateway.status().safety.mode == Mode.SAFE:
                            self._reset_requested = False
                            self._last_event = "REAL_ESTOP_RESET_TO_SAFE"
                    if self._arm_wait_since is not None:
                        state = self.gateway.status().safety.mode
                        if state == Mode.AUTO:
                            self._arm_wait_since = None
                            self._last_arm_error = None
                        elif not self.gateway.firmware_arm_pending("AUTO"):
                            self._last_arm_error = (
                                "AUTO_ARM_NOT_COMPLETED_BEFORE_ESTOP" if state == Mode.ESTOP
                                else "AUTO_ARM_NOT_ACKNOWLEDGED_OR_CANCELED"
                            )
                            self._arm_wait_since = None
            except Exception as exc:
                with self._guard:
                    self._uart_poll_error = type(exc).__name__
                    self.gateway.emergency_stop()
                    self._last_event = "REAL_UART_POLL_ERROR_" + type(exc).__name__
                return

    def status(self) -> dict:
        with self._guard:
            # Bypass IntegratedSimulationRuntime.status(): avoid misleading VIRTUAL UI.
            status = OperatorRuntime.status(self)
            presenter = self.presenter.display(now=time.monotonic())
            status.update(presenter)
            esp32 = self.controller.telemetry.snapshot(now=time.monotonic())
            status["esp32_head_feedback"] = (
                {"yaw_deg": esp32.head_yaw_deg, "pitch_deg": esp32.head_pitch_deg,
                 "fresh": esp32.head_fresh}
            )
            status["esp32_walking_feedback"] = {
                "state": esp32.walking_state, "busy": esp32.busy,
                "idle_confirmed": esp32.body_idle_confirmed
            }
            follow = getattr(self, "_last_cycle", None)
            status["follow_debug"] = (
                {"reason": follow.plan.reason,
                 "head_sent": follow.head_sent,
                 "body_sent": follow.body_sent,
                 "bearing_deg": round(follow.plan.body_bearing_deg, 1),
                 "body_command": follow.plan.body_command,
                 "yaw_target_deg": round(follow.plan.yaw_target_deg, 1)}
                if follow is not None else None
            )
            status["camera_resolution"] = self._preview_resolution
            status["gesture_control_enabled"] = self.gesture_control_enabled
            status["search_state"] = ("SEARCHING" if self.gateway.safety.recovering
                                      else self._search_result)
            status["search_direction"] = self._search_direction
            status["search_remaining_s"] = (
                round(max(0.0, self.gateway.safety._recovery_deadline -
                          time.monotonic()), 2)
                if self.gateway.safety.recovering else 0.0
            )
            status["search_reacquire_frames"] = self.gateway.safety._recovery_confirmed
            status["search_max_turns"] = 1
            status["led_indicated_color"] = (
                "WHITE" if status["mode"] == "AUTO" and status["fw_safety"] == "ARMED"
                else "YELLOW" if self.gesture_control_enabled and
                     status["mode"] == "SAFE" and status["fw_safety"] == "DISARMED"
                else "RED"
            )
            status["led_note"] = "ESP32 LED GPIO25/26/27; actual light not electrically sensed"
            status["horizontal_direction"] = "REVERSED_FOR_OPERATOR" if self.invert_horizontal else "LEGACY"
            status["simulation"] = False
            status["uart_poll_error"] = self._uart_poll_error
            status["uart_last_safety_event"] = self.sink.last_safety_event
            status["uart_last_servo_diagnostic"] = self.sink.last_servo_diagnostic
            status["arm_error"] = self._last_arm_error
            status["fault_cause"] = (
                "UART_POLL_" + self._uart_poll_error if self._uart_poll_error else
                self.sink.last_safety_event if status["mode"] == "ESTOP"
                    and self.sink.last_safety_event else
                "ESP32_REPORTED_LATCHED" if status["fw_safety"] == "LATCHED" else
                "SERIAL_TRANSPORT_FAULT" if status["transport_fault"] else
                ("ESTOP_UNKNOWN__" + status["stop_reason"]) if
                    status["mode"] == "ESTOP" else None
            )
            status["simulated_esp32"] = False
            status["head_only_test"] = self.head_enabled and not self.body_enabled
            status["body_motion_permitted"] = self.body_enabled
            status["pitch_tracking_enabled"] = self.pitch_bounds != (0.0, 0.0)
            status["actuation_scope"] = (
                "HEAD_YAW_3DEG_AND_FEEDBACK_GATED_BODY_TURNS" if self.body_enabled
                else "HEAD_YAW_ONLY_3DEG" if self.head_enabled
                else "MONITOR_ONLY"
            )
            status["servo_power_declared"] = self.power_declared
            status["motor_status"] = (
                "ON申告・首Yaw±3°/左右旋回（動作完了待ち）"
                if self.body_enabled else
                "ON申告・首Yaw±3°のみ" if self.head_enabled else
                "モーター指令無効（通電状態は未測定）"
            )
            status["auto_ready"] = bool(
                status["auto_ready"] and self.head_enabled
                and presenter["presenter_selected"]
                and presenter["lock_state"] == "TRACKING"
                and status["mode"] == Mode.SAFE.value
                and status["fw_safety"] == "DISARMED"
                and not self.gateway.firmware_arm_pending("AUTO")
                and not status["transport_fault"]
            )
            return status

    def process_detection(self, detection, rtt_ms: float) -> None:
        with self._guard:
            now = time.monotonic()
            was_searching = self.gateway.safety.recovering
            proposal = None if was_searching else self._search_candidate(
                detection, rtt_ms, now)
            if proposal and self.gateway.safety.begin_recovery(now=now):
                self._search_direction = proposal
                self._search_started_at = now
                self._search_result = "SEARCHING"
            super().process_detection(detection, rtt_ms)
            if self.gateway.safety.recovering:
                if self.presenter.state not in ("TEMPORARILY_LOST", "TRACKING"):
                    self.gateway.request_motion("STOP", source="AUTO")
                    self._search_direction = None
                    self._search_result = "REJECTED_AMBIGUOUS_TARGET"
                elif proposal is not None:
                    # The person was visible on the preceding valid frame,
                    # but this frame contains nobody. One micro-turn only.
                    result = self.gateway.request_recovery_turn(proposal)
                    if result.approved:
                        self.controller._pending_turn = proposal
                        self.controller.planner.motion_pending = True
                        self._last_event = "SEARCH_ONE_MICRO_TURN_" + proposal
                    else:
                        self.gateway.request_motion("STOP", source="AUTO")
                        self._search_direction = None
                        self._search_result = "SEARCH_TURN_BLOCKED_" + result.reason
            elif was_searching and self.gateway.status().safety.mode == Mode.AUTO:
                self._search_result = "REACQUIRED_3_FRAMES"
                self._search_direction = None
            elif was_searching:
                self._search_result = "SEARCH_STOPPED"
                self._search_direction = None
            current_event = self.presenter.gesture_event_id
            if current_event > self._gesture_handled_event and current_event != self._gesture_pending_event:
                self._gesture_pending_event = current_event
                self._gesture_pending_since = time.monotonic()
            if self._gesture_pending_event:
                ready = self.status()
                if (not self.gesture_control_enabled or
                        ready["lock_state"] != "TRACKING" or
                        time.monotonic() - self._gesture_pending_since > 3.0):
                    self._gesture_handled_event = self._gesture_pending_event
                    self._gesture_pending_event = 0
                elif ready["auto_ready"]:
                    event = self._gesture_pending_event
                    self._gesture_pending_event = 0
                    self._gesture_handled_event = event
                    result = self.action("auto")
                    self._last_event = ("GESTURE_AUTO_ARM_REQUESTED" if result.get("ok")
                                        else "GESTURE_ARM_BLOCKED_" + str(result.get("error")))

    def action(self, action: str) -> dict:
        with self._guard:
            if action == "gesture_on":
                if not self.head_enabled or self.gateway.status().safety.mode != Mode.SAFE:
                    return {"ok": False, "error": "GESTURE_STANDBY_REQUIRES_SAFE"}
                self.gesture_control_enabled = True
                self.presenter.clear("WAIT_BOTH_HANDS_OVER_HEAD")
                self._gesture_handled_event = self.presenter.gesture_event_id
                self._gesture_pending_event = 0
                return {"ok": True, "gesture_control_enabled": True}
            if action == "gesture_off":
                self.gesture_control_enabled = False
                self._gesture_pending_event = 0
                self._search_direction = None
                self._search_result = "CANCELED"
                super().action("stop")
                self.presenter.clear("GESTURE_STANDBY_DISABLED")
                return {"ok": True, "gesture_control_enabled": False}
            if action in ("stop", "estop"):
                self.gesture_control_enabled = False
                self._gesture_pending_event = 0
                self._search_direction = None
                self._search_result = "CANCELED"
            if action == "manual":
                return {"ok": False, "error": "BODY_MOVEMENT_NOT_APPROVED"}
            if action in {"auto", "reset"} and not self.head_enabled:
                return {"ok": False, "error": "SERIAL_PROBE_ONLY"}
            if action == "auto":
                state = self.gateway.status().safety.mode
                fw_state = self.gateway.firmware_safety_status()
                if state == Mode.ESTOP:
                    return {"ok": False, "error": "ESTOP_LATCHED_RESET_REQUIRED"}
                if state == Mode.AUTO:
                    # CRITICAL: duplicate SAFETY_ARM while already ARMED is
                    # rejected by firmware and can lead to ESTOP. Idempotent.
                    return {"ok": True, "already_active": True}
                if self.gateway.firmware_arm_pending("AUTO"):
                    # Repeated UI clicks during the async handshake must not
                    # send another SAFETY_ARM or reset the deadline.
                    return {"ok": True, "pending": True}
                if state != Mode.SAFE or fw_state != "DISARMED":
                    return {"ok": False, "error": "ESP32_NOT_DISARMED_OR_NOT_READY"}
            response = super().action(action)
            if action == "auto" and response.get("pending"):
                self._arm_wait_since = time.monotonic()
                self._last_arm_error = None
            return response

    def manual(self, direction: str) -> dict:
        if direction == "stop":
            return self.action("stop")
        return {"ok": False, "error": "BODY_MOVEMENT_NOT_APPROVED"}


def main(argv: Optional[list[str]] = None) -> None:
    ap = argparse.ArgumentParser(
        description="Real ESP32 integrated control; safe default UART diagnostics only")
    ap.add_argument("--serial-port", required=True)
    ap.add_argument("--pc-host", default=None,
                    help="Optional Pi camera via TCP/SSH to RTX4060 PC")
    ap.add_argument("--pc-port", type=int, default=8765)
    ap.add_argument("--camera-index", type=int, default=0)
    ap.add_argument("--camera-width", type=int, default=640)
    ap.add_argument("--camera-height", type=int, default=480)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--port", type=int, default=8766)
    power_group = ap.add_mutually_exclusive_group(required=True)
    power_group.add_argument("--servo-power-on", action="store_true",
                             help="Servo V+ is ON (operator declaration, NOT telemetry)")
    power_group.add_argument("--servo-power-off-confirmed", action="store_true",
                             help="Legacy monitor-only path; servo V+ physically OFF")
    ap.add_argument("--enable-head-test", action="store_true",
                    help="Only under secured/calibrated bench test; max yaw +/-3deg")
    ap.add_argument("--enable-body-turns", action="store_true",
                    help="Repeated body turns gated by fresh ESP32 idle feedback")
    ap.add_argument("--body-turn-calibrated", action="store_true")
    ap.add_argument("--pitch-calibrated", action="store_true",
                    help="Confirm pitch has been physically calibrated")
    ap.add_argument("--pitch-min-deg", type=float, default=None)
    ap.add_argument("--pitch-max-deg", type=float, default=None)
    ap.add_argument("--turn-stable-frames", type=int, default=4)
    ap.add_argument("--turn-settle-seconds", type=float, default=1.0)
    ap.add_argument("--turn-edge-fraction", type=float, default=.15)
    ap.add_argument("--legacy-horizontal-direction", action="store_true",
                    help="Restore previous right/left tracking sign if physical test requires")
    ap.add_argument("--bench-secured", action="store_true")
    ap.add_argument("--neutral-calibrated", action="store_true")
    ap.add_argument("--physical-cutoff-tested", action="store_true")
    args = ap.parse_args(argv)
    if args.enable_body_turns and not args.enable_head_test:
        ap.error("Body turns require --enable-head-test")
    if args.enable_body_turns and not args.body_turn_calibrated:
        ap.error("Body turns require verified body-turn calibration")
    if (args.pitch_min_deg is not None or args.pitch_max_deg is not None
            or args.pitch_calibrated):
        if not args.enable_head_test or not args.pitch_calibrated:
            ap.error("pitch requires explicit enable-head-test and pitch calibration")
        if args.pitch_min_deg is None or args.pitch_max_deg is None:
            ap.error("pitch requires both physically calibrated min/max angles")
        if not (-12.0 <= args.pitch_min_deg <= 0.0 <=
                args.pitch_max_deg <= 12.0 and
                args.pitch_min_deg < args.pitch_max_deg):
            ap.error("pitch min/max outside firmware range or do not contain neutral")
    if args.turn_stable_frames < 2 or not 0 <= args.turn_settle_seconds <= 10:
        ap.error("invalid turn stability/settling settings")
    if not 0.05 <= args.turn_edge_fraction <= .45:
        ap.error("turn edge fraction must be in .05 .. .45")
    if args.enable_head_test and not args.servo_power_on:
        ap.error("Head motor test requires explicitly declared servo V+ ON")
    if args.enable_head_test and not (
        args.bench_secured and args.neutral_calibrated
        and args.physical_cutoff_tested
    ):
        ap.error("Head actuation requires physical bench, neutral and cutoff verification")
    if args.port != 8766:
        ap.error("Use the established loopback UI port 8766")
    if args.pc_host and args.pc_host != "127.0.0.1":
        ap.error("Use the existing SSH loopback tunnel, not a public TCP detector")
    if not args.serial_port.startswith("/dev/serial/by-id/"):
        ap.error("Use stable Pi /dev/serial/by-id/... device, not guessed ttyUSB")

    runtime = LockedRealRuntime(
        args.serial_port,
        head_enabled=args.enable_head_test,
        body_enabled=args.enable_body_turns,
        power_declared="ON_OPERATOR_DECLARED" if args.servo_power_on else "OFF_OPERATOR_DECLARED",
        pitch_bounds=(args.pitch_min_deg or 0., args.pitch_max_deg or 0.),
        turn_stable_frames=args.turn_stable_frames,
        turn_settle_seconds=args.turn_settle_seconds,
        turn_edge_fraction=args.turn_edge_fraction,
        invert_horizontal=not args.legacy_horizontal_direction,
    )
    worker = None
    server = None
    try:
        runtime.start()
        if args.pc_host:
            worker = CameraInferenceWorker(
                runtime, args.pc_host, port=args.pc_port,
                camera_index=args.camera_index,
                camera_width=args.camera_width,
                camera_height=args.camera_height,
                jpeg_quality=args.jpeg_quality,
            )
            worker.start()
        token = secrets.token_urlsafe(32)
        server = OperatorServer(("127.0.0.1", args.port), runtime, token)
        print("REAL ESP32 UART. Real servo is NOT physically verified by software.")
        print("Mode:", (
            "SUPERVISED YAW +/-3deg AND REPEATED FEEDBACK-GATED BODY TURNS"
            if args.enable_body_turns else
            "SUPERVISED YAW +/-3deg" if args.enable_head_test else
            "SERIAL PROBE; NO ARM"
        ))
        print("Servo V+:", runtime.power_declared, "(operator-declared, not electrically sensed)")
        print("STOP/ESTOP do not remove power; keep physical cutoff accessible.")
        print(f"Operator URL: http://127.0.0.1:{args.port}/")
        print("Operator token:", token)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n[Real ESP32 integration] Shutdown.")
    finally:
        if worker is not None:
            worker.close()
            worker.join(timeout=1.)
        if server is not None:
            server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
