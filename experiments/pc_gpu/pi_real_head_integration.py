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

from diagnostic_capture import DiagnosticCapture
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
from v883_person_dual_loop import (
    CAMERA_X_TO_ROBOT_YAW_DIRECTION, HEAD_PITCH_IMAGE_DIRECTION,
    HEAD_X_DEADZONE_PX, HEAD_Y_DEADZONE_PX,
    V883PersonDualLoop, VIRTUAL_FRAME_HEIGHT, VIRTUAL_FRAME_WIDTH,
)

_POSE = re.compile(r"^HEAD_POSE_SET:([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?)$")


class _SmoothHeadAiming:
    """Feedback-referenced proportional aim; no per-camera-frame angle winding.

    ESP32 reports its internal interpolated servo angle (NOT a shaft encoder).
    Never sum camera-pixel errors into the target angle on every GPU frame.
    Bound the correction relative to the latest fresh MCU angle, then slew
    each setpoint; higher camera FPS must not increase the commanded speed.
    """

    _send_interval_s = 0.05  # Up to 20 Hz, when the camera actually delivers it.
    _aim_tau_s = 0.08       # Low delay, time-based; independent of camera FPS.
    _yaw_gain_deg_per_px = 0.012
    _pitch_gain_deg_per_px = 0.010
    _correction_limit_deg = 0.60
    _minimum_distance_gain = 0.35
    # A small, verified head correction is preferable to saturating the neck.
    # The body takes larger image errors; actual metric distance is unknown.
    _adaptive_near_yaw_limit_deg = 1.5
    _adaptive_far_yaw_limit_deg = 0.75

    @staticmethod
    def _validate_pitch_speed(speed_deg_s: float) -> None:
        if not math.isfinite(speed_deg_s) or not 0.5 <= speed_deg_s <= 10.0:
            raise ValueError("pitch speed must be between 0.5 and 10 deg/s")

    def _init_head_aiming(self, speed_deg_s: float) -> None:
        if not math.isfinite(speed_deg_s) or not 0.5 <= speed_deg_s <= 10.0:
            raise ValueError("head speed must be between 0.5 and 10 deg/s")
        self.head_speed_deg_s = speed_deg_s
        self.pitch_speed_deg_s = speed_deg_s
        self.invert_vertical = False
        self._smoothed_error_x: Optional[float] = None
        self._smoothed_error_y: Optional[float] = None
        self._aim_last_at: Optional[float] = None
        self._head_pitch_feedback_deg: Optional[float] = None
        # Optical reference only: the gesture frame establishes a relative
        # image-Y baseline. It does NOT calibrate the mechanical servo origin.
        self.relative_pitch_reference = False
        self.pitch_reference_y: Optional[float] = None
        self.distance_adaptive_head = False
        self.responsive_follow = False
        self.distance_gain = 1.0
        self._distance_gain_at: Optional[float] = None
        # Physical runtime explicitly gates aiming until the first AUTO
        # session is armed. Standalone planner tests keep the opt-in default.
        self._head_tracking_allowed = True
        self._head_tracking_guard_frames = 0
        self._head_tracking_guard_active = False

    def set_actuation_gate(self, allowed: bool) -> None:
        """Hold reported servo pose while SAFE or briefly after reacquisition."""
        if not allowed:
            self._head_tracking_allowed = False
            self._head_tracking_guard_frames = 3
            return
        if not self._head_tracking_allowed:
            self._head_tracking_guard_frames = 3
        self._head_tracking_allowed = True

    def _adaptive_yaw_limit(self) -> float:
        if not self.distance_adaptive_head:
            return 3.0
        return max(self._adaptive_far_yaw_limit_deg,
                   min(self._adaptive_near_yaw_limit_deg,
                       self._adaptive_near_yaw_limit_deg * self.distance_gain))

    def _reference_pitch_target(self, target):
        if not self.relative_pitch_reference or self.pitch_min == self.pitch_max:
            return target
        normalized = target.error_y / float(target.frame_height)
        if self.pitch_reference_y is None:
            self.pitch_reference_y = normalized
        return replace(target, error_y=round(
            (normalized - self.pitch_reference_y) * target.frame_height))

    def _filter_aim(self, target, *, now: float):
        if self._smoothed_error_x is None:
            self._smoothed_error_x = float(target.error_x)
            self._smoothed_error_y = float(target.error_y)
        else:
            elapsed = max(0.0, now - self._aim_last_at)
            tau = 0.045 if self.responsive_follow else self._aim_tau_s
            alpha = 1.0 - math.exp(-min(elapsed, 0.5) / tau)
            self._smoothed_error_x += alpha * (target.error_x - self._smoothed_error_x)
            self._smoothed_error_y += alpha * (target.error_y - self._smoothed_error_y)
        self._aim_last_at = now
        if self.distance_adaptive_head:
            # A 2-D box provides an apparent-size proxy, NOT metric depth.
            # Use both dimensions so an unusually wide pose does not alone
            # cause aggressive head movement. The baseline is frame-relative.
            fraction = math.sqrt(
                min(1.0, target.box_width / target.frame_width) *
                min(1.0, target.box_height / target.frame_height))
            raw_gain = max(self._minimum_distance_gain,
                           min(1.0, fraction / .38))
            if self._distance_gain_at is None:
                self.distance_gain = raw_gain
            else:
                dt = max(0.0, min(.5, now - self._distance_gain_at))
                alpha = 1.0 - math.exp(-dt / .3)
                self.distance_gain += alpha * (raw_gain - self.distance_gain)
            self._distance_gain_at = now
        return replace(target, error_x=round(self._smoothed_error_x),
                       error_y=round(self._smoothed_error_y))

    def target_lost(self):
        # Never steer on remembered / interpolated pixels after a dropout.
        self._smoothed_error_x = None
        self._smoothed_error_y = None
        self._aim_last_at = None
        self.pitch_reference_y = None
        self._distance_gain_at = None
        self.distance_gain = 1.0
        return super().target_lost()

    @staticmethod
    def _proportional_delta(error: int, deadzone: int, gain: float) -> float:
        if abs(error) <= deadzone:
            return 0.0
        signed = error - math.copysign(deadzone, error)
        return max(-_SmoothHeadAiming._correction_limit_deg,
                   min(_SmoothHeadAiming._correction_limit_deg, signed * gain))

    def _limit_head_command(self, plan, *, now: float, previous: tuple,
                            target, yaw_feedback, pitch_feedback,
                            feedback_fresh: bool):
        previous_yaw, previous_pitch, previous_at, previous_sent_yaw, previous_sent_pitch = previous
        if pitch_feedback is None and self.pitch_min == self.pitch_max == 0.0:
            # Yaw-only tests have no active pitch control. Do not require a
            # pitch sample solely to hold this axis at its locked zero.
            pitch_feedback = 0.0
        valid_yaw = (feedback_fresh and yaw_feedback is not None
                     and math.isfinite(yaw_feedback) and -3.0 <= yaw_feedback <= 3.0)
        valid_pitch = (feedback_fresh and pitch_feedback is not None
                       and math.isfinite(pitch_feedback)
                       and self.pitch_min <= pitch_feedback <= self.pitch_max)
        # Stale/invalid ESP32 angle means STOP proposing new servo positions.
        # In particular never fall back to adding one pixel correction per FPS.
        if not valid_yaw or not valid_pitch:
            self.yaw_target_deg, self.pitch_target_deg = previous_yaw, previous_pitch
            self.last_pose_send_at = previous_at
            self.last_sent_yaw, self.last_sent_pitch = previous_sent_yaw, previous_sent_pitch
            return replace(plan, yaw_target_deg=previous_yaw,
                           pitch_target_deg=previous_pitch, head_pose_command=None)

        if previous_at is None and previous_sent_yaw is None:
            # First actual ESP32 feedback defines the servo origin of this
            # session, not an assumed software zero from an earlier lock.
            previous_yaw, previous_pitch = yaw_feedback, pitch_feedback
            self.yaw_target_deg, self.pitch_target_deg = previous_yaw, previous_pitch
        virtual_x = round(target.error_x * VIRTUAL_FRAME_WIDTH / target.frame_width)
        virtual_y = round(target.error_y * VIRTUAL_FRAME_HEIGHT / target.frame_height)
        # A face within ~8% of the image center is adequately centered.
        # Tiny detector jitter must not wind the neck back and forth.
        yaw_deadzone = (max(HEAD_X_DEADZONE_PX, round(VIRTUAL_FRAME_WIDTH * .08))
                        if self.distance_adaptive_head else HEAD_X_DEADZONE_PX)
        yaw_delta = self._proportional_delta(
            virtual_x, yaw_deadzone, self._yaw_gain_deg_per_px)
        # Scale AFTER limiting the delta. Otherwise a large faraway pixel
        # error hits the same 0.6-degree cap and defeats distance damping.
        yaw_delta *= self.distance_gain if self.distance_adaptive_head else 1.0
        pitch_delta = (self._proportional_delta(
            virtual_y, HEAD_Y_DEADZONE_PX, self._pitch_gain_deg_per_px)
            if plan.pitch_tracking_enabled else 0.0)
        # Use a smaller neck envelope when the person is far away. At the
        # edge, transfer the remaining angular bearing to the body instead
        # of repeatedly pushing the camera out of frame.
        yaw_soft_limit = self._adaptive_yaw_limit()
        yaw_requested = yaw_delta * CAMERA_X_TO_ROBOT_YAW_DIRECTION
        if (abs(yaw_feedback) > yaw_soft_limit and
                yaw_requested * yaw_feedback >= 0):
            # A prior lock may have left the actual neck at +/-3 deg.
            # Never jump back toward 0 on a centered face, or push farther
            # outward. A visible edge can instead request a body turn.
            desired_yaw = yaw_feedback
        else:
            desired_yaw = max(-yaw_soft_limit, min(yaw_soft_limit,
                yaw_feedback + yaw_requested))
        # If the image is mirrored for display, *only* yaw sign is swapped
        # before _filter_aim, as in the previously tested implementation.
        if self.relative_pitch_reference and self.pitch_reference_y is not None:
            # Absolute, neutral-referenced setpoint. When the face is always
            # above image center, feedback-integrating against that impossible
            # center leaves Pitch pinned at -2 forever. Relative displacement
            # makes motion observable without widening the calibrated range.
            desired_pitch = (virtual_y * .035 * HEAD_PITCH_IMAGE_DIRECTION *
                             (-1.0 if self.invert_vertical else 1.0))
        else:
            desired_pitch = (pitch_feedback + pitch_delta * HEAD_PITCH_IMAGE_DIRECTION *
                             (-1.0 if self.invert_vertical else 1.0))
        desired_pitch = max(self.pitch_min, min(self.pitch_max, desired_pitch))
        self._head_tracking_guard_active = (
            not self._head_tracking_allowed or self._head_tracking_guard_frames > 0)
        if self._head_tracking_guard_active:
            # While SAFE, do not prepare commands that will jump on ARM.
            # After AUTO resumes, accept three full images before moving.
            if self._head_tracking_allowed:
                self._head_tracking_guard_frames -= 1
            self.yaw_target_deg, self.pitch_target_deg = yaw_feedback, pitch_feedback
            self.last_pose_send_at = previous_at
            self.last_sent_yaw, self.last_sent_pitch = previous_sent_yaw, previous_sent_pitch
            return replace(plan, yaw_target_deg=yaw_feedback,
                           pitch_target_deg=pitch_feedback,
                           head_pose_command=None)
        elapsed = (0.12 if previous_at is None
                   else max(0.0, now - previous_at))
        if previous_at is not None and elapsed + 1e-9 < self._send_interval_s:
            yaw, pitch, command = previous_yaw, previous_pitch, None
        else:
            # A camera stall must not turn the next packet into a huge jump.
            max_elapsed = min(elapsed, 0.20)
            yaw_step = self.head_speed_deg_s * max_elapsed
            pitch_step = self.pitch_speed_deg_s * max_elapsed
            yaw = round(previous_yaw + max(-yaw_step, min(yaw_step,
                                                         desired_yaw - previous_yaw)), 1)
            pitch = round(previous_pitch + max(-pitch_step, min(pitch_step,
                                                             desired_pitch - previous_pitch)), 1)
            changed = (previous_at is None or
                       abs(yaw - previous_yaw) >= 0.1 or
                       abs(pitch - previous_pitch) >= 0.1)
            command = f"HEAD_POSE_SET:{yaw:.1f},{pitch:.1f}" if changed else None
        self.yaw_target_deg = yaw
        self.pitch_target_deg = pitch
        # The V8.8.3 base planner records a send before this rate limiter.
        # Restore its bookkeeping unless the limited command really exists.
        self.last_pose_send_at = now if command else previous_at
        self.last_sent_yaw = yaw if command else previous_sent_yaw
        self.last_sent_pitch = pitch if command else previous_sent_pitch
        return replace(plan, yaw_target_deg=yaw, pitch_target_deg=pitch,
                       head_pose_command=command)


class HeadOnlyPlanner(_SmoothHeadAiming, V883PersonDualLoop):
    """Limit the planner's internal state, optionally using calibrated pitch."""

    def __init__(self, *, pitch_min: float = 0.0, pitch_max: float = 0.0,
                 invert_horizontal: bool = False,
                 invert_vertical: bool = False,
                 relative_pitch_reference: bool = False,
                 distance_adaptive_head: bool = False,
                 responsive_follow: bool = False,
                 head_speed_deg_s: float = 3.0,
                 pitch_speed_deg_s: Optional[float] = None):
        super().__init__()
        self._init_head_aiming(head_speed_deg_s)
        if pitch_speed_deg_s is not None:
            self._validate_pitch_speed(pitch_speed_deg_s)
            self.pitch_speed_deg_s = pitch_speed_deg_s
        self.pitch_min = pitch_min
        self.pitch_max = pitch_max
        self.invert_horizontal = invert_horizontal
        self.invert_vertical = invert_vertical
        self.relative_pitch_reference = relative_pitch_reference
        self.distance_adaptive_head = distance_adaptive_head
        self.responsive_follow = responsive_follow

    def step(self, target, *, now: float, **kwargs):
        if self.invert_horizontal:
            target = replace(target, error_x=-target.error_x)
        target = self._reference_pitch_target(target)
        target = self._filter_aim(target, now=now)
        pitch_feedback = kwargs.pop("head_pitch_feedback_deg", self._head_pitch_feedback_deg)
        previous = (self.yaw_target_deg, self.pitch_target_deg,
                    self.last_pose_send_at, self.last_sent_yaw, self.last_sent_pitch)
        plan = super().step(target, now=now, **kwargs)
        plan = self._limit_head_command(
            plan, now=now, previous=previous, target=target,
            yaw_feedback=kwargs.get("head_yaw_feedback_deg"),
            pitch_feedback=pitch_feedback,
            feedback_fresh=kwargs.get("head_feedback_fresh", False))
        self.body_candidate = None
        self.body_candidate_count = 0
        self.motion_pending = False
        return replace(plan, body_command=None, suggested_body_turn=None,
                       yaw_target_deg=self.yaw_target_deg,
                       pitch_target_deg=self.pitch_target_deg)


class HeadBodyTurnPlanner(_SmoothHeadAiming, V883PersonDualLoop):
    """Measured-feedback turns with visual confirmation, no session turn quota."""

    def __init__(self, *, pitch_min: float = 0.0, pitch_max: float = 0.0,
                 stable_frames: int = 4, settle_s: float = 1.0,
                 edge_fraction: float = 0.15,
                 invert_horizontal: bool = False,
                 invert_vertical: bool = False,
                 relative_pitch_reference: bool = False,
                 reverse_body_turn_direction: bool = False,
                 distance_adaptive_head: bool = False,
                 responsive_follow: bool = False,
                 head_speed_deg_s: float = 3.0,
                 pitch_speed_deg_s: Optional[float] = None):
        super().__init__()
        self._init_head_aiming(head_speed_deg_s)
        if pitch_speed_deg_s is not None:
            self._validate_pitch_speed(pitch_speed_deg_s)
            self.pitch_speed_deg_s = pitch_speed_deg_s
        self.pitch_min = pitch_min
        self.pitch_max = pitch_max
        self.stable_frames = stable_frames
        self.settle_s = settle_s
        self.edge_fraction = edge_fraction
        self.invert_horizontal = invert_horizontal
        self.invert_vertical = invert_vertical
        self.relative_pitch_reference = relative_pitch_reference
        self.reverse_body_turn_direction = reverse_body_turn_direction
        self.distance_adaptive_head = distance_adaptive_head
        self.responsive_follow = responsive_follow
        self._turn_proposals = 0
        self._proposal_direction = None

    def motion_completed(self, now: float, command: str) -> None:
        super().motion_completed(now, command)
        self._turn_proposals = 0
        self._proposal_direction = None

    def step(self, target, *, now, **kwargs):
        if self.invert_horizontal:
            target = replace(target, error_x=-target.error_x)
        target = self._reference_pitch_target(target)
        target = self._filter_aim(target, now=now)
        pitch_feedback = kwargs.pop("head_pitch_feedback_deg", self._head_pitch_feedback_deg)
        previous = (self.yaw_target_deg, self.pitch_target_deg,
                    self.last_pose_send_at, self.last_sent_yaw, self.last_sent_pitch)
        plan = super().step(target, now=now, **kwargs)
        plan = self._limit_head_command(
            plan, now=now, previous=previous, target=target,
            yaw_feedback=kwargs.get("head_yaw_feedback_deg"),
            pitch_feedback=pitch_feedback,
            feedback_fresh=kwargs.get("head_feedback_fresh", False))
        if self._head_tracking_guard_active:
            self.motion_pending = False
            self.body_candidate = None
            self.body_candidate_count = 0
            self._turn_proposals = 0
            self._proposal_direction = None
            return replace(plan, yaw_target_deg=self.yaw_target_deg,
                           pitch_target_deg=self.pitch_target_deg,
                           body_command=None, suggested_body_turn=None,
                           reason="TRACKING_STABILIZING")
        if self.reverse_body_turn_direction:
            # Reverse ONLY body pivot commands. Camera image and head-yaw
            # calibration are independent of the walking mechanism's sign.
            def swap(cmd):
                return ("TRACK_RIGHT" if cmd == "TRACK_LEFT" else
                        "TRACK_LEFT" if cmd == "TRACK_RIGHT" else cmd)
            plan = replace(plan, body_command=swap(plan.body_command),
                           suggested_body_turn=swap(plan.suggested_body_turn))
        body_command = plan.body_command
        reason = plan.reason
        if body_command is not None:
            normalized_error = abs(target.error_x) / float(target.frame_width)
            settled = (self.last_turn_completed_at is None or
                       now - self.last_turn_completed_at >= self.settle_s)
            # Head servo handles smaller movements. Only after yaw reaches
            # its calibrated boundary and the face approaches the edge do
            # we request a feedback-gated body turn while vision is still valid.
            feedback_yaw = kwargs.get("head_yaw_feedback_deg")
            yaw_limit = self._adaptive_yaw_limit()
            head_near_limit = (abs(self.yaw_target_deg) >= .85 * yaw_limit
                               and kwargs.get("head_feedback_fresh", False)
                               and feedback_yaw is not None
                               and abs(feedback_yaw) >= .70 * yaw_limit
                               and self.yaw_target_deg * feedback_yaw > 0)
            # An exceptionally large, still-visible side displacement can
            # escape the frame before the neck reaches +/-2.5 degrees.
            # Permit a body turn slightly earlier only with corroborating
            # fresh measured MCU angle, consistent turn direction, stable
            # frame votes, and all existing gateway/firmware motion gates.
            extreme_visible_edge = (
                normalized_error >= max(.30, self.edge_fraction * 2.)
                and abs(self.yaw_target_deg) >= min(1.5, .60 * yaw_limit)
                and kwargs.get("head_feedback_fresh", False)
                and feedback_yaw is not None
                and abs(feedback_yaw) >= min(1.2, .50 * yaw_limit)
                and self.yaw_target_deg * feedback_yaw > 0
            )
            if (normalized_error < self.edge_fraction or
                    not (head_near_limit or extreme_visible_edge) or not settled):
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
                       body_command=body_command,
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
        self.last_head_pwm: dict[int, str] = {}

    def send(self, command: str) -> None:
        cmd = command.strip().upper()
        if cmd.startswith("RGB_"):
            # The historical Phase10 transport is intentionally immutable.
            # Its motor allowlist does not include the two new *indicators*.
            # Send ONLY RED/YELLOW through the same guarded serial handle.
            # WHITE remains exclusively owned by ESP32's ARMED watchdog.
            if cmd not in ("RGB_RED", "RGB_YELLOW"):
                raise ValueError("Pi may only indicate RED or YELLOW")
            port = getattr(self._inner, "_serial", None)
            lock = getattr(self._inner, "_lock", None)
            if port is not None and lock is not None:
                payload = (cmd + "\n").encode("ascii")
                with lock:
                    if not bool(getattr(port, "is_open", False)):
                        raise ConnectionError("serial port not open")
                    if port.write(payload) != len(payload):
                        raise IOError("incomplete LED indication write")
                return
            # VirtualEsp32 tests have no pyserial handle.
            self._inner.send(cmd)
            return
        if cmd.startswith("TRACK_") or cmd in {"LEFT", "RIGHT", "FORWARD", "BACKWARD", "STAND"}:
            if not (self.body_enabled and cmd in {"TRACK_LEFT", "TRACK_RIGHT"}):
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
                    pwm = re.match(r"^\[ServoDiag\] PWM ch=(2|3)\s", line)
                    if pwm:
                        self.last_head_pwm[int(pwm[1])] = line[:180]
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
        # SAFE/STOP frames still pass through the V8.8.3 planner to display
        # targets, but their unsent pose must NEVER become the next AUTO
        # starting angle. Resume from fresh MCU-reported angles instead.
        current = time.monotonic() if now is None else now
        if isinstance(self.planner, _SmoothHeadAiming):
            feedback = self.telemetry.snapshot(now=current)
            self.planner._head_pitch_feedback_deg = (
                feedback.head_pitch_deg if feedback.head_fresh else None)
        if isinstance(self.planner, _SmoothHeadAiming):
            feedback = self.telemetry.snapshot(now=current)
            if feedback.head_fresh and feedback.head_yaw_deg is not None:
                yaw = feedback.head_yaw_deg
                pitch = feedback.head_pitch_deg
                if (not -3.0 <= yaw <= 3.0 or pitch is None or
                        not self.planner.pitch_min <= pitch <= self.planner.pitch_max):
                    # An uncalibrated, parked servo angle is diagnostic data,
                    # not a reason to remotely ESTOP a monitor-only session.
                    # Never send SAFETY_ESTOP from passive camera processing.
                    # In AUTO, however, unexpected feedback must still latch
                    # immediately, even after a prior valid head command.
                    if (self.enable_actuation and
                            self.gateway.status(now=current).safety.mode == Mode.AUTO):
                        self.gateway.emergency_stop()
                        return FollowCycle(Mode.ESTOP, False, self.planner.target_lost(),
                                           False, False, "HEAD_FEEDBACK_OUTSIDE_APPROVED_RANGE")
                elif self.planner.last_pose_send_at is None:
                    self.planner.yaw_target_deg = yaw
                    self.planner.pitch_target_deg = pitch
        if not self.gateway.safety.recovering:
            return super().process(result, rtt_ms=rtt_ms, now=current)
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
                 invert_horizontal: bool = False,
                 invert_vertical: bool = False,
                 relative_pitch_reference: bool = False,
                 reverse_body_turn_direction: bool = False,
                 distance_adaptive_head: bool = False,
                 responsive_follow: bool = False,
                 auto_recalibrate_brief_loss: bool = False,
                 head_speed_deg_s: float = 2.0,
                 pitch_speed_deg_s: Optional[float] = None,
                 capture_diagnostics: bool = False) -> None:
        if body_enabled and not head_enabled:
            raise ValueError("body turns require head control opt-in")
        super().__init__()
        # Gesture selects the actual person; never silently select a passerby.
        self.presenter = PresenterLock(gesture_enabled=True,
                                       gesture_confirm_frames=3,
                                       transient_recovery_s=1.5)
        self.gesture_control_enabled = False
        self._gesture_handled_event = 0
        self._gesture_pending_event = 0
        self._gesture_pending_since = 0.0
        self.invert_horizontal = invert_horizontal
        self.invert_vertical = invert_vertical
        self.relative_pitch_reference = relative_pitch_reference
        self.reverse_body_turn_direction = reverse_body_turn_direction
        self.distance_adaptive_head = distance_adaptive_head
        self.responsive_follow = responsive_follow
        self.auto_recalibrate_brief_loss = auto_recalibrate_brief_loss
        self.head_speed_deg_s = head_speed_deg_s
        self.pitch_speed_deg_s = (head_speed_deg_s if pitch_speed_deg_s is None
                                  else pitch_speed_deg_s)
        self._preview_jpeg: Optional[bytes] = None
        self._preview_at: Optional[float] = None
        self._preview_resolution: Optional[str] = None
        self._capture_diagnostics: dict = {}
        self._capture_timing: dict = {}
        self._diagnostics = DiagnosticCapture() if capture_diagnostics else None
        self._last_detection_at: Optional[float] = None
        self._camera_interval_s: Optional[float] = None
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
                invert_horizontal=invert_horizontal,
                invert_vertical=invert_vertical,
                relative_pitch_reference=relative_pitch_reference,
                reverse_body_turn_direction=reverse_body_turn_direction,
                distance_adaptive_head=distance_adaptive_head,
                responsive_follow=responsive_follow,
                head_speed_deg_s=head_speed_deg_s,
                pitch_speed_deg_s=pitch_speed_deg_s,
            ) if body_enabled else HeadOnlyPlanner(
                pitch_min=pitch_bounds[0], pitch_max=pitch_bounds[1],
                invert_horizontal=invert_horizontal,
                invert_vertical=invert_vertical,
                relative_pitch_reference=relative_pitch_reference,
                distance_adaptive_head=distance_adaptive_head,
                responsive_follow=responsive_follow,
                head_speed_deg_s=head_speed_deg_s,
                pitch_speed_deg_s=pitch_speed_deg_s)),
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

    def _head_feedback_error(self) -> Optional[str]:
        """Report calibration readiness without changing ESP32 safety state."""
        feedback = self.controller.telemetry.snapshot(now=time.monotonic())
        if not feedback.head_fresh or feedback.head_yaw_deg is None or feedback.head_pitch_deg is None:
            return "HEAD_FEEDBACK_NOT_FRESH"
        if not -3.0 <= feedback.head_yaw_deg <= 3.0:
            return "YAW_OUTSIDE_APPROVED_RANGE"
        if not self.pitch_bounds[0] <= feedback.head_pitch_deg <= self.pitch_bounds[1]:
            return "PITCH_OUTSIDE_CALIBRATED_RANGE"
        return None

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
        yaw_limit = self.controller.planner._adaptive_yaw_limit()
        if (previous is None or not previous.target_valid
                or previous.plan.suggested_body_turn not in ("TRACK_LEFT", "TRACK_RIGHT")
                or abs(previous.plan.yaw_target_deg) < .85 * yaw_limit):
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

    def _can_hold_brief_visual_loss(self, detection, rtt_ms: float, now: float) -> bool:
        """Retain an existing ARM briefly, sending NO blind motor commands.

        No fresh person: permit at most 1.5 s of stationary search, with
        multi-frame visual reconfirmation. STOP and ESP32 loss still disarm.
        """
        if (not self.auto_recalibrate_brief_loss or
                not self.gesture_control_enabled or
                self.gateway.status().safety.mode != Mode.AUTO or
                self.gateway.firmware_safety_status() != "ARMED" or
                self.presenter.state != "TRACKING" or
                self.presenter._locked is None or
                len(self.presenter.candidates) != 1 or
                self.presenter._frame_time is None or
                not 0 <= now - self.presenter._frame_time <= .45 or
                detection.error or detection.protocol_version != 2 or
                type(detection.frame_seq) is not int or
                detection.frame_seq != self.presenter._last_seq + 1 or
                detection.people != [] or
                self.presenter._frame_size != (detection.frame_width,
                                               detection.frame_height) or
                not math.isfinite(rtt_ms) or not 0 <= rtt_ms <= 500):
            return False
        previous = getattr(self, "_last_cycle", None)
        if previous is None or not previous.target_valid:
            return False
        telemetry = self.controller.telemetry.snapshot(now=now)
        # A moving leg turn is NOT a reason to tear down the tracking lease:
        # no additional head/body command is permitted during recovery.
        # This passive hold works even while the previous turn finishes.
        return telemetry.head_fresh and not self.gateway.status().transport_fault

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
        pitch_reference = self.controller.planner.pitch_reference_y
        relative_pitch = self.relative_pitch_reference and pitch_reference is not None
        pitch_line_y = (max(0, min(height - 1, round(cy + pitch_reference * height)))
                        if relative_pitch else cy)
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
        if relative_pitch:
            cv2.line(img, (0, pitch_line_y), (width - 1, pitch_line_y),
                     (245, 145, 40), 1, cv2.LINE_AA)
            cv2.putText(img, "PITCH BASELINE", (8, max(16, pitch_line_y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .43, (245, 145, 40), 1,
                        cv2.LINE_AA)
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
            dx, dy = aim_x - cx, aim_y - pitch_line_y
            with self._guard:
                self._last_camera_aim = (aim_x, aim_y)
                self._last_camera_aim_at = now
            cv2.circle(img, (aim_x, aim_y), 9, (25, 172, 255), 3, cv2.LINE_AA)
            cv2.arrowedLine(img, (aim_x, aim_y), (cx, pitch_line_y),
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
                  and abs(follow.plan.yaw_target_deg) >= .85 *
                      self.controller.planner._adaptive_yaw_limit()):
                zone = "BODY TURN PREPARING"
            elif abs(dx) < width*.035 and abs(dy) < height*.06:
                zone = "CENTERED"
            else:
                zone = "HEAD AIM"
            screen_side = "RIGHT" if dx > 0 else "LEFT" if dx < 0 else "CENTER"
            heading = f"{zone} | {source} | SCREEN {screen_side} | dx:{dx:+d}px dy:{dy:+d}px"
            if follow is not None and drive_mode == Mode.AUTO:
                heading += (f" | YAW {follow.plan.yaw_target_deg:+.1f}"
                            f" PITCH {follow.plan.pitch_target_deg:+.1f}")
        elif state in ("LOST", "TEMPORARILY_LOST") and last_aim is not None and now-last_seen_at < 3.:
            lx, ly = last_aim
            direction = "RIGHT" if lx > cx else "LEFT" if lx < cx else "CENTER"
            heading = (f"SEARCHING {self._search_direction} - LAST SEEN {direction}"
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
            jpeg = encoded.tobytes()
            with self._guard:
                self._preview_jpeg = jpeg
                self._preview_at = now
                self._preview_resolution = f"{width}x{height}"
            if self._diagnostics is not None:
                self._diagnostics.frame(jpeg, at=now)

    def get_preview(self) -> Optional[bytes]:
        with self._guard:
            return self._preview_jpeg

    def set_camera_state(self, active: bool, connected: bool) -> None:
        with self._guard:
            super().set_camera_state(active, connected)
            if not connected:
                self._last_detection_at = None
                self._camera_interval_s = None
                self._preview_jpeg = None
                self._preview_at = None
                self._preview_resolution = None
                # New PC connection must not inherit old operator gesture
                # authorization. An operator must re-enable standby.
                self.gesture_control_enabled = False
                self._gesture_pending_event = 0
                self._search_direction = None
                self._search_result = "LINK_LOST"

    def set_capture_diagnostics(self, info: dict) -> None:
        with self._guard:
            self._capture_diagnostics = dict(info)

    def set_capture_timing(self, timing: dict) -> None:
        with self._guard:
            self._capture_timing = dict(timing)

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
                 "yaw_target_deg": round(follow.plan.yaw_target_deg, 1),
                 "pitch_target_deg": round(follow.plan.pitch_target_deg, 1),
                 "pitch_aim_valid": follow.plan.pitch_tracking_enabled,
                 "pitch_at_limit": (self.pitch_bounds != (0.0, 0.0) and
                                    (abs(follow.plan.pitch_target_deg - self.pitch_bounds[0]) < .11
                                     or abs(follow.plan.pitch_target_deg - self.pitch_bounds[1]) < .11))}
                if follow is not None else None
            )
            status["camera_resolution"] = self._preview_resolution
            status["camera_capture_mode"] = dict(self._capture_diagnostics)
            status["camera_stage_timing_ms"] = dict(self._capture_timing)
            status["preview_available"] = self._preview_jpeg is not None
            status["preview_age_ms"] = (
                round((time.monotonic() - self._preview_at) * 1000)
                if self._preview_at is not None else None)
            status["diagnostic_capture_enabled"] = self._diagnostics is not None
            status["camera_effective_fps"] = (
                round(1.0 / self._camera_interval_s, 1)
                if self._camera_interval_s is not None and self._camera_interval_s > 0
                else None)
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
            if self._search_direction == "HOLD":
                status["search_max_turns"] = 0
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
            status["uart_head_pwm"] = {
                "yaw_ch2": self.sink.last_head_pwm.get(2),
                "pitch_ch3": self.sink.last_head_pwm.get(3),
            }
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
            status["relative_pitch_reference"] = self.relative_pitch_reference
            status["reverse_body_turn_direction"] = self.reverse_body_turn_direction
            status["distance_adaptive_head"] = self.distance_adaptive_head
            status["apparent_distance_gain"] = round(
                self.controller.planner.distance_gain, 3)
            status["responsive_follow"] = self.responsive_follow
            status["auto_recalibrate_brief_loss"] = self.auto_recalibrate_brief_loss
            status["pitch_reference_y_normalized"] = (
                round(self.controller.planner.pitch_reference_y, 4)
                if self.controller.planner.pitch_reference_y is not None else None)
            status["vertical_direction"] = ("REVERSED" if self.invert_vertical
                                            else "LEGACY")
            status["pitch_speed_limit_deg_s"] = self.pitch_speed_deg_s
            status["head_feedback_error"] = self._head_feedback_error()
            status["head_pitch_approved_range"] = list(self.pitch_bounds)
            status["head_speed_limit_deg_s"] = self.head_speed_deg_s
            head_scope = "HEAD_YAW_3DEG"
            if self.pitch_bounds != (0.0, 0.0):
                head_scope += "_PITCH_CALIBRATED"
            status["actuation_scope"] = (
                head_scope + "_AND_FEEDBACK_GATED_BODY_TURNS" if self.body_enabled
                else head_scope if self.head_enabled and self.pitch_bounds != (0.0, 0.0)
                else "HEAD_YAW_ONLY_3DEG" if self.head_enabled
                else "MONITOR_ONLY"
            )
            pitch_label = (f" / Pitch {self.pitch_bounds[0]:g}～{self.pitch_bounds[1]:g}°"
                           if self.pitch_bounds != (0.0, 0.0) else "")
            status["servo_power_declared"] = self.power_declared
            status["motor_status"] = (
                "ON申告・首Yaw±3°" + pitch_label + " / 左右旋回（動作完了待ち）" +
                (" / 旋回方向反転" if self.reverse_body_turn_direction else "")
                if self.body_enabled else
                "ON申告・首Yaw±3°" + pitch_label if self.head_enabled else
                "モーター指令無効（通電状態は未測定）"
            )
            status["auto_ready"] = bool(
                status["auto_ready"] and self.head_enabled
                and presenter["presenter_selected"]
                and presenter["lock_state"] == "TRACKING"
                and status["mode"] == Mode.SAFE.value
                and status["fw_safety"] == "DISARMED"
                # Firmware ARM is not a head move. Missing head telemetry
                # already blocks individual pose sends; a *known* angle
                # outside the calibrated range must block ARM earlier.
                and status["head_feedback_error"] not in (
                    "YAW_OUTSIDE_APPROVED_RANGE", "PITCH_OUTSIDE_CALIBRATED_RANGE")
                and not self.gateway.firmware_arm_pending("AUTO")
                and not status["transport_fault"]
            )
            return status

    def process_detection(self, detection, rtt_ms: float) -> None:
        with self._guard:
            now = time.monotonic()
            if self._last_detection_at is not None:
                elapsed = now - self._last_detection_at
                if 0.01 <= elapsed <= 1.0:
                    self._camera_interval_s = (
                        elapsed if self._camera_interval_s is None
                        else 0.8 * self._camera_interval_s + 0.2 * elapsed)
            self._last_detection_at = now
            was_searching = self.gateway.safety.recovering
            proposal = None if was_searching else self._search_candidate(
                detection, rtt_ms, now)
            hold = (not was_searching and proposal is None and
                    self._can_hold_brief_visual_loss(detection, rtt_ms, now))
            if (proposal or hold) and self.gateway.safety.begin_recovery(now=now):
                self._search_direction = proposal if proposal else "HOLD"
                self._search_started_at = now
                self._search_result = "SEARCHING" if proposal else "HOLDING_FOR_TARGET"
            # Crucial: only AUTO with a proven TRACKING lock may aim.
            # On the three recovering frames the head stays stationary even
            # though the detector already reports a recognizable face.
            aiming_allowed = (self.gateway.status().safety.mode == Mode.AUTO
                and self.gateway.firmware_safety_status() == "ARMED"
                and self.presenter.state == "TRACKING"
                and not self.gateway.safety.recovering
                and not was_searching)
            self.controller.planner.set_actuation_gate(aiming_allowed)
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
                # Geometry re-lock is permitted ONLY within this surviving
                # AUTO lease. No new gesture/ARM event is ever manufactured.
                self.presenter.confirm_bounded_reacquisition()
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
            if self._diagnostics is not None:
                state = self.status()
                self._diagnostics.event({
                    "type": "inference_cycle",
                    "frame_seq": detection.frame_seq,
                    "protocol_version": detection.protocol_version,
                    "size": [detection.frame_width, detection.frame_height],
                    "gpu_inference_ms": detection.inference_ms,
                    "pi_roundtrip_ms": rtt_ms,
                    "person_found": detection.found,
                    "person_candidates": (detection.people or [])[:12],
                    "gesture_indices": (detection.gesture_indices or [])[:12],
                    "aim_points": (detection.aim_points or [])[:12],
                    "aim_source": detection.aim_source,
                    "pc_error": detection.error,
                    "presenter": {key: state.get(key) for key in (
                        "lock_state", "lock_reason", "selected_index", "presenter_selected")},
                    "safety": {key: state.get(key) for key in (
                        "mode", "reason", "fw_safety", "stop_reason", "fault_cause",
                        "uart_last_safety_event", "uart_poll_error",
                        "head_feedback_error", "head_pitch_approved_range")},
                    "head_feedback": state.get("esp32_head_feedback"),
                    "follow": state.get("follow_debug"),
                })

    def get_diagnostic_bundle(self) -> Optional[bytes]:
        """Authenticated download only; never opens a port or sends motion."""
        if self._diagnostics is None:
            return None
        with self._guard:
            state = self.status()
            public_keys = (
                "mode", "reason", "fw_safety", "stop_reason", "fault_cause",
                "uart_last_safety_event", "uart_poll_error", "transport_fault",
                "camera_active", "pc_connected", "camera_resolution",
                "preview_available", "preview_age_ms", "camera_effective_fps",
                "inference_ms", "rtt_ms", "lock_state", "lock_reason",
                "presenter_selected", "gesture_control_enabled",
                "esp32_head_feedback", "esp32_walking_feedback", "follow_debug",
                "search_state", "head_speed_limit_deg_s", "pitch_tracking_enabled",
                "head_feedback_error", "head_pitch_approved_range",
                "pitch_speed_limit_deg_s", "vertical_direction",
                "relative_pitch_reference", "pitch_reference_y_normalized",
                "reverse_body_turn_direction",
                "distance_adaptive_head", "apparent_distance_gain",
                "responsive_follow", "auto_recalibrate_brief_loss",
                "uart_head_pwm", "uart_last_servo_diagnostic",
                "camera_capture_mode", "camera_stage_timing_ms",
            )
            snapshot = {key: state.get(key) for key in public_keys}
        return self._diagnostics.export(status=snapshot)

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
                feedback_error = self._head_feedback_error()
                if feedback_error in ("YAW_OUTSIDE_APPROVED_RANGE",
                                      "PITCH_OUTSIDE_CALIBRATED_RANGE"):
                    return {"ok": False, "error": "HEAD_FEEDBACK_NOT_CALIBRATED",
                            "detail": feedback_error}
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
    ap.add_argument("--camera-fps", type=int, default=30,
                    help="Requested camera capture FPS (5..60); actual rate may be lower")
    ap.add_argument("--camera-mjpeg", action="store_true",
                    help="Try UVC MJPEG transport; measure actual FourCC/FPS in diagnostics")
    ap.add_argument("--capture-diagnostics", action="store_true",
                    help="Opt in to bounded RAM image+decision capture and authenticated ZIP download")
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--preview-every-n-frames", type=int, default=1,
                    help="Display/diagnostic JPEG only every N inference frames; all tracking frames still processed")
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
    ap.add_argument("--reverse-vertical-direction", action="store_true",
                    help="Reverse vertical servo aim direction after verifying actual motion; calibrated limits remain enforced")
    ap.add_argument("--relative-pitch-reference", action="store_true",
                    help="Use the first locked face height as the image-Y reference, keeping existing calibrated servo limits")
    ap.add_argument("--reverse-body-turn-direction", action="store_true",
                    help="Swap physical TRACK_LEFT/RIGHT only, after observing reversed floor rotation")
    ap.add_argument("--distance-adaptive-head", action="store_true",
                    help="Dampen head yaw correction when the person's image box is small; not meter-based depth")
    ap.add_argument("--responsive-follow", action="store_true",
                    help="Use a lower-latency visual aim filter; physical slew caps still apply")
    ap.add_argument("--auto-recalibrate-brief-loss", action="store_true",
                    help="Within an already ARMED session, hold still during a bounded brief dropout and visually reacquire")
    ap.add_argument("--turn-stable-frames", type=int, default=4)
    ap.add_argument("--turn-settle-seconds", type=float, default=1.0)
    ap.add_argument("--turn-edge-fraction", type=float, default=.15)
    direction_group = ap.add_mutually_exclusive_group()
    direction_group.add_argument("--legacy-horizontal-direction", action="store_true",
                                 help="V8.8.3 verified left/right sign (now the default)")
    direction_group.add_argument("--reverse-horizontal-direction", action="store_true",
                                 help="Reverse yaw sign only after a secured physical test")
    ap.add_argument("--head-speed-deg-per-second", type=float, default=2.0,
                    help="Maximum change in Pi head setpoint, default 2 deg/s")
    ap.add_argument("--pitch-speed-deg-per-second", type=float, default=None,
                    help="Optional independent vertical slew limit (0.5..10 deg/s); defaults to head speed")
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
    if not math.isfinite(args.head_speed_deg_per_second) or not (
        0.5 <= args.head_speed_deg_per_second <= 10.0
    ):
        ap.error("head speed must be between 0.5 and 10 deg/s")
    if args.pitch_speed_deg_per_second is not None and (
        not math.isfinite(args.pitch_speed_deg_per_second) or not
        0.5 <= args.pitch_speed_deg_per_second <= 10.0
    ):
        ap.error("pitch speed must be between 0.5 and 10 deg/s")
    if (args.pitch_speed_deg_per_second is not None or
            args.reverse_vertical_direction or
            args.relative_pitch_reference) and not args.pitch_calibrated:
        ap.error("independent pitch control requires physical pitch calibration")
    if not 5 <= args.camera_fps <= 60:
        ap.error("camera fps request must be 5..60")
    if not 1 <= args.preview_every_n_frames <= 10:
        ap.error("preview decimation must be 1..10")
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
        invert_horizontal=args.reverse_horizontal_direction,
        invert_vertical=args.reverse_vertical_direction,
        relative_pitch_reference=args.relative_pitch_reference,
        reverse_body_turn_direction=args.reverse_body_turn_direction,
        distance_adaptive_head=args.distance_adaptive_head,
        responsive_follow=args.responsive_follow,
        auto_recalibrate_brief_loss=args.auto_recalibrate_brief_loss,
        head_speed_deg_s=args.head_speed_deg_per_second,
        pitch_speed_deg_s=args.pitch_speed_deg_per_second,
        capture_diagnostics=args.capture_diagnostics,
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
                camera_fps=args.camera_fps,
                jpeg_quality=args.jpeg_quality,
                camera_mjpeg=args.camera_mjpeg,
                preview_every_n_frames=args.preview_every_n_frames,
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
