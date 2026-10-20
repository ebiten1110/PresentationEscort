"""V8.8.3-calibrated dual-loop planning for remote YOLO person detections.

No OpenCV, serial, network access, or motor side effects.
The math and thresholds are transferred from
raspberry_pi/follow_dual_loop_v8_8.py (verified physical baseline).
DO NOT reverse the original motion direction mappings.

This is a planning adapter, NOT the original V8.8.3 runtime and not an
ESP32 motion driver.  A real controller must use fresh ESP32 head yaw
feedback, ESP32 busy/idle feedback, and PiMotionGateway's safety gate.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Optional

from person_target_adapter import PersonTarget

# Calibrated V8.8.3 constants. The parity tests flag configuration drift.
VIRTUAL_FRAME_WIDTH = 640
VIRTUAL_FRAME_HEIGHT = 480
HEAD_X_DEADZONE_PX = 14
HEAD_Y_DEADZONE_PX = 12
HEAD_YAW_GAIN_DEG_PER_PX = 0.055
HEAD_PITCH_GAIN_DEG_PER_PX = 0.045
HEAD_MAX_YAW_STEP_DEG = 2.5
HEAD_MAX_PITCH_STEP_DEG = 2.0
HEAD_YAW_MIN_DEG = -20.0
HEAD_YAW_MAX_DEG = 20.0
HEAD_PITCH_MIN_DEG = -12.0
HEAD_PITCH_MAX_DEG = 12.0
CAMERA_X_TO_ROBOT_YAW_DIRECTION = -1.0
HEAD_PITCH_IMAGE_DIRECTION = 1.0
CAMERA_HORIZONTAL_FOV_DEG = 60.0
BODY_BEARING_START_DEG = 9.0
BODY_BEARING_CONTINUE_DEG = 4.5
BODY_BEARING_STOP_DEG = 3.0
BODY_TURN_CONFIRM_FRAMES = 2
BODY_TURN_CONTINUE_CONFIRM_FRAMES = 1
BODY_RESTART_COOLDOWN_SECONDS = 0.18
BODY_TURN_CONTINUE_WINDOW_SECONDS = 2.50
BODY_NEGATIVE_BEARING_COMMAND = "TRACK_LEFT"
BODY_POSITIVE_BEARING_COMMAND = "TRACK_RIGHT"
HEAD_POSE_COMMAND_INTERVAL_SECONDS = 0.055
HEAD_COMMAND_EPSILON_DEG = 0.25


@dataclass(frozen=True)
class FollowPlan:
    target_found: bool
    yaw_target_deg: float
    pitch_target_deg: float
    residual_deg: float
    body_bearing_deg: float
    bearing_from_feedback: bool
    pitch_tracking_enabled: bool
    head_pose_command: Optional[str]
    suggested_body_turn: Optional[str]
    body_command: Optional[str]
    reason: str


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _step_from_error(
    error_px: int, deadzone: int, gain: float, max_step: float
) -> float:
    """Same deadzone-subtraction and step clipping as V8.8.3."""
    if abs(error_px) <= deadzone:
        return 0.0
    effective = (
        error_px - deadzone if error_px > 0 else error_px + deadzone
    )
    return _clamp(effective * gain, -max_step, max_step)


def _virtual_error(error_px: int, frame_size: int, virtual_size: int) -> int:
    if frame_size <= 0:
        raise ValueError("invalid image dimension")
    return int(round(error_px * virtual_size / frame_size))


class V883PersonDualLoop:
    """Stateful head/body planner with motion feedback interlock.

    Without *fresh* head-yaw feedback, body_command is always None.
    The planner emits proposed HEAD_POSE_SET values but never sends them.
    The pitch target is held if a person box occupies most of the frame,
    because a truncated full-body bbox is not a stable head location.

    Once a body command is proposed, it stays pending until an external
    controller calls motion_completed(). Busy state must be supplied by the
    ESP32 status consumer; the planner cannot infer hardware completion.
    """

    def __init__(self) -> None:
        self.yaw_target_deg = 0.0
        self.pitch_target_deg = 0.0
        self.last_pose_send_at: Optional[float] = None
        self.last_sent_yaw: Optional[float] = None
        self.last_sent_pitch: Optional[float] = None
        self.body_candidate: Optional[str] = None
        self.body_candidate_count = 0
        self.motion_pending = False
        self.last_turn_completed_at: Optional[float] = None

    def reset(self) -> None:
        """Full reset; only safe when actual body is known idle/disarmed."""
        self.__init__()

    def target_lost(self) -> FollowPlan:
        self.body_candidate = None
        self.body_candidate_count = 0
        return FollowPlan(
            target_found=False,
            yaw_target_deg=self.yaw_target_deg,
            pitch_target_deg=self.pitch_target_deg,
            residual_deg=0.0,
            body_bearing_deg=0.0,
            bearing_from_feedback=False,
            pitch_tracking_enabled=False,
            head_pose_command=None,
            suggested_body_turn=None,
            body_command=None,
            reason="NO_TARGET_STOP_REQUIRED",
        )

    def motion_completed(self, now: float, command: str) -> None:
        """Only call upon a verified ESP32 IDLE acknowledgement."""
        if not isfinite(now):
            raise ValueError("invalid time")
        if command not in ("TRACK_LEFT", "TRACK_RIGHT"):
            raise ValueError("motion command not recognized")
        self.motion_pending = False
        self.last_turn_completed_at = now
        self.body_candidate = None
        self.body_candidate_count = 0

    def step(
        self,
        target: PersonTarget,
        *,
        now: float,
        head_yaw_feedback_deg: Optional[float] = None,
        head_feedback_fresh: bool = False,
        body_idle: bool = False,
    ) -> FollowPlan:
        if not isfinite(now):
            raise ValueError("invalid time")
        if target.frame_width <= 0 or target.frame_height <= 0:
            raise ValueError("invalid frame")
        error_x = _virtual_error(
            target.error_x, target.frame_width, VIRTUAL_FRAME_WIDTH
        )
        error_y = _virtual_error(
            target.error_y, target.frame_height, VIRTUAL_FRAME_HEIGHT
        )

        yaw_step = _step_from_error(
            error_x, HEAD_X_DEADZONE_PX,
            HEAD_YAW_GAIN_DEG_PER_PX, HEAD_MAX_YAW_STEP_DEG,
        ) * CAMERA_X_TO_ROBOT_YAW_DIRECTION
        self.yaw_target_deg = _clamp(
            self.yaw_target_deg + yaw_step,
            HEAD_YAW_MIN_DEG, HEAD_YAW_MAX_DEG,
        )
        # BBox can be cropped at top/bottom, or nearly fill the frame.
        # An apparent head point then becomes unreliable for Pitch control.
        pitch_ok = (
            target.box_height < target.frame_height * 0.90
            and target.box_height > 0
        )
        if pitch_ok:
            pitch_step = _step_from_error(
                error_y, HEAD_Y_DEADZONE_PX,
                HEAD_PITCH_GAIN_DEG_PER_PX, HEAD_MAX_PITCH_STEP_DEG,
            ) * HEAD_PITCH_IMAGE_DIRECTION
            self.pitch_target_deg = _clamp(
                self.pitch_target_deg + pitch_step,
                HEAD_PITCH_MIN_DEG, HEAD_PITCH_MAX_DEG,
            )

        residual = (
            target.error_x / target.frame_width
            * CAMERA_HORIZONTAL_FOV_DEG
            * CAMERA_X_TO_ROBOT_YAW_DIRECTION
        )
        feedback_valid = (
            head_feedback_fresh
            and head_yaw_feedback_deg is not None
            and isfinite(head_yaw_feedback_deg)
            and HEAD_YAW_MIN_DEG <= head_yaw_feedback_deg <= HEAD_YAW_MAX_DEG
        )
        camera_yaw = (
            float(head_yaw_feedback_deg)
            if feedback_valid else self.yaw_target_deg
        )
        bearing = camera_yaw + residual
        suggested = (
            BODY_NEGATIVE_BEARING_COMMAND if bearing < -BODY_BEARING_STOP_DEG
            else BODY_POSITIVE_BEARING_COMMAND if bearing > BODY_BEARING_STOP_DEG
            else None
        )
        recent_turn = (
            self.last_turn_completed_at is not None
            and 0 <= now - self.last_turn_completed_at
            <= BODY_TURN_CONTINUE_WINDOW_SECONDS
        )
        required_error = (
            BODY_BEARING_CONTINUE_DEG if recent_turn
            else BODY_BEARING_START_DEG
        )
        required_frames = (
            BODY_TURN_CONTINUE_CONFIRM_FRAMES if recent_turn
            else BODY_TURN_CONFIRM_FRAMES
        )
        body_command = None
        reason = "BODY_ALIGNED_OR_WITHIN_HYSTERESIS"
        if not feedback_valid:
            self.body_candidate = None
            self.body_candidate_count = 0
            reason = "WAIT_FRESH_ESP32_HEAD_FEEDBACK"
        elif not body_idle or self.motion_pending:
            self.body_candidate = None
            self.body_candidate_count = 0
            reason = "WAIT_ESP32_IDLE_CONFIRMATION"
        elif abs(bearing) < required_error:
            self.body_candidate = None
            self.body_candidate_count = 0
        else:
            current_command = (
                BODY_NEGATIVE_BEARING_COMMAND if bearing < 0
                else BODY_POSITIVE_BEARING_COMMAND
            )
            if current_command == self.body_candidate:
                self.body_candidate_count += 1
            else:
                self.body_candidate = current_command
                self.body_candidate_count = 1
            reason = "WAIT_BODY_DIRECTION_CONFIRMATION"
            cooldown_ok = (
                self.last_turn_completed_at is None
                or now - self.last_turn_completed_at
                >= BODY_RESTART_COOLDOWN_SECONDS
            )
            if self.body_candidate_count >= required_frames and cooldown_ok:
                body_command = current_command
                self.motion_pending = True
                self.body_candidate = None
                self.body_candidate_count = 0
                reason = "BODY_TURN_PROPOSED"

        head_command = None
        changed = (
            self.last_sent_yaw is None
            or self.last_sent_pitch is None
            or abs(self.yaw_target_deg - self.last_sent_yaw)
            >= HEAD_COMMAND_EPSILON_DEG
            or abs(self.pitch_target_deg - self.last_sent_pitch)
            >= HEAD_COMMAND_EPSILON_DEG
        )
        interval_ok = (
            self.last_pose_send_at is None
            or now - self.last_pose_send_at
            >= HEAD_POSE_COMMAND_INTERVAL_SECONDS
        )
        if changed and interval_ok:
            head_command = (
                f"HEAD_POSE_SET:{self.yaw_target_deg:.1f},"
                f"{self.pitch_target_deg:.1f}"
            )
            self.last_sent_yaw = self.yaw_target_deg
            self.last_sent_pitch = self.pitch_target_deg
            self.last_pose_send_at = now

        return FollowPlan(
            target_found=True,
            yaw_target_deg=self.yaw_target_deg,
            pitch_target_deg=self.pitch_target_deg,
            residual_deg=residual,
            body_bearing_deg=bearing,
            bearing_from_feedback=bool(feedback_valid),
            pitch_tracking_enabled=pitch_ok,
            head_pose_command=head_command,
            suggested_body_turn=suggested,
            body_command=body_command,
            reason=reason,
        )
