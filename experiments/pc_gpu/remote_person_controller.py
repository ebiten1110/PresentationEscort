"""Combine Pi ESP32 telemetry with V8.8.3 planning and motion safety.

Hardware output is disabled by default.  External code must explicitly
construct with enable_actuation=True and separately arm AUTO.
Use ONLY when the same process owns the sole ESP32 serial port.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from detection_protocol import DetectionResult
from esp32_telemetry import Esp32Telemetry
from motion_gateway import PiMotionGateway
from person_target_adapter import detection_to_person_target
from safety_manager import Mode
from v883_person_dual_loop import FollowPlan, V883PersonDualLoop


@dataclass(frozen=True)
class FollowCycle:
    mode: Mode
    target_valid: bool
    plan: FollowPlan
    head_sent: bool
    body_sent: bool
    reason: str


class RemotePersonController:
    def __init__(
        self,
        gateway: PiMotionGateway,
        *,
        telemetry: Optional[Esp32Telemetry] = None,
        planner: Optional[V883PersonDualLoop] = None,
        enable_actuation: bool = False,
    ) -> None:
        self.gateway = gateway
        self.telemetry = telemetry if telemetry is not None else Esp32Telemetry()
        self.planner = planner if planner is not None else V883PersonDualLoop()
        self.enable_actuation = bool(enable_actuation)
        self._pending_turn: Optional[str] = None

    def feed_esp32(self, serial_bytes: bytes, *, now: Optional[float] = None) -> None:
        timestamp = time.monotonic() if now is None else now
        self.telemetry.feed(serial_bytes, now=timestamp)
        status = self.telemetry.snapshot(now=timestamp)
        if status.motion_completion and self._pending_turn is not None:
            completed = self.telemetry.consume_completed_turn(now=timestamp)
            if completed == self._pending_turn:
                self.planner.motion_completed(timestamp, completed)
                self._pending_turn = None

    def process(
        self,
        result: DetectionResult,
        *,
        rtt_ms: float,
        now: Optional[float] = None,
    ) -> FollowCycle:
        current = time.monotonic() if now is None else now
        target = detection_to_person_target(result)
        valid = target is not None and not bool(result.error)
        self.gateway.observe_detection(valid, rtt_ms, now=current)
        mode = self.gateway.status(now=current).safety.mode
        if not valid:
            self.planner.cancel_unsent_proposals()
            self._pending_turn = None
            return FollowCycle(mode, False, self.planner.target_lost(), False, False, "NO_PERSON_OR_INVALID")

        status = self.telemetry.snapshot(now=current)
        plan = self.planner.step(
            target, now=current,
            head_yaw_feedback_deg=status.head_yaw_deg,
            head_feedback_fresh=status.head_fresh,
            body_idle=status.body_idle_confirmed,
        )
        if not self.enable_actuation:
            self.planner.cancel_unsent_proposals()
            return FollowCycle(mode, True, plan, False, False, "DRY_RUN_ONLY")
        if mode != Mode.AUTO:
            self.planner.cancel_unsent_proposals()
            return FollowCycle(mode, True, plan, False, False, "NOT_ARMED")
        head_sent = False
        body_sent = False
        if plan.head_pose_command is not None and status.head_fresh:
            head = self.gateway.request_head_pose(
                plan.yaw_target_deg, plan.pitch_target_deg, now=current,
            )
            head_sent = head.approved
        if not head_sent and plan.head_pose_command is not None:
            self.planner.last_pose_send_at = None
            self.planner.last_sent_yaw = None
            self.planner.last_sent_pitch = None
        if plan.body_command is not None:
            body = self.gateway.request_motion(plan.body_command, source="AUTO", now=current)
            body_sent = body.approved
            if body_sent:
                self._pending_turn = plan.body_command
            else:
                self.planner.cancel_unsent_proposals()
        return FollowCycle(mode, True, plan, head_sent, body_sent, "CONTROL_APPLIED")
