"""Conservative Pi-side control gate. No serial I/O and no motor side effects.

This module only decides whether a command is permissible. It does not
guarantee the physical ESP32 can stop after a Pi crash or cable removal.
Hardware deployment requires a separate ESP32-side link watchdog.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class Mode(str, Enum):
    SAFE = "SAFE"
    AUTO = "AUTO"
    MANUAL = "MANUAL"
    ESTOP = "ESTOP"


@dataclass(frozen=True)
class SafetyPolicy:
    link_timeout_s: float = 0.8
    target_timeout_s: float = 0.5
    manual_deadman_s: float = 0.35
    max_rtt_ms: float = 500.0
    auto_confirm_frames: int = 3
    allow_translation: bool = False

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0
            for value in (
                self.link_timeout_s,
                self.target_timeout_s,
                self.manual_deadman_s,
                self.max_rtt_ms,
            )
        ):
            raise ValueError("timeouts and RTT limit must be positive finite numbers")
        if self.auto_confirm_frames < 1:
            raise ValueError("auto_confirm_frames must be >= 1")


@dataclass(frozen=True)
class ControlDecision:
    approved: bool
    command: str
    reason: str
    mode: Mode


@dataclass(frozen=True)
class SafetyStatus:
    mode: Mode
    reason: str
    auto_ready: bool
    confirmed_frames: int
    link_age_s: Optional[float]
    target_age_s: Optional[float]


_TURN_COMMANDS = frozenset({"TRACK_LEFT", "TRACK_RIGHT"})
_TRANSLATION_COMMANDS = frozenset({"TRACK_FORWARD", "TRACK_BACKWARD"})


class SafetyManager:
    """Fail-closed logical control gate.

    Explicit arming is required after every fault. A successful detection
    never implicitly re-enables motion. E-stop is latched. STOP is allowed
    in all modes. Call tick() independently and regularly when driving
    physical hardware; a blocking receive loop is *not* a reliable watchdog.
    """

    def __init__(self, policy: Optional[SafetyPolicy] = None) -> None:
        self.policy = policy if policy is not None else SafetyPolicy()
        self.mode = Mode.SAFE
        self.reason = "STARTUP_DISARMED"
        self.last_link_at: Optional[float] = None
        self.last_target_at: Optional[float] = None
        self.last_manual_at: Optional[float] = None
        self.confirmed_frames = 0
        # Only the explicitly opted-in real-robot runtime may open this
        # bounded search window. Normal missing detections still stop AUTO.
        self._recovery_deadline: Optional[float] = None
        self._recovery_started: Optional[float] = None
        self._recovery_turn_used = False
        self._recovery_confirmed = 0

    @property
    def recovering(self) -> bool:
        return self._recovery_deadline is not None and self.mode == Mode.AUTO

    def _clear_recovery(self) -> None:
        self._recovery_deadline = None
        self._recovery_started = None
        self._recovery_turn_used = False
        self._recovery_confirmed = 0

    def begin_recovery(self, *, now: Optional[float] = None,
                       duration_s: float = 1.5) -> bool:
        """Hold AUTO briefly for ONE search pulse, with a hard time limit.

        This must be called by a trusted local controller only after checking
        the selected person's last bearing, pose continuity, and ESP32 IDLE.
        It does not grant generic motion permission.
        """
        current = self._now(now)
        self.tick(current)
        if (not math.isfinite(duration_s) or not 0 < duration_s <= 1.5
                or self.mode != Mode.AUTO or self.recovering
                or not self._link_is_fresh(current)
                or not self._target_is_fresh(current)):
            return False
        self._recovery_started = current
        self._recovery_deadline = current + duration_s
        self._recovery_turn_used = False
        self._recovery_confirmed = 0
        self.reason = "BOUNDED_VISUAL_SEARCH"
        return True

    def authorize_recovery_turn(self, command: str, *,
                                now: Optional[float] = None) -> ControlDecision:
        """Authorize at most one micro-turn immediately after genuine loss."""
        current = self._now(now)
        self.tick(current)
        cmd = command.strip().upper()
        if (not self.recovering or self._recovery_turn_used
                or cmd not in _TURN_COMMANDS
                or self._recovery_started is None
                or current - self._recovery_started > 0.45
                or not self._link_is_fresh(current)
                or not self._target_is_fresh(current)
                or self._recovery_confirmed != 0):
            return ControlDecision(False, "STOP", "SEARCH_TURN_NOT_AUTHORIZED", self.mode)
        self._recovery_turn_used = True
        return ControlDecision(True, cmd, "ONE_BOUNDED_SEARCH_TURN", self.mode)

    @staticmethod
    def _now(now: Optional[float]) -> float:
        value = time.monotonic() if now is None else float(now)
        if not math.isfinite(value):
            raise ValueError("timestamp must be finite")
        return value

    def disarm(self, reason: str = "OPERATOR_DISARM") -> None:
        if self.mode == Mode.ESTOP:
            return
        self.mode = Mode.SAFE
        self.reason = reason
        self.last_manual_at = None
        self._clear_recovery()

    def emergency_stop(self) -> None:
        self.mode = Mode.ESTOP
        self.reason = "EMERGENCY_STOP"
        self.confirmed_frames = 0
        self.last_target_at = None
        self.last_manual_at = None
        self._clear_recovery()

    def reset_emergency_stop(self) -> None:
        """Only explicit operator acknowledgement; never automatically arm."""
        if self.mode == Mode.ESTOP:
            self.mode = Mode.SAFE
            self.reason = "ESTOP_RESET_REARM_REQUIRED"
            self.confirmed_frames = 0
            self.last_target_at = None
            self.last_link_at = None
            self._clear_recovery()

    def connection_lost(self) -> None:
        self.last_link_at = None
        self.last_target_at = None
        self.confirmed_frames = 0
        self._clear_recovery()
        # MANUAL is a separate operator channel: it must remain usable
        # when the AI/PC inference link fails. Deadman still applies.
        if self.mode == Mode.AUTO:
            self.disarm("LINK_LOST")

    def _link_is_fresh(self, current: float) -> bool:
        return (
            self.last_link_at is not None
            and 0 <= current - self.last_link_at <= self.policy.link_timeout_s
        )

    def _target_is_fresh(self, current: float) -> bool:
        return (
            self.last_target_at is not None
            and 0 <= current - self.last_target_at <= self.policy.target_timeout_s
        )

    def tick(self, now: Optional[float] = None) -> None:
        """Check liveness; must be called even when no frames arrive."""
        current = self._now(now)
        if self.mode == Mode.ESTOP:
            return
        if self.mode == Mode.AUTO and not self._link_is_fresh(current):
            self.disarm("LINK_TIMEOUT")
            self.confirmed_frames = 0
            self.last_target_at = None
            return
        if self.recovering and (self._recovery_deadline is None
                                or current >= self._recovery_deadline):
            self.confirmed_frames = 0
            self.last_target_at = None
            self.disarm("SEARCH_TIMEOUT")
            return
        if self.recovering:
            return
        if self.mode == Mode.AUTO and not self._target_is_fresh(current):
            self.confirmed_frames = 0
            self.disarm("TARGET_TIMEOUT")
        if self.mode == Mode.MANUAL and (
            self.last_manual_at is None
            or current - self.last_manual_at > self.policy.manual_deadman_s
            or current < self.last_manual_at
        ):
            self.disarm("MANUAL_DEADMAN_TIMEOUT")

    def observe_detection(
        self, found: bool, rtt_ms: float, now: Optional[float] = None
    ) -> None:
        """Observe one completed PC inference response, not a generic packet.

        Invalid/stale responses are faults; a fresh response without a
        validated person is *not* permission to continue AUTO.
        """
        current = self._now(now)
        if self.mode == Mode.ESTOP:
            return
        self.tick(current)
        if (
            not math.isfinite(rtt_ms)
            or rtt_ms < 0
            or rtt_ms > self.policy.max_rtt_ms
        ):
            self.confirmed_frames = 0
            self.last_target_at = None
            self.last_link_at = None
            # A bad AI response stops AUTO, but MANUAL is independently
            # supervised so it can still recover a malfunctioning AI.
            if self.mode == Mode.AUTO:
                self.disarm("STALE_OR_INVALID_RESULT")
            return

        self.last_link_at = current
        if self.recovering:
            if found:
                self._recovery_confirmed += 1
                if self._recovery_confirmed >= 3:
                    self._clear_recovery()
                    self.last_target_at = current
                    self.confirmed_frames = self.policy.auto_confirm_frames
                    self.reason = "SEARCH_REACQUIRED_3_FRAMES"
            else:
                self._recovery_confirmed = 0
            return
        if found:
            self.last_target_at = current
            self.confirmed_frames = min(
                self.policy.auto_confirm_frames, self.confirmed_frames + 1
            )
        else:
            self.last_target_at = None
            self.confirmed_frames = 0
            if self.mode == Mode.AUTO:
                self.disarm("TARGET_LOST")

    def arm_auto(self, now: Optional[float] = None) -> bool:
        current = self._now(now)
        self.tick(current)
        if self.mode == Mode.ESTOP:
            return False
        if not (
            self._link_is_fresh(current)
            and self._target_is_fresh(current)
            and self.confirmed_frames >= self.policy.auto_confirm_frames
        ):
            return False
        self.mode = Mode.AUTO
        self.reason = "OPERATOR_ARMED_AUTO"
        return True

    def arm_manual(self, now: Optional[float] = None) -> bool:
        current = self._now(now)
        self.tick(current)
        # Operator MANUAL must be available without a PC inference link.
        # Authentication must occur at the operator interface.
        if self.mode == Mode.ESTOP:
            return False
        self.mode = Mode.MANUAL
        self.reason = "OPERATOR_ARMED_MANUAL"
        self.last_manual_at = current
        return True

    def authorize(
        self,
        command: str,
        *,
        source: str,
        now: Optional[float] = None,
    ) -> ControlDecision:
        """Allowlisted motion requests only; rejection always returns STOP.

        source is AUTO or MANUAL; the source must match the selected mode.
        Translation defaults to disabled pending calibration and safety tests.
        Head-angle commands are not accepted by this first gate.
        """
        current = self._now(now)
        self.tick(current)
        name = command.strip().upper()
        origin = source.strip().upper()
        if name == "STOP":
            self.disarm("OPERATOR_STOP")
            return ControlDecision(True, "STOP", self.reason, self.mode)
        if self.mode == Mode.ESTOP:
            return ControlDecision(False, "STOP", "EMERGENCY_STOP", self.mode)
        if name not in _TURN_COMMANDS | _TRANSLATION_COMMANDS:
            return ControlDecision(False, "STOP", "UNKNOWN_OR_UNSAFE_COMMAND", self.mode)
        if name in _TRANSLATION_COMMANDS and not self.policy.allow_translation:
            return ControlDecision(False, "STOP", "TRANSLATION_DISABLED", self.mode)
        if self.recovering:
            return ControlDecision(False, "STOP", "SEARCH_ONLY_ONE_TURN", self.mode)
        if origin not in (Mode.AUTO.value, Mode.MANUAL.value) or origin != self.mode.value:
            return ControlDecision(False, "STOP", "MODE_MISMATCH_OR_DISARMED", self.mode)
        if self.mode == Mode.AUTO:
            if not self._link_is_fresh(current):
                self.disarm("LINK_TIMEOUT")
                return ControlDecision(False, "STOP", self.reason, self.mode)
            if not self._target_is_fresh(current):
                self.disarm("TARGET_TIMEOUT")
                return ControlDecision(False, "STOP", self.reason, self.mode)
        else:
            # Each manual command acts as a short-lived hold-to-run input.
            # This call must originate from a separate, authenticated operator
            # channel, NOT from an untrusted AI detection packet.
            self.last_manual_at = current
        return ControlDecision(True, name, "AUTHORIZED", self.mode)

    def status(self, now: Optional[float] = None) -> SafetyStatus:
        current = self._now(now)
        self.tick(current)
        return SafetyStatus(
            mode=self.mode,
            reason=self.reason,
            auto_ready=(
                self.mode != Mode.ESTOP
                and self._link_is_fresh(current)
                and self._target_is_fresh(current)
                and self.confirmed_frames >= self.policy.auto_confirm_frames
            ),
            confirmed_frames=self.confirmed_frames,
            link_age_s=(
                None if self.last_link_at is None else max(0.0, current - self.last_link_at)
            ),
            target_age_s=(
                None if self.last_target_at is None else max(0.0, current - self.last_target_at)
            ),
        )
