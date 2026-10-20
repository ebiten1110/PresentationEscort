"""Verified ESP32 safety handshake over the SAME trusted Pi USB serial link.

SAFETY_ARM is accepted only after an explicit operator request through
PiMotionGateway. A stale, missing, rejected or latched firmware report
never grants authorization. This is not encryption or an E-stop relay.
"""
from __future__ import annotations

import math
import re
import time
from typing import Optional

_STATE = re.compile(r"^\[Safety\] State: (ARMED|DISARMED|LATCHED)$")


class Esp32SafetyLink:
    def __init__(
        self,
        sink,
        *,
        ack_ttl_s: float = 0.55,
        arm_response_timeout_s: float = 0.40,
        heartbeat_interval_s: float = 0.15,
    ) -> None:
        if not (0 < heartbeat_interval_s < ack_ttl_s < 0.75):
            raise ValueError("heartbeat < ack_ttl < ESP32 timeout required")
        self.sink = sink
        self.ack_ttl_s = ack_ttl_s
        self.arm_response_timeout_s = arm_response_timeout_s
        self.heartbeat_interval_s = heartbeat_interval_s
        self._pending = ""
        self.state = "UNKNOWN"
        self.state_at: Optional[float] = None
        self.requested_mode: Optional[str] = None
        self.arm_requested_at: Optional[float] = None
        self._accepted = False
        self.last_ping_at: Optional[float] = None

    def request_arm(self, mode: str, *, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else float(now)
        if mode not in ("AUTO", "MANUAL") or not math.isfinite(current):
            return False
        if self.requested_mode is not None or self.state == "LATCHED":
            return False
        self.state = "UNKNOWN"
        self.state_at = None
        self._accepted = False
        self.requested_mode = mode
        self.arm_requested_at = current
        self.last_ping_at = None
        self.sink.send("SAFETY_ARM")
        return True

    def feed(self, raw: bytes, *, now: Optional[float] = None) -> Optional[str]:
        current = time.monotonic() if now is None else float(now)
        if not math.isfinite(current):
            raise ValueError("timestamp must be finite")
        data = (self._pending + raw.decode("utf-8", errors="replace"))
        rows = data.split("\n")
        self._pending = rows.pop()[-256:]
        authorized_mode: Optional[str] = None
        for row in rows:
            row = row.strip()
            if row == "[Safety] ARM_ACCEPTED":
                if self.requested_mode is not None:
                    self._accepted = True
            elif row == "[Safety] ARM_REJECTED":
                self.requested_mode = None
                self.arm_requested_at = None
                self._accepted = False
                self.state = "DISARMED"
                self.state_at = current
            else:
                match = _STATE.fullmatch(row)
                if match:
                    self.state = match.group(1)
                    self.state_at = current
                    if (
                        self.state == "ARMED"
                        and self._accepted
                        and self.requested_mode is not None
                        and self.arm_requested_at is not None
                        and 0 <= current - self.arm_requested_at
                            <= self.arm_response_timeout_s
                    ):
                        authorized_mode = self.requested_mode
                        self.requested_mode = None
                        self.arm_requested_at = None
                        self._accepted = False
                    elif self.state != "ARMED":
                        # STATUS sent before SAFETY_ARM can arrive after the
                        # operator has initiated a new handshake. Until an
                        # explicit ARM_ACCEPTED is seen, an old DISARMED line
                        # is informational only. No motion is allowed while
                        # pending, and unanswered requests still time out.
                        stale_disarmed = (
                            self.state == "DISARMED"
                            and self.requested_mode is not None
                            and not self._accepted
                        )
                        if not stale_disarmed:
                            self.requested_mode = None
                            self.arm_requested_at = None
                            self._accepted = False
        return authorized_mode

    def fresh_armed(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else float(now)
        return bool(
            self.state == "ARMED"
            and self.state_at is not None
            and 0 <= current - self.state_at <= self.ack_ttl_s
        )

    def pending_expired(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else float(now)
        return bool(
            self.requested_mode is not None
            and self.arm_requested_at is not None
            and current - self.arm_requested_at > self.arm_response_timeout_s
        )

    def send_ping_if_due(self, *, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else float(now)
        if not self.fresh_armed(current):
            return False
        if (
            self.last_ping_at is not None
            and 0 <= current - self.last_ping_at < self.heartbeat_interval_s
        ):
            return False
        self.sink.send("SAFETY_PING")
        self.last_ping_at = current
        return True

    def note_stop(self, *, latched: bool = False) -> None:
        self.state = "LATCHED" if latched else "DISARMED"
        self.state_at = None
        self.requested_mode = None
        self.arm_requested_at = None
        self._accepted = False
        self.last_ping_at = None
