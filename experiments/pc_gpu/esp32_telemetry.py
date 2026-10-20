"""Parse ESP32 V8.8.3 serial telemetry (head, walking, busy).

No serial I/O, dependencies, threads or physical side effects. Data is accepted
only from real ESP32 serial output supplied by the caller, never from YOLO.
BODY turn readiness needs a fresh STATUS snapshot with Busy: NO, idle state,
and fresh HeadState. A stale or partial line fails closed.
"""
from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from typing import Optional


_FLOAT = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+))"
_HEAD = re.compile(
    r"^\[HeadState\]\s+currentYaw=" + _FLOAT
    + r"\s+targetYaw=" + _FLOAT
    + r"\s+currentPitch=" + _FLOAT
    + r"\s+targetPitch=" + _FLOAT
    + r"\s+moving=(YES|NO)$"
)
_WALKING = re.compile(r"^\[Walking\] State:\s+([A-Z_]+)$")
_STATUS_WALKING = re.compile(r"^Walking State:\s+([A-Z_]+)$")
_BUSY = re.compile(r"^Busy:\s+(YES|NO)$")
_ALLOWED_STATES = frozenset({
    "IDLE", "STAND", "START_WALK", "FORWARD_WALK", "STOP_WALK",
    "TURN_LEFT", "TURN_RIGHT", "TRACK_LEFT", "TRACK_RIGHT",
    "TRACK_FORWARD", "TRACK_BACKWARD",
})


@dataclass(frozen=True)
class Esp32State:
    head_yaw_deg: Optional[float]
    head_pitch_deg: Optional[float]
    head_fresh: bool
    walking_state: str
    busy: Optional[bool]
    body_idle_confirmed: bool
    motion_completion: bool


class Esp32Telemetry:
    def __init__(self, *, head_ttl_s: float = 0.45, status_ttl_s: float = 0.40) -> None:
        if head_ttl_s <= 0 or status_ttl_s <= 0:
            raise ValueError("TTL must be positive")
        self.head_ttl_s = head_ttl_s
        self.status_ttl_s = status_ttl_s
        self.reset()

    def reset(self) -> None:
        self._pending = ""
        self._yaw: Optional[float] = None
        self._pitch: Optional[float] = None
        self._head_at: Optional[float] = None
        self._walking_state = "UNKNOWN"
        self._walking_at: Optional[float] = None
        self._busy: Optional[bool] = None
        self._busy_at: Optional[float] = None
        self._last_active_turn: Optional[str] = None
        self._turn_completed = False

    def feed(self, data: bytes, *, now: Optional[float] = None) -> None:
        timestamp = time.monotonic() if now is None else float(now)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        # Disallow unbounded memory consumption from corrupt UART text.
        text = self._pending + data.decode("utf-8", errors="replace")
        lines = text.split("\n")
        if text.endswith("\n"):
            self._pending = ""
        else:
            self._pending = lines.pop()[-512:]
        for line in lines:
            self.feed_line(line.strip(), now=timestamp)

    def feed_line(self, line: str, *, now: Optional[float] = None) -> None:
        timestamp = time.monotonic() if now is None else float(now)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        head = _HEAD.fullmatch(line)
        if head:
            yaw, _, pitch, _, _ = head.groups()
            yaw_f, pitch_f = float(yaw), float(pitch)
            if -20 <= yaw_f <= 20 and -12 <= pitch_f <= 12:
                self._yaw, self._pitch, self._head_at = yaw_f, pitch_f, timestamp
            return
        walking = _WALKING.fullmatch(line) or _STATUS_WALKING.fullmatch(line)
        if walking:
            name = walking.group(1)
            self._walking_state = name if name in _ALLOWED_STATES else "UNKNOWN"
            self._walking_at = timestamp
            # A new state line invalidates any old Busy response.
            self._busy, self._busy_at = None, None
            if name in ("TRACK_LEFT", "TRACK_RIGHT"):
                self._last_active_turn = name
            if name == "IDLE" and self._last_active_turn is not None:
                # Completion is not exposed until a fresh Busy:NO confirms idle.
                self._turn_completed = True
            return
        busy = _BUSY.fullmatch(line)
        if busy:
            self._busy = busy.group(1) == "YES"
            self._busy_at = timestamp

    @staticmethod
    def _recent(at: Optional[float], now: float, ttl: float) -> bool:
        return at is not None and 0 <= now - at <= ttl

    def snapshot(self, *, now: Optional[float] = None) -> Esp32State:
        current = time.monotonic() if now is None else float(now)
        if not math.isfinite(current):
            raise ValueError("timestamp must be finite")
        fresh_head = self._recent(self._head_at, current, self.head_ttl_s)
        fresh_walk = self._recent(self._walking_at, current, self.status_ttl_s)
        fresh_busy = self._recent(self._busy_at, current, self.status_ttl_s)
        idle = fresh_walk and fresh_busy and self._walking_state == "IDLE" and self._busy is False
        return Esp32State(
            head_yaw_deg=self._yaw if fresh_head else None,
            head_pitch_deg=self._pitch if fresh_head else None,
            head_fresh=fresh_head,
            walking_state=self._walking_state if fresh_walk else "UNKNOWN",
            busy=self._busy if fresh_busy else None,
            body_idle_confirmed=idle,
            motion_completion=bool(self._turn_completed and idle),
        )

    def consume_completed_turn(self, *, now: Optional[float] = None) -> Optional[str]:
        state = self.snapshot(now=now)
        if not state.motion_completion:
            return None
        command = self._last_active_turn
        self._last_active_turn = None
        self._turn_completed = False
        return command
