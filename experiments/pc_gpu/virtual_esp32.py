"""Strict, in-memory ESP32 V1 serial emulator for Phase 9 integration tests.

No pyserial, socket, GPIO, camera or actuator code anywhere in this module.
Models only enough ESP32 wire protocol to exercise the SAME PiMotionGateway,
Esp32FeedbackPoller and RemotePersonController used by the future deployment.
It is not a mechanical simulator and does not prove safety on the robot.
"""
from __future__ import annotations

import re
import threading
import time
from typing import Callable, Optional


_HEAD = re.compile(r"^HEAD_POSE_SET:([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?)$")


class VirtualEsp32:
    requires_watchdog = True
    requires_firmware_safety = True

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        motion_duration_s: float = 0.12,
    ) -> None:
        self._clock = clock
        self._lock = threading.RLock()
        self._queue = bytearray()
        self.commands: list[str] = []
        self.state = "DISARMED"
        self.walking_state = "IDLE"
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self.last_ping_at: Optional[float] = None
        self.motion_end_at: Optional[float] = None
        self.motion_duration_s = motion_duration_s
        self.closed = False
        self.link_broken = False
        self._enqueue("[Safety] Protocol=V1 TimeoutMs=750")
        self._enqueue("[Safety] State: DISARMED")

    def _enqueue(self, line: str) -> None:
        self._queue.extend((line + "\n").encode("ascii"))

    def _tick(self) -> None:
        now = self._clock()
        if (
            self.state == "ARMED"
            and self.last_ping_at is not None
            and now - self.last_ping_at > 0.750
        ):
            self.state = "LATCHED"
            self.walking_state = "IDLE"
            self.motion_end_at = None
            self._enqueue("[Safety] FAULT=HEARTBEAT_TIMEOUT")
            self._enqueue("[Safety] State: LATCHED")
        if self.motion_end_at is not None and now >= self.motion_end_at:
            self.walking_state = "IDLE"
            self.motion_end_at = None
            self._enqueue("[Walking] State: IDLE")

    def send(self, command: str) -> None:
        with self._lock:
            if self.closed or self.link_broken:
                raise OSError("virtual USB link disconnected")
            self._tick()
            name = command.strip().upper()
            self.commands.append(name)
            if name == "STOP" or name == "SAFETY_DISARM":
                if self.state == "ARMED":
                    self.state = "DISARMED"
                self.walking_state = "IDLE"
                self.motion_end_at = None
                self._enqueue("[Walking] State: IDLE")
                self._enqueue("[Safety] State: " + self.state)
                return
            if name == "SAFETY_ESTOP":
                self.state = "LATCHED"
                self.walking_state = "IDLE"
                self.motion_end_at = None
                self._enqueue("[Safety] State: LATCHED")
                return
            if name == "SAFETY_RESET":
                if self.state == "LATCHED":
                    self.state = "DISARMED"
                    self._enqueue("[Safety] RESET_ACCEPTED")
                else:
                    self._enqueue("[Safety] RESET_REJECTED")
                self._enqueue("[Safety] State: " + self.state)
                return
            if name == "SAFETY_ARM":
                if self.state == "DISARMED" and self.walking_state == "IDLE":
                    self.state = "ARMED"
                    self.last_ping_at = self._clock()
                    self._enqueue("[Safety] ARM_ACCEPTED")
                else:
                    self._enqueue("[Safety] ARM_REJECTED")
                self._enqueue("[Safety] State: " + self.state)
                return
            if name == "SAFETY_PING":
                if self.state == "ARMED":
                    self.last_ping_at = self._clock()
                return
            if name == "SAFETY_VERSION":
                self._enqueue("[Safety] Protocol=V1 TimeoutMs=750")
                self._enqueue("[Safety] State: " + self.state)
                return
            if name == "SAFETY_STATUS":
                self._enqueue("[Safety] State: " + self.state)
                return
            if name == "HEAD_STATUS":
                self._report_head()
                return
            if name == "STATUS":
                self._enqueue("[Safety] State: " + self.state)
                self._enqueue("Walking State: " + self.walking_state)
                self._enqueue("Busy: " + ("YES" if self.walking_state != "IDLE" else "NO"))
                self._report_head()
                return
            # All motion/servo commands are rejected without valid ARM.
            if self.state != "ARMED":
                self._enqueue("[Safety] MOTION_REJECTED=" + name)
                return
            pose = _HEAD.fullmatch(name)
            if pose:
                yaw, pitch = map(float, pose.groups())
                if -20 <= yaw <= 20 and -12 <= pitch <= 12:
                    self.yaw_deg, self.pitch_deg = yaw, pitch
                    self._report_head()
                else:
                    self._enqueue("[Safety] MOTION_REJECTED=" + name)
                return
            if name in ("TRACK_LEFT", "TRACK_RIGHT") and self.walking_state == "IDLE":
                self.walking_state = name
                self.motion_end_at = self._clock() + self.motion_duration_s
                self._enqueue("[Walking] State: " + name)
                return
            self._enqueue("[Safety] MOTION_REJECTED=" + name)

    def _report_head(self) -> None:
        self._enqueue(
            "[HeadState] currentYaw={:.1f} targetYaw={:.1f} currentPitch={:.1f} targetPitch={:.1f} moving=NO"
            .format(self.yaw_deg, self.yaw_deg, self.pitch_deg, self.pitch_deg)
        )

    def read_available(self, *, max_bytes: int = 4096) -> bytes:
        with self._lock:
            if self.closed or self.link_broken:
                raise OSError("virtual USB link disconnected")
            if not 1 <= max_bytes <= 8192:
                raise ValueError("invalid max_bytes")
            self._tick()
            result = bytes(self._queue[:max_bytes])
            del self._queue[:max_bytes]
            return result

    def disconnect(self) -> None:
        with self._lock:
            self.link_broken = True

    def reconnect(self) -> None:
        with self._lock:
            self.link_broken = False
            self._tick()
            self._enqueue("[Safety] State: " + self.state)

    def close(self) -> None:
        with self._lock:
            self.closed = True
