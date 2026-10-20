"""Serial command transport for Pi → ESP32, opt-in only.

Importing this file does not open a port. Opening an ESP32 USB serial port
may reset the board and trigger its startup servo-centering code.
"""
from __future__ import annotations

import re
import threading
from typing import Callable, Optional

_HEAD_POSE = re.compile(r"^HEAD_POSE_SET:([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?)$")
_ALLOWED = frozenset({
    "STOP", "TRACK_LEFT", "TRACK_RIGHT",
    "TRACK_FORWARD", "TRACK_BACKWARD",
    "HEAD_STATUS", "STATUS",
})


class SerialCommandSink:
    """Bounded pyserial.write transport with a strict motor allowlist."""

    requires_watchdog = True

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 115200,
        write_timeout_s: float = 0.15,
        serial_factory: Optional[Callable] = None,
    ) -> None:
        if not port or not port.strip():
            raise ValueError("serial port is required")
        if baudrate <= 0 or write_timeout_s <= 0:
            raise ValueError("baudrate and write timeout must be positive")
        if serial_factory is None:
            import serial  # Lazy import: no pyserial needed by dry-run/CI.
            serial_factory = serial.Serial

        self.port = port
        self._lock = threading.Lock()
        self._serial = serial_factory(
            port=port, baudrate=baudrate, timeout=0.1,
            write_timeout=write_timeout_s,
        )

    def send(self, command: str) -> None:
        normalized = command.strip().upper()
        pose = _HEAD_POSE.fullmatch(normalized)
        if pose is not None:
            yaw, pitch = float(pose.group(1)), float(pose.group(2))
            if not (-20.0 <= yaw <= 20.0 and -12.0 <= pitch <= 12.0):
                raise ValueError("head angles outside safe V8.8.3 range")
        elif normalized not in _ALLOWED:
            raise ValueError("unsupported or unsafe serial command")
        payload = (normalized + "\n").encode("ascii")
        with self._lock:
            if not bool(getattr(self._serial, "is_open", False)):
                raise ConnectionError("serial port not open")
            written = self._serial.write(payload)
            if written != len(payload):
                raise IOError("incomplete serial command write")

    def read_available(self, *, max_bytes: int = 2048) -> bytes:
        """Nonblocking bounded telemetry read, same lock as writes."""
        if not 1 <= max_bytes <= 8192:
            raise ValueError("max_bytes out of range")
        with self._lock:
            if not bool(getattr(self._serial, "is_open", False)):
                raise ConnectionError("serial port not open")
            waiting = int(getattr(self._serial, "in_waiting", 0))
            if waiting <= 0:
                return b""
            return self._serial.read(min(waiting, max_bytes))

    def close(self) -> None:
        with self._lock:
            self._serial.close()
