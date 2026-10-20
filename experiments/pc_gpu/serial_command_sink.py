"""Serial command transport for Pi → ESP32, opt-in only.

Importing this file does not open a port. Opening an ESP32 USB serial port
may reset the board and trigger its startup servo-centering code.
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

_ALLOWED = frozenset({
    "STOP", "TRACK_LEFT", "TRACK_RIGHT",
    "TRACK_FORWARD", "TRACK_BACKWARD",
})


class SerialCommandSink:
    """Bounded pyserial.write transport with a strict motor allowlist."""

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
        if normalized not in _ALLOWED:
            raise ValueError("unsupported or unsafe serial command")
        payload = (normalized + "\n").encode("ascii")
        with self._lock:
            if not bool(getattr(self._serial, "is_open", False)):
                raise ConnectionError("serial port not open")
            written = self._serial.write(payload)
            if written != len(payload):
                raise IOError("incomplete serial command write")

    def close(self) -> None:
        with self._lock:
            self._serial.close()
