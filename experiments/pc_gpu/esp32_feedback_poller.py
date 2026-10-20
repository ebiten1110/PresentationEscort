"""Opt-in Pi UART telemetry poller for the unmodified V8.8.3 ESP32.

Connect it to the *same* SerialCommandSink used by PiMotionGateway.
Never opens a port itself. Calling poll is the caller's responsibility.
"""
from __future__ import annotations

import math
import time
from typing import Optional

from esp32_telemetry import Esp32Telemetry
from remote_person_controller import RemotePersonController


class Esp32FeedbackPoller:
    def __init__(
        self,
        sink,
        controller: RemotePersonController,
        *,
        status_interval_s: float = 0.20,
    ) -> None:
        if not math.isfinite(status_interval_s) or status_interval_s < 0.10:
            raise ValueError("status interval must be >= 0.10s")
        self.sink = sink
        self.controller = controller
        self.status_interval_s = status_interval_s
        self.last_request_at: Optional[float] = None

    def poll(self, *, now: Optional[float] = None) -> int:
        """Read UART response and issue read-only STATUS queries periodically."""
        current = time.monotonic() if now is None else float(now)
        if not math.isfinite(current):
            raise ValueError("invalid timestamp")
        received = self.sink.read_available(max_bytes=4096)
        if received:
            self.controller.feed_esp32(received, now=current)
        if (
            self.last_request_at is None
            or current - self.last_request_at >= self.status_interval_s
        ):
            self.sink.send("STATUS")
            self.last_request_at = current
        return len(received)
