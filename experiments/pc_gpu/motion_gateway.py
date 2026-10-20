"""Pi-side motion gateway and independent timeout watcher.

No motor I/O is performed at import time. ESP32 firmware needs its own
watchdog: Python cannot enforce STOP if the Pi crashes or USB is removed.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional, Protocol

from safety_manager import ControlDecision, Mode, SafetyManager, SafetyStatus


class CommandSink(Protocol):
    def send(self, command: str) -> None: ...


@dataclass(frozen=True)
class GatewayStatus:
    safety: SafetyStatus
    last_command: str
    transport_fault: bool
    stop_reason: str
    motion_pending: bool


class MemoryCommandSink:
    """Hardware-free sink for CI, offline tests, and simulation."""

    def __init__(self) -> None:
        self.commands: list[str] = []

    def send(self, command: str) -> None:
        self.commands.append(command)


class PiMotionGateway:
    """All motion commands pass through a single fail-closed gate.

    A separate polling thread checks timeouts while GPU recv() blocks.
    Requests need explicit AUTO/MANUAL arming; no automatic rearming.
    Transport write failures latch ESTOP and must be treated as unsafe:
    STOP might not have reached the physical ESP32.
    """

    def __init__(
        self,
        sink: CommandSink,
        *,
        safety: Optional[SafetyManager] = None,
        poll_interval_s: float = 0.05,
    ) -> None:
        if not 0.01 <= poll_interval_s <= 0.25:
            raise ValueError("poll_interval_s must be 0.01 .. 0.25")
        self.sink = sink
        self._requires_watchdog = bool(getattr(sink, "requires_watchdog", False))
        self.safety = safety if safety is not None else SafetyManager()
        self.poll_interval_s = poll_interval_s
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._transport_fault = False
        self._motion_pending = False
        self._last_command = "NONE"
        self._stop_reason = "STARTUP"
        self._closed = False

    def _stop_locked(self, reason: str, *, force: bool = False) -> None:
        self._stop_reason = reason
        if not force and not self._motion_pending:
            return
        self._motion_pending = False
        try:
            self.sink.send("STOP")
            self._last_command = "STOP"
        except Exception:
            self._transport_fault = True
            self._last_command = "STOP_DELIVERY_FAILED"
            self.safety.emergency_stop()

    def _tick_locked(self, now: Optional[float] = None) -> None:
        self.safety.tick(now=now)
        if self.safety.mode not in (Mode.AUTO, Mode.MANUAL):
            self._stop_locked(self.safety.reason)

    def check_watchdog(self, now: Optional[float] = None) -> GatewayStatus:
        with self._lock:
            self._tick_locked(now)
            return self._status_locked(now)

    def _status_locked(self, now: Optional[float] = None) -> GatewayStatus:
        result = self.safety.status(now=now)
        if result.mode not in (Mode.AUTO, Mode.MANUAL):
            self._stop_locked(result.reason)
        return GatewayStatus(
            safety=result,
            last_command=self._last_command,
            transport_fault=self._transport_fault,
            stop_reason=self._stop_reason,
            motion_pending=self._motion_pending,
        )

    def status(self, now: Optional[float] = None) -> GatewayStatus:
        with self._lock:
            return self._status_locked(now)

    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError("cannot restart closed gateway")
            if self._thread is not None:
                return
            self._stop_locked("STARTUP_DISARMED", force=True)
            if self._transport_fault:
                raise RuntimeError("startup STOP delivery failed")
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._watch_loop, name="pi-motion-watchdog", daemon=True
            )
            self._thread.start()

    def _watch_loop(self) -> None:
        while not self._stop_event.wait(self.poll_interval_s):
            try:
                with self._lock:
                    self._tick_locked()
            except Exception:
                with self._lock:
                    self.safety.emergency_stop()
                    self._stop_locked("WATCHDOG_INTERNAL_ERROR", force=True)

    def observe_detection(self, found: bool, rtt_ms: float, now: Optional[float] = None) -> None:
        with self._lock:
            self.safety.observe_detection(found, rtt_ms, now=now)
            self._tick_locked(now)

    def connection_lost(self) -> None:
        with self._lock:
            self.safety.connection_lost()
            # AUTO stops. Independently authenticated MANUAL can continue,
            # guarded by manual deadman even when the AI link fails.
            if self.safety.mode not in (Mode.AUTO, Mode.MANUAL):
                self._stop_locked("LINK_LOST")

    def arm_auto(self, now: Optional[float] = None) -> bool:
        with self._lock:
            if self._requires_watchdog and not (self._thread and self._thread.is_alive()):
                return False
            if self._transport_fault or self._closed:
                return False
            self._tick_locked(now)
            if self.safety.mode == Mode.MANUAL:
                self._stop_locked("MODE_SWITCH", force=self._motion_pending)
            return self.safety.arm_auto(now=now)

    def arm_manual(self, now: Optional[float] = None) -> bool:
        """Caller must authenticate operator independently of AI traffic."""
        with self._lock:
            if self._requires_watchdog and not (self._thread and self._thread.is_alive()):
                return False
            if self._transport_fault or self._closed:
                return False
            self._tick_locked(now)
            if self.safety.mode == Mode.AUTO:
                self._stop_locked("MODE_SWITCH", force=self._motion_pending)
            return self.safety.arm_manual(now=now)

    def request_motion(
        self, command: str, *, source: str, now: Optional[float] = None
    ) -> ControlDecision:
        with self._lock:
            if self._requires_watchdog and not (self._thread and self._thread.is_alive()):
                self.safety.emergency_stop()
                self._stop_locked("WATCHDOG_NOT_RUNNING", force=True)
                return ControlDecision(False, "STOP", "WATCHDOG_NOT_RUNNING", Mode.ESTOP)
            if self._transport_fault or self._closed:
                return ControlDecision(False, "STOP", "SERIAL_FAULT_OR_CLOSED", self.safety.mode)
            self._tick_locked(now)
            decision = self.safety.authorize(command, source=source, now=now)
            if not decision.approved or decision.command == "STOP":
                self._stop_locked(decision.reason, force=decision.command == "STOP")
                return decision
            try:
                self.sink.send(decision.command)
                self._last_command = decision.command
                self._motion_pending = True
                return decision
            except Exception:
                self._transport_fault = True
                self.safety.emergency_stop()
                self._stop_locked("MOTION_SEND_FAILED", force=True)
                return ControlDecision(False, "STOP", "MOTION_SEND_FAILED", Mode.ESTOP)

    def emergency_stop(self) -> None:
        with self._lock:
            self.safety.emergency_stop()
            self._stop_locked("EMERGENCY_STOP", force=True)

    def reset_emergency_stop(self) -> None:
        with self._lock:
            if not self._transport_fault:
                self.safety.reset_emergency_stop()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self.safety.emergency_stop()
            self._stop_locked("SHUTDOWN", force=True)
            self._stop_event.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        closer = getattr(self.sink, "close", None)
        if callable(closer):
            closer()

    def __enter__(self) -> "PiMotionGateway":
        self.start()
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.close()
