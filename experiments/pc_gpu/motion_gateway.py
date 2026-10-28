"""Pi-side motion gateway and independent timeout watcher.

No motor I/O is performed at import time. ESP32 firmware needs its own
watchdog: Python cannot enforce STOP if the Pi crashes or USB is removed.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional, Protocol

from safety_manager import ControlDecision, Mode, SafetyManager, SafetyStatus
from esp32_safety_link import Esp32SafetyLink


class CommandSink(Protocol):
    def send(self, command: str) -> None: ...


@dataclass(frozen=True)
class GatewayStatus:
    safety: SafetyStatus
    last_command: str
    transport_fault: bool
    stop_reason: str
    motion_pending: bool
    fault_cause: Optional[str] = None
    arm_error: Optional[str] = None


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
        self._firmware_safety = Esp32SafetyLink(sink) if getattr(sink, "requires_firmware_safety", False) else None
        self.safety = safety if safety is not None else SafetyManager()
        self.poll_interval_s = poll_interval_s
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._transport_fault = False
        self._motion_pending = False
        self._head_active = False
        self._last_command = "NONE"
        self._stop_reason = "STARTUP"
        self._fault_cause: Optional[str] = None
        self._arm_error: Optional[str] = None
        self._closed = False

    def _stop_locked(self, reason: str, *, force: bool = False) -> None:
        self._stop_reason = reason
        if not force and not (self._motion_pending or self._head_active):
            return
        self._motion_pending = False
        self._head_active = False
        try:
            self.sink.send("STOP")
            self._last_command = "STOP"
            if self._firmware_safety is not None:
                self._firmware_safety.note_stop(latched=(self.safety.mode == Mode.ESTOP))
        except Exception:
            self._transport_fault = True
            self._last_command = "STOP_DELIVERY_FAILED"
            self._fault_cause = "STOP_DELIVERY_FAILED"
            self.safety.emergency_stop()

    def _tick_locked(self, now: Optional[float] = None) -> None:
        current = self.safety._now(now)
        fw = self._firmware_safety
        # Check ESP32 ACK freshness before the AI watchdog disarms AUTO,
        # otherwise a stale firmware status can be hidden as a target timeout.
        if (
            fw is not None
            and self.safety.mode in (Mode.AUTO, Mode.MANUAL)
            and not fw.fresh_armed(current)
        ):
            self._fault_cause = "FIRMWARE_SAFETY_TIMEOUT"
            self.safety.emergency_stop()
            self._stop_locked("FIRMWARE_SAFETY_TIMEOUT", force=True)
        self.safety.tick(now=current)
        if fw is not None:
            if fw.pending_expired(current):
                self._arm_error = "FIRMWARE_ARM_ACK_TIMEOUT"
                fw.note_stop()
                try:
                    self.sink.send("SAFETY_DISARM")
                except Exception:
                    self._transport_fault = True
                    self._fault_cause = "DISARM_WRITE_FAILED"
                    self.safety.emergency_stop()
            if self.safety.mode in (Mode.AUTO, Mode.MANUAL):
                if not fw.fresh_armed(current):
                    self._fault_cause = "FIRMWARE_SAFETY_TIMEOUT"
                    self.safety.emergency_stop()
                    self._stop_locked("FIRMWARE_SAFETY_TIMEOUT", force=True)
                else:
                    try:
                        fw.send_ping_if_due(now=current)
                    except Exception:
                        self._transport_fault = True
                        self._fault_cause = "SAFETY_PING_FAILED"
                        self.safety.emergency_stop()
                        self._stop_locked("SAFETY_PING_FAILED", force=True)
        if self.safety.mode not in (Mode.AUTO, Mode.MANUAL):
            # Firmware may still be ARMED after a head-only AUTO cycle.
            # Force one STOP even if there is no body motion pending.
            fw_needs_disarm = fw is not None and fw.state == "ARMED"
            self._stop_locked(self.safety.reason, force=fw_needs_disarm)

    def check_watchdog(self, now: Optional[float] = None) -> GatewayStatus:
        with self._lock:
            self._tick_locked(now)
            return self._status_locked(now)

    def _status_locked(self, now: Optional[float] = None) -> GatewayStatus:
        result = self.safety.status(now=now)
        if result.mode not in (Mode.AUTO, Mode.MANUAL):
            fw = self._firmware_safety
            self._stop_locked(result.reason, force=bool(fw and fw.state == "ARMED"))
        return GatewayStatus(
            safety=result,
            last_command=self._last_command,
            transport_fault=self._transport_fault,
            stop_reason=self._stop_reason,
            motion_pending=self._motion_pending,
            fault_cause=self._fault_cause,
            arm_error=self._arm_error,
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
                    self._fault_cause = "WATCHDOG_INTERNAL_ERROR"
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

    def observe_firmware_feedback(self, raw: bytes, *, now: Optional[float] = None) -> None:
        """Consume ESP32 safety ACK/STATUS from the sole serial reader."""
        with self._lock:
            fw = self._firmware_safety
            if fw is None:
                return
            current = self.safety._now(now)
            requested = fw.feed(raw, now=current)
            if fw.state == "LATCHED":
                if self.safety.mode != Mode.ESTOP:
                    self._fault_cause = "ESP32_REPORTED_LATCHED"
                self.safety.emergency_stop()
                self._stop_locked("ESP32_LATCHED", force=True)
                return
            if requested == "AUTO":
                self._arm_error = None
                if not self.safety.arm_auto(now=current):
                    self._stop_locked("AUTO_ARM_REJECTED", force=True)
            elif requested == "MANUAL":
                self._arm_error = None
                if not self.safety.arm_manual(now=current):
                    self._stop_locked("MANUAL_ARM_REJECTED", force=True)

    def firmware_arm_pending(self, requested_mode: str) -> bool:
        """Read-only handshake status for the operator UI; not permission to move."""
        with self._lock:
            fw = self._firmware_safety
            return bool(fw is not None and fw.requested_mode == requested_mode)

    def firmware_safety_status(self) -> str:
        with self._lock:
            return self._firmware_safety.state if self._firmware_safety is not None else "NOT_REQUIRED"

    def arm_auto(self, now: Optional[float] = None) -> bool:
        with self._lock:
            if self._requires_watchdog and not (self._thread and self._thread.is_alive()):
                return False
            if self._transport_fault or self._closed:
                return False
            self._tick_locked(now)
            if self.safety.mode == Mode.MANUAL:
                self.safety.disarm("MODE_SWITCH")
                self._stop_locked("MODE_SWITCH", force=True)
            if self._firmware_safety is not None:
                current = self.safety._now(now)
                if not self.safety.status(now=current).auto_ready:
                    return False
                try:
                    # First explicit call begins the ESP32 handshake.
                    # AUTO becomes active only upon a subsequent ARM_ACCEPTED.
                    initiated = self._firmware_safety.request_arm("AUTO", now=current)
                    if initiated:
                        self._arm_error = None
                except Exception:
                    self._transport_fault = True
                    self._fault_cause = "SAFETY_ARM_WRITE_FAILED"
                    self.safety.emergency_stop()
                    return False
                return False
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
                self.safety.disarm("MODE_SWITCH")
                self._stop_locked("MODE_SWITCH", force=True)
            if self._firmware_safety is not None:
                try:
                    self._firmware_safety.request_arm("MANUAL", now=self.safety._now(now))
                except Exception:
                    self._transport_fault = True
                    self.safety.emergency_stop()
                return False
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
                self._fault_cause = "MOTION_SEND_FAILED"
                self._stop_locked("MOTION_SEND_FAILED", force=True)
                return ControlDecision(False, "STOP", "MOTION_SEND_FAILED", Mode.ESTOP)

    def request_head_pose(
        self,
        yaw_deg: float,
        pitch_deg: float,
        *,
        now: Optional[float] = None,
    ) -> ControlDecision:
        """Only AUTO with fresh detection may move the head.

        Enforce an independent numeric range check even if the transport
        validates the formatted command. An ESTOP never centers the head
        automatically (centering itself is an actuator movement).
        """
        import math

        with self._lock:
            if self._requires_watchdog and not (self._thread and self._thread.is_alive()):
                self.safety.emergency_stop()
                self._stop_locked("WATCHDOG_NOT_RUNNING", force=True)
                return ControlDecision(False, "STOP", "WATCHDOG_NOT_RUNNING", Mode.ESTOP)
            if self._transport_fault or self._closed:
                return ControlDecision(False, "STOP", "SERIAL_FAULT_OR_CLOSED", self.safety.mode)
            self._tick_locked(now)
            if (
                not math.isfinite(yaw_deg)
                or not math.isfinite(pitch_deg)
                or not (-20 <= yaw_deg <= 20 and -12 <= pitch_deg <= 12)
            ):
                return ControlDecision(False, "STOP", "HEAD_ANGLE_REJECTED", self.safety.mode)
            current = self.safety._now(now)
            if (
                self.safety.mode != Mode.AUTO
                or not self.safety._link_is_fresh(current)
                or not self.safety._target_is_fresh(current)
            ):
                return ControlDecision(False, "STOP", "HEAD_NOT_AUTHORIZED", self.safety.mode)
            command = f"HEAD_POSE_SET:{yaw_deg:.1f},{pitch_deg:.1f}"
            try:
                self.sink.send(command)
                self._last_command = command
                self._head_active = True
                return ControlDecision(True, command, "AUTHORIZED", self.safety.mode)
            except Exception:
                self._transport_fault = True
                self.safety.emergency_stop()
                self._fault_cause = "HEAD_SEND_FAILED"
                self._stop_locked("HEAD_SEND_FAILED", force=True)
                return ControlDecision(False, "STOP", "HEAD_SEND_FAILED", Mode.ESTOP)

    def emergency_stop(self) -> None:
        with self._lock:
            self._fault_cause = "OPERATOR_ESTOP"
            self.safety.emergency_stop()
            if self._firmware_safety is not None:
                try:
                    self.sink.send("SAFETY_ESTOP")
                except Exception:
                    self._transport_fault = True
            self._stop_locked("EMERGENCY_STOP", force=True)

    def request_firmware_reset(self) -> bool:
        """Operator-only request after inspecting the physical device.

        Never runs automatically after a timeout or reconnection. Sending
        RESET does NOT itself rearm Pi nor firmware motion control.
        """
        with self._lock:
            if (
                self._firmware_safety is None
                or self.safety.mode != Mode.ESTOP
                or self._transport_fault
            ):
                return False
            try:
                self.sink.send("SAFETY_RESET")
                return True
            except Exception:
                self._transport_fault = True
                return False

    def reset_emergency_stop(self, now: Optional[float] = None) -> None:
        """Operator acknowledgement; cannot reset before fresh ESP32 DISARMED."""
        with self._lock:
            if self._transport_fault:
                return
            if self._firmware_safety is not None:
                fw = self._firmware_safety
                if fw.state != "DISARMED" or fw.state_at is None:
                    return
                current = self.safety._now(now)
                if not 0 <= current - fw.state_at <= fw.ack_ttl_s:
                    return
            self.safety.reset_emergency_stop()
            if self.safety.mode == Mode.SAFE:
                self._fault_cause = None
                self._arm_error = None

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
