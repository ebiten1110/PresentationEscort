"""Controlled Pi camera -> PC YOLO -> presenter lock -> REAL ESP32 bridge.

Default: read-only UART monitor (servo power may be ON; no software ARM).
Opt-in, supervised: head yaw <=3 degrees around verified neutral, pitch zero.
Body tracking turns require separate opt-in, bounded pulse count and calibration.
No translation or automatic ARM on startup/re-connection.
Power/physical ESTOP status is operator-declared, not measured by software.
"""
from __future__ import annotations

import argparse
import math
import re
import secrets
import time
from dataclasses import replace
from typing import Callable, Optional

from esp32_feedback_poller import Esp32FeedbackPoller
from motion_gateway import PiMotionGateway
from pi_operator_console import CameraInferenceWorker, OperatorRuntime, OperatorServer
from pi_presenter_lock_demo import LockedSimulationRuntime
from remote_person_controller import RemotePersonController
from safety_manager import Mode
from serial_command_sink import SerialCommandSink
from v883_person_dual_loop import V883PersonDualLoop

_POSE = re.compile(r"^HEAD_POSE_SET:([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?)$")


class HeadOnlyPlanner(V883PersonDualLoop):
    """Limit the planner's INTERNAL state, not only its returned proposal."""

    def step(self, *args, **kwargs):
        # The V8.8 planner proposes full-range body motions. Never retain
        # or forward these proposals to the head-only real actuator mode.
        plan = super().step(*args, **kwargs)
        self.yaw_target_deg = max(-3.0, min(3.0, self.yaw_target_deg))
        self.pitch_target_deg = 0.0
        self.body_candidate = None
        self.body_candidate_count = 0
        self.motion_pending = False
        if plan.head_pose_command is not None:
            self.last_sent_yaw = self.yaw_target_deg
            self.last_sent_pitch = 0.0
            head_command = f"HEAD_POSE_SET:{self.yaw_target_deg:.1f},0.0"
        else:
            head_command = None
        return replace(
            plan, body_command=None, suggested_body_turn=None,
            yaw_target_deg=self.yaw_target_deg,
            pitch_target_deg=0.0,
            head_pose_command=head_command,
        )


class HeadBodyTurnPlanner(V883PersonDualLoop):
    """Repeated turns subject to ESP32 idle and head feedback, not a session quota."""
    def step(self, *args, **kwargs):
        plan = super().step(*args, **kwargs)
        self.yaw_target_deg = max(-3.0, min(3.0, self.yaw_target_deg))
        self.pitch_target_deg = 0.0
        head_command = None
        if plan.head_pose_command is not None:
            self.last_sent_yaw = self.yaw_target_deg
            self.last_sent_pitch = 0.0
            head_command = f"HEAD_POSE_SET:{self.yaw_target_deg:.1f},0.0"
        return replace(plan, yaw_target_deg=self.yaw_target_deg,
                       pitch_target_deg=0.0, head_pose_command=head_command)


class HeadOnlySerialSink:
    requires_watchdog = True
    requires_firmware_safety = True

    def __init__(self, port: str, *, head_enabled: bool,
                 body_enabled: bool = False):
        self._inner = SerialCommandSink(port)
        self.head_enabled = head_enabled
        self.body_enabled = body_enabled
        self._serial_partial = ""
        self.last_safety_event: Optional[str] = None

    def send(self, command: str) -> None:
        cmd = command.strip().upper()
        if cmd.startswith("TRACK_") or cmd in {"LEFT", "RIGHT", "FORWARD", "BACKWARD", "STAND"}:
            if not (self.body_enabled and cmd in {"TRACK_LEFT", "TRACK_RIGHT"}
):
                raise ValueError("body command outside allowed bounded-turn budget")
        pose = _POSE.fullmatch(cmd)
        if pose is not None:
            yaw, pitch = float(pose[1]), float(pose[2])
            if (not self.head_enabled or not math.isfinite(yaw)
                    or not math.isfinite(pitch) or abs(yaw) > 3.0
                    or abs(pitch) > 0.001):
                raise ValueError("head motion beyond calibrated head-only test scope")
        elif cmd.startswith("HEAD_"):
            # HEAD_STATUS read-only is allowed; all other head variants blocked.
            if cmd != "HEAD_STATUS":
                raise ValueError("unapproved head command")
        elif cmd == "SAFETY_ARM" and not self.head_enabled:
            raise ValueError("ARM is forbidden in UART probe mode")
        self._inner.send(cmd)

    def read_available(self, *, max_bytes: int = 2048) -> bytes:
        raw = self._inner.read_available(max_bytes=max_bytes)
        if raw:
            incoming = self._serial_partial + raw.decode("ascii", errors="replace")
            *lines, self._serial_partial = incoming.split("\n")
            self._serial_partial = self._serial_partial[-256:]
            for line in lines:
                line = line.strip()
                # Store ESP32's actual diagnostic line, not a guess inferred
                # from presenter detection or an old ARM-success message.
                if line.startswith("[Safety] FAULT=") or line in (
                    "[Safety] ARM_REJECTED",
                    "[Safety] RESET_REJECTED",
                ) or line.startswith("[Safety] MOTION_REJECTED="):
                    self.last_safety_event = line[:180]
        return raw

    def close(self) -> None:
        self._inner.close()


class LockedRealRuntime(LockedSimulationRuntime):
    """Reuse real V2 presenter-lock logic and safety gateway without a virtual MCU."""

    def __init__(self, serial_port: str, *, head_enabled: bool = False,
                 power_declared: str = "UNKNOWN", body_enabled: bool = False) -> None:
        if body_enabled and not head_enabled:
            raise ValueError("body turns require head control opt-in")
        super().__init__()
        # Superclass has not started any threads or issued any commands.
        self.sink.close()
        self.sink = HeadOnlySerialSink(
            serial_port, head_enabled=head_enabled, body_enabled=body_enabled)
        self.gateway = PiMotionGateway(self.sink)
        self.controller = RemotePersonController(
            self.gateway,
            planner=(HeadBodyTurnPlanner() if body_enabled else HeadOnlyPlanner()),
            enable_actuation=head_enabled,
        )
        self._feedback = Esp32FeedbackPoller(self.sink, self.controller)
        self.head_enabled = head_enabled
        self.body_enabled = body_enabled
        self.power_declared = power_declared
        self._last_event = "REAL_UART_READY_HEAD_DISABLED" if not head_enabled else "REAL_HEAD_TEST_READY"
        self._uart_poll_error: Optional[str] = None
        self._arm_wait_since: Optional[float] = None
        self._last_arm_error: Optional[str] = None

    def _poll_loop(self) -> None:
        """Real UART reader: persist errors beyond the next presenter event."""
        while not self._exit.wait(0.025):
            try:
                with self._guard:
                    self._feedback.poll()
                    if self._reset_requested:
                        self.gateway.reset_emergency_stop()
                        if self.gateway.status().safety.mode == Mode.SAFE:
                            self._reset_requested = False
                            self._last_event = "REAL_ESTOP_RESET_TO_SAFE"
                    if self._arm_wait_since is not None:
                        state = self.gateway.status().safety.mode
                        if state == Mode.AUTO:
                            self._arm_wait_since = None
                            self._last_arm_error = None
                        elif not self.gateway.firmware_arm_pending("AUTO"):
                            self._last_arm_error = (
                                "AUTO_ARM_NOT_COMPLETED_BEFORE_ESTOP" if state == Mode.ESTOP
                                else "AUTO_ARM_NOT_ACKNOWLEDGED_OR_CANCELED"
                            )
                            self._arm_wait_since = None
            except Exception as exc:
                with self._guard:
                    self._uart_poll_error = type(exc).__name__
                    self.gateway.emergency_stop()
                    self._last_event = "REAL_UART_POLL_ERROR_" + type(exc).__name__
                return

    def status(self) -> dict:
        with self._guard:
            # Bypass IntegratedSimulationRuntime.status(): avoid misleading VIRTUAL UI.
            status = OperatorRuntime.status(self)
            presenter = self.presenter.display(now=time.monotonic())
            status.update(presenter)
            esp32 = self.controller.telemetry.snapshot(now=time.monotonic())
            status["esp32_head_feedback"] = (
                {"yaw_deg": esp32.head_yaw_deg, "pitch_deg": esp32.head_pitch_deg,
                 "fresh": esp32.head_fresh}
            )
            status["esp32_walking_feedback"] = {
                "state": esp32.walking_state, "busy": esp32.busy,
                "idle_confirmed": esp32.body_idle_confirmed
            }
            follow = getattr(self, "_last_cycle", None)
            status["follow_debug"] = (
                {"reason": follow.plan.reason,
                 "head_sent": follow.head_sent,
                 "body_sent": follow.body_sent,
                 "bearing_deg": round(follow.plan.body_bearing_deg, 1),
                 "body_command": follow.plan.body_command,
                 "yaw_target_deg": round(follow.plan.yaw_target_deg, 1)}
                if follow is not None else None
            )
            status["simulation"] = False
            status["uart_poll_error"] = self._uart_poll_error
            status["uart_last_safety_event"] = self.sink.last_safety_event
            status["arm_error"] = self._last_arm_error
            status["fault_cause"] = (
                "UART_POLL_" + self._uart_poll_error if self._uart_poll_error else
                self.sink.last_safety_event if status["mode"] == "ESTOP"
                    and self.sink.last_safety_event else
                "ESP32_REPORTED_LATCHED" if status["fw_safety"] == "LATCHED" else
                "SERIAL_TRANSPORT_FAULT" if status["transport_fault"] else
                ("ESTOP_UNKNOWN__" + status["stop_reason"]) if
                    status["mode"] == "ESTOP" else None
            )
            status["simulated_esp32"] = False
            status["head_only_test"] = self.head_enabled and not self.body_enabled
            status["body_motion_permitted"] = self.body_enabled
            status["actuation_scope"] = (
                "HEAD_YAW_3DEG_AND_FEEDBACK_GATED_BODY_TURNS" if self.body_enabled
                else "HEAD_YAW_ONLY_3DEG" if self.head_enabled
                else "MONITOR_ONLY"
            )
            status["servo_power_declared"] = self.power_declared
            status["motor_status"] = (
                "ON申告・首Yaw±3°/左右旋回（動作完了待ち）"
                if self.body_enabled else
                "ON申告・首Yaw±3°のみ" if self.head_enabled else
                "モーター指令無効（通電状態は未測定）"
            )
            status["auto_ready"] = bool(
                status["auto_ready"] and self.head_enabled
                and presenter["presenter_selected"]
                and presenter["lock_state"] == "TRACKING"
                and status["mode"] == Mode.SAFE.value
                and status["fw_safety"] == "DISARMED"
                and not self.gateway.firmware_arm_pending("AUTO")
                and not status["transport_fault"]
            )
            return status

    def action(self, action: str) -> dict:
        with self._guard:
            if action == "manual":
                return {"ok": False, "error": "BODY_MOVEMENT_NOT_APPROVED"}
            if action in {"auto", "reset"} and not self.head_enabled:
                return {"ok": False, "error": "SERIAL_PROBE_ONLY"}
            if action == "auto":
                state = self.gateway.status().safety.mode
                fw_state = self.gateway.firmware_safety_status()
                if state == Mode.ESTOP:
                    return {"ok": False, "error": "ESTOP_LATCHED_RESET_REQUIRED"}
                if state == Mode.AUTO:
                    # CRITICAL: duplicate SAFETY_ARM while already ARMED is
                    # rejected by firmware and can lead to ESTOP. Idempotent.
                    return {"ok": True, "already_active": True}
                if self.gateway.firmware_arm_pending("AUTO"):
                    # Repeated UI clicks during the async handshake must not
                    # send another SAFETY_ARM or reset the deadline.
                    return {"ok": True, "pending": True}
                if state != Mode.SAFE or fw_state != "DISARMED":
                    return {"ok": False, "error": "ESP32_NOT_DISARMED_OR_NOT_READY"}
            response = super().action(action)
            if action == "auto" and response.get("pending"):
                self._arm_wait_since = time.monotonic()
                self._last_arm_error = None
            return response

    def manual(self, direction: str) -> dict:
        if direction == "stop":
            return self.action("stop")
        return {"ok": False, "error": "BODY_MOVEMENT_NOT_APPROVED"}


def main(argv: Optional[list[str]] = None) -> None:
    ap = argparse.ArgumentParser(
        description="Real ESP32 integrated control; safe default UART diagnostics only")
    ap.add_argument("--serial-port", required=True)
    ap.add_argument("--pc-host", default=None,
                    help="Optional Pi camera via TCP/SSH to RTX4060 PC")
    ap.add_argument("--pc-port", type=int, default=8765)
    ap.add_argument("--camera-index", type=int, default=0)
    ap.add_argument("--port", type=int, default=8766)
    power_group = ap.add_mutually_exclusive_group(required=True)
    power_group.add_argument("--servo-power-on", action="store_true",
                             help="Servo V+ is ON (operator declaration, NOT telemetry)")
    power_group.add_argument("--servo-power-off-confirmed", action="store_true",
                             help="Legacy monitor-only path; servo V+ physically OFF")
    ap.add_argument("--enable-head-test", action="store_true",
                    help="Only under secured/calibrated bench test; max yaw +/-3deg")
    ap.add_argument("--enable-body-turns", action="store_true",
                    help="Repeated body turns gated by fresh ESP32 idle feedback")
    ap.add_argument("--body-turn-calibrated", action="store_true")
    ap.add_argument("--bench-secured", action="store_true")
    ap.add_argument("--neutral-calibrated", action="store_true")
    ap.add_argument("--physical-cutoff-tested", action="store_true")
    args = ap.parse_args(argv)
    if args.enable_body_turns and not args.enable_head_test:
        ap.error("Body turns require --enable-head-test")
    if args.enable_body_turns and not args.body_turn_calibrated:
        ap.error("Body turns require verified body-turn calibration")
    if args.enable_head_test and not args.servo_power_on:
        ap.error("Head motor test requires explicitly declared servo V+ ON")
    if args.enable_head_test and not (
        args.bench_secured and args.neutral_calibrated
        and args.physical_cutoff_tested
    ):
        ap.error("Head actuation requires physical bench, neutral and cutoff verification")
    if args.port != 8766:
        ap.error("Use the established loopback UI port 8766")
    if args.pc_host and args.pc_host != "127.0.0.1":
        ap.error("Use the existing SSH loopback tunnel, not a public TCP detector")
    if not args.serial_port.startswith("/dev/serial/by-id/"):
        ap.error("Use stable Pi /dev/serial/by-id/... device, not guessed ttyUSB")

    runtime = LockedRealRuntime(
        args.serial_port,
        head_enabled=args.enable_head_test,
        body_enabled=args.enable_body_turns,
        power_declared="ON_OPERATOR_DECLARED" if args.servo_power_on else "OFF_OPERATOR_DECLARED",
    )
    worker = None
    server = None
    try:
        runtime.start()
        if args.pc_host:
            worker = CameraInferenceWorker(
                runtime, args.pc_host, port=args.pc_port,
                camera_index=args.camera_index,
            )
            worker.start()
        token = secrets.token_urlsafe(32)
        server = OperatorServer(("127.0.0.1", args.port), runtime, token)
        print("REAL ESP32 UART. Real servo is NOT physically verified by software.")
        print("Mode:", (
            "SUPERVISED YAW +/-3deg AND REPEATED FEEDBACK-GATED BODY TURNS"
            if args.enable_body_turns else
            "SUPERVISED YAW +/-3deg" if args.enable_head_test else
            "SERIAL PROBE; NO ARM"
        ))
        print("Servo V+:", runtime.power_declared, "(operator-declared, not electrically sensed)")
        print("STOP/ESTOP do not remove power; keep physical cutoff accessible.")
        print(f"Operator URL: http://127.0.0.1:{args.port}/")
        print("Operator token:", token)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n[Real ESP32 integration] Shutdown.")
    finally:
        if worker is not None:
            worker.close()
            worker.join(timeout=1.)
        if server is not None:
            server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
