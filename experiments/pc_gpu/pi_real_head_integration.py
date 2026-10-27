"""Controlled Pi camera -> PC YOLO -> presenter lock -> REAL ESP32 bridge.

Default: UART telemetry probe only with servo V+ physically OFF. No ARM.
Explicit supervised mode: ONLY head YAW <=3 deg (pitch stays zero), NO body.
No automatic forward/back, no uncalibrated head operation, no public HTTP.
This is a guarded development bridge, NOT approval for show-floor operation.
"""
from __future__ import annotations

import argparse
import math
import re
import secrets
import threading
from dataclasses import replace
from typing import Optional

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
    def step(self, *args, **kwargs):
        plan = super().step(*args, **kwargs)
        # Remove every body command and keep the head's initial travel tiny.
        return replace(
            plan, body_command=None, suggested_body_turn=None,
            yaw_target_deg=max(-3.0, min(3.0, plan.yaw_target_deg)),
            pitch_target_deg=0.0,
        )


class HeadOnlySerialSink:
    requires_watchdog = True
    requires_firmware_safety = True

    def __init__(self, port: str, *, head_enabled: bool):
        self._inner = SerialCommandSink(port)
        self.head_enabled = head_enabled

    def send(self, command: str) -> None:
        cmd = command.strip().upper()
        if cmd.startswith("TRACK_") or cmd in {"LEFT", "RIGHT", "FORWARD", "BACKWARD", "STAND"}:
            raise ValueError("body and translation commands are physically disabled")
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
        return self._inner.read_available(max_bytes=max_bytes)

    def close(self) -> None:
        self._inner.close()


class LockedRealRuntime(LockedSimulationRuntime):
    """Reuse real V2 presenter-lock logic and safety gateway without a virtual MCU."""

    def __init__(self, serial_port: str, *, head_enabled: bool = False) -> None:
        super().__init__()
        # Superclass has not started any threads or issued any commands.
        self.sink.close()
        self.sink = HeadOnlySerialSink(serial_port, head_enabled=head_enabled)
        self.gateway = PiMotionGateway(self.sink)
        self.controller = RemotePersonController(
            self.gateway, planner=HeadOnlyPlanner(),
            enable_actuation=head_enabled,
        )
        self._feedback = Esp32FeedbackPoller(self.sink, self.controller)
        self.head_enabled = head_enabled
        self._last_event = "REAL_UART_READY_HEAD_DISABLED" if not head_enabled else "REAL_HEAD_TEST_READY"

    def status(self) -> dict:
        with self._guard:
            # Bypass IntegratedSimulationRuntime.status(): avoid misleading VIRTUAL UI.
            status = OperatorRuntime.status(self)
            presenter = self.presenter.display(now=__import__("time").monotonic())
            status.update(presenter)
            status["simulation"] = False
            status["simulated_esp32"] = False
            status["head_only_test"] = self.head_enabled
            status["body_motion_permitted"] = False
            status["auto_ready"] = bool(
                status["auto_ready"] and self.head_enabled
                and presenter["presenter_selected"]
                and presenter["lock_state"] == "TRACKING"
            )
            return status

    def action(self, action: str) -> dict:
        with self._guard:
            if action == "manual":
                return {"ok": False, "error": "BODY_MOVEMENT_NOT_APPROVED"}
            if action in {"auto", "reset"} and not self.head_enabled:
                return {"ok": False, "error": "SERIAL_PROBE_ONLY"}
            return super().action(action)

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
    ap.add_argument("--servo-power-off-confirmed", action="store_true", required=True)
    ap.add_argument("--enable-head-test", action="store_true",
                    help="Only under secured/calibrated bench test; max yaw +/-3deg")
    ap.add_argument("--bench-secured", action="store_true")
    ap.add_argument("--neutral-calibrated", action="store_true")
    ap.add_argument("--physical-cutoff-tested", action="store_true")
    args = ap.parse_args(argv)
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

    runtime = LockedRealRuntime(args.serial_port, head_enabled=args.enable_head_test)
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
        print("REAL ESP32 UART. BODY MOTION BLOCKED. Real servo is NOT approved by software.")
        print("Mode:", "SUPERVISED YAW +/-3deg" if args.enable_head_test else "SERIAL PROBE; NO ARM")
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
