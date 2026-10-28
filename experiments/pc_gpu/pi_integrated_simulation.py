"""Phase 9: integrated operator UI + PC detector + simulated ESP32.

All actuator commands terminate in VirtualEsp32 (bytearrays in memory).
There is deliberately NO import of SerialCommandSink, pyserial, GPIO or ESP32
upload facility. A simulated ARM response is never physical authorization.
"""
from __future__ import annotations

import argparse
import secrets
import threading
from typing import Optional

from esp32_feedback_poller import Esp32FeedbackPoller
from pi_operator_console import (
    CameraInferenceWorker,
    OperatorRuntime,
    OperatorServer,
)
from remote_person_controller import RemotePersonController
from motion_gateway import PiMotionGateway
from safety_manager import Mode
from virtual_esp32 import VirtualEsp32


class IntegratedSimulationRuntime(OperatorRuntime):
    """One logical process owns camera results, UI, safety and fake UART."""

    def __init__(self) -> None:
        # Deliberately do not call super(): it installs the Phase 7
        # memory-only sink. Here the sink models the full ESP32 V1 wire protocol.
        self.sink = VirtualEsp32()
        self.gateway = PiMotionGateway(self.sink)
        # enable_actuation=True *only* talks to the in-memory VirtualEsp32.
        self.controller = RemotePersonController(self.gateway, enable_actuation=True)
        self._feedback = Esp32FeedbackPoller(self.sink, self.controller)
        self._guard = threading.RLock()
        self._camera_active = False
        self._connected = False
        self._target_seen = False
        self._last_rtt_ms = None
        self._last_inference_ms = None
        self._last_event = "VIRTUAL_ESP32_OFFLINE"
        self._uart_poll_error: Optional[str] = None
        self._started = False
        self._exit = threading.Event()
        self._feedback_thread: Optional[threading.Thread] = None
        self._reset_requested = False

    def start(self) -> None:
        with self._guard:
            if self._started:
                return
            self.gateway.start()
            self._started = True
            self._last_event = "VIRTUAL_ESP32_READY"
            self._feedback_thread = threading.Thread(
                target=self._poll_loop, daemon=True, name="virtual-esp32-feedback"
            )
            self._feedback_thread.start()

    def _poll_loop(self) -> None:
        while not self._exit.wait(0.025):
            try:
                with self._guard:
                    self._feedback.poll()
                    if self._reset_requested:
                        self.gateway.reset_emergency_stop()
                        if self.gateway.status().safety.mode == Mode.SAFE:
                            self._reset_requested = False
                            self._last_event = "VIRTUAL_ESTOP_RESET_TO_SAFE"
            except Exception as exc:
                with self._guard:
                    self._uart_poll_error = type(exc).__name__
                    self.gateway.emergency_stop(cause="UART_POLL_" + type(exc).__name__)
                    self._last_event = "UART_POLL_FAILED_" + type(exc).__name__
                return

    def action(self, action: str) -> dict:
        with self._guard:
            if action in ("auto", "manual"):
                if self.gateway.status().safety.mode == Mode.ESTOP:
                    return {"ok": False, "error": "ESTOP_LATCHED"}
                accepted = (
                    self.gateway.arm_auto() if action == "auto"
                    else self.gateway.arm_manual()
                )
                pending = self.gateway.firmware_arm_pending(action.upper())
                self._last_event = (
                    action.upper() + "_VIRTUAL_ARMED" if accepted
                    else action.upper() + "_VIRTUAL_ARM_PENDING" if pending
                    else action.upper() + "_DENIED"
                )
                return {
                    "ok": accepted or pending,
                    "pending": pending,
                    "error": None if accepted or pending else "ARM_NOT_READY",
                }
            if action == "reset":
                if self.gateway.status().safety.mode != Mode.ESTOP:
                    return {"ok": False, "error": "NOT_ESTOP"}
                if not self.gateway.request_firmware_reset():
                    return {"ok": False, "error": "VIRTUAL_RESET_REJECTED"}
                self._reset_requested = True
                self._last_event = "VIRTUAL_RESET_PENDING"
                return {"ok": True, "pending": True}
            self._reset_requested = False if action == "estop" else self._reset_requested
            return super().action(action)

    def status(self) -> dict:
        with self._guard:
            result = super().status()
            result["simulated_esp32"] = True
            result["fw_safety"] = "VIRTUAL:" + result["fw_safety"]
            result["simulated_commands"] = len(self.sink.commands)
            return result

    def close(self) -> None:
        self._exit.set()
        feedback = self._feedback_thread
        if feedback is not None and feedback is not threading.current_thread():
            feedback.join(timeout=1.0)
        super().close()


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description="Integrated PC detector / safety / virtual ESP32 only"
    )
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--pc-host", default=None)
    parser.add_argument("--pc-port", type=int, default=8765)
    parser.add_argument("--camera-index", type=int, default=0)
    args = parser.parse_args(argv)

    token = secrets.token_urlsafe(32)
    runtime = IntegratedSimulationRuntime()
    worker: Optional[CameraInferenceWorker] = None
    server: Optional[OperatorServer] = None
    runtime.start()
    try:
        if args.pc_host:
            worker = CameraInferenceWorker(
                runtime, args.pc_host, port=args.pc_port,
                camera_index=args.camera_index
            )
            worker.start()
        server = OperatorServer(("127.0.0.1", args.port), runtime, token)
        print("INTEGRATED SIMULATION — commands go ONLY to virtual ESP32.")
        print("No serial port or physical motors can be accessed in this mode.")
        print(f"Operator URL: http://127.0.0.1:{args.port}/")
        print("Operator token (do not share):", token)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n[Integrated simulation] Stopping.")
    finally:
        if worker is not None:
            worker.close()
            worker.join(timeout=1.0)
        if server is not None:
            server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
