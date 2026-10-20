"""Phase 11 presenter-lock demo: V2 multi-person PC frames + virtual ESP32.

This is NOT face recognition, re-identification, or physical hardware control.
A trusted operator selects a visible candidate with the authenticated console.
Lost/ambiguous tracking immediately disarms AUTO, and re-selection is manual.
Commands go exclusively to VirtualEsp32 (no USB serial hardware).
"""
from __future__ import annotations

import argparse
import secrets
import time
from typing import Optional

from detection_protocol import DetectionResult
from pi_integrated_simulation import IntegratedSimulationRuntime
from pi_operator_console import CameraInferenceWorker, OperatorServer
from presenter_lock import PresenterLock
from safety_manager import Mode


class LockedSimulationRuntime(IntegratedSimulationRuntime):
    def __init__(self) -> None:
        super().__init__()
        self.presenter = PresenterLock()

    def status(self) -> dict:
        with self._guard:
            result = super().status()
            state = self.presenter.display(now=time.monotonic())
            result.update(state)
            result["auto_ready"] = (
                result["auto_ready"]
                and result["presenter_selected"]
                and result["lock_state"] == "TRACKING"
            )
            return result

    def set_camera_state(self, active: bool, connected: bool) -> None:
        with self._guard:
            if not connected:
                self.presenter.reset_stream()
            super().set_camera_state(active, connected)

    def process_detection(self, detection: DetectionResult, rtt_ms: float) -> None:
        with self._guard:
            now = time.monotonic()
            if not 0 <= rtt_ms <= 500:
                self.presenter.clear("INVALID_GPU_RTT")
            selected = self.presenter.observe(detection, now=now)
            super().process_detection(selected, rtt_ms)
            self._last_event = "PRESENTER_" + self.presenter.reason

    def action(self, action: str) -> dict:
        with self._guard:
            if action == "auto":
                view = self.presenter.display(now=time.monotonic())
                if (
                    not view["presenter_selected"]
                    or view["lock_state"] != "TRACKING"
                ):
                    return {"ok": False, "error": "PRESENTER_LOCK_REQUIRED"}
            result = super().action(action)
            if action in ("stop", "estop"):
                self.presenter.clear("EXPLICIT_OPERATOR_RESELECT_REQUIRED")
            return result

    def target_action(self, payload: dict) -> dict:
        """Authenticated operator-only selection, valid in SAFE mode."""
        with self._guard:
            if self.gateway.status().safety.mode != Mode.SAFE:
                return {"ok": False, "error": "SELECT_ONLY_WHILE_SAFE"}
            if payload == {"release": True}:
                self.presenter.clear("RELEASED_BY_OPERATOR")
                self.gateway.request_motion("STOP", source="MANUAL")
                return {"ok": True}
            if set(payload) != {"index", "frame_seq"}:
                return {"ok": False, "error": "INVALID_SELECTION"}
            ok, reason = self.presenter.select(
                payload["index"], payload["frame_seq"], now=time.monotonic()
            )
            return {"ok": ok, "error": None if ok else reason}


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Operator-selected presenter lock, virtual ESP32 only")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--pc-host", default=None)
    parser.add_argument("--pc-port", type=int, default=8765)
    parser.add_argument("--camera-index", type=int, default=0)
    args = parser.parse_args(argv)

    runtime = LockedSimulationRuntime()
    token = secrets.token_urlsafe(32)
    worker: Optional[CameraInferenceWorker] = None
    server: Optional[OperatorServer] = None
    runtime.start()
    try:
        if args.pc_host:
            worker = CameraInferenceWorker(runtime, args.pc_host, port=args.pc_port, camera_index=args.camera_index)
            worker.start()
        server = OperatorServer(("127.0.0.1", args.port), runtime, token)
        print("PHASE 11: operator-presenter LOCK, virtual ESP32 ONLY.")
        print("This is geometry continuity, NOT verified person identity.")
        print(f"Operator URL: http://127.0.0.1:{args.port}/")
        print("Operator token:", token)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n[Presenter Lock] Stopped.")
    finally:
        if worker is not None:
            worker.close()
            worker.join(timeout=1.)
        if server is not None:
            server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
