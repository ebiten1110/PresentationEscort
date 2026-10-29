"""Presentation Escort operator dashboard (dry-run / loopback only).

Run on Pi:
  python3 experiments/pc_gpu/pi_operator_console.py
Access through SSH tunnel:
  ssh -L 8766:127.0.0.1:8766 pi@raspberrypi.local

This tool NEVER opens an ESP32 serial port. Physical actuation is explicitly
absent, not merely hidden behind a flag. It shares PiMotionGateway's safety
logic with future hardware control. Manual steering is simulated in memory.
"""
from __future__ import annotations

import argparse
import hmac
import json
import secrets
import socket
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlsplit

from detection_protocol import DetectionResult, recv_json, send_message
from motion_gateway import MemoryCommandSink, PiMotionGateway
from remote_person_controller import RemotePersonController
from safety_manager import Mode

ASSETS = Path(__file__).resolve().parent
MAX_BODY = 1024
VALID_ACTIONS = frozenset({"auto", "manual", "stop", "estop", "reset"})
MANUAL_ACTIONS = {
    "left": "TRACK_LEFT",
    "right": "TRACK_RIGHT",
    "stop": "STOP",
}


class OperatorRuntime:
    """Thread-safe state around the existing safety gate, memory-only sink."""

    def __init__(self) -> None:
        self.sink = MemoryCommandSink()
        self.gateway = PiMotionGateway(self.sink)
        # No path from this runtime to hardware even if the UI is compromised.
        self.controller = RemotePersonController(self.gateway, enable_actuation=False)
        self._guard = threading.RLock()
        self._camera_active = False
        self._connected = False
        self._target_seen = False
        self._last_rtt_ms: Optional[float] = None
        self._last_inference_ms: Optional[float] = None
        self._last_event = "READY_DRY_RUN"
        self._last_cycle = None
        self._started = False

    def start(self) -> None:
        with self._guard:
            if not self._started:
                self.gateway.start()
                self._started = True

    def status(self) -> dict:
        with self._guard:
            gs = self.gateway.status()
            return {
                "simulation": True,
                "mode": gs.safety.mode.value,
                "reason": gs.safety.reason,
                "auto_ready": gs.safety.auto_ready,
                "confirmed_frames": gs.safety.confirmed_frames,
                "target_found": self._target_seen,
                "pc_connected": self._connected,
                "camera_active": self._camera_active,
                "rtt_ms": self._last_rtt_ms,
                "inference_ms": self._last_inference_ms,
                "last_command": gs.last_command,
                "transport_fault": gs.transport_fault,
                "stop_reason": gs.stop_reason,
                "fault_cause": getattr(gs, "fault_cause", None),
                "arm_error": getattr(gs, "arm_error", None),
                "fw_arm_pending": self.gateway.firmware_arm_pending("AUTO"),
                "uart_poll_error": getattr(self, "_uart_poll_error", None),
                "fw_safety": self.gateway.firmware_safety_status(),
                "message": self._last_event,
            }

    def set_camera_state(self, active: bool, connected: bool) -> None:
        with self._guard:
            self._camera_active, self._connected = active, connected
            if not connected:
                self._target_seen = False
                self.gateway.connection_lost()

    def process_detection(self, detection: DetectionResult, rtt_ms: float) -> None:
        with self._guard:
            cycle = self.controller.process(detection, rtt_ms=rtt_ms)
            self._last_cycle = cycle
            self._target_seen = cycle.target_valid
            self._last_rtt_ms = round(float(rtt_ms), 1)
            self._last_inference_ms = round(float(detection.inference_ms), 1)
            self._last_event = "PERSON_FOUND" if cycle.target_valid else "PERSON_LOST"

    def action(self, action: str) -> dict:
        with self._guard:
            if action not in VALID_ACTIONS:
                return {"ok": False, "error": "UNKNOWN_ACTION"}
            if action == "estop":
                self.gateway.emergency_stop()
                self._last_event = "EMERGENCY_STOP"
                return {"ok": True}
            if action == "stop":
                self.gateway.request_motion("STOP", source="MANUAL")
                self._last_event = "OPERATOR_STOP"
                return {"ok": True}
            if action == "reset":
                # Simulation-only operator reset, no physical device involved.
                self.gateway.reset_emergency_stop()
                self._last_event = "RESET_REQUESTED"
                return {"ok": self.gateway.status().safety.mode == Mode.SAFE}
            if action == "auto":
                ok = self.gateway.arm_auto()
                self._last_event = "AUTO_ARMED" if ok else "AUTO_ARM_DENIED"
                return {"ok": ok, "error": None if ok else "TARGET_NOT_READY"}
            if action == "manual":
                ok = self.gateway.arm_manual()
                self._last_event = "MANUAL_ARMED" if ok else "MANUAL_ARM_DENIED"
                return {"ok": ok, "error": None if ok else "SAFETY_BLOCKED"}
            raise AssertionError("unreachable")

    def manual(self, direction: str) -> dict:
        with self._guard:
            command = MANUAL_ACTIONS.get(direction)
            if command is None:
                return {"ok": False, "error": "UNSUPPORTED_DIRECTION"}
            if command == "STOP":
                return self.action("stop")
            decision = self.gateway.request_motion(command, source="MANUAL")
            self._last_event = "MANUAL_" + (direction.upper() if decision.approved else "DENIED")
            return {"ok": decision.approved, "error": None if decision.approved else decision.reason}

    def close(self) -> None:
        with self._guard:
            self.gateway.close()


class CameraInferenceWorker(threading.Thread):
    """Optional Pi webcam->PC inference in DRY RUN; no ESP32 device access."""

    def __init__(
        self,
        runtime: OperatorRuntime,
        host: str,
        *,
        port: int = 8765,
        camera_index: int = 0,
        timeout_s: float = 30.0,  # First CUDA inference can exceed 10s; Pi safety tick runs independently.
        camera_width: int = 640,
        camera_height: int = 480,
        camera_fps: int = 30,
        jpeg_quality: int = 80,
        camera_mjpeg: bool = False,
        preview_every_n_frames: int = 1,
    ) -> None:
        super().__init__(name="camera-inference-dry-run", daemon=True)
        self.runtime = runtime
        self.host = host
        self.port = port
        self.camera_index = camera_index
        self.timeout_s = timeout_s
        if camera_width < 160 or camera_width > 1920 or camera_height < 120 or camera_height > 1080:
            raise ValueError("camera resolution outside supported range")
        if not 30 <= jpeg_quality <= 95:
            raise ValueError("jpeg quality outside supported range")
        if not 5 <= camera_fps <= 60:
            raise ValueError("camera fps request outside supported range")
        if type(preview_every_n_frames) is not int or not 1 <= preview_every_n_frames <= 10:
            raise ValueError("invalid preview decimation")
        self.preview_every_n_frames = preview_every_n_frames
        self._completed_inferences = 0
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps
        self.jpeg_quality = jpeg_quality
        self.camera_mjpeg = camera_mjpeg
        self.done = threading.Event()

    def close(self) -> None:
        self.done.set()

    def run(self) -> None:
        import cv2  # lazy dependency: UI can run on PC without OpenCV.
        while not self.done.is_set():
            cam = cv2.VideoCapture(self.camera_index)
            if self.camera_mjpeg:
                # Optional USB UVC MJPEG negotiation, never assumed to work.
                # Some cameras deliver substantially fewer FPS in YUYV.
                cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            # This is a *request*, not an achievable-FPS guarantee. The
            # USB camera, Pi JPEG encode, PC inference, and synchronous TCP
            # roundtrip jointly determine the actual frame rate.
            cam.set(cv2.CAP_PROP_FPS, self.camera_fps)
            cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            report_cap = getattr(self.runtime, "set_capture_diagnostics", None)
            if callable(report_cap) and cam.isOpened():
                actual_fourcc = int(cam.get(cv2.CAP_PROP_FOURCC))
                fourcc = "".join(chr((actual_fourcc >> (8 * n)) & 255)
                                 for n in range(4)).strip("\x00")
                report_cap({"requested_mjpeg": self.camera_mjpeg,
                            "fourcc": fourcc,
                            "reported_fps": cam.get(cv2.CAP_PROP_FPS)})
            self.runtime.set_camera_state(cam.isOpened(), False)
            try:
                if not cam.isOpened():
                    self.done.wait(1.0)
                    continue
                with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
                    sock.settimeout(self.timeout_s)
                    self.runtime.set_camera_state(True, True)
                    while not self.done.is_set():
                        started = time.monotonic()
                        ok, frame = cam.read()
                        captured_at = time.monotonic()
                        if not ok:
                            raise RuntimeError("camera frame unavailable")
                        ok, compressed = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                        encoded_at = time.monotonic()
                        if not ok:
                            raise RuntimeError("JPEG encode failed")
                        send_message(sock, compressed.tobytes())
                        result = DetectionResult.from_dict(recv_json(sock))
                        returned_at = time.monotonic()
                        rtt_ms = (returned_at - started) * 1000.0
                        report_timing = getattr(self.runtime, "set_capture_timing", None)
                        if callable(report_timing):
                            report_timing({
                                "capture_ms": round((captured_at - started) * 1000, 1),
                                "encode_ms": round((encoded_at - captured_at) * 1000, 1),
                                "remote_infer_roundtrip_ms": round((returned_at - encoded_at) * 1000, 1),
                            })
                        self.runtime.process_detection(result, rtt_ms)
                        # Drawing and re-JPEG-encoding the operator preview
                        # costs ~25ms/frame on Pi. Keep EVERY inference and
                        # safety observation; throttle display work only.
                        self._completed_inferences += 1
                        if self._completed_inferences % self.preview_every_n_frames == 0:
                            publish = getattr(self.runtime, "publish_preview", None)
                            if callable(publish):
                                publish(frame, result)
            except (OSError, ConnectionError, TimeoutError, RuntimeError, ValueError) as exc:
                with self.runtime._guard:
                    self.runtime._last_event = "PC_LINK_ERROR_" + type(exc).__name__
            finally:
                self.runtime.set_camera_state(False, False)
                cam.release()
            self.done.wait(1.0)


class OperatorServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], runtime: OperatorRuntime, token: str):
        if address[0] not in ("127.0.0.1", "localhost"):
            raise ValueError("dashboard must bind to loopback, use SSH tunnel")
        super().__init__(address, OperatorHandler)
        if not token or len(token) < 24:
            self.server_close()
            raise ValueError("strong operator token required")
        self.runtime, self.token = runtime, token


class OperatorHandler(BaseHTTPRequestHandler):
    server: OperatorServer
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args) -> None:
        # Do not log authorization tokens or request bodies.
        pass

    def respond(self, code: int, body: bytes, content_type: str = "application/json") -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        # The authenticated preview uses URL.createObjectURL(JPEG blob).
        # Without this narrowly scoped img-src exception, the browser shows
        # a broken-image icon despite a successful /api/preview.jpg response.
        self.send_header("Content-Security-Policy", "default-src 'none'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def json(self, status: int, payload: dict) -> None:
        self.respond(status, json.dumps(payload, separators=(",", ":")).encode("utf-8"))

    def check_host(self) -> bool:
        # Defend loopback service against DNS rebinding / cross-origin abuse.
        host = self.headers.get("Host", "")
        return host in (
            f"127.0.0.1:{self.server.server_port}",
            f"localhost:{self.server.server_port}",
        )

    def authenticated(self) -> bool:
        value = self.headers.get("Authorization", "")
        return hmac.compare_digest(value, "Bearer " + self.server.token)

    def origin_allowed(self) -> bool:
        origin = self.headers.get("Origin")
        return origin is None or origin in (
            f"http://127.0.0.1:{self.server.server_port}",
            f"http://localhost:{self.server.server_port}",
        )

    def do_GET(self) -> None:
        if not self.check_host():
            return self.json(403, {"error": "FORBIDDEN_HOST"})
        path = urlsplit(self.path).path
        if path == "/":
            return self.respond(200, (ASSETS / "operator_console.html").read_bytes(), "text/html; charset=utf-8")
        if path == "/ui.js":
            return self.respond(200, (ASSETS / "operator_console.js").read_bytes(), "application/javascript; charset=utf-8")
        if path == "/ui.css":
            return self.respond(200, (ASSETS / "operator_console.css").read_bytes(), "text/css; charset=utf-8")
        if path == "/api/preview.jpg":
            if not self.authenticated():
                return self.json(401, {"error": "UNAUTHORIZED"})
            preview = getattr(self.server.runtime, "get_preview", None)
            if not callable(preview):
                return self.json(404, {"error": "PREVIEW_UNAVAILABLE"})
            image = preview()
            return self.respond(200, image, "image/jpeg") if image else self.json(404, {"error": "NO_CAMERA_FRAME"})
        if path == "/api/diagnostics.zip":
            if not self.authenticated():
                return self.json(401, {"error": "UNAUTHORIZED"})
            exporter = getattr(self.server.runtime, "get_diagnostic_bundle", None)
            if not callable(exporter):
                return self.json(404, {"error": "DIAGNOSTICS_DISABLED"})
            data = exporter()
            return (self.respond(200, data, "application/zip") if data is not None
                    else self.json(404, {"error": "DIAGNOSTICS_DISABLED"}))
        if path == "/api/status":
            if not self.authenticated():
                return self.json(401, {"error": "UNAUTHORIZED"})
            return self.json(200, self.server.runtime.status())
        return self.json(404, {"error": "NOT_FOUND"})

    def do_POST(self) -> None:
        if not self.check_host() or not self.origin_allowed():
            return self.json(403, {"error": "FORBIDDEN_ORIGIN"})
        if not self.authenticated():
            return self.json(401, {"error": "UNAUTHORIZED"})
        path = urlsplit(self.path).path
        if path not in ("/api/action", "/api/manual", "/api/target"):
            return self.json(404, {"error": "NOT_FOUND"})
        if self.headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json":
            return self.json(415, {"error": "JSON_ONLY"})
        try:
            size = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            return self.json(400, {"error": "INVALID_CONTENT_LENGTH"})
        if not 1 <= size <= MAX_BODY:
            self.close_connection = True
            return self.json(413, {"error": "INVALID_BODY_SIZE"})
        try:
            obj = json.loads(self.rfile.read(size))
        except (ValueError, UnicodeDecodeError):
            return self.json(400, {"error": "INVALID_JSON"})
        if not isinstance(obj, dict):
            return self.json(400, {"error": "INVALID_REQUEST"})
        if path == "/api/target":
            if not (set(obj) == {"index", "frame_seq"} or obj == {"release": True}):
                return self.json(400, {"error": "INVALID_TARGET_REQUEST"})
            handler = getattr(self.server.runtime, "target_action", None)
            if not callable(handler):
                return self.json(409, {"error": "PRESENTER_LOCK_NOT_AVAILABLE"})
            result = handler(obj)
            return self.json(200 if result["ok"] else 409, result)
        if len(obj) != 1:
            return self.json(400, {"error": "INVALID_REQUEST"})
        if path == "/api/action":
            if not isinstance(obj.get("action"), str):
                return self.json(400, {"error": "INVALID_ACTION"})
            result = self.server.runtime.action(obj["action"])
        else:
            if not isinstance(obj.get("direction"), str):
                return self.json(400, {"error": "INVALID_DIRECTION"})
            result = self.server.runtime.manual(obj["direction"])
        return self.json(200 if result["ok"] else 409, result)


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="LOCAL DRY-RUN console; no ESP32 hardware")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--pc-host", default=None, help="Enable Pi USB camera -> GPU PC dry-run")
    parser.add_argument("--pc-port", type=int, default=8765)
    parser.add_argument("--camera-index", type=int, default=0)
    args = parser.parse_args(argv)

    token = secrets.token_urlsafe(32)
    runtime = OperatorRuntime()
    runtime.start()
    worker: Optional[CameraInferenceWorker] = None
    server: Optional[OperatorServer] = None
    try:
        if args.pc_host:
            worker = CameraInferenceWorker(runtime, args.pc_host, port=args.pc_port, camera_index=args.camera_index)
            worker.start()
        server = OperatorServer(("127.0.0.1", args.port), runtime, token)
        print("Presentation Escort — SIMULATION ONLY; no ESP32 serial access.")
        print(f"Operator URL: http://127.0.0.1:{args.port}/")
        print("Operator token (do not share):", token)
        print(f"From PC: ssh -L {args.port}:127.0.0.1:{args.port} pi@<pi-address>")
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n[Console] Stopping.")
    finally:
        if worker is not None:
            worker.close()
        if server is not None:
            server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
