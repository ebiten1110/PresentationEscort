"""Windows USB-direct, offline Presentation Escort.

USB camera -> local CUDA YOLO Pose / ByteTrack -> existing LockedRealRuntime
-> USB serial ESP32. No Pi, Wi-Fi, SSH, TCP detection server, or cloud needed.

Defaults to MONITOR: real ESP32 telemetry is read, but actuation is disabled.
Head/body modes require independent, affirmative physical safety checks.
The existing firmware safety handshake and gateway watchdog are reused.
"""
from __future__ import annotations

import argparse
import math
import secrets
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from pi_operator_console import OperatorServer
from pi_real_head_integration import LockedRealRuntime


class LocalUsbCameraWorker(threading.Thread):
    """Capture and inference locally; never queue stale frames for actuation."""

    def __init__(self, runtime: LockedRealRuntime, detector, *,
                 camera_index: int, backend: str, image_width: int,
                 image_height: int, camera_fps: int,
                 preview_every: int) -> None:
        super().__init__(name="windows-usb-camera-inference", daemon=True)
        self.runtime = runtime
        self.detector = detector
        self.camera_index = camera_index
        self.backend = backend
        self.image_width = image_width
        self.image_height = image_height
        self.camera_fps = camera_fps
        self.preview_every = preview_every
        self.done = threading.Event()
        self._frames = 0

    def close(self) -> None:
        self.done.set()

    def _open_camera(self):
        import cv2
        choices = (["dshow", "msmf"] if self.backend == "auto"
                   else [self.backend])
        backend_codes = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}
        for name in choices:
            for request_mjpeg in (True, False):
                cam = cv2.VideoCapture(self.camera_index, backend_codes[name])
                if not cam.isOpened():
                    cam.release()
                    continue
                # MJPEG is optional. Fall back to camera-native format if it
                # causes the same stream errors seen on the Pi.
                if request_mjpeg:
                    cam.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*"MJPG"))
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
                cam.set(cv2.CAP_PROP_FPS, self.camera_fps)
                cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, frame = cam.read()
                if ok and frame is not None:
                    print(f"[Camera] {name} index={self.camera_index} "
                          f"mjpeg={request_mjpeg} "
                          f"actual={frame.shape[1]}x{frame.shape[0]}",
                          flush=True)
                    self.runtime.set_capture_diagnostics({
                        "local_inference": True,
                        "backend": name,
                        "camera_index": self.camera_index,
                        "requested_mjpeg": request_mjpeg,
                        "reported_fps": cam.get(cv2.CAP_PROP_FPS),
                    })
                    return cam, frame
                print(f"[Camera] {name} mjpeg={request_mjpeg} "
                      "opened but first read failed", flush=True)
                cam.release()
        raise RuntimeError(
            "USB_CAMERA_UNAVAILABLE: verify camera index, Windows camera "
            "permission, USB data cable, and other apps using the camera"
        )

    def run(self) -> None:
        import cv2
        camera = None
        try:
            # Opening the camera is non-actuating. No motion until operator ARM.
            camera, first_frame = self._open_camera()
            self.detector.reset_tracking()
            self.runtime.set_camera_state(True, True)
            frame = first_frame
            while not self.done.is_set():
                started = time.monotonic()
                if frame is None:
                    ok, frame = camera.read()
                    if not ok or frame is None:
                        raise RuntimeError("USB_CAMERA_FRAME_LOST")
                captured_at = time.monotonic()
                # A local Python call; no JPEG encoding, sockets or SSH tunnel.
                result = self.detector.detect(frame)
                detected_at = time.monotonic()
                elapsed_ms = (detected_at - started) * 1000.0
                # SafetyManager enforces 500 ms max result age and a separate
                # target watchdog. Never pretend a slow frame was fresh.
                self.runtime.set_capture_timing({
                    "capture_ms": round((captured_at - started) * 1000.0, 1),
                    "local_infer_ms": round((detected_at - captured_at) * 1000.0, 1),
                    "local_total_ms": round(elapsed_ms, 1),
                })
                self.runtime.process_detection(result, elapsed_ms)
                self._frames += 1
                if self._frames % self.preview_every == 0:
                    self.runtime.publish_preview(frame, result)
                frame = None
        except Exception as exc:
            print(f"[Camera/GPU] stopped safely: {type(exc).__name__}: {exc}",
                  file=sys.stderr, flush=True)
            with self.runtime._guard:
                self.runtime._last_event = "LOCAL_CAMERA_OR_GPU_ERROR_" + type(exc).__name__
        finally:
            # Fail closed on capture/AI failure or operator shutdown.
            self.runtime.set_camera_state(False, False)
            if camera is not None:
                camera.release()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="USB camera + local CUDA YOLO + USB ESP32, no network")
    parser.add_argument("--serial-port", required=True,
                        help="Confirmed ESP32 USB COM port, for example COM8")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-backend", choices=["auto", "dshow", "msmf"],
                        default="auto")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--confidence", type=float, default=0.45)
    parser.add_argument("--model", default="yolo26n-pose.pt",
                        help="Existing local pose model checkpoint")
    parser.add_argument("--preview-every", type=int, default=2)
    parser.add_argument("--mode", choices=["monitor", "head", "body"],
                        default="monitor",
                        help="Monitor is non-actuating; head/body require safety checks")
    parser.add_argument("--bench-secured", action="store_true")
    parser.add_argument("--neutral-calibrated", action="store_true")
    parser.add_argument("--physical-cutoff-tested", action="store_true")
    parser.add_argument("--servo-power-on-confirmed", action="store_true")
    parser.add_argument("--body-turn-calibrated", action="store_true")
    parser.add_argument("--pitch-calibrated", action="store_true")
    parser.add_argument("--capture-diagnostics", action="store_true")
    args = parser.parse_args(argv)
    if not 0 <= args.camera_index <= 20:
        parser.error("camera-index must be 0..20")
    if not 160 <= args.camera_width <= 1920 or not 120 <= args.camera_height <= 1080:
        parser.error("unsupported camera dimensions")
    if not 5 <= args.camera_fps <= 60 or not 1 <= args.preview_every <= 10:
        parser.error("invalid camera-fps / preview-every")
    if not 320 <= args.image_size <= 1280:
        parser.error("image-size must be 320..1280")
    if not math.isfinite(args.confidence) or not 0.1 <= args.confidence <= 0.95:
        parser.error("confidence must be 0.1..0.95")
    if args.mode != "monitor" and not (
            args.bench_secured and args.neutral_calibrated and
            args.physical_cutoff_tested and args.servo_power_on_confirmed and
            args.pitch_calibrated):
        parser.error("head/body require verified bench, neutral, cutoff, "
                     "servo-power, and pitch calibration flags")
    if args.mode == "body" and not args.body_turn_calibrated:
        parser.error("body mode requires physically verified body-turn calibration")
    if not Path(args.model).is_file():
        parser.error("model checkpoint not found locally: " + args.model +
                     ". Download while online or pass --model PATH")
    return args


def main(argv=None) -> None:
    args = parse_args(argv)
    from pc_detector_server import PersonDetector, ServerConfig
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable: do not fall back to silent CPU inference")
    # Reuse the same pose model and GPU tracker as the Windows preview server.
    detector = PersonDetector(ServerConfig(
        host="127.0.0.1", port=0, model=args.model,
        confidence=args.confidence, image_size=args.image_size,
        device="0", tracking=True,
    ), gesture_pose_model=args.model)
    actuating = args.mode in ("head", "body")
    runtime = LockedRealRuntime(
        args.serial_port,
        head_enabled=actuating,
        body_enabled=(args.mode == "body"),
        power_declared=("ON_OPERATOR_DECLARED" if args.servo_power_on_confirmed
                        else "UNKNOWN"),
        pitch_bounds=(-2.0, 2.0) if actuating else (0.0, 0.0),
        turn_stable_frames=3,
        turn_settle_seconds=0.7,
        turn_edge_fraction=0.15,
        reverse_body_turn_direction=(args.mode == "body"),
        relative_pitch_reference=actuating,
        distance_adaptive_head=actuating,
        responsive_follow=actuating,
        vision_verified_head=actuating,
        camera_horizontal_fov_deg=65.0,
        auto_recalibrate_brief_loss=actuating,
        head_speed_deg_s=3.5,
        pitch_speed_deg_s=2.5 if actuating else None,
        capture_diagnostics=args.capture_diagnostics,
    )
    worker = LocalUsbCameraWorker(
        runtime, detector,
        camera_index=args.camera_index, backend=args.camera_backend,
        image_width=args.camera_width, image_height=args.camera_height,
        camera_fps=args.camera_fps,
        preview_every=args.preview_every,
    )
    server: Optional[OperatorServer] = None
    try:
        runtime.start()
        worker.start()
        token = secrets.token_urlsafe(32)
        server = OperatorServer(("127.0.0.1", 8766), runtime, token)
        print(f"[Windows USB Direct] mode={args.mode.upper()} | Wi-Fi/SSH: NOT USED")
        print("Operator URL: http://127.0.0.1:8766/")
        print("Operator token:", token, flush=True)
        print("STOP does not cut servo power. Keep physical cutoff accessible.",
              flush=True)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n[Windows USB Direct] stopping", flush=True)
    finally:
        worker.close()
        worker.join(timeout=2.0)
        if server is not None:
            server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
