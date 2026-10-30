"""Live Windows preview for the Presentation Escort PC/GPU YOLO server.

Run beside experiments/pc_gpu/pc_detector_server.py on the development branch.
This is an alternative entry point, keeping the existing V2 TCP/JPEG/JSON
protocol unchanged. It never opens the ESP32 or writes motion commands.

Controls: Q or Escape to exit. Window close also stops the preview.
"""
from __future__ import annotations

import argparse
import socket
import time
from typing import Any

import cv2
import numpy as np

from pc_detector_server import PersonDetector, ServerConfig, decode_jpeg
from detection_protocol import recv_message, send_json

WINDOW_TITLE = "Presentation Escort | YOLO Person Preview"


def annotate(frame: np.ndarray, result: Any) -> np.ndarray:
    """Overlay bounding boxes from the actual YOLO result sent to the Pi."""
    image = frame.copy()
    height, width = image.shape[:2]
    people = getattr(result, "people", None) or []

    # Every V2 person candidate is drawn; detection is not identity recognition.
    for index, person in enumerate(people, 1):
        x1 = max(0, min(width - 1, int(person["x1"])))
        y1 = max(0, min(height - 1, int(person["y1"])))
        x2 = max(0, min(width - 1, int(person["x2"])))
        y2 = max(0, min(height - 1, int(person["y2"])))
        confidence = float(person["confidence"])
        cv2.rectangle(image, (x1, y1), (x2, y2), (70, 220, 75), 2)
        active_gesture = (index - 1) in getattr(result, "gesture_indices", [])
        label = ("HANDS UP " if active_gesture else "PERSON ") + f"{index} {confidence:.0%}"
        ids = getattr(result, "track_ids", []) or []
        if index - 1 < len(ids) and ids[index - 1] is not None:
            label += f" ID:{ids[index - 1]}"
        if active_gesture:
            cv2.rectangle(image, (x1, y1), (x2, y2), (10, 220, 250), 3)
        label_y = y1 - 7 if y1 >= 22 else min(height - 6, y1 + 17)
        cv2.putText(image, label, (x1 + 2, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, label, (x1 + 2, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (10, 110, 20), 1, cv2.LINE_AA)

    # The selected detection is a continuity-based candidate, not an identity.
    if bool(result.found):
        center = (int(result.center_x), int(result.center_y))
        cv2.circle(image, center, 6, (255, 160, 0), -1, cv2.LINE_AA)
        cv2.circle(image, center, 9, (255, 255, 255), 1, cv2.LINE_AA)

    # Semi-opaque strip allows status text to remain visible on bright images.
    strip = image.copy()
    cv2.rectangle(strip, (0, 0), (width, 27), (30, 30, 30), -1)
    cv2.addWeighted(strip, 0.7, image, 0.3, 0.0, dst=image)
    status = "FOUND" if result.found else "NO PERSON"
    label = f"{status} | people:{len(people)} | infer:{result.inference_ms:.0f}ms"
    cv2.putText(image, label, (7, 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.49, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def waiting_frame(message: str) -> np.ndarray:
    img = np.full((360, 640, 3), (25, 32, 38), dtype=np.uint8)
    cv2.putText(img, "Presentation Escort | YOLO PREVIEW", (24, 95),
                cv2.FONT_HERSHEY_SIMPLEX, 0.77, (238, 238, 238), 2, cv2.LINE_AA)
    cv2.putText(img, message, (24, 170), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (130, 230, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "Q / ESC : Quit", (24, 265),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
    return img


def show(frame: np.ndarray) -> bool:
    """Return False when operator requests to close the preview."""
    cv2.imshow(WINDOW_TITLE, frame)
    key = cv2.waitKey(1) & 0xFF
    if key in (ord("q"), ord("Q"), 27):
        return False
    try:
        if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
            return False
    except cv2.error:
        return False
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO person detection + Windows live preview")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument("--confidence", type=float, default=0.45)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--pose-model", default="yolo26n-pose.pt",
                        help="Pose model for two-hands-over-head gesture and face aim")
    parser.add_argument("--no-pose", action="store_true",
                        help="Legacy person boxes only (gesture control disabled)")
    parser.add_argument("--device", default="0")
    parser.add_argument("--no-tracker", action="store_true",
                        help="Disable default GPU ByteTrack continuity for diagnosis")
    args = parser.parse_args()

    # Do not accidentally publish unauthenticated camera inference on a LAN.
    if args.host not in ("127.0.0.1", "localhost"):
        parser.error("Live preview mode only permits loopback host; use SSH tunnel")

    config = ServerConfig(args.host, args.port, args.model,
                          args.confidence, args.image_size, args.device,
                          tracking=not args.no_tracker)
    detector = PersonDetector(config,
                              gesture_pose_model=None if args.no_pose else args.pose_model)
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_TITLE, 960, 540)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((config.host, config.port))
            server.listen(1)
            server.settimeout(0.2)
            print(f"[Preview] listening on {config.host}:{config.port}", flush=True)
            print("[Preview] waiting for Pi camera through SSH tunnel (Q / Esc to exit)", flush=True)
            if not show(waiting_frame("Waiting for Pi camera...")):
                return
            while True:
                try:
                    conn, address = server.accept()
                except socket.timeout:
                    if not show(waiting_frame("Waiting for Pi camera...")):
                        return
                    continue
                print(f"[Client] connected {address[0]}:{address[1]}", flush=True)
                detector.reset_tracking()
                with conn:
                    conn.settimeout(35)
                    try:
                        while True:
                            frame = decode_jpeg(recv_message(conn))
                            result = detector.detect(frame)
                            send_json(conn, result.to_dict())
                            print(f"[Detect] found={result.found} candidates={len(result.people or [])} "
                                  f"infer={result.inference_ms:.1f}ms", flush=True)
                            if not show(annotate(frame, result)):
                                return
                    except (ConnectionError, OSError, TimeoutError) as exc:
                        print(f"[Client] disconnected: {exc}", flush=True)
                    except Exception as exc:
                        print(f"[Client] error: {type(exc).__name__}: {exc}", flush=True)
                        raise
                    if not show(waiting_frame("Pi disconnected. Waiting...")):
                        return
    except KeyboardInterrupt:
        print("\n[Preview] stopped", flush=True)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
