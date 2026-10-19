from __future__ import annotations

import argparse
import socket
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
from ultralytics import YOLO

from detection_protocol import DetectionResult, recv_message, send_json
from person_tracker import PersonBox, PersonTracker


@dataclass
class ServerConfig:
    host: str
    port: int
    model: str
    confidence: float
    image_size: int
    device: str


class PersonDetector:
    def __init__(self, config: ServerConfig) -> None:
        self.config = config
        print(f"[Model] loading {config.model} device={config.device}")
        self.model = YOLO(config.model)
        self.last_finish_time: Optional[float] = None
        self.tracker = PersonTracker(min_confidence=config.confidence)
        print("[Model] ready")

    def reset_tracking(self) -> None:
        """Clear person-lock state on every new Pi connection."""
        self.tracker.reset()
        self.last_finish_time = None

    def detect(self, frame: np.ndarray) -> DetectionResult:
        frame_height, frame_width = frame.shape[:2]
        started = time.perf_counter()
        results = self.model.predict(
            source=frame,
            conf=self.config.confidence,
            imgsz=self.config.image_size,
            classes=[0],
            device=self.config.device,
            verbose=False,
        )

        inference_ms = (time.perf_counter() - started) * 1000.0
        now = time.perf_counter()
        server_fps = 0.0
        if self.last_finish_time is not None:
            elapsed = now - self.last_finish_time
            if elapsed > 0:
                server_fps = 1.0 / elapsed
        self.last_finish_time = now

        candidates: list[PersonBox] = []
        if results:
            detected_boxes = results[0].boxes
            if detected_boxes is not None:
                for idx in range(len(detected_boxes)):
                    if int(detected_boxes.cls[idx].item()) != 0:
                        continue
                    x1, y1, x2, y2 = detected_boxes.xyxy[idx].tolist()
                    x1 = max(0, min(int(round(x1)), frame_width - 1))
                    y1 = max(0, min(int(round(y1)), frame_height - 1))
                    x2 = max(x1 + 1, min(int(round(x2)), frame_width))
                    y2 = max(y1 + 1, min(int(round(y2)), frame_height))
                    candidates.append(
                        PersonBox(
                            x1=x1, y1=y1, x2=x2, y2=y2,
                            confidence=float(detected_boxes.conf[idx].item()),
                        )
                    )

        selected = self.tracker.select(candidates, frame_width, frame_height)
        common = dict(
            frame_width=frame_width,
            frame_height=frame_height,
            inference_ms=inference_ms,
            server_fps=server_fps,
            source="pc_gpu_person_continuity",
        )
        if selected is None:
            return DetectionResult(found=False, **common)

        return DetectionResult(
            found=True,
            center_x=int(round(selected.cx)),
            center_y=int(round(selected.cy)),
            width=selected.width,
            height=selected.height,
            confidence=selected.confidence,
            class_id=0,
            class_name="person",
            **common,
        )

def decode_jpeg(payload: bytes) -> np.ndarray:
    array = np.frombuffer(payload, dtype=np.uint8)
    frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("failed to decode JPEG frame")
    return frame


def handle_client(
    conn: socket.socket,
    address: Tuple[str, int],
    detector: PersonDetector,
) -> None:
    print(f"[Client] connected {address[0]}:{address[1]}")
    conn.settimeout(5.0)
    detector.reset_tracking()

    try:
        while True:
            frame_payload = recv_message(conn)
            frame = decode_jpeg(frame_payload)
            result = detector.detect(frame)
            send_json(conn, result.to_dict())
            print(
                "[Detect] "
                f"found={result.found} "
                f"center=({result.center_x},{result.center_y}) "
                f"size=({result.width}x{result.height}) "
                f"conf={result.confidence:.2f} "
                f"infer={result.inference_ms:.1f}ms "
                f"fps={result.server_fps:.1f}"
            )
    except (ConnectionError, TimeoutError, socket.timeout) as exc:
        print(f"[Client] disconnected: {exc}")
    except Exception as exc:
        try:
            send_json(
                conn,
                DetectionResult(found=False, error=str(exc)).to_dict(),
            )
        except Exception:
            pass
        print(f"[Client] error: {exc}")
    finally:
        conn.close()


def parse_args() -> ServerConfig:
    parser = argparse.ArgumentParser(
        description="Presentation Escort PC/GPU person detector"
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument("--confidence", type=float, default=0.45)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument(
        "--device",
        default="0",
        help="Ultralytics device, e.g. 0, cpu, cuda:0",
    )
    args = parser.parse_args()
    return ServerConfig(
        host=args.host,
        port=args.port,
        model=args.model,
        confidence=args.confidence,
        image_size=args.image_size,
        device=args.device,
    )


def main() -> None:
    config = parse_args()
    detector = PersonDetector(config)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((config.host, config.port))
        server.listen(1)
        print(f"[Server] listening on {config.host}:{config.port}")

        while True:
            conn, address = server.accept()
            handle_client(conn, address, detector)


if __name__ == "__main__":
    main()
