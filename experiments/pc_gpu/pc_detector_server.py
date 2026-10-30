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
from gesture_pose import both_hands_over_head, face_or_upper_body_aim
from crowd_pose_dedup import collapse_same_face_duplicates


@dataclass
class ServerConfig:
    host: str
    port: int
    model: str
    confidence: float
    image_size: int
    device: str
    tracking: bool = False


class PersonDetector:
    def __init__(self, config: ServerConfig, *, gesture_pose_model: Optional[str] = None) -> None:
        self.config = config
        self.gesture_pose_model = gesture_pose_model
        model_name = gesture_pose_model or config.model
        print(f"[Model] loading {model_name} device={config.device}")
        self.model = YOLO(model_name)
        self.last_finish_time: Optional[float] = None
        self.tracker = PersonTracker(min_confidence=config.confidence)
        self.frame_seq = 0
        print("[Model] ready; tracking=" + ("ByteTrack" if config.tracking else "off"))

    def reset_tracking(self) -> None:
        """Clear person-lock state on every new Pi connection."""
        self.tracker.reset()
        predictor = getattr(self.model, "predictor", None)
        if predictor is not None:
            for tracker in getattr(predictor, "trackers", ()) or ():
                if hasattr(tracker, "reset"):
                    tracker.reset()
        self.last_finish_time = None
        self.frame_seq = 0

    def detect(self, frame: np.ndarray) -> DetectionResult:
        frame_height, frame_width = frame.shape[:2]
        self.frame_seq += 1
        started = time.perf_counter()
        # ByteTrack uses low-score boxes internally to bridge short occlusions.
        # The external presenter-lock protocol still accepts only trusted
        # >= config.confidence boxes; low-score results never steer motors.
        options = dict(
            source=frame,
            conf=min(0.18, self.config.confidence) if self.config.tracking
                 else self.config.confidence,
            imgsz=self.config.image_size,
            classes=[0],
            device=self.config.device,
            verbose=False,
        )
        if self.config.tracking:
            results = self.model.track(persist=True, tracker="bytetrack.yaml",
                                       **options)
        else:
            results = self.model.predict(**options)

        inference_ms = (time.perf_counter() - started) * 1000.0
        now = time.perf_counter()
        server_fps = 0.0
        if self.last_finish_time is not None:
            elapsed = now - self.last_finish_time
            if elapsed > 0:
                server_fps = 1.0 / elapsed
        self.last_finish_time = now

        candidates: list[PersonBox] = []
        pose_points: list[list[list[float]]] = []
        candidate_track_ids: list[Optional[int]] = []
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
                    track_id = None
                    if (self.config.tracking and detected_boxes.id is not None):
                        track_id = int(detected_boxes.id[idx].item())
                    candidates.append(
                        PersonBox(
                            x1=x1, y1=y1, x2=x2, y2=y2,
                            confidence=float(detected_boxes.conf[idx].item()),
                        )
                    )
                    candidate_track_ids.append(track_id)
                    if self.gesture_pose_model and results[0].keypoints is not None:
                        points = results[0].keypoints.data[idx].tolist()
                        pose_points.append(points)
                    else:
                        pose_points.append([])

        # Crowd robustness: YOLO can output two overlapping boxes with
        # different ByteTrack IDs for one identical face. Collapse only
        # conservative same-face duplicates *before* building V2 metadata.
        # Never merge distinct faces or uncertain equal-confidence detections.
        candidates, pose_points, candidate_track_ids = (
            collapse_same_face_duplicates(
                candidates, pose_points, candidate_track_ids)
        )

        # Protocol V2 transfers every valid person candidate. The Pi makes
        # the final operator-confirmed association; confidence MUST NOT
        # select or silently re-acquire a presenter.
        people = [
            {
                "x1": box.x1, "y1": box.y1, "x2": box.x2, "y2": box.y2,
                "confidence": box.confidence,
            }
            for box in candidates
            if box.valid(frame_width, frame_height, self.config.confidence)
        ]
        # V2 optional metadata stays aligned with the *filtered* people.
        # A tracker ID is evidence of frame continuity, never a biometric ID.
        track_ids = [
            candidate_track_ids[i] for i, box in enumerate(candidates)
            if box.valid(frame_width, frame_height, self.config.confidence)
        ] if self.config.tracking else []
        gestures: list[int] = []
        aims: list[dict] = []
        if self.gesture_pose_model:
            filtered = [(box, pose_points[i]) for i, box in enumerate(candidates)
                        if box.valid(frame_width, frame_height, self.config.confidence)]
            for person_index, (box, points) in enumerate(filtered):
                if both_hands_over_head(points):
                    gestures.append(person_index)
                x, y, source = face_or_upper_body_aim(
                    points, (box.x1, box.y1, box.x2, box.y2))
                aims.append({"x": x, "y": y, "source": source})
        selected = self.tracker.select(candidates, frame_width, frame_height)
        common = dict(
            protocol_version=2,
            frame_seq=self.frame_seq,
            people=people,
            track_ids=track_ids,
            gesture_indices=gestures,
            aim_points=aims,
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
    parser.add_argument("--tracker", action="store_true",
                        help="Enable persistent GPU ByteTrack for frame continuity")
    args = parser.parse_args()
    return ServerConfig(
        host=args.host,
        port=args.port,
        model=args.model,
        confidence=args.confidence,
        image_size=args.image_size,
        device=args.device,
        tracking=args.tracker,
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
