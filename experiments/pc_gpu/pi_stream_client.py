from __future__ import annotations

import argparse
import socket
import time
from dataclasses import dataclass

import cv2

from detection_protocol import DetectionResult, recv_json, send_message


@dataclass
class ClientConfig:
    host: str
    port: int
    camera_index: int
    width: int
    height: int
    jpeg_quality: int
    timeout: float


def parse_args() -> ClientConfig:
    parser = argparse.ArgumentParser(
        description="Pi camera -> PC/GPU detection smoke test"
    )
    parser.add_argument("--host", required=True, help="PC IP address")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    return ClientConfig(
        host=args.host,
        port=args.port,
        camera_index=args.camera_index,
        width=args.width,
        height=args.height,
        jpeg_quality=args.jpeg_quality,
        timeout=args.timeout,
    )


def encode_frame(frame, quality: int) -> bytes:
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return encoded.tobytes()


def main() -> None:
    config = parse_args()
    cap = cv2.VideoCapture(config.camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        raise RuntimeError("camera open failed")

    try:
        with socket.create_connection(
            (config.host, config.port),
            timeout=config.timeout,
        ) as sock:
            sock.settimeout(config.timeout)
            print(f"[Client] connected to {config.host}:{config.port}")

            while True:
                started = time.perf_counter()
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("camera capture failed")

                payload = encode_frame(frame, config.jpeg_quality)
                send_message(sock, payload)
                result = DetectionResult.from_dict(recv_json(sock))
                round_trip_ms = (
                    time.perf_counter() - started
                ) * 1000.0

                print(
                    "[Result] "
                    f"found={result.found} "
                    f"center=({result.center_x},{result.center_y}) "
                    f"size=({result.width}x{result.height}) "
                    f"conf={result.confidence:.2f} "
                    f"infer={result.inference_ms:.1f}ms "
                    f"rtt={round_trip_ms:.1f}ms"
                )
    except KeyboardInterrupt:
        print("\n[Client] stopped")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
