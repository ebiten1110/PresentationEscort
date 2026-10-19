"""PC inference -> Pi image-space tracking preview. NEVER opens ESP32 serial.

Run with:
    python3 experiments/pc_gpu/pi_tracking_dry_run.py --host <PC_IP>
"""
from __future__ import annotations

import argparse
import socket
import time

import cv2

from detection_protocol import DetectionResult, recv_json, send_message
from person_target_adapter import detection_to_person_target
from pi_stream_client import encode_frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safe person tracking dry-run")
    parser.add_argument("--host", required=True, help="PC detector IPv4")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--minimum-confidence", type=float, default=0.45)
    parser.add_argument("--aim-height", type=float, default=0.22)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--max-rtt-ms", type=float, default=500.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("[Safety] DRY RUN ONLY - no ESP32 commands will be sent.")
    camera = cv2.VideoCapture(args.camera_index)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not camera.isOpened():
        raise RuntimeError("camera open failed")

    try:
        with socket.create_connection((args.host, args.port), timeout=args.timeout) as sock:
            sock.settimeout(args.timeout)
            print(f"[PC] Connected to {args.host}:{args.port}")

            while True:
                started = time.monotonic()
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("camera frame unavailable")

                send_message(sock, encode_frame(frame, args.jpeg_quality))
                result = DetectionResult.from_dict(recv_json(sock))
                rtt_ms = (time.monotonic() - started) * 1000.0

                if rtt_ms > args.max_rtt_ms:
                    print(f"[Safety] STALE rtt={rtt_ms:.1f}ms; target discarded")
                    continue
                if result.error:
                    print(f"[Safety] Detector error: {result.error}")
                    continue

                target = detection_to_person_target(
                    result,
                    confidence_min=args.minimum_confidence,
                    aim_height_fraction=args.aim_height,
                )
                if target is None:
                    print(f"[Target] NO_PERSON rtt={rtt_ms:.1f}ms")
                    continue

                residual = target.residual_bearing_deg()
                print(
                    "[Target] "
                    f"aim=({target.center_x},{target.aim_y}) "
                    f"error=({target.error_x:+d},{target.error_y:+d})px "
                    f"residual={residual:+.1f}deg "
                    f"confidence={target.confidence:.2f} "
                    f"infer={result.inference_ms:.1f}ms rtt={rtt_ms:.1f}ms"
                )
    except KeyboardInterrupt:
        print("\n[Main] interrupted")
    except (ConnectionError, TimeoutError, socket.timeout, OSError) as exc:
        print(f"[Safety] PC connection unavailable: {exc}")
    finally:
        camera.release()
        print("[Safety] DRY RUN stopped; motors were never enabled.")


if __name__ == "__main__":
    main()
