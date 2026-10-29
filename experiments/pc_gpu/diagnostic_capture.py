"""Bounded, operator-opt-in, in-memory evidence for offline diagnosis.

No motor commands, network access or automatic file writes. A ZIP is only
created when an authenticated operator explicitly requests a download.
Images can contain faces; the resulting archive is personal information.
"""
from __future__ import annotations

import io
import json
import math
import threading
import time
import zipfile
from collections import deque
from datetime import datetime, timezone


def _safe_json(value, depth: int = 0):
    """Bound untrusted detection payloads; diagnostics must not break control."""
    if depth > 6:
        return None
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        return value[:500]
    if isinstance(value, (tuple, list)):
        return [_safe_json(item, depth + 1) for item in value[:50]]
    if isinstance(value, dict):
        return {str(key)[:70]: _safe_json(item, depth + 1)
                for key, item in list(value.items())[:50]}
    return None


class DiagnosticCapture:
    def __init__(self, *, max_events: int = 600, max_frames: int = 48,
                 frame_interval_s: float = 0.25):
        if not (1 <= max_events <= 3000 and 1 <= max_frames <= 100
                and 0.1 <= frame_interval_s <= 10):
            raise ValueError("diagnostic ring limits invalid")
        self._events = deque(maxlen=max_events)
        self._frames = deque(maxlen=max_frames)
        self._interval = frame_interval_s
        self._last_frame_at = None
        self._guard = threading.Lock()

    def event(self, payload: dict) -> None:
        # Keep only serializable operator data, never credentials or raw UART.
        row = _safe_json(payload)
        row["utc"] = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        with self._guard:
            self._events.append(row)

    def frame(self, jpeg: bytes, *, at: float | None = None) -> None:
        now = time.monotonic() if at is None else float(at)
        # Already encoded for the preview, so no extra video encode on Pi.
        if len(jpeg) > 400_000 or not jpeg.startswith(b"\xff\xd8"):
            return
        with self._guard:
            if (self._last_frame_at is not None
                    and now - self._last_frame_at < self._interval):
                return
            self._last_frame_at = now
            self._frames.append((time.time(), bytes(jpeg)))

    def export(self, *, status: dict) -> bytes:
        with self._guard:
            events = list(self._events)
            frames = list(self._frames)
        # The caller supplies an explicit public diagnostics subset only.
        info = {"schema_version": 1, "generated_utc":
                datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "status": _safe_json(status),
                "events_count": len(events), "images_count": len(frames),
                "frames_contain_people": True}
        output = io.BytesIO()
        with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED,
                             compresslevel=3) as archive:
            archive.writestr("status.json", json.dumps(info, ensure_ascii=False,
                                                        indent=2, allow_nan=False))
            archive.writestr("decisions.jsonl", "".join(
                json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n"
                for row in events))
            archive.writestr("README.txt", (
                "PresentationEscort diagnostic evidence (no audio).\n"
                "status.json = latest safety/camera status.\n"
                "decisions.jsonl = recent YOLO/person/gesture and Pi decisions.\n"
                "frames/*.jpg = most recent annotated camera images (up to 48).\n"
                "CAUTION: images may contain identifiable people. Share privately.\n"
                "No operator token or network credentials are included.\n"
            ))
            for index, (timestamp, jpeg) in enumerate(frames):
                filename = f"frames/{index:03d}_{int(timestamp * 1000)}.jpg"
                archive.writestr(filename, jpeg, compress_type=zipfile.ZIP_STORED)
        return output.getvalue()
