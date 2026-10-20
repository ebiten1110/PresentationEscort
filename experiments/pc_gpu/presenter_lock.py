"""Fail-closed operator-selected presenter geometry lock (NO face ID/ReID).

The PC sends ALL person boxes (protocol V2). The human operator selects one
candidate while SAFE. Every subsequent frame must contain one unambiguous
geometric successor or tracking is invalidated instantly; loss/crossing
requires explicit re-selection. Never silently acquire the nearest stranger.

This reduces accidental target switching, but cannot guarantee personal
identity when visually similar people exchange positions. Hardware multi-
person AUTO remains NO-GO until real tests / identity safeguards pass.
"""
from __future__ import annotations

from dataclasses import replace
from math import hypot, isfinite
from typing import Optional

from detection_protocol import DetectionResult
from person_tracker import PersonBox


class PresenterLock:
    MAX_PEOPLE = 12
    MAX_FRAME_AGE_S = 0.50
    CONFIDENCE_MIN = 0.45

    def __init__(self) -> None:
        self._last_seq = 0
        self._frame_time: Optional[float] = None
        self._frame_size: Optional[tuple[int, int]] = None
        self.candidates: list[PersonBox] = []
        self._history: dict[int, tuple[float, list[PersonBox]]] = {}
        self._locked: Optional[PersonBox] = None
        self.locked = False
        self.state = "UNLOCKED"
        self.reason = "OPERATOR_SELECTION_REQUIRED"

    def reset_stream(self) -> None:
        """Discard frame sequence and any identity after a TCP reconnect."""
        self._last_seq = 0
        self._frame_time = None
        self._frame_size = None
        self.candidates = []
        self._history.clear()
        self.clear("PC_STREAM_DISCONNECTED")

    def clear(self, reason: str = "OPERATOR_SELECTION_REQUIRED") -> None:
        self._locked = None
        self.locked = False
        self.state = "UNLOCKED"
        self.reason = reason

    def _lose(self, reason: str) -> None:
        self._locked = None
        self.locked = False
        self.state = "LOST"
        self.reason = reason

    def observe(self, detection: DetectionResult, *, now: float) -> DetectionResult:
        """Process complete V2 frames; output only a human-locked person."""
        suppressed = replace(
            detection, found=False, center_x=0, center_y=0,
            width=0, height=0, confidence=0., class_id=-1, class_name="",
        )
        if (
            not isfinite(now)
            or detection.error
            or detection.protocol_version != 2
            or isinstance(detection.frame_seq, bool)
            or not isinstance(detection.frame_seq, int)
            or detection.frame_seq <= self._last_seq
            or detection.frame_seq - self._last_seq > 1 and self._last_seq != 0
            or not isinstance(detection.people, list)
            or len(detection.people) > self.MAX_PEOPLE
            or not (0 < detection.frame_width <= 8192
                    and 0 < detection.frame_height <= 8192)
        ):
            self.candidates = []
            self._frame_time = None
            self._lose("INVALID_STALE_OR_LEGACY_PROTOCOL")
            return suppressed

        new_frame_size = (detection.frame_width, detection.frame_height)
        if self._frame_size is not None and new_frame_size != self._frame_size:
            self._lose("CAMERA_RESOLUTION_CHANGED")
        self._frame_size = new_frame_size
        self._last_seq = detection.frame_seq
        self._frame_time = now

        candidates: list[PersonBox] = []
        for item in detection.people:
            if not isinstance(item, dict) or set(item) != {
                "x1", "y1", "x2", "y2", "confidence"
            }:
                self.candidates = []
                self._lose("INVALID_PERSON_BOX")
                return suppressed
            try:
                # Reject booleans, floats for coordinates, NaN and malformed input.
                coords = tuple(item[name] for name in ("x1", "y1", "x2", "y2"))
                if any(type(v) is not int for v in coords):
                    raise ValueError("bbox coordinates must be integers")
                conf = item["confidence"]
                if isinstance(conf, bool) or not isinstance(conf, (int, float)):
                    raise ValueError("confidence must be numeric")
                box = PersonBox(*coords, float(conf))
                if not box.valid(*new_frame_size, self.CONFIDENCE_MIN):
                    raise ValueError("invalid person box")
            except (ValueError, TypeError, OverflowError):
                self.candidates = []
                self._lose("INVALID_PERSON_BOX")
                return suppressed
            candidates.append(box)
        self.candidates = candidates
        self._history[self._last_seq] = (now, list(candidates))
        self._history = {
            seq: item for seq, item in self._history.items()
            if now - item[0] <= self.MAX_FRAME_AGE_S
        }
        if not self.locked or self._locked is None:
            return suppressed

        original = self._locked
        allowed: list[PersonBox] = []
        for box in candidates:
            # Strict 0.5-body-width displacement and limited size variation.
            if (
                0.55 <= box.width / original.width <= 1.80
                and 0.55 <= box.height / original.height <= 1.80
                and abs(box.cx - original.cx) <= max(12.0, original.width * 0.52)
                and abs(box.cy - original.cy) <= max(12.0, original.height * 0.30)
            ):
                allowed.append(box)
        if len(allowed) != 1:
            self._lose("TARGET_MISSING_OR_MULTIPLE_MATCHES")
            return suppressed
        chosen = allowed[0]
        # A nearby second person can exchange places on the next frame.
        # Do not continue steering through an interaction/occlusion.
        if self._near_another(chosen, candidates):
            self._lose("AMBIGUOUS_NEARBY_PERSON")
            return suppressed
        self._locked = chosen
        self.state = "TRACKING"
        self.reason = "LOCKED_GEOMETRY_ONLY"
        return replace(
            detection, found=True, center_x=round(chosen.cx),
            center_y=round(chosen.cy), width=chosen.width,
            height=chosen.height, confidence=chosen.confidence,
            class_id=0, class_name="person",
        )

    @staticmethod
    def _near_another(chosen: PersonBox, candidates: list[PersonBox]) -> bool:
        return any(
            other is not chosen and
            hypot(chosen.cx - other.cx, chosen.cy - other.cy)
            < max(32.0, 0.80 * max(chosen.width, other.width))
            for other in candidates
        )

    def select(self, index: int, frame_seq: int, *, now: float) -> tuple[bool, str]:
        if self.locked:
            return False, "UNLOCK_FIRST"
        if (
            type(index) is not int or type(frame_seq) is not int
            or not isfinite(now) or self._frame_time is None
            or now < self._frame_time
            or now - self._frame_time > self.MAX_FRAME_AGE_S
            or frame_seq not in self._history
        ):
            return False, "STALE_OR_INVALID_SELECTION"
        history_at, historical = self._history[frame_seq]
        if (
            now - history_at > self.MAX_FRAME_AGE_S
            or not 0 <= index < len(historical)
        ):
            return False, "STALE_OR_INVALID_SELECTION"
        old = historical[index]
        # The UI can be one or two frames behind. Revalidate selection
        # against the actual most recent camera frame, never trust an index
        # that may have shifted due to detection reordering.
        matches = [
            box for box in self.candidates
            if (
                0.7 <= box.width / old.width <= 1.4
                and 0.7 <= box.height / old.height <= 1.4
                and abs(box.cx - old.cx) <= max(10.0, old.width * 0.35)
                and abs(box.cy - old.cy) <= max(10.0, old.height * 0.22)
            )
        ]
        if len(matches) != 1 or self._near_another(matches[0], self.candidates):
            return False, "SELECTION_AMBIGUOUS_OR_MOVED"
        self._locked = matches[0]
        self.locked = True
        self.state = "SELECTED_WAIT_NEXT_FRAME"
        self.reason = "LOCK_SELECTED_NEEDS_FRESH_FRAMES"
        return True, "SELECTED"

    def display(self, *, now: float) -> dict:
        recent = (
            self._frame_time is not None and isfinite(now)
            and 0 <= now - self._frame_time <= self.MAX_FRAME_AGE_S
        )
        fw, fh = self._frame_size or (1, 1)
        return {
            "lock_state": self.state if recent else "NO_FRESH_FRAME",
            "lock_reason": self.reason,
            "presenter_selected": self.locked and recent,
            "frame_seq": self._last_seq if recent else 0,
            "candidate_count": len(self.candidates) if recent else 0,
            "candidates": [
                {"index": i, "x_percent": round(box.cx / fw * 100),
                 "y_percent": round(box.cy / fh * 100),
                 "confidence": round(box.confidence, 2)}
                for i, box in enumerate(self.candidates)
            ] if recent else [],
        }
