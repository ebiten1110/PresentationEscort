"""Conservative bounding-box continuity lock for one presenter.

Pure Python. A continuity guard, not face recognition or true ReID.
Missing/implausible detections return None; never generate a motion command here.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Optional, Sequence


@dataclass(frozen=True)
class PersonBox:
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    def valid(self, width: int, height: int, min_conf: float) -> bool:
        return (
            0 <= self.x1 < self.x2 <= width
            and 0 <= self.y1 < self.y2 <= height
            and isfinite(self.confidence)
            and self.confidence >= min_conf
        )


class PersonTracker:
    """Suppress abrupt jumps to other detections.

    On mismatch, output None immediately (no stale bbox is reused).
    After release_after_misses consecutive unmatched frames, the lock is
    released; acquisition can occur on a subsequent frame. This is not
    identity verification and may reacquire a different person after loss.
    """

    def __init__(
        self,
        min_confidence: float = 0.45,
        release_after_misses: int = 5,
        max_position_jump: float = 0.75,
        max_size_ratio: float = 2.5,
    ) -> None:
        if not (0.0 <= min_confidence <= 1.0):
            raise ValueError("min_confidence must be between 0 and 1")
        if release_after_misses < 1:
            raise ValueError("release_after_misses must be positive")
        self.min_confidence = min_confidence
        self.release_after_misses = release_after_misses
        self.max_position_jump = max_position_jump
        self.max_size_ratio = max_size_ratio
        self.reset()

    def reset(self) -> None:
        self.last: Optional[PersonBox] = None
        self.last_frame_size: Optional[tuple[int, int]] = None
        self.misses = 0

    def _matches(self, old: PersonBox, new: PersonBox) -> bool:
        if not (1 / self.max_size_ratio <= new.width / old.width <= self.max_size_ratio):
            return False
        if not (1 / self.max_size_ratio <= new.height / old.height <= self.max_size_ratio):
            return False
        x_allowance = max(24.0, self.max_position_jump * max(old.width, new.width))
        y_allowance = max(24.0, self.max_position_jump * max(old.height, new.height))
        return abs(new.cx - old.cx) <= x_allowance and abs(new.cy - old.cy) <= y_allowance

    @staticmethod
    def _distance(old: PersonBox, new: PersonBox) -> float:
        dx = (new.cx - old.cx) / max(old.width, new.width, 1)
        dy = (new.cy - old.cy) / max(old.height, new.height, 1)
        size_delta = abs(new.width / old.width - 1.0) + abs(new.height / old.height - 1.0)
        return dx * dx + dy * dy + 0.15 * size_delta

    def select(self, candidates: Sequence[PersonBox], frame_width: int, frame_height: int) -> Optional[PersonBox]:
        if frame_width < 1 or frame_height < 1:
            self.reset()
            return None
        if self.last_frame_size is not None and self.last_frame_size != (frame_width, frame_height):
            self.reset()
        self.last_frame_size = (frame_width, frame_height)

        valid = [box for box in candidates if box.valid(frame_width, frame_height, self.min_confidence)]
        if self.last is None:
            if not valid:
                return None
            picked = max(valid, key=lambda box: box.confidence)
            self.last, self.misses = picked, 0
            return picked

        matches = [box for box in valid if self._matches(self.last, box)]
        if matches:
            picked = min(matches, key=lambda box: self._distance(self.last, box))
            self.last, self.misses = picked, 0
            return picked

        self.misses += 1
        if self.misses >= self.release_after_misses:
            self.last = None
            self.misses = 0
        return None
