"""Pure COCO-17 pose geometry for intentional two-hands-over-head selection.

No webcam, torch, model, or motor dependencies. The pose detection's matching
person box is supplied by the caller; it is not face identification.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

# COCO17: 0 nose, 5/6 shoulders, 7/8 elbows, 9/10 wrists.
NOSE, LEFT_SHOULDER, RIGHT_SHOULDER = 0, 5, 6
LEFT_WRIST, RIGHT_WRIST = 9, 10


def _point(points: Sequence[Sequence[float]], index: int,
           confidence_min: float = 0.50) -> Optional[tuple[float, float]]:
    if len(points) <= index or len(points[index]) < 3:
        return None
    x, y, confidence = points[index][:3]
    if not all(isinstance(v, (float, int)) and math.isfinite(v)
               for v in (x, y, confidence)) or confidence < confidence_min:
        return None
    return float(x), float(y)


def both_hands_over_head(points: Sequence[Sequence[float]]) -> bool:
    """Intentional gesture: both confident wrists clearly above the face.

    If the face is cropped, require both wrists well above the shoulders.
    Uncertain landmarks never activate a gesture.
    """
    ls = _point(points, LEFT_SHOULDER)
    rs = _point(points, RIGHT_SHOULDER)
    lw = _point(points, LEFT_WRIST, 0.58)
    rw = _point(points, RIGHT_WRIST, 0.58)
    if not all((ls, rs, lw, rw)):
        return False
    shoulder_span = max(1.0, math.dist(ls, rs))
    gap = max(10., 0.17 * shoulder_span)
    if not (lw[1] + gap < ls[1] and rw[1] + gap < rs[1]):
        return False
    nose = _point(points, NOSE)
    return nose is None or (lw[1] < nose[1] - 5 and rw[1] < nose[1] - 5)


def face_or_upper_body_aim(points: Sequence[Sequence[float]], box: tuple[int, int, int, int]
                           ) -> tuple[int, int, str]:
    """Aim at confidently detected face, otherwise shoulder midpoint or upper box."""
    x1, y1, x2, y2 = box
    nose = _point(points, NOSE, 0.60)
    if nose and x1 <= nose[0] <= x2 and y1 <= nose[1] <= y2:
        return round(nose[0]), round(nose[1]), "face"
    left = _point(points, LEFT_SHOULDER, 0.50)
    right = _point(points, RIGHT_SHOULDER, 0.50)
    if left and right:
        cx, cy = (left[0] + right[0]) / 2., (left[1] + right[1]) / 2.
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            return round(cx), round(cy), "shoulders"
    return round((x1 + x2) / 2), round(y1 + .23 * (y2 - y1)), "upper_body"
