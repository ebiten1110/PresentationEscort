"""Convert remote PERSON bounding boxes to safe image-space servo targets.

No robot hardware, networking, OpenCV or NumPy dependency. This module
does not command motors. It deliberately does NOT infer distance from the
person bbox using the V8.8.3 face-based thresholds.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Optional

from detection_protocol import DetectionResult


@dataclass(frozen=True)
class PersonTarget:
    center_x: int
    aim_y: int
    error_x: int
    error_y: int
    box_width: int
    box_height: int
    frame_width: int
    frame_height: int
    confidence: float
    aim_from_pose: bool = False

    def residual_bearing_deg(self, horizontal_fov_deg: float = 60.0,
                             image_to_yaw_sign: float = -1.0) -> float:
        """V8.8.3 residual term ONLY; actual body bearing also needs camera yaw."""
        return (
            self.error_x / float(self.frame_width)
            * horizontal_fov_deg
            * image_to_yaw_sign
        )


def detection_to_person_target(
    detection: DetectionResult,
    *,
    confidence_min: float = 0.45,
    aim_height_fraction: float = 0.22,
    center_offset_x: int = 0,
    center_offset_y: int = 0,
) -> Optional[PersonTarget]:
    """Accept only valid person boxes and aim near upper body / head.

    aim_height_fraction=0.22 means 22% downward from the top of the person box.
    It is only a visual aim estimate, not a face localization or identity check.
    """
    if not detection.found:
        return None
    if detection.class_id != 0 or detection.class_name.lower() != "person":
        return None
    if not isfinite(detection.confidence) or detection.confidence < confidence_min:
        return None
    if not 0 <= aim_height_fraction <= 1:
        raise ValueError("aim_height_fraction must be in [0, 1]")

    fw = int(detection.frame_width)
    fh = int(detection.frame_height)
    bw = int(detection.width)
    bh = int(detection.height)
    cx = int(detection.center_x)
    cy = int(detection.center_y)

    if not (0 < fw <= 8192 and 0 < fh <= 8192):
        return None
    if not (0 < bw <= fw and 0 < bh <= fh):
        return None
    if not (0 <= cx < fw and 0 <= cy < fh):
        return None

    top = cy - bh / 2.0
    aim_y = round(top + aim_height_fraction * bh)
    aim_y = max(0, min(fh - 1, aim_y))
    aim_x = cx
    if (type(detection.aim_x) is int and type(detection.aim_y) is int
            and 0 <= detection.aim_x < fw and 0 <= detection.aim_y < fh):
        aim_x, aim_y = detection.aim_x, detection.aim_y

    return PersonTarget(
        center_x=aim_x,
        aim_y=aim_y,
        error_x=aim_x - (fw // 2 + center_offset_x),
        error_y=aim_y - (fh // 2 + center_offset_y),
        box_width=bw,
        box_height=bh,
        frame_width=fw,
        frame_height=fh,
        confidence=float(detection.confidence),
        aim_from_pose=detection.aim_source in ("face", "shoulders"),
    )
