"""Conservative collapse of duplicate YOLO person boxes with the SAME face.

Crowd cameras sometimes return two tracker IDs for the identical face and
overlapping body. An ambiguous genuine second person is NEVER removed unless
the visual evidence meets all strict conditions below.
"""
from __future__ import annotations

from math import hypot
from typing import Optional, Sequence

from gesture_pose import face_or_upper_body_aim
from person_tracker import PersonBox


def _iou(a: PersonBox, b: PersonBox) -> float:
    w = max(0, min(a.x2, b.x2) - max(a.x1, b.x1))
    h = max(0, min(a.y2, b.y2) - max(a.y1, b.y1))
    overlap = w * h
    area_a = (a.x2 - a.x1) * (a.y2 - a.y1)
    area_b = (b.x2 - b.x1) * (b.y2 - b.y1)
    return overlap / max(1, area_a + area_b - overlap)


def collapse_same_face_duplicates(
    candidates: list[PersonBox],
    pose_points: list[list],
    track_ids: list[Optional[int]],
) -> tuple[list[PersonBox], list[list], list[Optional[int]]]:
    """Keep one detection only for high-evidence model duplicates.

    Requirements:
    - Both pose estimates contain a confidently detected nose (face aim).
    - Nose positions differ by <= 7 image pixels, and person boxes IoU >= .62.
    - Detection confidence differs by >= .10 so the stronger hypothesis wins.

    The conservative confidence-gap requirement avoids merging two similarly
    plausible people whose faces truly overlap: fail closed in that case.
    Preserve order and synchronized track/keypoint metadata for gesture locks.
    """
    if not (len(candidates) == len(pose_points) == len(track_ids)):
        raise ValueError("person candidate metadata is not aligned")
    if len(candidates) < 2:
        return candidates, pose_points, track_ids
    aims = [
        face_or_upper_body_aim(points, (box.x1, box.y1, box.x2, box.y2))
        for box, points in zip(candidates, pose_points)
    ]
    removed = set()
    # Sort candidates by confidence so a lower-confidence duplicate is only
    # removed in favor of the higher-confidence, more reliable detection.
    order = sorted(range(len(candidates)),
                   key=lambda i: candidates[i].confidence, reverse=True)
    for n, i in enumerate(order):
        if i in removed or aims[i][2] != "face":
            continue
        for j in order[n + 1:]:
            if j in removed or aims[j][2] != "face":
                continue
            if candidates[i].confidence - candidates[j].confidence < .10:
                continue
            if hypot(aims[i][0] - aims[j][0],
                     aims[i][1] - aims[j][1]) > 7:
                continue
            if _iou(candidates[i], candidates[j]) < .62:
                continue
            removed.add(j)
    if not removed:
        return candidates, pose_points, track_ids
    kept = [i for i in range(len(candidates)) if i not in removed]
    return ([candidates[i] for i in kept],
            [pose_points[i] for i in kept],
            [track_ids[i] for i in kept])
