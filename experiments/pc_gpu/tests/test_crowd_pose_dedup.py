"""Crowd duplicate YOLO pose output regression tests.

Frame 1089 in 2026-10-30 on-device diagnostics returned the same face at
(372,200) and (371,201) in overlapping boxes under tracker IDs 83 / 326.
"""
from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from crowd_pose_dedup import collapse_same_face_duplicates
from person_tracker import PersonBox


def face_points(x, y):
    pts = [[0., 0., 0.] for _ in range(17)]
    pts[0] = [float(x), float(y), .95]
    return pts


class CrowdDuplicateDetectionTests(unittest.TestCase):
    def test_diagnostic_two_ids_same_face_one_box(self):
        boxes = [
            PersonBox(172, 101, 616, 480, .6297372),
            PersonBox(139, 94, 500, 480, .4553431),
        ]
        pts = [face_points(372, 200), face_points(371, 201)]
        result, result_pts, result_ids = collapse_same_face_duplicates(
            boxes, pts, [83, 326])
        self.assertEqual(len(result), 1)
        self.assertEqual(result_ids, [83])
        self.assertEqual(result_pts, [pts[0]])

    def test_genuine_separate_faces_survive_even_with_overlapping_boxes(self):
        boxes = [
            PersonBox(94, 141, 370, 480, .80),
            PersonBox(160, 116, 580, 480, .60),
        ]
        result, _, ids = collapse_same_face_duplicates(
            boxes, [face_points(216, 263), face_points(356, 210)],
            [332, 83])
        self.assertEqual(len(result), 2)
        self.assertEqual(ids, [332, 83])

    def test_uncertain_equal_confidence_duplicates_remain_ambiguous(self):
        boxes = [
            PersonBox(172, 101, 616, 480, .62),
            PersonBox(139, 94, 500, 480, .58),
        ]
        result, _, ids = collapse_same_face_duplicates(
            boxes, [face_points(372, 200), face_points(371, 201)],
            [83, 326])
        self.assertEqual(len(result), 2)
        self.assertEqual(ids, [83, 326])

    def test_no_reliable_face_cannot_be_merged(self):
        boxes = [
            PersonBox(172, 101, 616, 480, .85),
            PersonBox(139, 94, 500, 480, .48),
        ]
        result, _, _ = collapse_same_face_duplicates(boxes, [[], []],
                                                      [83, 326])
        self.assertEqual(len(result), 2)


if __name__ == "__main__":
    unittest.main()
