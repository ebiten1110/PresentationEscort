"""Unit tests for PERSON bbox conversion (no hardware dependencies)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import DetectionResult
from person_target_adapter import detection_to_person_target


def sample(**overrides):
    fields = {
        "found": True,
        "center_x": 480,
        "center_y": 240,
        "width": 160,
        "height": 320,
        "confidence": 0.95,
        "class_id": 0,
        "class_name": "person",
        "frame_width": 640,
        "frame_height": 480,
    }
    fields.update(overrides)
    return DetectionResult(**fields)


class PersonTargetTests(unittest.TestCase):
    def test_aim_near_head_not_bbox_center(self):
        target = detection_to_person_target(sample())
        self.assertIsNotNone(target)
        self.assertEqual(target.center_x, 480)
        self.assertEqual(target.aim_y, 150)
        self.assertEqual(target.error_x, 160)
        self.assertEqual(target.error_y, -90)

    def test_right_in_image_has_negative_residual_in_v883(self):
        target = detection_to_person_target(sample())
        self.assertAlmostEqual(target.residual_bearing_deg(), -15.0)

    def test_left_in_image_has_positive_residual(self):
        target = detection_to_person_target(sample(center_x=160))
        self.assertAlmostEqual(target.residual_bearing_deg(), 15.0)

    def test_no_person(self):
        self.assertIsNone(detection_to_person_target(sample(found=False)))

    def test_low_confidence(self):
        self.assertIsNone(detection_to_person_target(sample(confidence=0.25)))

    def test_wrong_class(self):
        self.assertIsNone(detection_to_person_target(sample(class_id=1)))

    def test_wrong_name(self):
        self.assertIsNone(detection_to_person_target(sample(class_name="bicycle")))

    def test_invalid_box(self):
        self.assertIsNone(detection_to_person_target(sample(height=600)))

    def test_invalid_frame(self):
        self.assertIsNone(detection_to_person_target(sample(frame_width=0)))

    def test_clipped_aim(self):
        target = detection_to_person_target(sample(center_y=2, height=220))
        self.assertEqual(target.aim_y, 0)

    def test_offset(self):
        target = detection_to_person_target(sample(center_x=320), center_offset_x=12)
        self.assertEqual(target.error_x, -12)

    def test_invalid_fraction(self):
        with self.assertRaises(ValueError):
            detection_to_person_target(sample(), aim_height_fraction=-0.1)


if __name__ == "__main__":
    unittest.main()
