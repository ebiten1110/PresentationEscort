"""Two-axis face aim, bounded slew, and old physical yaw sign (no hardware)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from person_target_adapter import detection_to_person_target
from pi_real_head_integration import HeadBodyTurnPlanner, HeadOnlyPlanner


def face_target(*, x: int, y: int):
    detection = DetectionResult(
        found=True, center_x=x, center_y=110, width=90, height=160,
        class_id=0, class_name="person", confidence=.95,
        aim_x=x, aim_y=y, aim_source="face",
        frame_width=320, frame_height=240,
    )
    target = detection_to_person_target(detection)
    assert target is not None
    return target


class SmoothFaceTests(unittest.TestCase):
    def test_v883_physical_yaw_sign_restored_by_default(self):
        planner = HeadOnlyPlanner()
        # Camera screen LEFT requests positive yaw in the old V8.8.3 mapping.
        plan = planner.step(face_target(x=40, y=120), now=1.)
        self.assertGreater(plan.yaw_target_deg, 0.)
        self.assertEqual(plan.pitch_target_deg, 0.)

    def test_explicit_reversal_remains_available(self):
        planner = HeadOnlyPlanner(invert_horizontal=True)
        plan = planner.step(face_target(x=40, y=120), now=1.)
        self.assertLess(plan.yaw_target_deg, 0.)

    def test_face_target_slews_both_axes_at_three_deg_per_second(self):
        planner = HeadOnlyPlanner(pitch_min=-3., pitch_max=3.)
        angles = []
        for i in range(26):
            plan = planner.step(face_target(x=40, y=20), now=1.+i*.1)
            angles.append((plan.yaw_target_deg, plan.pitch_target_deg))
            self.assertLessEqual(abs(plan.yaw_target_deg), 3.)
            self.assertLessEqual(abs(plan.pitch_target_deg), 3.)
            if i:
                self.assertLessEqual(abs(angles[i][0]-angles[i-1][0]), .35)
                self.assertLessEqual(abs(angles[i][1]-angles[i-1][1]), .35)
        self.assertGreater(angles[-1][0], 2.5)
        self.assertLess(angles[-1][1], -2.5)

    def test_disabled_pitch_remains_fixed_at_zero(self):
        planner = HeadOnlyPlanner()
        for i in range(15):
            plan = planner.step(face_target(x=120, y=10), now=1.+i*.15)
            self.assertEqual(plan.pitch_target_deg, 0.)
            self.assertTrue(plan.head_pose_command is None or
                            plan.head_pose_command.endswith(",0.0"))

    def test_short_camera_jitter_within_deadzone_does_not_steer(self):
        planner = HeadOnlyPlanner(pitch_min=-3., pitch_max=3.)
        for i, offset in enumerate((0, 2, -3, 4, -2, 1, 0)):
            plan = planner.step(face_target(x=160+offset, y=120+offset),
                                now=1.+i*.15)
            self.assertEqual(plan.yaw_target_deg, 0.)
            self.assertEqual(plan.pitch_target_deg, 0.)

    def test_lost_target_sends_no_pose_and_resets_pixel_filter(self):
        planner = HeadOnlyPlanner()
        planner.step(face_target(x=40, y=100), now=1.)
        self.assertIsNotNone(planner._smoothed_error_x)
        lost = planner.target_lost()
        self.assertIsNone(lost.head_pose_command)
        self.assertIsNone(planner._smoothed_error_x)
        self.assertIsNone(planner._smoothed_error_y)

    def test_body_turn_blocked_while_head_still_slewing(self):
        planner = HeadBodyTurnPlanner()
        for i in range(7):
            plan = planner.step(face_target(x=40, y=120), now=1.+i*.1,
                                head_yaw_feedback_deg=plan.yaw_target_deg if i else 0.,
                                head_feedback_fresh=True, body_idle=True)
            self.assertIsNone(plan.body_command)

    def test_speed_limit_validation(self):
        for rate in (0., -3., float("nan"), float("inf"), 20.):
            with self.assertRaises(ValueError):
                HeadOnlyPlanner(head_speed_deg_s=rate)


if __name__ == "__main__":
    unittest.main()
