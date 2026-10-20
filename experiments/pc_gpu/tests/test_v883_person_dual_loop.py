"""Hardware-free V8.8.3-calibrated dual-loop regression tests."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import DetectionResult
from person_target_adapter import detection_to_person_target
from v883_person_dual_loop import V883PersonDualLoop


def person(cx: int, cy: int = 120, *, fw: int = 320,
           fh: int = 240, bw: int = 90, bh: int = 120):
    d = DetectionResult(
        found=True, center_x=cx, center_y=cy,
        frame_width=fw, frame_height=fh,
        width=bw, height=bh,
        confidence=0.9, class_id=0, class_name="person",
    )
    target = detection_to_person_target(d)
    assert target is not None
    return target


class DualLoopTests(unittest.TestCase):
    def setUp(self):
        self.ctl = V883PersonDualLoop()

    def test_horizontal_sign_and_gain_matches_v883(self):
        plan = self.ctl.step(person(250), now=1)
        self.assertEqual(plan.yaw_target_deg, -2.5)
        self.assertAlmostEqual(plan.residual_deg, -16.875)
        self.assertFalse(plan.bearing_from_feedback)
        self.assertIsNone(plan.body_command)

    def test_left_image_is_positive_yaw_and_positive_bearing(self):
        plan = self.ctl.step(person(70), now=1)
        self.assertEqual(plan.yaw_target_deg, 2.5)
        self.assertGreater(plan.body_bearing_deg, 0)
        self.assertEqual(plan.suggested_body_turn, "TRACK_RIGHT")

    def test_head_pose_clamped_to_original_limits(self):
        for i in range(30):
            plan = self.ctl.step(person(310, 30, bh=90), now=1+i*.1)
        self.assertEqual(plan.yaw_target_deg, -20)
        self.assertLessEqual(abs(plan.pitch_target_deg), 12)

    def test_cropped_person_suppresses_unreliable_pitch(self):
        plan = self.ctl.step(person(200, 120, bh=239), now=1)
        self.assertFalse(plan.pitch_tracking_enabled)
        self.assertEqual(plan.pitch_target_deg, 0.0)

    def test_normal_person_updates_pitch(self):
        plan = self.ctl.step(person(160, 75, bh=100), now=1)
        self.assertTrue(plan.pitch_tracking_enabled)
        self.assertNotEqual(plan.pitch_target_deg, 0.0)

    def test_resolution_scaling_preserves_head_step(self):
        plan_small = self.ctl.step(person(250), now=1)
        other = V883PersonDualLoop()
        plan_large = other.step(person(500, fw=640, fh=480, cy=240, bw=180, bh=240), now=1)
        self.assertEqual(plan_small.yaw_target_deg, plan_large.yaw_target_deg)

    def test_fresh_feedback_required_for_body_command(self):
        for i in range(5):
            plan = self.ctl.step(person(270), now=1+i*.1, body_idle=True)
        self.assertEqual(plan.suggested_body_turn, "TRACK_LEFT")
        self.assertIsNone(plan.body_command)

    def test_two_confirmed_frames_propose_left(self):
        self.ctl.step(
            person(270), now=1,
            head_yaw_feedback_deg=0, head_feedback_fresh=True, body_idle=True
        )
        plan = self.ctl.step(
            person(270), now=1.1,
            head_yaw_feedback_deg=0, head_feedback_fresh=True, body_idle=True
        )
        self.assertEqual(plan.body_command, "TRACK_LEFT")
        self.assertTrue(plan.bearing_from_feedback)

    def test_two_confirmed_frames_propose_right(self):
        self.ctl.step(person(50), now=1, head_yaw_feedback_deg=0,
                      head_feedback_fresh=True, body_idle=True)
        plan = self.ctl.step(person(50), now=1.1, head_yaw_feedback_deg=0,
                             head_feedback_fresh=True, body_idle=True)
        self.assertEqual(plan.body_command, "TRACK_RIGHT")

    def test_busy_esp32_prohibits_command(self):
        for i in range(4):
            plan = self.ctl.step(person(270), now=1+i*.1, head_yaw_feedback_deg=0,
                                 head_feedback_fresh=True, body_idle=False)
        self.assertIsNone(plan.body_command)
        self.assertEqual(plan.reason, "WAIT_ESP32_IDLE_CONFIRMATION")

    def test_pending_motion_prevents_repeated_pulses(self):
        self.ctl.step(person(270), now=1, head_yaw_feedback_deg=0,
                      head_feedback_fresh=True, body_idle=True)
        self.ctl.step(person(270), now=1.1, head_yaw_feedback_deg=0,
                      head_feedback_fresh=True, body_idle=True)
        for i in range(5):
            plan = self.ctl.step(person(270), now=1.2+i*.1,
                                 head_yaw_feedback_deg=0,
                                 head_feedback_fresh=True, body_idle=True)
            self.assertIsNone(plan.body_command)

    def test_hysteresis_after_ack_and_cooldown(self):
        self.ctl.step(person(270), now=1, head_yaw_feedback_deg=0,
                      head_feedback_fresh=True, body_idle=True)
        self.ctl.step(person(270), now=1.1, head_yaw_feedback_deg=0,
                      head_feedback_fresh=True, body_idle=True)
        self.ctl.motion_completed(1.15, "TRACK_LEFT")
        early = self.ctl.step(person(270), now=1.2, head_yaw_feedback_deg=0,
                              head_feedback_fresh=True, body_idle=True)
        self.assertIsNone(early.body_command)
        later = self.ctl.step(person(270), now=1.4, head_yaw_feedback_deg=0,
                              head_feedback_fresh=True, body_idle=True)
        self.assertEqual(later.body_command, "TRACK_LEFT")

    def test_target_loss_no_head_or_body_command(self):
        self.ctl.step(person(270), now=1)
        lost = self.ctl.target_lost()
        self.assertFalse(lost.target_found)
        self.assertIsNone(lost.head_pose_command)
        self.assertIsNone(lost.body_command)
        self.assertEqual(lost.reason, "NO_TARGET_STOP_REQUIRED")

    def test_pose_rate_limit(self):
        first = self.ctl.step(person(270), now=1)
        second = self.ctl.step(person(270), now=1.01)
        third = self.ctl.step(person(270), now=1.1)
        self.assertIsNotNone(first.head_pose_command)
        self.assertIsNone(second.head_pose_command)
        self.assertIsNotNone(third.head_pose_command)

    def test_invalid_esp32_angle_cannot_authorize_turn(self):
        for i in range(5):
            plan = self.ctl.step(person(270), now=1+i*.1,
                                 head_yaw_feedback_deg=80,
                                 head_feedback_fresh=True, body_idle=True)
        self.assertIsNone(plan.body_command)

    def test_no_forward_or_backward_from_person_bbox(self):
        for x in [10, 50, 160, 260, 310]:
            ctl = V883PersonDualLoop()
            for i in range(3):
                p = ctl.step(person(x), now=1+i*.1, head_yaw_feedback_deg=0,
                             head_feedback_fresh=True, body_idle=True)
                self.assertNotIn(p.body_command, ["TRACK_FORWARD", "TRACK_BACKWARD"])

    def test_original_physical_baseline_constants_are_still_present(self):
        # Structural parity safeguard: ensures the original physical turn
        # calibration was not changed out from under this pure-Python port.
        repo_root = Path(__file__).resolve().parents[3]
        original = (repo_root / "raspberry_pi" / "follow_dual_loop_v8_8.py").read_text(
            encoding="utf-8"
        )
        for expected in (
            "HEAD_YAW_MIN_DEG = -20.0",
            "HEAD_YAW_MAX_DEG = 20.0",
            "HEAD_PITCH_MIN_DEG = -12.0",
            "HEAD_PITCH_MAX_DEG = 12.0",
            "HEAD_X_DEADZONE_PX = 14",
            "HEAD_Y_DEADZONE_PX = 12",
            "HEAD_YAW_GAIN_DEG_PER_PX = 0.055",
            "HEAD_PITCH_GAIN_DEG_PER_PX = 0.045",
            "BODY_BEARING_START_DEG = 9.0",
            "BODY_BEARING_CONTINUE_DEG = 4.5",
            "BODY_BEARING_STOP_DEG = 3.0",
            "BODY_NEGATIVE_BEARING_COMMAND",
            '"TRACK_LEFT"',
            "BODY_POSITIVE_BEARING_COMMAND",
            '"TRACK_RIGHT"',
            "CAMERA_X_TO_ROBOT_YAW_DIRECTION",
        ):
            self.assertIn(expected, original)


if __name__ == "__main__":
    unittest.main()
