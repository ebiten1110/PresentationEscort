"""Two-axis proportional face aim, FPS independence and safety (no hardware)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from person_target_adapter import detection_to_person_target
from pi_real_head_integration import HeadBodyTurnPlanner, HeadOnlyPlanner
from pi_operator_console import CameraInferenceWorker


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
    def test_relative_pitch_reference_unpins_high_face_without_expanding_range(self):
        planner = HeadOnlyPlanner(pitch_min=-2., pitch_max=2.,
                                  relative_pitch_reference=True,
                                  head_speed_deg_s=2., pitch_speed_deg_s=1.5)
        planner.pitch_target_deg = -2.
        feedback = -2.
        for i in range(20):
            plan = planner.step(face_target(x=160, y=80), now=1.+i*.12,
                                head_yaw_feedback_deg=0.,
                                head_pitch_feedback_deg=feedback,
                                head_feedback_fresh=True)
            feedback = plan.pitch_target_deg
        self.assertAlmostEqual(feedback, 0., delta=.15)
        for i in range(20):
            plan = planner.step(face_target(x=160, y=160), now=4.+i*.12,
                                head_yaw_feedback_deg=0.,
                                head_pitch_feedback_deg=feedback,
                                head_feedback_fresh=True)
            feedback = plan.pitch_target_deg
            self.assertLessEqual(abs(feedback), 2.)
        self.assertGreater(feedback, 1.2)
        planner.target_lost()
        self.assertIsNone(planner.pitch_reference_y)

    def test_reverse_body_turn_does_not_reverse_head_yaw(self):
        def obtain_turn(reverse):
            planner = HeadBodyTurnPlanner(pitch_min=-2., pitch_max=2.,
                stable_frames=2, head_speed_deg_s=2.,
                reverse_body_turn_direction=reverse)
            feedback_yaw = 0.
            for i in range(35):
                plan = planner.step(face_target(x=5, y=120), now=1.+i*.12,
                    head_yaw_feedback_deg=feedback_yaw,
                    head_pitch_feedback_deg=0., head_feedback_fresh=True,
                    body_idle=True)
                feedback_yaw = plan.yaw_target_deg
                if plan.body_command:
                    return plan.body_command, plan.yaw_target_deg
            self.fail("expected bounded turn proposal")
        normal, normal_yaw = obtain_turn(False)
        flipped, flipped_yaw = obtain_turn(True)
        self.assertEqual({normal, flipped}, {"TRACK_LEFT", "TRACK_RIGHT"})
        self.assertEqual(normal_yaw, flipped_yaw)

    def test_v883_physical_yaw_sign_restored_by_default(self):
        planner = HeadOnlyPlanner()
        # Camera screen LEFT requests positive yaw in the old V8.8.3 mapping.
        plan = planner.step(face_target(x=40, y=120), now=1.,
                            head_yaw_feedback_deg=0., head_feedback_fresh=True)
        self.assertGreater(plan.yaw_target_deg, 0.)
        self.assertEqual(plan.pitch_target_deg, 0.)

    def test_explicit_reversal_remains_available(self):
        planner = HeadOnlyPlanner(invert_horizontal=True)
        plan = planner.step(face_target(x=40, y=120), now=1.,
                            head_yaw_feedback_deg=0., head_feedback_fresh=True)
        self.assertLess(plan.yaw_target_deg, 0.)

    def test_face_target_slews_both_axes_at_three_deg_per_second(self):
        planner = HeadOnlyPlanner(pitch_min=-3., pitch_max=3., head_speed_deg_s=3.)
        angles = []
        feedback_yaw, feedback_pitch = 0., 0.
        for i in range(26):
            plan = planner.step(
                face_target(x=40, y=20), now=1.+i*.1,
                head_yaw_feedback_deg=feedback_yaw,
                head_pitch_feedback_deg=feedback_pitch,
                head_feedback_fresh=True)
            angles.append((plan.yaw_target_deg, plan.pitch_target_deg))
            feedback_yaw, feedback_pitch = plan.yaw_target_deg, plan.pitch_target_deg
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
            plan = planner.step(face_target(x=120, y=10), now=1.+i*.15,
                                head_yaw_feedback_deg=0., head_feedback_fresh=True)
            self.assertEqual(plan.pitch_target_deg, 0.)
            self.assertTrue(plan.head_pose_command is None or
                            plan.head_pose_command.endswith(",0.0"))

    def test_short_camera_jitter_within_deadzone_does_not_steer(self):
        planner = HeadOnlyPlanner(pitch_min=-3., pitch_max=3.)
        for i, offset in enumerate((0, 2, -3, 4, -2, 1, 0)):
            plan = planner.step(face_target(x=160+offset, y=120+offset),
                                now=1.+i*.15, head_yaw_feedback_deg=0.,
                                head_pitch_feedback_deg=0., head_feedback_fresh=True)
            self.assertEqual(plan.yaw_target_deg, 0.)
            self.assertEqual(plan.pitch_target_deg, 0.)

    def test_lost_target_sends_no_pose_and_resets_pixel_filter(self):
        planner = HeadOnlyPlanner()
        planner.step(face_target(x=40, y=100), now=1.,
                     head_yaw_feedback_deg=0., head_feedback_fresh=True)
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

    def test_sixty_fps_stagnant_feedback_does_not_accumulate_degrees(self):
        planner = HeadOnlyPlanner(pitch_min=-3., pitch_max=3.)
        # Old pixel integrator would saturate at +/-3 degrees merely because
        # the GPU returned more frames, even if the neck had not moved.
        for i in range(240):
            plan = planner.step(face_target(x=40, y=20), now=1.+i/60.,
                                head_yaw_feedback_deg=0., head_pitch_feedback_deg=0.,
                                head_feedback_fresh=True)
            self.assertLessEqual(abs(plan.yaw_target_deg), .7)
            self.assertLessEqual(abs(plan.pitch_target_deg), .7)
        self.assertLessEqual(abs(planner.yaw_target_deg), .7)

    def test_same_elapsed_time_has_similar_twenty_and_sixty_fps_motion(self):
        def final_yaw(fps):
            planner = HeadOnlyPlanner(head_speed_deg_s=2.)
            for i in range(fps*2):
                planner.step(face_target(x=40, y=120), now=1.+i/fps,
                             head_yaw_feedback_deg=0., head_feedback_fresh=True)
            return planner.yaw_target_deg
        self.assertLessEqual(abs(final_yaw(20)-final_yaw(60)), .2)

    def test_when_face_centers_head_holds_reported_angle(self):
        planner = HeadOnlyPlanner(pitch_min=-3., pitch_max=3.)
        yaw_positions = []
        for i in range(6):
            plan = planner.step(face_target(x=160, y=120), now=1.+i*.1,
                                head_yaw_feedback_deg=1.1,
                                head_pitch_feedback_deg=-0.9,
                                head_feedback_fresh=True)
            yaw_positions.append(plan.yaw_target_deg)
        self.assertEqual(yaw_positions, sorted(yaw_positions))
        # The controller should converge to the reported current angle,
        # not keep winding past it with a zero-pixel error.
        self.assertAlmostEqual(plan.yaw_target_deg, 1.1, places=1)
        self.assertAlmostEqual(plan.pitch_target_deg, -0.9, places=1)

    def test_stale_head_feedback_cannot_issue_new_head_pose(self):
        planner = HeadOnlyPlanner(pitch_min=-2., pitch_max=2.)
        for i in range(50):
            plan = planner.step(face_target(x=20, y=15), now=1.+i/30.,
                                head_yaw_feedback_deg=None,
                                head_pitch_feedback_deg=None,
                                head_feedback_fresh=False)
            self.assertIsNone(plan.head_pose_command)
            self.assertEqual((plan.yaw_target_deg, plan.pitch_target_deg), (0., 0.))

    def test_camera_fps_request_is_bounded(self):
        self.assertEqual(CameraInferenceWorker(None, "127.0.0.1", camera_fps=60).camera_fps, 60)
        for invalid in (0, 3, 61, 100):
            with self.assertRaises(ValueError):
                CameraInferenceWorker(None, "127.0.0.1", camera_fps=invalid)

    def test_speed_limit_validation(self):
        for rate in (0., -3., float("nan"), float("inf"), 20.):
            with self.assertRaises(ValueError):
                HeadOnlyPlanner(head_speed_deg_s=rate)


if __name__ == "__main__":
    unittest.main()
