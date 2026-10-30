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


def sized_face_target(*, x: int, width: int, height: int):
    detection = DetectionResult(
        found=True, center_x=x, center_y=110, width=width, height=height,
        class_id=0, class_name="person", confidence=.95,
        aim_x=x, aim_y=120, aim_source="face",
        frame_width=320, frame_height=240,
    )
    target = detection_to_person_target(detection)
    assert target is not None
    return target


class SmoothFaceTests(unittest.TestCase):
    def test_faraway_person_damps_yaw_not_physical_angle_limit(self):
        def first_command(width, height, adaptive):
            planner = HeadOnlyPlanner(head_speed_deg_s=10.,
                distance_adaptive_head=adaptive)
            plan = planner.step(sized_face_target(x=40, width=width, height=height),
                now=1., head_yaw_feedback_deg=0.,
                head_feedback_fresh=True)
            return plan.yaw_target_deg, planner.distance_gain
        near, near_gain = first_command(95, 175, True)
        far, far_gain = first_command(20, 40, True)
        legacy, legacy_gain = first_command(20, 40, False)
        self.assertAlmostEqual(near_gain, 1.)
        self.assertLess(far_gain, .5)
        self.assertLess(abs(far), abs(near))
        self.assertLess(abs(far), abs(legacy))
        self.assertEqual(legacy_gain, 1.)

    def test_responsive_follow_reduces_filter_lag_without_exceeding_slew_limit(self):
        def second_command(responsive):
            planner = HeadOnlyPlanner(head_speed_deg_s=10.,
                                       responsive_follow=responsive)
            planner.step(face_target(x=160, y=120), now=1.,
                         head_yaw_feedback_deg=0., head_feedback_fresh=True)
            plan = planner.step(face_target(x=125, y=120), now=1.10,
                                head_yaw_feedback_deg=0., head_feedback_fresh=True)
            return plan.yaw_target_deg
        old = second_command(False)
        quick = second_command(True)
        self.assertGreater(abs(quick), abs(old))
        self.assertLessEqual(abs(quick), 1.)

    def test_far_distance_neck_soft_limit_hands_off_to_body(self):
        far = sized_face_target(x=15, width=24, height=48)
        head = HeadOnlyPlanner(distance_adaptive_head=True, head_speed_deg_s=3.5)
        yaw = 0.
        for i in range(24):
            plan = head.step(far, now=1.+i*.12,
                             head_yaw_feedback_deg=yaw,
                             head_feedback_fresh=True)
            yaw = plan.yaw_target_deg
        self.assertLessEqual(abs(yaw), 1.2,
                             "far person should not pin the camera at +/-3 deg")
        body = HeadBodyTurnPlanner(distance_adaptive_head=True,
            stable_frames=2, head_speed_deg_s=3.5)
        yaw = 0.
        turn = None
        for i in range(32):
            plan = body.step(far, now=1.+i*.12,
                             head_yaw_feedback_deg=yaw,
                             head_pitch_feedback_deg=0.,
                             head_feedback_fresh=True, body_idle=True)
            yaw = plan.yaw_target_deg
            if plan.body_command:
                turn = plan
                break
        self.assertIsNotNone(turn,
                             "body should take over while the distant head remains near neutral")
        self.assertLess(abs(turn.yaw_target_deg), 2.5)

    def test_adaptive_close_person_uses_body_before_neck_hits_three_degrees(self):
        target = sized_face_target(x=260, width=95, height=175)
        planner = HeadBodyTurnPlanner(distance_adaptive_head=True,
            stable_frames=2, head_speed_deg_s=3.5)
        feedback = 0.
        turned = False
        for i in range(40):
            plan = planner.step(target, now=1. + i * .12,
                head_yaw_feedback_deg=feedback,
                head_feedback_fresh=True, body_idle=True)
            feedback = plan.yaw_target_deg
            self.assertLessEqual(abs(feedback), 1.51)
            if plan.body_command:
                turned = True
                break
        self.assertTrue(turned, "body should take a large angular displacement")

    def test_locked_center_does_not_reset_an_already_rotated_neck(self):
        planner = HeadOnlyPlanner(distance_adaptive_head=True)
        plan = planner.step(face_target(x=160, y=120), now=1.,
            head_yaw_feedback_deg=-3., head_feedback_fresh=True)
        self.assertEqual(plan.yaw_target_deg, -3.)
        for i in range(1, 6):
            plan = planner.step(face_target(x=160, y=120), now=1. + .15 * i,
                head_yaw_feedback_deg=-3., head_feedback_fresh=True)
            self.assertEqual(plan.yaw_target_deg, -3.)

    def test_head_and_body_hold_until_armed_and_three_fresh_frames(self):
        planner = HeadBodyTurnPlanner(distance_adaptive_head=True,
            stable_frames=2, head_speed_deg_s=3.5,
            pitch_min=-2., pitch_max=2.)
        target = face_target(x=40, y=120)
        planner.set_actuation_gate(False)
        for i in range(8):
            plan = planner.step(target, now=1. + .12 * i,
                head_yaw_feedback_deg=-2.5,
                head_pitch_feedback_deg=.3,
                head_feedback_fresh=True, body_idle=True)
            self.assertIsNone(plan.head_pose_command)
            self.assertIsNone(plan.body_command)
            self.assertEqual(plan.yaw_target_deg, -2.5)
        planner.set_actuation_gate(True)
        for i in range(3):
            plan = planner.step(target, now=2. + .12 * i,
                head_yaw_feedback_deg=-2.5,
                head_pitch_feedback_deg=.3,
                head_feedback_fresh=True, body_idle=True)
            self.assertEqual(plan.reason, "TRACKING_STABILIZING")
            self.assertIsNone(plan.head_pose_command)
            self.assertIsNone(plan.body_command)
        plan = planner.step(target, now=2.4,
            head_yaw_feedback_deg=-2.5, head_pitch_feedback_deg=.3,
            head_feedback_fresh=True, body_idle=True)
        self.assertLessEqual(abs(plan.yaw_target_deg - (-2.5)), .5)

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

    def test_preview_decimation_preserves_tracking_frame_contract(self):
        worker = CameraInferenceWorker(None, "127.0.0.1",
                                        preview_every_n_frames=3)
        self.assertEqual(worker.preview_every_n_frames, 3)
        for invalid in (0, -1, 11, 1.5, True):
            with self.assertRaises(ValueError):
                CameraInferenceWorker(None, "127.0.0.1",
                                      preview_every_n_frames=invalid)

    def test_speed_limit_validation(self):
        for rate in (0., -3., float("nan"), float("inf"), 20.):
            with self.assertRaises(ValueError):
                HeadOnlyPlanner(head_speed_deg_s=rate)


if __name__ == "__main__":
    unittest.main()
