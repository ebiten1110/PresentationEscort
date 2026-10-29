"""Phase 28: GPU track continuity and visual-servo recovery (hardware-free)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from presenter_lock import PresenterLock
from person_target_adapter import detection_to_person_target
from pi_real_head_integration import HeadOnlyPlanner


def frame(seq: int, x: int | None, *, track_id: int | None = 7,
          gesture: bool = False, second: int | None = None,
          confidence: float = .94) -> DetectionResult:
    people = []
    track_ids = []
    if x is not None:
        people.append({"x1": x - 20, "y1": 50, "x2": x + 20,
                       "y2": 170, "confidence": confidence})
        track_ids.append(track_id)
    if second is not None:
        people.append({"x1": second - 20, "y1": 50,
                       "x2": second + 20, "y2": 170,
                       "confidence": .96})
        track_ids.append(8)
    return DetectionResult(
        found=bool(people), frame_width=320, frame_height=240,
        frame_seq=seq, protocol_version=2, people=people,
        track_ids=track_ids, gesture_indices=[0] if gesture else [],
    )


def locked_tracker():
    lock = PresenterLock(gesture_enabled=True,
                         gesture_confirm_frames=3,
                         transient_recovery_s=1.5)
    for seq in range(1, 4):
        result = lock.observe(frame(seq, 95, gesture=True),
                              now=1.0 + .1 * seq)
    assert lock.state == "TRACKING" and result.found
    return lock


def target(x: int, *, box_width: int = 90, box_height: int = 160):
    result = DetectionResult(
        found=True, center_x=x, center_y=120,
        width=box_width, height=box_height,
        frame_width=320, frame_height=240,
        aim_x=x, aim_y=120, aim_source="face",
        confidence=.94, class_id=0, class_name="person")
    converted = detection_to_person_target(result)
    assert converted is not None
    return converted


class GpuTrackingContinuityTests(unittest.TestCase):
    def test_same_gpu_track_allows_quick_single_person_motion(self):
        lock = locked_tracker()
        result = lock.observe(frame(4, 145), now=1.4)
        self.assertTrue(result.found)
        self.assertEqual(lock.state, "TRACKING")
        self.assertEqual(lock._locked_track_id, 7)
        self.assertGreater(lock._velocity_x, 0.)

    def test_same_fast_jump_without_id_does_not_silently_handoff(self):
        lock = locked_tracker()
        result = lock.observe(frame(4, 145, track_id=None), now=1.4)
        self.assertFalse(result.found)
        self.assertEqual(lock.state, "LOST")

    def test_short_loss_reacquires_using_predicted_position(self):
        lock = locked_tracker()
        self.assertTrue(lock.observe(frame(4, 145), now=1.4).found)
        self.assertFalse(lock.observe(frame(5, None), now=1.5).found)
        self.assertEqual(lock.state, "TEMPORARILY_LOST")
        result = lock.observe(frame(6, 190), now=1.6)
        self.assertTrue(result.found)
        self.assertEqual(lock.state, "TRACKING")
        # Geometry alone never triggers a new gesture or ARM.
        self.assertEqual(lock.gesture_event_id, 1)
        self.assertTrue(lock._needs_resume_gesture)
        lock.confirm_bounded_reacquisition()
        self.assertFalse(lock._needs_resume_gesture)

    def test_ambiguous_crowd_still_invalidates_lock(self):
        lock = locked_tracker()
        result = lock.observe(frame(4, 120, second=146), now=1.4)
        self.assertFalse(result.found)
        self.assertEqual(lock.state, "LOST")

    def test_bad_tracker_ids_fail_closed(self):
        lock = locked_tracker()
        bad = frame(4, 99)
        bad.track_ids = [7, 8]
        self.assertFalse(lock.observe(bad, now=1.4).found)
        self.assertEqual(lock.reason, "INVALID_TRACK_IDS")

    def test_protocol_v2_accepts_optional_temporal_track_ids(self):
        item = frame(1, 100)
        restored = DetectionResult.from_dict(item.to_dict())
        self.assertEqual(restored.track_ids, [7])
        self.assertEqual(restored.protocol_version, 2)
        self.assertEqual(DetectionResult(found=False).track_ids, [])


class VisualServoTests(unittest.TestCase):
    def test_small_optical_probe_waits_for_camera_response(self):
        planner = HeadOnlyPlanner(vision_verified_head=True,
                                  distance_adaptive_head=True,
                                  head_speed_deg_s=3.5)
        first = planner.step(target(40), now=1.0,
                             head_yaw_feedback_deg=0.,
                             head_feedback_fresh=True)
        self.assertIsNotNone(first.head_pose_command)
        self.assertLessEqual(abs(first.yaw_target_deg), .3)
        second = planner.step(target(40), now=1.12,
                              head_yaw_feedback_deg=first.yaw_target_deg,
                              head_feedback_fresh=True)
        self.assertEqual(second.yaw_target_deg, first.yaw_target_deg)
        self.assertIsNone(second.head_pose_command)
        third = planner.step(target(90), now=1.35,
                             head_yaw_feedback_deg=second.yaw_target_deg,
                             head_feedback_fresh=True)
        self.assertLessEqual(abs(third.yaw_target_deg), .6)

    def test_far_person_yaw_probe_is_smaller(self):
        def first(width, height):
            planner = HeadOnlyPlanner(vision_verified_head=True,
                                      distance_adaptive_head=True,
                                      head_speed_deg_s=10.)
            return planner.step(
                target(40, box_width=width, box_height=height),
                now=1.0, head_yaw_feedback_deg=0.,
                head_feedback_fresh=True).yaw_target_deg
        self.assertLess(abs(first(20, 40)), abs(first(95, 170)))

    def test_repeated_worsening_temporarily_holds_yaw_only(self):
        planner = HeadOnlyPlanner(vision_verified_head=True,
                                  head_speed_deg_s=3.5)
        first = planner.step(target(80), now=1.,
                             head_yaw_feedback_deg=0.,
                             head_feedback_fresh=True)
        second = planner.step(target(40), now=1.32,
                              head_yaw_feedback_deg=first.yaw_target_deg,
                              head_feedback_fresh=True)
        third = planner.step(target(5), now=1.65,
                             head_yaw_feedback_deg=second.yaw_target_deg,
                             head_feedback_fresh=True)
        self.assertEqual(third.yaw_target_deg, second.yaw_target_deg)
        self.assertGreater(planner._yaw_hold_until, 2.)
        still = planner.step(target(5), now=1.85,
                             head_yaw_feedback_deg=third.yaw_target_deg,
                             head_feedback_fresh=True)
        self.assertEqual(still.yaw_target_deg, third.yaw_target_deg)
        planner.target_lost()
        self.assertIsNone(planner._yaw_visual_probe)

    def test_default_head_planner_retains_legacy_mode(self):
        planner = HeadOnlyPlanner()
        self.assertFalse(planner.vision_verified_head)


if __name__ == "__main__":
    unittest.main()
