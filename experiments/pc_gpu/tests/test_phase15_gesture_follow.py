"""Gesture-initiated, operator-opted-in, real-UART virtual-bench tests.

No camera/model import and NO actual USB/motor operation.
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from gesture_pose import both_hands_over_head, face_or_upper_body_aim
from presenter_lock import PresenterLock
from person_target_adapter import detection_to_person_target
from pi_real_head_integration import HeadBodyTurnPlanner, LockedRealRuntime
from virtual_esp32 import VirtualEsp32


def points(raised: bool = True):
    p = [[0., 0., 0.] for _ in range(17)]
    p[0] = [255., 90., .96]         # nose
    p[5] = [235., 140., .91]        # shoulders
    p[6] = [275., 140., .90]
    p[9] = [231., 45. if raised else 190., .93]
    p[10] = [283., 40. if raised else 185., .91]
    return p


def frame(seq, *, hands=True, visible=True, two=False, aim=True):
    people = ([{"x1": 208, "y1": 25, "x2": 300, "y2": 215, "confidence": .90}]
              if visible else [])
    if two:
        people.append({"x1": 10, "y1": 30, "x2": 100, "y2": 210, "confidence": .88})
    return DetectionResult(
        found=bool(people), frame_width=320, frame_height=240,
        protocol_version=2, frame_seq=seq, people=people,
        gesture_indices=[0] if hands and visible else [],
        aim_points=[{"x": 255, "y": 90, "source": "face"}] if aim and visible else [],
        inference_ms=27.,
    )


def wait_for(predicate, timeout_s=1.1):
    limit = time.monotonic() + timeout_s
    while time.monotonic() < limit:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError("condition not reached")


class PoseGeometryTests(unittest.TestCase):
    def test_both_wrists_above_head_required(self):
        self.assertTrue(both_hands_over_head(points()))
        self.assertFalse(both_hands_over_head(points(raised=False)))
        invisible = points()
        invisible[10][2] = 0.3
        self.assertFalse(both_hands_over_head(invisible))

    def test_face_center_and_shoulder_fallback(self):
        self.assertEqual(face_or_upper_body_aim(points(), (200, 20, 310, 220)),
                         (255, 90, "face"))
        missing_nose = points()
        missing_nose[0][2] = 0.
        self.assertEqual(face_or_upper_body_aim(missing_nose, (200, 20, 310, 220)),
                         (255, 140, "shoulders"))

    def test_real_face_aim_transfers_to_error_calculation(self):
        lock = PresenterLock(gesture_enabled=True, gesture_confirm_frames=4)
        for i in range(1, 5):
            selected = lock.observe(frame(i), now=i * .05)
        self.assertTrue(selected.found)
        self.assertEqual((selected.aim_x, selected.aim_y), (255, 90))
        target = detection_to_person_target(selected)
        self.assertTrue(target.aim_from_pose)
        self.assertEqual((target.error_x, target.error_y), (95, -30))

    def test_gesture_requires_stable_only_one_candidate(self):
        lock = PresenterLock(gesture_enabled=True)
        self.assertFalse(lock.observe(frame(1, hands=False), now=.05).found)
        for i in range(2, 6):
            selected = lock.observe(frame(i, hands=True, two=True), now=.05*i)
        self.assertTrue(selected.found)
        self.assertEqual(lock.display(now=.25)["gesture_event_id"], 1)

    def test_two_gesturing_people_does_not_select(self):
        lock = PresenterLock(gesture_enabled=True)
        for i in range(1, 9):
            sample = frame(i, two=True)
            sample.gesture_indices = [0, 1]
            self.assertFalse(lock.observe(sample, now=.05*i).found)
        self.assertEqual(lock.gesture_event_id, 0)

    def test_complete_loss_requires_new_gesture(self):
        lock = PresenterLock(gesture_enabled=True, transient_recovery_s=.2)
        for i in range(1, 5):
            lock.observe(frame(i), now=.05*i)
        self.assertTrue(lock.locked)
        lock.observe(frame(5, visible=False), now=.25)
        self.assertTrue(lock.locked)
        lock.observe(frame(6, visible=False), now=.60)
        self.assertFalse(lock.locked)
        for i in range(7, 10):
            self.assertFalse(lock.observe(frame(i, hands=False), now=.60 + i*.05).found)
        self.assertFalse(lock.locked)
        for i in range(10, 14):
            recovered = lock.observe(frame(i), now=.60 + i*.05)
        self.assertTrue(recovered.found)
        self.assertEqual(lock.gesture_event_id, 2)


class GestureAutoRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.uart = VirtualEsp32()
        serial_patch = patch("pi_real_head_integration.SerialCommandSink", return_value=self.uart)
        serial_patch.start()
        self.addCleanup(serial_patch.stop)
        self.runtime = LockedRealRuntime(
            "mock-ch340", head_enabled=True, body_enabled=True,
            power_declared="ON_OPERATOR_DECLARED",
        )
        self.addCleanup(self.runtime.close)
        self.runtime.start()
        wait_for(lambda: self.runtime.status()["fw_safety"] == "DISARMED")

    def test_gesture_never_starts_motors_before_operator_opt_in(self):
        for i in range(1, 9):
            self.runtime.process_detection(frame(i), 55.)
        self.assertEqual(self.runtime.status()["lock_state"], "TRACKING")
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotIn("SAFETY_ARM", self.uart.commands)

    def test_explicit_gesture_standby_starts_auto_once(self):
        self.assertTrue(self.runtime.action("gesture_on")["ok"])
        for i in range(1, 12):
            self.runtime.process_detection(frame(i), 55.)
        wait_for(lambda: self.runtime.status()["mode"] == "AUTO")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)
        self.assertTrue(self.runtime.status()["gesture_control_enabled"])
        self.runtime.action("stop")
        self.assertFalse(self.runtime.status()["gesture_control_enabled"])

    def test_loss_requires_fresh_raised_hands_for_new_arm(self):
        self.assertTrue(self.runtime.action("gesture_on")["ok"])
        for i in range(1, 10):
            self.runtime.process_detection(frame(i), 55.)
        wait_for(lambda: self.runtime.status()["mode"] == "AUTO")
        self.runtime.presenter.transient_recovery_s = 0
        self.runtime.process_detection(frame(10, visible=False), 55.)
        wait_for(lambda: self.runtime.status()["mode"] == "SAFE")
        self.assertTrue(self.runtime.status()["gesture_control_enabled"])
        for i in range(11, 16):
            self.runtime.process_detection(frame(i, hands=False), 55.)
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)
        for i in range(16, 25):
            self.runtime.process_detection(frame(i, hands=True), 55.)
        wait_for(lambda: self.runtime.status()["mode"] == "AUTO")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 2)

    def test_brief_loss_requires_new_gesture_not_reappearance(self):
        self.assertTrue(self.runtime.action("gesture_on")["ok"])
        for i in range(1, 10):
            self.runtime.process_detection(frame(i), 55.)
        wait_for(lambda: self.runtime.status()["mode"] == "AUTO")
        self.runtime.process_detection(frame(10, visible=False), 55.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        for i in range(11, 16):
            self.runtime.process_detection(frame(i, hands=False), 55.)
        self.assertEqual(self.runtime.status()["lock_state"], "TRACKING")
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)
        for i in range(16, 26):
            self.runtime.process_detection(frame(i, hands=True), 55.)
        wait_for(lambda: self.runtime.status()["mode"] == "AUTO")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 2)

    def test_disconnect_clears_gesture_standby(self):
        self.assertTrue(self.runtime.action("gesture_on")["ok"])
        self.runtime.set_camera_state(False, False)
        self.assertFalse(self.runtime.status()["gesture_control_enabled"])
        self.assertNotIn("SAFETY_ARM", self.uart.commands)

    def test_horizontal_direction_inverted_from_legacy(self):
        target = detection_to_person_target(DetectionResult(
            found=True, class_id=0, class_name="person", confidence=.90,
            center_x=280, center_y=115, width=70, height=130,
            frame_width=320, frame_height=240,
        ))
        normal = HeadBodyTurnPlanner(invert_horizontal=False)
        reversed_planner = HeadBodyTurnPlanner(invert_horizontal=True)
        left = normal.step(target, now=1., head_yaw_feedback_deg=0.,
                           head_feedback_fresh=True, body_idle=True)
        right = reversed_planner.step(target, now=1., head_yaw_feedback_deg=0.,
                                      head_feedback_fresh=True, body_idle=True)
        self.assertLess(left.body_bearing_deg, 0)
        self.assertGreater(right.body_bearing_deg, 0)
        self.assertLess(left.yaw_target_deg, 0)
        self.assertGreater(right.yaw_target_deg, 0)


if __name__ == "__main__":
    unittest.main()
