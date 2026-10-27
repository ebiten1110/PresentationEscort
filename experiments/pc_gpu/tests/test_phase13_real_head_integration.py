"""Phase 13 guardrail tests. All UART interactions are mocked; no motors."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from person_target_adapter import detection_to_person_target
from pi_real_head_integration import HeadOnlyPlanner, HeadOnlySerialSink, LockedRealRuntime


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.commands = []
        self.closed = False

    def send(self, command):
        self.commands.append(command)

    def read_available(self, *, max_bytes=2048):
        return b""

    def close(self):
        self.closed = True


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        self.patcher = patch("pi_real_head_integration.SerialCommandSink", return_value=self.fake)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)

    def test_serial_probe_has_no_arm_or_motor_motion(self):
        sink = HeadOnlySerialSink("fake", head_enabled=False)
        sink.send("STATUS")
        sink.send("STOP")
        with self.assertRaises(ValueError):
            sink.send("SAFETY_ARM")
        with self.assertRaises(ValueError):
            sink.send("HEAD_POSE_SET:1,0")
        with self.assertRaises(ValueError):
            sink.send("TRACK_LEFT")
        self.assertEqual(self.fake.commands, ["STATUS", "STOP"])
        sink.close()

    def test_head_only_clamps_and_disables_body(self):
        sink = HeadOnlySerialSink("fake", head_enabled=True)
        sink.send("HEAD_POSE_SET:2.5,0.0")
        with self.assertRaises(ValueError):
            sink.send("HEAD_POSE_SET:4,0")
        with self.assertRaises(ValueError):
            sink.send("HEAD_POSE_SET:1,1")
        with self.assertRaises(ValueError):
            sink.send("TRACK_RIGHT")
        self.assertEqual(self.fake.commands, ["HEAD_POSE_SET:2.5,0.0"])
        sink.close()

    def test_real_probe_never_pretends_to_be_virtual(self):
        runtime = LockedRealRuntime("fake", head_enabled=False)
        self.addCleanup(runtime.close)
        status = runtime.status()
        self.assertFalse(status["simulation"])
        self.assertFalse(status["auto_ready"])
        self.assertFalse(status["body_motion_permitted"])
        self.assertFalse(runtime.action("auto")["ok"])
        self.assertFalse(runtime.action("manual")["ok"])

    def test_head_planner_never_emits_body_or_pitch(self):
        detection = DetectionResult(
            found=True, center_x=270, center_y=90,
            width=70, height=140, confidence=.9,
            class_id=0, class_name="person",
            frame_width=320, frame_height=240)
        target = detection_to_person_target(detection)
        self.assertIsNotNone(target)
        planner = HeadOnlyPlanner()
        for n in range(3):
            plan = planner.step(
                target, now=10 + n * .1, head_yaw_feedback_deg=0.,
                head_feedback_fresh=True, body_idle=True)
            self.assertIsNone(plan.body_command)
            self.assertIsNone(plan.suggested_body_turn)
            self.assertLessEqual(abs(plan.yaw_target_deg), 3)
            self.assertEqual(plan.pitch_target_deg, 0)


if __name__ == "__main__":
    unittest.main()
