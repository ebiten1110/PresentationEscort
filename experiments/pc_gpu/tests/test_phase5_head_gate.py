"""Software-only head-motion safety and telemetry-driven V8.8.3 integration."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from esp32_telemetry import Esp32Telemetry
from motion_gateway import MemoryCommandSink, PiMotionGateway
from remote_person_controller import RemotePersonController
from safety_manager import Mode


def detect(cx=270, found=True):
    return DetectionResult(
        found=found, center_x=cx, center_y=110, width=65, height=100,
        confidence=0.9, class_id=0, class_name="person",
        frame_width=320, frame_height=240,
    )


class HeadGateTests(unittest.TestCase):
    def setUp(self):
        self.sink = MemoryCommandSink()
        self.g = PiMotionGateway(self.sink)
        self.addCleanup(self.g.close)

    def prime(self):
        for i in range(3):
            self.g.observe_detection(True, 70, now=10+i*.1)

    def test_head_blocked_disarmed(self):
        self.prime()
        d = self.g.request_head_pose(-3, 1, now=10.25)
        self.assertFalse(d.approved)
        self.assertFalse(any(x.startswith("HEAD_POSE_SET") for x in self.sink.commands))

    def test_head_allowed_auto(self):
        self.prime()
        self.assertTrue(self.g.arm_auto(now=10.25))
        d = self.g.request_head_pose(-3, 2, now=10.26)
        self.assertTrue(d.approved)
        self.assertEqual(self.sink.commands[-1], "HEAD_POSE_SET:-3.0,2.0")

    def test_head_blocked_in_manual(self):
        self.assertTrue(self.g.arm_manual(now=10))
        d = self.g.request_head_pose(1, 0, now=10.1)
        self.assertFalse(d.approved)

    def test_head_blocked_estop(self):
        self.prime()
        self.g.arm_auto(now=10.25)
        self.g.emergency_stop()
        self.assertFalse(self.g.request_head_pose(1, 0, now=10.26).approved)

    def test_head_oversized_or_nan_rejected(self):
        self.prime()
        self.g.arm_auto(now=10.25)
        for yaw, pitch in [(90, 0), (0, 13), (float("nan"), 0), (0, float("inf"))]:
            self.assertFalse(self.g.request_head_pose(yaw, pitch, now=10.26).approved)

    def test_head_blocked_after_target_lost(self):
        self.prime()
        self.g.arm_auto(now=10.25)
        self.g.observe_detection(False, 100, now=10.3)
        self.assertFalse(self.g.request_head_pose(0, 0, now=10.31).approved)


class RemoteControllerTests(unittest.TestCase):
    def setUp(self):
        self.sink = MemoryCommandSink()
        self.g = PiMotionGateway(self.sink)
        self.addCleanup(self.g.close)
        self.ctrl = RemotePersonController(self.g, enable_actuation=True)
        self.t = self.ctrl.telemetry

    def telemetry_idle(self, now=10.0):
        self.ctrl.feed_esp32(
            b"[HeadState] currentYaw=0.0 targetYaw=0 currentPitch=0.0 targetPitch=0 moving=NO\n"
            b"Walking State: IDLE\nBusy: NO\n",
            now=now,
        )

    def prime(self):
        for i in range(3):
            self.ctrl.process(detect(270), rtt_ms=70, now=10+i*.1)

    def test_default_controller_is_dry_run(self):
        dry = RemotePersonController(self.g)
        self.telemetry_idle()
        dry.process(detect(), rtt_ms=70, now=10)
        self.assertEqual(self.sink.commands, [])

    def test_no_head_without_esp32_feedback(self):
        self.prime()
        self.assertTrue(self.g.arm_auto(now=10.25))
        cycle = self.ctrl.process(detect(270), rtt_ms=70, now=10.26)
        self.assertFalse(cycle.head_sent)
        self.assertFalse(cycle.body_sent)

    def test_head_sent_only_after_telemetry_and_arm(self):
        self.prime()
        self.assertTrue(self.g.arm_auto(now=10.25))
        self.telemetry_idle(now=10.25)
        cycle = self.ctrl.process(detect(270), rtt_ms=70, now=10.26)
        self.assertTrue(cycle.head_sent)
        self.assertTrue(any(x.startswith("HEAD_POSE_SET") for x in self.sink.commands))

    def test_body_waits_idle_feedback(self):
        self.prime()
        self.g.arm_auto(now=10.25)
        self.ctrl.feed_esp32(
            b"[HeadState] currentYaw=0 targetYaw=0 currentPitch=0 targetPitch=0 moving=NO\n", now=10.25
        )
        for now in (10.26, 10.30):
            c = self.ctrl.process(detect(270), rtt_ms=70, now=now)
        self.assertFalse(c.body_sent)
        self.assertNotIn("TRACK_LEFT", self.sink.commands)

    def test_fresh_idle_and_yaw_allows_body(self):
        self.prime()
        self.g.arm_auto(now=10.25)
        self.telemetry_idle(now=10.25)
        c1 = self.ctrl.process(detect(270), rtt_ms=70, now=10.26)
        c2 = self.ctrl.process(detect(270), rtt_ms=70, now=10.36)
        self.assertTrue(c1.head_sent)
        self.assertTrue(c2.body_sent)
        self.assertIn("TRACK_LEFT", self.sink.commands)

    def test_lost_person_sends_stop(self):
        self.prime()
        self.g.arm_auto(now=10.25)
        self.telemetry_idle(now=10.25)
        self.ctrl.process(detect(270), rtt_ms=70, now=10.26)
        self.ctrl.process(detect(270), rtt_ms=70, now=10.36)
        self.ctrl.process(detect(found=False), rtt_ms=70, now=10.40)
        self.assertEqual(self.sink.commands[-1], "STOP")
        self.assertEqual(self.g.status(now=10.41).safety.mode, Mode.SAFE)


if __name__ == "__main__":
    unittest.main()
