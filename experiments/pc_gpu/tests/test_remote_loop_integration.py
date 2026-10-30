"""Remote YOLO person → V8.8.3 planner → Pi safety gateway integration.

Software-only, MemoryCommandSink; does not access the camera, GPU,
UART, ESP32 or physical servos.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import DetectionResult
from motion_gateway import MemoryCommandSink, PiMotionGateway
from person_target_adapter import detection_to_person_target
from safety_manager import Mode
from v883_person_dual_loop import V883PersonDualLoop


def target(cx=270):
    result = DetectionResult(
        found=True,
        center_x=cx,
        center_y=100,
        width=70,
        height=140,
        confidence=0.8,
        class_id=0,
        class_name="person",
        frame_width=320,
        frame_height=240,
    )
    out = detection_to_person_target(result)
    assert out is not None
    return out


class RemoteLoopIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.sink = MemoryCommandSink()
        self.gateway = PiMotionGateway(self.sink)
        self.controller = V883PersonDualLoop()
        self.addCleanup(self.gateway.close)

    def ready(self):
        for i in range(3):
            self.gateway.observe_detection(True, 160.0, now=10.0+i*.1)

    def candidate(self, cx=270):
        self.controller.step(
            target(cx), now=10.0, head_yaw_feedback_deg=0,
            head_feedback_fresh=True, body_idle=True,
        )
        return self.controller.step(
            target(cx), now=10.1, head_yaw_feedback_deg=0,
            head_feedback_fresh=True, body_idle=True,
        ).body_command

    def test_no_auto_motion_before_explicit_arm(self):
        self.ready()
        proposal = self.candidate()
        self.assertEqual(proposal, "TRACK_LEFT")
        denial = self.gateway.request_motion(proposal, source="AUTO", now=10.25)
        self.assertFalse(denial.approved)
        self.assertNotIn("TRACK_LEFT", self.sink.commands)

    def test_armed_yolo_plan_flows_through_single_gateway(self):
        self.ready()
        command = self.candidate()
        self.assertTrue(self.gateway.arm_auto(now=10.25))
        result = self.gateway.request_motion(command, source="AUTO", now=10.25)
        self.assertTrue(result.approved)
        self.assertEqual(self.sink.commands[-1], "TRACK_LEFT")

    def test_person_lost_stops_already_issued_motion(self):
        self.ready()
        command = self.candidate()
        self.gateway.arm_auto(now=10.25)
        self.gateway.request_motion(command, source="AUTO", now=10.25)
        plan = self.controller.target_lost()
        self.gateway.observe_detection(False, 160.0, now=10.3)
        self.assertIsNone(plan.body_command)
        self.assertEqual(self.sink.commands[-1], "STOP")
        self.assertEqual(self.gateway.status(now=10.31).safety.mode, Mode.SAFE)

    def test_gpu_stall_issues_stop_via_independent_tick(self):
        self.ready()
        command = self.candidate()
        self.gateway.arm_auto(now=10.25)
        self.gateway.request_motion(command, source="AUTO", now=10.25)
        self.gateway.check_watchdog(now=11.1)
        self.assertEqual(self.sink.commands[-1], "STOP")
        self.assertEqual(self.gateway.status(now=11.1).safety.mode, Mode.SAFE)

    def test_missing_yaw_feedback_never_proposes_actual_turn(self):
        self.ready()
        for i in range(3):
            plan = self.controller.step(
                target(270), now=10+i*.1,
                head_yaw_feedback_deg=None,
                head_feedback_fresh=False,
                body_idle=False,
            )
            self.assertIsNone(plan.body_command)
        self.assertNotIn("TRACK_LEFT", self.sink.commands)

    def test_translation_never_approved_from_full_body_bbox(self):
        self.ready()
        self.gateway.arm_auto(now=10.25)
        denial = self.gateway.request_motion("TRACK_FORWARD", source="AUTO", now=10.25)
        self.assertFalse(denial.approved)
        self.assertNotIn("TRACK_FORWARD", self.sink.commands)


if __name__ == "__main__":
    unittest.main()
