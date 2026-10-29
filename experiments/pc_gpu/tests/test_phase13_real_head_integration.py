"""Phase 13 guardrail tests. All UART interactions are mocked; no motors."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from person_target_adapter import PersonTarget, detection_to_person_target
from pi_real_head_integration import (HeadBodyTurnPlanner, HeadOnlyPlanner,
                                      HeadOnlySerialSink, LockedRealRuntime)
from safety_manager import Mode, SafetyManager
from presenter_lock import PresenterLock


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
    def test_short_auto_recovery_keeps_same_arm_without_forging_new_gesture(self):
        presenter = PresenterLock(gesture_enabled=True,
            gesture_confirm_frames=2, transient_recovery_s=1.5)
        person = {"x1": 65, "y1": 15, "x2": 155, "y2": 225,
                  "confidence": .95}
        def observed(seq, boxes, gesture=False):
            return presenter.observe(DetectionResult(
                found=bool(boxes), protocol_version=2, frame_seq=seq,
                frame_width=320, frame_height=240, people=boxes,
                gesture_indices=[0] if gesture else []), now=1.+seq*.1)
        observed(1, [person], True)
        observed(2, [person], True)
        observed(3, [person])
        self.assertEqual(presenter.state, "TRACKING")
        original_event = presenter.gesture_event_id
        safety = SafetyManager()
        for now in (1.1, 1.2, 1.3):
            safety.observe_detection(True, 60., now=now)
        self.assertTrue(safety.arm_auto(now=1.3))
        self.assertTrue(safety.begin_recovery(now=1.4))
        self.assertFalse(observed(4, []).found)
        safety.observe_detection(False, 60., now=1.4)
        for i in (5, 6, 7):
            self.assertTrue(observed(i, [person]).found)
            safety.observe_detection(True, 60., now=1.+i*.1)
        self.assertFalse(safety.recovering)
        self.assertEqual(safety.mode, Mode.AUTO)
        presenter.confirm_bounded_reacquisition()
        self.assertEqual(presenter.gesture_event_id, original_event)
        self.assertEqual(presenter.reason, "AUTO_RECALIBRATED_SAME_TRACK")
        safety.disarm("OPERATOR_DISARM")
        for now in (1.8, 1.9, 2.0):
            safety.observe_detection(True, 60., now=now)
        self.assertEqual(safety.mode, Mode.SAFE)

    def test_bounded_automatic_recalibration_requires_existing_auto_and_unique_target(self):
        runtime = LockedRealRuntime.__new__(LockedRealRuntime)
        runtime.auto_recalibrate_brief_loss = True
        runtime.gesture_control_enabled = True
        state = SimpleNamespace(mode=Mode.AUTO)
        runtime.gateway = SimpleNamespace(
            status=lambda: SimpleNamespace(safety=state),
            firmware_safety_status=lambda: "ARMED")
        runtime.presenter = SimpleNamespace(
            state="TRACKING", _locked=object(), candidates=[object()],
            _frame_time=4.9, _last_seq=12, _frame_size=(320, 240))
        runtime._last_cycle = SimpleNamespace(target_valid=True)
        runtime.controller = SimpleNamespace(
            _pending_turn=None, planner=SimpleNamespace(motion_pending=False),
            telemetry=SimpleNamespace(snapshot=lambda now: SimpleNamespace(
                head_fresh=True, body_idle_confirmed=True)))
        lost = DetectionResult(found=False, frame_width=320, frame_height=240,
                               protocol_version=2, frame_seq=13, people=[])
        self.assertTrue(runtime._can_hold_brief_visual_loss(lost, 65., 5.))
        state.mode = Mode.SAFE
        self.assertFalse(runtime._can_hold_brief_visual_loss(lost, 65., 5.))
        state.mode = Mode.AUTO
        runtime.presenter.candidates.append(object())
        self.assertFalse(runtime._can_hold_brief_visual_loss(lost, 65., 5.))
        runtime.presenter.candidates.pop()
        runtime.controller._pending_turn = "TRACK_LEFT"
        # Passive recovery holds AUTO but issues NO new motion commands.
        # A body turn already finishing must not destroy the face lock.
        self.assertTrue(runtime._can_hold_brief_visual_loss(lost, 65., 5.))

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

    @staticmethod
    def _vertical_target(error_y=130, error_x=0):
        return PersonTarget(
            center_x=320 + error_x, aim_y=240 + error_y,
            error_x=error_x, error_y=error_y,
            box_width=220, box_height=475,
            frame_width=640, frame_height=480,
            confidence=.91, aim_from_pose=True)

    def test_calibrated_pitch_actuates_even_for_cropped_person(self):
        planner = HeadOnlyPlanner(
            pitch_min=-2., pitch_max=2., head_speed_deg_s=.5,
            pitch_speed_deg_s=2.)
        plan = planner.step(self._vertical_target(), now=10.,
                            head_yaw_feedback_deg=0.,
                            head_pitch_feedback_deg=0.,
                            head_feedback_fresh=True, body_idle=True)
        self.assertTrue(plan.pitch_tracking_enabled)
        self.assertGreater(plan.pitch_target_deg, 0.)
        self.assertLessEqual(plan.pitch_target_deg, .3)
        self.assertIsNotNone(plan.head_pose_command)
        self.assertIsNone(plan.body_command)

    def test_pitch_speed_independent_of_yaw_and_clamped(self):
        planner = HeadOnlyPlanner(
            pitch_min=-2., pitch_max=2., head_speed_deg_s=.5,
            pitch_speed_deg_s=2.)
        plan = planner.step(self._vertical_target(error_x=200), now=2.,
                            head_yaw_feedback_deg=0., head_pitch_feedback_deg=0.,
                            head_feedback_fresh=True, body_idle=True)
        self.assertLessEqual(abs(plan.yaw_target_deg), .1)
        self.assertGreaterEqual(plan.pitch_target_deg, .2)
        for i in range(1, 22):
            plan = planner.step(self._vertical_target(), now=2.+i*.15,
                                head_yaw_feedback_deg=0.,
                                head_pitch_feedback_deg=plan.pitch_target_deg,
                                head_feedback_fresh=True, body_idle=True)
            self.assertLessEqual(plan.pitch_target_deg, 2.)
        self.assertEqual(plan.pitch_target_deg, 2.)

    def test_reverse_vertical_changes_only_pitch_direction(self):
        normal = HeadOnlyPlanner(
            pitch_min=-2., pitch_max=2., invert_vertical=False,
            pitch_speed_deg_s=2.)
        reverse = HeadOnlyPlanner(
            pitch_min=-2., pitch_max=2., invert_vertical=True,
            pitch_speed_deg_s=2.)
        target = self._vertical_target(error_x=50)
        kwargs = {"now": 7., "head_yaw_feedback_deg": 0.,
                  "head_pitch_feedback_deg": 0.,
                  "head_feedback_fresh": True, "body_idle": True}
        p1 = normal.step(target, **kwargs)
        p2 = reverse.step(target, **kwargs)
        self.assertEqual(p1.yaw_target_deg, p2.yaw_target_deg)
        self.assertEqual(p1.pitch_target_deg, -p2.pitch_target_deg)

    def test_real_runtime_exposes_pitch_configuration(self):
        runtime = LockedRealRuntime(
            "fake", head_enabled=True, pitch_bounds=(-2., 2.),
            head_speed_deg_s=.5, pitch_speed_deg_s=1.5,
            invert_vertical=True)
        self.addCleanup(runtime.close)
        state = runtime.status()
        self.assertEqual(state["pitch_speed_limit_deg_s"], 1.5)
        self.assertEqual(state["vertical_direction"], "REVERSED")
        self.assertEqual(state["head_pitch_approved_range"], [-2., 2.])
        self.assertIn("Pitch -2～2°", state["motor_status"])
        self.assertIn("PITCH_CALIBRATED", state["actuation_scope"])

    def test_reject_invalid_pitch_speed(self):
        with self.assertRaises(ValueError):
            HeadOnlyPlanner(pitch_speed_deg_s=0.2)

    def test_extreme_visible_offset_can_request_early_body_turn(self):
        planner = HeadBodyTurnPlanner(
            pitch_min=-2., pitch_max=2., stable_frames=2,
            head_speed_deg_s=2., pitch_speed_deg_s=1.5)
        target = self._vertical_target(error_y=0, error_x=230)
        measured_yaw = 0.
        proposals = []
        for i in range(24):
            plan = planner.step(
                target, now=1.+i*.12, head_yaw_feedback_deg=measured_yaw,
                head_pitch_feedback_deg=0.,
                head_feedback_fresh=True, body_idle=True)
            measured_yaw = plan.yaw_target_deg
            if plan.body_command:
                proposals.append((plan.body_command, measured_yaw))
                break
        self.assertTrue(proposals, "extreme side movement should allow a bounded turn")
        self.assertLess(abs(proposals[0][1]), 2.5)

    def test_esp32_pitch_pwm_diagnostic_is_kept_per_channel(self):
        self.fake.read_available = lambda **kwargs: (
            b"[ServoDiag] PWM ch=3 angle=-1.5 duty=301 i2c=ACK\n"
            b"[ServoDiag] PWM ch=2 angle=0.8 duty=305 i2c=ACK\n")
        sink = HeadOnlySerialSink("fake", head_enabled=True,
                                  pitch_min=-2., pitch_max=2.)
        sink.read_available()
        self.assertIn("ch=3", sink.last_head_pwm[3])
        self.assertIn("ch=2", sink.last_head_pwm[2])
        self.assertIn("ch=2", sink.last_servo_diagnostic)
        sink.close()

    def test_camera_stage_timing_is_available_in_status(self):
        runtime = LockedRealRuntime("fake", head_enabled=False)
        self.addCleanup(runtime.close)
        runtime.set_capture_diagnostics({"requested_mjpeg": True,
                                         "fourcc": "MJPG", "reported_fps": 30.})
        runtime.set_capture_timing({"capture_ms": 40., "encode_ms": 6.,
                                    "remote_infer_roundtrip_ms": 50.})
        state = runtime.status()
        self.assertEqual(state["camera_capture_mode"]["fourcc"], "MJPG")
        self.assertEqual(state["camera_stage_timing_ms"]["capture_ms"], 40.)


if __name__ == "__main__":
    unittest.main()
