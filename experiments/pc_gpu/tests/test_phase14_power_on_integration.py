"""Power-ON real-transport regression with a virtual replacement for the UART.

No real serial ports are opened and no physical motors are driven.
"""
from __future__ import annotations

import contextlib
import io
import re
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pi_real_head_integration import HeadOnlyPlanner, HeadBodyTurnPlanner, HeadOnlySerialSink, LockedRealRuntime, main
from detection_protocol import DetectionResult
from pi_showtime_smoke import frame
from person_target_adapter import detection_to_person_target
from virtual_esp32 import VirtualEsp32

_HEAD_CMD = re.compile(r"HEAD_POSE_SET:([-+]?\d+(?:\.\d+)?),([-+]?\d+(?:\.\d+)?)")


def until(predicate, *, max_wait_s=1.2):
    deadline = time.monotonic() + max_wait_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError("expected condition not reached")


class RealPowerOnTests(unittest.TestCase):
    def setUp(self):
        self.uart = VirtualEsp32()
        self.patch_serial = patch("pi_real_head_integration.SerialCommandSink", return_value=self.uart)
        self.patch_serial.start()
        self.addCleanup(self.patch_serial.stop)

    def launch(self, *, head_enabled=True, body_enabled=False):
        r = LockedRealRuntime(
            "mock-ch340", head_enabled=head_enabled,
            body_enabled=body_enabled,
            power_declared="ON_OPERATOR_DECLARED",
        )
        self.addCleanup(r.close)
        r.start()
        return r

    def select(self, r):
        r.process_detection(frame(1), 65.)
        self.assertTrue(r.target_action({"index": 0, "frame_seq": 1})["ok"])
        for i in range(2, 6):
            r.process_detection(frame(i), 65.)
        self.assertEqual(r.status()["lock_state"], "TRACKING")
        until(lambda: r.status()["fw_safety"] == "DISARMED")

    def test_gesture_selects_person_but_never_auto_arms_by_default(self):
        r = self.launch(body_enabled=True)
        for seq in range(1, 7):
            item = frame(seq)
            item.gesture_indices = [0]
            r.process_detection(item, 65.)
        status = r.status()
        self.assertEqual(status["lock_state"], "TRACKING")
        self.assertTrue(status["presenter_selected"])
        self.assertTrue(status["auto_ready"])
        self.assertEqual(status["mode"], "SAFE")
        self.assertNotIn("SAFETY_ARM", self.uart.commands)

    def test_transient_loss_stops_but_reuses_same_presenter_without_auto_arm(self):
        r = self.launch(body_enabled=True)
        for seq in range(1, 7):
            item = frame(seq)
            item.gesture_indices = [0]
            r.process_detection(item, 65.)
        self.assertTrue(r.action("auto")["ok"])
        until(lambda: r.status()["mode"] == "AUTO")
        r.process_detection(frame(7, visible=False), 65.)
        self.assertEqual(r.status()["lock_state"], "TEMPORARILY_LOST")
        self.assertFalse(r.status()["presenter_selected"])
        self.assertEqual(r.status()["mode"], "SAFE")
        r.process_detection(frame(8), 65.)
        self.assertEqual(r.status()["lock_state"], "TRACKING")
        self.assertEqual(r.status()["mode"], "SAFE")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)

    def test_calibrated_pitch_range_gate(self):
        sink = HeadOnlySerialSink("mock-ch340", head_enabled=True, body_enabled=True,
                                  pitch_min=-4., pitch_max=5.)
        self.addCleanup(sink.close)
        sink.send("HEAD_POSE_SET:2.0,4.0")
        self.assertIn("HEAD_POSE_SET:2.0,4.0", self.uart.commands)
        with self.assertRaises(ValueError):
            sink.send("HEAD_POSE_SET:2.0,8.0")

    def test_power_on_does_not_arm_without_operator(self):
        r = self.launch()
        self.assertEqual(r.status()["servo_power_declared"], "ON_OPERATOR_DECLARED")
        self.assertEqual(r.status()["actuation_scope"], "HEAD_YAW_ONLY_3DEG")
        self.assertEqual(r.status()["mode"], "SAFE")
        self.assertNotIn("SAFETY_ARM", self.uart.commands)
        self.assertFalse(any(cmd.startswith("HEAD_POSE_SET:") for cmd in self.uart.commands))
        self.assertEqual(r.status()["simulation"], False)

    def test_power_on_monitor_never_arms_or_moves(self):
        r = self.launch(head_enabled=False)
        self.select(r)
        self.assertFalse(r.action("auto")["ok"])
        self.assertFalse(r.action("manual")["ok"])
        self.assertEqual(r.status()["actuation_scope"], "MONITOR_ONLY")
        self.assertNotIn("SAFETY_ARM", self.uart.commands)
        self.assertFalse(any(cmd.startswith("HEAD_POSE_SET:") for cmd in self.uart.commands))

    def test_selected_presenter_only_yaw_stop_and_no_rearm(self):
        r = self.launch()
        self.select(r)
        self.assertTrue(r.action("auto")["ok"])
        until(lambda: r.status()["mode"] == "AUTO")
        for i in range(6, 20):
            r.process_detection(frame(i), 65.)
            time.sleep(.03)
        until(lambda: any(c.startswith("HEAD_POSE_SET:") for c in self.uart.commands))
        sent = list(self.uart.commands)
        poses = [c for c in sent if c.startswith("HEAD_POSE_SET:")]
        self.assertTrue(poses)
        self.assertTrue(all((_HEAD_CMD.fullmatch(c) is not None
                             and abs(float(_HEAD_CMD.fullmatch(c)[1])) <= 3.0
                             and float(_HEAD_CMD.fullmatch(c)[2]) == 0.0)
                            for c in poses), poses)
        self.assertFalse(any(c.startswith("TRACK_") for c in sent))
        self.assertFalse(r.action("manual")["ok"])
        r.process_detection(frame(20, visible=False), 65.)
        until(lambda: r.status()["mode"] == "SAFE")
        self.assertIn("STOP", self.uart.commands)
        self.assertFalse(r.action("auto")["ok"])

    def test_repeated_body_turns_no_count_cap(self):
        r = self.launch(body_enabled=True)
        self.assertEqual(r.status()["actuation_scope"], "HEAD_YAW_3DEG_AND_FEEDBACK_GATED_BODY_TURNS")
        self.assertNotIn("SAFETY_ARM", self.uart.commands)
        self.select(r)
        self.assertTrue(r.action("auto")["ok"])
        until(lambda: r.status()["mode"] == "AUTO")
        # Slower neck setpoints and body settling deliberately lengthen each
        # turn cycle; the important invariant is no session turn-count cap.
        for seq in range(6, 180):
            r.process_detection(frame(seq), 65.)
            time.sleep(.04)
            if len([c for c in self.uart.commands if c in ("TRACK_LEFT", "TRACK_RIGHT")]) >= 3:
                break
        turns = [c for c in self.uart.commands if c in ("TRACK_LEFT", "TRACK_RIGHT")]
        self.assertGreaterEqual(len(turns), 3)
        self.assertNotIn("TRACK_FORWARD", self.uart.commands)
        self.assertNotIn("TRACK_BACKWARD", self.uart.commands)

    def test_presenter_loss_is_not_same_as_uart_fault(self):
        r = self.launch(body_enabled=True)
        self.select(r)
        self.assertTrue(r.action("auto")["ok"])
        until(lambda: r.status()["mode"] == "AUTO")
        # Valid camera detection can continue while the operator-selected
        # presenter is no longer one of the visible candidates.
        from pi_showtime_smoke import frame as make_frame
        bad = make_frame(6, visible=False)
        r.process_detection(bad, 65.)
        status = r.status()
        self.assertEqual(status["mode"], "SAFE")
        self.assertEqual(status["lock_reason"], "WAIT_SAME_PERSON_REAPPEAR")
        self.assertFalse(status["target_found"])
        self.assertFalse(status["auto_ready"])
        self.assertFalse(status["fw_arm_pending"])
        self.assertIsNone(status["fault_cause"])
        self.assertIn("STOP", self.uart.commands)

    def test_serial_poll_failure_cause_survives_presenter_event(self):
        r = self.launch(body_enabled=True)
        self.select(r)
        self.uart.link_broken = True
        until(lambda: r.status()["mode"] == "ESTOP")
        first = r.status()
        self.assertEqual(first["uart_poll_error"], "OSError")
        self.assertEqual(first["fault_cause"], "UART_POLL_OSError")
        r.process_detection(frame(6, visible=False), 65.)
        latest = r.status()
        self.assertEqual(latest["uart_poll_error"], "OSError")
        self.assertEqual(latest["fault_cause"], "UART_POLL_OSError")
        self.assertNotIn("SAFETY_ARM", self.uart.commands)

    def test_firmware_handshake_success_is_exposed(self):
        r = self.launch()
        self.select(r)
        result = r.action("auto")
        self.assertTrue(result["ok"])
        until(lambda: r.status()["mode"] == "AUTO")
        state = r.status()
        self.assertFalse(state["fw_arm_pending"])
        self.assertIsNone(state["arm_error"])
        self.assertEqual(state["fw_safety"], "ARMED")

    def test_uart_reports_real_safety_fault_line(self):
        r = self.launch()
        # The real-UART wrapper must parse \n-separated actual firmware
        # responses, including on partial serial reads.
        from unittest.mock import patch as mock_patch
        with mock_patch.object(r.sink._inner, "read_available", side_effect=[
            b"[Safety] FAULT=HEARTBEAT_",
            b"TIMEOUT\n[Safety] State: LATCHED\n",
        ]):
            r.sink.read_available()
            r.sink.read_available()
        self.assertEqual(r.sink.last_safety_event,
                         "[Safety] FAULT=HEARTBEAT_TIMEOUT")

    def test_second_auto_while_running_is_idempotent(self):
        r = self.launch(body_enabled=True)
        self.select(r)
        self.assertTrue(r.action("auto")["ok"])
        until(lambda: r.status()["mode"] == "AUTO")
        before = self.uart.commands.count("SAFETY_ARM")
        self.assertEqual(before, 1)
        response = r.action("auto")
        self.assertTrue(response["ok"])
        self.assertTrue(response["already_active"])
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), before)
        self.assertEqual(r.status()["mode"], "AUTO")

    def test_second_auto_during_pending_uses_one_arm_request(self):
        r = self.launch()
        self.select(r)
        first = r.action("auto")
        self.assertTrue(first["ok"])
        self.assertTrue(first["pending"])
        second = r.action("auto")
        self.assertTrue(second["ok"])
        self.assertTrue(second["pending"])
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)
        until(lambda: r.status()["mode"] == "AUTO")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)

    def test_auto_not_armed_without_firmware_disarmed_status(self):
        r = self.launch()
        self.select(r)
        # A non-DISARMED firmware status is never another ARM opportunity.
        r.gateway._firmware_safety.state = "UNKNOWN"
        self.assertFalse(r.status()["auto_ready"])
        response = r.action("auto")
        self.assertFalse(response["ok"])
        self.assertEqual(response["error"], "ESP32_NOT_DISARMED_OR_NOT_READY")
        self.assertNotIn("SAFETY_ARM", self.uart.commands)

    def test_operator_status_exposes_esp32_internal_feedback_not_physical_sensor(self):
        r = self.launch()
        r.controller.telemetry.feed(
            b"[HeadState] currentYaw=-3.0 targetYaw=-3.0 currentPitch=0.0 targetPitch=0.0 moving=NO\n"
            b"Walking State: IDLE\nBusy: NO\n",
        )
        state = r.status()
        self.assertTrue(state["esp32_head_feedback"]["fresh"])
        self.assertEqual(state["esp32_head_feedback"]["yaw_deg"], -3.0)
        self.assertEqual(state["esp32_walking_feedback"]["state"], "IDLE")
        self.assertTrue(state["esp32_walking_feedback"]["idle_confirmed"])

    def test_uart_pwm_diagnostic_parsed_independently_of_safety(self):
        sink = HeadOnlySerialSink("mock-ch340", head_enabled=True)
        self.addCleanup(sink.close)
        from unittest.mock import patch as mock_patch
        with mock_patch.object(sink._inner, "read_available", side_effect=[
            b"[ServoDiag] PCA9685_I2C=ACK address=0x40\n",
            b"[ServoDiag] PWM ch=2 angle=-3.0 duty=309 i2c=NO_ACK\n",
        ]):
            sink.read_available()
            sink.read_available()
        self.assertEqual(
            sink.last_servo_diagnostic,
            "[ServoDiag] PWM ch=2 angle=-3.0 duty=309 i2c=NO_ACK"
        )
        self.assertIsNone(sink.last_safety_event)

    def test_serial_disconnect_forces_estop(self):
        r = self.launch()
        self.uart.link_broken = True
        until(lambda: r.status()["mode"] == "ESTOP")
        self.assertTrue(r.status()["transport_fault"])
        self.assertFalse(r.action("auto")["ok"])


class PlannerTests(unittest.TestCase):
    def test_internal_state_and_commands_never_exceed_yaw_bound(self):
        p = HeadOnlyPlanner()
        for i in range(80):
            target = detection_to_person_target(DetectionResult(
                found=True, center_x=243, center_y=115, width=70,
                height=180, confidence=.88, class_id=0,
                class_name="person", frame_width=320, frame_height=240
            ))
            self.assertIsNotNone(target)
            result = p.step(target, now=1.0 + i * .07,
                            head_yaw_feedback_deg=0,
                            head_feedback_fresh=True, body_idle=True)
            self.assertLessEqual(abs(p.yaw_target_deg), 3.0)
            self.assertEqual(p.pitch_target_deg, 0.0)
            self.assertEqual(result.pitch_target_deg, 0.0)
            self.assertIsNone(result.body_command)
            self.assertFalse(p.motion_pending)
            if result.head_pose_command:
                match = _HEAD_CMD.fullmatch(result.head_pose_command)
                self.assertIsNotNone(match)
                self.assertLessEqual(abs(float(match[1])), 3.0)
                self.assertEqual(float(match[2]), 0.0)

    def test_body_planner_requires_feedback_not_quota(self):
        p = HeadBodyTurnPlanner()
        target = detection_to_person_target(DetectionResult(
            found=True, center_x=270, center_y=110, width=70,
            height=160, confidence=.88, class_id=0, class_name="person",
            frame_width=320, frame_height=240))
        self.assertIsNotNone(target)
        first = p.step(target, now=10.0, head_yaw_feedback_deg=None,
                       head_feedback_fresh=False, body_idle=False)
        self.assertIsNone(first.body_command)
        commands = []
        # MCU feedback follows the preceding commanded neck angle. A body
        # turn must wait for the head to reach its boundary at the new speed.
        measured_yaw = first.yaw_target_deg
        for i in range(1, 22):
            decision = p.step(target, now=10.0 + i * .10,
                              head_yaw_feedback_deg=measured_yaw,
                              head_feedback_fresh=True, body_idle=True)
            measured_yaw = decision.yaw_target_deg
            if decision.body_command:
                commands.append(decision.body_command)
                break
        self.assertTrue(any(c in ("TRACK_LEFT", "TRACK_RIGHT") for c in commands))
        self.assertLessEqual(abs(p.yaw_target_deg), 3.0)
        self.assertEqual(p.pitch_target_deg, 0.0)


class CliTests(unittest.TestCase):
    def test_no_power_declaration_denied_before_uart_open(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                main(["--serial-port", "/dev/serial/by-id/test"])
        self.assertEqual(raised.exception.code, 2)

    def test_head_mode_denied_without_calibration_flags(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                main([
                    "--serial-port", "/dev/serial/by-id/test",
                    "--servo-power-on", "--enable-head-test"
                ])
        self.assertEqual(raised.exception.code, 2)

    def test_body_turns_require_explicit_calibration(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                main([
                    "--serial-port", "/dev/serial/by-id/test",
                    "--servo-power-on", "--enable-head-test", "--enable-body-turns",
                    "--bench-secured", "--neutral-calibrated", "--physical-cutoff-tested",
                ])
        self.assertEqual(raised.exception.code, 2)

    def test_head_mode_denied_if_power_off_declared(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                main([
                    "--serial-port", "/dev/serial/by-id/test",
                    "--servo-power-off-confirmed", "--enable-head-test",
                    "--bench-secured", "--neutral-calibrated",
                    "--physical-cutoff-tested",
                ])
        self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
