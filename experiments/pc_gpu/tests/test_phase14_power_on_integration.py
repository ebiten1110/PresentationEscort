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

from pi_real_head_integration import HeadOnlyPlanner, LockedRealRuntime, main
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

    def launch(self, *, head_enabled=True):
        r = LockedRealRuntime(
            "mock-ch340", head_enabled=head_enabled,
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
            target = detection_to_person_target(frame(1))
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
