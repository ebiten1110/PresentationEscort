"""Bounded target-loss recovery against an in-memory ESP32 only."""
from __future__ import annotations

import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pi_real_head_integration import LockedRealRuntime
from pi_showtime_smoke import frame
from virtual_esp32 import VirtualEsp32


def until(predicate, seconds=1.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError("expected state not observed")


class SearchTests(unittest.TestCase):
    def setUp(self):
        self.uart = VirtualEsp32(motion_duration_s=.18)
        serial_patch = patch("pi_real_head_integration.SerialCommandSink",
                             return_value=self.uart)
        serial_patch.start()
        self.addCleanup(serial_patch.stop)
        self.runtime = LockedRealRuntime(
            "mock-ch340", head_enabled=True, body_enabled=True,
            turn_stable_frames=100, power_declared="ON_OPERATOR_DECLARED")
        self.addCleanup(self.runtime.close)
        self.runtime.start()
        self.assertTrue(self.runtime.action("gesture_on")["ok"])
        for seq in range(1, 8):
            sample = frame(seq)
            sample.gesture_indices = [0]
            self.runtime.process_detection(sample, 65.)
            time.sleep(.025)
        until(lambda: self.runtime.status()["mode"] == "AUTO")
        for seq in range(8, 12):
            self.runtime.process_detection(frame(seq), 65.)
            time.sleep(.03)
        until(lambda: self.runtime.controller.telemetry.snapshot().body_idle_confirmed)
        self.assertGreaterEqual(abs(self.runtime._last_cycle.plan.yaw_target_deg), 2.5)

    def begin(self):
        self.runtime.process_detection(frame(12, visible=False), 65.)
        return self.runtime.status()

    def test_one_short_search_pulse_and_white_led(self):
        status = self.begin()
        self.assertEqual(status["search_state"], "SEARCHING")
        self.assertEqual(status["mode"], "AUTO")
        self.assertEqual(status["led_indicated_color"], "WHITE")
        self.assertEqual(len([c for c in self.uart.commands
                              if c in ("TRACK_LEFT", "TRACK_RIGHT")]), 1)
        self.runtime.process_detection(frame(13, visible=False), 65.)
        self.assertEqual(len([c for c in self.uart.commands
                              if c in ("TRACK_LEFT", "TRACK_RIGHT")]), 1)

    def test_three_reacquire_frames_without_second_arm(self):
        self.begin()
        self.runtime.process_detection(frame(13), 65.)
        self.assertEqual(self.runtime.status()["search_state"], "SEARCHING")
        self.runtime.process_detection(frame(14), 65.)
        self.assertEqual(self.runtime.status()["search_reacquire_frames"], 2)
        self.runtime.process_detection(frame(15), 65.)
        self.assertEqual(self.runtime.status()["search_state"], "REACQUIRED_3_FRAMES")
        self.assertEqual(self.runtime.status()["mode"], "AUTO")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)

    def test_two_frames_then_loss_restart_confirmation(self):
        self.begin()
        self.runtime.process_detection(frame(13), 65.)
        self.runtime.process_detection(frame(14), 65.)
        self.runtime.process_detection(frame(15, visible=False), 65.)
        self.assertEqual(self.runtime.status()["search_reacquire_frames"], 0)
        self.assertEqual(len([c for c in self.uart.commands
                              if c in ("TRACK_LEFT", "TRACK_RIGHT")]), 1)

    def test_translation_never_used(self):
        self.begin()
        denied = self.runtime.gateway.request_recovery_turn("TRACK_FORWARD")
        self.assertFalse(denied.approved)
        self.assertFalse(any(x in self.uart.commands for x in
                             ("TRACK_FORWARD", "TRACK_BACKWARD")))

    def test_second_turn_denied(self):
        self.begin()
        denied = self.runtime.gateway.request_recovery_turn("TRACK_LEFT")
        self.assertFalse(denied.approved)

    def test_stop_immediately_disarms(self):
        self.begin()
        self.runtime.action("stop")
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.runtime.status()["search_state"], "CANCELED")
        self.assertIn("STOP", self.uart.commands)

    def test_estop_latches(self):
        self.begin()
        self.runtime.action("estop")
        self.assertEqual(self.runtime.status()["mode"], "ESTOP")
        self.assertFalse(self.runtime.action("auto")["ok"])

    def test_disconnect_cancels_search(self):
        self.begin()
        self.runtime.set_camera_state(False, False)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.runtime.status()["search_state"], "LINK_LOST")

    def test_timeout_stops_without_camera_packets(self):
        self.begin()
        until(lambda: self.runtime.status()["mode"] == "SAFE", seconds=2.0)
        self.assertIn(self.runtime.status()["reason"], ("LINK_TIMEOUT", "SEARCH_TIMEOUT"))
        self.assertFalse(self.runtime.gateway.safety.recovering)
        self.assertIn("STOP", self.uart.commands)

    def test_other_person_visible_is_not_blind_search(self):
        other = frame(12)
        other.people = [{"x1": 8, "y1": 35, "x2": 65,
                         "y2": 195, "confidence": .91}]
        self.runtime.process_detection(other, 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_stale_result_is_not_search(self):
        self.runtime.process_detection(frame(12, visible=False), 700.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_sequence_gap_is_not_search(self):
        self.runtime.process_detection(frame(14, visible=False), 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_legacy_protocol_is_not_search(self):
        bad = frame(12, visible=False)
        bad.protocol_version = 1
        self.runtime.process_detection(bad, 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_motion_busy_blocks_search(self):
        self.runtime.controller.telemetry.feed(
            b"Walking State: TRACK_RIGHT\nBusy: YES\n",
            now=time.monotonic())
        self.runtime.process_detection(frame(12, visible=False), 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_stale_head_feedback_blocks_search(self):
        self.runtime.controller.telemetry.reset()
        self.runtime.process_detection(frame(12, visible=False), 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_missing_direction_blocks_search(self):
        old = self.runtime._last_cycle
        self.runtime._last_cycle = replace(
            old, plan=replace(old.plan, suggested_body_turn=None))
        self.runtime.process_detection(frame(12, visible=False), 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_waiting_disabled_blocks_search(self):
        self.runtime.gesture_control_enabled = False
        self.runtime.process_detection(frame(12, visible=False), 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_expired_person_frame_blocks_search(self):
        time.sleep(.51)
        self.runtime.process_detection(frame(12, visible=False), 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotEqual(self.runtime.status()["search_state"], "SEARCHING")

    def test_no_auto_arm_from_new_person_after_loss(self):
        self.begin()
        until(lambda: self.runtime.status()["mode"] == "SAFE", seconds=2.0)
        for seq in range(13, 17):
            self.runtime.process_detection(frame(seq), 65.)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.uart.commands.count("SAFETY_ARM"), 1)


if __name__ == "__main__":
    unittest.main()
