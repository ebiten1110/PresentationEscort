"""Software-only firmware safety ACK / watchdog protocol tests."""
from __future__ import annotations
import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from esp32_safety_link import Esp32SafetyLink
from motion_gateway import MemoryCommandSink, PiMotionGateway
from safety_manager import Mode


class FirmwareSink(MemoryCommandSink):
    requires_firmware_safety = True
    # Native ESP32 tests separately verify actual watchdog behavior.


class FirmwareSafetyLinkTests(unittest.TestCase):
    def setUp(self):
        self.sink = FirmwareSink()
        self.fw = Esp32SafetyLink(self.sink)

    def test_no_initial_firmware_authorization(self):
        self.assertFalse(self.fw.fresh_armed(now=10.0))
        self.assertIsNone(self.fw.feed(b"[Safety] State: ARMED\n", now=10.0))
        self.assertTrue(self.fw.fresh_armed(now=10.0))
        self.assertIsNone(self.fw.requested_mode)
        # State announcements alone cannot complete an operator ARM request.

    def test_explicit_arm_needs_accepted_and_state(self):
        self.assertTrue(self.fw.request_arm("AUTO", now=10.0))
        self.assertEqual(self.sink.commands, ["SAFETY_ARM"])
        self.assertIsNone(self.fw.feed(b"[Safety] ARM_ACCEPTED\n", now=10.05))
        self.assertEqual(self.fw.feed(b"[Safety] State: ARMED\n", now=10.07), "AUTO")
        self.assertTrue(self.fw.fresh_armed(now=10.08))

    def test_stale_disarmed_before_arm_ack_does_not_cancel_request(self):
        self.assertTrue(self.fw.request_arm("AUTO", now=10.0))
        self.assertIsNone(self.fw.feed(b"[Safety] State: DISARMED\n", now=10.02))
        self.assertEqual(self.fw.requested_mode, "AUTO")
        self.assertIsNone(self.fw.feed(b"[Safety] ARM_ACCEPTED\n", now=10.05))
        self.assertEqual(self.fw.feed(b"[Safety] State: ARMED\n", now=10.06), "AUTO")

    def test_latched_status_during_pending_arm_still_aborts(self):
        self.assertTrue(self.fw.request_arm("MANUAL", now=10.0))
        self.fw.feed(b"[Safety] State: LATCHED\n", now=10.02)
        self.assertIsNone(self.fw.requested_mode)
        self.assertFalse(self.fw.fresh_armed(now=10.03))

    def test_reject_no_automatic_retry(self):
        self.fw.request_arm("AUTO", now=10.0)
        self.fw.feed(b"[Safety] ARM_REJECTED\n[Safety] State: LATCHED\n", now=10.1)
        self.assertFalse(self.fw.fresh_armed(now=10.1))
        self.assertFalse(self.fw.request_arm("AUTO", now=10.2))

    def test_late_ack_is_not_an_arm(self):
        self.fw.request_arm("AUTO", now=10.0)
        self.assertIsNone(self.fw.feed(b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=10.5))

    def test_heartbeat_respects_interval(self):
        self.fw.request_arm("AUTO", now=10.0)
        self.fw.feed(b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=10.05)
        self.assertTrue(self.fw.send_ping_if_due(now=10.08))
        self.assertFalse(self.fw.send_ping_if_due(now=10.10))
        self.assertTrue(self.fw.send_ping_if_due(now=10.25))
        self.assertEqual(self.sink.commands.count("SAFETY_PING"), 2)

    def test_expired_ack_disables_heartbeat(self):
        self.fw.request_arm("AUTO", now=10.0)
        self.fw.feed(b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=10.05)
        self.assertFalse(self.fw.send_ping_if_due(now=10.7))

    def test_fragmented_response(self):
        self.fw.request_arm("MANUAL", now=10.0)
        self.assertIsNone(self.fw.feed(b"[Safety] ARM_ACC", now=10.1))
        self.assertEqual(self.fw.feed(b"EPTED\n[Safety] State: ARMED\n", now=10.15), "MANUAL")

    def test_disarm_clears_previous_grant(self):
        self.fw.request_arm("AUTO", now=10.0)
        self.fw.feed(b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=10.05)
        self.fw.note_stop()
        self.assertFalse(self.fw.fresh_armed(now=10.1))

    def test_wrong_commands_cannot_be_requested(self):
        self.assertFalse(self.fw.request_arm("EMERGENCY", now=10.0))


class GatewayFirmwareHandshakeTests(unittest.TestCase):
    def setUp(self):
        self.sink = FirmwareSink()
        self.g = PiMotionGateway(self.sink)
        self.addCleanup(self.g.close)

    def test_auto_is_pending_until_esp32_ack(self):
        now = time.monotonic()
        for _ in range(3):
            self.g.observe_detection(True, 55.0, now=now)
        self.assertFalse(self.g.arm_auto(now=now))
        self.assertIn("SAFETY_ARM", self.sink.commands)
        self.assertEqual(self.g.status(now=now).safety.mode, Mode.SAFE)
        self.g.observe_firmware_feedback(
            b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=now+0.01
        )
        self.assertEqual(self.g.status(now=now+0.01).safety.mode, Mode.AUTO)

    def test_without_firmware_ack_motion_is_denied(self):
        now = time.monotonic()
        for _ in range(3):
            self.g.observe_detection(True, 45.0, now=now)
        self.g.arm_auto(now=now)
        result = self.g.request_motion("TRACK_LEFT", source="AUTO", now=now+0.01)
        self.assertFalse(result.approved)
        self.assertNotIn("TRACK_LEFT", self.sink.commands)

    def test_firmware_stale_status_emergency_stops(self):
        now = time.monotonic()
        for _ in range(3):
            self.g.observe_detection(True, 40.0, now=now)
        self.g.arm_auto(now=now)
        self.g.observe_firmware_feedback(
            b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=now+0.01
        )
        self.g.check_watchdog(now=now+0.57)
        self.assertEqual(self.g.status(now=now+0.58).safety.mode, Mode.ESTOP)
        self.assertIn("STOP", self.sink.commands)

    def test_manual_requires_fw_ack_but_not_gpu(self):
        now = time.monotonic()
        self.assertFalse(self.g.arm_manual(now=now))
        self.g.observe_firmware_feedback(
            b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=now+0.01
        )
        self.assertEqual(self.g.status(now=now+0.01).safety.mode, Mode.MANUAL)

    def test_firmware_latched_state_ends_auto(self):
        now = time.monotonic()
        for _ in range(3):
            self.g.observe_detection(True, 40.0, now=now)
        self.g.arm_auto(now=now)
        self.g.observe_firmware_feedback(
            b"[Safety] ARM_ACCEPTED\n[Safety] State: ARMED\n", now=now+0.01
        )
        self.g.observe_firmware_feedback(b"[Safety] State: LATCHED\n", now=now+0.1)
        self.assertEqual(self.g.status(now=now+0.11).safety.mode, Mode.ESTOP)

    def test_explicit_firmware_reset_requires_ack_and_rearm(self):
        now = time.monotonic()
        for _ in range(3):
            self.g.observe_detection(True, 40.0, now=now)
        self.g.arm_auto(now=now)
        self.g.observe_firmware_feedback(
            b"[Safety] ARM_ACCEPTED\\n[Safety] State: ARMED\\n".replace(b"\\n", b"\n"),
            now=now+0.01,
        )
        self.g.emergency_stop()
        self.assertEqual(self.g.status(now=now+0.02).safety.mode, Mode.ESTOP)
        self.assertTrue(self.g.request_firmware_reset())
        self.assertIn("SAFETY_RESET", self.sink.commands)
        self.g.reset_emergency_stop(now=now+0.02)
        self.assertEqual(self.g.status(now=now+0.02).safety.mode, Mode.ESTOP)
        # ESP32 confirms reset to DISARMED, but never immediately rearms.
        self.g.observe_firmware_feedback(b"[Safety] State: DISARMED\n", now=now+0.03)
        self.g.reset_emergency_stop(now=now+0.04)
        self.assertEqual(self.g.status(now=now+0.04).safety.mode, Mode.SAFE)
        self.assertFalse(self.g.request_motion("TRACK_LEFT", source="AUTO", now=now+0.04).approved)

    def test_firmware_reset_not_sent_when_not_estopped(self):
        self.assertFalse(self.g.request_firmware_reset())
        self.assertNotIn("SAFETY_RESET", self.sink.commands)

    def test_old_firmware_never_acknowledges(self):
        now = time.monotonic()
        for _ in range(3):
            self.g.observe_detection(True, 40.0, now=now)
        self.g.arm_auto(now=now)
        self.g.observe_firmware_feedback(b"Control: V8.8_DUAL_LOOP_CAMERA_BODY\n", now=now+0.05)
        self.g.check_watchdog(now=now+0.50)
        self.assertNotEqual(self.g.status(now=now+0.51).safety.mode, Mode.AUTO)


if __name__ == "__main__":
    unittest.main()
