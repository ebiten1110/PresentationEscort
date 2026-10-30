"""Software-only ESP32 telemetry regressions, no UART or robot needed."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from esp32_telemetry import Esp32Telemetry


HEAD = b"[HeadState] currentYaw=-8.5 targetYaw=-9.0 currentPitch=3.1 targetPitch=4.0 moving=YES\n"


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.t = Esp32Telemetry()

    def test_head_parses_exact_v883_format(self):
        self.t.feed(HEAD, now=10.0)
        s = self.t.snapshot(now=10.1)
        self.assertTrue(s.head_fresh)
        self.assertEqual(s.head_yaw_deg, -8.5)
        self.assertEqual(s.head_pitch_deg, 3.1)

    def test_split_uart_packet(self):
        self.t.feed(HEAD[:21], now=10.0)
        self.assertFalse(self.t.snapshot(now=10.0).head_fresh)
        self.t.feed(HEAD[21:], now=10.1)
        self.assertTrue(self.t.snapshot(now=10.2).head_fresh)

    def test_status_idle_requires_busy_no(self):
        self.t.feed(b"Walking State: IDLE\n", now=10)
        self.assertFalse(self.t.snapshot(now=10).body_idle_confirmed)
        self.t.feed(b"Busy: NO\n", now=10.01)
        self.assertTrue(self.t.snapshot(now=10.02).body_idle_confirmed)

    def test_idle_expires(self):
        self.t.feed(b"Walking State: IDLE\nBusy: NO\n", now=10)
        self.assertFalse(self.t.snapshot(now=10.5).body_idle_confirmed)

    def test_head_expires(self):
        self.t.feed(HEAD, now=10)
        self.assertIsNone(self.t.snapshot(now=10.6).head_yaw_deg)

    def test_walking_update_invalidates_previous_busy(self):
        self.t.feed(b"Walking State: IDLE\nBusy: NO\n", now=10)
        self.t.feed(b"[Walking] State: TRACK_LEFT\n", now=10.05)
        self.assertFalse(self.t.snapshot(now=10.06).body_idle_confirmed)

    def test_busy_yes_not_idle(self):
        self.t.feed(b"Walking State: IDLE\nBusy: YES\n", now=10)
        self.assertFalse(self.t.snapshot(now=10.01).body_idle_confirmed)

    def test_unknown_state_not_idle(self):
        self.t.feed(b"Walking State: UNKN0WN\nBusy: NO\n", now=10)
        self.assertFalse(self.t.snapshot(now=10.01).body_idle_confirmed)

    def test_turn_completion_requires_idle_and_busy_no(self):
        self.t.feed(b"[Walking] State: TRACK_LEFT\n", now=10)
        self.t.feed(b"[Walking] State: IDLE\n", now=10.1)
        self.assertFalse(self.t.snapshot(now=10.1).motion_completion)
        self.t.feed(b"Busy: NO\n", now=10.15)
        self.assertEqual(self.t.consume_completed_turn(now=10.2), "TRACK_LEFT")
        self.assertIsNone(self.t.consume_completed_turn(now=10.21))

    def test_right_completion(self):
        self.t.feed(b"[Walking] State: TRACK_RIGHT\n[Walking] State: IDLE\nBusy: NO\n", now=11)
        self.assertEqual(self.t.consume_completed_turn(now=11), "TRACK_RIGHT")

    def test_no_completion_without_active_turn(self):
        self.t.feed(b"[Walking] State: IDLE\nBusy: NO\n", now=10)
        self.assertIsNone(self.t.consume_completed_turn(now=10.01))

    def test_malformed_head_ignored(self):
        self.t.feed(b"[HeadState] currentYaw=NaN targetYaw=0 currentPitch=0 targetPitch=0 moving=YES\n", now=10)
        self.assertFalse(self.t.snapshot(now=10).head_fresh)

    def test_out_of_range_yaw_ignored(self):
        self.t.feed(b"[HeadState] currentYaw=88 targetYaw=0 currentPitch=0 targetPitch=0 moving=NO\n", now=10)
        self.assertFalse(self.t.snapshot(now=10).head_fresh)

    def test_timestamp_future_is_stale(self):
        self.t.feed(HEAD, now=10)
        self.assertFalse(self.t.snapshot(now=9).head_fresh)

    def test_serial_noise_does_not_arm_telemetry(self):
        self.t.feed(b"ZZ[HeadState] currentYaw=0 targetYaw=0 currentPitch=0 targetPitch=0 moving=NO\nGARBAGE\n", now=10)
        self.assertFalse(self.t.snapshot(now=10).head_fresh)

    def test_reset_removes_all_state(self):
        self.t.feed(HEAD + b"Walking State: IDLE\nBusy: NO\n", now=10)
        self.t.reset()
        s = self.t.snapshot(now=10)
        self.assertFalse(s.head_fresh)
        self.assertFalse(s.body_idle_confirmed)

    def test_limit_unterminated_uart_line(self):
        self.t.feed(b"x" * 10000, now=10)
        self.assertLessEqual(len(self.t._pending), 512)
        self.assertFalse(self.t.snapshot(now=10).head_fresh)


if __name__ == "__main__":
    unittest.main()
