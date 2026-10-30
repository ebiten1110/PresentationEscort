"""Simulated UART feedback polling, no serial hardware."""
from __future__ import annotations
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from esp32_feedback_poller import Esp32FeedbackPoller
from motion_gateway import MemoryCommandSink, PiMotionGateway
from remote_person_controller import RemotePersonController


class FakeSink(MemoryCommandSink):
    def __init__(self):
        super().__init__()
        self.receive = b""
    def read_available(self, *, max_bytes=4096):
        data = self.receive[:max_bytes]
        self.receive = self.receive[max_bytes:]
        return data


class PollerTests(unittest.TestCase):
    def setUp(self):
        self.sink = FakeSink()
        self.gate = PiMotionGateway(self.sink)
        self.ctrl = RemotePersonController(self.gate)
        self.poller = Esp32FeedbackPoller(self.sink, self.ctrl)
        self.addCleanup(self.gate.close)

    def test_queries_status_not_motion(self):
        self.assertEqual(self.poller.poll(now=10), 0)
        self.assertEqual(self.sink.commands, ["STATUS"])
        self.poller.poll(now=10.1)
        self.assertEqual(self.sink.commands, ["STATUS"])
        self.poller.poll(now=10.21)
        self.assertEqual(self.sink.commands, ["STATUS", "STATUS"])

    def test_telemetry_read_changes_state(self):
        self.sink.receive = (
            b"[HeadState] currentYaw=-6.0 targetYaw=-7 currentPitch=1 targetPitch=2 moving=YES\n"
            b"Walking State: IDLE\nBusy: NO\n"
        )
        count = self.poller.poll(now=10)
        self.assertGreater(count, 0)
        t = self.ctrl.telemetry.snapshot(now=10.01)
        self.assertTrue(t.body_idle_confirmed)
        self.assertEqual(t.head_yaw_deg, -6)

    def test_stale_status_disables_body(self):
        self.sink.receive = b"Walking State: IDLE\nBusy: NO\n"
        self.poller.poll(now=10)
        self.assertFalse(self.ctrl.telemetry.snapshot(now=11).body_idle_confirmed)

    def test_invalid_interval_rejected(self):
        with self.assertRaises(ValueError):
            Esp32FeedbackPoller(self.sink, self.ctrl, status_interval_s=0.05)


if __name__ == "__main__":
    unittest.main()
