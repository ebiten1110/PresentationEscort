"""Software-only tests: no ESP32 or serial hardware."""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motion_gateway import MemoryCommandSink, PiMotionGateway
from safety_manager import Mode, SafetyManager, SafetyPolicy


class PretendHardwareSink(MemoryCommandSink):
    requires_watchdog = True


class FailingSink:
    def send(self, command):
        raise OSError("serial cable disconnected")


class GatewayTests(unittest.TestCase):
    def make(self, *, policy=None, sink=None):
        sink = sink if sink is not None else MemoryCommandSink()
        manager = SafetyManager(policy)
        gateway = PiMotionGateway(sink, safety=manager, poll_interval_s=0.02)
        self.addCleanup(gateway.close)
        return gateway, sink

    @staticmethod
    def prime(g, now=10.0):
        for i in range(3):
            g.observe_detection(True, 130.0, now=now + i * 0.05)

    def test_hardware_sink_requires_live_watchdog(self):
        g, sink = self.make(sink=PretendHardwareSink())
        self.assertFalse(g.arm_manual())
        self.assertFalse(g.arm_auto())
        g.start()
        self.assertTrue(g.arm_manual())
        self.assertEqual(sink.commands[0], "STOP")

    def test_initial_stop_and_disarmed(self):
        g, sink = self.make()
        g.start()
        self.assertEqual(sink.commands, ["STOP"])
        self.assertEqual(g.status().safety.mode, Mode.SAFE)

    def test_no_motion_until_armed(self):
        g, sink = self.make()
        self.prime(g)
        result = g.request_motion("TRACK_LEFT", source="AUTO", now=10.2)
        self.assertFalse(result.approved)
        # Failed authorization sends an explicit STOP as a defensive command.
        self.assertEqual(sink.commands, ["STOP"])

    def test_explicit_arm_allows_turn(self):
        g, sink = self.make()
        self.prime(g)
        self.assertTrue(g.arm_auto(now=10.15))
        self.assertTrue(g.request_motion("TRACK_LEFT", source="AUTO", now=10.16).approved)
        self.assertEqual(sink.commands, ["TRACK_LEFT"])

    def test_no_detection_triggers_stop(self):
        g, sink = self.make()
        self.prime(g)
        g.arm_auto(now=10.15)
        g.request_motion("TRACK_LEFT", source="AUTO", now=10.16)
        g.observe_detection(False, 135.0, now=10.17)
        self.assertEqual(g.status(now=10.18).safety.mode, Mode.SAFE)
        self.assertEqual(sink.commands, ["TRACK_LEFT", "STOP"])

    def test_link_timeout_stops_even_with_no_new_frames(self):
        g, sink = self.make()
        self.prime(g)
        g.arm_auto(now=10.15)
        g.request_motion("TRACK_RIGHT", source="AUTO", now=10.16)
        g.check_watchdog(now=11.1)
        self.assertEqual(sink.commands, ["TRACK_RIGHT", "STOP"])
        self.assertEqual(g.status(now=11.1).safety.mode, Mode.SAFE)

    def test_emergency_stop_is_latched(self):
        g, sink = self.make()
        self.prime(g)
        g.arm_auto(now=10.15)
        g.request_motion("TRACK_LEFT", source="AUTO", now=10.16)
        g.emergency_stop()
        self.assertEqual(g.status(now=10.17).safety.mode, Mode.ESTOP)
        self.assertEqual(sink.commands, ["TRACK_LEFT", "STOP"])
        self.assertFalse(g.arm_auto(now=10.18))

    def test_default_translation_is_blocked(self):
        g, sink = self.make()
        self.prime(g)
        g.arm_auto(now=10.15)
        r = g.request_motion("TRACK_FORWARD", source="AUTO", now=10.16)
        self.assertFalse(r.approved)
        # Even a forbidden forward request is answered by STOP, never movement.
        self.assertEqual(sink.commands, ["STOP"])

    def test_send_failure_latches_transport_fault(self):
        g, sink = self.make(sink=FailingSink())
        self.prime(g)
        g.arm_auto(now=10.15)
        r = g.request_motion("TRACK_LEFT", source="AUTO", now=10.16)
        self.assertFalse(r.approved)
        self.assertTrue(g.status(now=10.17).transport_fault)
        self.assertEqual(g.status(now=10.17).safety.mode, Mode.ESTOP)

    def test_startup_stop_failure_rejects_use(self):
        g, sink = self.make(sink=FailingSink())
        with self.assertRaises(RuntimeError):
            g.start()
        self.assertFalse(g.arm_manual(now=10.0))

    def test_manual_without_detector_connection(self):
        g, sink = self.make()
        self.assertTrue(g.arm_manual(now=10.0))
        self.assertTrue(g.request_motion("TRACK_LEFT", source="MANUAL", now=10.1).approved)
        self.assertEqual(sink.commands, ["TRACK_LEFT"])

    def test_mode_switch_stops_previous_motion(self):
        g, sink = self.make()
        self.prime(g)
        self.assertTrue(g.arm_auto(now=10.15))
        self.assertTrue(g.request_motion("TRACK_LEFT", source="AUTO", now=10.16).approved)
        self.assertTrue(g.arm_manual(now=10.17))
        self.assertEqual(sink.commands, ["TRACK_LEFT", "STOP"])
        self.assertEqual(g.status(now=10.17).safety.mode, Mode.MANUAL)

    def test_manual_can_survive_detector_disconnect(self):
        g, sink = self.make()
        self.assertTrue(g.arm_manual(now=10.0))
        self.assertTrue(g.request_motion("TRACK_RIGHT", source="MANUAL", now=10.1).approved)
        g.connection_lost()
        self.assertEqual(g.status(now=10.11).safety.mode, Mode.MANUAL)
        self.assertEqual(sink.commands, ["TRACK_RIGHT"])

    def test_manual_deadman_stops(self):
        g, sink = self.make()
        g.arm_manual(now=10.0)
        g.request_motion("TRACK_RIGHT", source="MANUAL", now=10.1)
        g.check_watchdog(now=10.6)
        self.assertEqual(g.status(now=10.6).safety.mode, Mode.SAFE)
        self.assertEqual(sink.commands[-1], "STOP")

    def test_shutdown_sends_stop(self):
        g, sink = self.make()
        self.prime(g)
        g.arm_auto(now=10.15)
        g.request_motion("TRACK_LEFT", source="AUTO", now=10.16)
        g.close()
        self.assertEqual(sink.commands[-1], "STOP")
        self.assertFalse(g.arm_auto(now=10.17))

    def test_background_watchdog_stops_blocked_inference(self):
        policy = SafetyPolicy(link_timeout_s=0.10, target_timeout_s=0.10)
        g, sink = self.make(policy=policy)
        g.start()
        for _ in range(3):
            g.observe_detection(True, 40.0)
        self.assertTrue(g.arm_auto())
        self.assertTrue(g.request_motion("TRACK_LEFT", source="AUTO").approved)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and sink.commands[-1] != "STOP":
            time.sleep(0.01)
        self.assertEqual(sink.commands[-1], "STOP")
        self.assertEqual(g.status().safety.mode, Mode.SAFE)


if __name__ == "__main__":
    unittest.main()
