"""Software-only safety tests (CI runs); physical hardware tests are deferred."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety_manager import Mode, SafetyManager, SafetyPolicy


class SafetyManagerTests(unittest.TestCase):
    def setUp(self):
        self.s = SafetyManager()
        self.s.observe_detection(True, 130.0, now=10.0)
        self.s.observe_detection(True, 130.0, now=10.1)
        self.s.observe_detection(True, 130.0, now=10.2)

    def test_never_auto_arms(self):
        self.assertEqual(self.s.mode, Mode.SAFE)
        self.assertFalse(self.s.authorize("TRACK_LEFT", source="AUTO", now=10.3).approved)

    def test_arm_after_three_frames(self):
        self.assertTrue(self.s.arm_auto(now=10.3))
        self.assertTrue(self.s.authorize("TRACK_LEFT", source="AUTO", now=10.3).approved)

    def test_reject_translation_default(self):
        self.s.arm_auto(now=10.3)
        result = self.s.authorize("TRACK_FORWARD", source="AUTO", now=10.3)
        self.assertEqual(result.command, "STOP")
        self.assertFalse(result.approved)

    def test_target_lost_forces_disarm(self):
        self.s.arm_auto(now=10.3)
        self.s.observe_detection(False, 125.0, now=10.4)
        self.assertEqual(self.s.mode, Mode.SAFE)
        self.assertFalse(self.s.arm_auto(now=10.41))

    def test_link_timeout_forces_disarm(self):
        self.s.arm_auto(now=10.3)
        self.s.tick(now=11.1)
        self.assertEqual(self.s.mode, Mode.SAFE)
        self.assertEqual(self.s.reason, "LINK_TIMEOUT")

    def test_stale_rtt_forces_disarm(self):
        self.s.arm_auto(now=10.3)
        self.s.observe_detection(True, 800.0, now=10.4)
        self.assertEqual(self.s.mode, Mode.SAFE)
        self.assertEqual(self.s.reason, "STALE_OR_INVALID_RESULT")

    def test_estop_latched_until_explicit_reset(self):
        self.s.arm_auto(now=10.3)
        self.s.emergency_stop()
        self.assertEqual(self.s.mode, Mode.ESTOP)
        self.s.observe_detection(True, 100.0, now=10.35)
        self.assertFalse(self.s.arm_auto(now=10.36))
        self.s.reset_emergency_stop()
        self.assertEqual(self.s.mode, Mode.SAFE)
        self.assertFalse(self.s.arm_auto(now=10.37))

    def test_manual_deadman(self):
        self.assertTrue(self.s.arm_manual(now=10.3))
        self.assertTrue(self.s.authorize("TRACK_RIGHT", source="MANUAL", now=10.4).approved)
        self.s.tick(now=10.9)
        self.assertEqual(self.s.mode, Mode.SAFE)

    def test_mode_mismatch_denies(self):
        self.s.arm_manual(now=10.3)
        self.assertFalse(self.s.authorize("TRACK_LEFT", source="AUTO", now=10.35).approved)

    def test_unknown_command_denied(self):
        self.s.arm_auto(now=10.3)
        self.assertFalse(self.s.authorize("STAND", source="AUTO", now=10.35).approved)

    def test_stop_always_allowed(self):
        self.s.emergency_stop()
        decision = self.s.authorize("STOP", source="MANUAL", now=10.3)
        self.assertTrue(decision.approved)
        self.assertEqual(decision.command, "STOP")
        self.assertEqual(self.s.mode, Mode.ESTOP)

    def test_disconnect_requires_rearm(self):
        self.s.arm_auto(now=10.3)
        self.s.connection_lost()
        self.s.observe_detection(True, 100.0, now=10.5)
        self.assertEqual(self.s.mode, Mode.SAFE)

    def test_no_person_in_manual_does_not_force_auto_restart(self):
        self.s.arm_manual(now=10.3)
        self.s.observe_detection(False, 130.0, now=10.4)
        self.assertEqual(self.s.mode, Mode.MANUAL)

    def test_invalid_policy_rejected(self):
        with self.assertRaises(ValueError):
            SafetyPolicy(link_timeout_s=0)

    def test_manual_arm_does_not_require_gpu(self):
        s = SafetyManager()
        self.assertTrue(s.arm_manual(now=20.0))
        self.assertTrue(s.authorize("TRACK_LEFT", source="MANUAL", now=20.05).approved)

    def test_manual_survives_gpu_disconnect(self):
        s = SafetyManager()
        self.assertTrue(s.arm_manual(now=20.0))
        s.connection_lost()
        self.assertEqual(s.mode, Mode.MANUAL)
        self.assertTrue(s.authorize("TRACK_RIGHT", source="MANUAL", now=20.08).approved)

    def test_manual_deadman_applies_without_gpu(self):
        s = SafetyManager()
        s.arm_manual(now=20.0)
        s.tick(now=20.50)
        self.assertEqual(s.mode, Mode.SAFE)

    def test_auto_does_not_survive_gpu_disconnect(self):
        self.s.arm_auto(now=10.3)
        self.s.connection_lost()
        self.assertEqual(self.s.mode, Mode.SAFE)

    def test_status_ready_only_not_armed(self):
        status = self.s.status(now=10.25)
        self.assertEqual(status.mode, Mode.SAFE)
        self.assertTrue(status.auto_ready)


if __name__ == "__main__":
    unittest.main()
