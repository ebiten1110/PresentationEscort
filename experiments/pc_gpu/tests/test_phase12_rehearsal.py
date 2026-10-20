"""Phase 12: executable offline rehearsal / record keeping tests.

No physical or network access. Validation of a checklist or Git bundle does
not claim a physical STOP/servo test happened.
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from phase10_gate import TEMPLATE
from phase12_rehearsal import (
    evaluate_readiness, main, save_report, simulation_checks,
)


class OfflineSmokeTests(unittest.TestCase):
    def test_critical_virtual_scenarios(self):
        checks = simulation_checks()
        self.assertGreaterEqual(len(checks), 9)
        self.assertTrue(all(checks.values()), checks)

    def test_does_not_import_physical_serial_or_gpio(self):
        source = (Path(__file__).resolve().parents[1] / "phase12_rehearsal.py").read_text()
        self.assertNotIn("SerialCommandSink(", source)
        self.assertNotIn("import serial\n", source)
        self.assertNotIn("import RPi.GPIO", source)
        self.assertNotIn("cv2.VideoCapture(", source)
        self.assertNotIn("socket.create_connection(", source)

    def test_operator_never_silently_reacquires_target(self):
        self.assertTrue(simulation_checks()["lost_lock_never_auto_reacquires"])


class ReadinessTests(unittest.TestCase):
    def test_default_report_does_not_assert_hardware_pass(self):
        r = evaluate_readiness()
        self.assertTrue(r["offline_rehearsal_passed"], r)
        self.assertFalse(r["readiness_evidence_complete"])
        self.assertFalse(r["motor_activation_permitted"])
        self.assertFalse(r["real_servo_test_performed"])
        self.assertFalse(r["real_esp32_flash_performed"])
        self.assertIn("PHYSICAL_TESTS_NOT_VERIFIED", r["blockers"])
        self.assertIn("VERIFIED_OFF_DEVICE_SOURCE_BACKUP_MISSING", r["blockers"])

    def test_missing_backup_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = evaluate_readiness(backup_dir=Path(tmp))
        self.assertFalse(r["backup"]["verified"])
        self.assertFalse(r["motor_activation_permitted"])

    def test_complete_human_report_never_unlocks_motors(self):
        doc = json.loads(TEMPLATE.read_text())
        for item in doc["checks"].values():
            item["status"] = "PASS"
            item["evidence"] = "Witness checked physical safety on the robot"
        doc["operator_attestation"] = "Operator A"
        doc["observer_attestation"] = "Observer B"
        with tempfile.TemporaryDirectory() as tmp:
            report_file = Path(tmp) / "witness.json"
            report_file.write_text(json.dumps(doc))
            result = evaluate_readiness(hardware_report=report_file)
        self.assertTrue(result["hardware_report"]["go"])
        self.assertFalse(result["motor_activation_permitted"])
        self.assertIn("VERIFIED_OFF_DEVICE_SOURCE_BACKUP_MISSING", result["blockers"])

    def test_invalid_checklist_returns_no_go(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = Path(tmp) / "broken.json"
            report.write_text('{"schema_version":1,"project":"PresentationEscort","checks":{}}')
            r = evaluate_readiness(hardware_report=report)
        self.assertFalse(r["hardware_report"]["go"])
        self.assertIn("PHYSICAL_TESTS_NOT_VERIFIED", r["blockers"])

    def test_bad_source_audit_is_reported_not_hidden(self):
        with patch("phase12_rehearsal.code_audit") as fake:
            result = evaluate_readiness()
            self.assertTrue(fake.called)
        self.assertIn("source", result)

    def test_corrupted_static_audit_blocks_rehearsal(self):
        with patch("phase12_rehearsal.repository_audit", return_value={"core": False}):
            r = evaluate_readiness()
        self.assertFalse(r["offline_rehearsal_passed"])
        self.assertIn("STATIC_SAFETY_AUDIT_FAILED", r["blockers"])

    def test_broken_virtual_scenario_blocks_rehearsal(self):
        with patch("phase12_rehearsal.simulation_checks", return_value={"watchdog":False}):
            r = evaluate_readiness()
        self.assertFalse(r["offline_rehearsal_passed"])
        self.assertIn("SIMULATED_CRITICAL_PATH_FAILED", r["blockers"])

    def test_output_unique_and_sha256_verified(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = {"motor_activation_permitted": False, "test": "virtual"}
            first = save_report(report, Path(tmp))
            second = save_report(report, Path(tmp))
            self.assertNotEqual(first, second)
            for p in (first, second):
                raw = p.read_bytes()
                expected = hashlib.sha256(raw).hexdigest()
                checksum = Path(str(p) + ".sha256").read_text()
                self.assertTrue(checksum.startswith(expected))
                self.assertEqual(json.loads(raw), report)

    def test_offline_main_exit_code_is_not_motor_approval(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = main(["--output-dir", tmp])
            self.assertEqual(result, 0)
            files = list(Path(tmp).glob("*.json"))
            self.assertEqual(len(files), 1)
            doc = json.loads(files[0].read_text())
            self.assertFalse(doc["motor_activation_permitted"])
            self.assertFalse(doc["readiness_evidence_complete"])

    def test_no_hardware_report_returning_true_by_default(self):
        self.assertFalse(evaluate_readiness()["hardware_report"]["go"])


if __name__ == "__main__":
    unittest.main()
