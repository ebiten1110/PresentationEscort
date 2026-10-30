"""Phase 10 human acceptance gate: never grants motion, fails closed."""
from __future__ import annotations
import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from phase10_gate import TEMPLATE, evaluate


class GoNoGoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = json.loads(TEMPLATE.read_text(encoding="utf-8"))

    def test_original_template_is_no_go(self):
        result = evaluate(copy.deepcopy(self.base))
        self.assertFalse(result["go"])
        self.assertGreater(len(result["remaining"]), 10)

    def test_only_attestations_do_not_substitute_for_real_tests(self):
        doc = copy.deepcopy(self.base)
        doc["operator_attestation"] = "operator"
        doc["observer_attestation"] = "observer"
        self.assertFalse(evaluate(doc)["go"])

    def test_complete_human_report_is_marked_reported_only(self):
        doc = copy.deepcopy(self.base)
        for item in doc["checks"].values():
            item["status"] = "PASS"
            item["evidence"] = "2026-10-28 supervised physical test log"
        doc["operator_attestation"] = "operator"
        doc["observer_attestation"] = "observer"
        result = evaluate(doc)
        self.assertTrue(result["go"])
        self.assertIn("does not authorize", result["caution"])

    def test_missing_one_observation_is_no_go(self):
        doc = copy.deepcopy(self.base)
        for item in doc["checks"].values():
            item["status"] = "PASS"
            item["evidence"] = "physical test video"
        doc["operator_attestation"] = "operator"
        doc["observer_attestation"] = "observer"
        doc["checks"]["P08_stop_and_estop_on_support"]["status"] = "FAIL"
        self.assertFalse(evaluate(doc)["go"])

    def test_trivial_evidence_is_no_go(self):
        doc = copy.deepcopy(self.base)
        for item in doc["checks"].values():
            item["status"] = "PASS"
            item["evidence"] = "ok"
        doc["operator_attestation"] = "operator"
        doc["observer_attestation"] = "observer"
        self.assertFalse(evaluate(doc)["go"])

    def test_unexpected_or_removed_checks_invalid(self):
        doc = copy.deepcopy(self.base)
        doc["checks"]["unknown"] = {"status": "PASS", "evidence": "any"}
        self.assertEqual(evaluate(doc)["reason"], "INVALID_OR_INCOMPLETE_CHECKLIST")
        doc = copy.deepcopy(self.base)
        doc["checks"].pop("P08_stop_and_estop_on_support")
        self.assertFalse(evaluate(doc)["go"])

    def test_simulation_test_results_not_accepted_as_physical_evidence(self):
        # The template does not contain a shortcut based on CI counts.
        doc = copy.deepcopy(self.base)
        doc["software_tests_passed"] = 9999
        self.assertFalse(evaluate(doc)["go"])


if __name__ == "__main__":
    unittest.main()
