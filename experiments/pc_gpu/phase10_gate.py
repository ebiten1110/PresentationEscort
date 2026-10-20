"""Phase 10 human-observed hardware acceptance report (offline-only).

A PASS is self-reported by an on-site operator, NOT verified by software.
This script only evaluates a JSON checklist and will never arm hardware.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[2]
TEMPLATE = ROOT / "docs" / "PHASE10_HARDWARE_GATE_TEMPLATE.json"
ACCEPTED = frozenset({"PASS", "FAIL", "NOT_RUN"})


def evaluate(report: dict, *, template: Optional[dict] = None) -> dict:
    base = template if template is not None else json.loads(TEMPLATE.read_text(encoding="utf-8"))
    wanted = base.get("checks")
    checks = report.get("checks")
    if (
        report.get("schema_version") != 1
        or report.get("project") != "PresentationEscort"
        or not isinstance(wanted, dict)
        or not isinstance(checks, dict)
        or set(wanted) != set(checks)
    ):
        return {"go": False, "reason": "INVALID_OR_INCOMPLETE_CHECKLIST", "remaining": list(wanted or {})}
    issues = []
    for key in wanted:
        answer = checks.get(key)
        if not isinstance(answer, dict) or answer.get("status") not in ACCEPTED:
            issues.append(key)
        elif answer["status"] != "PASS" or not isinstance(answer.get("evidence"), str) or len(answer["evidence"].strip()) < 8:
            issues.append(key)
    signed = all(
        isinstance(report.get(key), str)
        and len(report[key].strip()) >= 3
        for key in ("operator_attestation", "observer_attestation")
    )
    if not signed:
        issues.append("TWO_OPERATOR_ATTESTATIONS_REQUIRED")
    # The evaluator never writes to a robot or changes launch mode.
    return {
        "go": not issues,
        "reason": "HUMAN_REPORTED_GATE_COMPLETE" if not issues else "NO_GO",
        "remaining": issues,
        "caution": "Attested observations only; this output does not authorize automatic motor startup.",
    }


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Offline human test checklist evaluator")
    parser.add_argument("--report", type=Path, default=TEMPLATE)
    args = parser.parse_args(argv)
    try:
        report = json.loads(args.report.read_text(encoding="utf-8"))
        result = evaluate(report)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print("NO-GO:", str(exc))
        return 2
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["go"] else 2


if __name__ == "__main__":
    sys.exit(main())
