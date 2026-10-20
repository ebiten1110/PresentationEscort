"""Phase 12: offline, read-only October 28 rehearsal and readiness report.

This script NEVER opens a camera, socket, serial port, GPIO or motor.
It cannot approve motor activation. Its observations concern *this checkout*
and a virtual ESP32 only; real hardware requires human-supervised checks.

Examples:
    python3 experiments/pc_gpu/phase12_rehearsal.py
    python3 experiments/pc_gpu/phase12_rehearsal.py --backup-dir ~/PresentationEscort-backups/<snapshot>
    python3 experiments/pc_gpu/phase12_rehearsal.py --output-dir ~/PresentationEscort-reports
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from detection_protocol import DetectionResult
from phase10_gate import TEMPLATE, evaluate
from phase10_release import audit as code_audit, verify_backup
from pi_deployment_launcher import repository_audit
from presenter_lock import PresenterLock
from virtual_esp32 import VirtualEsp32


def _candidate(x: int, confidence: float = 0.80) -> dict:
    return {"x1": x, "y1": 20, "x2": x + 60, "y2": 190,
            "confidence": confidence}


def _frame(seq: int, boxes: list[dict]) -> DetectionResult:
    return DetectionResult(
        found=bool(boxes), class_id=0, class_name="person",
        frame_width=320, frame_height=240,
        protocol_version=2, frame_seq=seq, people=boxes,
    )


def simulation_checks() -> dict[str, bool]:
    """Repeat a tiny deterministic critical-path smoke test (not 274+ CI)."""
    checks: dict[str, bool] = {}
    tick = [100.0]
    dev = VirtualEsp32(clock=lambda: tick[0])
    try:
        dev.send("TRACK_LEFT")
        checks["startup_denies_motion"] = (
            dev.state == "DISARMED" and dev.walking_state == "IDLE"
        )
        dev.send("SAFETY_ARM")
        ack = dev.read_available()
        checks["operator_arm_needs_firmware_ack"] = (
            dev.state == "ARMED" and b"[Safety] ARM_ACCEPTED" in ack
            and b"State: ARMED" in ack
        )
        dev.send("HEAD_POSE_SET:7.0,3.0")
        checks["head_pose_limited_and_virtual"] = (
            dev.yaw_deg == 7 and dev.pitch_deg == 3
            and isinstance(dev, VirtualEsp32)
        )
        dev.send("STOP")
        dev.send("TRACK_RIGHT")
        checks["stop_disarms_and_blocks_resume"] = (
            dev.state == "DISARMED" and dev.walking_state == "IDLE"
        )
        dev.send("SAFETY_ARM")
        tick[0] += .751
        dev.read_available()
        dev.send("SAFETY_PING")
        dev.send("TRACK_LEFT")
        checks["late_ping_cannot_clear_timeout"] = (
            dev.state == "LATCHED" and dev.walking_state == "IDLE"
        )
        dev.send("SAFETY_RESET")
        checks["reset_requires_explicit_rearm"] = (
            dev.state == "DISARMED" and dev.walking_state == "IDLE"
        )
        lock = PresenterLock()
        lock.observe(_frame(1, [_candidate(20, .60),
                                _candidate(220, .99)]), now=100)
        checks["no_implicit_presenter_lock"] = not lock.locked
        selected, _ = lock.select(0, 1, now=100.05)
        followed = lock.observe(
            _frame(2, [_candidate(221, .99), _candidate(22, .56)]),
            now=100.12,
        )
        checks["operator_lock_ignores_stranger_confidence"] = (
            selected and followed.found and followed.center_x == 52
        )
        crossed = lock.observe(
            _frame(3, [_candidate(27), _candidate(43)]), now=100.2
        )
        checks["crowding_loses_lock"] = (
            not crossed.found and not lock.locked
        )
        regained = lock.observe(_frame(4, [_candidate(220)]), now=100.3)
        checks["lost_lock_never_auto_reacquires"] = (
            not regained.found and not lock.locked
        )
    finally:
        dev.close()
    return checks


def _load_report(path: Optional[Path]) -> dict:
    source = path if path is not None else TEMPLATE
    return json.loads(source.expanduser().read_text(encoding="utf-8"))


def evaluate_readiness(
    *,
    root: Optional[Path] = None,
    backup_dir: Optional[Path] = None,
    hardware_report: Optional[Path] = None,
) -> dict:
    """Gather factual code/simulation evidence; hardware remains manual."""
    from phase10_release import ROOT
    repo = (root or ROOT).resolve()
    source = code_audit(repo)
    static = repository_audit(repo)
    virtual = simulation_checks()
    backup = {"verified": False, "reason": "NOT_PROVIDED"}
    if backup_dir is not None:
        try:
            backup = verify_backup(backup_dir)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            backup = {"verified": False, "reason": "BACKUP_INVALID_" + type(exc).__name__}
    gate = evaluate(_load_report(hardware_report))
    blockers = []
    if not source["ready_for_source_backup"]:
        blockers.append("PINNED_SOURCE_OR_GIT_AUDIT_FAILED")
    if not all(static.values()):
        blockers.append("STATIC_SAFETY_AUDIT_FAILED")
    if not all(virtual.values()):
        blockers.append("SIMULATED_CRITICAL_PATH_FAILED")
    if not backup.get("verified", False):
        blockers.append("VERIFIED_OFF_DEVICE_SOURCE_BACKUP_MISSING")
    if not gate["go"]:
        blockers.append("PHYSICAL_TESTS_NOT_VERIFIED")
    # Even a fully signed paper checklist does not remotely grant permission
    # to move the real robot, modify firmware, or mark hardware safe.
    return {
        "schema_version": 1,
        "project": "PresentationEscort",
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": source,
        "static_safety_checks": static,
        "virtual_smoke_checks": virtual,
        "backup": backup,
        "hardware_report": gate,
        "blockers": blockers,
        "offline_rehearsal_passed": bool(
            source["ready_for_source_backup"]
            and all(static.values()) and all(virtual.values())
        ),
        "readiness_evidence_complete": not blockers,
        "motor_activation_permitted": False,
        "real_esp32_flash_performed": False,
        "real_servo_test_performed": False,
        "disclaimer": (
            "Read-only offline simulation and user-supplied attestations only; "
            "does not grant physical motion or firmware flashing permission."
        ),
    }


def save_report(report: dict, output_dir: Path) -> Path:
    """Immutable timestamped report; never overwrite existing evidence."""
    target = output_dir.expanduser().resolve()
    target.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    name = "PresentationEscort-preflight-" + stamp + ".json"
    path = target / name
    data = (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode()
    with path.open("xb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    # SHA256 is an integrity checksum, not a signature of the operator.
    sha = hashlib.sha256(data).hexdigest()
    with (target / (name + ".sha256")).open("x", encoding="ascii") as output:
        output.write(sha + "  " + name + "\n")
    return path


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Offline Phase 12 rehearsal, NO ESP32 or motors")
    parser.add_argument("--backup-dir", type=Path)
    parser.add_argument("--hardware-report", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    try:
        report = evaluate_readiness(
            backup_dir=args.backup_dir, hardware_report=args.hardware_report,
        )
        print(json.dumps({
            "offline_rehearsal_passed": report["offline_rehearsal_passed"],
            "readiness_evidence_complete": report["readiness_evidence_complete"],
            "motor_activation_permitted": False,
            "blockers": report["blockers"],
            "virtual_smoke_checks": report["virtual_smoke_checks"],
        }, indent=2, ensure_ascii=False))
        if args.output_dir:
            dest = save_report(report, args.output_dir)
            print("Offline report saved:", dest)
        # Exit 0 means offline software checks pass, NOT a real hardware GO.
        return 0 if report["offline_rehearsal_passed"] else 2
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        print("NO-GO:", type(exc).__name__, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
