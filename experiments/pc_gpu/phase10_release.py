"""Phase 10: verify pinned code and create a source/history backup.

All operations are local Git/filesystem only. Never opens an ESP32 serial
port, connects to a robot or writes firmware. The pinned Phase 9 baseline is
not a promise that the currently installed ESP32 firmware matches source.

  python3 experiments/pc_gpu/phase10_release.py check
  python3 experiments/pc_gpu/phase10_release.py backup --dest ~/PresentationEscort-backups
  python3 experiments/pc_gpu/phase10_release.py verify --backup-dir ~/PresentationEscort-backups/<snapshot>

Requires Git. Backups are written OUTSIDE the working checkout, never
overwrite existing files, and have SHA-256 checksums.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "docs" / "PHASE10_CODE_BASELINE.json"
_HEX40 = re.compile(r"^[a-f0-9]{40}$")
_MAX_FILES = 100


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=repo, capture_output=True, text=True,
        timeout=30, check=True,
    )
    return result.stdout.strip()


def _blob_sha(raw: bytes) -> str:
    prefix = b"blob " + str(len(raw)).encode("ascii") + b"\0"
    return hashlib.sha1(prefix + raw).hexdigest()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(part)
    return digest.hexdigest()


def validate_manifest(manifest: dict) -> None:
    if (
        manifest.get("schema_version") != 1
        or manifest.get("project") != "PresentationEscort"
        or manifest.get("hardware_validated") is not False
        or not _HEX40.fullmatch(str(manifest.get("baseline_commit", "")))
    ):
        raise ValueError("invalid or unapproved release manifest")
    files = manifest.get("files")
    if not isinstance(files, dict) or not 1 <= len(files) <= _MAX_FILES:
        raise ValueError("invalid file manifest")
    for rel, value in files.items():
        path = Path(rel)
        if (
            not isinstance(rel, str)
            or path.is_absolute()
            or ".." in path.parts
            or not _HEX40.fullmatch(str(value))
        ):
            raise ValueError("unsafe path or blob hash in manifest")


def audit(
    root: Path = ROOT,
    *,
    manifest_path: Optional[Path] = None,
    require_git: bool = True,
) -> dict:
    root = Path(root).resolve()
    path = Path(manifest_path) if manifest_path else root / "docs" / "PHASE10_CODE_BASELINE.json"
    doc = json.loads(path.read_text(encoding="utf-8"))
    validate_manifest(doc)
    file_results = {}
    for relative, expected in doc["files"].items():
        file = root / relative
        try:
            if file.is_symlink():
                raise ValueError("symlink prohibited")
            received = _blob_sha(file.read_bytes())
            file_results[relative] = received == expected
        except (OSError, ValueError):
            file_results[relative] = False
    baseline_reachable = False
    current = None
    dirty = None
    branch = None
    if require_git:
        try:
            current = _git(root, "rev-parse", "HEAD")
            branch = _git(root, "branch", "--show-current")
            dirty = bool(_git(root, "status", "--porcelain", "--untracked-files=normal"))
            subprocess.run(
                ["git", "merge-base", "--is-ancestor", doc["baseline_commit"], "HEAD"],
                cwd=root, capture_output=True, timeout=30, check=True,
            )
            baseline_reachable = True
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            pass
    all_matched = all(file_results.values())
    return {
        "baseline_commit": doc["baseline_commit"],
        "head_commit": current,
        "branch": branch,
        "baseline_reachable": baseline_reachable,
        "working_tree_dirty": dirty,
        "file_checks": file_results,
        "pinned_files_match": all_matched,
        # A missing local Git history is a blocker for a reliable rollback.
        "ready_for_source_backup": bool(
            require_git and all_matched and baseline_reachable and dirty is False
        ),
        "hardware_approved": False,
    }


def make_backup(root: Path, dest: Path, *, checked: Optional[dict] = None) -> Path:
    root = Path(root).resolve()
    dest = Path(dest).expanduser().resolve()
    check = checked or audit(root)
    if not check["ready_for_source_backup"]:
        raise ValueError("backup blocked: source mismatch, missing history, or dirty checkout")
    if root == dest or root in dest.parents:
        raise ValueError("backup directory must be outside project checkout")
    dest.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    snap = dest / ("PresentationEscort-" + stamp + "-" + check["head_commit"][:12])
    # Exclusive create; never overwrite another backup.
    snap.mkdir(mode=0o700, exist_ok=False)
    names = ["source.tar.gz", "history.bundle"]
    try:
        _git(root, "archive", "--format=tar.gz", "-o", str(snap / names[0]), "HEAD")
        _git(root, "bundle", "create", str(snap / names[1]), "HEAD")
        manifest = {
            "format": "PresentationEscort-phase10-backup-v1",
            "source_commit": check["head_commit"],
            "baseline_commit": check["baseline_commit"],
            "branch": check["branch"],
            "created_utc": stamp,
            "firmware_binary_included": False,
            "hardware_validated": False,
            "files": {name: _sha256(snap / name) for name in names},
        }
        (snap / "backup.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )
    except Exception:
        # Incomplete backups cannot be mistaken for valid ones.
        for entry in snap.iterdir():
            if entry.is_file():
                entry.unlink()
        snap.rmdir()
        raise
    return snap


def verify_backup(folder: Path) -> dict:
    directory = Path(folder).expanduser().resolve()
    doc = json.loads((directory / "backup.json").read_text(encoding="utf-8"))
    if doc.get("format") != "PresentationEscort-phase10-backup-v1":
        raise ValueError("unrecognized backup format")
    if not _HEX40.fullmatch(str(doc.get("source_commit", ""))):
        raise ValueError("invalid source commit in backup")
    if doc.get("firmware_binary_included") is not False:
        raise ValueError("unexpected firmware binary claim")
    files = doc.get("files")
    if not isinstance(files, dict) or set(files) != {"source.tar.gz", "history.bundle"}:
        raise ValueError("incomplete backup manifest")
    result = {}
    for name, expected in files.items():
        file = directory / name
        result[name] = (
            isinstance(expected, str) and len(expected) == 64
            and not file.is_symlink() and file.is_file()
            and _sha256(file) == expected
        )
    return {"verified": all(result.values()), "files": result, "commit": doc["source_commit"]}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Offline source integrity / backup only")
    parser.add_argument("command", choices=("check", "backup", "verify"))
    parser.add_argument("--dest", default="~/PresentationEscort-backups")
    parser.add_argument("--backup-dir")
    args = parser.parse_args(argv)
    try:
        if args.command == "verify":
            if not args.backup_dir:
                parser.error("verify requires --backup-dir")
            result = verify_backup(Path(args.backup_dir))
            print(json.dumps(result, indent=2))
            return 0 if result["verified"] else 2
        result = audit()
        print(json.dumps(result, indent=2))
        if args.command == "check":
            return 0 if result["ready_for_source_backup"] else 2
        dest = make_backup(ROOT, Path(args.dest), checked=result)
        print("Backup created and hashed:", dest)
        print("Backup verification:", json.dumps(verify_backup(dest), indent=2))
        return 0
    except (OSError, ValueError, json.JSONDecodeError, subprocess.CalledProcessError,
            subprocess.TimeoutExpired) as exc:
        print("BLOCKED:", str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
