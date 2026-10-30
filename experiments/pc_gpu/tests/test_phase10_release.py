"""Phase 10 offline release and backup tests; no actual hardware or GPU."""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from phase10_release import (
    ROOT, audit, make_backup, validate_manifest, verify_backup,
    _blob_sha,
)


def run_git(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", *args], cwd=repo, text=True, stderr=subprocess.PIPE
    ).strip()


class ReleaseManifestTests(unittest.TestCase):
    def test_actual_manifest_uses_pinned_phase9_source_hashes(self):
        status = audit(ROOT, require_git=False)
        self.assertTrue(status["pinned_files_match"], status["file_checks"])
        self.assertFalse(status["hardware_approved"])

    def test_file_mutation_is_detected_without_git(self):
        data = json.loads((ROOT / "docs/PHASE10_CODE_BASELINE.json").read_text())
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp)
            rel = next(iter(data["files"]))
            (dest / "docs").mkdir()
            (dest / rel).parent.mkdir(parents=True, exist_ok=True)
            (dest / rel).write_bytes(b"not the pinned source")
            (dest / "docs/PHASE10_CODE_BASELINE.json").write_text(
                json.dumps(data), encoding="utf-8"
            )
            result = audit(dest, require_git=False)
            self.assertFalse(result["pinned_files_match"])
            self.assertFalse(result["file_checks"][rel])

    def test_manifest_rejects_escape_paths(self):
        bad = {
            "schema_version": 1, "project": "PresentationEscort",
            "hardware_validated": False, "baseline_commit": "a" * 40,
            "files": {"../escape": "a" * 40},
        }
        with self.assertRaises(ValueError):
            validate_manifest(bad)

    def test_manifest_must_never_claim_hardware_approved(self):
        invalid = {
            "schema_version": 1, "project": "PresentationEscort",
            "hardware_validated": True, "baseline_commit": "a" * 40,
            "files": {"file.txt": "a" * 40},
        }
        with self.assertRaises(ValueError):
            validate_manifest(invalid)

    def test_blob_calculation_matches_git(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            run_git(repo, "init", "-q")
            (repo / "blob.txt").write_text("hi\n", encoding="utf-8")
            self.assertEqual(
                _blob_sha((repo / "blob.txt").read_bytes()),
                run_git(repo, "hash-object", "blob.txt"),
            )


class BackupTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.repo = Path(self.tempdir.name) / "source"
        self.repo.mkdir()
        run_git(self.repo, "init", "-q")
        run_git(self.repo, "config", "user.name", "Unit Tester")
        run_git(self.repo, "config", "user.email", "tests@example.invalid")
        (self.repo / "a.txt").write_text("pinned core\n", encoding="utf-8")
        run_git(self.repo, "add", "a.txt")
        run_git(self.repo, "commit", "-qm", "baseline")
        self.base_sha = run_git(self.repo, "rev-parse", "HEAD")
        (self.repo / "docs").mkdir()
        doc = {
            "schema_version": 1,
            "project": "PresentationEscort",
            "hardware_validated": False,
            "baseline_commit": self.base_sha,
            "files": {"a.txt": _blob_sha((self.repo / "a.txt").read_bytes())},
        }
        (self.repo / "docs/PHASE10_CODE_BASELINE.json").write_text(
            json.dumps(doc), encoding="utf-8"
        )
        run_git(self.repo, "add", "docs/PHASE10_CODE_BASELINE.json")
        run_git(self.repo, "commit", "-qm", "manifest")
        self.dest = Path(self.tempdir.name) / "backups"

    def test_clean_repository_is_ready_for_source_backup(self):
        status = audit(self.repo)
        self.assertTrue(status["ready_for_source_backup"], status)
        self.assertTrue(status["baseline_reachable"])
        self.assertFalse(status["working_tree_dirty"])

    def test_dirty_uncommitted_source_blocks_backup(self):
        (self.repo / "a.txt").write_text("unclean source\n")
        check = audit(self.repo)
        self.assertFalse(check["ready_for_source_backup"])
        with self.assertRaises(ValueError):
            make_backup(self.repo, self.dest)
        self.assertFalse(self.dest.exists())

    def test_untracked_files_block_backup(self):
        (self.repo / "secret.local").write_text("not for backup")
        self.assertFalse(audit(self.repo)["ready_for_source_backup"])

    def test_valid_backup_has_archive_bundle_and_hashes(self):
        created = make_backup(self.repo, self.dest)
        self.assertTrue((created / "source.tar.gz").is_file())
        self.assertTrue((created / "history.bundle").is_file())
        result = verify_backup(created)
        self.assertTrue(result["verified"], result)
        self.assertEqual(result["commit"], run_git(self.repo, "rev-parse", "HEAD"))
        doc = json.loads((created / "backup.json").read_text())
        self.assertIs(doc["firmware_binary_included"], False)
        run_git(self.repo, "bundle", "verify", str(created / "history.bundle"))

    def test_corrupted_archive_is_rejected(self):
        created = make_backup(self.repo, self.dest)
        with (created / "source.tar.gz").open("ab") as handle:
            handle.write(b"tampering")
        self.assertFalse(verify_backup(created)["verified"])

    def test_missing_bundle_is_rejected(self):
        created = make_backup(self.repo, self.dest)
        (created / "history.bundle").unlink()
        self.assertFalse(verify_backup(created)["verified"])

    def test_archive_never_written_inside_checkout(self):
        with self.assertRaises(ValueError):
            make_backup(self.repo, self.repo / "accidental-backups")

    def test_backup_does_not_override_existing_snapshot(self):
        # Freeze the timestamp so this tests the *same* snapshot path even
        # when two Git archive operations straddle a UTC second boundary.
        from unittest.mock import patch
        with patch("phase10_release.datetime") as clock:
            clock.now.return_value.strftime.return_value = "20261029T000000Z"
            created = make_backup(self.repo, self.dest)
            with self.assertRaises(FileExistsError):
                make_backup(self.repo, self.dest)
        self.assertTrue(verify_backup(created)["verified"])

    def test_checkout_not_on_baseline_history_is_rejected(self):
        # An unrelated new repository with a copied manifest is NOT a valid
        # rollback source simply because the on-disk file happens to match.
        outside = Path(self.tempdir.name) / "imposter"
        outside.mkdir()
        run_git(outside, "init", "-q")
        run_git(outside, "config", "user.name", "Unit Tester")
        run_git(outside, "config", "user.email", "tests@example.invalid")
        (outside / "a.txt").write_text("pinned core\n")
        (outside / "docs").mkdir()
        (outside / "docs/PHASE10_CODE_BASELINE.json").write_bytes(
            (self.repo / "docs/PHASE10_CODE_BASELINE.json").read_bytes()
        )
        run_git(outside, "add", ".")
        run_git(outside, "commit", "-qm", "unrelated")
        result = audit(outside)
        self.assertTrue(result["pinned_files_match"])
        self.assertFalse(result["baseline_reachable"])
        self.assertFalse(result["ready_for_source_backup"])


if __name__ == "__main__":
    unittest.main()
