"""CSP camera fix and privacy-bounded diagnostics. No hardware I/O."""
from __future__ import annotations

import http.client
import io
import json
import sys
import threading
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from diagnostic_capture import DiagnosticCapture
from detection_protocol import DetectionResult
from pi_operator_console import OperatorServer
from pi_real_head_integration import LockedRealRuntime


class StubSerial:
    def send(self, command):
        return None

    def read_available(self, *, max_bytes=4096):
        return b""

    def close(self):
        return None


class RingTests(unittest.TestCase):
    def test_bounded_bundle_contains_recent_images_and_decisions(self):
        recorder = DiagnosticCapture(max_events=2, max_frames=2, frame_interval_s=.1)
        for index in range(4):
            recorder.event({"frame_seq": index, "mode": "ESTOP"})
            recorder.frame(b"\xff\xd8" + bytes([index]) + b"\xff\xd9", at=1. + index*.2)
        payload = recorder.export(status={"fw_safety": "LATCHED"})
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            names = archive.namelist()
            self.assertEqual(len([p for p in names if p.startswith("frames/")]), 2)
            self.assertEqual(json.loads(archive.read("status.json"))["status"]["fw_safety"], "LATCHED")
            decisions = [json.loads(row) for row in
                         archive.read("decisions.jsonl").decode().splitlines()]
            self.assertEqual([row["frame_seq"] for row in decisions], [2, 3])
            self.assertFalse(any("token" in row for row in decisions))

    def test_non_jpeg_is_ignored(self):
        recorder = DiagnosticCapture()
        recorder.frame(b"untrusted-data")
        with zipfile.ZipFile(io.BytesIO(recorder.export(status={}))) as archive:
            self.assertFalse(any(name.startswith("frames/") for name in archive.namelist()))

    def test_corrupt_gpu_number_does_not_break_diagnostics(self):
        recorder = DiagnosticCapture()
        recorder.event({"inference_ms": float("nan")})
        with zipfile.ZipFile(io.BytesIO(recorder.export(status={"rtt_ms": float("inf")}))) as archive:
            row = json.loads(archive.read("decisions.jsonl"))
            self.assertIsNone(row["inference_ms"])
            self.assertIsNone(json.loads(archive.read("status.json"))["status"]["rtt_ms"])


class AuthenticatedExportTests(unittest.TestCase):
    def setUp(self):
        serial_patch = patch("pi_real_head_integration.SerialCommandSink", return_value=StubSerial())
        serial_patch.start()
        self.addCleanup(serial_patch.stop)
        self.runtime = LockedRealRuntime("test-uart", capture_diagnostics=True)
        self.addCleanup(self.runtime.close)
        self.runtime.start()
        self.token = "local-test-token-" + "a" * 40
        self.server = OperatorServer(("127.0.0.1", 0), self.runtime, self.token)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.)

    def download(self, *, auth: bool):
        conn = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        headers = {"Authorization": "Bearer " + self.token} if auth else {}
        conn.request("GET", "/api/diagnostics.zip", headers=headers)
        response = conn.getresponse()
        code = response.status
        data = response.read()
        headers = dict(response.getheaders())
        conn.close()
        return code, data, headers

    def test_latched_safe_diagnostics_download_does_not_arm(self):
        self.runtime.process_detection(DetectionResult(
            found=False, frame_width=640, frame_height=480,
            protocol_version=2, frame_seq=10, inference_ms=12.5), 55.)
        self.assertEqual(self.download(auth=False)[0], 401)
        code, data, headers = self.download(auth=True)
        self.assertEqual(code, 200)
        self.assertEqual(headers["Content-Type"], "application/zip")
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            status = json.loads(archive.read("status.json"))
            self.assertEqual(status["schema_version"], 1)
            self.assertEqual(status["events_count"], 1)
            self.assertEqual(status["status"]["mode"], "SAFE")
            self.assertIn(b'"frame_seq": 10', archive.read("decisions.jsonl"))
        self.assertNotEqual(self.runtime.status()["mode"], "AUTO")


if __name__ == "__main__":
    unittest.main()
