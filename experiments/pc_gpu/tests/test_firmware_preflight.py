"""Stage 8 deployment-preflight regression tests: no actual serial devices."""
from __future__ import annotations

import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from firmware_preflight import FirmwarePreflight
from pi_deployment_launcher import diagnostic_probe, repository_audit, run, parse_args


HEADER = b"[Safety] Protocol=V1 TimeoutMs=750\n"
DISARMED = b"[Safety] State: DISARMED\n"


class FakeSink:
    def __init__(self, responses=()):
        self.responses = list(responses)
        self.commands = []

    def send(self, command):
        self.commands.append(command)

    def read_available(self, *, max_bytes=4096):
        return self.responses.pop(0) if self.responses else b""


class FirmwarePreflightTests(unittest.TestCase):
    def test_empty_state_rejected(self):
        s = FirmwarePreflight().status()
        self.assertFalse(s.safe_for_diagnostics)
        self.assertEqual(s.reason, "FIRMWARE_VERSION_UNCONFIRMED")

    def test_correct_version_and_disarmed_required(self):
        s = FirmwarePreflight()
        s.feed(HEADER)
        self.assertFalse(s.status().safe_for_diagnostics)
        s.feed(DISARMED)
        self.assertTrue(s.status().safe_for_diagnostics)

    def test_state_alone_is_never_enough(self):
        s = FirmwarePreflight()
        s.feed(DISARMED)
        self.assertFalse(s.status().safe_for_diagnostics)

    def test_split_lines(self):
        s = FirmwarePreflight()
        s.feed(HEADER[:12])
        self.assertFalse(s.status().safe_for_diagnostics)
        s.feed(HEADER[12:] + DISARMED[:14])
        self.assertFalse(s.status().safe_for_diagnostics)
        s.feed(DISARMED[14:])
        self.assertTrue(s.status().safe_for_diagnostics)

    def test_wrong_protocol_never_allowed_even_after_good_version(self):
        s = FirmwarePreflight()
        s.feed(HEADER + DISARMED)
        self.assertTrue(s.status().safe_for_diagnostics)
        s.feed(b"[Safety] Protocol=V2 TimeoutMs=750\n")
        self.assertEqual(s.status().reason, "FIRMWARE_PROTOCOL_MISMATCH")

    def test_armed_state_blocks_diagnostic(self):
        s = FirmwarePreflight()
        s.feed(HEADER + b"[Safety] State: ARMED\n")
        self.assertFalse(s.status().safe_for_diagnostics)
        self.assertEqual(s.status().reason, "FIRMWARE_NOT_DISARMED")

    def test_latched_state_blocks_diagnostic(self):
        s = FirmwarePreflight()
        s.feed(HEADER + b"[Safety] State: LATCHED\n")
        self.assertFalse(s.status().safe_for_diagnostics)

    def test_malformed_state_not_accepted(self):
        s = FirmwarePreflight()
        s.feed(HEADER + b"[Safety] State: DISARMED_NOW\n")
        self.assertFalse(s.status().safe_for_diagnostics)

    def test_noise_does_not_count_as_header(self):
        s = FirmwarePreflight()
        s.feed(b"x[Safety] Protocol=V1 TimeoutMs=750\n" + DISARMED)
        self.assertFalse(s.status().safe_for_diagnostics)

    def test_bounded_fragment_buffer(self):
        s = FirmwarePreflight()
        s.feed(b"x" * 50000)
        self.assertLessEqual(len(s._pending), 256)


class DeploymentLauncherTests(unittest.TestCase):
    def test_repository_safety_audit_is_complete(self):
        result = repository_audit()
        self.assertTrue(all(result.values()), result)

    def test_empty_or_wrong_checkout_fails_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            result = repository_audit(Path(folder))
        self.assertFalse(all(result.values()))

    def test_default_stage_is_offline_audit(self):
        self.assertEqual(parse_args([]).stage, "audit")
        with io.StringIO() as buf, redirect_stdout(buf):
            self.assertEqual(run([]), 0)
            self.assertIn("No camera, serial port, or motors", buf.getvalue())

    def test_diagnostic_requires_explicit_power_off_confirmation(self):
        with io.StringIO() as buf, redirect_stdout(buf):
            self.assertEqual(run(["--stage", "diagnostic", "--serial-port", "/dev/ttyUSB0"]), 2)
            self.assertIn("--servo-power-off-confirmed", buf.getvalue())

    def test_serial_flags_not_allowed_in_console(self):
        with io.StringIO() as buf, redirect_stdout(buf):
            self.assertEqual(run(["--stage", "console", "--serial-port", "/dev/ttyUSB0"]), 2)
            self.assertIn("forbidden", buf.getvalue())

    def test_no_hidden_hardware_mode(self):
        with self.assertRaises(SystemExit) as e:
            parse_args(["--stage", "hardware"])
        self.assertNotEqual(e.exception.code, 0)

    def test_probe_sends_only_version_and_status(self):
        sink = FakeSink([HEADER[:15], HEADER[15:], DISARMED])
        with patch("pi_deployment_launcher.time.sleep"):
            ok, reason = diagnostic_probe(sink, response_timeout_s=0.5)
        self.assertTrue(ok, reason)
        self.assertEqual(sink.commands, ["SAFETY_VERSION", "SAFETY_STATUS"])
        self.assertFalse(any("ARM" in x or "TRACK" in x or "HEAD_POSE_SET" in x
                             for x in sink.commands))

    def test_probe_rejects_legacy_firmware(self):
        sink = FakeSink([b"Presentation Escort ESP32\nV8.8 DUAL LOOP\n", DISARMED])
        with patch("pi_deployment_launcher.time.sleep"):
            ok, reason = diagnostic_probe(sink, response_timeout_s=0.5)
        self.assertFalse(ok)
        self.assertEqual(reason, "FIRMWARE_VERSION_UNCONFIRMED")
        self.assertEqual(sink.commands, ["SAFETY_VERSION", "SAFETY_STATUS"])

    def test_probe_rejects_armed_state(self):
        sink = FakeSink([HEADER + b"[Safety] State: ARMED\n"])
        with patch("pi_deployment_launcher.time.sleep"):
            ok, reason = diagnostic_probe(sink, response_timeout_s=0.5)
        self.assertFalse(ok)
        self.assertEqual(reason, "FIRMWARE_NOT_DISARMED")

    def test_probe_refuses_dangerously_short_boot_wait(self):
        with self.assertRaises(ValueError):
            diagnostic_probe(FakeSink(), reset_wait_s=0.1)

    def test_pi_console_remains_memory_only(self):
        original = (Path(__file__).resolve().parents[1] / "pi_operator_console.py").read_text()
        self.assertIn("MemoryCommandSink()", original)
        self.assertNotIn("SerialCommandSink(", original)


if __name__ == "__main__":
    unittest.main()
