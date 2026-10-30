"""Offline Windows USB runner: syntax and safety-contract regression tests."""
from __future__ import annotations

import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]
PY_RUNNER = ROOT / "experiments" / "pc_gpu" / "windows_usb_follow.py"
PS_LAUNCHER = ROOT / "scripts" / "start_windows_usb_follow.ps1"


class WindowsUsbEntrypointTests(unittest.TestCase):
    def test_python_runner_compiles_without_camera_or_cuda(self):
        text = PY_RUNNER.read_text(encoding="utf-8")
        compile(text, str(PY_RUNNER), "exec")
        ast.parse(text)

    def test_monitor_mode_is_default_and_actuation_requires_flags(self):
        source = PY_RUNNER.read_text(encoding="utf-8")
        self.assertIn('default="monitor"', source)
        for flag in ("--bench-secured", "--neutral-calibrated",
                     "--physical-cutoff-tested", "--servo-power-on-confirmed",
                     "--pitch-calibrated", "--body-turn-calibrated"):
            self.assertIn(flag, source)
        self.assertIn("runtime.set_camera_state(False, False)", source)
        self.assertIn("tracking=True", source)
        self.assertIn("not Path(args.model).is_file()", source)

    def test_usb_launcher_no_ssh_and_defaults_to_monitor(self):
        source = PS_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn('[string]$Mode = "Monitor"', source)
        self.assertIn('"--serial-port", $EspPort', source)
        self.assertNotIn("ssh @", source.lower())
        self.assertNotIn("start_pi_follow.sh", source)
        self.assertIn("PhysicalCutoffTested", source)


if __name__ == "__main__":
    unittest.main()
