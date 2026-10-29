"""Static regression checks for Raspberry Pi and Windows startup scripts."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]
PI = ROOT / "scripts" / "start_pi_follow.sh"
WIN = ROOT / "scripts" / "start_windows_follow.ps1"


class StartupScriptTests(unittest.TestCase):
    def test_wifi_password_not_embedded(self):
        source = PI.read_text(encoding="utf-8")
        self.assertIn('SSID="obnizAP2"', source)
        self.assertIn("nmcli --ask --wait 30 device wifi connect", source)
        self.assertIn("connection.autoconnect yes", source)

    def test_low_voltage_blocks_before_serial_open(self):
        source = PI.read_text(encoding="utf-8")
        self.assertIn("vcgencmd get_throttled", source)
        self.assertIn("POWER_HEX & 0x5", source)
        self.assertLess(source.index("POWER_HEX & 0x5"),
                        source.index('exec "$PYTHON_BIN"'))

    def test_real_follow_flags_and_no_auto_arm(self):
        source = PI.read_text(encoding="utf-8")
        for argument in (
            "--pc-host 127.0.0.1", "--servo-power-on",
            "--enable-body-turns", "--pitch-calibrated",
            "--reverse-body-turn-direction", "--distance-adaptive-head",
            "--auto-recalibrate-brief-loss", "--preview-every-n-frames 3",
            "--physical-cutoff-tested",
        ):
            with self.subTest(argument=argument):
                self.assertIn(argument, source)
        self.assertNotIn("SAFETY_ARM", source)

    def test_windows_starts_gpu_and_dual_loopback_tunnel(self):
        source = WIN.read_text(encoding="utf-8")
        self.assertIn("pc_detector_preview_server.py", source)
        self.assertIn('"127.0.0.1:8765:127.0.0.1:8765"', source)
        self.assertIn('"127.0.0.1:8766:127.0.0.1:8766"', source)
        self.assertIn("bash scripts/start_pi_follow.sh", source)


if __name__ == "__main__":
    unittest.main()
