"""End-to-end regression: camera result -> presenter selection -> virtual ESP32."""
import contextlib
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pi_showtime_smoke import run_smoke


class FullStackSmokeTests(unittest.TestCase):
    def test_full_stack_virtual_control_and_stop(self):
        with contextlib.redirect_stdout(io.StringIO()):
            run_smoke()


if __name__ == "__main__":
    unittest.main()
