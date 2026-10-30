"""Prevent real pyserial allowlist errors from latching the ESP32 at startup.

FakeUART only: no physical port, servo, camera or motor access.
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from serial_command_sink import SerialCommandSink
from pi_real_head_integration import HeadOnlySerialSink, LockedRealRuntime


class FakePort:
    is_open = True

    def __init__(self):
        self.commands = []

    @property
    def in_waiting(self):
        return 0

    def write(self, data):
        self.commands.append(data)
        return len(data)

    def close(self):
        self.is_open = False


class LedSerialRegressionTests(unittest.TestCase):
    def setUp(self):
        self.port = FakePort()

        def real_transport(fake_path):
            return SerialCommandSink(
                fake_path, serial_factory=lambda **kwargs: self.port)

        serial_patch = patch(
            "pi_real_head_integration.SerialCommandSink",
            side_effect=real_transport)
        serial_patch.start()
        self.addCleanup(serial_patch.stop)

    def test_red_and_yellow_use_the_real_serial_handle(self):
        sink = HeadOnlySerialSink("mock-port", head_enabled=False)
        self.addCleanup(sink.close)
        sink.send("RGB_RED")
        sink.send("RGB_YELLOW")
        sink.send("STATUS")
        self.assertEqual(self.port.commands, [
            b"RGB_RED\n", b"RGB_YELLOW\n", b"STATUS\n"])

    def test_pi_cannot_spoof_firmware_white(self):
        sink = HeadOnlySerialSink("mock-port", head_enabled=True)
        self.addCleanup(sink.close)
        for command in ("RGB_WHITE", "RGB_BLUE", "RGB_OFF", "RGB_ERROR"):
            with self.assertRaises(ValueError):
                sink.send(command)
        self.assertEqual(self.port.commands, [])

    def test_startup_led_does_not_raise_uart_poll_valueerror(self):
        runtime = LockedRealRuntime("mock-port", head_enabled=True)
        self.addCleanup(runtime.close)
        runtime.start()
        time.sleep(.20)
        status = runtime.status()
        self.assertEqual(status["mode"], "SAFE")
        self.assertIsNone(status["uart_poll_error"])
        self.assertIn(b"RGB_RED\n", self.port.commands)
        self.assertNotIn(b"SAFETY_ESTOP\n", self.port.commands)


if __name__ == "__main__":
    unittest.main()
