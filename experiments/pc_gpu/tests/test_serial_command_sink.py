"""Unit tests for serial transport with fake UART, no actual device."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from serial_command_sink import SerialCommandSink


class FakeUart:
    is_open = True

    def __init__(self, **kwargs):
        self.settings = kwargs
        self.writes = []
        self.short_write = False

    def write(self, payload):
        self.writes.append(payload)
        return len(payload) - 1 if self.short_write else len(payload)

    def close(self):
        self.is_open = False


class SerialCommandTests(unittest.TestCase):
    def make(self):
        uart = FakeUart()
        sink = SerialCommandSink(
            "/dev/ttyFAKE", serial_factory=lambda **kwargs: uart
        )
        self.addCleanup(sink.close)
        return sink, uart

    def test_stop_newline_and_utf8_safe_ascii(self):
        sink, uart = self.make()
        sink.send(" stop ")
        self.assertEqual(uart.writes, [b"STOP\n"])

    def test_turn_command(self):
        sink, uart = self.make()
        sink.send("track_left")
        self.assertEqual(uart.writes, [b"TRACK_LEFT\n"])

    def test_reject_head_motion_or_arbitrary_commands(self):
        sink, uart = self.make()
        with self.assertRaises(ValueError):
            sink.send("HEAD_POSE_SET:90,90")
        with self.assertRaises(ValueError):
            sink.send("STAND")
        self.assertEqual(uart.writes, [])

    def test_incomplete_write_is_failure(self):
        sink, uart = self.make()
        uart.short_write = True
        with self.assertRaises(IOError):
            sink.send("STOP")

    def test_closed_uart_is_failure(self):
        sink, uart = self.make()
        uart.close()
        with self.assertRaises(ConnectionError):
            sink.send("STOP")

    def test_open_config(self):
        recorded = {}
        def factory(**kwargs):
            recorded.update(kwargs)
            return FakeUart()
        sink = SerialCommandSink("/dev/ttyFAKE", serial_factory=factory)
        sink.close()
        self.assertEqual(recorded["baudrate"], 115200)
        self.assertEqual(recorded["write_timeout"], 0.15)

    def test_requires_port(self):
        with self.assertRaises(ValueError):
            SerialCommandSink("", serial_factory=FakeUart)


if __name__ == "__main__":
    unittest.main()
