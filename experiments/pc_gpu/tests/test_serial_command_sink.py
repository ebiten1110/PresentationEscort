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
        self.received = b""

    def write(self, payload):
        self.writes.append(payload)
        return len(payload) - 1 if self.short_write else len(payload)

    @property
    def in_waiting(self):
        return len(self.received)

    def read(self, size):
        data = self.received[:size]
        self.received = self.received[size:]
        return data

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

    def test_safe_head_pose_accepted(self):
        sink, uart = self.make()
        sink.send("HEAD_POSE_SET:-20.0,12.0")
        self.assertEqual(uart.writes[-1], b"HEAD_POSE_SET:-20.0,12.0\n")

    def test_head_outside_range_denied(self):
        sink, uart = self.make()
        for command in ("HEAD_POSE_SET:30,0", "HEAD_POSE_SET:0,13",
                        "HEAD_POSE_SET:NAN,0", "HEAD_POSE_SET:1,Infinity"):
            with self.assertRaises(ValueError):
                sink.send(command)
        self.assertEqual(uart.writes, [])

    def test_status_and_head_status_allowed(self):
        sink, uart = self.make()
        sink.send("STATUS")
        sink.send("HEAD_STATUS")
        self.assertEqual(uart.writes, [b"STATUS\n", b"HEAD_STATUS\n"])

    def test_bounded_nonblocking_uart_read(self):
        sink, uart = self.make()
        self.assertEqual(sink.read_available(), b"")
        uart.received = b"123456"
        self.assertEqual(sink.read_available(max_bytes=3), b"123")
        self.assertEqual(sink.read_available(), b"456")
        self.assertEqual(sink.read_available(), b"")

    def test_safety_handshake_commands_allowed(self):
        sink, uart = self.make()
        for command in (
            "SAFETY_ARM", "SAFETY_PING", "SAFETY_STATUS",
            "SAFETY_DISARM", "SAFETY_ESTOP", "SAFETY_RESET",
        ):
            sink.send(command)
        self.assertEqual(uart.writes, [
            b"SAFETY_ARM\n", b"SAFETY_PING\n", b"SAFETY_STATUS\n",
            b"SAFETY_DISARM\n", b"SAFETY_ESTOP\n", b"SAFETY_RESET\n",
        ])

    def test_firmware_version_query_is_read_only(self):
        sink, uart = self.make()
        sink.send("SAFETY_VERSION")
        self.assertEqual(uart.writes[-1], b"SAFETY_VERSION\n")

    def test_linux_serial_port_exclusive(self):
        import sys
        recorded = {}
        def factory(**kwargs):
            recorded.update(kwargs)
            return FakeUart()
        sink = SerialCommandSink("/dev/ttyFAKE", serial_factory=factory)
        sink.close()
        if sys.platform.startswith("linux"):
            self.assertIs(recorded["exclusive"], True)
        else:
            self.assertNotIn("exclusive", recorded)

    def test_requires_port(self):
        with self.assertRaises(ValueError):
            SerialCommandSink("", serial_factory=FakeUart)


if __name__ == "__main__":
    unittest.main()
