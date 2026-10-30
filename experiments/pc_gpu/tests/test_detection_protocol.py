"""Pure-stdlib regression tests. No camera, model download or ESP32 needed."""
from __future__ import annotations

import json
import socket
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import (
    DetectionResult,
    recv_json,
    recv_message,
    send_json,
    send_message,
)


class DetectionProtocolTests(unittest.TestCase):
    def setUp(self):
        self.sender, self.receiver = socket.socketpair()
        self.addCleanup(self.sender.close)
        self.addCleanup(self.receiver.close)
        self.sender.settimeout(1)
        self.receiver.settimeout(1)

    def test_detection_result_round_trip(self):
        result = DetectionResult(
            found=True, center_x=340, center_y=220,
            width=120, height=310, confidence=0.91,
            class_id=0, class_name="person",
            frame_width=640, frame_height=480,
        )
        send_json(self.sender, result.to_dict())
        received = DetectionResult.from_dict(recv_json(self.receiver))
        self.assertEqual(result, received)

    def test_no_target_result_round_trip(self):
        no_target = DetectionResult(found=False, frame_width=640, frame_height=480)
        send_json(self.sender, no_target.to_dict())
        received = DetectionResult.from_dict(recv_json(self.receiver))
        self.assertFalse(received.found)
        self.assertEqual(received.center_x, 0)

    def test_future_extra_field_is_ignored(self):
        result = DetectionResult.from_dict({"found": True, "center_x": 1, "new_field": 99})
        self.assertTrue(result.found)
        self.assertEqual(result.center_x, 1)

    def test_binary_message_round_trip(self):
        blob = b"\x00\xff\x80hello\n"
        send_message(self.sender, blob)
        self.assertEqual(recv_message(self.receiver), blob)

    def test_truncated_payload_raises(self):
        self.sender.sendall(struct.pack("!I", 10) + b"a")
        self.sender.shutdown(socket.SHUT_WR)
        with self.assertRaises(ConnectionError):
            recv_message(self.receiver)

    def test_oversize_payload_is_not_sent(self):
        with self.assertRaises(ValueError):
            send_message(self.sender, b"x" * (8 * 1024 * 1024 + 1))

    def test_oversize_announced_payload_is_rejected(self):
        self.sender.sendall(struct.pack("!I", 8 * 1024 * 1024 + 1))
        with self.assertRaises(ValueError):
            recv_message(self.receiver)

    def test_invalid_json_is_rejected(self):
        send_message(self.sender, b"not-json")
        with self.assertRaises(json.JSONDecodeError):
            recv_json(self.receiver)


if __name__ == "__main__":
    unittest.main()
