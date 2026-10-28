"""Phase 7 loopback dashboard tests. No Pi, camera, ESP32 or GPU needed."""
from __future__ import annotations

import http.client
import json
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import DetectionResult
from pi_operator_console import OperatorRuntime, OperatorServer


class OperatorConsoleTests(unittest.TestCase):
    def setUp(self):
        self.runtime = OperatorRuntime()
        self.runtime.start()
        self.token = "test-token-" + "c" * 32
        self.server = OperatorServer(("127.0.0.1", 0), self.runtime, self.token)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.runtime.close()

    def request(self, method, path, body=None, *, token=None, origin=None, host=None, content_type="application/json"):
        conn = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        headers = {}
        if token is not None:
            headers["Authorization"] = "Bearer " + token
        if origin is not None:
            headers["Origin"] = origin
        if host is not None:
            headers["Host"] = host
        if body is not None:
            headers["Content-Type"] = content_type
            if not isinstance(body, bytes):
                body = json.dumps(body).encode("utf-8")
        conn.request(method, path, body=body, headers=headers)
        response = conn.getresponse()
        status, payload = response.status, response.read()
        hdr = dict(response.getheaders())
        conn.close()
        return status, payload, hdr

    def post(self, path, obj, **kw):
        return self.request("POST", path, obj, token=self.token, **kw)

    def test_loopback_only_binding(self):
        with self.assertRaises(ValueError):
            OperatorServer(("0.0.0.0", 0), self.runtime, self.token)

    def test_random_token_strength_enforced(self):
        with self.assertRaises(ValueError):
            OperatorServer(("127.0.0.1", 0), self.runtime, "short")

    def test_static_html_available_without_token(self):
        code, body, hdr = self.request("GET", "/")
        self.assertEqual(code, 200)
        self.assertIn(b"Presentation Escort", body)
        self.assertIn(b'id="runtime-subtitle"', body)
        self.assertIn(b'id="motor"', body)
        self.assertEqual(hdr["Cache-Control"], "no-store")
        self.assertIn("default-src 'none'", hdr["Content-Security-Policy"])

    def test_api_status_denied_without_token(self):
        self.assertEqual(self.request("GET", "/api/status")[0], 401)
        self.assertEqual(self.request("GET", "/api/status", token="wrong")[0], 401)

    def test_api_status_requires_exact_bearer(self):
        code, body, _ = self.request("GET", "/api/status", token=self.token)
        self.assertEqual(code, 200)
        state = json.loads(body)
        self.assertEqual(state["mode"], "SAFE")
        self.assertTrue(state["simulation"])
        self.assertEqual(state["fw_safety"], "NOT_REQUIRED")

    def test_host_header_rebinding_rejected(self):
        self.assertEqual(self.request("GET", "/api/status", token=self.token, host="bad.test")[0], 403)

    def test_cross_origin_post_rejected(self):
        code, _, _ = self.post("/api/action", {"action": "manual"}, origin="http://evil.test")
        self.assertEqual(code, 403)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")

    def test_valid_local_origin_allowed(self):
        origin = f"http://127.0.0.1:{self.server.server_port}"
        code, body, _ = self.post("/api/action", {"action": "manual"}, origin=origin)
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(body)["ok"])

    def test_reject_unauthorized_mode_change(self):
        code, _, _ = self.request("POST", "/api/action", {"action":"manual"}, token="wrong")
        self.assertEqual(code, 401)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")

    def test_auto_refused_without_detection(self):
        code, body, _ = self.post("/api/action", {"action":"auto"})
        self.assertEqual(code, 409)
        self.assertEqual(json.loads(body)["error"], "TARGET_NOT_READY")

    def test_manual_left_and_right_simulated_only(self):
        self.assertEqual(self.post("/api/action", {"action":"manual"})[0], 200)
        self.assertEqual(self.post("/api/manual", {"direction":"left"})[0], 200)
        self.assertEqual(self.post("/api/manual", {"direction":"right"})[0], 200)
        self.assertEqual(self.runtime.sink.commands[-2:], ["TRACK_LEFT", "TRACK_RIGHT"])

    def test_forward_backward_rejected(self):
        self.post("/api/action", {"action":"manual"})
        code, _, _ = self.post("/api/manual", {"direction":"forward"})
        self.assertEqual(code, 409)
        self.assertNotIn("TRACK_FORWARD", self.runtime.sink.commands)

    def test_manual_button_release_stops(self):
        self.post("/api/action", {"action":"manual"})
        self.post("/api/manual", {"direction":"left"})
        code, _, _ = self.post("/api/manual", {"direction":"stop"})
        self.assertEqual(code, 200)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.runtime.sink.commands[-1], "STOP")

    def test_emergency_stop_latched(self):
        self.post("/api/action", {"action":"manual"})
        self.post("/api/manual", {"direction":"left"})
        self.assertEqual(self.post("/api/action", {"action":"estop"})[0], 200)
        self.assertEqual(self.runtime.status()["mode"], "ESTOP")
        self.assertEqual(self.post("/api/action", {"action":"manual"})[0], 409)
        self.assertEqual(self.runtime.status()["last_command"], "STOP")
        self.assertEqual(self.post("/api/action", {"action":"reset"})[0], 200)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")

    def test_invalid_json_and_content_type(self):
        code, _, _ = self.request("POST", "/api/action", b"{bad", token=self.token)
        self.assertEqual(code, 400)
        code, _, _ = self.request("POST", "/api/action", b'{"action":"manual"}', token=self.token, content_type="text/plain")
        self.assertEqual(code, 415)

    def test_extra_fields_rejected(self):
        code, _, _ = self.post("/api/action", {"action":"manual","foo":"bar"})
        self.assertEqual(code, 400)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")

    def test_unknown_actions_rejected(self):
        self.assertEqual(self.post("/api/action", {"action":"flash_esp32"})[0], 409)
        self.assertEqual(self.post("/api/manual", {"direction":"head_center"})[0], 409)

    def test_valid_detection_enables_simulated_auto(self):
        result = DetectionResult(
            found=True, center_x=170, center_y=90,
            width=80, height=120, confidence=.9,
            class_id=0, class_name="person",
            frame_width=320, frame_height=240,
        )
        for _ in range(3):
            self.runtime.process_detection(result, 75.)
        self.assertTrue(self.runtime.status()["auto_ready"])
        self.assertEqual(self.post("/api/action", {"action":"auto"})[0], 200)
        self.assertEqual(self.runtime.status()["mode"], "AUTO")
        self.assertFalse(self.runtime.controller.enable_actuation)
        self.assertFalse(any(x.startswith("HEAD_POSE_SET") for x in self.runtime.sink.commands))

    def test_auto_disarmed_on_camera_disconnect(self):
        result = DetectionResult(
            found=True, center_x=160, center_y=100,
            width=80, height=100, confidence=.9,
            class_id=0, class_name="person", frame_width=320,
            frame_height=240,
        )
        for _ in range(3):
            self.runtime.process_detection(result, 60)
        self.runtime.action("auto")
        self.runtime.set_camera_state(False, False)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")

    def test_manual_deadman_without_held_button(self):
        self.runtime.action("manual")
        self.runtime.manual("left")
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and self.runtime.status()["mode"] != "SAFE":
            time.sleep(0.02)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.runtime.sink.commands[-1], "STOP")

    def test_no_external_control_surface(self):
        text = (Path(__file__).resolve().parents[1] / "pi_operator_console.py").read_text()
        self.assertNotIn("SerialCommandSink(", text)
        self.assertNotIn("0.0.0.0", text.replace('("0.0.0.0", 0)', ""))  # no all-interface binding


if __name__ == "__main__":
    unittest.main()
