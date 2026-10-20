"""Phase 9 full in-memory integration: UI API ↔ Pi safety ↔ V8.8.3 ↔ fake ESP32.

No camera, UART, web network outside 127.0.0.1, GPIO or physical motors.
"""
from __future__ import annotations

import http.client
import json
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import DetectionResult
from pi_integrated_simulation import IntegratedSimulationRuntime
from pi_operator_console import OperatorServer
from pi_deployment_launcher import run
from virtual_esp32 import VirtualEsp32


def person(x=270, found=True):
    return DetectionResult(
        found=found, center_x=x, center_y=115, width=75, height=120,
        confidence=0.90, class_id=0, class_name="person",
        frame_width=320, frame_height=240, inference_ms=45.0
    )


def wait_for(predicate, timeout=1.2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("timed out waiting for expected virtual device state")


class VirtualFirmwareTests(unittest.TestCase):
    def test_start_disarmed_and_reject_motion(self):
        dev = VirtualEsp32()
        dev.send("TRACK_LEFT")
        self.assertEqual(dev.state, "DISARMED")
        self.assertEqual(dev.walking_state, "IDLE")
        self.assertIn(b"MOTION_REJECTED=TRACK_LEFT", dev.read_available())

    def test_arm_needs_heartbeat_and_times_out(self):
        time_value = [10.0]
        dev = VirtualEsp32(clock=lambda: time_value[0])
        dev.send("SAFETY_ARM")
        self.assertEqual(dev.state, "ARMED")
        time_value[0] = 10.2
        dev.send("SAFETY_PING")
        time_value[0] = 10.951
        dev.read_available()
        self.assertEqual(dev.state, "LATCHED")
        dev.send("SAFETY_PING")
        dev.send("TRACK_RIGHT")
        self.assertEqual(dev.walking_state, "IDLE")
        self.assertIn(b"MOTION_REJECTED=TRACK_RIGHT", dev.read_available())

    def test_explicit_reset_never_rearms(self):
        dev = VirtualEsp32()
        dev.send("SAFETY_ESTOP")
        self.assertEqual(dev.state, "LATCHED")
        dev.send("SAFETY_RESET")
        self.assertEqual(dev.state, "DISARMED")
        dev.send("TRACK_LEFT")
        self.assertEqual(dev.walking_state, "IDLE")

    def test_status_reports_head_pose_and_busy(self):
        dev = VirtualEsp32(motion_duration_s=100.0)
        dev.send("SAFETY_ARM")
        dev.send("HEAD_POSE_SET:9.0,3.0")
        dev.send("TRACK_RIGHT")
        dev.send("STATUS")
        data = dev.read_available()
        self.assertIn(b"Walking State: TRACK_RIGHT", data)
        self.assertIn(b"Busy: YES", data)
        self.assertIn(b"currentYaw=9.0", data)

    def test_disconnect_blocks_transmission(self):
        dev = VirtualEsp32()
        dev.disconnect()
        with self.assertRaises(OSError):
            dev.send("STOP")
        with self.assertRaises(OSError):
            dev.read_available()


class IntegratedRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.runtime = IntegratedSimulationRuntime()
        self.runtime.start()
        self.addCleanup(self.runtime.close)

    def prime(self, x=270):
        for _ in range(3):
            self.runtime.process_detection(person(x), 70.0)

    def arm(self, which="auto", x=270):
        if which == "auto":
            self.prime(x)
        reply = self.runtime.action(which)
        self.assertTrue(reply["ok"], reply)
        wait_for(lambda: self.runtime.status()["mode"] == which.upper())

    def test_all_actuators_are_virtual_by_construction(self):
        import inspect
        import pi_integrated_simulation
        self.assertIsInstance(self.runtime.sink, VirtualEsp32)
        self.assertTrue(self.runtime.status()["simulation"])
        self.assertTrue(self.runtime.status()["simulated_esp32"])
        self.assertNotIn("SerialCommandSink(", inspect.getsource(pi_integrated_simulation))
        self.assertNotIn("from serial_command_sink import", inspect.getsource(pi_integrated_simulation))
        self.assertEqual(self.runtime.status()["fw_safety"].split(":")[0], "VIRTUAL")

    def test_initial_safe_and_firmware_disarmed(self):
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.runtime.sink.state, "DISARMED")
        self.assertNotIn("TRACK_LEFT", self.runtime.sink.commands)

    def test_auto_arm_needs_person(self):
        self.assertFalse(self.runtime.action("auto")["ok"])
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertNotIn("SAFETY_ARM", self.runtime.sink.commands)

    def test_auto_requires_virtual_firmware_ack(self):
        self.prime()
        response = self.runtime.action("auto")
        self.assertTrue(response["pending"])
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        wait_for(lambda: self.runtime.status()["mode"] == "AUTO")
        self.assertIn("SAFETY_ARM", self.runtime.sink.commands)

    def test_head_follow_goes_through_gateway(self):
        self.arm("auto", x=160)
        wait_for(lambda: self.runtime.controller.telemetry.snapshot().head_fresh)
        self.runtime.process_detection(person(160), 70)
        wait_for(lambda: any(x.startswith("HEAD_POSE_SET:") for x in self.runtime.sink.commands))
        self.assertEqual(self.runtime.status()["mode"], "AUTO")

    def test_head_only_target_loss_sends_stop(self):
        self.arm("auto", x=160)
        wait_for(lambda: self.runtime.controller.telemetry.snapshot().head_fresh)
        self.runtime.process_detection(person(160), 70)
        wait_for(lambda: any(x.startswith("HEAD_POSE_SET:") for x in self.runtime.sink.commands))
        before = len(self.runtime.sink.commands)
        self.runtime.process_detection(person(found=False), 70)
        self.assertIn("STOP", self.runtime.sink.commands[before:])
        self.assertEqual(self.runtime.status()["mode"], "SAFE")

    def test_automatic_body_turn_goes_through_single_gateway(self):
        self.arm("auto")
        wait_for(lambda: self.runtime.controller.telemetry.snapshot().body_idle_confirmed)
        self.runtime.process_detection(person(270), 70)
        self.runtime.process_detection(person(270), 70)
        self.assertIn("TRACK_LEFT", self.runtime.sink.commands)
        self.assertFalse(any(x.startswith("TRACK_FORWARD") for x in self.runtime.sink.commands))

    def test_manual_turn_and_deadman_stop(self):
        self.arm("manual")
        result = self.runtime.manual("left")
        self.assertTrue(result["ok"])
        self.assertIn("TRACK_LEFT", self.runtime.sink.commands)
        wait_for(lambda: self.runtime.status()["mode"] == "SAFE", timeout=1.0)
        self.assertEqual(self.runtime.sink.commands[-1], "STOP")

    def test_gpu_link_loss_ends_auto(self):
        self.arm("auto")
        self.runtime.set_camera_state(False, False)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertEqual(self.runtime.sink.state, "DISARMED")

    def test_manual_is_independent_of_gpu_link(self):
        self.arm("manual")
        self.runtime.set_camera_state(False, False)
        self.assertEqual(self.runtime.status()["mode"], "MANUAL")
        self.assertTrue(self.runtime.manual("right")["ok"])

    def test_estop_never_automatically_rearms(self):
        self.arm("manual")
        self.assertTrue(self.runtime.action("estop")["ok"])
        self.assertEqual(self.runtime.status()["mode"], "ESTOP")
        self.assertEqual(self.runtime.sink.state, "LATCHED")
        self.assertFalse(self.runtime.action("manual")["ok"])
        self.assertTrue(self.runtime.action("reset")["pending"])
        wait_for(lambda: self.runtime.status()["mode"] == "SAFE")
        self.assertEqual(self.runtime.sink.state, "DISARMED")
        self.assertEqual(self.runtime.status()["mode"], "SAFE")
        self.assertFalse(self.runtime.manual("left")["ok"])

    def test_virtual_uart_disconnect_latches_estop(self):
        self.arm("manual")
        self.runtime.sink.disconnect()
        wait_for(lambda: self.runtime.status()["mode"] == "ESTOP")
        self.assertFalse(self.runtime.action("manual")["ok"])

    def test_no_distance_translation(self):
        self.arm("manual")
        result = self.runtime.manual("forward")
        self.assertFalse(result["ok"])
        self.assertNotIn("TRACK_FORWARD", self.runtime.sink.commands)


class IntegratedApiTests(unittest.TestCase):
    def setUp(self):
        import threading
        self.runtime = IntegratedSimulationRuntime()
        self.runtime.start()
        self.token = "phase9-test-" + "x" * 32
        self.server = OperatorServer(("127.0.0.1", 0), self.runtime, self.token)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)
        self.runtime.close()

    def request(self, method, path, obj=None, *, authorized=True):
        conn = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2.0)
        headers = {"Content-Type": "application/json"}
        if authorized:
            headers["Authorization"] = "Bearer " + self.token
        conn.request(method, path, body=json.dumps(obj) if obj is not None else None, headers=headers)
        resp = conn.getresponse()
        code, data = resp.status, json.loads(resp.read().decode())
        conn.close()
        return code, data

    def test_local_status_exposes_virtual_firmware(self):
        code, payload = self.request("GET", "/api/status")
        self.assertEqual(code, 200)
        self.assertTrue(payload["simulated_esp32"])
        self.assertTrue(payload["simulation"])
        self.assertTrue(payload["fw_safety"].startswith("VIRTUAL:"))

    def test_unauthenticated_estop_api_rejected(self):
        code, body = self.request("POST", "/api/action", {"action": "estop"}, authorized=False)
        self.assertEqual(code, 401)
        self.assertEqual(self.runtime.status()["mode"], "SAFE")

    def test_http_manual_switch_and_estop(self):
        code, body = self.request("POST", "/api/action", {"action": "manual"})
        self.assertEqual(code, 200)
        self.assertTrue(body["pending"])
        wait_for(lambda: self.runtime.status()["mode"] == "MANUAL")
        code, body = self.request("POST", "/api/manual", {"direction": "left"})
        self.assertEqual(code, 200)
        self.assertIn("TRACK_LEFT", self.runtime.sink.commands)
        self.assertEqual(self.request("POST", "/api/action", {"action": "estop"})[0], 200)
        self.assertEqual(self.runtime.status()["mode"], "ESTOP")

    def test_http_auto_denied_before_detector_frames(self):
        code, data = self.request("POST", "/api/action", {"action": "auto"})
        self.assertEqual(code, 409)
        self.assertEqual(data["error"], "ARM_NOT_READY")


class IntegratedLauncherTests(unittest.TestCase):
    def test_cli_integrated_cannot_take_serial_options(self):
        self.assertEqual(run(["--stage", "integrated", "--serial-port", "/dev/ttyUSB0"]), 2)

    def test_cli_integrated_routes_to_virtual_only(self):
        with patch("pi_integrated_simulation.main") as fake:
            self.assertEqual(run(["--stage", "integrated", "--pc-host", "192.0.2.1"]), 0)
            fake.assert_called_once_with(["--port", "8766", "--pc-host", "192.0.2.1"])

    def test_cli_has_no_hardware_mode(self):
        with self.assertRaises(SystemExit):
            run(["--stage", "hardware"])


if __name__ == "__main__":
    unittest.main()
