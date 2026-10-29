"""Phase 11: multi-person presenter selection and no-handoff regressions.

Software-only. No GPU, webcam, ESP32 serial or motor use.
"""
from __future__ import annotations
import http.client
import json
from dataclasses import replace
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import DetectionResult
from presenter_lock import PresenterLock
from pi_operator_console import OperatorServer
from pi_presenter_lock_demo import LockedSimulationRuntime


def candidate(x, y=30, w=72, h=170, conf=.8):
    return {"x1": x, "y1": y, "x2": x+w, "y2": y+h, "confidence": conf}


def frame(seq=1, boxes=None, *, version=2):
    if boxes is None:
        boxes = [candidate(20)]
    return DetectionResult(
        found=bool(boxes), center_x=60, center_y=115,
        width=72, height=170, confidence=.88, class_id=0,
        class_name="person", frame_width=320, frame_height=240,
        protocol_version=version, frame_seq=seq, people=boxes,
    )


def await_mode(run, expected, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if run.status()["mode"] == expected:
            return
        time.sleep(.02)
    raise AssertionError("mode did not become " + expected)


class PresenterLockTests(unittest.TestCase):
    def setUp(self):
        self.lock = PresenterLock()

    def acquire(self, boxes=None):
        result = self.lock.observe(frame(1, boxes), now=10.)
        self.assertFalse(result.found)
        self.assertEqual(self.lock.select(0, 1, now=10.1), (True, "SELECTED"))

    def test_no_auto_acquisition(self):
        self.assertFalse(self.lock.observe(frame(), now=10).found)
        self.assertFalse(self.lock.locked)

    def test_operator_selection_then_next_frame(self):
        self.acquire()
        result = self.lock.observe(frame(2), now=10.2)
        self.assertTrue(result.found)
        self.assertEqual(result.center_x, 56)
        self.assertEqual(self.lock.state, "TRACKING")

    def test_confidence_swap_never_switches_stranger(self):
        self.acquire([candidate(20,conf=.5), candidate(210,conf=.99)])
        selected = self.lock.observe(frame(2, [candidate(213,conf=.99), candidate(25,conf=.50)]), now=10.2)
        self.assertTrue(selected.found)
        self.assertEqual(selected.center_x, 61)

    def test_target_loss_does_not_auto_reacquire_other_person(self):
        self.acquire()
        self.assertFalse(self.lock.observe(frame(2, [candidate(215)]), now=10.1).found)
        self.assertFalse(self.lock.locked)
        self.assertFalse(self.lock.observe(frame(3, [candidate(215)]), now=10.2).found)
        self.assertEqual(self.lock.state, "LOST")

    def test_face_continuity_survives_same_person_box_height_jitter(self):
        self.lock = PresenterLock(gesture_enabled=True, gesture_confirm_frames=2)
        def face_frame(seq, height):
            data = frame(seq, [candidate(70, y=2, w=375, h=height, conf=.8)])
            return replace(data, frame_width=640, frame_height=480,
                gesture_indices=[0], aim_points=[{"x":245,"y":84,"source":"face"}])
        self.lock.observe(face_frame(1, 281), now=10.)
        self.lock.observe(face_frame(2, 281), now=10.2)
        self.assertEqual(self.lock.state, "TRACKING")
        result = self.lock.observe(face_frame(3, 476), now=10.4)
        self.assertTrue(result.found)
        self.assertEqual(self.lock.state, "TRACKING")

    def test_short_loss_single_overlapping_person_is_visual_only(self):
        self.lock = PresenterLock(gesture_enabled=True, gesture_confirm_frames=2,
                                  transient_recovery_s=1.5)
        def tracked(seq, boxes, gesture):
            data = frame(seq, boxes)
            return replace(data, frame_width=640, frame_height=480,
                           gesture_indices=[0] if gesture else [])
        initial = candidate(0, y=0, w=163, h=479, conf=.8)
        self.lock.observe(tracked(1, [initial], True), now=10.)
        self.lock.observe(tracked(2, [initial], True), now=10.2)
        event_id = self.lock.gesture_event_id
        self.assertFalse(self.lock.observe(tracked(3, [], False), now=10.4).found)
        self.assertEqual(self.lock.state, "TEMPORARILY_LOST")
        recovered = self.lock.observe(tracked(4, [candidate(8, y=0, w=361,
                                                  h=480, conf=.8)], False), now=10.6)
        self.assertTrue(recovered.found)
        self.assertEqual(self.lock.reason, "VISUAL_REACQUIRED_GESTURE_REQUIRED")
        self.assertEqual(self.lock.gesture_event_id, event_id)

        # Only the existing safety controller's confirmed in-AUTO recovery
        # may clear the reminder; it must not forge a fresh gesture event.
        self.lock.confirm_bounded_reacquisition()
        self.assertEqual(self.lock.reason, "AUTO_RECALIBRATED_SAME_TRACK")
        self.assertEqual(self.lock.gesture_event_id, event_id)

    def test_small_far_person_can_move_across_two_slow_camera_frames(self):
        self.acquire([candidate(70, w=19, h=55, conf=.9)])
        first = self.lock.observe(
            frame(2, [candidate(98, w=20, h=55, conf=.9)]), now=10.2)
        self.assertTrue(first.found)
        self.assertTrue(self.lock.locked)

    def test_far_person_does_not_handoff_to_distant_stranger(self):
        self.acquire([candidate(70, w=19, h=55, conf=.9)])
        result = self.lock.observe(
            frame(2, [candidate(220, w=20, h=55, conf=.9)]), now=10.2)
        self.assertFalse(result.found)
        self.assertEqual(self.lock.state, "LOST")

    def test_occlusion_and_crossing_immediate_loss(self):
        self.acquire()
        self.assertFalse(self.lock.observe(frame(2, [candidate(20), candidate(38)]), now=10.15).found)
        self.assertIn(self.lock.reason, {"AMBIGUOUS_NEARBY_PERSON", "TARGET_MISSING_OR_MULTIPLE_MATCHES"})
        self.assertFalse(self.lock.locked)

    def test_person_approaches_target_causes_stop(self):
        self.acquire()
        self.assertFalse(self.lock.observe(frame(2, [candidate(25), candidate(55)]), now=10.2).found)
        self.assertFalse(self.lock.locked)

    def test_target_can_move_modestly(self):
        self.acquire()
        self.assertTrue(self.lock.observe(frame(2, [candidate(34)]), now=10.2).found)
        self.assertTrue(self.lock.observe(frame(3, [candidate(50)]), now=10.4).found)

    def test_bad_bbox_payload_invalidates(self):
        self.acquire()
        self.assertFalse(self.lock.observe(frame(2, [{"x1": False, "x2": 90, "y1": 0, "y2": 200, "confidence": .8}]), now=10.1).found)
        self.assertFalse(self.lock.locked)

    def test_missing_v2_people_invalidates(self):
        self.acquire()
        self.assertFalse(self.lock.observe(frame(2, version=1), now=10.1).found)
        self.assertEqual(self.lock.reason, "INVALID_STALE_OR_LEGACY_PROTOCOL")

    def test_replayed_frame_cannot_keep_tracking(self):
        self.acquire()
        self.assertTrue(self.lock.observe(frame(2), now=10.1).found)
        self.assertFalse(self.lock.observe(frame(2), now=10.2).found)
        self.assertFalse(self.lock.locked)

    def test_frame_skip_loses_lock(self):
        self.acquire()
        self.assertFalse(self.lock.observe(frame(4), now=10.2).found)
        self.assertFalse(self.lock.locked)

    def test_more_than_twelve_boxes_block_tracking(self):
        self.acquire()
        boxes = [candidate(i*3, y=0, w=30, h=110) for i in range(13)]
        self.assertFalse(self.lock.observe(frame(2, boxes), now=10.1).found)

    def test_stale_selection_denied(self):
        self.lock.observe(frame(1), now=10.)
        self.assertFalse(self.lock.select(0, 1, now=10.6)[0])

    def test_display_index_history_revalidates_reordering(self):
        self.lock.observe(frame(1, [candidate(20), candidate(210)]), now=10.)
        self.lock.observe(frame(2, [candidate(212), candidate(21)]), now=10.2)
        ok, msg = self.lock.select(0, 1, now=10.25)
        self.assertTrue(ok, msg)
        out = self.lock.observe(frame(3, [candidate(216), candidate(23)]), now=10.3)
        self.assertTrue(out.found)
        self.assertEqual(out.center_x, 59)

    def test_select_nearby_ambiguous_denied(self):
        self.lock.observe(frame(1, [candidate(20),candidate(36)]), now=10.)
        self.assertFalse(self.lock.select(0,1, now=10.1)[0])

    def test_disconnect_resets_frame_sequence_and_lock(self):
        self.acquire()
        self.lock.reset_stream()
        self.assertFalse(self.lock.locked)
        self.assertFalse(self.lock.observe(frame(1), now=11.).found)

    def test_release_explicitly_requires_reselection(self):
        self.acquire()
        self.lock.clear()
        self.assertFalse(self.lock.observe(frame(2), now=10.2).found)

    def test_selected_frame_contains_bbox_not_server_choice(self):
        self.acquire([candidate(20), candidate(210)])
        res = self.lock.observe(frame(2, [candidate(210), candidate(22)]), now=10.15)
        self.assertTrue(res.found)
        self.assertEqual(res.center_x, 58)

    def test_operator_receives_no_fresh_persons_after_timeout(self):
        self.lock.observe(frame(1), now=10.)
        self.assertEqual(self.lock.display(now=11.)["candidate_count"], 0)


class LockRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.run = LockedSimulationRuntime()
        self.run.start()
        self.addCleanup(self.run.close)
        self.seq = 0

    def send(self, boxes=None, *, version=2):
        self.seq += 1
        self.run.process_detection(frame(self.seq,boxes,version=version),70.)

    def lock(self):
        self.send()
        last=self.run.status()
        self.assertEqual(self.run.target_action({"index":0,"frame_seq":last["frame_seq"]}), {"ok":True,"error":None})
        for _ in range(4):
            self.send()
        self.assertTrue(self.run.status()["auto_ready"])

    def test_no_person_lock_auto_refused(self):
        for _ in range(4):self.send()
        self.assertFalse(self.run.action("auto")["ok"])
        self.assertNotIn("SAFETY_ARM",self.run.sink.commands)

    def test_operator_lock_enables_virtual_auto(self):
        self.lock()
        response=self.run.action("auto")
        self.assertTrue(response["ok"])
        await_mode(self.run,"AUTO")

    def test_stranger_enters_and_target_lost_disarms(self):
        self.lock()
        self.run.action("auto")
        await_mode(self.run,"AUTO")
        self.send([candidate(220)])
        self.assertEqual(self.run.status()["mode"],"SAFE")
        self.assertFalse(self.run.status()["presenter_selected"])
        self.assertIn("STOP",self.run.sink.commands)
        for _ in range(3):self.send([candidate(220)])
        self.assertFalse(self.run.action("auto")["ok"])

    def test_selection_must_be_while_safe(self):
        self.lock()
        self.run.action("auto")
        await_mode(self.run,"AUTO")
        self.assertFalse(self.run.target_action({"release":True})["ok"])

    def test_operator_stop_clears_lock(self):
        self.lock()
        self.run.action("stop")
        self.assertFalse(self.run.status()["presenter_selected"])

    def test_manual_can_operate_without_lock(self):
        self.send([candidate(20),candidate(210)])
        result=self.run.action("manual")
        self.assertTrue(result["ok"])
        await_mode(self.run,"MANUAL")
        self.assertTrue(self.run.manual("left")["ok"])

    def test_camera_disconnect_discards_lock_and_seq(self):
        self.lock()
        self.run.set_camera_state(False,False)
        self.assertFalse(self.run.status()["presenter_selected"])
        self.seq=0
        self.send()
        self.assertFalse(self.run.status()["auto_ready"])

    def test_protocol_v1_never_enables_auto(self):
        for _ in range(5):self.send(version=1)
        self.assertFalse(self.run.status()["auto_ready"])
        self.assertFalse(self.run.action("auto")["ok"])

    def test_lock_release_when_safe(self):
        self.lock()
        self.assertTrue(self.run.target_action({"release":True})["ok"])
        self.assertFalse(self.run.status()["presenter_selected"])


class OperatorTargetApiTests(unittest.TestCase):
    def setUp(self):
        self.run=LockedSimulationRuntime()
        self.run.start()
        self.token="phase11-test-"+"q"*32
        self.server=OperatorServer(("127.0.0.1",0),self.run,self.token)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.run.close()

    def request(self, body, *,authorized=True):
        conn=http.client.HTTPConnection("127.0.0.1",self.server.server_port,timeout=2)
        headers={"Content-Type":"application/json"}
        if authorized:headers["Authorization"]="Bearer "+self.token
        conn.request("POST","/api/target",json.dumps(body),headers=headers)
        response=conn.getresponse()
        code=response.status
        val=json.loads(response.read())
        conn.close()
        return code,val

    def test_operator_selection_authenticated(self):
        self.run.process_detection(frame(),60)
        stat=self.run.status()
        self.assertEqual(self.request({"index":0,"frame_seq":stat["frame_seq"]},authorized=False)[0],401)
        self.assertEqual(self.request({"index":0,"frame_seq":stat["frame_seq"]})[0],200)
        self.assertTrue(self.run.status()["presenter_selected"])

    def test_selection_cannot_be_accepted_without_frame(self):
        self.assertEqual(self.request({"index":0,"frame_seq":1})[0],409)

    def test_invalid_target_fields_400(self):
        self.assertEqual(self.request({"index":0,"frame_seq":1,"unexpected":True})[0],400)

    def test_preview_requires_authentication(self):
        self.run.get_preview = lambda: b"jpeg-preview"
        conn = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        conn.request("GET", "/api/preview.jpg")
        response = conn.getresponse()
        self.assertEqual(response.status, 401)
        response.read()
        conn.close()
        conn = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        conn.request("GET", "/api/preview.jpg",
                     headers={"Authorization": "Bearer " + self.token})
        response = conn.getresponse()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Content-Type"), "image/jpeg")
        self.assertEqual(response.read(), b"jpeg-preview")
        conn.close()

    def test_only_explicit_release(self):
        self.run.process_detection(frame(),60)
        seq=self.run.status()["frame_seq"]
        self.request({"index":0,"frame_seq":seq})
        self.assertEqual(self.request({"release":True})[0],200)
        self.assertFalse(self.run.status()["presenter_selected"])


if __name__=="__main__":
    unittest.main()
