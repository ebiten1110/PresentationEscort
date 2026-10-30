"""Regression checks for real Windows USB camera crowd-induced lock losses.

Reproduces key geometry and stable ByteTrack IDs from diagnostics 2026-10-30:
frame 739, where target 21 was rejected when bystander 150 was separate.
Any ambiguous overlap must still reject actuation.
"""
from __future__ import annotations

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from presenter_lock import PresenterLock


class PresenterCrowdContinuityTests(unittest.TestCase):
    @staticmethod
    def frame(seq, people, ids, gestures=(), aims=None):
        return DetectionResult(
            found=bool(people), protocol_version=2, frame_seq=seq,
            frame_width=640, frame_height=480,
            people=[dict(x1=a, y1=b, x2=c, y2=d, confidence=conf)
                    for a, b, c, d, conf in people],
            track_ids=list(ids), gesture_indices=list(gestures),
            aim_points=list(aims or []),
        )

    @staticmethod
    def person(x1, y1, x2, y2, confidence=0.75):
        return (x1, y1, x2, y2, confidence)

    def selected(self):
        lock = PresenterLock(gesture_enabled=True, gesture_confirm_frames=3,
                             transient_recovery_s=1.5)
        # Explicit 3-frame hands-up gesture, same trusted tracked face.
        for seq in (1, 2, 3):
            result = lock.observe(self.frame(
                seq, [self.person(33, 152, 392, 479)], [21], [0],
                [{"x": 221, "y": 275, "source": "face"}]),
                now=10.0 + (seq - 1) * 0.07)
        self.assertTrue(lock.locked)
        self.assertEqual(lock.state, "TRACKING")
        self.assertEqual(lock.gesture_event_id, 1)
        self.assertTrue(result.found)
        return lock

    def test_stable_face_and_id_survive_separate_bystander(self):
        lock = self.selected()
        result = lock.observe(self.frame(
            4, [self.person(39, 153, 399, 479),
                self.person(411, 359, 559, 480, 0.58)], [21, 150], [],
            [{"x": 230, "y": 272, "source": "face"},
             {"x": 481, "y": 450, "source": "shoulders"}]),
            now=10.21)
        self.assertTrue(result.found, f'{lock.state} / {lock.reason} / {lock.display(now=10.21)}')
        self.assertEqual(lock.state, "TRACKING")
        self.assertEqual(lock.gesture_event_id, 1)

    def test_missing_target_stops_instead_of_switching_to_bystander(self):
        lock = self.selected()
        result = lock.observe(self.frame(
            4, [self.person(475, 372, 593, 480)], [150], [],
            [{"x": 520, "y": 405, "source": "face"}]),
            now=10.21)
        self.assertFalse(result.found)
        self.assertEqual(lock.state, "TEMPORARILY_LOST")
        self.assertEqual(lock.gesture_event_id, 1)
        restored = lock.observe(self.frame(
            5, [self.person(37, 154, 391, 479),
                self.person(475, 372, 593, 480)], [21, 150], [],
            [{"x": 226, "y": 275, "source": "face"},
             {"x": 518, "y": 405, "source": "face"}]),
            now=10.28)
        self.assertEqual(lock.state, "TRACKING")
        self.assertEqual(lock.gesture_event_id, 1,
                         "reappearance must never synthesize an ARM gesture")
        self.assertTrue(restored.found)

    def selected_crowd(self, box, face, track_id):
        lock = PresenterLock(gesture_enabled=True, gesture_confirm_frames=3,
                             transient_recovery_s=1.5)
        for seq in (1, 2, 3):
            lock.observe(self.frame(
                seq, [box], [track_id], [0],
                [{"x": face[0], "y": face[1], "source": "face"}]),
                now=10.0 + (seq - 1) * .07)
        self.assertEqual(lock.state, "TRACKING")
        return lock

    def test_crowd_two_body_matches_but_only_one_matching_face(self):
        # At diagnostic frame 967 target ID 83 temporarily becomes 326.
        # Other person (332) has a separate face, though body boxes overlap.
        lock = self.selected_crowd(
            self.person(196, 116, 601, 480, .62), (356, 209), 83)
        result = lock.observe(self.frame(
            4, [self.person(0, 257, 55, 480, .70),
                self.person(94, 141, 370, 480, .60),
                self.person(270, 128, 525, 480, .55)],
            [335, 332, 326], [],
            [{"x": 28, "y": 308, "source": "upper_body"},
             {"x": 216, "y": 263, "source": "face"},
             {"x": 223, "y": 273, "source": "face"}]),
            now=10.21)
        # Stored face 356,209 uniquely matches new ID 326 face 360,210.
        self.assertTrue(result.found, f'{lock.state} / {lock.reason} / {lock.display(now=10.21)}')
        self.assertEqual(lock.state, "TRACKING")
        self.assertEqual(lock.gesture_event_id, 1)

    def test_crowd_nearby_bodies_but_distinct_faces_keep_lock(self):
        lock = self.selected_crowd(
            self.person(230, 103, 561, 480, .60), (411, 200), 326)
        result = lock.observe(self.frame(
            4, [self.person(0, 365, 59, 480, .76),
                self.person(48, 326, 223, 480, .52),
                self.person(207, 351, 324, 480, .65),
                self.person(213, 108, 621, 478, .58)],
            [360, 378, 379, 83], [],
            [{"x": 30, "y": 391, "source": "upper_body"},
             {"x": 136, "y": 361, "source": "upper_body"},
             {"x": 264, "y": 397, "source": "face"},
             {"x": 222, "y": 274, "source": "face"}]),
            now=10.21)
        self.assertTrue(result.found, f'{lock.state} / {lock.reason} / {lock.display(now=10.21)}')
        self.assertEqual(lock.state, "TRACKING")

    def test_unresolved_multi_candidate_ambiguity_stops_motors(self):
        lock = self.selected()
        # Two plausible bodies, no uniquely discriminating selected face.
        result = lock.observe(self.frame(
            4, [self.person(33, 152, 392, 479, .70),
                self.person(46, 157, 389, 479, .69)],
            [21, 41], [],
            [{"x": 221, "y": 275, "source": "shoulders"},
             {"x": 299, "y": 275, "source": "shoulders"}]),
            now=10.21)
        self.assertFalse(result.found)
        self.assertNotEqual(lock.state, "TRACKING")

    def test_candidate_with_wrong_or_duplicate_track_id_is_rejected(self):
        lock = self.selected()
        result = lock.observe(self.frame(
            4, [self.person(39, 153, 399, 479),
                self.person(411, 359, 559, 480)], [21, 21], [],
            [{"x": 230, "y": 272, "source": "face"},
             {"x": 481, "y": 450, "source": "shoulders"}]),
            now=10.21)
        self.assertFalse(result.found)
        self.assertEqual(lock.state, "LOST")

    def test_face_inside_other_person_box_rejects_lock(self):
        lock = self.selected()
        result = lock.observe(self.frame(
            4, [self.person(39, 153, 399, 479),
                self.person(175, 170, 420, 480)], [21, 150], [],
            [{"x": 230, "y": 272, "source": "face"},
             {"x": 280, "y": 260, "source": "shoulders"}]),
            now=10.21)
        self.assertFalse(result.found)
        self.assertEqual(lock.state, "LOST")


if __name__ == "__main__":
    unittest.main()
