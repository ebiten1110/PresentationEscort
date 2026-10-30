"""Hybrid tracking: gesture initiates identity, largest body supports only THAT identity.

No largest-by-accident ARM, no takeover by a newly appearing bystander.
Hardware STOP/watchdog behavior is covered by the existing runtime tests.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from detection_protocol import DetectionResult
from presenter_lock import PresenterLock


def frame(seq, people, ids, faces, gestures=()):
    return DetectionResult(
        found=bool(people), protocol_version=2, frame_seq=seq,
        frame_width=640, frame_height=480,
        people=[dict(x1=x1, y1=y1, x2=x2, y2=y2, confidence=conf)
                for x1,y1,x2,y2,conf in people],
        track_ids=list(ids), gesture_indices=list(gestures),
        aim_points=[{"x":x,"y":y,"source":source}
                    for x,y,source in faces],
    )


class HybridGestureLargestTests(unittest.TestCase):
    def begin(self):
        lock = PresenterLock(
            gesture_enabled=True, gesture_confirm_frames=3,
            transient_recovery_s=1.5)
        lock.hybrid_gesture_largest = True
        for seq in (1,2,3):
            result = lock.observe(frame(
                seq, [(22,124,260,470,.85)], [71],
                [(139,170,"face")], [0]), now=10.0+seq*.07)
        self.assertTrue(result.found)
        self.assertEqual(lock._locked_track_id,71)
        self.assertEqual(lock.gesture_event_id,1)
        return lock

    def test_gesture_selects_even_if_another_person_is_larger(self):
        lock=PresenterLock(gesture_enabled=True,gesture_confirm_frames=3)
        lock.hybrid_gesture_largest=True
        for seq in (1,2,3):
            result=lock.observe(frame(
                seq,
                [(22,124,240,470,.86), (313,20,620,480,.94)],
                [71,90],
                [(138,171,"face"), (455,100,"face")],
                [0]),now=10+seq*.07)
        self.assertTrue(result.found)
        self.assertEqual(lock._locked_track_id,71)
        self.assertEqual(lock.gesture_event_id,1)

    def test_larger_bystander_cannot_takeover_existing_gesture_target(self):
        lock=self.begin()
        result=lock.observe(frame(
            4,
            [(24,122,263,470,.85),(320,0,638,480,.94)],
            [71,90],
            [(141,170,"face"),(450,85,"face")]),
            now=10.28)
        self.assertTrue(result.found)
        self.assertEqual(lock._locked_track_id,71)
        self.assertEqual(lock.gesture_event_id,1)

    def test_same_face_new_tracker_id_can_continue_without_new_arm(self):
        lock=self.begin()
        result=lock.observe(frame(
            4, [(24,122,263,470,.88)], [108],
            [(142,171,"face")]), now=10.28)
        self.assertTrue(result.found)
        self.assertEqual(lock._locked_track_id,108)
        self.assertEqual(lock.gesture_event_id,1)

    def test_far_larger_bystander_never_becomes_presentation_target(self):
        lock=self.begin()
        result=lock.observe(frame(
            4, [(322,22,638,480,.95)], [90],
            [(450,85,"face")]), now=10.28)
        self.assertFalse(result.found)
        self.assertNotEqual(lock._locked_track_id,90)

    def test_no_gesture_never_auto_locks_even_with_unique_largest(self):
        lock=PresenterLock(gesture_enabled=True)
        lock.hybrid_gesture_largest=True
        for seq in range(1,15):
            result=lock.observe(frame(
                seq,[(10,20,600,478,.95)],[31],
                [(282,90,"face")]),now=10+seq*.06)
            self.assertFalse(result.found)
        self.assertFalse(lock.locked)
        self.assertEqual(lock.gesture_event_id,0)


if __name__=="__main__":
    unittest.main()
