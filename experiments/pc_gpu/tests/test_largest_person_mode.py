"""Opt-in largest-person acquisition cannot arm motors or switch identities."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from detection_protocol import DetectionResult
from presenter_lock import PresenterLock


def frame(seq, boxes, ids, faces):
    return DetectionResult(
        found=bool(boxes), frame_width=640, frame_height=480,
        protocol_version=2, frame_seq=seq,
        people=[dict(x1=x1,y1=y1,x2=x2,y2=y2,confidence=confidence)
                for x1,y1,x2,y2,confidence in boxes],
        track_ids=list(ids),
        aim_points=[{"x":x, "y":y, "source":src} for x,y,src in faces],
        gesture_indices=[],
    )


class LargestPersonTargetTests(unittest.TestCase):
    def candidate(self):
        return frame(1, [(285,35,630,475,.91), (30,300,100,470,.80)],
                     [88,45], [(430,110,"face"), (60,340,"face")])

    def test_stable_foreground_person_acquired_without_arm_event(self):
        p=PresenterLock(gesture_enabled=True,gesture_confirm_frames=3,
                        transient_recovery_s=1.5)
        p.largest_person_enabled=True
        for i in range(1,9):
            f=self.candidate()
            f.frame_seq=i
            result=p.observe(f,now=10+i/30)
            if i<8:
                self.assertFalse(result.found)
                self.assertFalse(p.locked)
        self.assertTrue(result.found)
        self.assertEqual(p.state,"TRACKING")
        self.assertEqual(p.gesture_event_id,0)
        self.assertEqual(p._locked_track_id,88)
        self.assertTrue(p.display(now=10.3)["largest_person_enabled"])

    def test_same_sized_people_not_auto_selected(self):
        p=PresenterLock(gesture_enabled=True)
        p.largest_person_enabled=True
        for i in range(1,15):
            f=frame(i,[(45,45,300,475,.87),(335,45,590,475,.88)],
                    [12,13],[(135,120,"face"),(455,125,"face")])
            result=p.observe(f,now=10+i/30)
            self.assertFalse(result.found)
        self.assertEqual(p.gesture_event_id,0)

    def test_crowded_face_overlap_not_auto_selected(self):
        p=PresenterLock(gesture_enabled=True)
        p.largest_person_enabled=True
        for i in range(1,15):
            f=frame(i,[(140,20,590,475,.95),(170,80,380,475,.75)],
                    [12,13],[(300,110,"face"),(325,115,"face")])
            result=p.observe(f,now=10+i/30)
            self.assertFalse(result.found)
        self.assertEqual(p.gesture_event_id,0)

    def test_default_mode_remains_gesture_only(self):
        p=PresenterLock(gesture_enabled=True)
        for i in range(1,15):
            f=self.candidate()
            f.frame_seq=i
            self.assertFalse(p.observe(f,now=10+i/30).found)
        self.assertFalse(p.locked)


if __name__ == "__main__":
    unittest.main()
