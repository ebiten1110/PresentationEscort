"""Tests for safe presenter continuity guard (hardware-free)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from person_tracker import PersonBox, PersonTracker


def box(x=30, y=10, w=100, h=210, conf=0.80):
    return PersonBox(x, y, x+w, y+h, conf)


class PersonTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = PersonTracker()

    def test_first_acquire(self):
        self.assertEqual(self.tracker.select([box()], 320, 240), box())

    def test_tracks_horizontal_motion(self):
        self.tracker.select([box()], 320, 240)
        self.assertEqual(self.tracker.select([box(x=60)], 320, 240), box(x=60))

    def test_reject_sudden_false_person_high_confidence(self):
        self.tracker.select([box(conf=0.55)], 320, 240)
        self.assertIsNone(self.tracker.select([box(x=203, y=40, w=29, h=48, conf=0.99)], 320, 240))
        self.assertEqual(self.tracker.select([box(x=32, conf=0.52)], 320, 240), box(x=32, conf=0.52))

    def test_prefers_nearby_person_over_higher_confidence_stranger(self):
        self.tracker.select([box()], 320, 240)
        adjacent = box(x=40, conf=0.55)
        stranger = box(x=210, y=80, w=40, h=70, conf=0.99)
        self.assertEqual(self.tracker.select([stranger, adjacent], 320, 240), adjacent)

    def test_short_occlusion_does_not_steal_lock(self):
        self.tracker.select([box()], 320, 240)
        self.assertIsNone(self.tracker.select([], 320, 240))
        self.assertIsNone(self.tracker.select([], 320, 240))
        self.assertEqual(self.tracker.select([box(x=35)], 320, 240), box(x=35))

    def test_release_and_reacquire_after_five_misses(self):
        self.tracker.select([box()], 320, 240)
        other = box(x=210, y=80, w=60, h=100)
        for _ in range(5):
            self.assertIsNone(self.tracker.select([other], 320, 240))
        self.assertEqual(self.tracker.select([other], 320, 240), other)

    def test_invalid_boxes_are_ignored(self):
        self.assertIsNone(self.tracker.select([box(x=-5)], 320, 240))
        self.assertIsNone(self.tracker.select([box(conf=0.1)], 320, 240))
        self.assertIsNone(self.tracker.select([box(w=900)], 320, 240))

    def test_frame_size_change_resets(self):
        self.tracker.select([box()], 320, 240)
        candidate = box(x=420, y=10, w=100, h=210)
        self.assertEqual(self.tracker.select([candidate], 640, 480), candidate)

    def test_no_reuse_of_stale_bbox(self):
        self.tracker.select([box()], 320, 240)
        self.assertIsNone(self.tracker.select([], 320, 240))

    def test_reset_clears_previous_identity(self):
        self.tracker.select([box()], 320, 240)
        self.tracker.reset()
        self.assertEqual(self.tracker.select([box(x=200, y=60, w=60, h=90)], 320, 240), box(x=200, y=60, w=60, h=90))


if __name__ == '__main__':
    unittest.main()
