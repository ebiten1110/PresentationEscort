"""Fail-closed operator-selected presenter geometry lock (NO face ID/ReID).

The PC sends ALL person boxes (protocol V2). The human operator selects one
candidate while SAFE. Every subsequent frame must contain one unambiguous
geometric successor or tracking is invalidated instantly; loss/crossing
requires explicit re-selection. Never silently acquire the nearest stranger.

This reduces accidental target switching, but cannot guarantee personal
identity when visually similar people exchange positions. Hardware multi-
person AUTO remains NO-GO until real tests / identity safeguards pass.
"""
from __future__ import annotations

from dataclasses import replace
from math import hypot, isfinite
from typing import Optional

from detection_protocol import DetectionResult
from person_tracker import PersonBox


class PresenterLock:
    MAX_PEOPLE = 12
    MAX_FRAME_AGE_S = 0.50
    CONFIDENCE_MIN = 0.45

    def __init__(self, *, auto_select_single: bool = False,
                 transient_recovery_s: float = 0.0,
                 gesture_enabled: bool = False,
                 gesture_confirm_frames: int = 4) -> None:
        if not 0.0 <= transient_recovery_s <= 2.0:
            raise ValueError("invalid transient recovery window")
        if not 2 <= gesture_confirm_frames <= 30:
            raise ValueError("gesture confirmation frames invalid")
        self.gesture_enabled = bool(gesture_enabled)
        self.gesture_confirm_frames = gesture_confirm_frames
        self.gesture_event_id = 0
        self._gesture_candidate: Optional[PersonBox] = None
        self._gesture_streak = 0
        self._gesture_candidate_face: Optional[tuple[int, int]] = None
        self._gesture_gap_frames = 0
        self._needs_resume_gesture = False
        self.auto_select_single = bool(auto_select_single)
        self.transient_recovery_s = transient_recovery_s
        self._single_candidate_streak = 0
        self._candidate_last: Optional[PersonBox] = None
        self._recovery_deadline: Optional[float] = None
        self._last_seq = 0
        self._frame_time: Optional[float] = None
        self._frame_size: Optional[tuple[int, int]] = None
        self.candidates: list[PersonBox] = []
        self._history: dict[int, tuple[float, list[PersonBox]]] = {}
        self._locked: Optional[PersonBox] = None
        self._locked_track_id: Optional[int] = None
        self._candidate_track_ids: list[Optional[int]] = []
        self._last_confirmed_at: Optional[float] = None
        self._velocity_x = 0.0
        self._velocity_y = 0.0
        self._locked_aim: Optional[tuple[int, int]] = None
        self._locked_aim_source: str = ""
        self.locked = False
        self.state = "UNLOCKED"
        self.reason = "OPERATOR_SELECTION_REQUIRED"

    def reset_stream(self) -> None:
        """Discard frame sequence and any identity after a TCP reconnect."""
        self._last_seq = 0
        self._frame_time = None
        self._frame_size = None
        self.candidates = []
        self._candidate_track_ids = []
        self._history.clear()
        self._gesture_candidate = None
        self._gesture_streak = 0
        self._needs_resume_gesture = False
        self._single_candidate_streak = 0
        self._candidate_last = None
        self._recovery_deadline = None
        self.clear("PC_STREAM_DISCONNECTED")

    def clear(self, reason: str = "OPERATOR_SELECTION_REQUIRED") -> None:
        self._locked = None
        self._locked_track_id = None
        self._last_confirmed_at = None
        self._velocity_x = 0.0
        self._velocity_y = 0.0
        self._locked_aim = None
        self._locked_aim_source = ""
        self._gesture_candidate = None
        self._gesture_streak = 0
        self._gesture_candidate_face = None
        self._gesture_gap_frames = 0
        self.locked = False
        self.state = "UNLOCKED"
        self._needs_resume_gesture = False
        self.reason = reason
        self._recovery_deadline = None
        self._single_candidate_streak = 0

    def _lose(self, reason: str) -> None:
        self._locked = None
        self._locked_track_id = None
        self._last_confirmed_at = None
        self._velocity_x = 0.0
        self._velocity_y = 0.0
        self._locked_aim = None
        self._locked_aim_source = ""
        self._gesture_candidate = None
        self._gesture_streak = 0
        self._gesture_candidate_face = None
        self._gesture_gap_frames = 0
        self.locked = False
        self.state = "LOST"
        self._needs_resume_gesture = False
        self.reason = reason
        self._recovery_deadline = None
        self._single_candidate_streak = 0

    def confirm_bounded_reacquisition(self) -> None:
        """Clear the gesture reminder after safety's 3-frame in-AUTO recovery.

        Never creates a new gesture event or arms motors. Call only when the
        existing AUTO session survived its bounded missing-target window.
        """
        if self.locked and self.state == "TRACKING" and self._needs_resume_gesture:
            self._needs_resume_gesture = False
            self._gesture_streak = 0
            self.reason = "AUTO_RECALIBRATED_SAME_TRACK"

    def observe(self, detection: DetectionResult, *, now: float) -> DetectionResult:
        """Process complete V2 frames; output only a human-locked person."""
        suppressed = replace(
            detection, found=False, center_x=0, center_y=0,
            width=0, height=0, confidence=0., class_id=-1, class_name="",
        )
        if (
            not isfinite(now)
            or detection.error
            or detection.protocol_version != 2
            or isinstance(detection.frame_seq, bool)
            or not isinstance(detection.frame_seq, int)
            or detection.frame_seq <= self._last_seq
            or detection.frame_seq - self._last_seq > 1 and self._last_seq != 0
            or not isinstance(detection.people, list)
            or len(detection.people) > self.MAX_PEOPLE
            or not (0 < detection.frame_width <= 8192
                    and 0 < detection.frame_height <= 8192)
        ):
            self.candidates = []
            self._frame_time = None
            self._lose("INVALID_STALE_OR_LEGACY_PROTOCOL")
            return suppressed

        new_frame_size = (detection.frame_width, detection.frame_height)
        if self._frame_size is not None and new_frame_size != self._frame_size:
            self._lose("CAMERA_RESOLUTION_CHANGED")
        previous_frame_time = self._frame_time
        self._frame_size = new_frame_size
        self._last_seq = detection.frame_seq
        self._frame_time = now

        candidates: list[PersonBox] = []
        for item in detection.people:
            if not isinstance(item, dict) or set(item) != {
                "x1", "y1", "x2", "y2", "confidence"
            }:
                self.candidates = []
                self._lose("INVALID_PERSON_BOX")
                return suppressed
            try:
                # Reject booleans, floats for coordinates, NaN and malformed input.
                coords = tuple(item[name] for name in ("x1", "y1", "x2", "y2"))
                if any(type(v) is not int for v in coords):
                    raise ValueError("bbox coordinates must be integers")
                conf = item["confidence"]
                if isinstance(conf, bool) or not isinstance(conf, (int, float)):
                    raise ValueError("confidence must be numeric")
                box = PersonBox(*coords, float(conf))
                if not box.valid(*new_frame_size, self.CONFIDENCE_MIN):
                    raise ValueError("invalid person box")
            except (ValueError, TypeError, OverflowError):
                self.candidates = []
                self._lose("INVALID_PERSON_BOX")
                return suppressed
            candidates.append(box)
        track_ids = detection.track_ids
        if (not isinstance(track_ids, list) or
                (track_ids and len(track_ids) != len(candidates)) or
                any((item is not None and
                     (type(item) is not int or item < 0))
                    for item in track_ids)):
            self.candidates = []
            self._lose("INVALID_TRACK_IDS")
            return suppressed
        previous_candidate_count = len(self.candidates)
        self.candidates = candidates
        self._candidate_track_ids = track_ids

        self._last_gestures = detection.gesture_indices if isinstance(detection.gesture_indices, list) else []
        self._history[self._last_seq] = (now, list(candidates))
        self._history = {
            seq: item for seq, item in self._history.items()
            if now - item[0] <= self.MAX_FRAME_AGE_S
        }
        if not self.locked or self._locked is None:
            if self.gesture_enabled:
                gestures = detection.gesture_indices
                index = gestures[0] if (isinstance(gestures, list)
                         and len(gestures) == 1 and type(gestures[0]) is int
                         and 0 <= gestures[0] < len(candidates)) else None
                proposed = candidates[index] if index is not None else None
                # No selection if another person is too close to disambiguate.
                if proposed is not None and self._near_another(proposed, candidates):
                    proposed = None
                previous = self._gesture_candidate
                if (proposed is None and len(candidates) == 1 and
                        previous is not None and self._gesture_streak > 0 and
                        self._gesture_gap_frames == 0 and
                        abs(candidates[0].cx - previous.cx) <=
                        max(20., previous.width * .35)):
                    # One missing pose frame at ~7fps is not a fresh gesture,
                    # but should not restart an otherwise held hands-up signal.
                    self._gesture_gap_frames = 1
                    self.reason = "GESTURE_ONE_FRAME_GAP"
                    return suppressed
                pose = (detection.aim_points[index] if
                        proposed is not None and isinstance(detection.aim_points, list)
                        and index < len(detection.aim_points) else None)
                proposed_face = ((pose["x"], pose["y"])
                        if isinstance(pose, dict) and pose.get("source") == "face"
                        and type(pose.get("x")) is int and type(pose.get("y")) is int
                        else None)
                face_stable = (proposed_face is not None and
                               self._gesture_candidate_face is not None and
                               abs(proposed_face[0] - self._gesture_candidate_face[0]) <= 42 and
                               abs(proposed_face[1] - self._gesture_candidate_face[1]) <= 42)
                stable = proposed is not None and previous is not None and (
                    abs(proposed.cx - previous.cx) <= max(15., previous.width * .30)
                    and (face_stable or (
                        abs(proposed.cy - previous.cy) <= max(15., previous.height * .30)
                        and .70 <= proposed.width / previous.width <= 1.45)))
                self._gesture_streak = (self._gesture_streak + 1) if stable else (1 if proposed else 0)
                self._gesture_candidate = proposed
                self._gesture_candidate_face = proposed_face
                self._gesture_gap_frames = 0
                self.reason = "HANDS_OVER_HEAD_CONFIRMING" if proposed else "WAIT_BOTH_HANDS_OVER_HEAD"
                if proposed is None or self._gesture_streak < self.gesture_confirm_frames:
                    return suppressed
                self._locked = proposed
                self.locked = True
                self.state = "TRACKING"
                self.reason = "GESTURE_SELECTED_PRESENTATION_TARGET"
                self.gesture_event_id += 1
                self._gesture_candidate = None
                self._gesture_streak = 0
                self._gesture_candidate_face = None
                self._gesture_gap_frames = 0
            elif self.auto_select_single and len(candidates) == 1:
                candidate = candidates[0]
                previous = self._candidate_last
                near = (previous is not None and
                        abs(candidate.cx - previous.cx) <= max(15, previous.width * .25)
                        and abs(candidate.cy - previous.cy) <= max(15, previous.height * .20)
                        and .75 <= candidate.width / previous.width <= 1.35)
                self._single_candidate_streak = self._single_candidate_streak + 1 if near else 1
                self._candidate_last = candidate
                if self._single_candidate_streak >= 4 and self.state != "LOST":
                    # Select a single stable candidate for UI convenience.
                    # Never ARM motors; AUTO remains operator-authorized.
                    self._locked = candidate
                    self.locked = True
                    self.state = "TRACKING"
                    self.reason = "SINGLE_STABLE_CANDIDATE_AUTO_SELECTED"
                else:
                    return suppressed
            else:
                self._single_candidate_streak = 0
                self._candidate_last = None
                return suppressed

        original = self._locked
        allowed: list[PersonBox] = []
        # Constant-velocity prediction is a matching hint, never a motor target.
        # Only confirmed camera measurements update it (not missing frames).
        elapsed = (max(0.0, now - self._last_confirmed_at)
                   if self._last_confirmed_at is not None else 0.0)
        horizon = min(elapsed, 0.35)
        predicted_x = original.cx + self._velocity_x * horizon
        predicted_y = original.cy + self._velocity_y * horizon
        for candidate_index, box in enumerate(candidates):
            same_gpu_track = bool(
                track_ids and self._locked_track_id is not None
                and track_ids[candidate_index] == self._locked_track_id)
            # A stable GPU association may span more than half a small,
            # distant bounding box. Keep an absolute frame-space bound,
            # a finite time window, and the existing crowd-ambiguity gate.
            if (same_gpu_track and elapsed <= 0.45
                    and .45 <= box.width / original.width <= 2.2
                    and .45 <= box.height / original.height <= 2.2
                    and abs(box.cx - predicted_x) <= max(
                        original.width * 1.0, detection.frame_width * .19)
                    and abs(box.cy - predicted_y) <= max(
                        original.height * .65, detection.frame_height * .14)):
                allowed.append(box)
                continue

            # Strict 0.5-body-width displacement and limited size variation.
            if (
                0.55 <= box.width / original.width <= 1.80
                and 0.55 <= box.height / original.height <= 1.80
                and abs(box.cx - original.cx) <= max(12.0, original.width * 0.52)
                and abs(box.cy - original.cy) <= max(12.0, original.height * 0.30)
            ):
                allowed.append(box)
        # The detector sometimes changes the *height* of the same person
        # box between cropped and full-body estimates on consecutive frames.
        # A uniquely visible, stable face is stronger continuity evidence
        # than the box center for this specific geometry-jitter case.
        if (not allowed and len(candidates) == 1 and
                self._locked_aim is not None and self._locked_aim_source == "face" and
                isinstance(detection.aim_points, list) and len(detection.aim_points) == 1):
            face = detection.aim_points[0]
            candidate = candidates[0]
            if (isinstance(face, dict) and face.get("source") == "face" and
                    type(face.get("x")) is int and type(face.get("y")) is int and
                    candidate.x1 <= face["x"] <= candidate.x2 and
                    candidate.y1 <= face["y"] <= candidate.y2 and
                    abs(face["x"] - self._locked_aim[0]) <= 28 and
                    abs(face["y"] - self._locked_aim[1]) <= 28 and
                    .55 <= candidate.width / original.width <= 1.80 and
                    .50 <= candidate.height / original.height <= 2.20):
                allowed = [candidate]
        # A single person can shift more than half of a *tiny* far-distance
        # box between consecutive ~5 FPS camera frames. Bound the additional
        # tolerance in image pixels and require a unique, high-confidence
        # candidate; in a crowd retain the original strict gate.
        if (not allowed and previous_candidate_count == 1 and
                len(candidates) == 1 and candidates[0].confidence >= .70):
            candidate = candidates[0]
            if (0.60 <= candidate.width / original.width <= 1.60
                    and 0.60 <= candidate.height / original.height <= 1.60
                    and abs(candidate.cx - original.cx) <= max(
                        original.width * .52, detection.frame_width * .10)
                    and abs(candidate.cy - original.cy) <= max(
                        original.height * .30, detection.frame_height * .10)):
                allowed = [candidate]
        # Face/box continuity also tolerates a swift horizontal crossing or
        # a crouch-to-stand transition. Only when BOTH adjacent frames have
        # one highly confident person; a crowd still invalidates the lock.
        if (not allowed and previous_candidate_count == 1 and
                len(candidates) == 1 and candidates[0].confidence >= .75 and
                self._locked_aim is not None and self._locked_aim_source == "face" and
                previous_frame_time is not None and
                0 <= now - previous_frame_time <= .30 and
                isinstance(detection.aim_points, list) and len(detection.aim_points) == 1):
            candidate = candidates[0]
            face = detection.aim_points[0]
            overlap_w = max(0, min(original.x2, candidate.x2) -
                            max(original.x1, candidate.x1))
            overlap_h = max(0, min(original.y2, candidate.y2) -
                            max(original.y1, candidate.y1))
            smaller_area = min(original.width * original.height,
                               candidate.width * candidate.height)
            valid_face = (isinstance(face, dict) and face.get("source") == "face"
                          and type(face.get("x")) is int and type(face.get("y")) is int
                          and candidate.x1 <= face["x"] <= candidate.x2
                          and candidate.y1 <= face["y"] <= candidate.y2)
            cross = (valid_face and .7 <= candidate.width / original.width <= 1.4
                     and .7 <= candidate.height / original.height <= 1.4
                     and abs(face["x"] - self._locked_aim[0]) <=
                     detection.frame_width * .28
                     and abs(face["y"] - self._locked_aim[1]) <=
                     detection.frame_height * .13
                     and smaller_area > 0 and
                     overlap_w * overlap_h / smaller_area >= .22)
            standing = (valid_face and .6 <= candidate.width / original.width <= 1.7
                        and .25 <= candidate.height / original.height <= 3.5
                        and abs(face["x"] - self._locked_aim[0]) <=
                        detection.frame_width * .09
                        and abs(face["y"] - self._locked_aim[1]) <=
                        detection.frame_height * .32
                        and overlap_w >= min(original.width, candidate.width) * .65
                        and overlap_h >= min(original.height, candidate.height) * .45)
            if cross or standing:
                allowed = [candidate]
        # A temporarily missing, single high-confidence person can return
        # near the predicted position even without the original overlap.
        # This does not create an ARM; Pi requires 3 new visual confirmations.
        if (not allowed and self.state == "TEMPORARILY_LOST"
                and self._recovery_deadline is not None
                and now <= self._recovery_deadline
                and elapsed <= .65 and len(candidates) == 1
                and candidates[0].confidence >= .78):
            candidate = candidates[0]
            if (.55 <= candidate.width / original.width <= 1.8
                    and .55 <= candidate.height / original.height <= 2.0
                    and abs(candidate.cx - predicted_x) <= max(
                        original.width * .8, detection.frame_width * .14)
                    and abs(candidate.cy - predicted_y) <= max(
                        original.height * .55, detection.frame_height * .12)):
                allowed = [candidate]
        # During a bounded detector dropout, retain a single significantly
        # overlapping person as a VISUAL hint only. The first lost frame
        # already disarmed AUTO; this must not issue any ARM request.
        if (not allowed and self.state == "TEMPORARILY_LOST" and
                self._recovery_deadline is not None and now <= self._recovery_deadline and
                len(candidates) == 1 and candidates[0].confidence >= .65):
            candidate = candidates[0]
            overlap_width = max(0, min(original.x2, candidate.x2) -
                                max(original.x1, candidate.x1))
            overlap_height = max(0, min(original.y2, candidate.y2) -
                                 max(original.y1, candidate.y1))
            smaller_area = min(original.width * original.height,
                               candidate.width * candidate.height)
            if (smaller_area > 0 and overlap_width * overlap_height / smaller_area >= .65
                    and .50 <= candidate.width / original.width <= 2.4
                    and .50 <= candidate.height / original.height <= 2.2
                    and abs(candidate.cx - original.cx) <= original.width * .85):
                allowed = [candidate]
        if len(allowed) > 1:
            # A crowded frame can put multiple BODY boxes in the same area
            # even though just one trusted, fresh FACE remains there.
            face_successor = self._unique_recent_face_successor(
                allowed, candidates, detection, now=now)
            if face_successor is not None:
                allowed = [face_successor]
        if len(allowed) != 1:
            # If the selected GPU ID disappears for a single frame while
            # *other* people remain visible, keep only a NON-ACTUATING visual
            # hint. Previously this cleared the lock immediately (e.g. 554,
            # 605, 648, 728); safety must still STOP and require a new gesture
            # after any disarm. Never infer the target from an unrelated ID.
            selected_id_missing = (
                self._locked_track_id is not None
                and self._locked_aim is not None
                and self._locked_aim_source == "face"
                and self._last_confirmed_at is not None
                and 0 <= now - self._last_confirmed_at <= .30
                and bool(track_ids)
                and self._locked_track_id not in track_ids
                and len(candidates) <= 3
                and all(
                    hypot(
                        max(box.x1 - self._locked_aim[0], 0,
                            self._locked_aim[0] - box.x2),
                        max(box.y1 - self._locked_aim[1], 0,
                            self._locked_aim[1] - box.y2),
                    ) >= max(65., detection.frame_width * .10)
                    for box in candidates
                )
            )
            # A crowd may briefly produce multiple plausible boxes for the
            # one selected target. Retain a bounded *stationary* visual hint
            # instead of discarding selection; no person is reported found.
            # No new ARM is generated by a mere reappearance.
            crowded_ambiguity = (
                self.gesture_enabled and self._locked is not None
                and self._locked_aim is not None
                and self._locked_aim_source == "face"
                and self._last_confirmed_at is not None
                and 0 <= now - self._last_confirmed_at <= .25
                and 1 <= len(candidates) <= 8
                and len(allowed) > 1
            )
            if self.transient_recovery_s > 0 and (
                    len(candidates) == 0 or selected_id_missing or crowded_ambiguity):
                if self._recovery_deadline is None:
                    self._recovery_deadline = now + self.transient_recovery_s
                if now <= self._recovery_deadline:
                    # No target is reported to the motion gate: the robot
                    # STOPs, but the geometrical hint survives briefly.
                    self.state = "TEMPORARILY_LOST"
                    self._needs_resume_gesture = self.gesture_enabled
                    self._gesture_streak = 0
                    self.reason = "WAIT_SAME_PERSON_REAPPEAR"
                    return suppressed
            self._lose("TARGET_MISSING_OR_MULTIPLE_MATCHES")
            return suppressed
        chosen = allowed[0]
        # A nearby second person can exchange places on the next frame.
        # Do not continue steering through an interaction/occlusion.
        if (
            self._near_another(chosen, candidates)
            and not self._distinct_face_match(chosen, candidates, detection)
            and not self._separate_stable_tracked_face(
                chosen, candidates, detection, now=now)
        ):
            if (self.gesture_enabled and self.transient_recovery_s > 0
                    and self._locked_aim is not None
                    and self._last_confirmed_at is not None
                    and 0 <= now - self._last_confirmed_at <= .25):
                if self._recovery_deadline is None:
                    self._recovery_deadline = now + self.transient_recovery_s
                if now <= self._recovery_deadline:
                    self.state = "TEMPORARILY_LOST"
                    self.reason = "CROWD_AMBIGUITY_STOP_WAIT_RECONFIRM"
                    self._needs_resume_gesture = True
                    self._gesture_streak = 0
                    return suppressed
            self._lose("AMBIGUOUS_NEARBY_PERSON")
            return suppressed
        self._recovery_deadline = None
        if self._last_confirmed_at is not None:
            dt = now - self._last_confirmed_at
            if .025 <= dt <= .50:
                # Bound image velocity: a wrong association must not predict
                # across the entire frame on the next dropped detection.
                vx = (chosen.cx - original.cx) / dt
                vy = (chosen.cy - original.cy) / dt
                self._velocity_x = max(-700., min(700., vx))
                self._velocity_y = max(-500., min(500., vy))
        else:
            self._velocity_x = self._velocity_y = 0.0
        self._last_confirmed_at = now
        self._locked = chosen
        self.state = "TRACKING"
        self.reason = ("VISUAL_REACQUIRED_GESTURE_REQUIRED"
                       if self._needs_resume_gesture else "LOCKED_GEOMETRY_ONLY")
        chosen_index = next((i for i, box in enumerate(candidates) if box is chosen), -1)
        self._locked_track_id = (
            track_ids[chosen_index]
            if track_ids and chosen_index >= 0 else None)
        if self._needs_resume_gesture:
            raised = (isinstance(detection.gesture_indices, list) and
                      chosen_index in detection.gesture_indices and
                      len(detection.gesture_indices) == 1)
            self._gesture_streak = self._gesture_streak + 1 if raised else 0
            if self._gesture_streak >= self.gesture_confirm_frames:
                # After a dropout, only a fresh sustained explicit gesture
                # can request a new ARM attempt; mere reappearance cannot.
                self.gesture_event_id += 1
                self._needs_resume_gesture = False
                self._gesture_streak = 0
                self.reason = "GESTURE_RESTART_REQUESTED"
        face_x = round(chosen.cx)
        face_y = round(chosen.cy)
        aim_source = ""
        if (chosen_index >= 0 and isinstance(detection.aim_points, list)
                and chosen_index < len(detection.aim_points)):
            aim = detection.aim_points[chosen_index]
            if (isinstance(aim, dict) and type(aim.get("x")) is int
                    and type(aim.get("y")) is int
                    and chosen.x1 <= aim["x"] <= chosen.x2
                    and chosen.y1 <= aim["y"] <= chosen.y2):
                face_x, face_y = aim["x"], aim["y"]
                aim_source = aim.get("source") if aim.get("source") in ("face", "shoulders", "upper_body") else ""
        self._locked_aim = (face_x, face_y) if aim_source == "face" else None
        self._locked_aim_source = aim_source
        return replace(
            detection, found=True, center_x=round(chosen.cx),
            center_y=round(chosen.cy), aim_x=face_x, aim_y=face_y,
            aim_source=aim_source,
            width=chosen.width,
            height=chosen.height, confidence=chosen.confidence,
            class_id=0, class_name="person",
        )

    @staticmethod
    def _near_another(chosen: PersonBox, candidates: list[PersonBox]) -> bool:
        return any(
            other is not chosen and
            hypot(chosen.cx - other.cx, chosen.cy - other.cy)
            < max(32.0, 0.80 * max(chosen.width, other.width))
            for other in candidates
        )

    def _distinct_face_match(self, chosen: PersonBox, candidates: list[PersonBox],
                             detection: DetectionResult) -> bool:
        """Continue a previously stable face despite overlapping body boxes.

        This verifies proximity in IMAGE space, never biometric identity.
        If a second face is not unambiguously separated, do not move.
        """
        if (self._locked_aim is None or self._locked_aim_source != "face" or
                not isinstance(detection.aim_points, list) or
                len(detection.aim_points) != len(candidates)):
            return False
        idx = next((i for i, box in enumerate(candidates) if box is chosen), -1)
        if idx < 0:
            return False
        point = detection.aim_points[idx]
        if not isinstance(point, dict) or point.get("source") != "face" or (
                type(point.get("x")) is not int or type(point.get("y")) is not int):
            return False
        if hypot(point["x"] - self._locked_aim[0],
                 point["y"] - self._locked_aim[1]) > 38:
            return False
        for i, other in enumerate(detection.aim_points):
            if i == idx:
                continue
            if not isinstance(other, dict) or other.get("source") != "face" or (
                    type(other.get("x")) is not int or type(other.get("y")) is not int):
                return False
            if hypot(point["x"] - other["x"], point["y"] - other["y"]) < max(
                    60., detection.frame_width * .13):
                return False
        return True

    def _separate_stable_tracked_face(
        self, chosen: PersonBox, candidates: list[PersonBox],
        detection: DetectionResult, *, now: float,
    ) -> bool:
        """Allow an unchanged GPU track beside *separate* other faces/bodies.

        A wide foreground body box makes center-distance alone reject people
        standing far apart in the image (diagnostic: frame 739). This exception
        requires independently matching tracker ID, recent stable face, and
        meaningful distance between that face and every other person's box.
        Crowds/overlapping faces remain ambiguous and must STOP.
        """
        if (
            self._locked is None or self._locked_track_id is None
            or self._locked_aim is None or self._locked_aim_source != "face"
            or self._last_confirmed_at is None
            or not 0 <= now - self._last_confirmed_at <= 0.25
            or not isinstance(detection.track_ids, list)
            or len(detection.track_ids) != len(candidates)
            or not isinstance(detection.aim_points, list)
            or len(detection.aim_points) != len(candidates)
            or not 1 < len(candidates) <= 8
        ):
            return False
        try:
            index = next(i for i, box in enumerate(candidates) if box is chosen)
        except StopIteration:
            return False
        tracks = detection.track_ids
        selected_track = tracks[index]
        if (
            selected_track is None
            or any(track is None or track == selected_track
                   for i, track in enumerate(tracks) if i != index)
            or (selected_track != self._locked_track_id
                and self._locked_track_id in tracks)
        ):
            return False
        same_track = selected_track == self._locked_track_id
        previous = self._locked
        if (
            not .55 <= chosen.width / previous.width <= 1.5
            or not .60 <= chosen.height / previous.height <= 1.55
            or abs(chosen.cx - previous.cx) > max(26., previous.width * .32)
            or abs(chosen.cy - previous.cy) > max(26., previous.height * .30)
        ):
            return False
        aim = detection.aim_points[index]
        if (
            not isinstance(aim, dict) or aim.get("source") != "face"
            or type(aim.get("x")) is not int or type(aim.get("y")) is not int
        ):
            return False
        face_x, face_y = aim["x"], aim["y"]
        if (
            not chosen.x1 <= face_x <= chosen.x2
            or not chosen.y1 <= face_y <= chosen.y2
            or hypot(face_x - self._locked_aim[0],
                     face_y - self._locked_aim[1]) >
                (28. if same_track else 13.)
            or (not same_track and chosen.confidence < .52)
        ):
            return False
        # Distinguish faces, not entire body boxes: people in a crowd may
        # have overlapping body silhouettes while their faces are clearly
        # separate. A person without a reliable face must instead have a
        # box safely away from the selected face.
        min_separation_px = max(70., detection.frame_width * .11)
        for i, other in enumerate(candidates):
            if i == index:
                continue
            other_aim = detection.aim_points[i]
            if (isinstance(other_aim, dict)
                    and other_aim.get("source") == "face"
                    and type(other_aim.get("x")) is int
                    and type(other_aim.get("y")) is int):
                if hypot(other_aim["x"] - face_x,
                         other_aim["y"] - face_y) < min_separation_px:
                    return False
                continue
            dx = max(other.x1 - face_x, 0, face_x - other.x2)
            dy = max(other.y1 - face_y, 0, face_y - other.y2)
            if hypot(dx, dy) < min_separation_px:
                return False
        return True

    def _unique_recent_face_successor(
        self, allowed: list[PersonBox], candidates: list[PersonBox],
        detection: DetectionResult, *, now: float,
    ) -> Optional[PersonBox]:
        """Choose only a unique, very close face among multiple body matches.

        In a crowd, 2+ body boxes can overlap the old position. Never select
        by nearest center alone. Require the former face, a recent time, and
        every other face (or uncertain body box) safely separated.
        """
        if (
            self._locked is None or self._locked_aim is None
            or self._locked_aim_source != "face"
            or self._last_confirmed_at is None
            or not 0 <= now - self._last_confirmed_at <= .22
            or not isinstance(detection.aim_points, list)
            or len(detection.aim_points) != len(candidates)
            or not 1 < len(allowed) <= 4
        ):
            return None
        matches = []
        for candidate in allowed:
            idx = next((i for i, box in enumerate(candidates)
                        if box is candidate), -1)
            if idx < 0:
                continue
            point = detection.aim_points[idx]
            if (isinstance(point, dict) and point.get("source") == "face"
                    and type(point.get("x")) is int
                    and type(point.get("y")) is int
                    and hypot(point["x"] - self._locked_aim[0],
                              point["y"] - self._locked_aim[1]) <= 18.
                    and candidate.confidence >= .52):
                matches.append(candidate)
        if len(matches) != 1:
            return None
        chosen = matches[0]
        # A second face or faceless silhouette too close to the chosen face
        # must always stop AUTO. Strict same-face duplicates are collapsed
        # earlier, before this method, by crowd_pose_dedup.
        if self._separate_stable_tracked_face(
                chosen, candidates, detection, now=now):
            return chosen
        if not self._distinct_face_match(chosen, candidates, detection):
            return None
        # Preserve tracker ID when the old ID remains among candidates.
        try:
            idx = next(i for i, box in enumerate(candidates) if box is chosen)
            track_ids = detection.track_ids
            if (self._locked_track_id is not None and track_ids
                    and self._locked_track_id in track_ids
                    and track_ids[idx] != self._locked_track_id):
                return None
        except (StopIteration, IndexError):
            return None
        return chosen

    def select(self, index: int, frame_seq: int, *, now: float) -> tuple[bool, str]:
        if self.locked:
            return False, "UNLOCK_FIRST"
        if (
            type(index) is not int or type(frame_seq) is not int
            or not isfinite(now) or self._frame_time is None
            or now < self._frame_time
            or now - self._frame_time > self.MAX_FRAME_AGE_S
            or frame_seq not in self._history
        ):
            return False, "STALE_OR_INVALID_SELECTION"
        history_at, historical = self._history[frame_seq]
        if (
            now - history_at > self.MAX_FRAME_AGE_S
            or not 0 <= index < len(historical)
        ):
            return False, "STALE_OR_INVALID_SELECTION"
        old = historical[index]
        # The UI can be one or two frames behind. Revalidate selection
        # against the actual most recent camera frame, never trust an index
        # that may have shifted due to detection reordering.
        matches = [
            box for box in self.candidates
            if (
                0.7 <= box.width / old.width <= 1.4
                and 0.7 <= box.height / old.height <= 1.4
                and abs(box.cx - old.cx) <= max(10.0, old.width * 0.35)
                and abs(box.cy - old.cy) <= max(10.0, old.height * 0.22)
            )
        ]
        if len(matches) != 1 or self._near_another(matches[0], self.candidates):
            return False, "SELECTION_AMBIGUOUS_OR_MOVED"
        self._locked = matches[0]
        selected_index = self.candidates.index(matches[0])
        self._locked_track_id = (
            self._candidate_track_ids[selected_index]
            if self._candidate_track_ids else None)
        self._last_confirmed_at = now
        self._velocity_x = self._velocity_y = 0.0
        self.locked = True
        self.state = "SELECTED_WAIT_NEXT_FRAME"
        self._recovery_deadline = None
        self.reason = "LOCK_SELECTED_NEEDS_FRESH_FRAMES"
        return True, "SELECTED"

    def display(self, *, now: float) -> dict:
        recent = (
            self._frame_time is not None and isfinite(now)
            and 0 <= now - self._frame_time <= self.MAX_FRAME_AGE_S
        )
        fw, fh = self._frame_size or (1, 1)
        return {
            "lock_state": self.state if recent else "NO_FRESH_FRAME",
            "lock_reason": self.reason,
            "tracker_id": self._locked_track_id,
            "velocity_px_s": [round(self._velocity_x, 1),
                              round(self._velocity_y, 1)],
            "gesture_enabled": self.gesture_enabled,
            "gesture_event_id": self.gesture_event_id,
            "gesture_hint": "両手を頭の上に挙げて、そのまま保持",
            "presenter_selected": self.locked and recent and self.state in ("TRACKING", "SELECTED_WAIT_NEXT_FRAME"),
            "selected_index": next(
                (i for i, box in enumerate(self.candidates)
                 if self.locked and self._locked is not None
                 and box is self._locked), None
            ),
            "frame_seq": self._last_seq if recent else 0,
            "candidate_count": len(self.candidates) if recent else 0,
            "candidates": [
                {"index": i,
                 "x1_percent": round(box.x1 / fw * 100, 2),
                 "y1_percent": round(box.y1 / fh * 100, 2),
                 "x2_percent": round(box.x2 / fw * 100, 2),
                 "y2_percent": round(box.y2 / fh * 100, 2),
                 "x_percent": round(box.cx / fw * 100),
                 "y_percent": round(box.cy / fh * 100),
                 "confidence": round(box.confidence, 2),
                 "gesturing": bool(self.gesture_enabled and
                    isinstance(getattr(self, "_last_gestures", None), list)
                    and i in self._last_gestures)}
                for i, box in enumerate(self.candidates)
            ] if recent else [],
        }
