"""Association, room boundaries and mapper transaction regressions."""

from dataclasses import replace
import unittest
from unittest.mock import Mock, patch

from frontier_search.config import MappingConfig
from frontier_search.contracts import Point2, Point3, RegionPrior
from frontier_search.perception.detection import RGBDetection
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.perception.tracking import associate_rooms, update_tracks
from test_mapper_update import request, scene_frame


def box(index=0, label='cup', confidence=.9):
    return RGBDetection(label, confidence, (index * 5, 0, index * 5 + 3, 3))


class ObjectTrackingTests(unittest.TestCase):
    def setUp(self):
        self.state = ({}, {}, 0, 0)
        self.cfg = MappingConfig(object_match_distance_m=.35)
        self.sequence = 0

    def update(self, detections, positions):
        self.sequence += 1
        tracks, evidence, counter, obs_counter = self.state
        # Positions are keyed by bbox so detector ordering cannot affect localization.
        lookup = {d.bbox_xyxy_px: p for d, p in zip(detections, positions)}
        with patch('frontier_search.perception.tracking.localize_detection',
                   side_effect=lambda d, *_: lookup[d.bbox_xyxy_px]):
            self.state = update_tracks(tracks, evidence, detections,
                scene_frame(sequence=self.sequence), self.cfg, counter, obs_counter)
        return tuple(self.state[0].values())

    def test_identity_survives_noise_missed_frames_and_long_revisit(self):
        first, = self.update((box(),), (Point3(1., 0., .5),))
        second, = self.update((box(),), (Point3(1.1, 0., .5),))
        self.assertEqual(first.track_id, second.track_id)
        self.assertAlmostEqual(second.position_world.x_m, 1.05)
        self.assertEqual(second.confirmation_count, 2)
        self.assertEqual(first.confirmation_count, 1)
        self.assertEqual(self.update((), ()), (second,))
        self.sequence = 100
        third, = self.update((box(),), (Point3(1., 0., .5),))
        self.assertEqual(third.track_id, first.track_id)
        self.assertEqual(third.first_seen_s, 1.)
        self.assertEqual(third.last_seen_s, 101.)
        self.assertEqual(len(self.state[1]), 3)

    def test_multiple_same_class_objects_are_matched_once_independent_of_order(self):
        initial = self.update((box(0), box(1)), (Point3(1., 0., 0.), Point3(1.3, 0., 0.)))
        result = self.update((box(1), box(0)), (Point3(1.28, 0., 0.), Point3(1.02, 0., 0.)))
        self.assertEqual([t.track_id for t in initial], [t.track_id for t in result])
        self.assertEqual([t.confirmation_count for t in result], [2, 2])
        self.assertAlmostEqual(result[0].position_world.x_m, 1.01)
        self.assertAlmostEqual(result[1].position_world.x_m, 1.29)

    def test_class_and_full_3d_distance_gate_association(self):
        self.update((box(),), (Point3(1., 0., 0.),))
        tracks = self.update((box(0, 'chair'), box(1)), (Point3(1., 0., 0.), Point3(1., 0., .5)))
        self.assertEqual(len(tracks), 3)
        self.assertTrue(all(t.confirmation_count == 1 for t in tracks))

    def test_anchor_prevents_gradual_drift(self):
        self.update((box(),), (Point3(0., 0., 0.),))
        self.update((box(),), (Point3(.3, 0., 0.),))
        tracks = self.update((box(),), (Point3(.4, 0., 0.),))
        self.assertEqual(len(tracks), 2)
        self.assertEqual(tracks[0].confirmation_count, 2)

    def test_unknown_positions_do_not_match_and_expire_next_frame(self):
        first, = self.update((box(),), (None,))
        second, = self.update((box(),), (None,))
        self.assertNotEqual(first.track_id, second.track_id)
        self.assertEqual(second.confirmation_count, 1)
        localized, = self.update((box(),), (Point3(1., 0., 0.),))
        self.assertNotEqual(second.track_id, localized.track_id)
        self.assertEqual(len(self.state[1]), 1)

    def test_one_track_cannot_accept_two_detections_from_one_frame(self):
        self.update((box(),), (Point3(1., 0., 0.),))
        tracks = self.update((box(0), box(1)), (Point3(1., 0., 0.), Point3(1.1, 0., 0.)))
        self.assertEqual(sorted(t.confirmation_count for t in tracks), [1, 2])

    def test_duplicate_box_keeps_strongest_and_low_confidence_is_ignored(self):
        tracks = self.update((box(confidence=.85), box(confidence=.95), box(1, confidence=.7)),
                             (Point3(1., 0., 0.), Point3(1., 0., 0.), Point3(2., 0., 0.)))
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].confidence, .95)
        self.assertEqual(tracks[0].confirmation_count, 1)

    def test_rooms_use_unique_continuous_polygon_containment(self):
        regions = (RegionPrior('R1', (Point2(0, 0), Point2(1, 0), Point2(1, 1), Point2(0, 1)), None),
                   RegionPrior('R2', (Point2(1, 0), Point2(2, 0), Point2(2, 1), Point2(1, 1)), None))
        detections = tuple(box(i) for i in range(5))
        self.update(detections, (Point3(.01, .01, 1.), Point3(1.5, .5, 1.),
                    Point3(1., .5, 1.), Point3(3., .5, 1.), None))
        assignments, classes = associate_rooms(self.state[0], regions)
        self.assertEqual(list(assignments.values()), ['R1', 'R2', None, None, None])
        self.assertEqual(classes, {'R1': ('cup',), 'R2': ('cup',)})
        duplicate = replace(regions[0], region_id='R3')
        assignments, _ = associate_rooms(self.state[0], regions + (duplicate,))
        self.assertIsNone(assignments['track-000001'])

    def test_concave_polygon_excludes_notch(self):
        region = RegionPrior('L', (Point2(0, 0), Point2(2, 0), Point2(2, 1),
                                  Point2(1, 1), Point2(1, 2), Point2(0, 2)), None)
        self.update((box(0), box(1)), (Point3(.5, 1.5, 0.), Point3(1.5, 1.5, 0.)))
        assignments, _ = associate_rooms(self.state[0], (region,))
        self.assertEqual(list(assignments.values()), ['L', None])

    def test_config_rejects_invalid_match_distance(self):
        for value in (0, -1, True, float('nan'), float('inf'), '0.3'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                MappingConfig(object_match_distance_m=value)

    def test_mapper_publishes_room_classes_and_rolls_back_late_failure(self):
        detector = Mock()
        detector.detect.return_value = (box(),)
        mapper = RGBDMapper(detector)
        mapper.reset(request())
        with patch('frontier_search.perception.tracking.localize_detection', return_value=Point3(1., 0., .5)):
            first = mapper.update(scene_frame(sequence=1))
            previous = mapper._state
            with patch('frontier_search.perception.mapper.build_candidates', side_effect=RuntimeError('late failure')):
                with self.assertRaises(RuntimeError):
                    mapper.update(scene_frame(sequence=2))
            self.assertIs(mapper._state, previous)
            self.assertEqual(len(previous.evidence), 1)
            second = mapper.update(scene_frame(sequence=2))
        self.assertEqual(first.regions[0].observed_object_classes, ('cup',))
        self.assertIsNone(second.regions[0].room_type)
        self.assertEqual(second.semantic_objects[0].confirmation_count, 2)
        self.assertEqual(first.semantic_objects[0].confirmation_count, 1)
        self.assertIsNone(second.target_claim)
        mapper.reset(request())
        self.assertEqual(mapper._state.track_regions, {})
        self.assertEqual(mapper._state.evidence, {})
