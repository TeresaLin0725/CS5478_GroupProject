"""Single-frame detector integration without simulator or model downloads."""

from dataclasses import replace
from pathlib import Path
from struct import pack
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

from frontier_search.contracts import Point3
from frontier_search.immutable import require_immutable
from frontier_search.perception.detection import RGBDetection, YOLODetector, localize_detection
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.sensors import Transform, ImageBuffer
from frontier_search.validation import ContractError
from test_mapper_update import request, scene_frame


class MapperDetectionTests(unittest.TestCase):
    def setUp(self):
        self.detector = Mock()
        self.detection = RGBDetection('cup', 0.9, (28, 28, 33, 33))
        self.detector.detect.return_value = (self.detection,)
        self.mapper = RGBDMapper(self.detector)
        self.mapper.reset(request())
        frame = scene_frame()
        self.frame = replace(frame, depth_image=replace(frame.depth_image,
            data=pack('<f', 2.0) * (61 * 61)))

    def test_rgb_only_input_and_capture_transform_position(self):
        transform = Transform(((0., 0., 1., 10.), (1., 0., 0., 20.),
                               (0., 1., 0., 30.), (0., 0., 0., 1.)))
        frame = replace(self.frame, camera_to_world=transform)
        state = self.mapper.update(frame)
        self.detector.detect.assert_called_once_with(frame.color_image)
        obj, = state.semantic_objects
        self.assertEqual(obj.position_world, Point3(12., 20., 30.))
        self.assertEqual(obj.confirmation_count, 1)
        self.assertEqual(obj.class_name, 'cup')
        self.assertEqual(obj.first_seen_s, frame.timestamp_s)
        self.assertIsNone(state.target_claim)
        evidence = self.mapper._state.evidence[obj.observation_ids[0]]
        self.assertEqual(evidence.capture_sequence, frame.sequence)
        self.assertEqual(evidence.position_world, obj.position_world)
        require_immutable(state)

    def test_confidence_filter_preserves_other_classes(self):
        self.detector.detect.return_value = (replace(self.detection, confidence=0.79),
            replace(self.detection, class_name='chair', confidence=0.8))
        state = self.mapper.update(self.frame)
        self.assertEqual([o.class_name for o in state.semantic_objects], ['chair'])

    def test_invalid_depth_keeps_position_unknown(self):
        for value in (0., -1., float('nan'), float('inf'), 0.05, 6.):
            frame = replace(self.frame, depth_image=replace(self.frame.depth_image,
                data=pack('<f', value) * (61 * 61)))
            with self.subTest(value=value):
                self.mapper.reset(request())
                obj, = self.mapper.update(frame).semantic_objects
                self.assertIsNone(obj.position_world)

    def test_median_depth_rejects_single_outlier(self):
        depth = bytearray(self.frame.depth_image.data)
        depth[4 * (30 * 61 + 30):4 * (30 * 61 + 31)] = pack('<f', 4.)
        frame = replace(self.frame, depth_image=replace(self.frame.depth_image, data=bytes(depth)))
        self.assertEqual(localize_detection(self.detection, frame, request().mapping_config),
                         localize_detection(self.detection, self.frame, request().mapping_config))

    def test_detector_errors_and_bad_records_leave_episode_unchanged(self):
        original = self.mapper._state
        bad_records = (replace(self.detection, confidence=float('nan')),
                       replace(self.detection, bbox_xyxy_px=(-1, 0, 3, 3)),
                       replace(self.detection, class_name=''), object())
        for bad in bad_records:
            self.detector.detect.return_value = (self.detection, bad)
            with self.assertRaises(ContractError):
                self.mapper.update(self.frame)
            self.assertIs(self.mapper._state, original)
        self.detector.detect.side_effect = RuntimeError('inference failed')
        with self.assertRaises(RuntimeError):
            self.mapper.update(self.frame)
        self.assertIs(self.mapper._state, original)
        self.detector.detect.side_effect = None
        self.detector.detect.return_value = (self.detection,)
        self.assertEqual(self.mapper.update(self.frame).semantic_objects[0].track_id, 'track-000001')

    def test_fresh_frame_records_reset_and_duplicate_rejection(self):
        first = self.mapper.update(self.frame)
        calls = self.detector.detect.call_count
        with self.assertRaises(ContractError):
            self.mapper.update(self.frame)
        self.assertEqual(self.detector.detect.call_count, calls)
        second = self.mapper.update(replace(self.frame, sequence=self.frame.sequence + 1))
        self.assertNotEqual(first.semantic_objects[0].track_id, second.semantic_objects[0].track_id)
        self.assertEqual(first.semantic_objects[0].confirmation_count, 1)
        self.detector.detect.return_value = ()
        self.assertEqual(self.mapper.update(replace(self.frame, sequence=self.frame.sequence + 2)).semantic_objects, ())
        self.mapper.reset(request())
        self.assertEqual(self.mapper._state.tracks, {})
        self.assertEqual(self.mapper._state.evidence, {})

    def test_optional_yolo_adapter_color_conversion_and_clipping(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('optional numpy is unavailable')
        boxes = SimpleNamespace(xyxy=np.array([[-0.2, 0., 2.2, 1.]]),
                                conf=np.array([0.9]), cls=np.array([0.]))
        model = Mock(names={0: 'cup'})
        model.predict.return_value = [SimpleNamespace(boxes=boxes)]
        with tempfile.TemporaryDirectory() as tmp:
            weights = Path(tmp) / 'model.pt'
            weights.write_bytes(b'test stub')
            with patch.dict('sys.modules', {'ultralytics': SimpleNamespace(YOLO=Mock(return_value=model))}):
                detector = YOLODetector(str(weights))
            image = ImageBuffer(2, 1, 'rgb8', bytes([1, 2, 3, 4, 5, 6]))
            result = detector.detect(image)
        self.assertEqual(result, (RGBDetection('cup', 0.9, (0, 0, 2, 1)),))
        self.assertEqual(model.predict.call_args[0][0].tolist(), [[[3, 2, 1], [6, 5, 4]]])

    def test_missing_weights_fail_before_optional_import(self):
        with self.assertRaises(FileNotFoundError):
            YOLODetector('nonexistent-test-weights.pt')
