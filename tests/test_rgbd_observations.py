"""Complete sensor-boundary checks before recording or mapper integration."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from rgbd_support import load_wall_observation
from frontier_search.fixtures.recording import save_rgbd_recording
from frontier_search.perception.observations import validate_rgbd_observation
from frontier_search.validation import ContractError


class RGBDObservationTests(unittest.TestCase):
    def setUp(self):
        self.observation = load_wall_observation()

    def test_supported_capture_and_zero_initial_sequence(self):
        self.assertIsNone(validate_rgbd_observation(self.observation))
        validate_rgbd_observation(replace(self.observation, sequence=0))

    def test_bad_identity_time_and_pose_rejected(self):
        for fields in ({"episode_id": " "}, {"sequence": True}, {"sequence": -1},
                       {"timestamp_s": -1}, {"timestamp_s": True}, {"robot_pose": None}):
            with self.subTest(fields=fields), self.assertRaises(ContractError):
                validate_rgbd_observation(replace(self.observation, **fields))
        malformed = load_wall_observation()
        object.__setattr__(malformed.robot_pose, "x_m", float("nan"))
        with self.assertRaises(ContractError):
            validate_rgbd_observation(malformed)

    def test_misaligned_and_unsupported_rgb_rejected(self):
        color = self.observation.color_image
        for fields in ({"width": color.width + 1}, {"encoding": "bgr8"},
                       {"data": color.data[:-1]}, {"width": True}):
            with self.subTest(fields=fields), self.assertRaises(ContractError):
                validate_rgbd_observation(replace(self.observation, color_image=replace(color, **fields)))

    def test_invalid_capture_does_not_create_recording_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            bad = replace(self.observation, sequence=-1)
            with self.assertRaises(ContractError):
                save_rgbd_recording(output, bad, {})
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
