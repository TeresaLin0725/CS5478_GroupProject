"""Recording replay uses only the standard library."""

from pathlib import Path
from dataclasses import replace
import json
import tempfile
import unittest

from frontier_search.fixtures.recording import load_rgbd_recording, save_rgbd_recording
from frontier_search.validation import ContractError
from tests.rgbd_support import load_wall_observation


class RGBDRecordingTests(unittest.TestCase):
    def test_optional_control_step_odometry_round_trip_and_legacy_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)/'capture'
            observation=replace(load_wall_observation(),odometry_linear_speed_mps=.01,odometry_angular_speed_radps=.12)
            save_rgbd_recording(directory,observation,{})
            self.assertEqual(load_rgbd_recording(directory),observation)
            path=directory/'observation.json'
            document=json.loads(path.read_text(encoding='utf-8'))
            document['observation'].pop('odometry_linear_speed_mps')
            document['observation'].pop('odometry_angular_speed_radps')
            path.write_text(json.dumps(document),encoding='utf-8')
            self.assertIsNone(load_rgbd_recording(directory).odometry_linear_speed_mps)

    def test_round_trip_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "capture"
            observation = load_wall_observation()
            save_rgbd_recording(directory, observation, {"kind": "synthetic_test"})
            self.assertEqual(load_rgbd_recording(directory), observation)
            with self.assertRaises(FileExistsError):
                save_rgbd_recording(directory, observation, {})
            self.assertEqual(load_rgbd_recording(directory), observation)

    def test_truncated_depth_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "capture"
            save_rgbd_recording(directory, load_wall_observation(), {})
            (directory / "depth.bin").write_bytes(b"")
            with self.assertRaises(ValueError):
                load_rgbd_recording(directory)

    def test_bad_metadata_uses_contract_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "capture"
            save_rgbd_recording(directory, load_wall_observation(), {})
            path = directory / "observation.json"
            original = path.read_text(encoding="utf-8")
            for change in (
                lambda d: d.update(format_version=True),
                lambda d: d["observation"]["color_image"].update(encoding="bgr8"),
                lambda d: d["observation"]["depth_image"].update(width=99),
                lambda d: d["observation"].pop("robot_pose"),
                lambda d: d["observation"].update(odometry_linear_speed_mps=.1),
            ):
                document = json.loads(original)
                change(document)
                path.write_text(json.dumps(document), encoding="utf-8")
                with self.assertRaises(ContractError):
                    load_rgbd_recording(directory)
