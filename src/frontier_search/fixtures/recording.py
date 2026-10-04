"""Save and replay one RGBDObservation without simulator dependencies. Used for offline mapper development and evaluation of target detection and tracking."""

from dataclasses import asdict
import json
from pathlib import Path
from typing import Mapping

from ..contracts import Pose, RGBDObservation
from ..sensors import CameraIntrinsics, ImageBuffer, Transform
from ..perception.observations import validate_rgbd_observation
from ..validation import ContractError


# Preserve an RGB-D observation and its source information for later replay.
def save_rgbd_recording(directory: Path, observation: RGBDObservation,
                        source: Mapping[str, object]) -> None:
    """Write a new recording directory; refuse to overwrite existing captures."""
    validate_rgbd_observation(observation)
    directory = Path(directory)
    record = asdict(observation)
    rgb = record["color_image"].pop("data")
    depth = record["depth_image"].pop("data")
    metadata = json.dumps({"format_version": 1, "source": dict(source),
                           "observation": record}, indent=2, allow_nan=False)
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "rgb.bin").write_bytes(rgb)
    (directory / "depth.bin").write_bytes(depth)
    (directory / "observation.json").write_text(metadata, encoding="utf-8")


# Restore a recorded RGB-D observation for perception development.
def load_rgbd_recording(directory: Path) -> RGBDObservation:
    """Read the saved buffers and calibration into the shared immutable contract."""
    directory = Path(directory)
    try:
        document = json.loads((directory / "observation.json").read_text(encoding="utf-8"))
        if type(document["format_version"]) is not int or document["format_version"] != 1:
            raise ValueError("unsupported RGB-D recording version")
        record = document["observation"]
        rgb_meta, depth_meta = record["color_image"], record["depth_image"]
        rgb = (directory / "rgb.bin").read_bytes()
        depth = (directory / "depth.bin").read_bytes()
        calibration = record["camera_intrinsics"]
        calibration["distortion"] = tuple(calibration["distortion"])
        observation = RGBDObservation(
            episode_id=record["episode_id"], sequence=record["sequence"],
            timestamp_s=record["timestamp_s"],
            color_image=ImageBuffer(data=rgb, **rgb_meta),
            depth_image=ImageBuffer(data=depth, **depth_meta),
            camera_intrinsics=CameraIntrinsics(**calibration),
            camera_to_world=Transform(tuple(tuple(row) for row in record["camera_to_world"]["matrix"])),
            robot_pose=Pose(**record["robot_pose"]),
        )
        validate_rgbd_observation(observation)
        return observation
    except ContractError:
        raise
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ContractError("invalid RGB-D recording: %s" % exc) from exc
