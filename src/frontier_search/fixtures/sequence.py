"""Portable RGB-D sequences with frozen configuration and content checksums."""

from dataclasses import dataclass
from hashlib import sha256
import json
from pathlib import Path
from typing import Mapping, Optional, Tuple

from ..contracts import GridCell, MapperInit, RGBDObservation
from ..experiments.logging import dumps
from .clearance import footprint_cells, validate_clearance
from ..perception.mapper import RGBDMapper
from ..perception.observations import validate_rgbd_observation
from ..runner.manifest import EpisodeManifest, load_manifest
from ..validation import ContractError, require
from .recording import load_rgbd_recording, save_rgbd_recording


FRAME_FILES = ('observation.json', 'rgb.bin', 'depth.bin')


@dataclass(frozen=True)
class RecordingSequence:
    manifest: EpisodeManifest
    observations: Tuple[RGBDObservation, ...]
    initial_clearance_cells: Tuple[GridCell, ...]
    clearance_provenance: str
    digest: str
    clearance_verification: Optional[Mapping[str, object]] = None


def mapper_init(manifest: EpisodeManifest, clearance: Tuple[GridCell, ...], verification=None) -> MapperInit:
    validate_clearance(manifest, clearance, verification)
    return MapperInit(manifest.episode_id, manifest.target_class, manifest.perception_seed,
                      manifest.mapping, manifest.observation, manifest.robot,
                      manifest.region_priors, clearance, manifest.oracle_room_labels)


def validate_sequence(manifest, observations, clearance, provenance, verification=None):
    manifest.validate()
    require(not manifest.enable_target_evaluation, 'W2 replay requires target evaluation disabled')
    require(isinstance(provenance, str) and bool(provenance.strip()), 'missing spawn-clearance provenance')
    require(bool(observations), 'recording sequence must contain at least one frame')
    RGBDMapper().reset(mapper_init(manifest, clearance, verification))
    first = observations[0]
    last_sequence, last_time = -1, -1.0
    for frame in observations:
        validate_rgbd_observation(frame)
        require(frame.episode_id == manifest.episode_id, 'recording episode differs from manifest')
        require(frame.sequence > last_sequence and frame.timestamp_s >= last_time,
                'recording frames must have increasing sequence and nondecreasing time')
        require(frame.camera_intrinsics == first.camera_intrinsics, 'recording camera intrinsics changed')
        require(abs(frame.robot_pose.z_m - first.robot_pose.z_m)
                <= manifest.mapping.base_elevation_margin_m + 1e-6,
                'flat-floor recording base elevation exceeds configured margin')
        last_sequence, last_time = frame.sequence, frame.timestamp_s
    require(first.robot_pose == manifest.start_pose, 'first capture pose differs from manifest start pose')


def _digest(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _child(root: Path, name: str) -> Path:
    require(isinstance(name, str) and bool(name) and not Path(name).is_absolute(),
            'sequence paths must be relative')
    path = (root / name).resolve()
    try:
        path.relative_to(root.resolve())
    except ValueError as exc:
        raise ContractError('sequence path escapes package directory') from exc
    return path


def write_sequence(directory: Path, manifest: EpisodeManifest,
                   observations: Tuple[RGBDObservation, ...], clearance: Tuple[GridCell, ...],
                   clearance_provenance: str,
                   sources: Optional[Tuple[Mapping[str, object], ...]] = None,
                   verification=None) -> None:
    """Create a new self-contained package; write its index last, never overwrite."""
    validate_sequence(manifest, observations, clearance, clearance_provenance, verification)
    require(sources is None or len(sources) == len(observations), 'source metadata count differs from frames')
    if sources is not None:
        require(all(isinstance(source, dict) for source in sources), 'frame sources must be JSON objects')
        dumps(sources)  # Validate serializability before creating the package.
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    manifest_path = directory / 'manifest.json'
    manifest_path.write_text(dumps(manifest) + '\n', encoding='utf-8')
    records = []
    for index, frame in enumerate(observations):
        relative = 'frames/%06d' % index
        destination = directory / relative
        source = sources[index] if sources is not None else {
            'kind': manifest.source, 'scene_id': manifest.scene_id, 'trajectory': 'recorded_not_policy_executed'}
        save_rgbd_recording(destination, frame, source)
        records.append({'path': relative, 'sha256': {name: _digest(destination / name) for name in FRAME_FILES}})
    index = dict(format_version=2 if verification is not None else 1, manifest='manifest.json', manifest_sha256=_digest(manifest_path),
                 initial_clearance_cells=clearance, clearance_provenance=clearance_provenance, frames=records)
    if verification is not None:
        index['clearance_verification'] = verification
    (directory / 'sequence.json').write_text(dumps(index) + '\n', encoding='utf-8')


def load_sequence(directory: Path) -> RecordingSequence:
    """Verify every input checksum and boundary before returning any observations."""
    directory = Path(directory).resolve()
    try:
        index_path = directory / 'sequence.json'
        index = json.loads(index_path.read_text(encoding='utf-8'))
        require(isinstance(index, dict), 'invalid sequence index')
        version = index.get('format_version')
        require(type(version) is int and version in (1, 2), 'unsupported recording sequence version')
        require(set(index) == {
            'format_version', 'manifest', 'manifest_sha256', 'initial_clearance_cells',
            'clearance_provenance', 'frames'} | ({'clearance_verification'} if version == 2 else set()),
                'invalid sequence index fields')
        manifest_path = _child(directory, index['manifest'])
        require(_digest(manifest_path) == index['manifest_sha256'], 'manifest checksum mismatch')
        manifest = load_manifest(manifest_path)
        require((version == 2) == (manifest.initial_clearance_mode == 'verified_start_area'),
                'sequence version differs from initial clearance mode')
        verification = index.get('clearance_verification')
        require(isinstance(index['frames'], list), 'sequence frames must be a list')
        frames, seen = [], set()
        for entry in index['frames']:
            require(isinstance(entry, dict) and set(entry) == {'path', 'sha256'}, 'invalid frame entry')
            folder = _child(directory, entry['path'])
            require(folder not in seen, 'duplicate frame directory')
            seen.add(folder)
            require(isinstance(entry['sha256'], dict) and set(entry['sha256']) == set(FRAME_FILES),
                    'missing frame checksums')
            for filename in FRAME_FILES:
                # Resolve each file as well, rejecting symlinks outside the package.
                path = _child(directory, str((folder / filename).relative_to(directory)))
                require(_digest(path) == entry['sha256'][filename], 'frame checksum mismatch: ' + filename)
            frames.append(load_rgbd_recording(folder))
        clearance = tuple(GridCell(**cell) for cell in index['initial_clearance_cells'])
        observations = tuple(frames)
        validate_sequence(manifest, observations, clearance, index['clearance_provenance'], verification)
        return RecordingSequence(manifest, observations, clearance, index['clearance_provenance'], _digest(index_path), verification)
    except ContractError:
        raise
    except (OSError, UnicodeError, KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ContractError('invalid recording sequence: %s' % exc) from exc
