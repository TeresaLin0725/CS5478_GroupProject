"""Offline fusion diagnostics; no profiler is installed in normal mapper runs."""

import cProfile
import gzip
from hashlib import sha256
import json
from math import ceil
from pathlib import Path
import platform
import pstats
from statistics import median
import sys
from time import perf_counter
from unittest.mock import patch

from ..experiments.logging import dumps
from . import fusion, mapper


BUFFER_NAMES = ('occupancy', 'coverage', 'free_masks', 'floor_seen', 'clear_counts')


def fingerprint(value):
    return sha256(dumps(value).encode('utf-8')).hexdigest()


def summarize(values):
    """Nearest-rank P95, including small samples (no interpolated tail)."""
    ordered = sorted(values)
    return dict(count=len(ordered), median_s=median(ordered),
                p95_s=ordered[ceil(.95 * len(ordered)) - 1],
                min_s=ordered[0], max_s=ordered[-1])


def profile_breakdown(profiler):
    """Disjoint cProfile buckets, not estimates of unprofiled wall time.

    Generator cumulative time covers execution while resumed, not time spent
    suspended in the caller. Remaining time includes endpoint and voxel evidence
    writes, setup and Python loop overhead. Sampling and projection share a
    function and are deliberately reported together.
    """
    stats = pstats.Stats(profiler)

    def cumulative(filename, name):
        return sum(record[3] for key, record in stats.stats.items()
                   if Path(key[0]).name == filename and key[2] == name)

    stages = dict(
        sampling_projection_s=cumulative('projection.py', 'iter_depth_world'),
        ray_traversal_s=cumulative('fusion.py', '_ray_voxels'),
        map_finalization_s=cumulative('fusion.py', '_finalize_free_space'))
    stages['endpoint_evidence_and_other_s'] = max(0., stats.total_tt - sum(stages.values()))
    return dict(instrumented_total_s=stats.total_tt, stages=stages)


def replay_fusion(sequence, observations, profile_directory=None, capture=False):
    """Use the real mapper's initialization, envelope and timestamp semantics.

    The module patch is confined to this sequential offline diagnostic. Only the
    fusion call is timed/profiled; copies, hashing, I/O, validation, detection and
    frontier generation are outside its timing boundary.
    """
    from ..fixtures.sequence import mapper_init

    subject = mapper.RGBDMapper()
    subject.reset(mapper_init(sequence.manifest, sequence.initial_clearance_cells,
                              sequence.clearance_verification))
    implementation = fusion.fuse_observation
    records, snapshots = [], []

    def measured(*args, **kwargs):
        observation, cfg = args[:2]
        profiler = cProfile.Profile() if profile_directory is not None else None
        started = perf_counter()
        if profiler is None:
            implementation(*args, **kwargs)
        else:
            profiler.runcall(implementation, *args, **kwargs)
        elapsed = perf_counter() - started
        record = dict(sequence=observation.sequence, timestamp_s=observation.timestamp_s,
                      fusion_wall_s=elapsed,
                      image_pixels=observation.depth_image.width * observation.depth_image.height,
                      sampled_pixel_slots=(len(range(0, observation.depth_image.width, cfg.depth_pixel_stride))
                                           * len(range(0, observation.depth_image.height, cfg.depth_pixel_stride))))
        if profiler is not None:
            record.update(profile_breakdown(profiler))
            profiler.dump_stats(str(profile_directory / ('frame-%06d.prof' % observation.sequence)))
        records.append(record)
        if capture:
            # The mapper always owns a clear-count grid, even when clearing is disabled.
            buffers = dict(zip(BUFFER_NAMES[:4], args[3:7]))
            buffers['clear_counts'] = kwargs.get('clear_counts')
            if buffers['clear_counts'] is None:
                buffers['clear_counts'] = [[0] * cfg.width_cells for _ in range(cfg.height_cells)]
            snapshots.append(dict(sequence=observation.sequence,
                                  buffers={key: [row[:] for row in value]
                                           for key, value in buffers.items()}))

    with patch.object(mapper, 'fuse_observation', measured):
        for observation in observations:
            subject.update(observation)
    return records, snapshots


def compare_snapshots(expected, actual):
    """Compare every private evidence cell, retaining a bounded diff example."""
    if len(expected) != len(actual):
        return dict(equal=False, reason='frame count differs')
    differences = []
    for old, new in zip(expected, actual):
        if old['sequence'] != new['sequence']:
            return dict(equal=False, reason='capture sequences differ')
        for name in BUFFER_NAMES:
            before, after = old['buffers'][name], new['buffers'][name]
            if before == after:
                continue
            if len(before) != len(after) or any(len(a) != len(b) for a, b in zip(before, after)):
                differences.append(dict(sequence=new['sequence'], buffer=name, reason='shape differs'))
                continue
            changed = [(r, c) for r, (a, b) in enumerate(zip(before, after))
                       for c, (x, y) in enumerate(zip(a, b)) if x != y]
            r, c = changed[0]
            differences.append(dict(sequence=new['sequence'], buffer=name,
                                    changed_cells=len(changed), first_cell=[r, c],
                                    expected=before[r][c], actual=after[r][c]))
    return dict(equal=not differences, differences=differences)


def read_baseline(path):
    with gzip.open(str(path), 'rt', encoding='utf-8') as stream:
        baseline = json.load(stream)
    if baseline.get('format_version') != 1:
        raise ValueError('unsupported fusion baseline version')
    return baseline


def state_transitions(snapshots):
    """Expose whether a recording actually exercises obstacle clearing."""
    result, previous = [], None
    for item in snapshots:
        buffers = item['buffers']
        occupied = buffers['occupancy']
        cleared = 0 if previous is None else sum(
            old == 100 and new == 0 for a, b in zip(previous, occupied) for old, new in zip(a, b))
        result.append(dict(sequence=item['sequence'],
                           occupied_cells=sum(value == 100 for row in occupied for value in row),
                           obstacle_to_free_cells=cleared,
                           pending_clear_cells=sum(value > 0 for row in buffers['clear_counts'] for value in row)))
        previous = occupied
    return result


def run_diagnostic(sequence, output, repeats=3, warmups=1, max_frames=None, baseline=None):
    if repeats < 1 or warmups < 0 or (max_frames is not None and max_frames < 1):
        raise ValueError('repeats/max_frames must be positive; warmups must be nonnegative')
    observations = sequence.observations[:max_frames]
    identity = dict(sequence_sha256=sequence.digest,
                    manifest_sha256=fingerprint(sequence.manifest),
                    capture_sequences=[item.sequence for item in observations])
    previous = read_baseline(baseline) if baseline is not None else None
    if previous is not None and previous['input'] != identity:
        raise ValueError('baseline inputs/configuration differ; refuse a speed/equality comparison')
    source_files = ('fusion.py', 'projection.py', 'mapper.py', 'observations.py')
    source_hashes = {name: sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
                     for name in source_files}
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for _ in range(warmups):
        replay_fusion(sequence, observations)
    timings, reference, repeat_checks = [], None, []
    for _ in range(repeats):
        records, snapshots = replay_fusion(sequence, observations, capture=True)
        timings.append(records)
        if reference is None:
            reference = snapshots
        else:
            repeat_checks.append(compare_snapshots(reference, snapshots))
    profile_directory = output / 'profiles'
    profile_directory.mkdir()
    profiled, profile_snapshots = replay_fusion(sequence, observations, profile_directory, capture=True)
    consistency = compare_snapshots(reference, profile_snapshots)
    comparison = compare_snapshots(previous['snapshots'], reference) if previous is not None else None
    unprofiled_summary = summarize([frame['fusion_wall_s'] for run in timings for frame in run])
    profiled_summary = summarize([frame['fusion_wall_s'] for frame in profiled])
    report = dict(format_version=1, input=identity,
                  runtime=dict(python=sys.version, executable=sys.executable, platform=platform.platform()),
                  source_sha256=source_hashes,
                  mapping=sequence.manifest.mapping, robot=sequence.manifest.robot,
                  warmup_passes=warmups, measured_passes=repeats,
                  unprofiled_summary=unprofiled_summary, profiled_summary=profiled_summary,
                  profiler_wall_ratio=profiled_summary['median_s'] / unprofiled_summary['median_s'],
                  unprofiled_runs=timings, profiled_frames=profiled,
                  profile_matches_unprofiled=consistency, repeat_checks=repeat_checks,
                  baseline_comparison=comparison,
                  state_sha256=[dict(sequence=item['sequence'], sha256=fingerprint(item['buffers']))
                                for item in reference],
                  state_transitions=state_transitions(reference),
                  timing_scope='fusion only; excludes loading, mapper validation/copies, detector, frontiers and snapshot serialization',
                  limitations=['cProfile perturbs Python/generator costs; stage shares are diagnostic only',
                               'sampling and coordinate projection remain combined in iter_depth_world',
                               'recordings without moving obstacles do not validate dynamic clearing'])
    if previous is not None:
        report['baseline_median_ratio'] = (previous['unprofiled_summary']['median_s']
                                           / unprofiled_summary['median_s'])
        report['baseline_runtime'] = previous['runtime']
    archive = dict(format_version=1, input=identity, snapshots=reference,
                   unprofiled_summary=unprofiled_summary, runtime=report['runtime'],
                   source_sha256=report['source_sha256'])
    with gzip.open(str(output / 'baseline.json.gz'), 'wt', encoding='utf-8') as stream:
        stream.write(dumps(archive) + '\n')
    (output / 'report.json').write_text(dumps(report) + '\n', encoding='utf-8')
    equal = (consistency['equal'] and all(item['equal'] for item in repeat_checks)
             and (comparison is None or comparison['equal']))
    return report, equal
