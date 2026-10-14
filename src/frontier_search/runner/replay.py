"""Offline recorded RGB-D -> A mapper -> B assessment -> C baseline decisions."""

import argparse
from hashlib import sha256
from html import escape
import json
from math import cos, hypot, sin
from pathlib import Path
from typing import Tuple

from ..contracts import CandidateEvaluationRequest, GridCell, NavigatorInit, PolicyName
from ..experiments.logging import dumps
from ..fixtures.analytic_rgbd import render_frame
from ..fixtures.recording import load_rgbd_recording
from ..fixtures.sequence import footprint_cells, load_sequence, mapper_init, write_sequence
from ..navigation.navigator import TeamNavigator
from ..perception.mapper import RGBDMapper
from ..perception.detection import YOLODetector
from ..policies.baselines import BaselinePolicy
from ..runner.episode import build_context, decide
from ..runner.manifest import load_manifest
from ..validation import require


BASELINES = ('nearest', 'information_gain', 'random')


class AssessmentControl:
    """Read-only recorded pose/time; any attempt to actuate is a replay error."""
    def __init__(self, observation):
        self.observation = observation

    def current_pose(self):
        return self.observation.robot_pose

    def sim_time_s(self):
        return self.observation.timestamp_s

    def set_velocity(self, linear_mps, angular_radps):
        raise RuntimeError('offline replay cannot execute velocity commands')


def generate_synthetic(directory: Path, manifest_path: Path) -> None:
    """Generate a reproducible moving/scanning sequence with explicit fixture geometry."""
    manifest = load_manifest(manifest_path)
    require(manifest.source == 'synthetic' and manifest.scene_id == 'analytic_floor_wall',
            'analytic generator requires the labelled synthetic manifest')
    start = manifest.start_pose
    require(start.z_m == manifest.mapping.floor_height_m and start.yaw_rad == 0.,
            'analytic fixture requires a floor-level base and initial yaw zero')
    radius = max(hypot(p.x_m, p.y_m) for p in manifest.robot.footprint_base) + manifest.robot.clearance_margin_m
    require(start.x_m + .4 + radius < 4.0, 'analytic trajectory intersects the wall')
    clearance = footprint_cells(manifest)
    require(all(manifest.mapping.origin_x_m + (cell.column + 1) * manifest.mapping.resolution_m < 4.
                for cell in clearance), 'analytic spawn cell overlaps the wall')
    trajectory = ((0., 0.), (0., .55), (0., -.55), (0., 0.), (.4, 0.), (.4, .5), (.4, -.5))
    frames = tuple(render_frame(sequence=i + 1, x=start.x_m + dx, y=start.y_m,
                               yaw=yaw, floor_z=start.z_m, pitch_down=.65,
                               episode_id=manifest.episode_id)
                   for i, (dx, yaw) in enumerate(trajectory))
    write_sequence(directory, manifest, frames, clearance,
                   'Analytic flat-floor fixture: initial footprint plus margin checked against x=4 wall; '
                   'development robot, no simulator or physical execution.')


def _hash(value):
    return sha256(dumps(value).encode('utf-8')).hexdigest()


def pack_recordings(directory: Path, manifest_path: Path, frame_paths: Tuple[Path, ...],
                    clearance_path: Path) -> None:
    """Bundle existing per-frame recordings, preserving their original source metadata."""
    manifest = load_manifest(manifest_path)
    clearance = json.loads(Path(clearance_path).read_text(encoding='utf-8'))
    expected = {'cells', 'provenance'}
    if manifest.initial_clearance_mode == 'verified_start_area':
        expected.add('verification')
    require(isinstance(clearance, dict) and set(clearance) == expected,
            'clearance file requires cells and provenance')
    cells = tuple(GridCell(**cell) for cell in clearance['cells'])
    observations = tuple(load_rgbd_recording(path) for path in frame_paths)
    sources = tuple(json.loads((Path(path) / 'observation.json').read_text(encoding='utf-8'))['source']
                    for path in frame_paths)
    write_sequence(directory, manifest, observations, cells, clearance['provenance'], sources, clearance.get('verification'))


def _assess(sequence, policy_names, detector=None):
    m = sequence.manifest
    mapper = RGBDMapper(detector)
    mapper.reset(mapper_init(m, sequence.initial_clearance_cells, sequence.clearance_verification))
    control = AssessmentControl(sequence.observations[0])
    navigator = TeamNavigator(control)
    navigator.reset(NavigatorInit(m.episode_id, m.robot, m.observation, m.navigation))
    policies = {}
    for name in policy_names:
        policy = BaselinePolicy(PolicyName(name))
        policy.reset(m.episode_id, m.policy_seed, m.policy)
        policies[name] = policy
    records = []
    for frame in sequence.observations:
        control.observation = frame
        state = mapper.update(frame)
        assessment = navigator.evaluate_candidates(CandidateEvaluationRequest(m.episode_id, state))
        context = build_context(state, assessment)
        decisions = {name: decide(policy, state, context, oracle_room_labels=m.oracle_room_labels)
                     for name, policy in policies.items()}
        known = sum(v != -1 for row in state.occupancy_map.data for v in row)
        record = dict(event='rgbd_replay_frame', mode='offline_selection_only', source=m.source,
                      episode_id=m.episode_id, sequence=frame.sequence, state=state,
                      initial_clearance_mode=m.initial_clearance_mode,
                      initial_clearance_radius_m=m.initial_clearance_radius_m,
                      initial_clearance_cells=len(sequence.initial_clearance_cells),
                      state_sha256=_hash(state), assessment=assessment,
                      eligible_frontier_ids=context.eligible_frontier_ids, decisions=decisions,
                      known_cells=known, explored_cells=sum(sum(row) for row in state.explored_map.data))
        records.append(record)
    return records


def _map_svg(record):
    state, assessment = record['state'], record['assessment']
    grid = state.occupancy_map
    w, h = grid.width_cells, grid.height_cells
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="600" role="img">' % (w, h),
           '<title>Observed map and assessed frontier approach poses; world +y points up</title>']
    for r, row in enumerate(grid.data):
        for c, value in enumerate(row):
            color = {0: '#f8fafc', 100: '#263238', -1: '#d1d5db'}[value]
            if value == -1 and state.explored_map.data[r][c]:
                color = '#b8dce6'
            svg.append('<rect x="%d" y="%d" width="1" height="1" fill="%s"/>' % (c, h-r-1, color))
    reachability = {a.frontier_id: a.reachable for a in assessment.assessments}
    def xy(p):
        return ((p.x_m - grid.origin_x_m) / grid.resolution_m,
                h - (p.y_m - grid.origin_y_m) / grid.resolution_m)
    for frontier in state.frontiers:
        for cell in frontier.support_cells:
            svg.append('<rect x="%d" y="%d" width="1" height="1" fill="none" stroke="#d97706" stroke-width=".09"/>'
                       % (cell.column, h-cell.row-1))
        x, y = xy(frontier.approach_pose)
        color = {True: '#15803d', False: '#dc2626', None: '#d97706'}[reachability[frontier.frontier_id]]
        svg.append('<circle cx="%.4f" cy="%.4f" r=".2" fill="%s"><title>%s</title></circle>'
                   % (x, y, color, escape(frontier.frontier_id)))
    x, y = xy(state.robot_pose)
    svg.append('<circle cx="%.4f" cy="%.4f" r=".25" fill="#2563eb"/>' % (x, y))
    svg.append('<path d="M %.4f %.4f L %.4f %.4f" stroke="#2563eb" stroke-width=".12"/>'
               % (x, y, x + cos(state.robot_pose.yaw_rad), y - sin(state.robot_pose.yaw_rad)))
    svg.append('</svg>')
    return '\n'.join(svg)


def _html(report, records):
    rows = []
    for index, record in enumerate(records):
        selected = ', '.join(name + ': ' + (decision.subgoal.frontier_id if decision.subgoal else decision.stop_reason)
                             for name, decision in record['decisions'].items())
        assessment = record['assessment'].assessments
        rows.append('<tr><td><a href="frames/%06d.svg">%d</a></td><td>%d</td><td>%d</td>'
                    '<td>%d</td><td>%d</td><td>%s</td></tr>' %
                    (index, record['sequence'], record['known_cells'], record['explored_cells'],
                     len(assessment), sum(a.reachable is True for a in assessment), escape(selected)))
    return (('<!doctype html><html lang="en"><meta charset="utf-8"><title>RGB-D replay</title>'
            '<style>body{font:16px system-ui;max-width:1050px;margin:32px auto;padding:0 20px;color:#1e293b}'
            'table{border-collapse:collapse;width:100%%}td,th{padding:9px;text-align:left;border-bottom:1px solid #cbd5e1}'
            'svg{max-width:100%%;border:1px solid #cbd5e1}a{color:#1d4ed8}</style>'
            '<h1>RGB-D replay</h1><p>Source: <strong>%s</strong>. Fixed recorded trajectory; decisions are not executed.</p>'
            '<p>Initial prior: %s; radius limit: %s m; seed cells: %s.</p>'
            '<p>Repeat verified: %s. <a href="report.json">Report</a> · <a href="events.jsonl">Full states and decisions</a></p>'
            '<table><thead><tr><th>Frame/map</th><th>Known cells</th><th>Observed cells</th><th>Frontiers</th>'
            '<th>Reachable</th><th>Selections</th></tr></thead><tbody>%s</tbody></table>'
            '<h2>Final observed map</h2>%s<p>White: free; dark: occupied; grey: unknown; blue-grey: observed but unknown clearance. '
            'Orange outlines: frontier support. Approach dots: green reachable, red rejected, orange inconclusive. '
            'Blue: recorded robot pose. World +y points up.</p></html>') %
            (escape(report['source']), escape(report['initial_clearance_mode']),
             report['initial_clearance_radius_m'], report['initial_clearance_cells'],
             str(report['repeat_verified']).lower(), ''.join(rows), _map_svg(records[-1])))


def replay(directory: Path, output: Path, policy_names: Tuple[str, ...] = BASELINES,
           verify_repeat: bool = True, detector_weights: Path = None):
    require(bool(policy_names) and len(set(policy_names)) == len(policy_names)
            and all(name in BASELINES for name in policy_names), 'replay requires unique offline baseline names')
    sequence = load_sequence(directory)
    detector = YOLODetector(str(detector_weights)) if detector_weights is not None else None
    detector_metadata = (dict(weights=str(detector_weights),
        weights_sha256=sha256(Path(detector_weights).read_bytes()).hexdigest())
        if detector_weights is not None else None)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = dict(mode='offline_rgbd_replay', source=sequence.manifest.source,
                  episode_id=sequence.manifest.episode_id, input_sha256=sequence.digest,
                  policies=policy_names, repeat_verified=False, status='running',
                  detector=detector_metadata,
                  initial_clearance_mode=sequence.manifest.initial_clearance_mode,
                  initial_clearance_radius_m=sequence.manifest.initial_clearance_radius_m,
                  initial_clearance_cells=len(sequence.initial_clearance_cells),
                  clearance_verification=sequence.clearance_verification)
    try:
        records = _assess(sequence, policy_names, detector)
        result_digest = _hash(records)
        if verify_repeat:
            repeated = _assess(sequence, policy_names, detector)
            require(_hash(repeated) == result_digest,
                    'repeat replay differs: inspect geometry, policy seeds and assessment timeouts')
            report['repeat_verified'] = True
        report.update(status='complete', result_sha256=result_digest, frames=len(records),
            first_known_cells=records[0]['known_cells'], final_known_cells=records[-1]['known_cells'],
            first_explored_cells=records[0]['explored_cells'], final_explored_cells=records[-1]['explored_cells'],
            reachable_assessments=sum(a.reachable is True for r in records for a in r['assessment'].assessments),
            rejected_assessments=sum(a.reachable is False for r in records for a in r['assessment'].assessments),
            inconclusive_assessments=sum(a.reachable is None for r in records for a in r['assessment'].assessments),
            recorded_translation_m=sum(hypot(b.robot_pose.x_m-a.robot_pose.x_m, b.robot_pose.y_m-a.robot_pose.y_m)
                                       for a, b in zip(sequence.observations, sequence.observations[1:])))
        (output / 'manifest.json').write_text(dumps(sequence.manifest) + '\n', encoding='utf-8')
        (output / 'events.jsonl').write_text(''.join(dumps(record) + '\n' for record in records), encoding='utf-8')
        (output / 'frames').mkdir()
        for index, record in enumerate(records):
            (output / 'frames' / ('%06d.svg' % index)).write_text(_map_svg(record), encoding='utf-8')
        (output / 'report.html').write_text(_html(report, records), encoding='utf-8')
    except Exception as exc:
        report.update(status='failed', error_type=type(exc).__name__, error=str(exc))
        (output / 'report.json').write_text(dumps(report) + '\n', encoding='utf-8')
        raise
    (output / 'report.json').write_text(dumps(report) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    generate = actions.add_parser('generate-synthetic', help='write an explicitly synthetic moving/scanning sequence')
    generate.add_argument('--manifest', type=Path, default=Path('configs/w2.rgbd-synthetic.json'))
    generate.add_argument('--output', type=Path, required=True)
    pack = actions.add_parser('pack', help='bundle existing recordings with a shared manifest and verified spawn mask')
    pack.add_argument('--manifest', type=Path, required=True)
    pack.add_argument('--frames', type=Path, nargs='+', required=True, help='frame directories in capture order')
    pack.add_argument('--clearance', type=Path, required=True, help='JSON object with cells and provenance')
    pack.add_argument('--output', type=Path, required=True)
    run = actions.add_parser('run', help='map and assess a recorded sequence without executing decisions')
    run.add_argument('--dataset', type=Path, required=True)
    run.add_argument('--output', type=Path, required=True)
    run.add_argument('--policies', choices=BASELINES, nargs='+', default=BASELINES)
    run.add_argument('--no-verify-repeat', action='store_true')
    run.add_argument('--detector-weights', type=Path,
                     help='enable RGB object detection with existing local YOLO weights')
    args = parser.parse_args()
    try:
        if args.action == 'generate-synthetic':
            generate_synthetic(args.output, args.manifest)
            print(dumps({'source': 'synthetic', 'dataset': str(args.output), 'status': 'written'}))
        elif args.action == 'pack':
            pack_recordings(args.output, args.manifest, tuple(args.frames), args.clearance)
            print(dumps({'dataset': str(args.output), 'status': 'written'}))
        else:
            print(dumps(replay(args.dataset, args.output, tuple(args.policies),
                              not args.no_verify_repeat, args.detector_weights)))
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.exit(2, 'replay error: %s\n' % exc)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
