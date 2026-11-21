"""Recovery overrides must remain distinct from the original policy response."""
from dataclasses import asdict, replace
import importlib.util
import json
from pathlib import Path
import re
import tempfile
import unittest
from unittest.mock import patch

from frontier_search.contracts import PolicyName, SelectionResult, Subgoal
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.experiments.logging import dumps

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('recovery_demo_report', ROOT/'scripts/build_demo_report.py')
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)


class RecoveryReportingTests(unittest.TestCase):
    def setUp(self):
        fixture = load_policy_fixture()
        self.state = fixture.state
        a, b = self.state.frontiers[:2]
        self.original = SelectionResult(self.state.episode_id,
            Subgoal(a.frontier_id, a.approach_pose, self.state.revision, PolicyName.NEAREST),
            None, 'minimum assessed path length')
        self.final = replace(self.original, subgoal=Subgoal(
            b.frontier_id, b.approach_pose, self.state.revision, PolicyName.NEAREST))
        self.decision = dict(state=asdict(self.state), context=asdict(fixture.context),
                             selection=asdict(self.original))

    def test_report_shows_dispatched_goal_and_keeps_original_policy_choice(self):
        row = report.decision_row(self.decision, 1)
        for source in ('epsilon_retry', 'stuck_retry'):
            with self.subTest(source=source):
                report.record_dispatch(row, {'subgoal': asdict(self.final.subgoal)}, source)
                self.assertEqual(row['goal_id'], self.final.subgoal.frontier_id)
                self.assertEqual(row['raw']['selection'], asdict(self.original))
                self.assertEqual(row['raw']['executed_subgoal'], asdict(self.final.subgoal))
                self.assertEqual(row['raw']['action_source'], source)
                self.assertEqual(row['provider'], 'Shared recovery rules')
                self.assertIn(self.original.subgoal.frontier_id, row['reason'])
                self.assertIn(self.final.subgoal.frontier_id, row['reason'])

    def test_report_recognizes_overrides_in_older_dispatch_logs(self):
        self.decision['selection'] = asdict(replace(self.original, subgoal=None,
                                                   stop_reason='no_eligible_frontiers'))
        row = report.decision_row(self.decision, 1)
        report.record_dispatch(row, {'subgoal': asdict(self.final.subgoal)})
        self.assertEqual(row['raw']['action_source'], 'recovery')
        self.assertEqual(row['goal_id'], self.final.subgoal.frontier_id)
        self.assertIn('No navigation goal selected', row['reason'])

    def test_normal_policy_dispatch_retains_its_explanation(self):
        row = report.decision_row(self.decision, 1)
        explanation = row['reason']
        report.record_dispatch(row, {'subgoal': asdict(self.original.subgoal)}, 'policy')
        self.assertEqual(row['reason'], explanation)
        self.assertEqual(row['raw']['action_source'], 'policy')

    def test_integrated_report_links_recovery_override_to_actual_navigation(self):
        summary = dict(decisions=1, attempts=1, simulated_time_s=3., evaluation={'evidence_distances_m': []},
                       source_fingerprint='fixture', policy='nearest', termination='success', wall_time_s=1.,
                       actual_distance_m=1., collisions=0, llm_calls=0, llm_fallbacks=0, profile='apartment')
        files = {'summary.json': summary, 'manifest.json': {'observation': {'arrival_headings_rad': [0.], 'dwell_s': 0.}},
                 'video.json': {'fps': 15, 'frames': []}}
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                rows = [dict(self.decision, event='decision'),
                        dict(event='epsilon_probe', mode='epsilon', frontier_id=self.final.subgoal.frontier_id)]
                if not legacy:
                    rows.append(dict(event='action_selected', action_source='epsilon_retry'))
                rows.extend([dict(event='dispatch', simulated_time_s=2., request={
                                 'goal_id': 'op-1', 'subgoal': asdict(self.final.subgoal)}),
                             dict(event='operation_terminal', feedback={
                                 'goal_id': 'op-1', 'timestamp_s': 3., 'status': 'reached', 'failure_reason': None}),
                             dict(event='target_evaluation', evidence={'detections': []})])
                with patch.object(report, 'read', side_effect=lambda p: files[p.name]), \
                     patch.object(report, 'logs', return_value=rows), tempfile.TemporaryDirectory() as directory:
                    result = report.integrated(None, 'fixture', 'fixture', recording=Path(directory))
                self.assertEqual(result['decisions'][0]['goal_id'], self.final.subgoal.frontier_id)
                self.assertEqual(result['decisions'][0]['raw']['selection'], asdict(self.original))
                self.assertEqual(result['decisions'][0]['raw']['action_source'], 'epsilon_retry')
                self.assertEqual(result['operations'][0]['decision'], 1)
                self.assertTrue(any(f['kind'] == 'Recovery dispatched goal' for f in result['feedback']))

    @unittest.skipUnless(importlib.util.find_spec('PIL'), 'viewer export imports optional Pillow')
    def test_viewer_distinguishes_policy_final_goal_and_actual_dispatch(self):
        from frontier_search.runner.integrated_recording import export_viewer
        rows = [dict(self.decision, event='decision'),
                dict(event='action_selected', simulated_time_s=2., selection=asdict(self.final),
                     policy_selection=asdict(self.original), action_source='epsilon_retry'),
                dict(event='dispatch', simulated_time_s=2., request={'subgoal': asdict(self.final.subgoal)})]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output/'events.jsonl').write_text('\n'.join(dumps(r) for r in rows), encoding='utf-8')
            export_viewer(output, {})
            html = (output/'index.html').read_text(encoding='utf-8')
        payload = json.loads(re.search(r'const data=(.*?);const s=data.summary;', html).group(1))
        texts = [e['text'] for e in payload['events']]
        self.assertIn('C original choice: '+self.original.subgoal.frontier_id, texts[0])
        self.assertIn('Final goal: '+self.final.subgoal.frontier_id, texts[1])
        self.assertIn('Probabilistic recovery retry', texts[1])
        self.assertEqual(texts[2], 'B dispatched: '+self.final.subgoal.frontier_id)


if __name__ == '__main__':
    unittest.main()
