"""Pairing, failure-inclusive estimands, interval and evidence checks for the benchmark harness."""
import json,sys,tempfile,unittest
from copy import deepcopy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from benchmark_protocol import POLICIES,task_specs,validate_pairing,wilson,aggregate,exact_mcnemar,paired_interval
from run_paired_benchmark import result_row,select_jobs
from benchmark_episode import replace_shared,atomic
from unittest.mock import patch

def results():
    return [dict(j,termination='success',success=True,simulated_time_s=100.,actual_distance_m=5.,
        wall_time_s=200.,collisions=0,llm_calls=0,llm_fallbacks=0) for j in task_specs()]

class BenchmarkTests(unittest.TestCase):
    def test_execution_scope_can_select_exactly_the_nine_llm_jobs(self):
        jobs=task_specs()
        self.assertEqual(len(select_jobs(jobs,only_llm=True)),9)
        self.assertEqual({j['policy'] for j in select_jobs(jobs,only_llm=True)},{'llm_guided'})
        self.assertEqual(len(select_jobs(jobs,only_baselines=True)),27)
        with self.assertRaises(ValueError):select_jobs(jobs,only_baselines=True,only_llm=True)

    def test_execution_scope_can_select_explicit_jobs_in_protocol_order(self):
        jobs=task_specs();requested=[jobs[9]['job_id'],jobs[2]['job_id'],jobs[17]['job_id']]
        selected=select_jobs(jobs,job_ids=requested)
        self.assertEqual([j['job_id'] for j in selected],
                         [j['job_id'] for j in jobs if j['job_id'] in requested])
        with self.assertRaisesRegex(ValueError,'duplicate'):
            select_jobs(jobs,job_ids=[requested[0],requested[0]])
        with self.assertRaisesRegex(ValueError,'unknown'):
            select_jobs(jobs,job_ids=['missing'])
        with self.assertRaises(ValueError):
            select_jobs(jobs,only_baselines=True,job_ids=requested)

    def test_call_exports_preserve_nested_enum_values(self):
        from enum import Enum
        class PolicyName(Enum):NEAREST='nearest'
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'llm-calls.json'
            atomic(path,[{'context':{'fallback':PolicyName.NEAREST}}])
            self.assertEqual(json.loads(path.read_text()),[{'context':{'fallback':'nearest'}}])

    def test_windows_reader_sharing_failure_is_retried_without_losing_content(self):
        with tempfile.TemporaryDirectory() as td:
            source=Path(td)/'progress.tmp';target=Path(td)/'progress.json'
            source.write_text('fresh',encoding='utf-8');target.write_text('old',encoding='utf-8')
            original=Path.replace;calls=[]
            def sharing(path,destination):
                calls.append(path)
                if len(calls)<3:raise PermissionError('reader sharing lock')
                return original(path,destination)
            with patch.object(Path,'replace',sharing),patch('benchmark_episode.time.sleep'):
                replace_shared(source,target)
            self.assertEqual(len(calls),3);self.assertEqual(target.read_text(),'fresh')

    def test_every_task_is_paired_across_four_policies(self):
        jobs=task_specs();self.assertEqual(len(jobs),36);self.assertEqual(validate_pairing(jobs),9)
        jobs[0]['start_pose']['yaw_rad']=9.
        with self.assertRaises(AssertionError):validate_pairing(jobs)

    def test_missing_and_duplicate_policies_rejected(self):
        with self.assertRaises(AssertionError):validate_pairing(task_specs()[1:])
        jobs=task_specs();jobs[0]['job_id']=jobs[1]['job_id']
        with self.assertRaises(AssertionError):validate_pairing(jobs)

    def test_fast_failure_is_not_rewarded_and_success_metrics_have_n(self):
        rows=results();rows[0].update(success=False,termination='no_feasible_frontiers',simulated_time_s=2.)
        s=aggregate(rows,task_specs(),draws=100)['paired_policy_stats']['random']
        self.assertEqual(s['n'],9);self.assertEqual(s['successes'],8)
        self.assertAlmostEqual(s['mean_penalized_time_s'],(600+8*100)/9)
        self.assertEqual(s['success_mean_time_s'],100.)

    def test_incomplete_groups_do_not_bias_primary_table(self):
        rows=results()[:3];s=aggregate(rows,task_specs(),draws=20)
        self.assertEqual(s['completed'],3);self.assertEqual(s['complete_pairs'],0)
        self.assertEqual(s['paired_policy_stats']['random']['n'],0)
        self.assertEqual(s['completed_policy_stats']['random']['n'],1)

    def test_intervals_and_exact_paired_test(self):
        lo,hi=wilson(0,9);self.assertAlmostEqual(lo,0);self.assertGreater(hi,.29)
        rows=results()
        for r in rows:
            if r['policy']=='nearest':r['success']=False
        test=exact_mcnemar(rows,'random','nearest')
        self.assertEqual(test['a_only_success'],9);self.assertEqual(test['b_only_success'],0)
        self.assertAlmostEqual(test['p_value'],2/512)
        d=paired_interval(rows,'random','nearest',lambda r:float(r['success']),100)
        self.assertEqual(d['ci95'],[1.,1.])

    def test_bootstrap_uses_pairs_and_is_repeatable(self):
        rows=results()
        for i,r in enumerate(rows):r['simulated_time_s']=i+10
        a=paired_interval(rows,'random','nearest',lambda r:r['simulated_time_s'],100)
        b=paired_interval(list(reversed(rows)),'random','nearest',lambda r:r['simulated_time_s'],100)
        self.assertEqual(a,b);self.assertEqual(a['mean_difference'],-1.)

    def test_duplicate_outcomes_are_not_silently_pooled(self):
        rows=results()
        with self.assertRaises(AssertionError):aggregate(rows+[rows[0]],task_specs(),draws=10)

    def test_a_success_without_independent_evidence_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            out=Path(td);j=task_specs()[0];j['manifest_path']='manifest.json'
            (out/'manifest.json').write_text('{}',encoding='utf-8')
            folder=out/'episodes'/j['job_id'];folder.mkdir(parents=True)
            summary=dict(termination='success',source_fingerprint='frozen',source_unchanged=True,
                simulated_time_s=1.,actual_distance_m=0.,evaluation={},wall_time_s=1.)
            (folder/'summary.json').write_text(json.dumps(summary),encoding='utf-8')
            r=result_row(out,j,dict(engine_fingerprint='frozen',base_manifest={'recovery':{'minimum_progress_m2':.1}}))
            self.assertFalse(r['success']);self.assertEqual(r['termination'],'integrity_error')

if __name__=='__main__':unittest.main()
