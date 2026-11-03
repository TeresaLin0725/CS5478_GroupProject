"""Ablation arms: opt-in switches, old behaviour when absent, and the cross-arm paired comparison."""
import csv,sys,tempfile,unittest
from dataclasses import replace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from benchmark_protocol import ARMS,NAV_KEYS,RECOVERY_KEYS,apply_arm,task_specs,validate_pairing
from benchmark_episode import nav_kwargs
from compare_arms import compare,exact_two_sided,load,report
from frontier_search.navigation.navigator import RelaxedRetry


class ArmSwitchTests(unittest.TestCase):
    def test_no_arm_keeps_the_original_square_footprint_and_no_retry(self):
        self.assertEqual(nav_kwargs({}),{})
        self.assertEqual(nav_kwargs({'nav_options':{}}),{})

    def test_no_arm_preserves_shared_round_protocol_preset(self):
        self.assertEqual(nav_kwargs({}, {'navigation_preset':'shared_round'}), {'start_margin_m':0.})

    def test_relaxed_job_retains_shared_round_protocol_preset(self):
        options=nav_kwargs({'nav_options':{'relaxed_retry':True}}, {'navigation_preset':'shared_round'})
        self.assertEqual(options['start_margin_m'],0.)
        self.assertIsInstance(options['relaxed_retry'],RelaxedRetry)

    def test_each_arm_adds_one_idea(self):
        self.assertEqual(ARMS['A1'],dict(round_footprint=True,relaxed_retry=False,stuck_retry=False,epsilon_retry=0.))
        self.assertTrue(ARMS['A2']['relaxed_retry'] and ARMS['A2']['stuck_retry'] and ARMS['A2']['epsilon_retry']==0.)
        self.assertEqual(ARMS['A3']['epsilon_retry'],.05)

    def test_nav_kwargs_follow_the_job(self):
        round_only=nav_kwargs({'nav_options':{'round_footprint':True,'relaxed_retry':False}})
        self.assertIn('footprint_override',round_only);self.assertNotIn('relaxed_retry',round_only)
        both=nav_kwargs({'nav_options':{'round_footprint':True,'relaxed_retry':True}})
        self.assertIsInstance(both['relaxed_retry'],RelaxedRetry)

    def test_apply_arm_returns_c_side_fields_and_stores_b_side_ones_on_the_job(self):
        job={};recovery=apply_arm(job,'A3')
        self.assertEqual(set(recovery),set(RECOVERY_KEYS));self.assertEqual(recovery['epsilon_retry'],.05)
        self.assertEqual(set(job['nav_options']),set(NAV_KEYS));self.assertEqual(job['arm'],'A3')
        with self.assertRaises(ValueError):apply_arm({},'A0')

    def test_tasks_stay_identical_and_pairing_stays_valid_in_every_arm(self):
        baseline=[{k:v for k,v in j.items()} for j in task_specs()]
        for arm in ARMS:
            jobs=task_specs()
            for job in jobs:apply_arm(job,arm)
            validate_pairing(jobs)
            self.assertEqual([j['job_id'] for j in jobs],[j['job_id'] for j in baseline])
            self.assertEqual(len(jobs),36)

    def test_manifest_with_arm_recovery_is_valid_and_hashable_config(self):
        from frontier_search.runner.integrated import manifest_for
        base=manifest_for('t')
        manifest=replace(base,recovery=replace(base.recovery,**apply_arm({},'A3')))
        manifest.validate()
        self.assertEqual((manifest.recovery.stuck_retry,manifest.recovery.epsilon_retry),(True,.05))
        self.assertEqual((base.recovery.stuck_retry,base.recovery.epsilon_retry),(False,0.))


def rows(successes,total=6,termination='no_feasible_frontiers'):
    return {'j%d'%i:dict(job_id='j%d'%i,policy='nearest',success=i<successes,collisions=0,
                         termination='success' if i<successes else termination) for i in range(total)}


class CompareTests(unittest.TestCase):
    def test_exact_mcnemar_known_values(self):
        self.assertEqual(exact_two_sided(0,0),1.0)
        self.assertAlmostEqual(exact_two_sided(0,5),2*(1/32))
        self.assertEqual(exact_two_sided(3,3),1.0)

    def test_paired_comparison_counts_gained_and_lost_jobs(self):
        base,other=rows(3),rows(3)
        other['j1']['success']=False;other['j4']['success']=True
        other['j5']['success']=True
        result=compare(base,other)
        self.assertEqual((result['only_other_succeeded'],result['only_base_succeeded']),(2,1))
        self.assertEqual(result['net'],1);self.assertEqual(result['gained'],['j4','j5']);self.assertEqual(result['lost'],['j1'])

    def test_jobs_missing_from_an_arm_are_reported_not_dropped_silently(self):
        other=rows(3);del other['j5']
        self.assertEqual(compare(rows(3),other)['missing_from_other'],['j5'])

    def test_report_summarises_every_arm_and_keeps_failures(self):
        out=report({'A0':rows(2),'A1':rows(5)})
        self.assertEqual(out['summaries']['A0']['no_feasible_frontiers'],4)
        self.assertEqual(out['summaries']['A1']['successes'],5)
        self.assertEqual(out['versus_baseline']['A1']['only_other_succeeded'],3)

    def test_load_reads_the_real_episodes_csv_layout(self):
        with tempfile.TemporaryDirectory() as td:
            with open(Path(td)/'episodes.csv','w',encoding='utf-8-sig',newline='') as f:
                w=csv.DictWriter(f,['job_id','policy','success','termination','collisions']);w.writeheader()
                w.writerow(dict(job_id='a',policy='random',success='True',termination='success',collisions='0'))
                w.writerow(dict(job_id='b',policy='random',success='False',termination='no_feasible_frontiers',collisions='2'))
            data=load(td)
        self.assertTrue(data['a']['success']);self.assertFalse(data['b']['success']);self.assertEqual(data['b']['collisions'],2)


if __name__=='__main__':unittest.main()
