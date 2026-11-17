"""Recovery must preserve hard caps through relabeling, splitting and merging."""
from dataclasses import replace
import unittest

from frontier_search.config import RecoveryConfig
from frontier_search.contracts import (CandidateAssessment, CandidateEvaluation, FailureReason, Frontier, GridCell,
    PlannerAttempt, PlannerResult, PolicyName, Pose, Subgoal)
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.runner.recovery import SpatialRecovery, probe_candidates


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.state = load_policy_fixture().state
        self.recovery = SpatialRecovery(RecoveryConfig(cooldown_s=2., opportunity_attempt_cap=2))
        self.recovery.reset(self.state.episode_id)

    def failed(self, state, frontier, reason=FailureReason.NO_PATH):
        self.recovery.record_dispatch(state, frontier.frontier_id)
        self.recovery.record_attempt(state, PlannerAttempt(
            Subgoal(frontier.frontier_id, frontier.approach_pose, state.revision, PolicyName.NEAREST),
            PlannerResult(False, None, None, reason), 'failed', reason))

    def excluded(self, state):
        return {e.frontier_id: e for e in self.recovery.exclusions(state, CandidateEvaluation(
            state.episode_id, state.revision, state.robot_pose, ()))}

    def test_revision_and_semantic_changes_do_not_unlock_geometry_failure(self):
        f = self.state.frontiers[0]
        self.failed(self.state, f)
        changed = replace(self.state, revision=9, timestamp_s=99., semantic_objects=())
        self.assertEqual(self.excluded(changed)[f.frontier_id].retry_condition, 'evidence_change')

    def test_relevant_map_change_unlocks_but_hard_cap_remains(self):
        f = self.state.frontiers[0]
        self.failed(self.state, f)
        cells = [list(row) for row in self.state.occupancy_map.data]
        cells[7][6] = 100
        changed = replace(self.state, occupancy_map=replace(self.state.occupancy_map,
                            data=tuple(tuple(row) for row in cells)))
        self.assertNotIn(f.frontier_id, self.excluded(changed))
        self.failed(changed, f)
        self.assertEqual(self.excluded(self.state)[f.frontier_id].retry_condition, 'exhausted')

    def test_unrelated_unknown_to_occupied_change_does_not_unlock(self):
        f = self.state.frontiers[0]
        self.failed(self.state, f)
        cells = [list(row) for row in self.state.occupancy_map.data]
        cells[39][39] = 100
        changed = replace(self.state, occupancy_map=replace(self.state.occupancy_map,
                            data=tuple(tuple(row) for row in cells)))
        self.assertIn(f.frontier_id, self.excluded(changed))

    def test_cooldown_uses_simulation_time(self):
        f = self.state.frontiers[0]
        self.failed(self.state, f, FailureReason.BLOCKED)
        self.assertIn(f.frontier_id, self.excluded(replace(self.state, timestamp_s=2.)))
        self.assertNotIn(f.frontier_id, self.excluded(replace(self.state, timestamp_s=3.)))

    def test_all_active_locks_must_be_geometry_locks_before_probing(self):
        for reason in ('blocked', 'timeout', 'no_progress'):
            for geometry_first in (False, True):
                with self.subTest(reason=reason, geometry_first=geometry_first):
                    self.recovery.reset(self.state.episode_id)
                    f = self.state.frontiers[0]
                    state = replace(self.state, frontiers=(f,))
                    geometry = CandidateEvaluation(state.episode_id, state.revision, state.robot_pose,
                        (CandidateAssessment(f.frontier_id, False, None, FailureReason.NO_PATH),))
                    if geometry_first:
                        self.recovery.record_evaluation(state, geometry)
                    if reason == 'no_progress':
                        self.recovery.record_progress(state, state, f.frontier_id)
                    else:
                        self.failed(state, f, FailureReason(reason))
                    if not geometry_first:
                        self.recovery.record_evaluation(state, geometry)
                    exclusions = self.recovery.exclusions(state, geometry)
                    self.assertEqual(probe_candidates(state, exclusions, self.recovery), [])
                    self.assertEqual(exclusions[0].retry_after_s,
                                     state.timestamp_s+self.recovery.config.cooldown_s)
                    later = replace(state, timestamp_s=state.timestamp_s+self.recovery.config.cooldown_s+.1)
                    exclusions = self.recovery.exclusions(later, geometry)
                    self.assertEqual([p.frontier_id for p in probe_candidates(later, exclusions, self.recovery)],
                                     [f.frontier_id])

    def test_probe_caps_survive_split_and_relabel(self):
        f = self.state.frontiers[0]
        self.failed(self.state, f)
        self.failed(self.state, f)
        children = (replace(f, frontier_id='child-a', parent_frontier_ids=(f.frontier_id,)),
                    replace(f, frontier_id='child-b'))
        state = replace(self.state, frontiers=children)
        exclusions = tuple(self.excluded(state).values())
        self.assertEqual(probe_candidates(state, exclusions, self.recovery), [])
        self.assertTrue(all(not self.recovery.can_probe(state, child) for child in children))

    def test_split_disappearance_relabel_preserve_shared_cap(self):
        f = self.state.frontiers[0]
        self.failed(self.state, f)
        self.failed(self.state, f)
        self.excluded(replace(self.state, frontiers=()))
        children = (replace(f, frontier_id='child-a', parent_frontier_ids=(f.frontier_id,)),
                    replace(f, frontier_id='child-b', parent_frontier_ids=(f.frontier_id,)))
        excluded = self.excluded(replace(self.state, frontiers=children))
        self.assertEqual(set(excluded), {'child-a', 'child-b'})
        self.assertTrue(all(e.attempt_count == 2 and e.retry_condition == 'exhausted' for e in excluded.values()))
        # Even missing lineage cannot erase spatial history.
        relabeled = replace(f, frontier_id='new-label')
        self.assertEqual(self.excluded(replace(self.state, frontiers=(relabeled,)))['new-label'].retry_condition,
                         'exhausted')

    def test_merge_accumulates_attempts_from_both_parents(self):
        a, b = self.state.frontiers[:2]
        self.failed(self.state, a)
        self.failed(self.state, b)
        merged = replace(a, frontier_id='merged', parent_frontier_ids=(a.frontier_id, b.frontier_id))
        result = self.excluded(replace(self.state, frontiers=(merged,)))['merged']
        self.assertEqual((result.attempt_count, result.retry_condition), (2, 'exhausted'))

    def test_cancelled_attempt_is_not_a_failure(self):
        f = self.state.frontiers[0]
        self.recovery.record_attempt(self.state, PlannerAttempt(
            Subgoal(f.frontier_id, f.approach_pose, 1, PolicyName.NEAREST), None, 'cancelled', None))
        self.assertEqual(self.excluded(self.state), {})

    def test_local_merge_cannot_reset_known_frontier_retry_history(self):
        self.recovery = SpatialRecovery(
            RecoveryConfig(opportunity_attempt_cap=2), locality_radius_m=.6)
        self.recovery.reset(self.state.episode_id)
        base = self.state.frontiers[0]
        a = replace(base, frontier_id='a', parent_frontier_ids=(),
                    approach_pose=Pose(.55, 1.95, 0., 0.),
                    support_cells=(GridCell(20, 20),))
        b = replace(a, frontier_id='b', approach_pose=Pose(-.15, 1.95, 0., 0.),
                    support_cells=(GridCell(20, 22),))
        state = replace(self.state, frontiers=(a, b))
        self.recovery.record_dispatch(state, 'a')
        self.recovery.record_dispatch(state, 'b')
        # A split/merge bridge lies within the locality radius of both groups.
        bridge = replace(a, frontier_id='bridge', parent_frontier_ids=('a', 'b'),
                         approach_pose=Pose(.15, 1.95, 0., 0.),
                         support_cells=(GridCell(20, 21),))
        self.excluded(replace(state, frontiers=(bridge,)))
        # The surviving anchor is 0.7 m from b, beyond the 0.6 m locality.
        for frontiers in ((a, b, bridge), (b, bridge, a)):
            excluded = self.excluded(replace(state, frontiers=frontiers))
            self.assertEqual(excluded['b'].attempt_count, 2)
            self.assertEqual(excluded['b'].retry_condition, 'exhausted')
            self.assertFalse(self.recovery.can_probe(state, b))

    def test_locality_still_separates_new_distant_frontiers(self):
        self.recovery = SpatialRecovery(
            RecoveryConfig(opportunity_attempt_cap=1), locality_radius_m=.6)
        self.recovery.reset(self.state.episode_id)
        a = replace(self.state.frontiers[0], frontier_id='a', parent_frontier_ids=())
        self.recovery.record_dispatch(replace(self.state, frontiers=(a,)), 'a')
        # Even matching support/parent lineage cannot attach a NEW distant id.
        b = replace(a, frontier_id='b', parent_frontier_ids=('a',),
                    approach_pose=replace(a.approach_pose, x_m=a.approach_pose.x_m+1.))
        self.assertNotIn('b', self.excluded(replace(self.state, frontiers=(b,))))

    def test_progress_counts_only_dispatch_neighborhood_unknown_cells(self):
        f = self.state.frontiers[0]
        cells = [list(row) for row in self.state.occupancy_map.data]
        cells[7][4] = 0
        cells[39][39] = 0  # unrelated distant observation
        after = replace(self.state, occupancy_map=replace(self.state.occupancy_map,
                        data=tuple(tuple(row) for row in cells)))
        area = self.recovery.record_progress(self.state, after, f.frontier_id)
        self.assertAlmostEqual(area, .01)


if __name__ == '__main__':
    unittest.main()
