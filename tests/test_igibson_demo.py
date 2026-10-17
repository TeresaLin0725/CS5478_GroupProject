"""Optional iGibson tests: run with data/venv-igibson (see scripts/setup_igibson.py)."""
import importlib.util
from math import atan2, hypot, pi
import unittest

HAS_IGIBSON = all(importlib.util.find_spec(name) for name in ('igibson', 'pybullet', 'scipy'))


@unittest.skipUnless(HAS_IGIBSON, 'iGibson runtime not installed')
class IGibsonTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from frontier_search.simulation.igibson_rs import IGibsonWorld
        # One simulator per process: iGibson's renderer owns a single GL context.
        cls.world = IGibsonWorld()

    @classmethod
    def tearDownClass(cls):
        cls.world.close()

    def test_1_initial_scan_hides_target_and_routes_stay_on_safe_cells(self):
        from frontier_search.runner.episode import build_context
        from frontier_search.simulation.igibson_rs import RGBDMapper, TARGET_CLASS, world_to_cell, wrap
        from frontier_search.validation import validate_state
        world = self.world
        mapper = RGBDMapper(world, 'test')
        start = world.pose().yaw_rad
        for k in range(6):
            goal = wrap(start + k * pi / 3)
            for _ in range(200):
                error = wrap(goal - world.pose().yaw_rad)
                if abs(error) < 0.05:
                    break
                world.drive(0.0, max(-1.2, min(1.2, 2.5 * error)))
            world.stop()
            mapper.observe()
        state, evaluation, routes = mapper.snapshot()
        validate_state(state)
        context = build_context(state, evaluation)
        self.assertIsNone(state.target_claim)
        self.assertNotIn(TARGET_CLASS, [t.class_name for t in state.semantic_objects])
        self.assertIn('sofa', [t.class_name for t in state.semantic_objects])
        self.assertTrue((mapper.cells == -1).any() and (mapper.cells == 100).any())
        self.assertTrue(context.eligible_frontier_ids)
        safe = mapper.traversable()
        for fid in context.eligible_frontier_ids:
            self.assertTrue(all(safe[world_to_cell(x, y)] for x, y in routes[fid][1:]))
        self.assertEqual(world.collision_steps, 0)

    def test_2_wheel_commands_follow_ccw_yaw_convention(self):
        from frontier_search.simulation.igibson_rs import wrap
        world = self.world
        before = world.pose()
        for _ in range(30):
            world.drive(0.0, 0.5)
        world.stop()
        self.assertGreater(wrap(world.pose().yaw_rad - before.yaw_rad), 0.3)
        before = world.pose()
        for _ in range(45):
            world.drive(0.3, 0.0)
        world.stop()
        moved = hypot(world.pose().x_m - before.x_m, world.pose().y_m - before.y_m)
        heading = atan2(world.pose().y_m - before.y_m, world.pose().x_m - before.x_m)
        self.assertGreater(moved, 0.3)
        self.assertLess(abs(wrap(heading - before.yaw_rad)), 0.2)

    def test_3_segmentation_claim_localises_target_from_kitchen(self):
        from frontier_search.simulation.igibson_rs import LEVEL_PITCH_RAD, RGBDMapper
        world = self.world
        # Arranged test pose (teleport), not the episode motion code.
        tx, ty, _ = world.target_position()
        yaw = atan2(ty - 2.2, tx + 0.5)
        body = next(iter(world.robot_bodies))
        world.p.resetBasePositionAndOrientation(body, (-0.5, 2.2, 0.0), world.p.getQuaternionFromEuler((0, 0, yaw)))
        world.stop(5)
        mapper = RGBDMapper(world, 'claim')
        mapper.observe(LEVEL_PITCH_RAD)
        world.stop(2)
        mapper.observe(LEVEL_PITCH_RAD)
        self.assertIsNotNone(mapper.claim)
        evidence = dict(mapper.target_evidence)
        for obs_id in mapper.claim.evidence_observation_ids:
            x, y, _ = evidence[obs_id]
            self.assertLess(hypot(x - tx, y - ty), 0.2)

    def test_4_two_distant_detections_do_not_complete_close_inspection(self):
        from frontier_search.simulation.igibson_rs import LEVEL_PITCH_RAD, RGBDMapper
        world = self.world
        tx, ty, _ = world.target_position()
        yaw = atan2(ty - 2.2, tx + .5)
        body = next(iter(world.robot_bodies))
        world.p.resetBasePositionAndOrientation(body, (-.5, 2.2, 0), world.p.getQuaternionFromEuler((0, 0, yaw)))
        world.stop(5)
        mapper = RGBDMapper(world, 'close-gate', close_distance_m=.85)
        mapper.observe(LEVEL_PITCH_RAD)
        world.stop(2)
        mapper.observe(LEVEL_PITCH_RAD)
        self.assertGreaterEqual(len(mapper.target_evidence), 2)
        self.assertIsNone(mapper.claim)
        self.assertIn('track-cracker_box', mapper.tracks)


if __name__ == '__main__':
    unittest.main()
