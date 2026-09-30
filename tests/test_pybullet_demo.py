"""Optional tests: install requirements-pybullet.txt to exercise real physics."""
import importlib.util
import unittest

HAS_BULLET = all(importlib.util.find_spec(name) for name in ('pybullet', 'numpy', 'PIL'))


@unittest.skipUnless(HAS_BULLET, 'optional PyBullet dependencies not installed')
class BulletTests(unittest.TestCase):
    def test_unseen_target_is_not_in_policy_state_and_frontiers_are_reachable(self):
        from frontier_search.simulation.bullet_demo import BulletWorld, RangeMapper
        from frontier_search.runner.episode import build_context
        from frontier_search.validation import validate_state
        world = BulletWorld()
        try:
            mapper = RangeMapper(world, 'test')
            mapper.observe()
            state, evaluation, paths = mapper.snapshot()
            validate_state(state)
            context = build_context(state, evaluation)
            self.assertIsNone(state.target_claim)
            self.assertNotIn('mug', [t.class_name for t in state.semantic_objects])
            self.assertTrue((mapper.cells == -1).any())
            self.assertTrue(context.eligible_frontier_ids)
            safe = mapper.traversable()
            for route in paths.values():
                self.assertTrue(all(safe[int(y/.25), int(x/.25)] for x,y in route))
        finally:
            world.close()

    def test_closed_doorway_produces_measured_contact(self):
        import pybullet as p
        from frontier_search.simulation.bullet_demo import BulletWorld
        world = BulletWorld(blocked=True)
        try:
            # Arrange a dedicated collision test, not the episode movement code.
            p.resetBasePositionAndOrientation(world.robot, (10,2.3,.22), (0,0,0,1))
            for _ in range(240):
                world.step_velocity(0,.7,0)
            self.assertGreater(world.collision_steps,0)
            self.assertLess(world.pose().y_m,3.0)
        finally:
            world.close()


if __name__ == '__main__':
    unittest.main()
