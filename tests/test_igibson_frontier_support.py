"""Reachable boundary support must survive a centroid hidden behind furniture."""
import importlib.util
import unittest


@unittest.skipUnless(importlib.util.find_spec('numpy') and importlib.util.find_spec('scipy'),
                     'optional RGB-D dependencies unavailable')
class BoundaryApproachTests(unittest.TestCase):
    def test_unreachable_centroid_publishes_native_numeric_pose(self):
        from frontier_search.contracts import Pose
        from frontier_search.simulation.igibson_rs import RGBDMapper
        from frontier_search.validation import validate_state

        class World:
            robot_radius, robot_height, sim_time = .18, .65, 0.
            def pose(self):
                return Pose(.8, -2.4, 0., 0.)

        mapper = RGBDMapper(World(), 'unreachable-centroid')
        mapper.free_hits[:] = 0
        mapper.free_hits[18:22, 25:29] = 1
        state, evaluation, routes = mapper.snapshot()
        self.assertTrue(state.frontiers)
        self.assertTrue(all(a.reachable is False for a in evaluation.assessments))
        self.assertEqual(routes, {})
        for frontier in state.frontiers:
            pose = frontier.approach_pose
            self.assertTrue(all(type(v) is float for v in (pose.x_m, pose.y_m, pose.z_m, pose.yaw_rad)))
        validate_state(state)

    def test_unobserved_centroid_does_not_hide_reachable_edge(self):
        from frontier_search.contracts import Pose
        from frontier_search.runner.episode import build_context
        from frontier_search.simulation.igibson_rs import RGBDMapper, world_to_cell
        from frontier_search.validation import validate_state

        class World:
            robot_radius, robot_height, sim_time = .18, .65, 0.
            def pose(self):
                return Pose(1.35, -2.4, 0, 0)

        mapper = RGBDMapper(World(), 'wrapped-boundary')
        mapper.free_hits[:] = 0
        # A sensed route around an unobserved large furniture island. Both its
        # outer and inner frontier centroids lie in that island, off the route.
        mapper.free_hits[12:57, 26:63] = 1
        mapper.free_hits[18:51, 32:57] = 0
        state, evaluation, routes = mapper.snapshot()
        validate_state(state)
        context = build_context(state, evaluation)
        self.assertTrue(context.eligible_frontier_ids)
        safe = mapper.traversable()
        for fid in context.eligible_frontier_ids:
            self.assertTrue(all(safe[world_to_cell(x,y)] for x,y in routes[fid]))
