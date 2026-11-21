"""Optional native Locobot motion test; requires the installed iGibson runtime."""
import importlib.util
from math import atan2, hypot
import unittest

HAS_IGIBSON = all(importlib.util.find_spec(name) for name in ('igibson', 'pybullet', 'scipy'))


@unittest.skipUnless(HAS_IGIBSON, 'iGibson runtime not installed')
class NativeSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from frontier_search.simulation.igibson_rs import IGibsonWorld
        # One simulator per process: iGibson's renderer owns a single GL context.
        cls.world = IGibsonWorld()

    @classmethod
    def tearDownClass(cls):
        cls.world.close()

    def test_wheel_commands_follow_ccw_yaw_convention(self):
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
