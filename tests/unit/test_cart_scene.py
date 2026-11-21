"""The scene must release the initially blocked aisle without returning into it."""
import importlib.util
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'optional numpy unavailable')
class CartSceneTests(unittest.TestCase):
    def test_textured_bottle_visuals_keep_collisions_and_fit_the_original_envelope(self):
        from pathlib import Path
        from tempfile import TemporaryDirectory
        import xml.etree.ElementTree as ET
        from frontier_search.simulation.indoor_challenges import furniture_urdf
        with TemporaryDirectory() as directory:
            mesh=Path(directory)/'bottle.obj'
            vertices=[(-.05,-.04,-.1),(.05,.04,.1)]
            mesh.write_text(''.join('v %s %s %s\n'%v for v in vertices),encoding='utf-8')
            before=ET.fromstring(furniture_urdf('cart'))
            after=ET.fromstring(furniture_urdf('cart',mesh))
            self.assertEqual([[(e.tag,e.attrib) for e in c.iter()] for c in before.findall('.//collision')],
                             [[(e.tag,e.attrib) for e in c.iter()] for c in after.findall('.//collision')])
            for link in after.findall('link'):
                if link.attrib['name'] in ('bottle0','bottle1'):
                    visual=link.find('visual')
                    scale=float(visual.find('geometry/mesh').attrib['scale'].split()[0])
                    origin=list(map(float,visual.find('origin').attrib['xyz'].split()))
                    for v in vertices:
                        x,y,z=[v[i]*scale+origin[i] for i in range(3)]
                        self.assertLessEqual(max(abs(x),abs(y)),.046)
                        self.assertGreaterEqual(z+.635,.57-1e-8)
                        self.assertLessEqual(z+.635,.755)
                elif link.attrib['name'].startswith('bottle'):
                    self.assertIsNone(link.find('visual'))

    def world(self, clear=True):
        from frontier_search.simulation.igibson_rs import IGibsonWorld
        world = IGibsonWorld.__new__(IGibsonWorld)
        world.profile = 'dynamic'
        world.cart_blocked = world.cart_finished = False
        world.cart_started_s = None
        world.steps = 481  # The scene clock has passed 16 s; robot is still behind the cart.
        world.environment_events = []
        world.cart_motion = SimpleNamespace(start=(.8, 1.15, 0), end=(.8, 2.7, 0), TRAVEL_M=.7,
                                            move=Mock(return_value=clear))
        return world

    def test_fallback_moves_before_robot_can_cross_and_stays_parked(self):
        world = self.world()
        pose = SimpleNamespace(x_m=.8, y_m=-2.4)
        world._update_challenge(pose)
        self.assertIsNotNone(world.cart_started_s)
        world.cart_motion.move.assert_called_once_with(0.)
        world.steps += 210
        world._update_challenge(pose)
        self.assertTrue(world.cart_finished)
        self.assertFalse(world.cart_blocked)
        self.assertEqual(world.cart_motion.move.call_args[0], (1.,))
        world.steps += 600
        world._update_challenge(pose)
        self.assertEqual(world.cart_motion.move.call_count, 2)
        self.assertEqual([e['event'] for e in world.environment_events], ['cart_crossing', 'cart_parked'])

    def test_new_obstruction_still_stops_the_cart_before_parking(self):
        world = self.world(clear=False)
        pose = SimpleNamespace(x_m=.8, y_m=-2.4)
        world._update_challenge(pose)
        self.assertTrue(world.cart_blocked)
        self.assertFalse(world.cart_finished)
        world.steps += 90
        world._update_challenge(pose)
        self.assertEqual(world.cart_motion.move.call_count, 1)


if __name__ == '__main__':
    unittest.main()
