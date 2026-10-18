"""Cart safety scenarios with an axis-aligned collision-query backend."""
import unittest

from frontier_search.simulation.cart_motion import CartMotion


class BoxPhysics:
    GEOM_BOX = 3

    def __init__(self):
        self.bodies, self.shapes = {}, {}
        self.next_id = 1
        self.hole = False

    def createCollisionShape(self, kind, halfExtents):
        key = self.next_id
        self.next_id += 1
        self.shapes[key] = list(halfExtents)
        return key

    def createMultiBody(self, baseMass, baseCollisionShapeIndex, basePosition):
        key = self.next_id
        self.next_id += 1
        self.bodies[key] = (list(basePosition), self.shapes[baseCollisionShapeIndex])
        return key

    def add_box(self, position, half):
        shape = self.createCollisionShape(self.GEOM_BOX, half)
        return self.createMultiBody(0, shape, position)

    def getNumBodies(self):
        return len(self.bodies)

    def getBodyUniqueId(self, index):
        return list(self.bodies)[index]

    def getClosestPoints(self, first, second, distance):
        a, ah = self.bodies[first]
        b, bh = self.bodies[second]
        overlaps = all(abs(x - y) <= hx + hy + distance
                       for x, y, hx, hy in zip(a, b, ah, bh))
        return [True] if overlaps else []

    def setCollisionFilterGroupMask(self, *args):
        pass

    def removeBody(self, body):
        del self.bodies[body]

    def removeCollisionShape(self, shape):
        del self.shapes[shape]

    def rayTestBatch(self, starts, ends):
        return [(-1 if self.hole else 0, -1, .5, (x, y, 0), (0, 0, 1))
                for x, y, z in starts]


class Cart:
    def __init__(self, physics):
        self.p = physics
        self.position = [1.4, -.1, 0]
        self.body = physics.add_box([1.4, -.1, .4], [.24, .20, .36])

    def get_body_ids(self):
        return [self.body]

    def get_position(self):
        return self.position[:]

    def set_position_orientation(self, position, orientation):
        self.position = list(position)
        self.p.bodies[self.body] = ([position[0], position[1], .4], [.24, .20, .36])


class CartMotionTests(unittest.TestCase):
    def setUp(self):
        self.p = BoxPhysics()
        self.cart = Cart(self.p)

    def test_chair_inside_sweep_rejects_original_route_even_with_clear_endpoints(self):
        # Thin leg lies between the endpoints, beyond either endpoint envelope.
        self.p.add_box([1.05, -.1, .4], [.01, .01, .3])
        motion = CartMotion(self.p, self.cart)
        self.assertEqual(motion.start, (1.4, -.35, 0))
        self.assertEqual(self.cart.position, list(motion.start))

    def test_no_route_fails_before_recording(self):
        self.p.add_box([1.05, -.6, .4], [.02, 1.0, .3])
        with self.assertRaisesRegex(RuntimeError, 'No collision-free'):
            CartMotion(self.p, self.cart)

    def test_unsupported_route_is_rejected(self):
        self.p.hole = True
        with self.assertRaises(RuntimeError):
            CartMotion(self.p, self.cart)

    def test_entering_robot_stops_cart_without_teleporting_through_it(self):
        motion = CartMotion(self.p, self.cart)
        before = self.cart.get_position()
        self.p.add_box([1.05, -.1, .3], [.10, .10, .25])
        self.assertFalse(motion.move(1))
        self.assertEqual(self.cart.position, before)

    def test_out_and_back_and_query_cleanup(self):
        counts = len(self.p.bodies), len(self.p.shapes)
        motion = CartMotion(self.p, self.cart)
        self.assertTrue(motion.move(1))
        self.assertEqual(self.cart.position, list(motion.end))
        self.assertTrue(motion.move(0))
        self.assertEqual(self.cart.position, list(motion.start))
        self.assertEqual((len(self.p.bodies), len(self.p.shapes)), counts)


if __name__ == '__main__':
    unittest.main()
