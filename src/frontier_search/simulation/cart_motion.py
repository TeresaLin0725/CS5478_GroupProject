"""Scene-side safety checks for the scripted cart; never exported to the mapper."""
from math import ceil


class CartMotion:
    # Conservative envelope around shelves, handle, bottles and wheels. Ignore
    # only the bottom 2 cm where the wheels meet the scanned floor.
    HALF_X, HALF_Y = .26, .23
    LOW_Z, HIGH_Z = .02, .78
    TRAVEL_M = .70

    def __init__(self, physics, cart):
        self.p, self.cart = physics, cart
        self.excluded = set(cart.get_body_ids())
        self.start = self.end = None
        # Prefer the original corridor; move the crossing away from the scanned
        # dining chairs only when the entire alternative sweep is verified.
        for y in (-.10, -.35, -.60, -.85, -1.10):
            start, end = (1.40, y, 0), (.70, y, 0)
            if self.clear_sweep(start, end) and self.floor_supported(start, end):
                self.start, self.end = start, end
                break
        if self.start is None:
            raise RuntimeError('No collision-free supported cart crossing in the Rs corridor; '
                               'refusing to run the dynamic scene.')
        cart.set_position_orientation(self.start, [0, 0, 0, 1])

    def clear_sweep(self, start, end):
        """Test the complete axis-aligned swept envelope, not just endpoints."""
        p = self.p
        half = [self.HALF_X + abs(end[0] - start[0]) / 2,
                self.HALF_Y + abs(end[1] - start[1]) / 2,
                (self.HIGH_Z - self.LOW_Z) / 2]
        centre = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2,
                  (self.LOW_Z + self.HIGH_Z) / 2]
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        probe = None
        try:
            probe = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                      basePosition=centre)
            # Query-only body: it must never push or block the robot in a step.
            p.setCollisionFilterGroupMask(probe, -1, 0, 0)
            for index in range(p.getNumBodies()):
                other = p.getBodyUniqueId(index)
                if other not in self.excluded and other != probe:
                    if p.getClosestPoints(probe, other, distance=.005):
                        return False
            return True
        finally:
            if probe is not None:
                p.removeBody(probe)
            p.removeCollisionShape(shape)

    def floor_supported(self, start, end):
        # Cover the swept footprint, rather than checking its centre alone.
        x0, x1 = min(start[0], end[0]) - self.HALF_X, max(start[0], end[0]) + self.HALF_X
        y0, y1 = min(start[1], end[1]) - self.HALF_Y, max(start[1], end[1]) + self.HALF_Y
        nx, ny = ceil((x1 - x0) / .10), ceil((y1 - y0) / .10)
        points = [(x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny)
                  for i in range(nx + 1) for j in range(ny + 1)]
        hits = self.p.rayTestBatch([[x, y, .015] for x, y in points],
                                   [[x, y, -.05] for x, y in points])
        return all(hit[0] >= 0 and hit[0] not in self.excluded
                   and abs(hit[3][2]) <= .015 and hit[4][2] > .9 for hit in hits)

    def move(self, fraction):
        position = [a + (b - a) * fraction for a, b in zip(self.start, self.end)]
        previous = self.cart.get_position()
        if not self.clear_sweep(previous, position):
            return False
        self.cart.set_position_orientation(position, [0, 0, 0, 1])
        return True
