"""Owner B: Locobot robot configuration matching A's development recording.

Source: A's ``simulation/record_sequence.py`` and ``Documentation/english/locobot_multiframe.md``.
A measures the installed Locobot URDF at run time (collision AABB union over all links) and
uses it as the base-frame footprint. The doc reports about +/-0.181 m horizontally and a
collision top of about 0.907 m above the base, so this file uses those rounded values.
The true measured polygon may be slightly asymmetric; it must be re-measured on the machine
that has iGibson before this is treated as final. These are A's DEVELOPMENT settings, not a
frozen team agreement (see Documentation/english/locobot_abc_handoff.md).

Robotics background
-------------------
* The footprint is a rectangle, not a disc. B's costmap plans on the circumscribed disc
  (radius = half-diagonal, about 0.256 m) and verifies poses, segments and rotations with the
  exact polygon. A disc this large inflates obstacles more than the rectangle needs, so narrow
  doorways that the real robot could pass may be rejected: a conservative configuration-space
  choice that should be revisited against the measured Locobot.
* The base is differential-drive and can turn in place, so a 2-D (x, y) A* is still valid.
* Speeds are A's recording bounds (0.1 m/s, 0.4 rad/s), far below what Locobot can do, to keep
  scripted captures safe and the mapper's frames well-conditioned.

Only the standard library is used, and annotations stay Python 3.8 compatible.
"""

from ..contracts import Point2, RobotConfig

LOCOBOT_HALF_EXTENT_M = 0.181   # reported collision AABB half-width/length, rounded
LOCOBOT_COLLISION_TOP_M = 0.907  # reported collision top above the base (not the 0.858 m camera height)

LOCOBOT_DEV_ROBOT = RobotConfig(
    footprint_base=tuple(Point2(x * LOCOBOT_HALF_EXTENT_M, y * LOCOBOT_HALF_EXTENT_M)
                         for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))),
    collision_z_min_m=0.0,
    collision_z_max_m=LOCOBOT_COLLISION_TOP_M,
    clearance_margin_m=0.02,
    motion_model="differential_drive_development_recording",
    can_turn_in_place=True,
    max_linear_speed_mps=0.1,
    max_angular_speed_radps=0.4,
    position_tolerance_m=0.05,
    yaw_tolerance_rad=0.1,
)

# OPTIONAL planning radius for ``TeamNavigator(planning_radius_m=...)``; NOT applied by default.
# Between the inscribed (0.181 m) and circumscribed (0.256 m) radius. On the ideal mock simulator,
# 0.20-0.22 m planned and drove through 0.5 m and 0.6 m gaps with no contact (random starts, goals and
# yaws; see tests/test_planning_radius.py, which also pins the known limit: a bend inside a narrow
# corridor is stopped by the turn guard). Wheel slip, latency and the real controller are not modelled:
# verify on iGibson (zero contact on 0.5 m and 0.6 m gaps) before using it.
LOCOBOT_PLANNING_RADIUS_M = 0.22
