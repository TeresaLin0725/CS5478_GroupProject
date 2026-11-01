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

from math import cos, pi, sin

from ..contracts import Point2, RobotConfig
from .navigator import NearObstacleLimit

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


# OPT-IN soft-contact preset for ``TeamNavigator(sim, **SOFT_CONTACT_NAV_OPTIONS)``; NOT applied by default.
# Goal: like a robot vacuum, a light touch is acceptable but a hard collision (contact then stuck) is not.
#   * planning_radius_m 0.182: planning on the circumscribed disc (0.257 m) treats the footprint (a +/-0.181 m AABB)
#     as a circle and rejects gaps the robot fits through (configuration-space inflation, Lozano-Perez 1983). The
#     measured Locobot collision geometry is round (base cylinder r 0.178 m, max radial extent about 0.19 m), so a disc
#     of 0.182 m plus the 0.01 m margin below (0.192 m) still contains it.
#   * planning_margin_m 0.01: halves the 2 cm planning buffer. On the benchmark map the left-room entrance only
#     opens for margin <= 1 cm with this radius; the buffer exists to absorb map (10 cm cell) and localization error,
#     so this is a deliberate trade of safety buffer for reachability. Do not set it to 0: 0.182 m is then below the
#     measured 0.188 m radial extent of the base.
#   * start_margin_m 0.0: the 2 cm clearance margin is a planning buffer, not physics, so a robot that touches
#     nothing is not declared immobile just because its pose lies inside the buffer.
# ``near_obstacle_limit`` (a speed cap near obstacles) is deliberately NOT part of the preset: in the no-render
# physics replay of the 36 benchmark navigation segments it caused stalls on millimetre floor lips in 8 to 11 percent
# of dispatches, while the planning options alone gave 198 of 198 clean arrivals. It stays available as an option.
# Evidence is a PyBullet replay with the real Locobot URDF and Rs scan mesh, NOT the native iGibson run.
SOFT_CONTACT_NAV_OPTIONS = dict(planning_radius_m=0.182, planning_margin_m=0.01, start_margin_m=0.0)


# OPT-IN round footprint. The Locobot base is a cylinder (collision radius 0.178 m, max radial extent about 0.188 m
# from the collision mesh); the +/-0.181 m square above is the AABB of all links, whose circumscribed circle (0.257 m)
# is about a third larger than the real body. A square footprint also makes every in-place turn check sweep the corners
# of that square, which refuses turns near walls that the round body performs freely. Use with
# ``ROUND_FOOTPRINT_NAV_OPTIONS`` (no ``planning_radius_m``: a 16-gon's own bounds already sit within 0.1864 to 0.19 m).
# Evidence: PyBullet replay with the real URDF and Rs mesh, NOT the native iGibson run. The vertices are the 16-gon
# circumscribing the measured base radius, so the polygon contains the base.
LOCOBOT_ROUND_RADIUS_M = 0.19
LOCOBOT_ROUND_SIDES = 16
SHARED_ROUND_CLEARANCE_M = 0.01


def round_footprint(radius_m: float = LOCOBOT_ROUND_RADIUS_M, sides: int = LOCOBOT_ROUND_SIDES):
    """Regular polygon whose vertices lie on the circle of ``radius_m`` (it contains the inscribed circle by construction)."""
    if radius_m <= 0 or sides < 8:
        raise ValueError("round footprint needs a positive radius and at least 8 sides")
    return tuple(Point2(radius_m * cos(2 * pi * i / sides), radius_m * sin(2 * pi * i / sides)) for i in range(sides))


def shared_round_robot(robot: RobotConfig) -> RobotConfig:
    """Use B's round geometry throughout a new episode; preserve other settings."""
    from dataclasses import replace
    return replace(robot, footprint_base=round_footprint(),
                   clearance_margin_m=SHARED_ROUND_CLEARANCE_M)


LOCOBOT_ROUND_ROBOT = RobotConfig(
    footprint_base=round_footprint(),
    collision_z_min_m=LOCOBOT_DEV_ROBOT.collision_z_min_m,
    collision_z_max_m=LOCOBOT_DEV_ROBOT.collision_z_max_m,
    clearance_margin_m=LOCOBOT_DEV_ROBOT.clearance_margin_m,
    motion_model=LOCOBOT_DEV_ROBOT.motion_model,
    can_turn_in_place=True,
    max_linear_speed_mps=LOCOBOT_DEV_ROBOT.max_linear_speed_mps,
    max_angular_speed_radps=LOCOBOT_DEV_ROBOT.max_angular_speed_radps,
    position_tolerance_m=LOCOBOT_DEV_ROBOT.position_tolerance_m,
    yaw_tolerance_rad=LOCOBOT_DEV_ROBOT.yaw_tolerance_rad,
)
# With the round footprint the planning disc is the footprint's own circumscribed circle, so only the two margins change.
ROUND_FOOTPRINT_NAV_OPTIONS = dict(planning_margin_m=0.01, start_margin_m=0.0)
# The same round footprint WITHOUT touching the shared manifest (A and C keep reading the square): B replaces the footprint
# internally. Use ``TeamNavigator(sim, **ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)``. A's frontier approach poses and spawn
# clearance still follow the manifest footprint; only B's checks (costmap, turns, sweeps) become round.
ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS = dict(footprint_override=round_footprint(), planning_margin_m=0.01, start_margin_m=0.0)
