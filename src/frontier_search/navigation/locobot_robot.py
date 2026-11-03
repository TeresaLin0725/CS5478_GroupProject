"""Owner B: legacy shared square envelope and explicit round navigation preset.

Source: A's ``simulation/record_sequence.py`` and ``Documentation/english/locobot_multiframe.md``.
A measures the installed Locobot URDF at run time (collision AABB union over all links) and
uses it as the base-frame footprint. The doc reports about +/-0.181 m horizontally and a
collision top of about 0.907 m above the base, so this file uses those rounded values.
These AABBs are bounding boxes, not the exact circular collision boundary. The
round preset below changes B's envelope only and keeps the shared robot record intact.
These are A's DEVELOPMENT settings, not a
frozen team agreement (see Documentation/english/locobot_abc_handoff.md).

Robotics background
-------------------
* The legacy development footprint is a rectangle. B's costmap plans on the circumscribed disc
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
#     as a circle and rejects gaps the robot fits through (configuration-space inflation, Lozano-Perez 1983).
#     this legacy preset retains the square execution footprint while changing the planning disc.
#     It is distinct from the round-footprint preset below.
#   * planning_margin_m 0.01: halves the 2 cm planning buffer. On the benchmark map the left-room entrance only
#     opens for margin <= 1 cm with this radius; the buffer exists to absorb map (10 cm cell) and localization error,
#     so this is a deliberate trade of safety buffer for reachability. This margin is separate from body geometry.
#   * start_margin_m 0.0: the 2 cm clearance margin is a planning buffer, not physics, so a robot that touches
#     nothing is not declared immobile just because its pose lies inside the buffer.
# ``near_obstacle_limit`` (a speed cap near obstacles) is deliberately NOT part of the preset: in the no-render
# physics replay of the 36 benchmark navigation segments it caused stalls on millimetre floor lips in 8 to 11 percent
# of dispatches, while the planning options alone gave 198 of 198 clean arrivals. It stays available as an option.
# Evidence is a PyBullet replay with the real Locobot URDF and Rs scan mesh, NOT the native iGibson run.
SOFT_CONTACT_NAV_OPTIONS = dict(planning_radius_m=0.182, planning_margin_m=0.01, start_margin_m=0.0)


# Round-footprint preset: 0.19 m is the VERTEX radius, not a physical body-radius measurement.
# The 16-gon's inscribed radius is 0.19*cos(pi/16) = 0.186349 m. Raw collision geometry
# from the assets matching the frozen Rs benchmark reaches 0.178 m; a zero-joint DIRECT
# measurement in PyBullet 3.2.7 reached at most 0.179007 m including engine effects
# (2026-11-03). This is not a native iGibson/pybullet-svl acceptance certificate.
# Keep this polygon unchanged. Costmap derives a 0.19 m planning disc, then represents
# it with a separate circumscribed polygon whose vertex radius is about 0.193722 m.
# Planning/start margins below are additional buffers; none is folded into the vertices.
LOCOBOT_ROUND_VERTEX_RADIUS_M = 0.19
LOCOBOT_ROUND_RADIUS_M = LOCOBOT_ROUND_VERTEX_RADIUS_M  # backwards-compatible name
LOCOBOT_ROUND_SIDES = 16


def round_footprint(radius_m: float = LOCOBOT_ROUND_RADIUS_M, sides: int = LOCOBOT_ROUND_SIDES):
    """16-gon by default; radius_m measures centre-to-vertex distance, excluding margins.

    This polygon contains a disc of radius radius_m*cos(pi/sides), not radius_m.
    The existing radius_m argument is retained for compatibility.
    """
    if radius_m <= 0 or sides < 8:
        raise ValueError("round footprint needs a positive radius and at least 8 sides")
    return tuple(Point2(radius_m * cos(2 * pi * i / sides), radius_m * sin(2 * pi * i / sides)) for i in range(sides))


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
# None/omitted planning_radius_m resolves to the footprint's 0.19 m circumradius.
# planning_margin_m is used for route/pose/turn checks; start_margin_m only relaxes
# the current pose and the first START_ZONE_M of departure, not the rest of the route.
ROUND_FOOTPRINT_NAV_OPTIONS = dict(planning_margin_m=0.01, start_margin_m=0.0)
# The same round footprint WITHOUT touching the shared manifest (A and C keep reading the square): B replaces the footprint
# internally. Use ``TeamNavigator(sim, **ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)``. A's frontier approach poses and spawn
# clearance still follow the manifest footprint; only B's checks (costmap, turns, sweeps) become round.
ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS = dict(footprint_override=round_footprint(), planning_margin_m=0.01, start_margin_m=0.0)
