"""Owner B: PLACEHOLDER robot used only for B's development and tests.

These numbers are NOT the frozen team values. The real robot, footprint, clearance
margin and limits must be chosen with A and C and recorded in the experiment
manifest (Interface Freeze section 8) before integration. This file does not
touch the shared ``config.py`` or any other owner's configuration.

Robotics background: the placeholder is a small circular, differential-drive-like
base (comparable in size to a Turtlebot2, radius about 0.175 m) that can turn in
place. That property is what justifies a 2-D grid planner (A*) in
``planner.py``; a car-like robot with a minimum turning radius would need a
planner that respects its kinematic constraints (for example Hybrid A*).
"""

from ..contracts import RobotConfig
from .geometry import circle_footprint

PLACEHOLDER_RADIUS_M = 0.18
PLACEHOLDER_MARGIN_M = 0.03

PLACEHOLDER_ROBOT = RobotConfig(
    footprint_base=circle_footprint(PLACEHOLDER_RADIUS_M),
    collision_z_min_m=0.0,
    collision_z_max_m=1.0,
    clearance_margin_m=PLACEHOLDER_MARGIN_M,
    motion_model="differential_drive_placeholder",
    can_turn_in_place=True,
    max_linear_speed_mps=0.5,
    max_angular_speed_radps=1.0,
    position_tolerance_m=0.1,
    yaw_tolerance_rad=0.1,
)
