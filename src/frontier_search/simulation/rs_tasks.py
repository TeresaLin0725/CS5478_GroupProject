"""Authored Rs task setup; target geometry stays with simulation/evaluation."""
from dataclasses import dataclass
from math import isfinite, pi
from typing import Tuple

from ..contracts import Pose


@dataclass(frozen=True)
class RsTarget:
    class_name: str
    model: str
    position: Tuple[float, float, float]
    instance_id: str
    # Orientation of the inserted YCB model (radians). Defaults keep the identity
    # quaternion used by the registered tasks; semantic experiments stand objects up.
    yaw_rad: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0

    def validate(self):
        if not self.class_name.strip() or not self.model.strip() or not self.instance_id.strip():
            raise ValueError('target class, model and instance ID are required')
        if len(self.position) != 3 or not all(isfinite(value) for value in self.position):
            raise ValueError('target position must contain three finite coordinates')
        if not all(isfinite(value) for value in (self.roll_rad, self.pitch_rad, self.yaw_rad)):
            raise ValueError('target orientation must be finite')
        return self


LEGACY_CUP = RsTarget('cup', '025_mug', (-1.4, 2.95, .9213), 'Rs-inserted-mug')
KITCHEN_BOWL = RsTarget('bowl', '024_bowl',
    (-1.3502419985830785, 2.9511864975094797, .9097093077414806), 'Rs-inserted-bowl')
BATHROOM_BOWL = RsTarget('bowl', '024_bowl',
    (-3.400241998583078, 3.0011864975094804, .43358444657109885), 'Rs-inserted-bowl')
RS_TARGETS = {'legacy_cup': LEGACY_CUP, 'kitchen_bowl': KITCHEN_BOWL, 'bathroom_bowl': BATHROOM_BOWL}


def bowl_start_pose():
    """Original living-room start; the initial scan covers 360 degrees."""
    return Pose(.8, -2.4, 0., pi/2)
