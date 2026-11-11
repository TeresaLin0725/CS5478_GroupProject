"""Authored Rs task setup; target geometry stays with simulation/evaluation."""
from dataclasses import dataclass
from math import pi
from typing import Tuple

from ..contracts import Pose


@dataclass(frozen=True)
class RsTarget:
    class_name: str
    model: str
    position: Tuple[float, float, float]
    instance_id: str


LEGACY_CUP = RsTarget('cup', '025_mug', (-1.4, 2.95, .9213), 'Rs-inserted-mug')
KITCHEN_BOWL = RsTarget('bowl', '024_bowl',
    (-1.3502419985830785, 2.9511864975094797, .9097093077414806), 'Rs-inserted-bowl')
BATHROOM_BOWL = RsTarget('bowl', '024_bowl',
    (-3.400241998583078, 3.0011864975094804, .43358444657109885), 'Rs-inserted-bowl')
RS_TARGETS = {'legacy_cup': LEGACY_CUP, 'kitchen_bowl': KITCHEN_BOWL, 'bathroom_bowl': BATHROOM_BOWL}


def bowl_start_pose():
    """Original living-room start; the initial scan covers 360 degrees."""
    return Pose(.8, -2.4, 0., pi/2)
