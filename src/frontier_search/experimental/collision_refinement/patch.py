"""A -> B immutable collision evidence; no changes to WorldState or global maps."""
from dataclasses import dataclass
from math import isfinite
from typing import Optional, Tuple
from ...config import MappingConfig
from ...contracts import GridCell, OccupancyGrid, RobotConfig, WorldState
from ...immutable import ImmutableRecord
from ...navigation.geometry import cells_overlapping_footprint, cell_value

VERSION = 'local-collision-5cm-v2'
RELEASE_SCOPES = ('window', 'current_conflicts')


def current_conflict_cells(state, robot):
    """A selects occupied coarse cells touched by the current normal-clearance footprint.

    This selects where independent fine FREE evidence may be consumed. It does
    not alter the coarse map, free unknown space, or assert that a child is free.
    """
    return tuple(GridCell(r,c) for r,c in cells_overlapping_footprint(
        state.occupancy_map,robot.footprint_base,state.robot_pose,robot.clearance_margin_m)
        if cell_value(state.occupancy_map,r,c)==100)


@dataclass(frozen=True)
class LocalCollisionPatch(ImmutableRecord):
    state: WorldState
    grid: OccupancyGrid
    robot: RobotConfig
    mapping: MappingConfig
    collision_z: tuple
    base_z_m: float
    version: str = VERSION
    release_cells: Optional[Tuple[GridCell, ...]] = None  # None: original full window; (): release no coarse obstacles

    def matches(self, grid, episode_id, robot, mapping, base_z, now, max_age):
        """Fail closed for wrong scene snapshot, geometry, height, time or schema."""
        s, local = self.state, self.grid
        if (self.version != VERSION or s.episode_id != episode_id or s.occupancy_map is not grid
                or self.robot != robot or self.mapping != mapping
                or local.frame_id != grid.frame_id or local.revision != grid.revision
                or s.revision != grid.revision or s.last_processed_sequence < 1
                or not 0 <= now-s.timestamp_s <= max_age+1e-9
                or self.base_z_m != base_z):
            return False
        expected = (max(mapping.floor_height_m+mapping.floor_tolerance_m,
                        base_z+robot.collision_z_min_m-mapping.base_elevation_margin_m),
                    base_z+robot.collision_z_max_m+mapping.base_elevation_margin_m)
        if self.collision_z != expected:
            return False
        if self.release_cells is not None and self.release_cells != current_conflict_cells(s,robot):
            return False
        if (grid.resolution_m, grid.origin_x_m, grid.origin_y_m, grid.width_cells, grid.height_cells) != (
                mapping.resolution_m, mapping.origin_x_m, mapping.origin_y_m,
                mapping.width_cells, mapping.height_cells):
            return False
        values = (local.resolution_m, local.origin_x_m, local.origin_y_m)
        if not all(type(v) in (float, int) and isfinite(v) for v in values) or local.resolution_m != .05:
            return False
        factor = grid.resolution_m/local.resolution_m
        if factor < 2 or abs(factor-round(factor)) > 1e-9:
            return False
        if (type(local.width_cells) is not int or type(local.height_cells) is not int
                or local.width_cells <= 0 or local.height_cells <= 0
                or local.width_cells % round(factor) or local.height_cells % round(factor)
                or len(local.data) != local.height_cells
                or any(len(row) != local.width_cells or any(type(v) is not int or v not in (-1, 0, 100)
                                                          for v in row) for row in local.data)):
            return False
        return all(abs(v-round(v)) <= 1e-8 for v in (
            (local.origin_x_m-grid.origin_x_m)/grid.resolution_m,
            (local.origin_y_m-grid.origin_y_m)/grid.resolution_m))
