"""Conservative obstacle endpoints on an uninflated physical occupancy grid."""

from dataclasses import replace
from math import floor, isfinite
from typing import Optional, Tuple

from ..config import MappingConfig
from ..contracts import GridCell, OccupancyGrid, Pose, RobotConfig
from ..validation import ContractError, require
from .projection import ProjectedDepthPoint


# Check the fixed geometry needed to assign world coordinates to grid cells.
def _validate_geometry(grid: OccupancyGrid) -> None:
    require(grid.frame_id == "world", "occupancy grid must use world coordinates")
    require(type(grid.width_cells) is int and type(grid.height_cells) is int
            and min(grid.width_cells, grid.height_cells) > 0, "invalid grid dimensions")
    require(isfinite(grid.resolution_m) and grid.resolution_m > 0
            and isfinite(grid.origin_x_m) and isfinite(grid.origin_y_m), "invalid grid geometry")


def _cell_at(grid: OccupancyGrid, x_m: float, y_m: float) -> Optional[GridCell]:
    column = floor((x_m - grid.origin_x_m) / grid.resolution_m)
    row = floor((y_m - grid.origin_y_m) / grid.resolution_m)
    if 0 <= row < grid.height_cells and 0 <= column < grid.width_cells:
        return GridCell(row, column)
    return None


# Locate a world point in the map, returning None when it lies outside the grid.
def world_to_grid(grid: OccupancyGrid, x_m: float, y_m: float) -> Optional[GridCell]:
    """Rows follow world +y, columns +x; lower bounds are inclusive, upper bounds exclusive."""
    _validate_geometry(grid)
    require(isfinite(x_m) and isfinite(y_m), "world coordinates must be finite")
    return _cell_at(grid, x_m, y_m)


# Add observed obstacles within the robot's collision height to the occupancy map.
def mark_obstacles(grid: OccupancyGrid, points: Tuple[ProjectedDepthPoint, ...],
                   robot_pose: Pose, robot_config: RobotConfig,
                   mapping_config: MappingConfig) -> OccupancyGrid:
    """Return a new grid revision without clearing or inflating any cells.

    One accepted surface point marks its cell occupied. Depth bounds and robot
    collision-height bounds are inclusive. Floor-band points are excluded;
    neither floor observations nor rays establish free space in this stage.
    Floor height is configured in world coordinates; robot heights are relative
    to the capture-time base z. Coverage and the input grid remain unchanged.
    """
    _validate_geometry(grid)
    require(type(grid.revision) is int and grid.revision >= 0, "invalid map revision")
    require(len(grid.data) == grid.height_cells
            and all(len(row) == grid.width_cells for row in grid.data), "map dimensions disagree with data")
    require(all(type(value) is int and value in (-1, 0, 100) for row in grid.data for value in row),
            "invalid occupancy encoding")
    cfg = mapping_config
    try:
        cfg.__post_init__()
    except (TypeError, ValueError) as exc:
        raise ContractError("invalid mapping configuration: %s" % exc) from exc
    require((grid.width_cells, grid.height_cells, grid.resolution_m, grid.origin_x_m, grid.origin_y_m)
            == (cfg.width_cells, cfg.height_cells, cfg.resolution_m, cfg.origin_x_m, cfg.origin_y_m),
            "mapping configuration and grid geometry differ")
    low, high = robot_config.collision_z_min_m, robot_config.collision_z_max_m
    require(isfinite(low) and isfinite(high) and low < high and isfinite(robot_pose.z_m),
            "invalid robot collision height interval")
    low, high = robot_pose.z_m + low, robot_pose.z_m + high
    require(isfinite(low) and isfinite(high), "world collision height overflow")
    cells = [list(row) for row in grid.data]
    for point in points:
        position = point.position_world
        require(all(isfinite(value) for value in (point.depth_m, position.x_m, position.y_m, position.z_m)),
                "obstacle evidence must be finite")
        if not cfg.depth_min_m <= point.depth_m <= cfg.depth_max_m:
            continue
        if abs(position.z_m - cfg.floor_height_m) <= cfg.floor_tolerance_m:
            continue
        if not low <= position.z_m <= high:
            continue
        cell = _cell_at(grid, position.x_m, position.y_m)
        if cell is not None:
            cells[cell.row][cell.column] = 100
    return replace(grid, revision=grid.revision + 1, data=tuple(tuple(row) for row in cells))
