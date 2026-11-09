"""Discrete ray evidence for a static scene on one configured flat floor.

Every vertical bin in a cell's collision column needs free ray evidence, plus
an observed floor endpoint, before that cell becomes free. An observed obstacle
always wins and is retained until reset. This is a sampled voxel model, not a
continuous-volume clearance proof; bin size and sensor sampling need real-scene
validation. No footprint inflation or hidden geometry is used here.
"""

from math import ceil, floor
from typing import Iterator, List, Tuple

from ..config import MappingConfig
from ..contracts import RGBDObservation
from .projection import iter_depth_world
from .depth_filter import filter_mapping_depth


def _ray_voxels(start: Tuple[float, ...], end: Tuple[float, ...],
                bounds: Tuple[float, ...]) -> Iterator[Tuple[int, ...]]:
    """Clip a segment and visit positive-length voxel intersections in order.

    Coordinates are in voxel units. Upper bounds may truncate the last z bin.
    Simultaneous crossings advance all axes; a corner touch supplies no evidence.
    """
    delta = tuple(b - a for a, b in zip(start, end))
    enter, leave = 0.0, 1.0
    for a, d, upper in zip(start, delta, bounds):
        if d == 0:
            if not 0 <= a < upper:
                return
        else:
            first, last = sorted((-a / d, (upper - a) / d))
            enter, leave = max(enter, first), min(leave, last)
    if enter >= leave or not any(delta):
        return
    cells = []
    for a, d, upper in zip(start, delta, bounds):
        value = a + enter * d
        index = floor(value)
        if d < 0 and abs(value - round(value)) < 1e-10:
            index = round(value) - 1
        cells.append(max(0, min(ceil(upper) - 1, index)))
    crossings = [(cell + (1 if d > 0 else 0) - a) / d if d else float('inf')
                 for cell, a, d in zip(cells, start, delta)]
    increments = [abs(1 / d) if d else float('inf') for d in delta]
    steps = [1 if d > 0 else -1 if d < 0 else 0 for d in delta]
    limits = [ceil(b) for b in bounds]
    # Keep voxel indices/crossings in local scalars. The original loop created
    # enumerate tuples and indexed three lists for every crossed voxel.
    c, r, z = cells
    tx, ty, tz = crossings
    ix, iy, iz = increments
    sx, sy, sz = steps
    nx, ny, nz = limits
    current = enter
    while current < leave:
        # Match min's comparison order without a function call per voxel.
        following = leave
        if tx < following:
            following = tx
        if ty < following:
            following = ty
        if tz < following:
            following = tz
        if following - current > 1e-12:
            yield c, r, z
        if following >= leave:
            break
        # Preserve axis order and tolerance, including simultaneous crossings.
        threshold = following + 1e-12
        if tx <= threshold:
            c += sx
            if c < 0 or c >= nx:
                return
            tx += ix
        if ty <= threshold:
            r += sy
            if r < 0 or r >= ny:
                return
            ty += iy
        if tz <= threshold:
            z += sz
            if z < 0 or z >= nz:
                return
            tz += iz
        current = following


def fuse_observation(observation: RGBDObservation, cfg: MappingConfig,
                     collision_z: Tuple[float, float],
                     occupancy: List[List[int]], coverage: List[List[bool]],
                     free_masks: List[List[int]], floor_seen: List[List[bool]],
                     clear_counts=None, increment_clear_count=True) -> None:
    """Fuse into caller-owned working copies; never touch a published snapshot."""
    observation = filter_mapping_depth(observation, cfg)
    low, high = collision_z
    bin_height = cfg.clearance_bin_height_m
    # Avoid a spurious near-zero top bin after translating the world z origin.
    vertical_extent = (high - low) / bin_height
    nearest = round(vertical_extent)
    if nearest > 0 and abs(vertical_extent - nearest) < 1e-12:
        vertical_extent = float(nearest)
    bin_count = ceil(vertical_extent)
    full_mask = (1 << bin_count) - 1
    matrix = observation.camera_to_world.matrix
    camera = tuple(row[3] for row in matrix[:3])
    start = ((camera[0] - cfg.origin_x_m) / cfg.resolution_m,
             (camera[1] - cfg.origin_y_m) / cfg.resolution_m,
             (camera[2] - low) / bin_height)
    bounds = (cfg.width_cells, cfg.height_cells, vertical_extent)
    # Dense evidence avoids allocating/hashing (row, column) keys for each voxel.
    # This is frame-local: accumulated masks cannot prove that an obstacle left.
    fresh_masks = ([[0] * cfg.width_cells for _ in range(cfg.height_cells)]
                   if clear_counts is not None else None)
    fresh_floor, hits = set(), set()
    for _, _, depth, x, y, z in iter_depth_world(observation, cfg.depth_pixel_stride):
        if not cfg.depth_min_m <= depth <= cfg.depth_max_m:
            continue
        column = floor((x - cfg.origin_x_m) / cfg.resolution_m)
        row = floor((y - cfg.origin_y_m) / cfg.resolution_m)
        if 0 <= row < cfg.height_cells and 0 <= column < cfg.width_cells:
            coverage[row][column] = True
            if abs(z - cfg.floor_height_m) <= cfg.floor_tolerance_m:
                floor_seen[row][column] = True
                fresh_floor.add((row, column))
            elif low <= z <= high:
                occupancy[row][column] = 100
                hits.add((row, column))
        # Axial depth margin uses the same parameterization as the measured ray.
        fraction = (depth - cfg.depth_surface_margin_m) / depth
        endpoint = tuple(a + fraction * (b - a)
                         for a, b in zip(camera, (x, y, z)))
        end = ((endpoint[0] - cfg.origin_x_m) / cfg.resolution_m,
               (endpoint[1] - cfg.origin_y_m) / cfg.resolution_m,
               (endpoint[2] - low) / bin_height)
        for c, r, z in _ray_voxels(start, end, bounds):
            coverage[r][c] = True
            bit = 1 << z
            free_masks[r][c] |= bit
            if fresh_masks is not None:
                fresh_masks[r][c] |= bit
    _finalize_free_space(cfg, occupancy, free_masks, floor_seen, clear_counts,
                         increment_clear_count, fresh_masks, fresh_floor, hits, full_mask)


def _finalize_free_space(cfg, occupancy, free_masks, floor_seen, clear_counts,
                         increment_clear_count, fresh_masks, fresh_floor, hits, full_mask):
    """Apply the unchanged floor/column and consecutive-fresh-frame rules.

    Keep this phase separate so the offline profiler can distinguish final map
    decisions from ray traversal and evidence writes without per-cell timers.
    """
    for r in range(cfg.height_cells):
        for c in range(cfg.width_cells):
            if clear_counts is not None and occupancy[r][c] == 100:
                clear = ((r, c) not in hits and (r, c) in fresh_floor
                         and fresh_masks[r][c] == full_mask)
                if not clear:
                    clear_counts[r][c] = 0
                elif increment_clear_count:
                    clear_counts[r][c] += 1
                if clear_counts[r][c] >= cfg.dynamic_clear_frames:
                    occupancy[r][c] = 0
                    clear_counts[r][c] = 0
            if (occupancy[r][c] != 100 and floor_seen[r][c]
                    and free_masks[r][c] == full_mask):
                occupancy[r][c] = 0
