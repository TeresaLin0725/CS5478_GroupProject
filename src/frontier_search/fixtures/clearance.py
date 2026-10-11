"""Validate declared initialization priors and B's clearance attestation.

Hashes bind the supplied claim to its inputs; they do not prove physical truth.
Only B's simulator/robot checks can establish clearance and floor support.
"""

from collections import deque
from hashlib import sha256
from math import hypot

from ..contracts import GridCell, OccupancyGrid
from ..experiments.logging import dumps
from ..navigation.geometry import cells_overlapping_footprint
from ..validation import is_finite_number, require


def footprint_cells(manifest):
    c = manifest.mapping
    grid = OccupancyGrid('world', 0, c.resolution_m, c.origin_x_m, c.origin_y_m,
                         c.width_cells, c.height_cells, ())
    return tuple(GridCell(r, col) for r, col in cells_overlapping_footprint(
        grid, manifest.robot.footprint_base, manifest.start_pose, manifest.robot.clearance_margin_m))


def clearance_binding(manifest, cells):
    """Return expected input fingerprints/height band, WITHOUT asserting verification."""
    context = {key: getattr(manifest, key) for key in (
        'episode_id', 'scene_id', 'source', 'environment_seed', 'start_pose', 'robot', 'mapping',
        'initial_clearance_mode', 'initial_clearance_radius_m')}
    c, pose, robot = manifest.mapping, manifest.start_pose, manifest.robot
    return dict(context_sha256=sha256(dumps(context).encode('utf-8')).hexdigest(),
        cells_sha256=sha256(dumps(sorted(cells, key=lambda x: (x.row, x.column))).encode('utf-8')).hexdigest(),
        required_z_min_m=max(c.floor_height_m+c.floor_tolerance_m,
                             pose.z_m+robot.collision_z_min_m-c.base_elevation_margin_m),
        required_z_max_m=pose.z_m+robot.collision_z_max_m+c.base_elevation_margin_m)


def validate_clearance(manifest, cells, verification=None):
    manifest.validate()
    c = manifest.mapping
    require(all(isinstance(cell, GridCell) and type(cell.row) is int and type(cell.column) is int
                for cell in cells), 'clearance requires integer GridCell indices')
    require(len(set(cells)) == len(cells), 'duplicate clearance cells')
    require(all(0 <= cell.row < c.height_cells and 0 <= cell.column < c.width_cells for cell in cells),
            'clearance cell outside map')
    required = set(footprint_cells(manifest))
    supplied = set(cells)
    if manifest.initial_clearance_mode == 'footprint':
        require(supplied == required, 'spawn-clearance mask must equal the initial footprint plus margin')
        require(verification is None, 'starting-area verification requires verified_start_area mode')
        return
    require(required <= supplied, 'verified start area must contain the full initial footprint plus margin')
    pose, radius = manifest.start_pose, manifest.initial_clearance_radius_m
    require(all(hypot(c.origin_x_m+x*c.resolution_m-pose.x_m,
                      c.origin_y_m+y*c.resolution_m-pose.y_m) <= radius+1e-9
                for cell in cells for x in (cell.column, cell.column+1) for y in (cell.row, cell.row+1)),
            'entire clearance cell square must lie within the declared starting radius')
    # Partial verified areas are allowed, but disconnected islands of scene truth are not.
    seen, pending = set(required), deque(required)
    while pending:
        cell = pending.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbour = GridCell(cell.row+dr, cell.column+dc)
            if neighbour in supplied and neighbour not in seen:
                seen.add(neighbour)
                pending.append(neighbour)
    require(seen == supplied, 'verified starting cells must connect to the initial footprint')
    fields = {'context_sha256', 'cells_sha256', 'checked_z_min_m', 'checked_z_max_m',
              'all_cells_clear', 'floor_support_verified', 'verifier', 'verification_method', 'scene_snapshot_id'}
    require(isinstance(verification, dict) and set(verification) == fields,
            'verified start area requires a complete clearance verification record')
    binding = clearance_binding(manifest, cells)
    require(all(verification[key] == binding[key] for key in ('context_sha256', 'cells_sha256')),
            'clearance verification does not match the manifest context or cells')
    require(verification['all_cells_clear'] is True and verification['floor_support_verified'] is True,
            'all supplied cells require verified clearance and floor support')
    require(all(isinstance(verification[key], str) and verification[key].strip()
                for key in ('verifier', 'verification_method', 'scene_snapshot_id')),
            'missing verifier, method or scene snapshot identity')
    low, high = verification['checked_z_min_m'], verification['checked_z_max_m']
    require(is_finite_number(low) and is_finite_number(high) and low < high
            and low <= binding['required_z_min_m'] and high >= binding['required_z_max_m'],
            'verified height interval does not contain the complete mapping collision envelope')
