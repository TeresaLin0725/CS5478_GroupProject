"""Owner B: bounded verified-starting-area verifier (Documentation/english/verified_start_area.md).

Robotics background
-------------------
* Configuration space vs. workspace: a cell is only usable as free space for the
  *whole robot* if the entire vertical column above it, over the robot's collision
  height band, is free of non-robot geometry. A centre-point test or a clear ray
  checks a single workspace point, not the swept body, so it is insufficient.
* Traversability: collision-free is not enough for a wheeled base; the cell must also
  be supported by traversable floor (no hole, no unsupported span, no step larger
  than the wheels can climb). The flat-floor mapping model has no stairs.
* Conservative growth: start from the cells the robot is known to occupy (footprint
  plus margin) and accept only cells that pass both checks AND connect to that seed
  through 4-neighbour adjacency. Rejected cells are never filled in to complete a
  circle; disconnected "islands" of free space are dropped because nothing proves
  the robot can reach them from where it stands.

The verifier is simulator-agnostic: it only needs a ``SceneProbe``. Scene truth is used for
this setup check only and is never handed to A or C; only the verified cells and the
attestation record leave this module. ``MockSceneProbe`` implements the probe over
the mock simulator's ground-truth grid; the real iGibson adapter will supply its
own probe (physics-engine volume queries and floor ray casts).

Only the standard library is used, and annotations stay Python 3.8 compatible.
"""

from collections import deque
from dataclasses import dataclass
from math import floor, hypot
from typing import Dict, Mapping, Optional, Tuple

from ..contracts import GridCell, OccupancyGrid
from ..fixtures.clearance import clearance_binding, footprint_cells, validate_clearance
from ..navigation.geometry import FREE, in_bounds

_EPS = 1e-9


class SceneProbe:
    """Physical queries over one scene snapshot, in world metres (axis-aligned rectangles)."""

    snapshot_id = ""
    description = ""

    def column_clear(self, x_min: float, y_min: float, x_max: float, y_max: float,
                     z_min: float, z_max: float) -> bool:
        """True if NO non-robot collision geometry intersects the box [x]x[y]x[z]."""
        raise NotImplementedError

    def floor_supported(self, x_min: float, y_min: float, x_max: float, y_max: float) -> bool:
        """True if traversable floor supports the whole rectangle (no hole or untraversable step)."""
        raise NotImplementedError


@dataclass(frozen=True)
class StartAreaResult:
    cells: Tuple[GridCell, ...]
    provenance: str
    verification: Dict[str, object]
    rejected: Tuple[Tuple[GridCell, str], ...]  # diagnostic: (cell, reason) for every excluded candidate

    def clearance_record(self) -> Dict[str, object]:
        """The clearance JSON A's ``replay pack`` expects: exactly cells, provenance, verification."""
        return dict(cells=self.cells, provenance=self.provenance, verification=self.verification)


def _candidate_cells(manifest) -> Tuple[GridCell, ...]:
    """Whole in-bounds cells whose four corners all lie within the declared radius."""
    c, pose, radius = manifest.mapping, manifest.start_pose, manifest.initial_clearance_radius_m
    cells = []
    for row in range(c.height_cells):
        for column in range(c.width_cells):
            if all(hypot(c.origin_x_m + x * c.resolution_m - pose.x_m,
                         c.origin_y_m + y * c.resolution_m - pose.y_m) <= radius + _EPS
                   for x in (column, column + 1) for y in (row, row + 1)):
                cells.append(GridCell(row, column))
    return tuple(cells)


def verify_start_area(manifest, probe: SceneProbe, verifier_id: str) -> StartAreaResult:
    """Verify the largest connected start area; reject the episode if the footprint itself fails.

    ``manifest`` must select ``verified_start_area`` and a radius. Raises ValueError when
    initialization cannot be verified (any footprint-plus-margin cell fails).
    """
    manifest.validate()
    if manifest.initial_clearance_mode != 'verified_start_area':
        raise ValueError("manifest does not select the verified_start_area experiment")
    if not verifier_id.strip():
        raise ValueError("verifier id must be nonempty")
    c = manifest.mapping
    res = c.resolution_m
    # Check the full height band that A's binding demands, not a smaller convenient one.
    footprint = footprint_cells(manifest)
    probe_band = clearance_binding(manifest, footprint)
    z_min, z_max = probe_band['required_z_min_m'], probe_band['required_z_max_m']

    candidates = _candidate_cells(manifest)
    passed, rejected = set(), {}
    for cell in candidates:
        x0, y0 = c.origin_x_m + cell.column * res, c.origin_y_m + cell.row * res
        if not probe.column_clear(x0, y0, x0 + res, y0 + res, z_min, z_max):
            rejected[cell] = 'collision column blocked'
        elif not probe.floor_supported(x0, y0, x0 + res, y0 + res):
            rejected[cell] = 'floor not supported/traversable'
        else:
            passed.add(cell)

    seed = set(footprint)
    failed = sorted(cell for cell in seed if cell not in passed)
    if failed:
        raise ValueError("spawn footprint clearance cannot be verified: %s"
                         % ', '.join('(%d,%d) %s' % (cell.row, cell.column, rejected.get(cell, 'outside radius'))
                                     for cell in failed))

    connected, pending = set(seed), deque(seed)
    while pending:
        cell = pending.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbour = GridCell(cell.row + dr, cell.column + dc)
            if neighbour in passed and neighbour not in connected:
                connected.add(neighbour)
                pending.append(neighbour)
    for cell in passed - connected:
        rejected[cell] = 'not connected to the initial footprint'

    cells = tuple(sorted(connected, key=lambda x: (x.row, x.column)))
    binding = clearance_binding(manifest, cells)
    method = ("whole-cell collision column over world z [%.3f, %.3f] m against non-robot geometry, "
              "plus floor-support check per whole cell; 4-neighbour component of the footprint; "
              "radius %.2f m; %s" % (z_min, z_max, manifest.initial_clearance_radius_m, probe.description))
    verification = dict(context_sha256=binding['context_sha256'], cells_sha256=binding['cells_sha256'],
                        checked_z_min_m=z_min, checked_z_max_m=z_max,
                        all_cells_clear=True, floor_support_verified=True,
                        verifier=verifier_id, verification_method=method,
                        scene_snapshot_id=probe.snapshot_id)
    validate_clearance(manifest, cells, verification)   # fail here, not later in A's replay
    provenance = ("B start-area verifier %s: %d of %d candidate cells verified within %.2f m"
                  % (verifier_id, len(cells), len(candidates), manifest.initial_clearance_radius_m))
    return StartAreaResult(cells, provenance, verification,
                           tuple(sorted(rejected.items(), key=lambda kv: (kv[0].row, kv[0].column))))


class MockSceneProbe(SceneProbe):
    """Probe over the mock simulator's ground-truth grid, with optional heights.

    Occupied or unknown grid cells are full-height walls unless ``obstacle_z`` gives them a
    (z_low, z_high) extent, so low or high objects that miss the robot's height band do not
    block. ``floor_z`` gives per-cell floor height (None = hole); the floor supports a cell
    only if it is within ``max_step_m`` of ``nominal_floor_z`` (an untraversable step otherwise).
    Out-of-bounds space is blocked and unsupported, as everywhere in B's geometry.
    """

    def __init__(self, grid: OccupancyGrid, nominal_floor_z: float = 0.0, max_step_m: float = 0.02,
                 obstacle_z: Optional[Mapping[Tuple[int, int], Tuple[float, float]]] = None,
                 floor_z: Optional[Mapping[Tuple[int, int], Optional[float]]] = None,
                 snapshot_id: str = "mock-ground-truth") -> None:
        self._grid, self._nominal, self._step = grid, nominal_floor_z, max_step_m
        self._obstacle_z = dict(obstacle_z or {})
        self._floor_z = dict(floor_z or {})
        self.snapshot_id = snapshot_id
        self.description = ("MockSceneProbe on ground-truth grid, max floor step %.3f m (mock; not a physics check)"
                            % max_step_m)

    def _cells(self, x_min, y_min, x_max, y_max):
        g = self._grid
        # Shrink by epsilon so rectangles that merely touch a cell edge do not count it.
        col_lo = int(floor((x_min - g.origin_x_m) / g.resolution_m + _EPS))
        col_hi = int(floor((x_max - g.origin_x_m) / g.resolution_m - _EPS))
        row_lo = int(floor((y_min - g.origin_y_m) / g.resolution_m + _EPS))
        row_hi = int(floor((y_max - g.origin_y_m) / g.resolution_m - _EPS))
        return [(r, col) for r in range(row_lo, row_hi + 1) for col in range(col_lo, col_hi + 1)]

    def column_clear(self, x_min, y_min, x_max, y_max, z_min, z_max):
        for row, column in self._cells(x_min, y_min, x_max, y_max):
            if not in_bounds(self._grid, row, column):
                return False
            if self._grid.data[row][column] == FREE and (row, column) not in self._obstacle_z:
                continue
            low, high = self._obstacle_z.get((row, column), (float('-inf'), float('inf')))
            if low < z_max and high > z_min:   # obstacle extent overlaps the robot's height band
                return False
        return True

    def floor_supported(self, x_min, y_min, x_max, y_max):
        for row, column in self._cells(x_min, y_min, x_max, y_max):
            if not in_bounds(self._grid, row, column):
                return False
            height = self._floor_z.get((row, column), self._nominal)
            if height is None or abs(height - self._nominal) > self._step + _EPS:
                return False
        return True
