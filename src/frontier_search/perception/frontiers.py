"""Deterministic observed-map frontiers, interior approach poses and lineage."""

from collections import deque
from dataclasses import dataclass, replace
from math import atan2, floor, hypot, pi
from typing import Dict, List, Optional, Tuple

from ..config import MappingConfig, ObservationConfig
from ..contracts import Frontier, GridCell, OccupancyGrid, Pose, RGBDObservation, RobotConfig
from .gain import camera_yaw_offset, estimate_gain
from ..navigation.geometry import footprint_collides


CARDINAL = ((-1, 0), (0, -1), (0, 1), (1, 0))
EIGHT = tuple((r, c) for r in (-1, 0, 1) for c in (-1, 0, 1) if r or c)


@dataclass(frozen=True)
class FrontierCandidate:
    support_cells: Tuple[GridCell, ...]
    approach_pose: Pose
    region_id: Optional[str]
    information_gain_m2: float
    distance_from_robot_m: float


@dataclass(frozen=True)
class FrontierMemory:
    frontier: Frontier
    last_seen_s: float
    active: bool = True


def extract_clusters(grid: OccupancyGrid, config: MappingConfig) -> Tuple[Tuple[GridCell, ...], ...]:
    """Free cells with an in-map cardinal unknown neighbor; row-major ordering.

    Out-of-map space never creates a frontier. Cluster adjacency is separately
    configurable (4 or 8); minimum size filtering does not change occupancy.
    """
    remaining = {(r, c) for r in range(grid.height_cells) for c in range(grid.width_cells)
                 if grid.data[r][c] == 0 and any(
                     0 <= r + dr < grid.height_cells and 0 <= c + dc < grid.width_cells
                     and grid.data[r + dr][c + dc] == -1 for dr, dc in CARDINAL)}
    clusters = []
    neighbors = CARDINAL if config.frontier_connectivity == 4 else EIGHT
    for seed in sorted(remaining):
        if seed not in remaining:
            continue
        remaining.remove(seed)
        pending, cells = [seed], []
        while pending:
            r, c = pending.pop()
            cells.append((r, c))
            for dr, dc in neighbors:
                cell = r + dr, c + dc
                if cell in remaining:
                    remaining.remove(cell)
                    pending.append(cell)
        if len(cells) >= config.frontier_min_cells:
            clusters.append(tuple(GridCell(r, c) for r, c in sorted(cells)))
    return tuple(clusters)


def _clearance(grid: OccupancyGrid) -> List[List[int]]:
    """Chebyshev cell distance to nonfree/out-of-map for approach placement only."""
    h, w = grid.height_cells, grid.width_cells
    distances = [[h + w + 1] * w for _ in range(h)]
    seeds = []
    for r in range(h):
        for c in range(w):
            if grid.data[r][c] != 0:
                distances[r][c] = 0
                seeds.append((0, r, c))
            elif r in (0, h - 1) or c in (0, w - 1):
                distances[r][c] = 1
                seeds.append((1, r, c))
    pending = deque((r, c) for _, r, c in sorted(seeds))
    while pending:
        r, c = pending.popleft()
        distance = distances[r][c] + 1
        for dr, dc in EIGHT:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and distances[nr][nc] > distance:
                distances[nr][nc] = distance
                pending.append((nr, nc))
    return distances


def _approach(grid, support, clearance, robot, observation, config):
    """Retreat into free cells; B still owns path reachability.

    Prefer the nearest interior cell with conservative disc clearance; otherwise
    check remaining cells against the exact footprint. Never publish a pose
    that overlaps blocked space. Recomputed for every observed map revision.
    """
    radius = max(hypot(p.x_m, p.y_m) for p in robot.footprint_base) + robot.clearance_margin_m
    middle_r = sum(c.row for c in support) / len(support)
    middle_c = sum(c.column for c in support) / len(support)
    distance = {(cell.row, cell.column): 0 for cell in support}
    queue = deque(sorted(distance))
    maximum = floor(config.frontier_approach_search_m / grid.resolution_m)
    while queue:
        r, c = queue.popleft()
        if distance[r, c] >= maximum:
            continue
        for dr, dc in CARDINAL:
            nr, nc = r + dr, c + dc
            if (0 <= nr < grid.height_cells and 0 <= nc < grid.width_cells
                    and grid.data[nr][nc] == 0 and (nr, nc) not in distance):
                distance[nr, nc] = distance[r, c] + 1
                queue.append((nr, nc))
    def ranking(cell):
        r, c = cell
        enough = (clearance[r][c] - 0.5) * grid.resolution_m > radius
        return (not enough, distance[cell] if enough else -clearance[r][c],
                (r - middle_r) ** 2 + (c - middle_c) ** 2, r, c)
    for r, c in sorted(distance, key=ranking):
        pose = _oriented_pose(grid, support, middle_r, middle_c, r, c, observation)
        if not footprint_collides(grid, robot.footprint_base, pose, robot.clearance_margin_m):
            return pose
    return None


def _oriented_pose(grid, support, middle_r, middle_c, r, c, observation):
    # Pick a representative boundary/unknown pair, without averaging opposing
    # normals (which cancel on a closed frontier around a free island).
    unknown = []
    for cell in support:
        for dr, dc in CARDINAL:
            nr, nc = cell.row + dr, cell.column + dc
            if (0 <= nr < grid.height_cells and 0 <= nc < grid.width_cells
                    and grid.data[nr][nc] == -1):
                unknown.append(((cell.row - middle_r) ** 2 + (cell.column - middle_c) ** 2,
                                (nr - r) ** 2 + (nc - c) ** 2, nr, nc))
    _, _, ur, uc = min(unknown)
    yaw = atan2(ur - r, uc - c) - camera_yaw_offset(observation)
    yaw = (yaw + pi) % (2 * pi) - pi
    return Pose(grid.origin_x_m + (c + 0.5) * grid.resolution_m,
                grid.origin_y_m + (r + 0.5) * grid.resolution_m, observation.robot_pose.z_m, yaw)


def build_candidates(grid: OccupancyGrid, observation: RGBDObservation, robot: RobotConfig,
                     mapping: MappingConfig, routine: ObservationConfig,
                     region_cells: Dict[str, Tuple[GridCell, ...]],
                     cluster_span_m: float = 0.) -> Tuple[FrontierCandidate, ...]:
    clusters = extract_clusters(grid, mapping)
    if cluster_span_m > 0:
        # A connected visible/unknown contour may span several rooms. Local
        # opportunities must not share a single approach pose and retry budget.
        span = max(1, int(cluster_span_m / grid.resolution_m))
        pieces = []
        for cluster in clusters:
            buckets = {}
            for cell in cluster:
                buckets.setdefault((cell.row // span, cell.column // span), []).append(cell)
            pieces.extend(tuple(cells) for _, cells in sorted(buckets.items())
                          if len(cells) >= mapping.frontier_min_cells)
        clusters = tuple(pieces)
    if not clusters:
        return ()
    clearance = _clearance(grid)
    regions = {name: set(cells) for name, cells in region_cells.items()}
    result = []
    for support in clusters:
        pose = _approach(grid, support, clearance, robot, observation, mapping)
        if pose is None:
            continue
        counts = [(sum(cell in cells for cell in support), name) for name, cells in regions.items()]
        # Associate with the majority of boundary support; deterministic ID tie.
        positive = sorted((-count, name) for count, name in counts if count)
        region = positive[0][1] if positive else None
        result.append(FrontierCandidate(support, pose, region,
            estimate_gain(grid, pose, observation, mapping, routine),
            hypot(pose.x_m - observation.robot_pose.x_m, pose.y_m - observation.robot_pose.y_m)))
    return tuple(result)


def assign_identities(candidates: Tuple[FrontierCandidate, ...], memory: Dict[str, FrontierMemory],
                      lineage: Dict[str, Tuple[str, ...]], counter: int, timestamp_s: float,
                      config: MappingConfig):
    """Return new records and history without modifying any input.

    A one-to-one overlap keeps its ID. Splits/merges retire matched parents and
    allocate new IDs with all direct matched parents. Dormant, unretired records
    remain matchable until the history window expires. Retired records never
    compete with their descendants; compact lineage and the serial persist.
    """
    candidates = tuple(sorted(candidates, key=lambda f: tuple((c.row, c.column) for c in f.support_cells)))
    kept = {key: entry for key, entry in memory.items()
            if entry.active or timestamp_s - entry.last_seen_s <= config.frontier_history_s}
    old = {key: {(c.row, c.column) for c in entry.frontier.support_cells} for key, entry in kept.items()}
    tolerance = config.frontier_match_tolerance_cells
    def dilate(cells):
        return {(r + dr, c + dc) for r, c in cells
                for dr in range(-tolerance, tolerance + 1) for dc in range(-tolerance, tolerance + 1)}
    dilated = {key: dilate(cells) for key, cells in old.items()}
    # Exact support wins before tolerant overlap. Otherwise two unchanged nearby
    # clusters could repeatedly look like a split/merge solely due to dilation.
    exact = {}
    locked = set()
    for index, candidate in enumerate(candidates):
        cells = {(c.row, c.column) for c in candidate.support_cells}
        same = [key for key in old if old[key] == cells and key not in locked]
        if same:
            identity = min(same, key=lambda key: (not kept[key].active, -kept[key].last_seen_s, key))
            exact[index] = identity
            locked.add(identity)
    matches = []
    children = {key: [] for key in kept}
    for index, candidate in enumerate(candidates):
        cells = {(c.row, c.column) for c in candidate.support_cells}
        vicinity = dilate(cells)
        parents = (exact[index],) if index in exact else tuple(sorted(key for key, previous in old.items()
            if key not in locked
            if max(len(cells & dilated[key]) / len(cells), len(previous & vicinity) / len(previous))
            >= config.frontier_match_fraction))
        matches.append(parents)
        for key in parents:
            children[key].append(index)
    result, new_lineage = [], dict(lineage)
    new_memory = {key: replace(entry, active=False) for key, entry in kept.items() if not children[key]}
    for candidate, parents in zip(candidates, matches):
        if len(parents) == 1 and len(children[parents[0]]) == 1:
            identity = parents[0]
            parents = kept[identity].frontier.parent_frontier_ids
        else:
            counter += 1
            identity = 'F%06d' % counter
        frontier = Frontier(identity, parents, candidate.support_cells, candidate.approach_pose,
                            candidate.region_id, candidate.information_gain_m2, candidate.distance_from_robot_m)
        result.append(frontier)
        new_memory[identity] = FrontierMemory(frontier, timestamp_s)
        new_lineage[identity] = parents
    return tuple(sorted(result, key=lambda f: f.frontier_id)), new_memory, new_lineage, counter
