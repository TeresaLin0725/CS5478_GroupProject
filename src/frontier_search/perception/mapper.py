"""Owner A: synchronous RGB-D occupancy and geometric coverage mapping.

Private mutable buffers hold working data; published WorldState snapshots must
be independently owned and immutable.
"""

from dataclasses import dataclass, field, replace
from random import Random
from math import atan2, hypot, pi
from typing import Dict, List, Optional, Tuple

from ..config import MappingConfig, ObservationConfig
from ..contracts import (
    DetectionEvidence, EvidenceBundle, Frontier, GridCell, MapperInit, ObjectTrack,
    RGBDObservation, RobotConfig, TargetClaim, WorldState, OccupancyGrid,
    ExploredGrid, RegionSummary, Pose,
)
from ..immutable import require_immutable
from ..sensors import CameraIntrinsics
from ..validation import ContractError, is_finite_number, require, validate_state
from .fusion import fuse_observation
from .detection import RGBDetector, localize_detection, validate_detection
from .claims import confirm_target
from .tracking import associate_rooms, update_tracks
from .frontiers import FrontierMemory, assign_identities, build_candidates, _clearance
from .observations import validate_rgbd_observation
from .priors import rasterize_region, validate_region_priors
from .gain import camera_yaw_offset
from ..navigation.geometry import cells_overlapping_footprint


@dataclass
class _EpisodeState:
    """Hold one episode's private maps, tracks, evidence and counters.

    RGBDMapper replaces this entire container on each successful reset.
    """

    request: MapperInit
    occupancy: List[List[int]]
    coverage: List[List[bool]]
    rng: Random
    free_masks: List[List[int]] = field(default_factory=list)
    floor_seen: List[List[bool]] = field(default_factory=list)
    collision_z: Optional[Tuple[float, float]] = None
    base_z_m: Optional[float] = None
    intrinsics: Optional[CameraIntrinsics] = None
    region_cells: Optional[Tuple[Tuple[GridCell, ...], ...]] = None
    tracks: Dict[str, ObjectTrack] = field(default_factory=dict)
    track_regions: Dict[str, Optional[str]] = field(default_factory=dict)
    frontiers: Dict[str, Frontier] = field(default_factory=dict)
    frontier_memory: Dict[str, FrontierMemory] = field(default_factory=dict)
    frontier_history: Dict[str, Tuple[str, ...]] = field(default_factory=dict)
    evidence: Dict[str, DetectionEvidence] = field(default_factory=dict)
    target_claim: Optional[TargetClaim] = None
    claim_evidence: Optional[EvidenceBundle] = None
    entity_counters: Dict[str, int] = field(default_factory=lambda: {
        "track": 0, "frontier": 0, "observation": 0, "claim": 0,
    })
    revision: int = 0
    last_processed_sequence: Optional[int] = None
    last_timestamp_s: Optional[float] = None
    latest_state: Optional[WorldState] = None
    inspection_ids: frozenset = frozenset()
    clear_counts: List[List[int]] = field(default_factory=list)



def _nonempty_text(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


# Check that configuration and priors are valid for a new mapping episode.
def _validate_init(request: MapperInit) -> None:
    require(isinstance(request, MapperInit), "reset requires MapperInit")
    try:
        require_immutable(request)
    except (TypeError, ValueError) as exc:
        raise ContractError("invalid mapper initialization: %s" % exc) from exc

    require(_nonempty_text(request.episode_id), "missing episode ID")
    require(_nonempty_text(request.target_class), "missing target class")
    require(type(request.seed) is int, "seed must be an integer")
    require(type(request.oracle_room_labels) is bool, "oracle_room_labels must be boolean")
    require(isinstance(request.mapping_config, MappingConfig), "missing mapping configuration")
    require(isinstance(request.observation_config, ObservationConfig), "missing observation configuration")
    require(isinstance(request.robot_config, RobotConfig), "missing robot configuration")
    cfg = request.mapping_config
    require(type(cfg.width_cells) is int and cfg.width_cells > 0,
            "grid width must be a positive integer")
    require(type(cfg.height_cells) is int and cfg.height_cells > 0,
            "grid height must be a positive integer")
    require(is_finite_number(cfg.resolution_m) and cfg.resolution_m > 0,
            "grid resolution must be finite and positive")
    require(is_finite_number(cfg.origin_x_m) and is_finite_number(cfg.origin_y_m),
            "grid origin must be finite")
    try:
        cfg.__post_init__()
    except (TypeError, ValueError, OverflowError) as exc:
        raise ContractError("invalid mapping configuration: %s" % exc) from exc
    robot = request.robot_config
    require(is_finite_number(robot.collision_z_min_m)
            and is_finite_number(robot.collision_z_max_m)
            and robot.collision_z_min_m < robot.collision_z_max_m,
            "invalid robot collision height interval")
    require(len(robot.footprint_base) >= 3
            and all(is_finite_number(p.x_m) and is_finite_number(p.y_m) for p in robot.footprint_base)
            and is_finite_number(robot.clearance_margin_m) and robot.clearance_margin_m >= 0,
            "invalid footprint or margin for frontier approach placement")
    require(bool(request.observation_config.arrival_headings_rad)
            and all(is_finite_number(h) for h in request.observation_config.arrival_headings_rad),
            "arrival sweep must contain finite headings")

    require(isinstance(request.initial_clearance_cells, tuple), "clearance cells must be a tuple")
    for cell in request.initial_clearance_cells:
        require(isinstance(cell, GridCell), "clearance requires GridCell records")
        require(type(cell.row) is int and type(cell.column) is int,
                "clearance indices must be integers")
        require(0 <= cell.row < cfg.height_cells and 0 <= cell.column < cfg.width_cells,
                "clearance cell outside grid")

    validate_region_priors(request.region_priors, request.oracle_room_labels)


class RGBDMapper:
    """Manage perception lifecycle through the PerceptionMapper API."""

    def __init__(self, detector: Optional[RGBDetector] = None) -> None:
        self._state = None  # type: Optional[_EpisodeState]
        self._detector = detector

    # Prepare the mapper for a new episode and clear previous episode data.
    def reset(self, request: MapperInit) -> None:
        """Validate and replace the episode, without publishing an observation.

        Room geometry never seeds free space. Only the supplied verified spawn
        cells are free, and they do not count as geometric observation coverage.
        Failed validation or allocation leaves the previous episode untouched.
        """
        _validate_init(request)
        cfg = request.mapping_config
        occupancy = [[-1 for _ in range(cfg.width_cells)] for _ in range(cfg.height_cells)]
        coverage = [[False for _ in range(cfg.width_cells)] for _ in range(cfg.height_cells)]
        for cell in request.initial_clearance_cells:
            occupancy[cell.row][cell.column] = 0
        new_state = _EpisodeState(request, occupancy, coverage, Random(request.seed))
        new_state.free_masks = [[0] * cfg.width_cells for _ in range(cfg.height_cells)]
        new_state.floor_seen = [[False] * cfg.width_cells for _ in range(cfg.height_cells)]
        new_state.clear_counts = [[0] * cfg.width_cells for _ in range(cfg.height_cells)]
        self._state = new_state

    def update(self, observation: RGBDObservation) -> WorldState:
        """Atomically process one fresh frame and publish an owned snapshot.

        An optional RGB detector publishes spatially associated object tracks
        and observed room classes. A confirmed target latches one terminal claim.
        The flat-floor model bounds base elevation drift by the configured margin;
        camera pose may translate and rotate freely with each observation.
        """
        previous = self._state
        require(previous is not None, "mapper must be reset before update")
        validate_rgbd_observation(observation)
        request = previous.request
        cfg, robot = request.mapping_config, request.robot_config
        require(observation.episode_id == request.episode_id, "observation episode mismatch")
        require(previous.last_processed_sequence is None
                or observation.sequence > previous.last_processed_sequence,
                "processed sequence must strictly increase")
        require(previous.last_timestamp_s is None or observation.timestamp_s >= previous.last_timestamp_s,
                "observation timestamp moved backwards")
        require(previous.intrinsics is None or previous.intrinsics == observation.camera_intrinsics,
                "camera calibration changed within episode")
        require(previous.base_z_m is None or abs(previous.base_z_m - observation.robot_pose.z_m)
                <= cfg.base_elevation_margin_m + 1e-6,
                "flat-floor mapper base elevation exceeds margin; reset for another floor")
        collision_z = previous.collision_z or (
            max(observation.robot_pose.z_m + robot.collision_z_min_m - cfg.base_elevation_margin_m,
                cfg.floor_height_m + cfg.floor_tolerance_m),
            observation.robot_pose.z_m + robot.collision_z_max_m + cfg.base_elevation_margin_m)
        require(all(is_finite_number(z) for z in collision_z) and collision_z[0] < collision_z[1],
                "collision envelope must extend above the configured floor band")
        bin_count = (collision_z[1] - collision_z[0]) / cfg.clearance_bin_height_m
        require(is_finite_number(bin_count) and 0 < bin_count <= 4096,
                "collision envelope requires between 1 and 4096 vertical bins")
        region_cells = previous.region_cells
        if region_cells is None:
            region_cells = tuple(rasterize_region(region, cfg) for region in request.region_priors)
        # All computation uses private copies. Rejected frames cannot consume a
        # sequence number or partially modify maps/evidence for the next update.
        working = replace(previous,
            occupancy=[row[:] for row in previous.occupancy],
            coverage=[row[:] for row in previous.coverage],
            free_masks=[row[:] for row in previous.free_masks],
            floor_seen=[row[:] for row in previous.floor_seen],
            clear_counts=[row[:] for row in previous.clear_counts])
        detections = () if self._detector is None else tuple(self._detector.detect(observation.color_image))
        if cfg.target_max_extent_m is not None:
            filtered=[]
            for detection in detections:
                validate_detection(detection,observation.color_image)
                if detection.class_name == request.target_class:
                    position=localize_detection(detection,observation,cfg)
                    if position is not None:
                        matrix=observation.camera_to_world.matrix
                        depth=sum(matrix[i][2]*(getattr(position,axis)-matrix[i][3])
                                  for i,axis in enumerate(('x_m','y_m','z_m')))
                        x0,y0,x1,y1=detection.bbox_xyxy_px
                        extent=max((x1-x0)*depth/observation.camera_intrinsics.fx,
                                   (y1-y0)*depth/observation.camera_intrinsics.fy)
                        if extent>cfg.target_max_extent_m:
                            continue
                filtered.append(detection)
            detections=tuple(filtered)
        tracks, evidence, track_counter, observation_counter = update_tracks(
            previous.tracks, previous.evidence, detections, observation, cfg,
            previous.entity_counters['track'], previous.entity_counters['observation'])
        track_regions, room_classes = associate_rooms(tracks, request.region_priors)
        claim = previous.target_claim
        claim_evidence = previous.claim_evidence
        claim_counter = previous.entity_counters['claim']
        eligible = None
        if cfg.inspection_distance_m is not None:
            eligible = set(previous.inspection_ids)
            dt = observation.timestamp_s - (previous.last_timestamp_s or 0.)
            old_pose = previous.latest_state.robot_pose if previous.latest_state else None
            pose = observation.robot_pose
            stopped = (old_pose is not None and dt > 0 and
                hypot(pose.x_m-old_pose.x_m, pose.y_m-old_pose.y_m)/dt <= cfg.inspection_linear_speed_mps and
                abs((pose.yaw_rad-old_pose.yaw_rad+pi) % (2*pi)-pi)/dt <= cfg.inspection_angular_speed_radps)
            if stopped:
                eligible.update(key for key, item in evidence.items()
                    if item.capture_sequence == observation.sequence and item.class_name == request.target_class
                    and item.position_world is not None and hypot(item.position_world.x_m-pose.x_m,
                        item.position_world.y_m-pose.y_m) <= cfg.inspection_distance_m)
            # Keep only the current confirmation window, bounding long episodes.
            eligible = {key for key in eligible if observation.timestamp_s-evidence[key].timestamp_s
                        <= cfg.confirmation_window_s + 1e-9}
            working.inspection_ids = frozenset(eligible)
        if claim is None:
            claim = confirm_target(tracks, evidence, request.target_class,
                observation.sequence, observation.timestamp_s, cfg, claim_counter, eligible)
            if claim is not None:
                claim_counter += 1
                claim_evidence = EvidenceBundle(request.episode_id, claim.claim_id,
                    tuple(evidence[key] for key in claim.evidence_observation_ids))
        fusion_args = (observation, cfg, collision_z, working.occupancy,
                       working.coverage, working.free_masks, working.floor_seen)
        if cfg.dynamic_clear_frames:
            fuse_observation(*fusion_args, clear_counts=working.clear_counts,
                increment_clear_count=(previous.last_timestamp_s is None or
                                       observation.timestamp_s > previous.last_timestamp_s))
        else:
            fuse_observation(*fusion_args)
        revision = previous.revision + 1
        geometry = dict(frame_id="world", revision=revision, resolution_m=cfg.resolution_m,
                        origin_x_m=cfg.origin_x_m, origin_y_m=cfg.origin_y_m,
                        width_cells=cfg.width_cells, height_cells=cfg.height_cells)
        occupancy = OccupancyGrid(data=tuple(tuple(row) for row in working.occupancy), **geometry)
        explored = ExploredGrid(data=tuple(tuple(row) for row in working.coverage), **geometry)
        regions = tuple(RegionSummary(
            region.region_id, region.boundary_world, "igibson_room_prior",
            sum(working.coverage[c.row][c.column] for c in cells) / len(cells) if cells else None,
            room_classes[region.region_id], region.room_type,
            "simulator_annotation" if region.room_type is not None else None)
            for region, cells in zip(request.region_priors, region_cells))
        candidates = build_candidates(occupancy, observation, robot, cfg, request.observation_config,
            {region.region_id: cells for region, cells in zip(request.region_priors, region_cells)},
            cluster_span_m=cfg.frontier_cluster_span_m)
        frontiers, memory, history, counter = assign_identities(
            candidates, previous.frontier_memory, previous.frontier_history,
            previous.entity_counters['frontier'], observation.timestamp_s, cfg)
        if cfg.inspection_distance_m is not None and claim is None:
            frontiers += _target_approaches(occupancy, observation, tracks, request.target_class, cfg,robot)
            if cfg.dynamic_clear_frames:
                frontiers += _navigation_probes(occupancy,observation,robot)
        state = WorldState("1.1" if cfg.inspection_distance_m is not None else "1.0", request.episode_id, revision, observation.timestamp_s,
                           observation.sequence, request.target_class, observation.robot_pose,
                           occupancy, explored, tuple(tracks.values()), regions, frontiers, claim)
        validate_state(state, oracle_room_labels=request.oracle_room_labels)
        working.revision = revision
        working.last_processed_sequence = observation.sequence
        working.last_timestamp_s = observation.timestamp_s
        working.latest_state = state
        working.intrinsics = observation.camera_intrinsics
        working.base_z_m = previous.base_z_m if previous.base_z_m is not None else observation.robot_pose.z_m
        working.collision_z = collision_z
        working.region_cells = region_cells
        working.frontiers = {f.frontier_id: f for f in frontiers}
        working.frontier_memory = memory
        working.frontier_history = history
        working.tracks = tracks
        working.track_regions = track_regions
        working.evidence = evidence
        working.target_claim = claim
        working.claim_evidence = claim_evidence
        working.entity_counters = dict(previous.entity_counters, frontier=counter,
                                      track=track_counter, observation=observation_counter,
                                      claim=claim_counter)
        self._state = working
        return state

    def get_claim_evidence(self, episode_id: str, claim_id: str) -> EvidenceBundle:
        """Return the immutable evidence frozen at the first claim, for scoring."""
        state = self._state
        require(state is not None, "mapper must be reset before evidence retrieval")
        require(episode_id == state.request.episode_id, "evidence episode mismatch")
        require(state.target_claim is not None and state.claim_evidence is not None,
                "no target claim available")
        require(claim_id == state.target_claim.claim_id, "unknown claim ID")
        return state.claim_evidence


def _target_approaches(grid, observation, tracks, target_class, cfg,robot):
    """Observed target positions propose known-free goals; B checks each full footprint/path."""
    result = []
    clearance=None
    radius=max(hypot(v.x_m,v.y_m) for v in robot.footprint_base)+robot.clearance_margin_m
    for track in sorted(tracks.values(), key=lambda item: item.track_id):
        if track.class_name != target_class or track.position_world is None:
            continue
        if (track.confirmation_count < cfg.confirmation_frames and
                observation.timestamp_s-track.last_seen_s > cfg.tentative_target_memory_s):
            continue
        if clearance is None:clearance=_clearance(grid)
        p = track.position_world
        sectors = {}
        for r, row in enumerate(grid.data):
            for c, value in enumerate(row):
                if value != 0:
                    continue
                x = grid.origin_x_m + (c+.5)*grid.resolution_m
                y = grid.origin_y_m + (r+.5)*grid.resolution_m
                distance = hypot(x-p.x_m, y-p.y_m)
                if not .4 <= distance <= cfg.inspection_distance_m-.03:
                    continue
                sector = int((atan2(y-p.y_m, x-p.x_m)+pi)/(pi/4)) % 8
                enough=(clearance[r][c]-.5)*grid.resolution_m>radius
                score = (not enough,abs(distance-(cfg.inspection_distance_m-.10)),
                         hypot(x-observation.robot_pose.x_m, y-observation.robot_pose.y_m), r, c)
                if sector not in sectors or score < sectors[sector][0]:
                    sectors[sector] = score, x, y, r, c
        for sector, (_, x, y, r, c) in sorted(sectors.items()):
            yaw = (atan2(p.y_m-y, p.x_m-x)-camera_yaw_offset(observation)+pi)%(2*pi)-pi
            result.append(Frontier('approach-%s-%d' % (track.track_id, sector), (),
                (GridCell(r,c),), Pose(x,y,observation.robot_pose.z_m,yaw), None, 0.,
                hypot(x-observation.robot_pose.x_m,y-observation.robot_pose.y_m), 'target_approach',track.track_id))
    return tuple(result)


def _navigation_probes(grid,observation,robot):
    """Known-free alternate viewpoints for stale/occluded routes, assessed by B.

    A fixed RGB-D camera cannot see floor immediately beside the base. Backing
    away through previously sensed free space can reveal a departed obstacle's
    entire collision column. No physical scene query or map override is used.
    """
    p=observation.robot_pose;sectors={}
    for r,row in enumerate(grid.data):
        for c,value in enumerate(row):
            if value!=0:continue
            x=grid.origin_x_m+(c+.5)*grid.resolution_m
            y=grid.origin_y_m+(r+.5)*grid.resolution_m
            distance=hypot(x-p.x_m,y-p.y_m)
            if not .75<=distance<=1.05:continue
            yaw=atan2(p.y_m-y,p.x_m-x)
            pose=Pose(x,y,p.z_m,yaw)
            cells=tuple(cells_overlapping_footprint(grid,robot.footprint_base,pose,robot.clearance_margin_m))
            if not cells or any(not(0<=rr<grid.height_cells and 0<=cc<grid.width_cells)
                                or grid.data[rr][cc]!=0 for rr,cc in cells):continue
            sector=int((atan2(y-p.y_m,x-p.x_m)+pi)/(pi/4))%8
            score=(abs(distance-.85),r,c)
            if sector not in sectors or score<sectors[sector][0]:sectors[sector]=(score,pose,r,c)
    return tuple(Frontier('probe-%03d-%03d'%(r,c),(),(GridCell(r,c),),pose,None,0.,
        hypot(pose.x_m-p.x_m,pose.y_m-p.y_m),'navigation_probe',None)
        for _,pose,r,c in (value for _,value in sorted(sectors.items())))
