"""Executable W2 manifest. Robot/scene choices must be supplied explicitly."""
from dataclasses import dataclass, field, fields
import json
from math import isfinite
from pathlib import Path
from typing import Tuple

from ..config import (EpisodeLimits, EvaluationConfig, MappingConfig, NavigationConfig,
                      ObservationConfig, PolicyConfig, RecoveryConfig)
from ..contracts import Point2, Pose, RegionPrior, RobotConfig
from ..immutable import require_immutable
from ..validation import require


@dataclass(frozen=True)
class EpisodeManifest:
    episode_id: str
    scene_id: str
    source: str
    target_class: str
    start_pose: Pose
    robot: RobotConfig
    policy_name: str = 'nearest'
    mapping: MappingConfig = field(default_factory=MappingConfig)
    observation: ObservationConfig = field(default_factory=ObservationConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    policy: PolicyConfig = field(default_factory=PolicyConfig)
    recovery: RecoveryConfig = field(default_factory=RecoveryConfig)
    limits: EpisodeLimits = field(default_factory=EpisodeLimits)
    evaluation: EvaluationConfig = field(default_factory=EvaluationConfig)
    region_priors: Tuple[RegionPrior, ...] = ()
    environment_seed: int = 0
    perception_seed: int = 1
    policy_seed: int = 2
    oracle_room_labels: bool = False
    capture_every_steps: int = 5
    module_deadline_s: float = 10.0
    # W2 is geometry-only. Claims are rejected unless scoring is explicitly enabled.
    enable_target_evaluation: bool = False
    initial_clearance_mode: str = 'footprint'
    initial_clearance_radius_m: float = 0.0

    def validate(self) -> None:
        require_immutable(self)
        require(bool(self.episode_id.strip() and self.scene_id.strip() and self.target_class.strip()),
                'episode, scene and target must be nonempty')
        require(self.source in ('synthetic', 'rgbd_replay', 'igibson'), 'invalid source')
        require(self.policy_name in ('nearest', 'random', 'information_gain', 'llm_guided'), 'invalid policy')
        require(self.initial_clearance_mode in ('footprint', 'verified_start_area'), 'invalid initial clearance mode')
        radius = self.initial_clearance_radius_m
        require(type(radius) in (int, float) and isfinite(radius) and radius >= 0,
                'initial clearance radius must be finite and nonnegative')
        require((self.initial_clearance_mode == 'footprint' and radius == 0)
                or (self.initial_clearance_mode == 'verified_start_area' and radius > 0),
                'footprint mode requires zero radius; verified start area requires a positive radius')
        for seed in (self.environment_seed, self.perception_seed, self.policy_seed):
            require(type(seed) is int, 'seeds must be integers')
        for flag in (self.oracle_room_labels, self.enable_target_evaluation):
            require(type(flag) is bool, 'flags must be boolean')
        positive = (self.module_deadline_s, self.limits.max_simulated_time_s,
                    self.limits.max_wall_time_s, self.navigation.batch_deadline_s,
                    self.navigation.navigation_deadline_s, self.navigation.pose_freshness_m,
                    self.navigation.map_freshness_s, self.observation.heading_tolerance_rad,
                    self.observation.operation_deadline_s, self.observation.drain_deadline_s,
                    self.recovery.progress_radius_m, self.evaluation.deadline_s)
        require(all(type(x) in (int, float) and isfinite(x) and x > 0 for x in positive),
                'deadlines and tolerances must be finite and positive')
        counters = (self.capture_every_steps, self.limits.max_steps, self.limits.max_attempts,
                    self.limits.max_decisions, self.observation.processed_samples_per_heading,
                    self.recovery.evaluation_retry_cap, self.recovery.navigation_retry_cap,
                    self.recovery.opportunity_attempt_cap, self.recovery.no_progress_cap,
                    self.recovery.max_cycles_without_navigation)
        require(all(type(x) is int and x > 0 for x in counters), 'budgets must be positive integers')
        require(type(self.navigation.max_replans) is int and self.navigation.max_replans >= 0,
                'invalid replan budget')
        require(all(isfinite(x) and x >= 0 for x in (self.observation.dwell_s,
                    self.recovery.cooldown_s, self.recovery.minimum_progress_m2)),
                'invalid dwell, cooldown or progress threshold')
        for headings in (self.observation.initial_headings_rad,
                         self.observation.arrival_headings_rad, self.observation.recovery_headings_rad):
            require(bool(headings) and all(isfinite(x) for x in headings), 'empty/invalid scan headings')
        r = self.robot
        require(len(r.footprint_base) >= 3 and r.can_turn_in_place,
                'W2 requires a polygon footprint and turn-in-place capability')
        area = sum(a.x_m*b.y_m - a.y_m*b.x_m for a, b in
                   zip(r.footprint_base, r.footprint_base[1:] + r.footprint_base[:1]))
        require(abs(area) > 1e-9 and r.collision_z_min_m < r.collision_z_max_m,
                'degenerate footprint or collision height')
        require(r.clearance_margin_m >= 0 and all(x > 0 for x in
                (r.max_linear_speed_mps, r.max_angular_speed_radps,
                 r.position_tolerance_m, r.yaw_tolerance_rad)), 'invalid robot geometry/limits')
        self.mapping.__post_init__()
        self.policy.__post_init__()


def load_manifest(path: Path) -> EpisodeManifest:
    """Strict field names prevent a typo from silently using a default."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    require(isinstance(data, dict), 'manifest must be a JSON object')
    require(set(data) <= {f.name for f in fields(EpisodeManifest)}, 'unknown manifest field')
    data['start_pose'] = Pose(**data['start_pose'])
    robot = dict(data['robot'])
    robot['footprint_base'] = tuple(Point2(**p) for p in robot['footprint_base'])
    data['robot'] = RobotConfig(**robot)
    for name, cls in (('mapping', MappingConfig), ('observation', ObservationConfig),
                      ('navigation', NavigationConfig), ('policy', PolicyConfig),
                      ('recovery', RecoveryConfig), ('limits', EpisodeLimits),
                      ('evaluation', EvaluationConfig)):
        if name in data:
            values = dict(data[name])
            if name == 'observation':
                for key in ('initial_headings_rad', 'arrival_headings_rad', 'recovery_headings_rad'):
                    if key in values:
                        values[key] = tuple(values[key])
            data[name] = cls(**values)
    data['region_priors'] = tuple(RegionPrior(p['region_id'],
        tuple(Point2(**v) for v in p['boundary_world']), p.get('room_type'))
        for p in data.get('region_priors', ()))
    manifest = EpisodeManifest(**data)
    manifest.validate()
    return manifest
