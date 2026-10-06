"""Authored progressive-map test double for C, NOT A's RGB-D mapper.

The one-pixel camera carries only timestamps/poses. Reveal stages are scripted
by proximity to authored goals. Never use these results as perception evidence.
"""
from dataclasses import replace
from math import cos, hypot, sin
from struct import pack

from ..config import ObservationConfig, RecoveryConfig
from ..contracts import (ExploredGrid, Frontier, GridCell, OccupancyGrid, Pose,
                         RGBDObservation, WorldState)
from ..navigation.dev_robot import PLACEHOLDER_ROBOT
from ..navigation.navigator import TeamNavigator
from ..perception.observations import validate_rgbd_observation
from ..policies.baselines import BaselinePolicy
from ..policies.llm import LLMGuidedPolicy
from ..contracts import PolicyName
from ..runner.episode import EpisodeRunner
from ..runner.manifest import EpisodeManifest
from ..sensors import CameraIntrinsics, ImageBuffer, Transform
from ..simulation.mock_adapter import MockSimulator
from ..validation import require


def mock_manifest(scenario='progress'):
    return EpisodeManifest('w2-'+scenario, 'mock_'+scenario, 'synthetic', 'cup',
        Pose(1.05, 1.55, 0, 0), PLACEHOLDER_ROBOT,
        observation=ObservationConfig(initial_headings_rad=(0.0, .4, 0.0),
            arrival_headings_rad=(-.3, .3, 0.0), recovery_headings_rad=(-.3, .3, 0.0), dwell_s=.05),
        recovery=RecoveryConfig(cooldown_s=.3, opportunity_attempt_cap=3, no_progress_cap=2))


def _grid(revision, right):
    data = tuple(tuple(0 if 5 <= r < 35 and 5 <= c < right else -1
                       for c in range(40)) for r in range(40))
    return OccupancyGrid('world', revision, .1, 0, 0, 40, 40, data)


class FixtureSimulator(MockSimulator):
    def capture(self):
        obs = super().capture()
        pose = obs.robot_pose
        c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
        transform = Transform(((s, 0., c, pose.x_m), (-c, 0., s, pose.y_m),
                               (0., -1., 0., .3), (0., 0., 0., 1.)))
        return RGBDObservation(obs.episode_id, obs.sequence, obs.timestamp_s,
            ImageBuffer(1, 1, 'rgb8', b'\x00\x00\x00'),
            ImageBuffer(1, 1, 'depth32f_le', pack('<f', 0.0)),
            CameraIntrinsics(1, 1, 1., 1., 0., 0.), transform, pose)


class ScriptedMapper:
    def __init__(self, scenario='progress'):
        self.scenario = scenario

    def reset(self, request):
        self.request = request
        self.revision, self.stage = 0, 0

    def update(self, obs):
        validate_rgbd_observation(obs)
        self.revision += 1
        goals = (Pose(1.95, 1.55, 0, 0), Pose(2.95, 1.55, 0, 0))
        if self.scenario == 'progress' and self.stage < 2:
            goal = goals[self.stage]
            if hypot(obs.robot_pose.x_m-goal.x_m, obs.robot_pose.y_m-goal.y_m) <= .11:
                self.stage += 1
        grid = _grid(self.revision, (23, 33, 35)[self.stage])
        explored = ExploredGrid(grid.frame_id, grid.revision, grid.resolution_m,
            grid.origin_x_m, grid.origin_y_m, grid.width_cells, grid.height_cells,
            tuple(tuple(v != -1 for v in row) for row in grid.data))
        frontiers = ()
        if self.stage < 2 and self.scenario != 'no_frontiers':
            goal = goals[self.stage]
            frontiers = (Frontier('F%d' % (self.stage+1), (),
                (GridCell(15, (22, 32)[self.stage]),), goal, None, 1.0,
                hypot(obs.robot_pose.x_m-goal.x_m, obs.robot_pose.y_m-goal.y_m)),)
        return WorldState('1.0', obs.episode_id, self.revision, obs.timestamp_s, obs.sequence,
            self.request.target_class, obs.robot_pose, grid, explored, (), (), frontiers, None)

    def get_claim_evidence(self, episode_id, claim_id):
        raise ValueError('geometry-only fixture never makes claims')


def build_mock_runner(manifest, logger=None):
    """Factory seam: a real factory replaces only adapter and mapper construction."""
    require(manifest.source == 'synthetic', 'mock factory cannot run a real manifest')
    require(manifest.scene_id in ('mock_progress', 'mock_blocked', 'mock_no_progress', 'mock_no_frontiers'),
            'unknown mock scenario')
    scenario = manifest.scene_id[len('mock_'):]
    true_grid = _grid(1, 35)
    if scenario == 'blocked':
        cells = [list(row) for row in true_grid.data]
        for row in range(5, 35):
            cells[row][16] = 100
        true_grid = replace(true_grid, data=tuple(tuple(row) for row in cells))
    simulator = FixtureSimulator(true_grid, manifest.robot)
    policy = (LLMGuidedPolicy() if manifest.policy_name == 'llm_guided'
              else BaselinePolicy(PolicyName(manifest.policy_name)))
    return EpisodeRunner(simulator, ScriptedMapper(scenario), TeamNavigator(simulator),
                         policy, None, logger=logger)
