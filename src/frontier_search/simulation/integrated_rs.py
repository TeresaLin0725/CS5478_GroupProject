"""Native Rs bridge for B's adapter. Rendered truth is retained privately for W3 scoring."""
from math import hypot, pi
import numpy as np
from ..contracts import Point3, Pose, EvaluationResult
from ..evaluation.retained import GroundTruthFrame, VisibleInstance
from ..evaluation.verifier import RetainedTargetEvaluator
from .igibson_adapter import IGibsonAdapter, IGibsonBackend
from .igibson_rs import IGibsonWorld


class RsBackend(IGibsonBackend):
    def __init__(self, config, profile='dynamic'):
        super().__init__(config)
        self.profile = profile

    def open(self, scene_id, seed, start_pose):
        if scene_id != 'Rs':
            raise ValueError('RsBackend requires the installed Rs scan')
        np.random.seed(seed)
        self.world = IGibsonWorld(start=(start_pose.x_m,start_pose.y_m,start_pose.yaw_rad),
            profile=self.profile, target_model='025_mug',target_position=(-1.4,2.95,.9213), image_width=self._config.image_width,
            image_height=self._config.image_height)
        w = self.world
        self._p, self._np, self._sim, self._scene = w.p, np, w.sim, w.scene
        self._robot, self._bodies = w.robot, w.robot_bodies
        self.snapshot_id = 'Rs:%s:seed%d:native-locobot' % (self.profile,seed)
        self.command = (0.,0.)

    def apply_action(self, linear, angular_command):
        self.command = linear, angular_command
        # stop() must apply brakes immediately, without advancing physics.
        self._robot.apply_action(np.array([linear/self.world.max_command[0],
                                          angular_command/self.world.max_command[1]]))

    def step(self):
        self.world.drive(self.command[0], -self.command[1])

    def render(self):
        # Kinematically repositioned mass-zero furniture can sleep in Bullet.
        # Native sync otherwise skips sleeping links, leaving RGB-D at an old
        # pose while collision geometry has already moved.
        self._sim.sync(force_sync=True)
        renderer = self._sim.renderer
        renderer.set_fov(self._config.vertical_fov_deg)
        rgb, xyz, seg = renderer.render_robot_cameras(modes=('rgb','3d','ins_seg'))
        self.cached = tuple(np.array(v,copy=True) for v in (rgb,xyz,seg,renderer.V))
        return rgb, xyz, np.array(renderer.get_intrinsics(),copy=True), np.array(renderer.V,copy=True)

    def truth(self, observation):
        rgb, xyz, seg, view = self.cached
        renderer, w = self._sim.renderer, self.world
        ids = np.rint(seg[:,:,0]*1024).astype(int).clip(0,1023)
        mask = renderer.instance_id_to_pb_id[ids] == w.target_body
        rows, cols = np.nonzero(mask)
        instances = ()
        if len(rows):
            # Independent silhouette denominator, same pose, no simulation step.
            hidden = [inst for inst in renderer.instances if inst not in w.target.renderer_instances]
            isolated = renderer.render(modes=('ins_seg',), hidden=hidden)[0]
            isolated_ids = np.rint(isolated[:,:,0]*1024).astype(int).clip(0,1023)
            total = np.count_nonzero(renderer.instance_id_to_pb_id[isolated_ids] == w.target_body)
            points = xyz[mask,:3]
            homogeneous = np.concatenate((points,np.ones((len(points),1))),axis=1)
            world_points = homogeneous @ np.linalg.inv(view).T
            surface = np.median(world_points[:,:3],axis=0)
            instances = (VisibleInstance('Rs-inserted-mug','cup',min(1.,len(rows)/max(1,total)),
                (int(cols.min()),int(rows.min()),int(cols.max()+1),int(rows.max()+1)),
                Point3(*(float(v) for v in surface))),)
        return GroundTruthFrame(observation.episode_id,observation.sequence,observation.timestamp_s,
            observation.color_image.width,observation.color_image.height,instances)

    def close(self):
        self.world.close()


class RetainingAdapter(IGibsonAdapter):
    """Only RGBDObservation leaves capture(); evaluator metadata stays in the capture owner."""
    def __init__(self, robot, mapping, config, store, profile='dynamic'):
        super().__init__(robot,mapping,config,lambda cfg: RsBackend(cfg,profile))
        self.truth_store, self.capture_motion = store, {}
        self.on_close = None

    def capture(self):
        obs = super().capture()
        self.truth_store.retain(self._backend.truth(obs))
        velocity, angular = self._backend._p.getBaseVelocity(next(iter(self._backend._bodies)))
        self.capture_motion[obs.sequence] = dict(pose=obs.robot_pose,
            linear_mps=hypot(velocity[0],velocity[1]),angular_radps=abs(angular[2]),
            command_stopped=abs(self._v)+abs(self._w)<1e-9)
        return obs

    def close(self):
        if not self._closed and self.on_close:
            self.on_close()
        super().close()


class CloseRetainedEvaluator(RetainedTargetEvaluator):
    """W3 identity/visibility/IoU/localization AND physical near/stopped evidence."""
    def __init__(self, store, adapter, distance=.95):
        super().__init__(store,'cup')
        self.adapter, self.distance, self.metrics = adapter, distance, {}
        self._close_result=None

    def reset(self,episode_id,config):
        super().reset(episode_id,config)
        self._close_result=None
        self.metrics={}

    def evaluate(self, request):
        result = super().evaluate(request)
        if self._close_result is not None:
            return self._close_result
        captures = [self.adapter.capture_motion[item.capture_sequence] for item in request.evidence.detections]
        distances = []
        for item, motion in zip(request.evidence.detections,captures):
            frame = self._source.get_frame(request.episode_id,item.capture_sequence)
            matched = [v for v in frame.instances if v.instance_id == result.matched_instance_id]
            if matched:
                p, q = motion['pose'], matched[0].position_world
                distances.append(hypot(p.x_m-q.x_m,p.y_m-q.y_m))
        distinct_times = len({item.timestamp_s for item in request.evidence.detections}) == len(captures)
        stopped = all(m['linear_mps']<=.04 and m['angular_radps']<=.10 for m in captures)
        near = len(distances)==len(captures) and all(d<=self.distance for d in distances)
        current_stopped = abs(self.adapter._v)+abs(self.adapter._w)<1e-9
        valid = result.valid_target and distinct_times and stopped and near and current_stopped
        self.metrics = dict(w3_valid=result.valid_target,evidence_distances_m=distances,
            distinct_times=distinct_times,stationary_evidence=stopped,near_evidence=near,terminal_stopped=current_stopped)
        self._close_result=EvaluationResult(result.episode_id,result.claim_id,bool(valid),
            result.matched_instance_id if valid else None,
            'retained_instance_and_close_stationary_evidence' if valid else result.reason if not result.valid_target
            else 'close_stationary_acceptance_failed')
        return self._close_result
