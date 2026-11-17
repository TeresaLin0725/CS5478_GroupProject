"""Read-only inspection instrumentation for this isolated diagnostic run."""
from dataclasses import asdict
from math import hypot, pi
import json
from pathlib import Path
import numpy as np
from PIL import Image
from frontier_search.perception.mapper import RGBDMapper


class DiagnosticMapper(RGBDMapper):
    routine = None

    def set_observation_routine(self, routine):
        self.routine = routine
        super().set_observation_routine(routine)

    def update(self, obs):
        previous = self._state
        result = super().update(obs)
        if self.routine != 'inspection':
            return result
        cfg = self._state.request.mapping_config
        dt = obs.timestamp_s-(previous.last_timestamp_s or 0.)
        old = previous.latest_state.robot_pose if previous.latest_state else None
        v = w = None
        if old is not None and dt > 0:
            v = hypot(obs.robot_pose.x_m-old.x_m, obs.robot_pose.y_m-old.y_m)/dt
            w = abs((obs.robot_pose.yaw_rad-old.yaw_rad+pi) % (2*pi)-pi)/dt
        stopped = (v is not None and v <= cfg.inspection_linear_speed_mps
                   and w <= cfg.inspection_angular_speed_radps)
        if obs.odometry_linear_speed_mps is not None:
            stopped = (stopped and obs.odometry_linear_speed_mps <= cfg.inspection_linear_speed_mps
                       and obs.odometry_angular_speed_radps <= cfg.inspection_angular_speed_radps)
        target = result.target_class
        raw = [d for d in self._detector.last_detections if d.class_name == target]
        fresh = [d for d in self._state.evidence.values()
                 if d.capture_sequence == obs.sequence and d.class_name == target]
        threshold = max(cfg.confidence_threshold, cfg.claim_confidence_threshold)
        window = [d for key,d in self._state.evidence.items()
                  if key in self._state.inspection_ids and d.confidence >= threshold]
        reasons = []
        if not raw: reasons.append('no_target_detection_above_detector_threshold')
        if raw and not fresh: reasons.append('no_accepted_localized_target')
        if fresh and not any(d.confidence >= threshold for d in fresh): reasons.append('claim_confidence')
        if not stopped: reasons.append('motion_or_duplicate_timestamp')
        distances = {d.observation_id: None if d.position_world is None else
                     hypot(d.position_world.x_m-obs.robot_pose.x_m,
                           d.position_world.y_m-obs.robot_pose.y_m) for d in fresh}
        if fresh and not any(d is not None and d <= cfg.inspection_distance_m for d in distances.values()):
            reasons.append('distance_or_missing_depth')
        if len({d.capture_sequence for d in window}) < cfg.confirmation_frames:
            reasons.append('insufficient_recent_eligible_frames')
        payload = dict(sequence=obs.sequence, timestamp_s=obs.timestamp_s,
            pose=asdict(obs.robot_pose), pose_linear_mps=v, pose_angular_radps=w,
            odometry_linear_mps=obs.odometry_linear_speed_mps,
            odometry_angular_radps=obs.odometry_angular_speed_radps,
            stopped=stopped, tile_every_capture=True,
            raw_target_detections=[asdict(d) for d in raw],
            fresh_target_evidence=[asdict(d) for d in fresh], distances_m=distances,
            recent_eligible_evidence=[asdict(d) for d in window],
            claim=None if result.target_claim is None else asdict(result.target_claim),
            rejection_conditions=reasons)
        folder=Path(self.diagnostic_folder)
        folder.mkdir(exist_ok=True)
        stem='frame-%06d'%obs.sequence
        Image.fromarray(np.frombuffer(obs.color_image.data,dtype=np.uint8).reshape(
            obs.color_image.height,obs.color_image.width,3)).save(folder/(stem+'.png'))
        np.save(folder/(stem+'-depth.npy'),np.frombuffer(obs.depth_image.data,dtype='<f4').reshape(
            obs.depth_image.height,obs.depth_image.width))
        with (folder/'confirmation.jsonl').open('a',encoding='utf-8') as file:
            file.write(json.dumps(payload)+'\n')
        self.diagnostic_logger.write('inspection_confirmation',obs.episode_id,**payload)
        return result
