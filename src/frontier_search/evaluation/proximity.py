"""Independent success: rendered cup evidence AND actual close, facing, stopped pose."""
from math import atan2,hypot,pi
from ..contracts import EvaluationResult


class ProximityEvaluator:
    def __init__(self,world,max_distance_m=.80):
        self._world=world
        self.max_distance_m=max_distance_m

    def evaluate(self,request):
        w=self._world; pose=w.current_pose(); target=w.target_position
        distance=hypot(pose.x_m-target[0],pose.y_m-target[1])
        heading=atan2(target[1]-pose.y_m,target[0]-pose.x_m)
        yaw_error=abs((heading-pose.yaw_rad+pi)%(2*pi)-pi)
        evidence=request.evidence.detections
        visible=len({e.capture_sequence for e in evidence})>=2 and all(
            w.visible_evidence.get(e.capture_sequence,0)>=20 for e in evidence)
        captures=[getattr(w,'evaluation_frames',{}).get(e.capture_sequence) for e in evidence]
        close_frames=all(c is not None and c['stopped'] and hypot(c['pose'].x_m-target[0],
            c['pose'].y_m-target[1])<=self.max_distance_m for c in captures)
        different_times=len({c['time_s'] for c in captures if c is not None})>=2
        localized=all(e.position_world is not None and hypot(e.position_world.x_m-target[0],
            e.position_world.y_m-target[1])<.2 for e in evidence)
        valid=visible and localized and close_frames and different_times and distance<=self.max_distance_m and yaw_error<=.20 and abs(w._v)+abs(w._w)<1e-8
        reason='close, facing, stopped and visible on distinct frames' if valid else 'close-target acceptance failed'
        self.metrics={'actual_target_distance_m':distance,'actual_target_yaw_error_rad':yaw_error,
                      'distinct_evidence_frames':len({e.capture_sequence for e in evidence}),
                      'visible_on_evidence':visible,'localized_evidence':localized,'robot_stopped':abs(w._v)+abs(w._w)<1e-8}
        self.metrics.update(close_on_each_evidence_frame=close_frames,distinct_capture_times=different_times)
        return EvaluationResult(request.episode_id,request.claim.claim_id,bool(valid),
            'evaluation-only-cup' if valid else None,reason)
