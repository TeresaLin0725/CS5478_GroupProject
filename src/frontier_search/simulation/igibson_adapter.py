"""Owner B: STUB for the real iGibson adapter. Not implemented; imports nothing.

iGibson is deliberately not imported or installed here. Installing it is a large,
separate step that needs the team's pinned version, assets and your approval.

Checklist from Documentation/implementation_checklist.md (B) and
development_fixtures.md section 3, to do once iGibson is available:

* Pin the iGibson and asset versions; record them in the experiment manifest.
* Capture RGB-D with capture-time camera-to-world transforms; fix the RGB
  encoding, depth units and invalid-depth value.
* Verify the spawn footprint clearance and supply only that mask to A.
* Runner-owned stepping with stop/pause/resume and cleanup in ``close``.
* Expose a velocity command and simulation clock (see ControlInterface in
  navigation/navigator.py).
* Do not expose scene floor maps, object instances or semantic room labels to
  robot modules; evaluator-only records go through a separate private capability.
* scene.floor_map / get_random_point / world_to_map / get_shortest_path are
  development helpers only: adapt axis order, origin, resolution and the
  cell-centre convention, and do not treat shortest paths as collision-free proof.
"""

from typing import Tuple

from ..contracts import GridCell, Pose, RGBDObservation


class IGibsonAdapter:
    def reset(self, episode_id: str, scene_id: str, start_pose: Pose, seed: int) -> None:
        raise NotImplementedError("iGibson integration is pending")

    def verify_spawn_clearance(self) -> Tuple[GridCell, ...]:
        raise NotImplementedError("iGibson integration is pending")

    def capture(self) -> RGBDObservation:
        raise NotImplementedError("iGibson integration is pending")

    def current_pose(self) -> Pose:
        raise NotImplementedError("iGibson integration is pending")

    def step(self) -> None:
        raise NotImplementedError("iGibson integration is pending")

    def stop(self) -> None:
        raise NotImplementedError("iGibson integration is pending")

    def pause(self) -> None:
        raise NotImplementedError("iGibson integration is pending")

    def resume(self) -> None:
        raise NotImplementedError("iGibson integration is pending")

    def close(self) -> None:
        raise NotImplementedError("iGibson integration is pending")
