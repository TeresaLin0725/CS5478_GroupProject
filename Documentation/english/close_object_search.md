# Close-object search extension (contract 1.1)

`Frontier` and `Subgoal` carry `kind` and `target_track_id`. A `target_approach`
must reference an observed target in the same WorldState. All four policies
prioritize reachable target approaches, then apply their frontier selection rule.
The original 1.0 fixtures keep their visual-search completion rule.

The current demo runs in the textured Gibson Rs scan with iGibson's physical
Locobot, moved by native differential-drive wheel commands. An optional close
inspection gate in the demo RGB-D mapper requires two rendered instance
segmentation observations at different simulator times, stationary motion and
an estimated planar target distance <= 0.85 m. The isolated evaluator checks
both evidence poses and the current pose against the true target position
(distance <= 0.93 m), stopped evidence and localization error <= 0.5 m.

The target approach is assessed on the footprint-inflated known-free depth map;
the planned point leaves 0.20 m of tolerance inside the observation threshold.
Frontier approaches are assessed against their actual support cells, since a
boundary wrapping furniture can have a centroid inside an obstacle.

The standalone procedural PyBullet demos were removed. The shared rectangular
SE(2) lattice, swept-rotation guards, recovery and contract regression tests are
retained independently. The iGibson demo uses its own round-base planner and
depth-fusion loop; it does not claim full W2 production integration.

The LLM replay is authored from the same scenario's Information Gain choices.
It checks the interface without measuring model inference. Live model calls
require explicitly selecting the live backend.

See [Chinese reproduction and acceptance notes](../chinese/igibson_demo.md).
