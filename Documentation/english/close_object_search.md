# Close-object search extension (contract 1.1)

`Frontier` and `Subgoal` carry `kind` and `target_track_id`. A `target_approach`
must reference an observed target in the same WorldState. All four policies
prioritize reachable target approaches, then apply their frontier selection rule.
The original 1.0 fixtures keep their visual-search completion rule.

The current integrated entry point is `runner/integrated.py`: native Gibson Rs
and Locobot → calibrated RGB-D → RGB-only YOLO11s → shared mapper and tracking
→ shared TeamNavigator → shared EpisodeRunner and recovery → isolated retained
instance evaluation. Instance segmentation and object truth belong to the
terminal evaluator, not the mapper or policy. The earlier standalone viewer
used a different annotated/segmentation detection path.

The production manifest records the actual parameters. At 2026-10-18, the
mapper requires two distinct-time nearby still observations within three
seconds, estimated planar distance at most 0.90 m, confidence at least 0.50,
and a 0.25 m target-extent prior. The independent check additionally requires
matching the visible physical cup instance, distance at most 0.95 m, physical
linear speed at most 0.04 m/s and angular speed at most 0.10 rad/s at each
evidence capture, and a stopped terminal command. It retains the configured
visibility, bbox IoU and localization checks. A tentative cup creates an
approach opportunity; it is not success.

The target approach is assessed on the footprint-inflated known-free depth map;
the preferred planned point leaves about 0.10 m inside the observation threshold.
Frontier approaches are assessed against their actual support cells, since a
boundary wrapping furniture can have a centroid inside an obstacle.

The integrated runner uses the shared navigator and mapper, with the full
configured footprint, collision height, unknown-space and rotation checks.
Recordings require same-source native validation and regressions. Earlier
standalone procedural PyBullet demos were removed; historical iGibson viewer
artifacts remain distinct from the current pipeline.

Historical LLM replay is authored and does not measure model inference. Current
integrated hosted calls preserve actual responses and distinguish all fallbacks.

## Optional control-step odometry

`RGBDObservation` may append `odometry_linear_speed_mps` and
`odometry_angular_speed_radps`. Both are nonnegative finite magnitudes, or both
are absent. Existing constructors and recordings remain valid; save/replay
preserves supplied values and defaults legacy captures to `None`.

The native adapter derives these values from successive known robot poses
at the 30 Hz control rate. `capture()` does not step physics; `stop()` does not
rewrite a previously measured motion value as zero. No target coordinates,
instance identities, evaluator outcomes or scene occupancy enter this channel.

The close-inspection mapper checks these values as well as its camera-interval
pose difference. This prevents fast base oscillation between RGB-D frames from
appearing stationary through averaging. The optimized manifest uses a
conservative 0.02 rad/s odometry/pose filter; the independent 0.10 rad/s physical
threshold is unchanged. All four policies share this filter and their public
WorldState/candidate inputs retain the same structure. Capture logs retain the
odometry values for review. Legacy captures without them use the original
pose-interval check and do not establish high-frequency physical stillness.

## Stopped target inspection

`ObservationConfig.inspection_hold_s` defaults to zero for compatibility. The
optimized integrated manifest sets it to 4 seconds **per heading**. After B
reports REACHED for an observed target approach, C requests the shared optional
`inspection` observation routine. B checks footprint-safe turns through the
fixed `inspection_headings_rad` offsets (-0.15, 0, +0.15 rad), holding the brakes
at each heading. Physical steps and regularly scheduled RGB-D captures continue.
A claim preempts this routine and is scored through the normal independent
evaluator. No claim is synthesized, and all global/operation budgets and
cancellation still apply. All four policies share this behavior. Normal
observation start/feedback/terminal events and designated scan barriers record it.

See the [current design audit](../chinese/demo_design_review.md) for actual code,
native single-run evidence, failed diagnostics and remaining experiment work.

See [the current demo record](../chinese/updated_scene_demo.md).
