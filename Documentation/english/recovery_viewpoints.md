# Unvisited recovery viewpoints

Optional mapping settings in an episode manifest:

```json
{
  "unvisited_recovery_viewpoints": true,
  "recovery_viewpoint_spacing_m": 0.6,
  "recovery_viewpoint_range_m": 3.0
}
```

The switch defaults to `false`, retaining the original local probe ring. When
enabled, alternate viewpoints are sampled on a coarse grid in known-free space,
up to the configured range from the current robot pose. Sample spacing and the
exclusion radius around past capture positions use the spacing setting. At
most eight candidates are published (the nearest admissible view per sector).

This expands recovery beyond the original 0.75–1.05 m local ring and avoids
returning to previously captured neighborhoods. The history records received
robot poses, not ground-truth geometry or target locations. It resets every
episode, and rejected mapper updates do not consume view history. It is a
coverage heuristic: passing through an area is not proof of exhaustive semantic
inspection, and coarse sampling can miss useful viewpoints in narrow spaces.

The existing runner defers these `navigation_probe` candidates whenever an
ordinary frontier or target approach is eligible. Footprint checks, unknown and
occupied cell handling, B's reachability checks, recovery caps and search budgets
remain in force. Enable `mapping.dynamic_clear_frames` as in the existing native
benchmark; that mode publishes recovery probes. Depth filtering and simulated
noise remain independently configurable.

Motivation: with the mapping depth filter enabled, the retained clutter seed-17
nearest run exhausted useful ordinary frontiers and dispatched many local
probes, logging 18 no-progress attempts before the 600 s cap. This candidate
recovery strategy requires native validation and does not by itself establish
that the clutter failure is fixed.

## Completed combined validation

This component is retained in the shared 35/36 clean and 35/36 mixed-noise
version. See the [same-version results](noise_experiment_results.md) for the
exact tested settings, remaining failures and limits of the evidence. The
combined suites do not isolate this component's contribution.
