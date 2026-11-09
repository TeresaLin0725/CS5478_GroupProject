# Optional mapping depth filter

`MappingConfig` exposes an opt-in range-gated median filter. In an episode
manifest set these fields under `mapping`:

```json
{
  "depth_filter_kernel_size": 5,
  "depth_filter_max_delta_m": 0.08
}
```

Set `depth_filter_kernel_size` to `0` to disable it (default), or use `3`/`5`
for the spatial window. The threshold is a depth difference in metres, not a
noise amplitude. Noise generation remains configured independently through
`sensor_noise`; enabling noise does not automatically enable filtering.

The filter runs only before geometric map fusion. Detection, target evidence,
raw retained evaluator frames, poses and camera calibration are unchanged.
It requires NumPy when enabled. It uses only the received depth image and
mapping depth bounds, never noise seeds, clean depths or simulator geometry.

Only valid measured centres with at least a strict majority of the full window
within the configured range threshold are replaced by their accepted median.
Invalid centres, missing depth, out-of-range measurements and isolated surfaces
without enough support remain unchanged. Borders are not replicated to create
artificial support. Large depth discontinuities are excluded from the median.
This does not guarantee preservation of all low-contrast or sub-pixel obstacles;
native collision and clean-regression checks remain necessary.

## Motivation and validation scope

In the retained apartment nearest-policy run with seed 17, the first decision
at 11.5333 simulated seconds has exactly the same pose with and without depth
noise. With depth standard deviation 0.01 m, 32 cells that were free in the clean
map are occupied in the noisy map (989 versus 1105 occupied cells overall).
This demonstrates a map-evidence change before policy trajectories diverge,
but does not independently establish the physical truth of every cell.

The existing fusion treats a single collision-band endpoint as occupied;
clearing needs consecutive fresh floor and full-column ray evidence. The
filter reduces measurement scatter before those existing rules are applied.
Navigation clearance, obstacle inflation and free-space evidence requirements
are unchanged.

Focused tests cover planar-noise reduction, depth steps, thin high-contrast
obstacles, invalid/sparse input, borders, immutability and fusion integration.
Native validation is kept in a new sealed snapshot under
`runs/depth-filter-validation-20261107-01`; earlier baseline and noise results
remain under their original snapshots. Native outcomes, not the unit tests,
determine whether this candidate fixes the diagnosed task failures.

## Completed combined validation

This component is retained in the shared 35/36 clean and 35/36 mixed-noise
version. See the [same-version results](noise_experiment_results.md) for the
exact tested settings, remaining failures and limits of the evidence. The
combined suites do not isolate this component's contribution.
