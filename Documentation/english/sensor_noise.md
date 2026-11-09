# Configurable depth noise

Noise is **off by default**. This is a robustness testing tool; injecting noise
does not train the policies or establish improved robustness. The initial model
adds independent Gaussian depth error and independent missing pixels. RGB,
localization, motion, and patch-shaped dropout are not perturbed in this version.
The default enabled levels (1 cm standard deviation and 1% dropout) are proposed
stress levels, not measurements from a physical camera.

## Direct runs

Run from the repository root in the normal iGibson environment. Choose a fresh
output directory for each run.

```powershell
# Clean control (also the default when no noise flags are supplied)
python scripts/run_integrated.py --policy nearest --no-noise --output runs/noise-clean-01

# Enable the default 1 cm / 1% settings
python scripts/run_integrated.py --policy nearest --noise --output runs/noise-mild-01

# Configure the standard deviation, dropout probability, and independent seed
python scripts/run_integrated.py --policy nearest --noise --depth-noise-std-m 0.03 --depth-dropout-prob 0.05 --noise-seed 101 --output runs/noise-strong-01

# Reuse JSON settings; CLI values take precedence
python scripts/run_integrated.py --noise-config configs/sensor-noise.json --noise --output runs/noise-configured-01

# Disable even if the config's enabled field is true
python scripts/run_integrated.py --noise-config configs/sensor-noise.json --no-noise --output runs/noise-disabled-01
```

Setting a magnitude or seed alone does **not** enable noise. Use `--noise`, or
set `enabled` to `true` in the JSON. `--noise` and `--no-noise` are mutually exclusive.

| JSON field | CLI flag | Default | Allowed values |
|---|---|---|---|
| `enabled` | `--noise` / `--no-noise` | `false` | Boolean |
| `depth_std_m` | `--depth-noise-std-m` | `0.01` | Finite, nonnegative metres |
| `depth_dropout_prob` | `--depth-dropout-prob` | `0.01` | Probability from 0 to 1 |
| `seed` | `--noise-seed` | `0` | Integer from 0 to 4294967295 |

Unknown JSON fields, booleans in numeric fields, NaN, infinity, and out-of-range
values are rejected. Set `depth_std_m` to zero for dropout only, or
`depth_dropout_prob` to zero for Gaussian noise only. Setting both to zero is an
identity transform even when enabled.

## Frozen benchmarks and the first pilot

The existing native environment, assets, weights, and matching regression
certificate are still required by the benchmark freezer. Freeze the settings in
a new suite directory, then run that suite without noise overrides:

```powershell
python scripts/run_paired_benchmark.py freeze --output runs/noise-both-01 --noise --noise-seed 1000
python scripts/run_paired_benchmark.py run --output runs/noise-both-01 --only-baselines
```

`all` accepts the same flags as `freeze`. The `run` action rejects noise flags
because its sealed manifests already define the settings. Explicitly configured
noise suites do not overwrite the repository's canonical experiment config.

For the first pilot, freeze four fresh suites with these options:

| Condition | Options at freeze time |
|---|---|
| Clean | `--no-noise --noise-seed 1000` |
| Depth only | `--noise --depth-dropout-prob 0 --noise-seed 1000` |
| Dropout only | `--noise --depth-noise-std-m 0 --noise-seed 1000` |
| Both | `--noise --noise-seed 1000` |

Run the same selected jobs in each suite to cover the three former failures and
successful controls. Replace `<suite>` with each new suite directory:

```powershell
python scripts/run_paired_benchmark.py run --output <suite> --job-id dynamic-s17-random --job-id apartment-s43-random --job-id apartment-s43-information_gain --job-id dynamic-s17-nearest --job-id apartment-s17-random --job-id apartment-s17-information_gain
```

This is a six-job diagnostic pilot, not a complete four-policy comparison. Its
scope can complete while the 36-job suite remains incomplete; inspect the
per-episode results. Extend to all baselines and multiple independent noise seeds
after diagnosing failures. Compare success, collisions, false claims, time with
failures penalized, and distance. Tune on development seeds, then freeze the
changes and evaluate on held-out seeds. Do not combine different noise conditions
into one policy-comparison table.

## Injection, reproducibility, and evidence

`EpisodeRunner` applies noise after `simulator.capture()` returns and before
`mapper.update()`. The native `RetainingAdapter` has already retained clean
target truth and physical motion. Noise affects depth fusion and depth-based
object localization, while independent success and collision evaluation keep
their physical evidence. No map cells or safety thresholds are directly changed.

Only finite input depths inside the mapping depth range are eligible. Existing
invalid or out-of-range values are preserved. New noisy readings outside that
range become zero (invalid), as do dropped pixels; they are not clipped to a
fake surface. RGB, pose, camera transform, sequence, timestamps, and odometry stay
unchanged. Disabled noise returns the original observation without copying its
buffers or consuming random numbers.

Independent NumPy PCG64 streams use `(noise seed, capture sequence, component)`.
Changing the Gaussian level does not change the dropout mask. Episode name,
policy RNG, global Python RNG, and global NumPy RNG do not influence noise.
Settings and capture sequence reproduce the same perturbation field in the same
numerical runtime; different policy trajectories still observe different
physical pixels. This does not promise identical noisy trajectories.

Direct runs use the configured seed exactly. Benchmarks derive each job's seed as
`(base noise seed + environment seed) modulo 2**32`, keeping it equal across
policies within each task and distinct across the three heading blocks.

Settings are stored in the manifest, start event, terminal summary, runtime
configuration, frozen protocol, and result rows. Summary counters cover processed
captures, eligible pixels, dropped pixels, readings invalidated by range, and
modified pixels. The benchmark report labels noise ON/OFF. Recording validation
rejects changed settings or seeds; certificates without noise settings must be
regenerated.

Synthetic checks establish injection correctness and repeatability. Native
robustness outcomes require actually running the simulator experiments above.
# Mapping-side noise handling

An independent, opt-in depth filter is documented in [depth_filter.md](depth_filter.md).
It is configured under the episode manifest's `mapping` object; noise injection
and filtering have separate switches.
