# Same-version clean and mixed-noise experiments

Status: completed and audited on 9 November 2026 (Singapore).

The combined implementation completed 35 of 36 cases both without noise and
with mixed depth noise, with zero recorded contacts in either suite. The two
failures occurred in different cases. These results demonstrate performance
under the tested conditions; they do not establish general robustness across
houses, noise strengths, or random noise realizations.

## Experimental design

Both suites used the same sealed implementation:

`4a28a90761acdfe8e79a12a3c5dd9cb2dc9b1daefbaac0cbc00c77503dacd3e4`

Each suite contains 36 completed records: three conditions in the iGibson Rs
house (`apartment`, `clutter`, `dynamic`), three task seed/initial-heading
blocks (17, 29, 43), and four policies (Random, Nearest, Information Gain,
LLM-guided). These are conditions within one house, rather than three
independent houses. Task geometry and target placement are fixed; the blocks
vary the initial heading and dedicated policy/perception random streams.

Every paired manifest was checked: the only configuration difference is
`sensor_noise.enabled`. The clean arm disables injection while retaining the
same inactive noise parameters. The noisy arm uses:

| Setting | Value |
|---|---:|
| Gaussian axial depth standard deviation | 0.01 m |
| Valid-depth pixel dropout probability | 0.01 |
| Base noise seed | 123 |
| Actual seed for task blocks 17 / 29 / 43 | 140 / 152 / 166 |
| Per-episode simulated-time budget | 600 s |
| Concurrent workers | At most 2 |

Noise is applied after simulator capture and before perception. RGB, poses,
and retained simulator truth are not corrupted. Gaussian and dropout streams
are independent and keyed by seed and capture sequence. Identical local seeds
across policies do not imply identical physical observations once trajectories
diverge.

Both arms retain the same robustness configuration:

| Component | Tested setting |
|---|---|
| Mapping depth filter | 5×5 range-gated median; 0.08 m maximum neighbor difference |
| Recovery viewpoints | Unvisited capture-pose heuristic; 0.6 m spacing; 3 m range; at most one coarse candidate per eight angular sectors |
| Inspection settling | Entry ratio 0.5: begin dwell within 0.025 rad, retain the original 0.05 rad outer tolerance |

The mapping filter is used by occupancy fusion, rather than RGB detection or
the independent success verifier. The combined implementation passed 650
regression tests before freezing. This experiment compares clean and noisy
measurements with those components enabled in both arms; it is not an
ablation of the individual components.

Five exact-match completed results were imported into the noisy suite and six
into the clean suite, including the clean failure. Reuse required matching
source and manifest content, complete evidence, and passing integrity checks.
A fixed source priority was used independently of outcome. Original files,
reuse receipts, and interrupted-attempt logs were preserved. Completed failures
were not rerun to replace them with successes.

## Results

Success requires independent verification of the submitted target evidence,
including distinct fresh captures, stationary/near evidence, and a stopped
terminal state. All 72 completed records passed the suite's integrity checks.

| Policy | Clean success | Mixed-noise success | Clean mean penalized time (s) | Noisy mean penalized time (s) |
|---|---:|---:|---:|---:|
| Random | 8/9 | 9/9 | 267.74 | 204.15 |
| Nearest | 9/9 | 8/9 | 186.37 | 245.56 |
| Information Gain | 9/9 | 9/9 | 177.49 | 183.11 |
| LLM-guided system | 9/9 | 9/9 | 148.81 | 127.81 |
| All cases | 35/36 | 35/36 | 195.10 | 190.16 |

Penalized time uses the episode's simulated completion time on success and
600 seconds on failure. An early failed termination is not counted as a faster
completion. Wall times are not a controlled comparison because imported and
new episodes ran under shared resource conditions.

Of the 36 pairs, 34 succeeded in both arms, one failed only without noise,
and one failed only with noise. No pair failed in both arms:

| Case | Clean outcome | Mixed-noise outcome |
|---|---|---|
| `dynamic-s17-random` | `no_feasible_frontiers`, 111.00 s | Success, 203.00 s |
| `dynamic-s43-nearest` | Success, 190.33 s | `no_feasible_frontiers`, 238.83 s |

The equal aggregate success counts therefore conceal case-specific changes.
The small descriptive difference in aggregate penalized time is not evidence
that adding noise generally improves performance.

### Hosted-model qualification

Both arms configured Google's official `generativelanguage.googleapis.com`
endpoint with `gemini-3.5-flash-lite`. The clean arm made 20 actual model calls;
one call in `dynamic-s29-llm_guided` timed out at approximately 30 seconds and
used the configured fallback. That task still succeeded. The noisy arm made
22 calls with no fallbacks.

The clean result is therefore an LLM-guided **system with fallback**, not an
episode controlled entirely by model responses. The original outcome remains
in the primary results. Hosted responses and network behavior are additional
sources of variation; equal local seeds do not make these decisions repeatable.

## Interpretation and retained version

The present evidence supports the statement: *in this fixed 36-case diagnostic
suite, the combined implementation retained 35/36 task success and zero
contacts under 1 cm axial depth noise plus 1% dropout; two individual outcomes
changed relative to clean sensing.*

It does not establish performance in unseen houses, across multiple independent
noise realizations per case, at other noise strengths, or the contribution of
each robustness component. Cases used while developing fixes are included.

The working runtime has been restored byte-for-byte to the shared sealed
source above. Later experimental backoff, fine recovery sampling, swept-diagonal
planning/tracking, and recovery-identity changes are excluded from this version.
Their evidence and prior working files remain in local ignored `runs/` folders.
The original 72 completed outcomes, including both failures, remain unchanged.

## Committed settings and results

- [Portable experiment specification](../../configs/robustness-35of36.json)
- [Restoration and regression checks](../../report/robustness-35of36/restoration-validation.json)
- [All 72 per-case outcomes](../../report/robustness-35of36/episodes.csv)
- [Policy statistics and verification receipts](../../report/robustness-35of36/summary.json)

The specification records the shared base manifest, 36 job overrides, two arm
switches, runtime settings, source fingerprint and model-weight hash. It is an
archival experiment specification, not input to `--noise-config`. To reconstruct
an episode manifest, override `episode_id`, `policy_name`, `start_pose`, and the
three task seeds from its job, select the arm's `sensor_noise.enabled`, and set
`sensor_noise.seed` to `(123 + environment_seed) % 2**32`. Scene condition is the
job's `profile`; use the recorded `shared_round` navigation preset/options.
All other manifest fields remain those of `base_manifest`.

The ordinary CLI defaults leave noise, filtering, unvisited recovery viewpoints
and tighter inspection settling disabled. Turning on noise alone does not
reproduce the full tested configuration. The committed specification preserves
those independently configured options. Hosted LLM responses are not replayable
from the local seeds.

## Local raw evidence

Full snapshots, event streams, independent evaluation evidence, original
protocols and reuse receipts remain local under ignored `runs/` directories;
they are not bundled in this commit. The JSON summary retains their protocol
hashes and original verification receipts. Local entry points:

- [Clean suite and paired report](../../runs/clean-full36-20261108-01/live.html)
- [Noisy suite report](../../runs/noise-full36-20261107-01/live.html)
- [Evidence hashes and frozen-version audit](../../runs/paired-final-review-20261109-01/evidence-audit.json)
- [Rollback archive and receipt](../../runs/revert-to-35of36-20261109-01/rollback-receipt.json)
