# Fixed 48-episode bowl benchmark

Completed 17 November 2026: **48/48 successes, zero collision/contact ticks, all result integrity checks passed**. Every policy succeeded in all 12 matched setups. The original benchmark's completed outcomes remain unchanged.

## Design and execution

Two target locations (kitchen/bathroom), three scene conditions (apartment/clutter/dynamic), two starts (A/B), and four policies (Random/Nearest/Information Gain/LLM-guided) yield 48 episodes. Each matched setup runs all four policies before advancing to the next setup, with at most two concurrent simulators.

The first setups were kitchen/clutter/B, bathroom/clutter/B, and bathroom/apartment/A, covering the original failures. All 12 physical feasibility checks passed. The original manifests, seeds, accessible scene layout, 1 cm depth noise and 1% depth dropout were preserved. Actual noise seed is 140.

The batch was interrupted after 26 completed episodes. Two unfinished attempts were archived locally and restarted on resume; no completed result was retried or replaced. Consequently `suite-verification.json` records `no_retries=false`, `no_completed_episode_retries=true`, and two retained interrupted attempts. Their log hashes and the preserved completed-result hashes appear in [resume-audit.json](resume-audit.json).

## Implemented fixes

- Retain all original spatial locality anchors when recovery groups merge, preventing retry counters from escaping enforcement.
- Run tiled detection on every close-inspection capture, restoring normal cadence afterward, including on interruption. Success thresholds and observation routines remain unchanged.
- During recovery only, permit bounded straight egress from unknown cells already intersecting the current footprint. The map stays unchanged; occupied cells, new unknown cells, contact, excessive drift and timeout stop the primitive. In the diagnostic Nearest reruns, about 4.3 cm of movement restored reachability even though the primitive itself reached its time bound before completing the scan.

Final validation: **677 tests, 6 skipped, no failures**, with unchanged source fingerprint during testing. The frozen engine fingerprint is `8a943c855186426351deee5b4930be148e140166d1abcd91d31e6dd64dd105c4`.

## Efficiency by target

Means over six matched setups per target and policy. Times are simulated seconds, not wall-clock latency.

| Policy | Kitchen time (s) | Kitchen path (m) | Bathroom time (s) | Bathroom path (m) |
|---|---:|---:|---:|---:|
| Random | 149.11 | 16.646 | 177.28 | 19.344 |
| Nearest | 128.78 | 9.788 | 154.83 | 12.114 |
| Information Gain | 129.89 | 13.488 | 69.89 | 5.371 |
| LLM-guided | 96.11 | 7.839 | 94.33 | 7.869 |

On kitchen tasks, LLM uses 26.0% less simulated time and 41.9% less travel than Information Gain. On bathroom tasks, it uses 35.0% more time and 46.5% more travel. LLM improves over Random and Nearest for both locations. All methods have the same observed success rate, so the measured advantage is efficiency rather than reliability.

LLM-guided made 33 actual Gemini calls with zero fallbacks. Monetary cost was not recorded. Shared resources, model latency and the interruption mean wall times should not be interpreted as isolated policy-speed measurements. Six fixed setups per location in one house do not establish broader generalization.

See [LLM effectiveness calculations](llm-effectiveness.md) and [machine-readable comparisons](llm-effectiveness.json) for percentage formulas, paired wins/ties/losses, descriptive bootstrap intervals and call overhead.

## Evidence and reproducibility

- [All results](results.json), [CSV table](episodes.csv), [execution order](schedule.json).
- [Frozen protocol](protocol.json), [original protocol SHA-256](protocol.sha256), and all 48 files under `manifests/`.
- [Suite verification](suite-verification.json), [regression certificate](tests.json), [test output](tests.log), and 12 physical-check records under `preflight/`.
- Every episode under `episodes/` has its result, execution summary and independent target-evaluation evidence. The three originally failed cases additionally include scene trajectories and the two accepted RGB evidence frames.
- `harness/` contains the exact frozen execution scripts, including the four-policy setup barrier and read-only inspection diagnostics. The matching source is committed in the repository; copied source snapshots are omitted.
- [Artifact hashes](artifact-sha256.json) preserve the packaged files' SHA-256 values. The original protocol retains machine-specific paths for auditability. Re-execution requires localizing those paths and generating a fresh protocol/seal; these records should not be edited in place.

Model weights, iGibson assets, Python environments, credentials, per-frame depth arrays and large raw event logs are not committed. Full local evidence remains in `runs/bowl-48-fixed-20261116-01/`; interrupted attempts remain under its `interrupted/` directory.
