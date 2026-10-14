# W3 step 4: independent target verifier

`evaluation.verifier.RetainedTargetEvaluator` implements the frozen
`TargetEvaluator` interface. It is constructed with an evaluation-only capture
source and the requested class from the experiment manifest. It receives no
mapper state beyond the terminal claim and frozen evidence bundle.

```python
from frontier_search.evaluation.retained import RetainedFrames
from frontier_search.evaluation.verifier import RetainedTargetEvaluator

truth_store = RetainedFrames()
# B's capture adapter retains independently generated GroundTruthFrame records
# in truth_store at capture time, keyed by the same episode and sequence as RGB-D.
evaluator = RetainedTargetEvaluator(truth_store, manifest.target_class)
# Supply evaluator=evaluator when constructing EpisodeRunner.
# The runner calls reset(episode_id, evaluation_config), then evaluate(request).
```

The capture owner supplies immutable `GroundTruthFrame` records with episode,
sequence, timestamp, image dimensions and a tuple of `VisibleInstance` records.
Each instance has a persistent simulator instance ID, canonical class, visible
box, visibility fraction and independent world-space visible-surface reference.
Boxes are integer half-open pixel bounds, as in detector evidence. Visibility is
the visible fraction of the instance's projected silhouette, not the fraction
of the whole image. The surface reference must describe observed target geometry,
not its bounding-volume centre or a position copied from detector output.
An empty instance tuple explicitly records no visible objects. Do not represent
missing annotation data as an empty frame.

The store rejects duplicate episode/sequence writes and retains all captures
until its owner discards it after scoring. The evaluator checks requested frame
identity and timestamp against each evidence record. Missing captures, invalid
records, missing visible-target surface references and mismatched metadata raise
errors; the existing runner records `evaluation_error`.

The fixed matching rule requires **every** cited frame to support the **same**
instance of the configured target class. Each match must pass all of:

- Visibility >= `minimum_visibility` (default 0.1).
- Box intersection-over-union >= `minimum_bbox_iou` (default 0.5).
- 3D distance to the independent visible-surface reference <=
  `localization_tolerance_m` (default 0.25 m).

There must be at least `required_evidence_frames` distinct sequences (default 2).
Only class names that exactly match the canonical requested label are eligible.
The result is `valid_target=False` if complete, valid records fail these checks;
the runner ends as `false_target_claim`. It never resumes search using that
feedback. An instance present elsewhere in a frame cannot validate the claim.
Multiple matching IDs use lexical ordering for a deterministic result.

One completed request/result is cached per episode. An identical repeated request
returns the same result; a different request after terminal scoring is rejected.
Reset validates configuration and clears this terminal cache, while capture
records remain owned by the separate store. Results and instance IDs belong only
in terminal logs. Do not inject the store or evaluator into A's mapper or policies.

`EvaluationConfig` now validates thresholds and frame counts. Manifest validation
also rechecks it and retains the requirement that mapper confirmation supply
enough frames. The runner/supervisor owns evaluation deadlines and simulator
stop/cancellation.

Validation uses the production mapper's frozen claim with separately authored
truth records. Tests cover correct/false matches, target absence, instance
switching, metadata failures, missing records, thresholds, lifecycle, and the
runner's stop/cancel/scoring path without calling policy selection.

**Remaining integration:** existing RGB-D recordings do not contain these hidden
capture records, and the live adapter does not yet populate `RetainedFrames`.
B must generate and retain calibrated instance annotations at capture time;
C must pass the evaluator only to the runner and enable evaluation in its manifest.
The existing unscored chair diagnostic cannot be retroactively called a verified
success. A real independently scored search remains a joint integration gate.
