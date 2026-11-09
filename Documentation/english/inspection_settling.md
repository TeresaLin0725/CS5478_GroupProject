# Inspection heading settling

Set `observation.inspection_heading_settle_ratio` in an episode manifest to
`0.5` to enter target inspection holds at half the normal heading tolerance.
The default `1.0` preserves the previous behavior (disabled). Values must be
finite numbers greater than zero and at most one.

With a 0.05 rad heading tolerance and ratio 0.5, the robot turns until its
error is at most 0.025 rad before starting the hold. While holding, errors up
to the original 0.05 rad are accepted. Exceeding that outer bound cancels the
hold and requires tight reacquisition. This leaves space for physical drift
instead of repeatedly starting a long hold at the tolerance boundary.

Only the inspection routine changes. Contact stops, map/sweep collision
checks, stall detection, operation deadlines, and target evidence requirements
remain in force. This does not declare a target found or change the evaluator.

The motivating mixed-noise dynamic-s43-information_gain trace had inspection
operations op-30 and op-33 fail at 319.7 and 340.0667 simulated seconds. Replaying
the recorded poses through the original controller reproduced both stall times:
heading error repeatedly crossed 0.05 rad and reset the four-second hold.
This establishes a controller failure mode, not that sensor noise directly
caused the oscillation. Native validation is needed to establish whether this
change resolves the complete episode.

## Completed combined validation

This component is retained in the shared 35/36 clean and 35/36 mixed-noise
version. See the [same-version results](noise_experiment_results.md) for the
exact tested settings, remaining failures and limits of the evidence. The
combined suites do not isolate this component's contribution.
