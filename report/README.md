# Experiment results and evidence

This index separates the final experiment from earlier development baselines. Reading these reports, viewing figures and downloading demonstration videos requires no simulator installation or provider account.

| Experiment | Scope and result | Evidence |
|---|---|---|
| Final repaired bowl benchmark, 16?17 November | 48/48 verified successes, zero collision/contact ticks; 12 episodes per policy | [Report, protocols and per-episode results](bowl-48-fixed-20261116-01/README.md) |
| Matched LLM rationale capture, 19 November | Separate 12/12-success run; 36 valid answers, two fallback calls, no separate thinking-summary text returned | [All episodes and decision rationales](llm-thinking-matched-20261119-01/README.md) |
| Semantic guidance study, 11 November | 124 episodes across 31 task/start combinations; Nearest, IG, DeepSeek v1/v2 | [Report and analysis](semantic-llm-20261111/README.md) |
| Original bowl benchmark, 10 November | 45/48 successes; retained as the pre-repair comparison, distinct from the final 48/48 suite | [Report and portable evidence](bowl-48-noise-20261110-02/README.md) |
| Clean/noisy cup robustness baseline | 35/36 successes in each arm | [Summary](robustness-35of36/summary.json), [interpretation](../Documentation/english/noise_experiment_results.md) |
| Midterm baseline, 25 October | 29/36 successes; four-policy demonstration videos and English/Chinese PDFs | [Report and videos](../midterm-report-20261025/README.md) |

[Paper figures](paper-figures/) contain exported PNG/PDF visuals, including model comparisons. The committed figures can be inspected directly; some figure/analysis scripts require original local run logs that are not included. See [reproduction scope and commands](../Documentation/reproducing.md).

## Interpreting the results

- In the final bowl suite, every policy succeeds. LLM guidance uses 96.11 simulated seconds and 7.839 m on average for the kitchen bowl, compared with IG's 129.89 s and 13.488 m. For the bathroom bowl, IG uses 69.89 s and 5.371 m, compared with LLM's 94.33 s and 7.869 m. Each mean covers six matched setups.
- The original 45/48 suite and final 48/48 suite are separate experiments. Repairs affected the shared execution stack rather than only the LLM selector. Earlier failures remain in their original archive.
- The semantic study distinguishes first discovery from independently verified completion. Its tasks, model, noise and settings differ from the bowl study; pooling their success rates or timing means would be misleading. The reported LLM-vs-IG discovery-time comparison was not statistically significant (p = 0.31).
- Simulation pauses during model inference. Simulated-time improvements do not imply wall-clock acceleration. Fixed setups in one house do not establish performance across houses.
- Returned decision rationales are model-provided explanations, not observations of private internal reasoning. The rationale-capture rerun is separate from the final benchmark.

Frozen archives retain original manifests, certificates and machine-specific paths for auditability. Do not overwrite them with new results. Detailed test logs that belong to the final archive are retained alongside its artifact hash manifest.
