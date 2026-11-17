# LLM effectiveness by target location

Source: bowl-48-fixed-20261116-01. All 48 episodes succeeded with zero contact ticks; all integrity checks passed. Each policy has six matched setups per target (three scene conditions × two starts). Two interrupted attempts were retained and restarted; completed outcomes were not retried.

Reduction = 100 × (baseline mean − LLM mean) / baseline mean. Positive values favor LLM. These are ratios of means, not means of per-case percentages. Times below are simulated, not wall-clock.

## Kitchen

| Policy | Mean simulated time (s) | Mean distance (m) |
|---|---:|---:|
| Random | 149.11 | 16.646 |
| Nearest | 128.78 | 9.788 |
| Information Gain | 129.89 | 13.488 |
| LLM-guided | 96.11 | 7.839 |

| Baseline | LLM time reduction | LLM distance reduction | Time wins/ties/losses | Distance wins/ties/losses |
|---|---:|---:|---|---|
| Random | 35.54% | 52.90% | {'wins': 5, 'ties': 0, 'losses': 1} | {'wins': 5, 'ties': 0, 'losses': 1} |
| Nearest | 25.37% | 19.91% | {'wins': 5, 'ties': 0, 'losses': 1} | {'wins': 3, 'ties': 0, 'losses': 3} |
| Information Gain | 26.01% | 41.88% | {'wins': 5, 'ties': 0, 'losses': 1} | {'wins': 5, 'ties': 0, 'losses': 1} |

Paired descriptive 95% bootstrap intervals for LLM minus baseline (10,000 resamples within each scene condition, two starts per stratum):

- Random: time -53.00 s, CI [-79.94, -26.06]; distance -8.806 m, CI [-13.211, -4.402].
- Nearest: time -32.67 s, CI [-49.11, -16.22]; distance -1.949 m, CI [-2.783, -1.115].
- Information Gain: time -33.78 s, CI [-46.61, -20.94]; distance -5.648 m, CI [-7.976, -3.321].

LLM overhead: 17 actual calls, 0 fallbacks, 148,823 tokens, 64.83 s total recorded call latency. Monetary cost was not recorded.

## Bathroom

| Policy | Mean simulated time (s) | Mean distance (m) |
|---|---:|---:|
| Random | 177.28 | 19.344 |
| Nearest | 154.83 | 12.114 |
| Information Gain | 69.89 | 5.371 |
| LLM-guided | 94.33 | 7.869 |

| Baseline | LLM time reduction | LLM distance reduction | Time wins/ties/losses | Distance wins/ties/losses |
|---|---:|---:|---|---|
| Random | 46.79% | 59.32% | {'wins': 5, 'ties': 0, 'losses': 1} | {'wins': 5, 'ties': 0, 'losses': 1} |
| Nearest | 39.07% | 35.04% | {'wins': 4, 'ties': 1, 'losses': 1} | {'wins': 4, 'ties': 1, 'losses': 1} |
| Information Gain | -34.98% | -46.52% | {'wins': 1, 'ties': 2, 'losses': 3} | {'wins': 1, 'ties': 2, 'losses': 3} |

Paired descriptive 95% bootstrap intervals for LLM minus baseline (10,000 resamples within each scene condition, two starts per stratum):

- Random: time -82.94 s, CI [-147.56, -18.33]; distance -11.475 m, CI [-19.046, -3.903].
- Nearest: time -60.50 s, CI [-95.56, -25.44]; distance -4.244 m, CI [-8.446, -0.042].
- Information Gain: time 24.44 s, CI [-9.50, 58.39]; distance 2.499 m, CI [0.524, 4.474].

LLM overhead: 16 actual calls, 0 fallbacks, 142,258 tokens, 89.23 s total recorded call latency. Monetary cost was not recorded.

## Interpretation

LLM is the most efficient method on the kitchen tasks in both mean simulated time and path length. For the bathroom tasks, Information Gain is more efficient than LLM; LLM still improves on Random and Nearest. There is no observed success-rate advantage because every method succeeds in every setup.

Only six fixed setups per target, in one house with one fixed noise realization, are available. Bootstrap intervals describe these matched tasks and do not establish generalization or population-level significance. Shared resources and a restart make wall-clock performance unsuitable for a clean policy-speed comparison. All methods share mapping, navigation, recovery and target-inspection logic; LLM guidance selects exploration actions rather than supplying the entire navigation system.
