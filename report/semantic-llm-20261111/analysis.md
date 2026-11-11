# Semantic navigation replicate analysis

Episodes: 124 across starts S0, S1, S2, S3. Failures are scored at the 600 s search cap in capped-time metrics.

## Overall

| Policy | Episodes | Success | False claims | Mean capped time s | Median capped s | Mean time on success s | Mean distance on success m |
|---|---:|---:|---:|---:|---:|---:|---:|
| nearest | 31 | 7/31 (23%) | 3 | 503.7 | 600.0 | 173.5 | 11.68 |
| information_gain | 31 | 13/31 (42%) | 2 | 415.0 | 600.0 | 158.8 | 13.73 |
| llm_guided | 31 | 14/31 (45%) | 4 | 394.2 | 600.0 | 144.3 | 11.73 |
| llm_guided_v2 | 31 | 13/31 (42%) | 2 | 401.7 | 600.0 | 127.1 | 9.82 |

## Search efficiency: time until the true target is first tracked

Target-class track within 0.5 m of the true position; never found = 600 s.

| Policy | Found | Mean find time s (capped) | Median s | Found but task not completed |
|---|---:|---:|---:|---:|
| nearest | 24/31 | 223.7 | 118.3 | 17 |
| information_gain | 28/31 | 147.8 | 112.7 | 15 |
| llm_guided | 29/31 | 126.5 | 86.0 | 15 |
| llm_guided_v2 | 28/31 | 131.9 | 76.3 | 15 |

| Group | Policy | Mean find time s (capped) |
|---|---|---:|
| consistent | nearest | 207.8 |
| consistent | information_gain | 143.0 |
| consistent | llm_guided | 106.1 |
| consistent | llm_guided_v2 | 117.6 |
| anomalous | nearest | 240.6 |
| anomalous | information_gain | 152.9 |
| anomalous | llm_guided | 148.2 |
| anomalous | llm_guided_v2 | 147.2 |

## By semantic relation of placement

| Group | Policy | Success | Mean capped time s |
|---|---|---:|---:|
| consistent | nearest | 7/16 | 413.4 |
| consistent | information_gain | 10/16 | 321.6 |
| consistent | llm_guided | 11/16 | 290.0 |
| consistent | llm_guided_v2 | 10/16 | 304.1 |
| anomalous | nearest | 0/15 | 600.0 |
| anomalous | information_gain | 3/15 | 514.6 |
| anomalous | llm_guided | 3/15 | 505.3 |
| anomalous | llm_guided_v2 | 3/15 | 505.7 |

## Per task (success count / capped mean s over starts)

| Task | nearest | information_gain | llm_guided | llm_guided_v2 |
|---|---:|---:|---:|---:|
| scissors-desk-aligned | 0/4 / 600 | 1/4 / 495 | 1/4 / 482 | 0/4 / 600 |
| cup-kitchen-aligned | 3/4 / 309 | 3/4 / 278 | 4/4 / 134 | 3/4 / 243 |
| banana-kitchen-aligned | 1/4 / 498 | 4/4 / 161 | 4/4 / 160 | 4/4 / 126 |
| bottle-kitchen-ambiguous | 3/4 / 246 | 2/4 / 351 | 2/4 / 383 | 3/4 / 248 |
| scissors-kitchen-misaligned | 0/4 / 600 | 3/4 / 280 | 3/4 / 245 | 3/4 / 246 |
| cup-bathroom-semantic-decoy | 0/4 / 600 | 0/4 / 600 | 0/4 / 600 | 0/4 / 600 |
| banana-bathroom-misaligned | 0/4 / 600 | 0/4 / 600 | 0/4 / 600 | 0/4 / 600 |
| bottle-bathroom-ambiguous | 0/3 / 600 | 0/3 / 600 | 0/3 / 600 | 0/3 / 600 |

## Paired comparisons on task completion (same start and task; capped time, 1 s tie tolerance)

| A vs B | Pairs | A faster | Tie | B faster | Mean A-B s | Sign-test p |
|---|---:|---:|---:|---:|---:|---:|
| llm_guided vs information_gain | 31 | 10 | 18 | 3 | -20.8 | 0.092 |
| llm_guided_v2 vs information_gain | 31 | 9 | 20 | 2 | -13.3 | 0.065 |
| llm_guided_v2 vs llm_guided | 31 | 4 | 23 | 4 | +7.5 | 1.000 |
| information_gain vs nearest | 31 | 13 | 15 | 3 | -88.7 | 0.021 |
| llm_guided vs nearest | 31 | 13 | 16 | 2 | -109.5 | 0.007 |
| llm_guided_v2 vs nearest | 31 | 12 | 16 | 3 | -102.0 | 0.035 |

## Paired comparisons on time-to-find the true target

| A vs B | Pairs | A faster | Tie | B faster | Mean A-B s | Sign-test p |
|---|---:|---:|---:|---:|---:|---:|
| llm_guided vs information_gain | 31 | 15 | 7 | 9 | -21.3 | 0.307 |
| llm_guided_v2 vs information_gain | 31 | 15 | 9 | 7 | -15.9 | 0.134 |
| llm_guided_v2 vs llm_guided | 31 | 10 | 15 | 6 | +5.4 | 0.454 |
| information_gain vs nearest | 31 | 22 | 4 | 5 | -75.9 | 0.002 |
| llm_guided vs nearest | 31 | 26 | 1 | 4 | -97.2 | 0.000 |
| llm_guided_v2 vs nearest | 31 | 25 | 2 | 4 | -91.8 | 0.000 |

## LLM decision behaviour

| Policy | Model decisions | = max gain/path | Fallbacks | Mean tokens / episode | Mean wall s / episode |
|---|---:|---:|---:|---:|---:|
| llm_guided | 278 | 123 (44%) | 1 | 70170 | 557 |
| llm_guided_v2 | 231 | 119 (52%) | 1 | 78114 | 613 |

## v2 belief about the true target region

Probability v2 assigned to the region that actually held the target (uniform over 4 regions = 0.25).

| Group | Episodes | First-decision prob (mean) | All-decision prob (mean) |
|---|---:|---:|---:|
| consistent | 16 | 0.28 | 0.35 |
| anomalous | 15 | 0.32 | 0.39 |

## First selected region equals target region

- nearest: 3/31; first regions: R1 x23, R2 x8
- information_gain: 2/31; first regions: R2 x16, R1 x15
- llm_guided: 4/31; first regions: R2 x16, R1 x10, R3 x5
- llm_guided_v2: 3/31; first regions: R2 x14, R1 x14, R4 x2, R3 x1

## Early navigation traps (no_feasible_frontiers before 100 s sim)

- nearest: 0/31
- information_gain: 0/31
- llm_guided: 0/31
- llm_guided_v2: 2/31 (scissors-desk-aligned@S0, bottle-bathroom-ambiguous@S0)

## Terminations

- nearest: search_limit 16, success 7, no_feasible_frontiers 5, false_target_claim 3
- information_gain: success 13, search_limit 8, no_feasible_frontiers 8, false_target_claim 2
- llm_guided: success 14, no_feasible_frontiers 7, search_limit 6, false_target_claim 4
- llm_guided_v2: success 13, no_feasible_frontiers 10, search_limit 5, false_target_claim 2, pipeline_error 1
