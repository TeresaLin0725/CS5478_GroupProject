# Semantic-guidance experiment

This study compares Nearest, Information Gain, and DeepSeek-guided frontier selection with two system prompts across 31 task/start combinations (124 episodes). Scissors, cup, bottle and banana targets appear in typical or atypical locations in the Rs apartment. It uses clean depth and is separate from the noisy bowl benchmark.

| Policy | Target found | Verified success | Mean capped discovery time (s) |
|---|---:|---:|---:|
| Nearest | 24/31 | 7/31 | 223.7 |
| Information Gain | 28/31 | 13/31 | 147.8 |
| LLM v1 | 29/31 | 14/31 | 126.5 |
| LLM v2 | 28/31 | 13/31 | 131.9 |

Undiscovered targets receive a 600 s penalty. Discovery and independently verified completion are different metrics: many episodes found the correct object but failed close-range verification. The reported LLM-vs-IG discovery-time difference is not statistically significant (paired sign test p = 0.31); these fixed tasks do not establish cross-house generalization.

- [Full analysis and paired comparisons](analysis.md)
- [Episode CSV](episodes.csv)
- [System prompts](prompts.md)
- [Experiment provenance](provenance.json)
- [Discovery-time figure](find-time-by-group.png)

The model was `deepseek-flash` (DeepSeek-V4.1-Flash), using provider-default reasoning. The two prompt versions share the same user-message structure. Archived rationales describe returned model explanations, not private internal reasoning.

For fresh execution use the committed semantic configurations and [reproduction instructions](../../Documentation/reproducing.md). New runs must record their actual model, seeds, configuration and fallback rates.
