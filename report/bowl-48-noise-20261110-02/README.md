# Original noisy bowl benchmark

This pre-repair experiment completed 48 episodes with 45 verified successes and zero contacts. Information Gain and LLM-guided each succeeded in all 12 of their episodes. The separate [final bowl benchmark](../bowl-48-fixed-20261116-01/README.md) achieved 48/48.

- [Results](results.json), [episode table](episodes.csv) and per-episode summaries under `episodes/`.
- [Protocol](protocol.json), [protocol hash](protocol.sha256) and [configuration audit](configuration-audit.json).
- Per-episode `trajectory-scene.webp` files provide compact previews of recorded routes.

Two bowl locations, two starts, three scene conditions and four policies yield 48 episodes. Depth noise is 0.01 m standard deviation with 1% dropout, using actual noise seed 140. Archived machine paths describe the original run and are not portable runtime configuration.

English event descriptions replace localized explanatory labels. Metrics and model replies are unchanged. The [archive manifest](archive-manifest.json) lists current artifact hashes; [localization provenance](../localization.json) records the original and localized file hashes. Chinese presentation-only reports and plots were removed; the JSON/CSV evidence remains.
