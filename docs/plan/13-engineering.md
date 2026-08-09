# 13 — Engineering practices

These decide whether the project finishes.

- **One command reproduces any result.** `python -m wheelopt.reproduce <run_id>`.

- **Content-addressed caching** on `hash(design + material + pipeline version + ROM version +
  scenario + seed)`. With FEA in the pipeline this is worth weeks.

- **Everything in DuckDB + Parquet**, one row per
  (design, scenario, seed, material realisation). Questions will arise that haven't been
  thought of yet; don't lose the data in log files.

- **Containerise the evaluator.** Pin MuJoCo, CalculiX, Chrono and CoACD versions. Physics
  engines change behaviour between releases, silently.

- **Structured failure.** Diverged sim, failed mesh, non-converged FEA, violated constraint —
  all return typed results. Nothing kills a 40-hour campaign with a traceback.

- **Log solver diagnostics per rollout:** warning counts, max penetration depth, contact
  impulse peaks, energy drift, FEA iteration counts, ROM fit residual. These are the artifact
  detectors.

- **Checkpoint optimiser state.** Campaigns run for days; assume interruption.

- **Visual regression.** Auto-render a short video of every design entering the top 20. Five
  seconds of video catches absurd behaviour that no metric flags — doubly true for compliant
  wheels, where "physically absurd" is visually obvious and numerically subtle.

- **Version the ROM.** Any change to ring discretisation, fitting procedure or material
  homogenisation invalidates every cached result. Put it in the cache key.

- **Determinism.** Seed everything. A design's score must be reproducible bit-for-bit given
  the same seed, config and versions. This is the Phase 0 gate and it is worth defending.

## Experiment discipline

Every campaign gets an entry in `docs/experiments/log.md` **before** it runs, stating the
hypothesis. Filling in the result afterwards is what makes the log useful rather than a
changelog.
