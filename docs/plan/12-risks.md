# 12 — Risk register

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| **R1** | **ROM fails to reproduce FEA/Chrono behaviour adequately** | Medium | **Critical** | Phase 1 gate at week 14; fallback is `T6` (soft tread on rigid hub), far easier to model, with narrowed claims |
| R2 | Optimiser exploits contact-solver or ring-discretisation artifacts | **High** | Critical | Solver-perturbation audit incl. discretisation sweep, cross-engine gate, complexity regulariser |
| R3 | FEA pipeline unreliable in batch (convergence failures on soft, buckling geometry) | **High** | High | Robust restart/relaxation schemes; non-convergence is a typed infeasibility, never a crash; log convergence rate as pipeline health |
| R4 | Material model wrong → designs don't transfer | **High** | High | DIY coupon tests; material randomisation; measure real stiffness in Phase 4 and recalibrate |
| R5 | Insufficient throughput | **High — realised** | High | ROM surrogate removes FEA from the loop; T0 pre-filter; aggressive caching; ring segment count tuned to the minimum passing the discretisation audit. **Measured 2026-08-07: the nominal design costs ~20 h per 3-D sweep**, not the 10–40 min this row assumed — 279 k DOF at ~23 min per increment, and coarsening does not help because the band, spoke and bore set the element size. Mitigated 2026-08-08 by the 2-D plane-strain screening tier (20 k DOF, 7.5× less solver time, `k_r` within 14% of 3-D), with 3-D kept as the reference for a small number of designs. See `docs/experiments/log.md` |
| R6 | Insufficient novelty vs NPT literature | Medium | High | Position explicitly against the NPT corpus; lead with C1 (dynamic closed-loop ROM search) and C3 (obstacle traversal at robot scale), not "we optimised spokes" |
| R7 | TPU stiffness drift invalidates hardware measurements | **High** | Medium | Break-in protocol; periodic re-measurement; report drift as a result |
| R8 | Compliant designs buckle or fatigue-fail in testing | Medium | Medium | Buckling and stress constraints with large safety factors; print a spare of each; treat failures as data |
| R9 | Controller confound | Medium | High | Joint controller optimisation from Phase 2; sequential-vs-simultaneous ablation |
| R10 | Rig repeatability worse than the effect size | Medium | High | Pilot the rig with the stock wheel and measure trial-to-trial variance *before* printing candidates; size the expected design gap against it |
| R11 | Scope creep into deformable terrain | **High** | High | Hard rule: no deformable terrain before Phase 3, and then only at T2b fidelity |
| R12 | Multi-objective results hard to communicate | Medium | Low | Three named preference profiles (obstacle-first, efficiency-first, balanced); one champion each |

## Review cadence

Revisit this register at each phase gate. Add new risks as discovered; mark retired risks as
retired rather than deleting them — knowing which risks did *not* materialise is useful when
writing up.
