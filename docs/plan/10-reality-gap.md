# 10 — Reality-gap defence

The single most documented failure mode in this field: when fitness is computed in
simulation, the search *will* opportunistically exploit inaccuracies between model and
reality to achieve high fitness with unrealistic behaviour
([Koos et al.](https://hal.science/hal-00687617/file/2012ACLI2214.pdf)).

Four mechanisms, all first-class requirements.

## 1. Domain randomisation

Per rollout: friction coefficients (±40%), restitution, chassis mass (±15%), motor constant
and gearbox efficiency, actuation latency, terrain seeds, initial pose — **plus material
parameters** (`07-materials.md`), which for this project is the dominant uncertainty.

A design that only works at one friction value is a simulator artifact, not a wheel.

## 2. Solver-perturbation audit

For the top 100 designs, re-evaluate under:

- halved timestep
- alternate integrator (RK4 vs implicit)
- ±50% contact softness (`solref` / `solimp`)
- different solver iteration counts
- different convex-decomposition hull budgets
- **different ring discretisation `N`** ← new and most informative

Compute each design's **coefficient of variation across settings**. Flag and discard designs
whose performance swings wildly — they are exploiting the solver.

Ring-discretisation sensitivity is the key one: *a design whose score depends on how many ring
segments were used is not a design, it is an artifact.*

Cost: a few hundred extra evaluations. Value: large, and it is a clean reportable
methodological contribution.

## 3. Cross-engine rank correlation — hard gate

Re-score the top 50 plus 20 random designs in Chrono with the ANCF FEA tire. Spearman ρ
against the ROM ranking:

| ρ | Action |
|---|---|
| > 0.7 | Proceed |
| 0.4 – 0.7 | Usable, must be reported honestly; investigate which metrics disagree |
| < 0.4 | **Stop and fix the model before optimising anything** |

For compliant wheels this gate is not optional. It is the evidence that the entire ROM
strategy works.

## 4. Complexity band

Intermediate-complexity designs tend to reveal real physical advantages, while highly complex
designs become dominated by simulation artifacts. Add a **complexity regulariser** (spoke
count, minimum feature size relative to ring discretisation and contact resolution) and prefer
the simplest design within statistical tie of the best.

Simple designs transfer; a 36-spoke fractal does not.

## Sim-to-real measurement (Phase 4)

The headline validity result is **Spearman ρ between simulated and measured performance,
reported separately for rigid and compliant designs.** If compliant designs transfer worse,
that is a real finding, not a failure.

Secondary: absolute error; whether the solver-perturbation audit score *predicted* transfer
quality (if it does, that is a genuinely valuable tool for the field).
