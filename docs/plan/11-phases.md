# 11 — Phased execution plan

Sizing assumes roughly one full-time person. The compliance work adds about 3 months relative
to a rigid-only version.

## Phase 0 — Foundations (weeks 1–4)

- Freeze the robot spec: chassis, mass, motor stall torque and no-load speed, gear ratio,
  battery, wheel interface. **Written to `configs/robot.yaml`.** Everything downstream depends
  on it.
- Repo skeleton, Hydra configs, DuckDB store, content-addressed caching.
- `θ → STEP + STL + mass properties` for `T0` (rigid cylinder) in build123d.
- `STL → CoACD hulls → MJCF`; get a rigid wheel rolling in MuJoCo.
- One scenario (S1), one metric, one hard-coded controller.
- CI running the full loop on 3 designs in under 5 minutes.

**Gate:** identical `θ` → identical score on two machines, two days apart.

## Phase 1 — FEA and ROM pipeline (weeks 5–14) — the critical path

- Add `T3` (compliant spoke) to the CAD layer with material-region tagging.
- Stand up CalculiX; automate STEP → mesh → hyperelastic material card → load cases → results
  parsing. Unattended, batch, restartable.
- Implement homogenised effective properties per (pattern, density, wall count).
- Run the DIY coupon characterisation (`07-materials.md`) — one week, in parallel.
- Implement the segmented ring ROM in MuJoCo; fit ROM parameters to FEA.
- **Validate the ROM against Chrono ANCF** on 10 designs across the space. Report fit error.
- Run the sanity checks in `06-compliance-rom.md`.

**Gate:** ROM reproduces FEA radial stiffness within 10% and Chrono dynamic response ranking
with ρ > 0.8 on 10 designs.

> **If this gate fails, the whole approach needs rethinking.** Better to know at week 14 than
> week 30. Fallback: restrict to `T6` (soft tread on rigid hub), a much easier modelling
> problem, and narrow the claims accordingly.

## Phase 2 — Optimisation loop (weeks 15–24)

- All scenarios S1–S8; full constraint pre-filter including buckling, fatigue, self-contact.
- Metric aggregation with CVaR over terrain × material realisations.
- Batch evaluation; target ≥ 300 design-evaluations/day at L3.
- Integrate Ax/BoTorch qLogNEHVI with conditional parameters; add controller co-optimisation.
- Stage A screening (500 designs) across `T0` and `T3`; sensitivity analysis.
- Fit the **geometry → ROM surrogate**; validate on held-out designs; report error. From here,
  most designs skip FEA.
- Baselines: random search and NSGA-II at equal budget.

**Gate:** MOBO beats random search on final hypervolume with non-overlapping bootstrap CIs;
ROM surrogate predicts ROM parameters within acceptable error on held-out designs.

## Phase 3 — Main campaign and validity audit (weeks 25–36)

- Add `T1`, `T2`, `T4`, `T5`, `T6`.
- Main cross-topology campaign: 2,500–3,500 designs, bandit-over-families.
- Solver-perturbation audit on top 100, including ring-discretisation sensitivity.
- Cross-engine gate with Chrono ANCF on 70 designs.
- **Ablation: rigid-contact (L0/L1) evaluation of the same designs** → show rigid simulation
  mis-ranks compliant wheels. Headline figure.
- RQ4 analysis: selection penalty under material uncertainty.

**Gate:** cross-engine ρ > 0.7. Produces contributions C1, C3, C4.

## Phase 4 — Hardware validation (weeks 37–50)

- Build the rig: adjustable step (10–200 mm in 10 mm increments), adjustable ramp, gap plate,
  standardised graded rubble tray, washboard plate. Fixed lighting, overhead camera with
  fiducials or motion capture, motor current sensing, IMU logging.
- Print **8 wheels**: top-2 compliant, top-2 rigid, 1 mid-Pareto compromise, 1 predicted-bad
  compliant design (**essential** — failures are needed to establish correlation), 2 baselines
  (stock wheel, simple grousered).
- **Break-in protocol:** fixed revolution count under nominal load before any measurement
  (`07-materials.md`). Re-measure one wheel periodically to quantify drift.
- ≥ 20 trials per wheel per scenario.
- **Primary result:** sim-to-real Spearman ρ, reported separately for rigid and compliant.
- Measure real static radial stiffness of each printed wheel (weights + dial indicator) and
  compare to FEA prediction — a direct, cheap validation of the material model.
- Recalibrate the material model from measured stiffness; re-run a short campaign; report
  whether the Pareto front moves.

**Gate:** sim-to-real rank correlation reported honestly, whatever it is. ρ = 0.6 with
analysis beats ρ = 0.95 that nobody believes.

## Phase 5 — Consolidation (weeks 51–60)

- Release the benchmark (C2): terrains, metrics, reference designs, baseline results, material
  cards, ROM implementation, Docker image, one-command reproduction.
- Write-up. Targets: ICRA/IROS main paper; RA-L for the benchmark; *Journal of Terramechanics*
  or *Mechanism and Machine Theory* for an extended version. The NPT-adjacent contribution may
  also suit *Tire Science and Technology* or an additive-manufacturing venue.
- Ablations: with/without controller co-optimisation, with/without material randomisation,
  with/without domain randomisation, MOBO vs NSGA-II vs random, CVaR vs mean, L1 vs L3
  fidelity, per-family vs cross-family.
