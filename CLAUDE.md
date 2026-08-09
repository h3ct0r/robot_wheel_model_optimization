# CLAUDE.md

Orientation for Claude. Keep this file short and current — it loads every session.
Detail lives in `docs/`. This file points; it does not duplicate.

## What this project is

Automatic optimisation of the 3D layout of **compliant (TPU) wheels** for a mobile
terrestrial robot: a 400 × 300 × 200 mm, ~10 kg four-wheel skid-steer platform, 24.5 N per
wheel, wheels 120–200 mm in diameter. A parametric CAD model generates candidate wheels; each is evaluated in
closed-loop dynamic simulation over an obstacle-traversal scenario suite; a multi-objective
optimiser proposes the next candidates. Compliance is the object of study, not a detail.

**Research question.** For a fixed chassis and drivetrain, how should wheel geometry and
material compliance be jointly chosen for rigid-obstacle mobility — and can that choice be
made in simulation cheaply enough to search, yet accurately enough to transfer to
FDM-printed TPU hardware?

Full statement and sub-questions: `docs/plan/02-research-questions.md`

## Current state

- **Phase:** 0 — Foundations (see `docs/plan/11-phases.md`)
- **Active work:** ROM feasibility spike, `docs/plan/16-first-week.md`.
  Steps 2 (CAD generator) and 3 (CalculiX radial compression) are **done and verified** —
  `scripts/verify_cad.py` 48/48, `scripts/verify_fea.py --full` 30/30 against CalculiX 2.23.
  **Step 4 (the segmented ring) is done, band included** (`rom-0.4.0`): `wheelopt.rom` fits
  a ring to the FEA `k_r(δ)` at **0.68% RMS on 24 segments**, and the MuJoCo realisation
  tracks the analytic ring to **0.03–0.05%** below 4 mm (4.0–4.8% at 5–6 mm — contact
  discretisation, MuJoCo's round capsules bridge the scallops and touch on more segments
  than the analytic active set; it was 6.1–6.6% before #26). The shear band is two derived
  stiffnesses,
  bending **and hoop**; omitting the hoop term makes the ring 5.28× too soft, all of it the
  `n=0` breathing mode (see the 2026-08-08 log entry). Both come from band geometry and
  modulus via `ring_for_design`, never chosen. **Known gap: the band does not shear**, and a
  shear band is named for that deformation — the ring puts 3 segments on the plate against
  the FEA's 4.4-arc patch, so **patch length from this ROM is a lower bound**. `F(δ)` can be
  right while the patch is not.
  **Steps 5–6 are done and the spike's answer is yes** (`wheelopt.sim.step_climb`,
  `scripts/run_step.py`): **5/5 signatures**, and the compliant wheel clears a **50 mm step
  against the rigid wheel's 20 mm** at matched mass, radius *and* rotational inertia — the
  inertia match is not optional, a solid cylinder has half a ring's and that alone would
  favour compliance. Run on `--tiny` inside its fitted range (0% of samples extrapolated).
  Two caveats that travel with it: cost of transport is a statement about
  `sim.step_climb.TPU_LOSS_FACTOR` (0.15, literature midpoint, 0.05–0.30 span, no DMA), and
  it had not been run on the nominal design — the spring law could not fit it, which is now
  addressed (next bullet). `run_step.py --law table` re-runs the whole rig off a tabulated
  law: 5/5 on `--tiny`. Step 1 is
  mostly closed: `wheelopt.platform.load_platform()` reads `configs/robot.yaml` and
  `tests/test_platform.py` asserts it agrees with `PlatformLimits` / `PARAM_BOUNDS` /
  `LoadCase.nominal_load_n`. Screening still reads the dataclass defaults on purpose
  (`constraints.py` does no I/O). `meta.frozen` is still **false** — the chassis envelope is
  a requirement, but mass, motors, battery and inertia are estimates.
- **The spring law: it was never the cubic, it was monotonicity** (2026-08-08, `rom-0.3.0`).
  `TabulatedLaw` is built — piecewise linear, knots plus interval slopes, fitted by
  `fit_tabulated_law` through a pure-numpy Lawson-Hanson `nnls`; both laws sit behind a
  `RadialLaw` protocol so nothing downstream cares which it got. But the premise that sent it
  there was wrong. **NNLS is convex, so a monotone table's error is the best any monotone law
  can achieve**, and on the nominal that is 12.87% against 2.35% for the same table
  constrained only to non-negative *force*. The nominal's tangent runs **42.8 → −7.0 → +17.3
  N/mm** — genuinely negative — and no monotone segment law can give a ring a negative
  tangent. So the constraint is now "a compressed segment may not **pull**", softening is
  allowed and reported (`is_monotone_nonneg`), and `RingFit.ok` gates on `is_valid_spring`.
  The nominal fits for the first time: **3.42% at 36 segments**. `fit_tabulated_law(monotone=…)`
  keeps the strict set available.
  **But fit error still cannot pick the resolution.** Unregularised, RMS falls 10.60% → 3.42%
  from 4 to 12 intervals while the fitted tangent starts swinging between −52 and +63 N/mm.
  `smoothing` (default 0.1, penalty on tangent change, still one convex NNLS) costs 0.11pp and
  halves the worst spurious tangent — a remedy, **not a cure**. The residual oscillation is
  diagnostic: deconvolving a *banded* wheel's whole-wheel curve into independent radial
  segments is ill-posed, because the band carries load between segments. That is now an
  argument **for** the claw redirection, where the segments are the claws and there is no
  deconvolution. Still open from the same curve: the nominal is **stiff at its own design
  load** — 24.5 N gives δ ≈ 1.3 mm, 1.5% of radius against a tyre's 15–20% — and the *coupled*
  tabulated fit stalls (15.57%, `converged=False`, against 8.32% uncoupled).
- **`detect_buckling` tests a magnitude, not a sign** (2026-08-08). Buckled where the tangent
  falls below `BUCKLING_STIFFNESS_FRACTION` (0.10) of the stiffest tangent reached *earlier* on
  the loading branch. The old `dF/dδ < 0` rule reported `None` for a curve flattening to
  +0.086 N/mm from 12.1. Measured margins: plateau case ratio 0.007, nominal −0.175, the tiny
  design's stiffening sweep 1.157 — a decade either side. The first sample is excluded from
  both the test and the reference; a contact-closure spike as yardstick would flag everything
  after it.
- **The ring divided where it multiplied, and MuJoCo settled it** (2026-08-09, `rom-0.4.0`,
  `TODO.md` #26 closed). A frictionless plate's normal force on a segment is `f_r/cos θ`, not
  `f_r·cos θ` — see `ring.vertical_reaction_n` for the virtual-work derivation. Measured
  per segment against MuJoCo (`condim="1"`, horizontal contact force exactly 0.0 N, reading
  back *its* `u_i` and *its* `λ_i`): `f_r/cos θ` matches to **6.2e-11**, `f_r·cos θ` is off by
  **25%** — exactly `1 − cos²30°`. **But the correction is small on today's designs**: the tiny
  design's fitted `a` moves 3.6% at 24 segments and under 0.3% at 36/48, because its patch is
  three segments wide. It scales with patch spread — 14.1% at ±30° — so it matters for claws.
  The 5–6 mm MuJoCo gap fell from 6.1–6.6% to 4.0–4.8% and the rest is the contact
  discretisation it was first attributed to; sub-4 mm is unchanged at 0.03–0.05%, as it must be.
  Step-climb re-run across the re-fit: still **5/5**.
- **A bandless ring can now splay** (2026-08-09, `solve_equilibrium_2dof`). Second DOF per
  segment, solved per segment by bisection since bandless segments are independent; a banded
  spec is refused. **Exactly inert until a second claw engages** at `R(1−cos π/n)` = 11.4 mm for
  12 claws, then 3% softer at 12 mm, 17% at 25 mm. Design load is δ ≈ 1 mm, so the flat-plate
  fit is unchanged and the benefit is all at a step. **The MJCF joint is still missing**, so
  the analytic ring splays and the simulated one does not — every result the project reports
  above 11.4 mm, the step-climb signatures included, is still the non-splaying one (`TODO.md`
  #20).
- **Two FEA tiers.** 3-D (`MeshSpec.dimension=3`, C3D10) is the reference and is
  **unaffordable at full size**: the nominal design is 50 779 elements / 279 k DOF at ~23 min
  per increment, ≈20 h per sweep, and coarsening does not help because the 3 mm band, 7 mm
  spoke and 4 mm bore set the element size rather than the size field. That is ~30× the
  budget in `12-risks.md`. **2-D plane strain** (`dimension=2`, CPE6, `--plane-strain`,
  `fea/section2d.py`) is the screening tier: 7.5× less solver time on `--tiny`, and 20 k DOF
  against 279 k on the nominal. Measured against 3-D at matched frictionless settings —
  force ratio 0.90, `k_r` ratio 0.86, patch length 0.95 (`verify_fea.py` section 6, which
  asserts ±25% rather than assuming 1). It cannot see lateral spoke buckling at all, so it
  screens and the 3-D tier decides. **Frictional 2-D needs `--contact-stiffness 5`** — the
  penalty is `factor × E / element_size`, so the fine section mesh over-stiffens contact and
  diverges at the default 20; the answer only moves 1.3% across a tenfold change. Do not use
  first-order elements as a speed knob: C3D4 locks and reports a plausible, too-high `k_r`.
- **Direction, 2026-08-08: every future design is bandless, and the family is `T7`, the
  compliant claw** — tapered free-tip fingers cantilevered off the hub, the tips themselves
  the running surface. Shape borrowed from the "Linear Claw" row of Table I in the PaTS-Wheel
  letter (`docs/papers`); the *mechanism* is not — that row is rigid bars sliding through a
  hub, usually gear-driven on wheel stall, whereas `T7`'s compliance is the printed TPU finger
  bending, so it stays inside the existing FEA → ROM pipeline. **PaTS-Wheel itself is not in
  that table**; it places itself between "Linkage Claw" and "Passive Pad Deform".
  `T8`/`T9` (linkage, pivot) are later. Built so far: `claw_taper_ratio` on `WheelParams`
  (1.0 = the uniform strut, unchanged), the outline tapering linearly in **arc length**, and
  `tip_thickness_mm` — which `spoke_min_wall` and the discrete-contact warning now read
  instead of the root, because 7 mm at 0.15 taper is a 1.05 mm tip that the old check passed.
  See `docs/plan/04-design-space.md` §Direction. **The band work of 2026-08-08 is now dormant
  by design** — bandless means both band stiffnesses are 0, `is_coupled` is False, and the
  ring takes the closed-form path, which is numerically what it did before coupling existed.
- **Segments are claws** (2026-08-08, `rom-0.3.0`). `MeshSpec.claw_sector` meshes **one claw
  and the hub** instead of the wheel (`fea/section2d.mesh_claw_sector`); the load case is still
  an ordinary flat plate, so nothing in `deck.py` / `runner.py` / `extract.py` changed.
  `rom.fit.ring_from_claw_curve` then builds a ring with **no fit in it** — segments are claws,
  `n_segments = n_spokes`, the measured curve *is* the spring law. Measured: **492 elements
  against 3155, 0.2 s against 41.7 s**, agreeing with the whole wheel to **0.07%**. Two things
  to know before believing that agreement: at δ ≤ 8 mm only one claw touches the whole wheel
  either (the next tip needs `R(1−cos30°) = 11.4 mm`), so it validates "one claw is one claw"
  and nothing about the multi-claw regime; and the hub-wedge option is **refused** below four
  bore nodes, because a 30° wedge keeps two and describes a claw pinned rather than clamped.
  **A claw tip sticks or slides, and that is a switch rather than a sensitivity** (2026-08-08,
  `TODO.md` #24, closed). Frictionless the nominal claw carries 4.59 N at 1 mm; at **every**
  μ from 0.2 to 1.2 it carries 22.69 N — identical to five figures, and five distinct cache
  keys, so those are five separate solves agreeing rather than one cached result. Both
  branches are **mesh-converged to under 1% over a 7× tip refinement**, so neither is the
  single-node artefact the 0.00 mm contact patch suggested. A ring segment slides radially and
  only radially, so it *is* the stick case — which is also the physical one for TPU on a hard
  floor. **Fit claws at μ ≥ 0.2, never frictionless**; `scripts/explore.py` now picks the
  default from the topology and prints it. **#20 is now the next substantial
  piece of work, and bigger than it reads**: a claw points radially, so a radial tip load
  compresses it as a *column* and a tangential one bends it as a *cantilever*. **Measured**
  on the claw sector with the two new contact-free cases (`TIP_RADIAL`, `TIP_TANGENTIAL`):
  **24.81 N/mm against 0.1851 N/mm — 134×**. (A first estimate said 576×; it used a free-tip
  `3EI/L³`, where the rigid tip cannot rotate, so the guided `12EI/L³` = 0.234 N/mm is the
  right closed form and the measurement sits just below it, as a taper should.) The ring has the stiff direction and
  not the soft one, and a step edge loads the soft one. Two external checks fell out of the
  same arithmetic: analytic `EA/L` 33.7 against the measured 22.69 N/mm, and fixed-free Euler
  **7.19 N** against the measured frictionless plateau **4.59 N** — so that plateau is a
  **buckling column**, not a softening material. **The spike's 50 mm-vs-20 mm headline was a
  banded `T3` design and does not transfer to claws**; a claw climb number from today's ROM is
  a lower bound on compliance.
- **Bandless variant (`rim_thickness_mm = 0`)** is supported end to end: no shear band, the
  spoke tips run on the ground. It is a *topology switch*, not the bottom of the `t_rim`
  range — screening exempts exactly 0 from the bounds and minimum-wall checks and nothing
  else. It makes `spoke_phase_deg` decisive (contact is discrete), so every bandless load
  case must state its phase; use `fea.loadcase.phase_for_tip_contact()`. See
  `docs/plan/04-design-space.md` §`T3b` for what it costs the ring ROM.
- **Watch:** the recurring failure across both stages is a value that is zero / a default /
  a reused artefact, reads as innocuous, and means something else. CAD: a tolerance argument
  that did nothing, an STL that changed with call order, a spoke tip up to 3.3 mm outside its
  own outer radius and therefore outside the FEA contact set. FEA: `D2=0` meaning *infinitely
  stiff*, `*TIME POINTS` where the last set silently wins, peak stress read at the unloaded
  end of the sweep, `--tiny` overriding a flag the user passed explicitly, a `--half-width`
  symmetry flag that nothing implemented and whose only effect was to split the cache, a
  patch comparison that read peak *nodal* pressure (which tracks node count, not load) at two
  different loads, a contact patch reporting zero width because every 2-D node is at z=0.
  **And the same failure in a test harness, not the code:** `grep -q "no convergence"` called
  seven CalculiX runs failures, because ccx prints that per *iteration* and every successful
  nonlinear solve prints it — it nearly retired a working tier. Check whether the step
  completed (`job.sta`), never whether a string appeared. A negative result that kills an
  option needs a second, differently-shaped check before it is believed.
  **And the same failure one level up, in a model rather than a value:** the ring's band was
  a *correct discretisation of an incomplete energy* — bending without the hoop term — and
  every internal test passed, because they all checked the operator against itself. What
  caught it was a comparison with a closed form this repo did not produce (a bare ring
  squeezed between two point loads, `0.1488 F R³/EI`), which came out 5.28× off. **A model
  needs at least one check against a number from outside the model.**
  See `docs/experiments/log.md`, 2026-08-05 to 2026-08-08. Set mesh
  tolerances through `cad/export.remesh()` (never `Shape.tessellate`/`export_stl` directly);
  the FEA `--full` tier must run the real solver — the pure tests cannot catch these.
- **FEA contract for step 4:** `wheelopt.fea.run_load_case(params, material, load_case)`
  returns a typed `FeaResult` (never raises) carrying `LoadCurve` (`k_r(δ)`), `ContactPatch`,
  loaded radius and buckling flag. No hysteresis loss factor — hyperelasticity is
  path-independent; that needs `*VISCO`+Prony (DMA data we lack). Damping must come from a
  material parameter, not FEA.
- **Blocking unknown:** whether an FEA-calibrated reduced-order ring model reproduces
  Chrono ANCF behaviour well enough to optimise against. Phase 1 gate, week 14.

<!-- Update the three lines above whenever the phase or focus changes. -->

## Repo map

| Path | Contents |
|---|---|
| `docs/plan/` | The project plan, split into independently loadable sections |
| `docs/decisions/` | ADRs — non-obvious decisions and their reasoning. **Read before proposing architecture changes.** |
| `docs/experiments/log.md` | Append-only run log: hypothesis, config, result, interpretation |
| `docs/plan/TODO.md` | **Open work, numbered.** What is left and why. Read before picking a task |
| `configs/` | Hydra configs. `robot.yaml` is the frozen platform spec everything depends on |
| `src/wheelopt/cad/` | Parametric geometry (build123d) → STEP + STL + mass properties |
| `src/wheelopt/fea/` | CalculiX batch driver: STEP → mesh → load cases → ROM parameters |
| `src/wheelopt/rom/` | Segmented ring. `ring.py`/`fit.py` pure numpy; `mjcf.py` needs MuJoCo |
| `src/wheelopt/sim/` | MuJoCo scenario runners. `step_climb.py` is the step-5 signature rig |
| `src/wheelopt/metrics/` | Metric extraction and robust aggregation |
| `src/wheelopt/opt/` | Optimiser drivers (Ax/BoTorch, CMA-ES, baselines) |
| `src/wheelopt/viz.py` | PDF report plots (`--plot-pdf`). matplotlib is optional and lazy |
| `src/wheelopt/report.py` | Self-contained HTML reports for `scripts/explore.py`, the manual playground |
| `data/cache/` | Content-addressed artifact cache. Never committed |

## Commands

<!-- Keep this section honest — remove anything that doesn't work. -->

All commands assume the project environment is active. **Plain `conda activate conda3.12`
fails on this machine** — `~/.zshrc`'s conda-init block is commented out and points at a
deleted `miniconda3` prefix — so source the live one first:

```bash
source /opt/homebrew/Caskroom/miniforge/base/etc/profile.d/conda.sh
conda activate conda3.12
```

See the Environment section of `README.md` for the one-time `conda init` fix.

```bash
# Unit tests. No CAD kernel needed; covers params, materials, constraints,
# centreline geometry and mass properties.
python -m unittest discover -s tests -t .

# Screen a design without building geometry (milliseconds, no OCCT).
python scripts/gen_wheel.py --screen-only --spokes 14 --thickness 6.0

# Build, export STEP + STL, report mass properties. Needs build123d.
python scripts/gen_wheel.py --radius 85 --spokes 12 --profile curved --out data/wheels

# CAD verification battery. Run after any change to the geometry layer.
python scripts/verify_cad.py

# FEA: mesh + deck only, no solver. The developer loop.
python scripts/run_fea.py --dry-run --tiny --case flat --case step_edge

# FEA: solve, and write a vector PDF of the design and the extracted metrics.
python scripts/run_fea.py --tiny --case flat --case step_edge --plot-pdf

# FEA: the plane-strain screening tier. Seconds, not hours. Frictional contact on the
# fine section mesh needs a softened penalty; the default 20 diverges.
python scripts/run_fea.py --plane-strain --size-spoke 0.0025 --size-rim 0.003 \
    --size-hub 0.002 --contact-stiffness 5 --case flat

# FEA verification battery. --full adds the wheel sweeps (tens of minutes uncached).
python scripts/verify_fea.py --full

# ROM: fit the ring to the FEA curve, and optionally press it in MuJoCo.
python scripts/run_rom.py --tiny --mujoco

# Steps 5-6: drive the fitted ring at a step beside a rigid wheel and judge the five
# signatures. --sweep adds the tallest step each clears (~20 extra runs, a few minutes).
python scripts/run_step.py --tiny --sweep

# The manual playground: one design (or several) through the whole chain, into one
# self-contained HTML page. This is the thing to reach for when turning a knob by hand.
python scripts/explore.py --spokes 8 --thickness 6 --no-sim     # ~40 s cold, ~2 s cached
python scripts/explore.py --compare spokes=6,10,14 --no-sim     # shared axes
python scripts/explore.py --rim-thickness 0 --claw-taper 0.6 --spoke-phase -90
```

### Environment

One conda env, `conda3.12` (Python 3.12). Conda rather than a venv because **PyChrono has no
PyPI distribution** and the ground-truth tier (ADR-0004) will need it; CalculiX is conda-only
on macOS too. Everything else installs from PyPI into the same env.

```bash
brew install --cask miniforge
conda create -y -n conda3.12 -c conda-forge python=3.12 calculix=2.23
conda activate conda3.12
pip install -e '.[cad,fea,dev]'
ccx -v          # CalculiX 2.23 — the solver is a binary, invoked as a subprocess (ADR-0005)
```

`pip install -e .` alone gets the numpy-only layers (screening, centreline, mass properties);
`[cad]` adds build123d, `[fea]` adds gmsh, `[viz]` adds matplotlib for `--plot-pdf`. Do
**not** use the system `python3` — homebrew's 3.14.6 has a broken `pyexpat` that breaks pip
itself.

## Invariants — do not violate these

1. **FEA never runs inside the optimisation loop.** It is an offline, cached, per-design
   pre-processing step that produces ROM parameters. See ADR-0002.
2. **Mass, inertia and stiffness are always derived from geometry and material**, never
   hard-coded or held constant across designs. A constant-stiffness bug is silent and fatal.
3. **Constraints are a fast pre-filter returning a typed violation vector.** An infeasible
   design costs milliseconds, not a 6-minute FEA run. Never raise for infeasibility.
4. **Nothing kills a campaign.** Diverged sim, failed mesh, non-converged FEA, bad
   decomposition — all return typed failure results. No bare exceptions escape an evaluation.
5. **Every cache key includes the pipeline version and the ROM version.** Changing ring
   discretisation, fitting procedure or material homogenisation invalidates prior results.
   More generally: **anything that can change the numbers is in the key, by default.**
   Exclusions are named explicitly and justified one at a time (`SOLVER_TIMING_ONLY`), never
   assumed for a whole struct — `SolverSpec` was excluded wholesale as "timing", which
   silently shared one cache entry between three different contact models.
6. **Objectives stay multi-objective.** Do not scalarise into a weighted sum. See ADR-0006.
7. **Every design is scored over terrain seeds × material realisations, aggregated with
   CVaR at 25%** — not the mean. Robustness to unknown material properties is a requirement,
   not a refinement.
8. **Rigid-contact "soft contact" (`solref`/`solimp`) is not a compliance model.** It is a
   numerical regulariser. Never use it as a stand-in for TPU. See ADR-0001.

## Conventions

- Python, `src/` layout, package name `wheelopt`.
- Hydra for configuration. No magic numbers in code — they belong in `configs/`.
- Experiment records go to DuckDB + Parquet, one row per
  (design, scenario, seed, material realisation).
- Geometry is authored in build123d and exported to **STEP** for FEA and **STL** for
  simulation. STL is never the source of truth. See ADR-0003.
- Units: SI throughout in code (metres, kilograms, seconds, newtons). Millimetres are
  permitted **only** in CAD parameter definitions and docs, and must be converted at the
  boundary.

## Working agreements

- Before proposing a change to simulator choice, modelling fidelity or CAD toolchain,
  read the relevant ADR in `docs/decisions/`. Several plausible-sounding alternatives
  (Isaac soft bodies, MuJoCo `flex`, OpenSCAD) were evaluated and rejected for specific
  documented reasons.
- When a decision changes, update the ADR (supersede, don't delete) and the
  **Current state** section above.
- When an experiment is run, append to `docs/experiments/log.md`. Claude has no memory
  of previous sessions; that file is the only record of what has been tried.
- **Open work lives in `docs/plan/TODO.md`, numbered, IDs never reused.** Add an entry when a
  follow-up is identified; move it to the closed table when it lands, with the date of the log
  entry that holds the evidence. Two records with two jobs: TODO says what is left, the log
  says what happened. Never put a result in TODO, and if the two disagree the log is right.
- This is a research project. Negative results are results — record them.

## Plan index

| File | Section |
|---|---|
| `docs/OVERVIEW.md` | Plain-language summary + glossary of every acronym. Orientation, not detail |
| `docs/plan/00-index.md` | Index with "read when" guidance |
| `docs/plan/TODO.md` | Open work, numbered, plus standing gaps in what is modelled |
| `docs/plan/01-assessment.md` | Honest assessment, prior work, what will kill the project |
| `docs/plan/02-research-questions.md` | Research questions and where the novelty lives |
| `docs/plan/03-architecture.md` | System architecture and data flow |
| `docs/plan/04-design-space.md` | Topology families, parameters, materials, constraints |
| `docs/plan/05-simulators.md` | Engine-by-engine capability analysis and tiering |
| `docs/plan/06-compliance-rom.md` | **The technical core** — reduced-order compliance modelling |
| `docs/plan/07-materials.md` | TPU characterisation without equipment; cyclic softening |
| `docs/plan/08-metrics.md` | Scenario suite, objectives, logged metrics |
| `docs/plan/09-optimiser.md` | Optimiser design and budget |
| `docs/plan/10-reality-gap.md` | Domain randomisation, artifact audits, validity gates |
| `docs/plan/11-phases.md` | Phased execution plan with go/no-go gates |
| `docs/plan/12-risks.md` | Risk register |
| `docs/plan/13-engineering.md` | Engineering practices |
| `docs/plan/14-cad-toolchain.md` | CAD and asset pipeline |
| `docs/plan/15-extensions.md` | Extensions once the core works |
| `docs/plan/16-first-week.md` | Concrete one-week feasibility spike |
| `docs/plan/99-sources.md` | Bibliography |
