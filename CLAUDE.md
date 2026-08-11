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
  **Step 4 (the segmented ring) is done, band included** (`rom-0.6.0`): `wheelopt.rom` fits
  a ring to the FEA `k_r(δ)` at **0.59% RMS on 24 segments** (0.68% before #12 softened the
  contact penalty; the fit moved, the gap below did not), and the MuJoCo realisation
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
  law: 5/5 on `--tiny`. **That 50-vs-20 is a banded `T3` and does not transfer to claws**; the
  claw number is 30-vs-20, three bullets down. Step 1 is
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
- **The ring's second freedom is a hinge at the claw root, and the FEA settled it**
  (2026-08-09, `rom-0.5.0`, `TODO.md` #27 closed). `solve_equilibrium_hinge` +
  `ring_bodies(tangential="hinge")`; `RingSpec.root_radius_m` is new and comes from
  `hub_radius_mm`. A tangential *slide* moves a claw's tip **outward** as it splays; a root
  hinge moves it **inward**, and the `TIP_TANGENTIAL` sweep measures that for free because it
  leaves the tread's radial DOF loose. Measured on the R 60 claw: at 36 mm of tip travel the
  FEA tip comes in **+19.673 mm**, the hinge predicts **+22.564** and the slide predicts
  **−13.9**. That is the check from outside the model the watch list demands, and it is the
  reason to prefer the hinge — not the rig, which runs on either now. Per segment against
  MuJoCo, both KKT conditions hold to **1e-10** (moment residual over contact force = 6.1e-11 m
  on a 40 mm claw). One correction inside it: the MJCF pivot sits one **capsule radius**
  inboard of the true root, because contact is under the capsule's centre and pivoting at the
  root would shorten the moment arm by 9.8% at 24 segments — in the flattering direction. The
  solve is one bisection on the contact angle `ψ = θ + φ`, not a nested one on the force:
  0.96 s against >100 s, and the rigid limit is exact to 4e-8 instead of 4e-5.
  **Force barely moves, geometry does**: against the slide at the same tip stiffness the
  vertical reaction differs 0.013% / 0.66% / 1.43% at δ = 12/18/25 mm, so every flat-plate fit
  taken with the slide stands. `solve_equilibrium_2dof` and `tangential="slide"` are kept, as
  the thing the hinge is compared against.
- **The rig was never folding because of the element — it was the damper** (2026-08-09). The
  loss-factor damping was applied through `qfrc_applied`, which `implicitfast` integrates
  **explicitly**, and it had been scaled by the segment mass. That is not the inertia the joint
  presents: every hinge axis is parallel to the axle and the axle is free, so one claw's torque
  is reacted by the other eleven and the carriage. Measured, unit force in and `qacc` out:
  hinge `dof_M0` 3.26e-6 kg·m² against an **effective 3.03e-7**, slide 2.0e-3 kg against
  **3.61e-4**, and the collective twelve-claw mode lower again — about **120×** below the mass
  used, giving `c·h/I ≈ 9` and −8.18 growth per step from round-off, in free flight, with
  `ncon == 0`. Eliminated first, by measurement: the softening radial law, friction, solver
  iterations, noslip, segment mass (2→20 g barely moves it) and contact itself. **Fix: emit the
  same damping as the joints' native `damping` attribute**, which `implicitfast` integrates
  implicitly. Not the "dissipation no material supplied" rejected earlier — that was damping
  *added* on top; this is the same number integrated differently. The springs stay explicit and
  `stable_timestep_s` still bounds them at `ω·h ≤ 0.2`; it now lives in `rom/mjcf.py` and the
  **static press asks it too**, having had the same latent bug.
- **The claw wheel clears 30 mm against the rigid wheel's 20** (2026-08-10, `--law claw`) —
  R 60, width 45, 12 claws, taper 0.6, bandless, `run_step.py --tangential hinge --sweep`,
  **5/5 signatures**, matched mass, radius and rotational inertia, both profiles monotone.
  **It is not the spike's 50-vs-20**, which was a banded `T3` and does not transfer.
  **And it supersedes the 60-vs-20 of 2026-08-09**, which was the same design, rig and element
  driven by a *fitted* law over a 6 mm range; the exact measured claw law gives 30. A **2×
  spread in the answer from the segment law alone**, and neither law is validated over the
  range the wheel uses — what argues for this one is that it is exact below 6 mm (0.036%) and
  the run's peak segment compression is 7.47 mm.
  **Quote it as a bucket.** The sweep steps in 10 mm and a 1% change in the law moves the
  answer a whole bucket, so a one-bucket gap between two designs is not a ranking.
  `run_step.py --sweep` prints the **profile** — `[###......]` — not just the maximum, because
  the predicate is not monotone and a bounce reads as a climb otherwise
  (`sim.step_climb.ClimbProfile`, with `censored` and `monotone`).
  Two caveats travel with it: the step-edge patch is the signature most exposed to the element
  (hinge against slide, 24.0 mm against 50.3 mm), and the hinge law is a rigid-bar idealisation
  good to 2% mid-range and 13% at a full claw length.
- **The claw ring's law is now measured, not fitted, and the fit was never the problem**
  (2026-08-10, `rom-0.6.0`, `TODO.md` #29 closed, #31 opened). `run_step.py --law claw` builds the ring from
  a **claw-sector** plate sweep through `ring_from_claw_curve` — no deconvolution, the measured
  curve *is* the segment law — and spends the whole-wheel curve on `fit.validate_ring`, a
  **held-out** check instead of training data (`RingFit.iterations == 0` marks it).
  Below second-claw engagement `ring.second_contact_delta_m` = `R(1−cos 2π/n)` = 8.04 mm the
  whole wheel **is** one claw: agreement **0.036%** over five points. So #29's diagnosis —
  ill-posed deconvolution — was wrong over that range, where there is no deconvolution at all.
  The real cause was **under-parameterisation**: `fit_tabulated_law`'s default
  `n_intervals = min(8, len(d)//2)` picks 3 at 6 points (10.42% RMS) where 4 passes (1.71%),
  because 3 intervals cannot represent a curve peaking at 2.4 mm.
  **Above engagement the ring fails, and it is the element.** Same law, same rig: the radial
  slide reads **+62.7%** at 9.6 mm and the root hinge **−49.5%**. Two idealisations bracketing
  the truth from opposite sides is not a law problem. The FEA also engages its second claw at
  **7.20 mm**, before the 8.04 mm geometric threshold, because a claw at ±30° meets the plate
  on its **flank** rather than its tip. That is the `T7` contact gap, now with a number: #31.
  On this design the platform's 24.5 N per wheel sits at δ ≈ 1.1 mm, well inside the valid
  regime; it is the *step* that leaves it.
- **A stiff segment law needs a smaller timestep than the rig's default** (2026-08-09,
  `rom.mjcf.stable_timestep_s`, re-exported from `sim.step_climb`). `qfrc_applied` is an
  *external* force, so `implicitfast` integrates it explicitly: measured, `ω·h ≥ 0.314`
  diverges inside 5 ms and `≤ 0.251` runs clean, so the bound is `ω·h ≤ 0.2` and the step is
  tightened automatically. **The radial-only rig had been running at 0.63 by luck** — an
  out-of-contact radial segment sits at exactly `u = 0` with exactly zero force, so nothing
  excites it. Heavier segments were rejected (they move the rigid comparator too) and, on the
  claw design, do not work anyway: 2→20 g barely delays the divergence. **This bound covers
  the springs only** — the damping is a separate problem with a separate answer, two bullets
  up.
- **The whole robot now exists, on rigid wheels, and it changes how the headline reads**
  (2026-08-10, `wheelopt.sim.rover`, `scripts/run_rover.py`, `TODO.md` #30). A free-jointed
  chassis box with `configs/robot.yaml`'s own mass, dimensions, inertia, wheelbase, track and
  motor curve, on four driven hinge axles. **A rigid wheel on the robot climbs three times what
  the same rigid wheel climbs on the single-wheel rig** — 1.00 R against 0.33 R at R 60 mm,
  1.06–1.18 R at R 85 mm — because three other wheels push while one climbs and a rigid chassis
  levers the front axle up. So the spike's "compliant 60 mm against rigid 20 mm" is a *rig*
  ratio, and most of it may be the rig's missing wheels rather than compliance. **Do not quote
  that 3x as a property of compliance until it is re-measured on the rover** (#30). Profiles are
  monotone at both radii; at the failing heights the robot rears to 90° and the chassis box
  strikes the riser, which is modelled because ground clearance is 70 mm and the body is a real
  contact geom. The platform loader now reads the seven vehicle fields that sat unread in the
  YAML, and `ring_bodies`/`coupling_tendons` take a `prefix` so four wheels can coexist.
- **`T7L`, the L claw: a foot on the tip, built and screened and deliberately not simulated**
  (2026-08-11, `TODO.md` #35). `WheelParams.tip_hook_mm` bends the last stretch of a claw
  through a right angle so it lies along the running surface — radial leg, filleted bend,
  tangential foot. **Zero is the default and byte-identical to the plain `T7` claw**, so no
  existing `design_hash` moves. Signed like the curvature, because a trailing foot folds closed
  under drive torque and a leading one is levered open; nothing measures that difference yet.
  **The point is contact over an arc rather than at a point**: `polygon_drop_mm` now reads
  `R(1−cos(π/n − β/2))` with `β = |hook|/R`, taking the R 60 twelve-claw drop from **2.04 to
  0.78 mm** at a 12 mm foot — the harshness axis of the bullet below. Two pieces of geometry are
  load-bearing rather than cosmetic: the **bend radius** is `0.75 t_tip` against a half-thickness
  of `0.5 t_tip`, because offsetting a corner of centreline radius ρ gives an inside face of
  radius `ρ−h` and below `ρ = h` the outline turns **inside out** — which OCCT may accept into a
  solid with a reversed patch and a plausible volume; and the **foot is built in polar**, since a
  straight 20 mm foot on R 60 stands 3.2 mm proud of the running surface. `verify_cad.py` §11
  checks self-intersection independently of the kernel: **60/60**. CAD, screening, the mid-plane
  figure and the 2-D FEA tier all took it for free (everything reads `spoke_outline`); a 12 mm
  foot meshes and solves, 90 increments, buckling limit point 30.9 N. **The ring ROM refuses it
  by name** — every segment element carries contact at a point on its own radius, so a fitted
  ring would describe a plain radial claw of the same length. That is #31 arriving by design.
- **Step climb on the rover cannot rank wheels; flat ground can** (2026-08-10, `TODO.md` #33
  opened, #30 amended). On `run_rover.py --sweep` a 3-claw wheel, a 6-claw, a 12-claw and a
  plain rigid cylinder **all clear exactly 1.00 R** at R 60 mm — four wheels, one answer, in
  10 mm buckets. The metric is saturated by the rover's own four-wheel push, not by the
  wheels. `--obstacle-height 0` is now a *scenario* (no step geom is emitted at all) measuring
  objective 3, RMS vertical chassis acceleration from `qacc` on the chassis free joint over
  the second half of the driving phase — the first half is the launch squat, which is about
  the motor. The same four designs separate **22.64 / 10.31 / 5.00 / 0.00 m/s²**, and axle
  work separates them 12×. Two checks from outside the sim track it: the closed-form polygon
  drop `R(1−cos π/n)` and `ring.ride_height_ripple_m`. Compliance cuts the ripple 25% below
  the rigid polygon at 12 claws and **3% at 3**, because a wheel only rides smoother than its
  own polygon if it deflects comparably to the drop, and 24.5 N gives ≈1 mm against a 30 mm
  drop. **Few-clawed numbers are extrapolated and cannot be un-extrapolated**: the law is
  measured to `--delta-max` (12 mm) and a 3-tip R 60 wheel needs 30; the run prints
  `EXTRAPOLATED` with the ratio, widening to 18 mm moved the answer 8%, and 35 mm diverges at
  10 cutbacks. **The metric has no counter-pressure of its own** — it ranks 36 claws above 12
  above 3 forever, and a smooth cylinder wins outright at 0.00. That floor exists to prove the
  metric is not measuring the solver, not to propose a wheel; S7's washboard, where compliance
  should actually win, is #33. Flags and figures: `docs/run_rover.md`.
- **Two FEA tiers.** 3-D (`MeshSpec.dimension=3`, C3D10) is the reference and is
  **unaffordable at full size**: the nominal design is 50 779 elements / 279 k DOF at ~23 min
  per increment, ≈20 h per sweep, and coarsening does not help because the 3 mm band, 7 mm
  spoke and 4 mm bore set the element size rather than the size field. That is ~30× the
  budget in `12-risks.md`. **2-D plane strain** (`dimension=2`, CPE6, `--plane-strain`,
  `fea/section2d.py`) is the screening tier: 7.5× less solver time on `--tiny`, and 20 k DOF
  against 279 k on the nominal. Measured against 3-D at matched frictionless settings —
  force ratio 0.90, `k_r` ratio 0.86, patch length 0.95 (`verify_fea.py` section 6, which
  asserts ±25% rather than assuming 1). It cannot see lateral spoke buckling at all, so it
  screens and the 3-D tier decides. Do not use
  first-order elements as a speed knob: C3D4 locks and reports a plausible, too-high `k_r`.
- **The contact penalty: default 5, and a floor under the element size** (2026-08-09,
  `TODO.md` #12 closed). It is `factor × E / max(element_size, contact_length_floor_m)`, and
  **both halves are now needed**. The factor moved 20 → 5 because the 3-D tier says it costs
  **0.7–0.8%** of the answer (4.29 → 4.26 N frictionless, 4.35 → 4.31 at μ=0.6) and buys the
  conditioning outright — the frictional run goes from 60 increments with 3 cutbacks to 50 with
  none. **2 was rejected**: −2.6 to −2.9% *and* the patch jumps 34.2 → 39.0 mm, which is
  penetration reported as conformity. The floor is the part that was not expected to matter:
  holding the factor at 5 and refining, 4 mm converges and **2.5 and 1.5 mm do not — at either
  factor** — so lowering the factor never bought fine-mesh robustness. Holding the *penalty* at
  the 4 mm value converges at all three, and the two finest agree to **0.02%**. Calibrated on
  one design; re-check on a 150 mm wheel. Both fields are in the cache key. `--plane-strain` no
  longer needs `--contact-stiffness 5`; there is one penalty for both tiers.
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
  **The redirection now has a number** (2026-08-10): with the root hinge of #27 and the
  measured claw law of #29 a claw wheel clears 30 mm against a rigid wheel's 20 at matched
  mass, radius and inertia, 5/5 signatures. The earlier worry that "the tips themselves the
  running surface" could not carry the tractive load was based on a rig that was diverging for
  an unrelated reason. **The *contact* is still the open one, and it is now #31**: the ROM
  loads a claw at its tip, a real claw beds onto its flank as it folds, and that is measurable
  — a claw 30° off the contact point starts carrying 0.84 mm before its tip could reach the
  plate, and the two available elements straddle the truth by +63% and −50%.
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
| `docs/run_rover.md` | Every `run_rover.py` flag with figures — the scene, the wheel models, the metrics |
| `configs/` | Hydra configs. `robot.yaml` is the frozen platform spec everything depends on |
| `src/wheelopt/cad/` | Parametric geometry (build123d) → STEP + STL + mass properties |
| `src/wheelopt/fea/` | CalculiX batch driver: STEP → mesh → load cases → ROM parameters |
| `src/wheelopt/rom/` | Segmented ring. `ring.py`/`fit.py` pure numpy; `mjcf.py` needs MuJoCo |
| `src/wheelopt/sim/` | MuJoCo scenario runners. `step_climb.py` is the step-5 signature rig |
| `src/wheelopt/store.py` | Experiment store: append-only Parquet, DuckDB queries, the determinism gate |
| `src/wheelopt/progress.py` | `Stage` / `Bar` — stage timings on stdout, progress bar on stderr |
| `src/wheelopt/video.py` | MP4 from rendered frames via `ffmpeg` (optional external binary) |
| `src/wheelopt/rom/build.py` | design → FEA → ring, shared by `run_step.py` and `run_rover.py` |
| `src/wheelopt/hashing.py` | `plain()` / `content_digest()` — the one way to hash inputs |
| `src/wheelopt/metrics/` | `aggregate.py` CVaR-25%, `threshold.py` the logistic P=0.9 height |
| `src/wheelopt/sim/s1_step.py` | Scenario S1: the step ladder × terrain seeds, into store rows |
| `src/wheelopt/opt/` | Optimiser drivers (Ax/BoTorch, CMA-ES, baselines) |
| `src/wheelopt/viz.py` | Report plots. `draw_wheel_section` mid-plane, `draw_wheel_profile` axial |
| `scripts/plot_geometry.py` | One figure per geometry parameter across its range, with verdicts |
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

# Every geometry parameter across its own range, one figure each, with the screening verdict
# under each design. Mid-plane section, except width and tread depth, which that view cannot
# see at all -- those get the axial one. Seconds, numpy only, no CAD kernel.
python scripts/plot_geometry.py --contact-sheet
python scripts/plot_geometry.py --only taper --format pdf

# Build, export STEP + STL, report mass properties. Needs build123d.
python scripts/gen_wheel.py --radius 85 --spokes 12 --profile curved --out data/wheels

# CAD verification battery. Run after any change to the geometry layer.
python scripts/verify_cad.py

# FEA: mesh + deck only, no solver. The developer loop.
python scripts/run_fea.py --dry-run --tiny --case flat --case step_edge

# FEA: solve, and write a vector PDF of the design and the extracted metrics.
python scripts/run_fea.py --tiny --case flat --case step_edge --plot-pdf

# FEA: the plane-strain screening tier. Seconds, not hours. The softened contact penalty it
# used to need by hand is the default since #12, so there is no --contact-stiffness here.
python scripts/run_fea.py --plane-strain --size-spoke 0.0025 --size-rim 0.003 \
    --size-hub 0.002 --case flat

# FEA verification battery. --full adds the wheel sweeps (tens of minutes uncached).
python scripts/verify_fea.py --full

# ROM: fit the ring to the FEA curve, and optionally press it in MuJoCo.
python scripts/run_rom.py --tiny --mujoco

# Steps 5-6: drive the fitted ring at a step beside a rigid wheel and judge the five
# signatures. --sweep adds the tallest step each clears (~20 extra runs, a few minutes).
python scripts/run_step.py --tiny --sweep

# The claw wheel: root hinge, and a segment law measured on one claw rather than deconvolved
# from the whole wheel. Bandless designs only; the whole-wheel curve becomes a held-out check.
python scripts/run_step.py --radius 60 --rim-thickness 0 --spokes 12 --thickness 6 \
    --claw-taper 0.6 --spoke-phase -90 --plane-strain --segments 12 --law claw \
    --tangential hinge --delta-max 0.012 --n-points 10 --sweep

# The whole robot at a step, filmed. Add --compliant for four segmented rings instead of four
# cylinders -- a picture, not a measurement (TODO #30/#31). Every stage reports its own time
# and the long ones carry a progress bar on stderr.
python scripts/run_rover.py --obstacle-height 80 --radius 85 --render
# --obstacle-height 0 is a different scenario, not a small step: no obstacle, and the run
# reports ride harshness (objective 3) with two analytic checks beside it. This is the metric
# that separates wheel designs; the step-climb sweep on the rover does not.
python scripts/run_rover.py --obstacle-height 0 --compliant --radius 60 --rim-thickness 0 \
    --spokes 3 --thickness 6 --claw-taper 0.6 --spoke-phase -90 --plane-strain --law claw
# --stl draws the real CAD geometry over each wheel, translucent grey at 40%, so the ring's
# amber capsules can be seen against the shape they stand for. Decoration: no collision, no
# mass, and every number is byte-identical with and without it (tests/test_rover.py).
# --render writes an MP4 (needs ffmpeg on PATH), a GIF and a contact sheet; the MP4 is ~13x
# smaller and full-colour, so --no-gif is usually what you want.
python scripts/run_rover.py --compliant --stl --radius 60 --rim-thickness 0 --spokes 12 \
    --thickness 6 --claw-taper 0.6 --spoke-phase -90 --plane-strain --law claw \
    --tangential hinge --obstacle-height 50 --render

# Scenario S1: the step ladder across terrain seeds, the P=0.9 height, and every run stored.
# ~25 s for 80 runs. --repeat 2 --gate is the Phase 0 determinism gate.
python scripts/run_s1.py
python scripts/run_s1.py --heights 20:120:20 --repeat 2 --gate

# The manual playground: one design (or several) through the whole chain, into one
# self-contained HTML page. This is the thing to reach for when turning a knob by hand.
python scripts/explore.py --spokes 8 --thickness 6 --no-sim     # ~40 s cold, ~2 s cached
python scripts/explore.py --compare spokes=6,10,14 --no-sim     # shared axes
python scripts/explore.py --rim-thickness 0 --claw-taper 0.6 --spoke-phase -90

# T7L, the L claw: a tangential foot at the tip. Bandless only; --tip-hook is SIGNED and 0 is
# the plain radial claw. CAD, screening and the 2-D FEA tier take it; the ring ROM refuses it
# by name (TODO #35), so --law claw stops rather than fitting a plain claw of the same length.
python scripts/gen_wheel.py --radius 60 --rim-thickness 0 --spokes 12 --thickness 6 \
    --claw-taper 0.6 --spoke-phase -90 --tip-hook 12 --out data/wheels
python scripts/verify_cad.py --only 11
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
`[cad]` adds build123d, `[fea]` adds gmsh, `[viz]` adds matplotlib for `--plot-pdf`,
`[store]` adds duckdb + pyarrow for `wheelopt.store`. Do
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
