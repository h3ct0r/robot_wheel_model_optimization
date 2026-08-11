# Open work

The numbered list of what is not done. **IDs are stable and never reused** — a closed item
keeps its number so that a log entry or a commit message saying "closes #17" stays resolvable.

## What this file is, and is not

It is the *index* of open work. It says what is left, why, and what it depends on — one short
entry each, with a pointer to where the detail lives.

It is **not** a second record of what has been tried. That is `docs/experiments/log.md`,
which is append-only and is the only place a result should be written down. When an item here
closes, the evidence goes in the log and the entry here moves to the closed list at the bottom
with a date. If the two ever disagree, the log is right.

It is also not the plan. `docs/plan/11-phases.md` says what the project is doing over sixty
weeks; this file says what is blocking the next fortnight of it.

---

## Next

### #20 — Add tangential claw compliance to the ring ROM

**Done for the flat-plate and step rigs; open for what the number now rests on.** The
element, its MJCF realisation and the driven rig all work as of 2026-08-09 — see the log entry
of that date for the evidence and #27 in the closed table for the element itself.

Where it stands. A bandless ring's claws each have a root hinge with a moment-rotation law
measured by a `TIP_TANGENTIAL` sweep on the design's own claw sector, and
`scripts/run_step.py --tangential hinge` drives it. The R 60 mm, 12-claw, taper 0.6 design
passes **5/5 signatures** and clears a **60 mm step against the rigid wheel's 20 mm** at
matched mass, radius and rotational inertia. That is the project's first end-to-end claw climb
number, and it is **not** the spike's 50-vs-20, which was a banded `T3`. It read 30-vs-20 until
the #12 re-run; see the 2026-08-09 log entry, which supersedes that.

What is left, in order:

0. ~~The climb height is worth one significant figure and the sweep knows it.~~ **Done**
   2026-08-09: `step_climb_profile` keeps every outcome and `ClimbProfile` reports `censored`
   and `monotone`, so a climb is distinguishable from a bounce and from a sweep that ran out
   of range. The 10 mm resolution and the one-bucket sensitivity remain — quote a bucket.
1. ~~14% of loaded samples run past the fitted range.~~ **Done 2026-08-09, and the answer is
   negative.** Widening to 12 mm gets the step run to 0% beyond the fit and leaves the climb
   at 60 mm, but no fit of this design over its working range passes the 5% gate, and the
   12 mm fits cannot carry 24.5 N at all. The design load needs δ ≈ 8.2 mm against a 6 mm
   passing fit. Continued as **#29**.
2. **The step-edge patch disagrees between elements** — 24.0 mm for the hinge against 50.3 mm
   for the slide on the same design. That is the outward tip inflating the patch, so the hinge
   is the one to believe, but the size of the gap says patch length is the metric most exposed
   to the element choice and it is one of the five signatures.
3. **The hinge law is a rigid-bar idealisation of a bending claw.** It reproduces the measured
   tip trajectory to 2% mid-range and 13% at a full claw length
   (`fit.hinge_kinematics_check`), which is good enough to choose the element and not
   obviously good enough to quote a climb height to two figures.

Also unresolved and cheap: `run_step` graded a collapsed wheel **4/5** before this landed,
because it accepts any run whose history stays finite. `fraction_beyond_fit` did say 100%, but
the headline did not. The collapse is gone; the grading hole is not.

---

## The claw family's own gaps

These arrived with the `T7` redirection on 2026-08-08 and are listed in that log entry.

### #30 — Re-run the compliant-versus-rigid comparison on the rover, not the test rig

Opened 2026-08-10 by the four-wheel rigid rover, which climbs **three times** what the same
rigid wheel climbs on the single-wheel rig: 1.00 R against 0.33 R at R 60 mm, and 1.06-1.18 R
at R 85 mm. Nothing about the wheel changed; three other driven wheels push while one climbs.

**This puts a question mark over the headline ratio.** "Compliant clears 60 mm against the
rigid wheel's 20" is a single-wheel-rig number, and on that rig the rigid wheel is handicapped
by having no other wheels. If a rigid wheel recovers to 1.00 R on a robot, most of that 3x was
the rig rather than the compliance. The single-wheel result is not wrong — it measures what it
says — but it does not transfer, and the ratio must not be quoted as a property of compliance
until it has been measured on the rover.

**Work.** Put the fitted ring in the rover's four mounts (`ring_bodies(prefix=...)` is ready)
and re-run the comparison at matched mass, radius and rotational inertia, as the single-wheel
rig already does. Expect it to be slow: four rings is 4x the segment bodies and joints on a
timestep already tightened by the explicit-integration bound.

**And the metric it was going to be measured with does not discriminate.** 2026-08-10: on
`--sweep`, a 3-claw wheel, a 6-claw, a 12-claw and a plain rigid cylinder **all clear exactly
1.00 R** at R 60 mm. Four different wheels, one answer, in 10 mm buckets. So #30 needs a
metric before it needs a law — the flat-ground harshness of #33 does separate the same four
designs 4.5x, and cost of transport separates them 12x.

**Blocked on #31** for a law valid over what a rover does to a wheel — #29 closed by giving
the claw ring an exact law below second-claw engagement and showing that the element, not the
law, is what fails above it. Also blocked on the three gaps the rover's module docstring names,
of which the first is the serious one: the ring is **planar**, so a wheel loaded out of plane
-- by roll, or by dropping off an edge -- is perfectly rigid. A rover exercises that constantly
and the single-wheel rig never did.

### #31 — A claw at ±2π/n meets the plate on its flank, and the ring has no such contact

**Partly closed 2026-08-11, and the "measure before choosing" instruction has been carried
out.** See the log entry of that date. What changed:

- **The onset now has a closed form and it is right.** A claw's deepest material is its tip
  *corner*, half a thickness off its own axis, so its reach is `R cos θ + h sin|θ|`.
  `RingSpec.tip_half_thickness_m` (`rom-0.7.0`) puts that in both solvers. It predicts
  second-claw engagement at **7.14 mm** where the point tip said 8.04 and the FEA measures
  **7.20** — one sample. The 0.84 mm discrepancy this item opened with is explained.
- **It does not fix the straddle, and that is the finding.** Above engagement the error goes
  from +62.7% / −49.5% (slide / hinge) to **+74.7% / −45.6%**. The hinge gains 4 pp; the slide
  gets *worse*, because a too-late onset had been partly cancelling a too-stiff element. So
  contact onset was never the cause. **The cause is bedding** — a claw lying down along its
  flank, loaded in bending over a patch that travels — and no correction to a point-contact
  element reaches it.
- **The bound is now measured rather than asserted.** `BuiltRing.validity_delta_m` and
  `RoverResult.multi_contact_fraction` say where a ring is trustworthy and how much of a run
  left that range. Measured: a flat rover run at nominal load is on one claw for **91%** of
  the driving phase, so the ROM covers it; a 40 mm step run compresses one segment to
  **21.98 mm** against a law measured to 12, which is the *other* failure and is reported
  separately.

**What is left is one decision, and it is now well posed.** Either build a segment that can
carry a distributed flank contact — a beam-on-foundation per claw, or a collision surface with
length — or accept the single-claw regime as the ROM's envelope and screen designs on where
their duty cycle sits inside it. The second is what the code does today, honestly and with
numbers. The first is the only way to a trustworthy `T7L` (#35) or a trustworthy step-climb
comparison on the rover (#30).

Original statement follows.

Opened 2026-08-10 by #29, which set out to fix a fit and found that the fit was never the
problem. The log entry of that date carries both tables.

Below second-claw engagement a bandless claw ring built from one claw's own measured curve
reproduces the whole wheel to **0.036%**. Above it, the *same law* in the *same rig* gives
**+62.7%** with a radial slide and **−49.5%** with a root hinge, at δ = 9.6 mm on the R 60 mm,
12-claw, taper 0.6 design. Two idealisations bracketing the truth from opposite sides is a
statement about the **element**, not the law.

**What the element is missing.** The ROM loads every claw at its tip, along its own radius. A
claw 30° from the contact point meets a flat plate side-on, and carries load through its
*flank* — partly as a column, partly in bending, over a patch rather than at a point. Neither
a pure radial slide nor a pure root hinge is that. Independent evidence for the same thing,
from the onset rather than the magnitude: the FEA has a second claw carrying at **7.20 mm**,
0.84 mm *before* the geometric threshold `R(1 − cos 2π/n)` = 8.04 mm
(`ring.second_contact_delta_m`), because flank contact starts before the tip arrives.

This is the same gap CLAUDE.md's `T7` bullet already names — "a real claw beds onto its side
as it folds, and nothing here models that" — now with a number and a boundary. It is also the
same family as "the shear band does not shear": a correct discretisation of an incomplete
kinematic description.

**Work.** Decide what a segment's contact is. Options not yet costed: a capsule per claw whose
*side* is the collision surface with a compliance law along its length; a two-segment claw
(root hinge plus a mid hinge) so the flank can conform; or accepting the single-claw regime as
the ROM's validity envelope and screening designs on where their design load sits relative to
it. **Measure before choosing**, and note that the third option is nearly free and may be
enough — on this design the platform's 24.5 N per wheel sits at δ ≈ 1.1 mm, comfortably inside
the valid regime, and it is the *step* that leaves it.

**Blocks #30**, which needs a law valid over what a rover actually does to a wheel.

### ~~#29 — No fit of the driven claw design passes the gate over its working range~~

**Closed 2026-08-10, and it closed by refuting its own diagnosis.** See the log entry of that
date. Three things came out of it:

- **The sub-engagement failure was under-parameterisation, not ill-posedness.** Below
  `R(1 − cos 2π/n)` the whole wheel *is* one claw — measured to 0.036% — so there is no
  deconvolution over that range to be ill-posed. `fit_tabulated_law`'s default
  `n_intervals = min(8, len(d) // 2)` picks **3** intervals at 6 points, giving 10.42%, where
  **4** gives 1.71% and passes. Three cannot represent a curve that peaks at 2.4 mm.
- **Candidate 2 works and is now wired in.** `run_step.py --law claw` builds the ring from a
  claw-sector plate sweep with no fit in it, and spends the whole-wheel curve on
  `validate_ring` — a **held-out** check instead of training data.
- **The climb is 30 mm against the rigid wheel's 20**, not 60. The 60 came from `--law table`
  on a 6 mm fit; the exact measured law gives 30. Same design, same rig, same element: a 2×
  spread from the segment law alone.

What is left is not a fitting problem and has its own number: **#31**.

### #33 — Ride harshness is measured on flat ground only, where a rigid wheel cannot lose

**Closed in substance 2026-08-11; what remains is a sweep, not a build.** See the log entry.
`RoverSpec.washboard_amplitude_m` / `washboard_wavelength_m` and `run_rover.py --washboard
--wavelength` add S7's corrugation — a strip of boxes sampling the sinusoid, entered at a
trough so the doorstep is a ramp and the transient belongs to the terrain, not its edge.
**The sign reverses, and it is not close**: at 10 mm peak-to-trough the compliant twelve-claw
wheel beats the rigid cylinder **4.8–6.9x** at every wavelength from 60 to 400 mm, at equal or
higher speed, and the metric orders rigid > 3-claw > 12-claw the right way round. The caveat
travels with it: 20–53% of those runs have two claws sharing (the #31 element gap), so the
**sign** is the result and the second digit is not. Still open here: the amplitude x wavelength
*sweep* S7 specifies (this measured one amplitude), and terrain seeds over it.

Original statement follows.

Opened 2026-08-10 by wiring up objective 3. See the log entry of that date for the numbers.

`run_rover.py --obstacle-height 0` now measures RMS vertical chassis acceleration, and it
**does** rank wheels where step climb on the rover does not: 22.64 / 10.31 / 5.00 m/s² for 3 /
6 / 12 claws, against a single saturated 1.00 R for all of them plus the rigid cylinder on the
climb sweep. Two independent analytic companions — the closed-form polygon drop and the ring's
loaded ripple — track it.

**What is missing is the scenario where compliance wins rather than loses by less.** On a
smooth plane a smooth rigid cylinder reads 0.00 m/s² and cannot be beaten; every compliant
wheel is scored on how close it gets back to a wheel nobody can print. `08-metrics.md` asks S7
for a **washboard** — sinusoidal ripple swept over amplitude and wavelength — which is where
the sign is supposed to reverse, and nothing here demonstrates that it does.

**Work.** Add the S7 terrain to `sim/rover.py` (a strip of boxes, or a heightfield), sweep
amplitude and wavelength, and check whether a compliant wheel beats the rigid comparator at
any point in that space. If it does not, that is a result about the ROM's damping — every
cost-of-transport and harshness number is a statement about `TPU_LOSS_FACTOR = 0.15`, a
literature midpoint on a 0.05–0.30 span with no DMA behind it.

**Also open, smaller.** The metric is quoted at one speed, and harshness scales with speed;
`tip_frequency_hz` is reported alongside it so the two are not confused, but nothing sweeps
speed yet. And a wheel with few claws needs its segment law measured over its own polygon
drop — 30 mm for 3 tips at R 60 — which the FEA currently **cannot** reach: it diverges at 10
cutbacks by 20 mm. The `EXTRAPOLATED` warning is honest about it; widening 12 → 18 mm moved the
answer 8%, so the ranking is safe and the second digit is not.

### #35 — The L claw exists in CAD and cannot be simulated

Opened 2026-08-11 with the `T7L` topology itself. See `04-design-space.md` §`T7L`.

`tip_hook_mm` puts a tangential foot on the claw tip. The geometry is built, screened, drawn
and meshed — `verify_cad.py` is 60/60 with a new section 11, and the 2-D FEA tier solves a
12 mm foot on the R 60 twelve-claw design end to end. **What cannot be done with it is the
thing the wheel is for.**

The ring ROM loads each segment at a point along its own radius, through either a radial slide
or a root hinge. A foot's whole purpose is that it does *not* do that: it beds along an arc, and
the load moves along the foot as the wheel rolls. So `run_step.py --law claw` and
`run_rover.py --compliant` will build a ring for a `T7L` design and that ring will describe a
radial claw of the same length — silently, because nothing in the pipeline knows the difference.

This is **#31 arriving by design rather than by accident**, and it makes #31 harder to defer:
for `T7` the flank contact was an error above second-claw engagement, and for `T7L` it is the
first-order behaviour at any load.

**The refusal has landed.** `rom.build.build_ring` turns an L claw away by name, before any
solver time, and names this item in the message; `run_step.py --law claw` and `run_rover.py
--compliant` therefore stop rather than produce a number about a different wheel. Three tests
pin it, including that a footless design is still turned away for its *band* and not for a foot
it does not have.

**What is left is the element, and it is a decision before it is code.** Extend the segment so
a claw can carry a distributed contact — a capsule whose *side* is the collision surface is the
cheap version, and the rover already draws segments as capsules — or keep the refusal and treat
`T7L` as a CAD/FEA-only family. Measure before choosing.

Two smaller things travel with it, both easy to miss:

- **Nothing measures the sign.** A trailing foot folds closed under drive torque and a leading
  one is levered open; the field is signed, both are buildable, and no experiment separates them.
- **An early FEA observation, not yet a result.** At R 60, twelve claws, taper 0.6, plane
  strain: the 12 mm foot completes its sweep (90 increments, buckling limit point at 30.9 N)
  while the *plain* claw at the same settings **diverges**. One design each, so it is a note
  rather than a finding — but if it holds, a spread contact is easier on the contact solver as
  well as on the ride, and that would be worth knowing before choosing the element above.

### #34 — `rim_thickness_mm`'s lower bound is below the TPU wall, and unlike the spoke's, nothing says why

**Closed 2026-08-11: same rationale as the spoke, now stated for both, once, with the cost
measured.** The range must be able to express a design `rim_min_wall` rejects, or the check
can never fire — a constraint no sample can violate is indistinguishable from one that was
deleted. Cost: ≈5.9% of a uniform sweep per field, rejected in milliseconds by screening.
Shared statement in `04-design-space.md` §Manufacturing; both `PARAM_BOUNDS` comments point
at it. The trigger for revisiting is a real optimiser measurably concentrating near the wall,
and the remedy then is a material-dependent bound, not a raised floor.

Original statement follows.

Opened 2026-08-10 by `scripts/plot_geometry.py`, which draws each parameter across its own
range and puts the screening verdict under every panel. Two ranges come out red at their
searched lower bound. **One of the two is deliberate and documented; the other is not, and
that asymmetry is the item.**

`PARAM_BOUNDS["rim_thickness_mm"]` and `PARAM_BOUNDS["spoke_thickness_mm"]` are both
**(1.2, 8.0)** while `PlatformLimits.min_wall_thickness_tpu_mm` is **1.6** — 1.2 being the
*rigid* wall. Measured at R 60 with both fields set together: 1.2 and 1.5 each return two
`INFEASIBLE` violations, 2.0 is clean.

For the spoke this is on purpose. `params.py` says so in place: the bound sits below the wall
"so that `spoke_min_wall` stays a live check rather than being made unreachable by the range".
That is a real argument — a constraint no sample can violate is a constraint that has stopped
testing anything — and it is a trade against wasted evaluations, not an oversight.

`rim_thickness_mm` carries no such note. It may be the same reasoning applied twice, or it may
be the spoke's bound copied. **The work is to decide and write it down**, not to move a number:
either record the same rationale for the rim, or raise the rim's floor to the wall. If the
rationale is the right one it should also be stated once, in `04-design-space.md`, rather than
in a comment on one of the two fields.

**Worth measuring either way:** what fraction of a real search's samples land in the
unreachable band, which is the cost the rationale is being traded against. Nothing is searching
yet, so nobody has paid it.

The same figures show two bounds infeasible at the *top* — `n_spokes` 24 and 36 hit
`interspoke_gap`, `tread_depth_mm` 4 exceeds a 3 mm band. Those are honestly design-dependent:
they follow from radius, thickness and band, and a scalar bound cannot know them.

### #28 — The slenderness threshold of 40 is far too permissive for a claw

**Closed 2026-08-11: the threshold cannot be fixed, because the axis is flat where the
answer moves.** The frictional deep sweep this item asked for was run — claw sector, μ = 0.6,
12 mm, tapers 1.0 / 0.6 / 0.4 on the R 60 twelve-claw design — and **the stick branch has a
limit point**: 105.4 / 39.4 / 22.7 N at 2–4 mm of deflection. A 4.6× collapse in buckling
load, across which the slenderness proxy creeps **6.3 → 7.2 → 7.8**. No threshold on an axis
that flat ranks that family; tuning the 40 down to catch claws would be calibrating a constant
on three points of one family. The check that *does* catch them is `fea_buckling`, which
measures each design's own limit point against 2.5× nominal — it fails all three tapered claws
where the warning stays silent, and it already fired unprompted on the `T7L` run of the same
day. The warning is kept for the corner it was written for (a 1.6 mm strut on R 100 reads 48,
where geometry alone predicts a poor ROM fit) and its comment now states its blindness with
the measurement. Deeper than 12 mm the tapered claws diverge (~15–17 mm, 8–10 cutbacks) —
consistent with snap-back, which CalculiX has no arc-length solver to traverse; recorded as
the boundary of what this rig can measure.

Original statement follows.

Opened 2026-08-09 by #21, which fixed the *proxy* and left the *threshold*.

`constraints.py` warns above `slenderness > 40`. On the frictionless claw-sector plate sweep
of that date every design showed a load plateau — effective slenderness 8.1 through 26.0, so
none of them warns — and every plateau sat **below** the per-claw design load. A screening
threshold that fires on nothing in the family it is meant to screen is not screening.

**Do not simply lower it.** The frictionless plateau is the *slip* branch, and #24 established
that the physical branch is **stick**: at every μ from 0.2 to 1.2 the nominal claw carries
22.69 N at 1 mm against the frictionless 4.59 N, and the column mode does not appear there. So
the question is which branch a slenderness warning is about. Answering it needs a frictional
sweep deep enough to find the stick branch's own limit point, if it has one.

Until then the proxy is right and the threshold is decoration.

## Deferred

Real, understood, and not on the critical path.

### #36 — Phase 0's three unbuilt bullets: `T0` in CAD, CoACD→MJCF, Hydra

Recorded 2026-08-11, when the rest of Phase 0 closed (CI and the cross-machine gate — see the
log entry of that date). These three are the checklist items deliberately **not** built, each
because it currently has no consumer, and building for no consumer is how a wrong interface
gets frozen:

- **`T0` rigid cylinder in the CAD layer.** The *simulated* rigid baseline exists and is what
  every comparison uses — an analytic cylinder with matched mass, radius and inertia. The CAD
  `T0` is for printing a hardware baseline, which is Phase 4, and it does not fit
  `WheelParams` (spokes are mandatory there), so it is a new topology switch with its own
  screening — real surface area to add for a part nobody prints yet.
- **`STL → CoACD hulls → MJCF`** (ADR-0007). Needed the day a rigid *shaped* wheel — `T1`
  grousered, `T2` lobed — must collide in MuJoCo. Today rigid wheels are cylinders and
  compliant ones are capsule rings; there is no mesh that needs decomposing. The ADR stands;
  the wiring waits for the first `T1`.
- **Hydra.** `configs/robot.yaml` is read by a tested loader and every CLI is argparse; the
  conventions section still names Hydra as the intended config system. Wiring it now churns
  eleven entry points for no behavioural change. The honest trigger is the optimiser (#9 in
  the plan), whose sweep configs are what Hydra is actually for.

What did close: CI runs the unit suite and lint on every push, and the **cross-machine
determinism gate is now a live experiment** — three designs' S1 ladders, run on Linux x86-64
against manifests committed from this macOS arm64 machine, bit-for-bit. `run_s1.py
--manifest-out/--manifest`, `store.manifest_from_records`/`compare_manifests`.

### #32 — `fit_tabulated_law`'s default interval count is a rule of thumb, and it is too coarse

Opened 2026-08-10 by #29, which found it while looking for something else.

`n_intervals = min(8, len(d) // 2)`. On six points that is **3**, and on the one case where
the exact answer is known — a bandless claw wheel below second-claw engagement, where the whole
wheel is a single measured curve — 3 intervals give **10.42%** RMS where 4 give **1.71%** and
pass the gate. Three cannot represent a curve that peaks at 2.4 mm and softens for the next
ten; the halving is not derived from anything about the curve.

**Deferred rather than fixed, because changing it moves every number this project has.** Every
banded fit on record was taken at the current default, and the honest way to change it is to
re-fit and re-measure rather than to raise a constant. The bandless claw path no longer needs
it at all (#29), which is why this is not urgent.

**Candidates.** Choose the count by cross-validation on the curve rather than from its length;
or keep the rule and raise the floor; or expose it and make every caller state one. The
`smoothing` penalty already there is the reason a finer table is not automatically worse.

**2026-08-11: the first candidate is built, opt-in, and the default is untouched.**
`fit.n_intervals_by_cv` picks the resolution by leave-one-out prediction — the number that
turns back up where fit error keeps falling — and on a curve generated from a known 4-interval
law it recovers **4** where the length rule picks 3; ties go to the coarser table, endpoints
are never dropped (the fit does not claim its own extrapolation), and a caller passes the
result explicitly as `n_intervals=`. What keeps this item open is unchanged: *switching the
default* to it means re-fitting every banded result on record, and that re-run has not been
done. The tool now exists for the day it is.

---

## Standing gaps

Not tasks with an owner or a next action, but things a reader should know are missing. Each is
argued where it lives; they are collected here so the list of what is *not* modelled is in one
place.

- **The shear band does not shear.** Bending and hoop only, so contact patch length from the
  ROM is a lower bound. `src/wheelopt/rom/ring.py` module docstring.
- **No hysteresis from FEA.** Hyperelasticity is path-independent; a loss factor needs
  `*VISCO` plus a Prony series, which needs DMA data this project does not have. Damping is a
  material parameter, `sim.step_climb.TPU_LOSS_FACTOR = 0.15`, a literature midpoint on a
  0.05–0.30 span. Every cost-of-transport number is a statement about that constant.
- **`configs/robot.yaml` is not frozen.** `meta.frozen: false`. The chassis envelope is a
  requirement; mass, motors, battery and inertia are estimates.
- **`MeshSpec.half_width_symmetry` is declared and not implemented**, and `__post_init__`
  refuses `True` rather than accepting it silently. See the field's comment.
- **No anisotropy.** CalculiX has no anisotropic hyperelastic model, and an FDM part is
  layered. Out of scope, not overlooked.
- **The ring is planar; a robot is not.** Each ring lies in its own x-z plane and its segments
  move radially and in-plane-tangentially only. A wheel loaded **out of plane** — by chassis
  roll, or by dropping off an edge — is perfectly rigid in this ROM. The single-wheel rig never
  exercised it; `wheelopt.sim.rover` does, constantly. Same family as "the band does not shear".
- **Skid steer scrubs, and that is not validated.** Four non-steered wheels cannot turn without
  sliding sideways, and lateral scrub of a segmented capsule ring has never been compared
  against anything. Only straight-line driving is supplied.
- **Phase 0 is part done, as of 2026-08-10.** Landed: the store (`wheelopt.store`, append-only
  Parquet + DuckDB), the metrics layer (`metrics.aggregate` CVaR-25%, `metrics.threshold` the
  logistic P=0.9 height), and **scenario S1 end to end** (`sim.s1_step`, `scripts/run_s1.py`) —
  80 runs in 23 s giving 44.7 ± 9.1 mm on rigid R 85 wheels. The determinism gate runs and
  passes **on one machine**: 80 repeated `run_id`s, zero disagreements.
  Still open: `T0` in the CAD layer, CoACD→MJCF, Hydra wiring, CI on three designs under five
  minutes, and the gate's actual claim — *two machines, two days apart*, which nothing has
  tested. `docs/plan/11-phases.md`.

---

## Closed

Kept so the numbering stays unambiguous. The evidence for each is in
`docs/experiments/log.md` under the date given.

| # | Item | Log entry |
|---|---|---|
| 1 | Set up the `conda3.12` environment | 2026-08-05 — CAD stage executed against a real kernel |
| 2 | Verify and fix the CAD stage against a real kernel | 2026-08-05 — same entry |
| 3 | Build the CalculiX FEA driver (first-week step 3) | 2026-08-06 — driver stands up; five silent bugs |
| 4 | Fill `configs/robot.yaml` with provisional values | 2026-08-07 — the platform was wrong |
| 5 | Re-run FEA sweeps at the re-specified platform; update `04-design-space` §`T3b` | 2026-08-07 — re-measuring at the new platform |
| 6 | Make `configs/robot.yaml` authoritative: loader plus agreement tests | 2026-08-07 — same entry |
| 7 | Decide how the nominal design gets an FEA evaluation at ~20 h/sweep | 2026-08-07 — same entry |
| 8 | Build the 2-D plane-strain FEA tier | 2026-08-08 — the 2-D tier works |
| 9 | Try the one-element-thick 3-D slab as the cheap tier | **no entry — see below** |
| 10 | Make the 2-D tier handle friction, or refuse it | 2026-08-08 — the 2-D tier works |
| 11 | Calibrate the 2-D tier on more than one design | 2026-08-08 — calibration across topologies |
| 13 | First-week step 4: the segmented-ring ROM fitted to `k_r(δ)` | 2026-08-08 — same entry |
| 14 | Add neighbour coupling to the ring, and re-fit | 2026-08-08 — the shear band |
| 15 | First-week steps 5–6: drive the ring at a 50 mm step and judge it | 2026-08-08 — steps 5 and 6 |
| 16 | Replace the cubic spring law with a tabulated one | 2026-08-08 — it was never the cubic |
| 17 | Make `detect_buckling` test a magnitude, not a sign | 2026-08-08 — same entry |
| 18 | Per-claw FEA sector, and a ROM whose segments are the claws | 2026-08-08 — segments are claws |
| 25 | `scripts/explore.py`, the manual playground and its HTML report | 2026-08-08 — a manual playground |
| 24 | Separate tip slip from structural response in the claw curve | 2026-08-08 — stick or slip |
| 26 | Settle how the ring resolves a frictionless contact force | 2026-08-09 — MuJoCo settles #26 |
| 27 | Replace the ring's tangential slide with a hinge at the claw root | 2026-08-09 — the root hinge, and the damper |
| 19 | Re-derive the `n_spokes` lower bound for claws | 2026-08-09 — the claw family's two screening gaps |
| 21 | Fix the slenderness proxy for tapered claws | 2026-08-09 — same entry |
| 23 | Drive a softening spring law in MuJoCo deliberately | 2026-08-09 — a softening segment, uneventful |
| 22 | Fix the coupled tabulated fit, which stalls | 2026-08-09 — #22 and #12: two deferred items |
| 12 | Reconsider the default `contact_stiffness_factor` of 20 | 2026-08-09 — same entry |
| 29 | No fit of the driven claw design passes the gate | 2026-08-10 — the law was never the problem |

Several of these closed differently from how their titles read, which is worth knowing before
trusting one:

- **#9 was never run and has no log entry.** It existed only as a workaround for a supposed
  plane-strain limitation that turned out not to exist, so #8 superseded it. C3D15 node
  ordering *was* validated along the way — counter-clockwise bottom triangle gives exactly the
  closed-form `E·ε·A`, flipped gives a nonpositive Jacobian — so the slab is still available
  if the 2-D tier's frictionless restriction ever needs lifting.
- **#16 was titled "tabulated *monotone* law"** and closed by establishing that monotonicity
  was the thing to remove.
- **#19 closed by moving the bound the other way.** It expected to widen `n_spokes` downward
  for claws and the measurement said a passive claw wheel wants **more** tips, not fewer —
  twelve, against the six already in the bounds. `PARAM_BOUNDS` is therefore unchanged and the
  claw-specific limit is a warning that fires only without a band.
- **#21 fixed the proxy and left the threshold**, which is now #28.
- **#22 was a cost fault, not an accuracy one.** The projection *was* the culprit, exactly as
  its suspect list said, and the free-block step cut it from 400 iterations and 4004 residual
  evaluations to **4 and 37** with an honest `converged=True`. But the error moved 14.55% →
  **14.54%**. The 8.32% it was framed against belongs to a *bandless* fit, which is a different
  problem, not the same one solved better. The other two suspects — finite-difference noise and
  too many parameters — are refuted.
- **#12 changed two things, and the second was not the one it was filed for.** The default
  factor moved 20 → 5 as expected (0.7–0.8% of the answer on the 3-D tier, 3 cutbacks to 0).
  The cap it asked about "for consideration" turned out to be **load-bearing**: the factor
  alone cannot make a fine mesh converge — at 2.5 and 1.5 mm both 20 and 5 diverge — and it is
  the absolute penalty that decides, so `contact_length_floor_m` is a fix and not a tidy-up.
- **#23 closed by not reproducing.** It was filed expecting dynamic snap-through and energy
  growth; a softening segment runs cleanly at every severity tested, because the payload is a
  dead weight rather than a prescribed displacement and there is always a branch to land on.
  What it *did* find is that the loss-factor damping has no defined stiffness to read on a
  softening branch, worth ~8% of cost of transport, and that its own proposed remedy — the
  minimum tangent — is negative and would inject energy.
- **#27 was right about the element and wrong about what it would fix.** The hinge is the
  correct element and the FEA says so directly — the claw's tip comes *inward* as it bends,
  by +19.7 mm measured against the hinge's +22.6 mm prediction and the slide's −13.9 mm. But
  the driven wheel's collapse was **not** the slide's kinematics. It was the loss-factor
  damping, integrated explicitly, on a joint whose effective inertia is 120x below the segment
  mass it had been scaled by. Moving the same damping into the joints' native `damping`
  attribute fixed it, and the *slide* rig runs now too.
- **#25 found a bug in its first hour that no existing test could see.** The rigid
  comparator's inertia sat exactly on MuJoCo's triangle-inequality boundary, so whether the
  model loaded depended on decimal rounding — rejected in 3 of 54 geometries, and never on
  `--tiny`. That is the case for building the tool before more physics, and it made it
  itself.
- **#18 closed with its machinery working and its purpose blocked.** The claw sector meshes
  and solves — 492 elements against 3155, 0.2 s against 41.7 s, agreeing with the whole wheel
  to 0.07% — and `ring_from_claw_curve` builds a ring from the curve with no fit in it. But
  the curve was not yet a spring law. **#24 closed that**: the radial curve is well determined
  on the stick branch, insensitive to friction above mu=0.2 and mesh-converged to under 1%.
- **#4 and #6 both concern `configs/robot.yaml` and neither froze it.** `meta.frozen` is
  still `false`; see the standing gaps above.
- **#29 closed by refuting its own diagnosis, and it moved the headline.** It was filed as
  ill-posed deconvolution on a bandless wheel; below second-claw engagement there is no
  deconvolution at all, because the whole wheel is one claw to 0.036%. The real cause of the
  sub-engagement failure was a default of 3 table intervals where 4 passes. And the claw
  design's climb is **30 mm against the rigid wheel's 20**, not the 60 recorded on 2026-08-09
  — the difference is the segment law, and nothing else. Continued as #31.
- **#26 closed against the incumbent, and the correction is small on today's designs.**
  MuJoCo matched `f_r/cos θ` to 6e-11 and `f_r·cos θ` to 25%, so the ring was wrong — but the
  tiny design's fitted `a` moves only 3.6% at 24 segments and under 0.3% at 36 and 48, because
  its patch is three segments wide. The correction scales with how far the patch spreads
  (14.1% at ±30°), so it matters for claws and barely at all for `T3`. Do not cite it as a
  reason previous `T3` numbers were wrong.
