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

Every segment in `src/wheelopt/rom/ring.py` and its MJCF realisation has a **radial slide
only**. For a claw that is a real omission: a cantilever claw bends *backwards* under drive
torque, and that backward bend is much of what makes a claw catch and grip a step edge.
`docs/plan/06-compliance-rom.md` §2 already lists torsional stiffness `k_θ` as a ROM parameter
to extract.

**Scoped 2026-08-08, and it is bigger than it reads.** A claw points radially, so a radial tip
load compresses it as a *column* and a tangential one bends it as a *cantilever*. At the
nominal claw, **measured** with the contact-free `TIP_RADIAL` / `TIP_TANGENTIAL` cases:
**24.81 N/mm against 0.1851 N/mm — a factor of 134**. The ring models the stiff one and omits
the soft one entirely, so for a claw this is not a refinement, it is the missing dominant
compliance.

**The FEA half and the analytic ROM half are done** (2026-08-08/09). `solve_equilibrium_2dof`
gives a bandless ring a tangential freedom, solved per segment by bisection. Measured: exactly
inert until a second claw engages at `R(1 − cos π/n)` — 11.4 mm for twelve claws — then 3%
softer at 12 mm and 17% at 25 mm. So the flat-plate fit **at design load is unaffected** (24.5 N
is δ ≈ 1 mm) and the whole benefit is at a step.

The `ROM_VERSION` bump landed with #26 (`rom-0.4.0`), and the five step-climb signatures were
re-run across the re-fit: still **5/5** on `--tiny`. #26 no longer blocks anything here.

What remains is the **MJCF joint**, so MuJoCo has the tangential freedom too. Today the
analytic ring can splay and the simulated one cannot, which means the two now disagree
*deliberately* above 11.4 mm — and the step-climb rig is the simulated one, so the 5/5 above
was judged without any of the splay this issue is about. Until the joint exists, the tangential
freedom changes no result the project actually reports.

It is also loaded exactly where it hurts. Flat rolling presses a claw radially, into the stiff
mode, and the ring is fine; a step edge and drive torque load it tangentially, which the ring
cannot move in at all. Obstacle traversal is the whole point of the project.

The spike's 50 mm-versus-20 mm headline was measured on a **banded `T3`** design, where the
band carries tangential load between segments — that result stands, and it does **not**
transfer to claws. Any claw climb number from the current ROM is a lower bound on compliance,
so probably pessimistic.

Work: a tangential degree of freedom per segment (a second slide, or a hinge at the root),
stiffness fitted to a tangential tip-load FEA case; in MJCF, one more joint per segment plus
its force law. Check whether it moves the step-climb signatures in `scripts/run_step.py` — the
climb result is the most likely to change, since a claw that cannot bend backwards cannot
hook. Bump `ROM_VERSION`.

---

## The claw family's own gaps

These arrived with the `T7` redirection on 2026-08-08 and are listed in that log entry.

### #19 — Re-derive the `n_spokes` lower bound for claws

`PARAM_BOUNDS["n_spokes"] = (6, 36)` in `src/wheelopt/cad/params.py` rejects the four-claw
design in Table I of the PaTS-Wheel letter (`docs/papers`). That bound was set for a banded
wheel, where many thin spokes are cheap and the band carries load between them; for `T7`,
fewer, longer and thicker claws are the point of the family.

**Do not simply widen it.** Derive it from the claw load case: how few claws can carry 24.5 N
at an acceptable tip deflection *and* an acceptable gap between successive tip contacts —
contact is discrete without a band, so ride harshness grows as `n` falls and
`spoke_phase_deg` becomes decisive.

Now unblocked: #24 established that the claw's radial curve is well determined on the stick
branch, so a claw count can be derived from it.

### #21 — Fix the slenderness proxy for tapered claws

`src/wheelopt/cad/constraints.py` computes `slenderness = spoke_span_mm / spoke_thickness_mm`.
For a tapered claw that reads the **root**, which is the stiffest section, so the proxy
understates slenderness and errs toward *accepting* a claw that buckles — the non-conservative
direction.

Left deliberately unfixed, with a comment in the code, when `claw_taper_ratio` was added:
the correct effective section for a tapered cantilever is not obviously the root, the tip or
the mean, and choosing one inside a millisecond pre-filter would be inventing buckling
physics. Resolve it properly — either from tapered-column buckling theory, or by calibrating
the proxy against FEA `detect_buckling` over a taper sweep, with the calibration recorded in
the log. The screening threshold is currently `slenderness > 40`.

---

## Deferred

Real, understood, and not on the critical path.

### #22 — The coupled tabulated fit stalls

`fit_tabulated_law` on a *banded* spec reports 15.57% RMS with `converged=False` on the
nominal at 24 segments, where the same fit uncoupled reaches 8.32%. The path is
`_levenberg_marquardt` with a projection onto non-negative parameters, seeded from the
uncoupled NNLS answer, with finite-difference Jacobians taken through `solve_equilibrium`'s
active-set plus Newton loop.

Suspects, in order: the projection fighting the damping loop (a clamped trial is evaluated,
but the gradient still points into the infeasible region, so damping ratchets up and the
search stalls); the piecewise-constant tangent making the inner Newton semismooth, so its
output is not smooth in the parameters and the finite differences are noise; eight parameters
simply being too many for a finite-difference Gauss-Newton at this cost. Analytic
sensitivities are available in principle — the equilibrium is differentiable in the parameters
away from active-set changes, by implicit differentiation of the KKT system.

**Low priority while every new design is bandless.** `T3` banded is the comparator, not the
target.

### #23 — Drive a softening spring law in MuJoCo deliberately

`TabulatedLaw` can now represent a segment with a negative tangent, and
`src/wheelopt/sim/step_climb.py` applies the law through `qfrc_applied` — so a segment on a
softening branch will **snap through dynamically** rather than settle. That is physically
right and has never been run: `scripts/run_step.py --tiny --law table` passes 5/5 with a law
that does not soften, so it does not test this and does not claim to.

Build the case on purpose: take the tiny design's fitted table, hand-edit one interval to a
negative slope that keeps the accumulated force non-negative, run the step rig. Watch for
energy growth, whether `RigSpec.timestep_s` has to fall, and whether
`segment_damping_n_s_per_m` is enough — it reads the tangent at `u = 0` and is therefore
*unaffected* by a softening branch further out, which may be exactly the wrong place to read
it. If so, derive damping from the minimum tangent instead of the initial one.

### #12 — Reconsider the default `contact_stiffness_factor` of 20

Measured 2026-08-08 on the plane-strain tier: dropping the factor from 20 to 5 to 2 moves the
answer by **1.3%** (3.90 / 3.88 / 3.86 N) while turning a diverged frictional run into a
converged one. If the result is that insensitive across a tenfold range, 20 buys nothing and
costs conditioning.

Check the same sensitivity on the 3-D tier before changing the default — the factor is in the
cache key, so changing it invalidates every cached result. Consider also whether the
`factor × E / element_size` scaling should be capped, since it grows without bound as the mesh
refines.

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
- **Phase 0 is not finished.** DuckDB store, Hydra wiring, CoACD→MJCF, scenario S1, CI on
  three designs under five minutes, and the determinism gate are all untouched — the ROM
  feasibility spike consumed the attention. `docs/plan/11-phases.md`.

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

Three of these closed differently from how their titles read, which is worth knowing before
trusting one:

- **#9 was never run and has no log entry.** It existed only as a workaround for a supposed
  plane-strain limitation that turned out not to exist, so #8 superseded it. C3D15 node
  ordering *was* validated along the way — counter-clockwise bottom triangle gives exactly the
  closed-form `E·ε·A`, flipped gives a nonpositive Jacobian — so the slab is still available
  if the 2-D tier's frictionless restriction ever needs lifting.
- **#16 was titled "tabulated *monotone* law"** and closed by establishing that monotonicity
  was the thing to remove.
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
- **#26 closed against the incumbent, and the correction is small on today's designs.**
  MuJoCo matched `f_r/cos θ` to 6e-11 and `f_r·cos θ` to 25%, so the ring was wrong — but the
  tiny design's fitted `a` moves only 3.6% at 24 segments and under 0.3% at 36 and 48, because
  its patch is three segments wide. The correction scales with how far the patch spreads
  (14.1% at ±30°), so it matters for claws and barely at all for `T3`. Do not cite it as a
  reason previous `T3` numbers were wrong.
