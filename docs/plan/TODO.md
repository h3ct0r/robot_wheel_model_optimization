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

0. **The climb height is worth one significant figure and the sweep knows it.** 10 mm buckets,
   and a **1%** change in the fitted law moves the answer a full bucket — measured across #12's
   contact floor, which shifts the law 0.3% in peak force and gives 60 mm with it and 50 mm
   without. Do not rank two designs on one bucket. Either drive the resolution from a
   sensitivity measurement rather than a round number, or report the climb as the profile
   (which heights cleared) instead of its maximum; the profile is what distinguishes a climb
   from the bounce the predicate admits.
1. **14% of loaded samples run past the fitted range** (peak 8.60 mm against a 6 mm fit). The
   climb number is therefore partly an extrapolation of the radial table. Widen the
   `RADIAL_FLAT` sweep for this design and re-run; if the number moves, the current one was
   the extrapolation talking.
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

### #28 — The slenderness threshold of 40 is far too permissive for a claw

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

*Empty as of 2026-08-09 — #22 and #12 both closed. Kept as a heading because the distinction
it draws is a real one and the list will fill again.*

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
| 27 | Replace the ring's tangential slide with a hinge at the claw root | 2026-08-09 — the root hinge, and the damper |
| 19 | Re-derive the `n_spokes` lower bound for claws | 2026-08-09 — the claw family's two screening gaps |
| 21 | Fix the slenderness proxy for tapered claws | 2026-08-09 — same entry |
| 23 | Drive a softening spring law in MuJoCo deliberately | 2026-08-09 — a softening segment, uneventful |
| 22 | Fix the coupled tabulated fit, which stalls | 2026-08-09 — #22 and #12: two deferred items |
| 12 | Reconsider the default `contact_stiffness_factor` of 20 | 2026-08-09 — same entry |

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
- **#26 closed against the incumbent, and the correction is small on today's designs.**
  MuJoCo matched `f_r/cos θ` to 6e-11 and `f_r·cos θ` to 25%, so the ring was wrong — but the
  tiny design's fitted `a` moves only 3.6% at 24 segments and under 0.3% at 36 and 48, because
  its patch is three segments wide. The correction scales with how far the patch spreads
  (14.1% at ±30°), so it matters for claws and barely at all for `T3`. Do not cite it as a
  reason previous `T3` numbers were wrong.
