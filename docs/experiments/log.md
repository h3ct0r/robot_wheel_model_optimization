# Experiment log

Append-only. Newest entries at the bottom.

**Write the entry — hypothesis and setup — *before* running.** Filling in the result
afterwards is what makes this a log rather than a changelog.

Claude has no memory of previous sessions. This file is the only record of what has been
tried, so an experiment that isn't logged here effectively didn't happen.

---

## Template

```markdown
## YYYY-MM-DD — <short title>

**Phase:** <0-5>
**Status:** planned | running | complete | abandoned
**Config:** <path or git hash>
**Run ID:** <id>

**Hypothesis.** What is expected and why.

**Setup.** What was run: designs, scenarios, seeds, budget, versions
(CAD / FEA / ROM / sim / optimiser).

**Result.** What actually happened. Numbers, not impressions.

**Interpretation.** What it means. What it rules in or out.

**Follow-up.** What to do next, or what this blocks/unblocks.
```

---

## 2026-08-04 — Project initialised

**Phase:** 0
**Status:** complete

**Setup.** Repository scaffolded. Plan split into `docs/plan/`, seven ADRs recorded in
`docs/decisions/` capturing decisions made during planning (simulator choice, ROM over FEM,
build123d, Chrono ground truth, CalculiX, multi-objective, CoACD).

**Result.** Documentation-only repo. No code, no experiments yet.

**Follow-up.** The one-week ROM feasibility spike in `docs/plan/16-first-week.md`. This is the
next action and it gates everything else — it tests whether an FEA-calibrated segmented-ring
model reproduces compliant-wheel behaviour well enough to optimise against.

---

## 2026-08-04 — CAD stage: compliant-spoke generator (first-week step 2)

**Phase:** 0
**Status:** complete, pending CAD-kernel verification
**Config:** `WheelParams` defaults — R=70, W=40, rim=3.0, hub=25, 16 spokes, t=2.0 mm,
kappa=0.004 /mm, CURVED, TPU_95A @ 40% infill, 3 walls

**Hypothesis.** A parametric `T3` compliant-spoke wheel can be generated, screened and
exported with mass properties derived from geometry, fast enough to sit in an optimisation
loop.

**Setup.** Six modules under `src/wheelopt/cad/`. Deliberate split: all geometry *maths*
(`centreline.py`) and mass properties (`massprops.py`) are pure numpy, so they are testable
without OCCT; `compliant_spoke.py` is a thin build123d adapter that only turns point arrays
into faces. 93 unit tests, all passing in 0.14 s.

**Result.**

- Mass properties validated against closed-form solids: box exact to 1e-12; cylinder polar
  and transverse moments to 4 decimal places at 2000 segments; annulus likewise. Scaling
  law `Izz ~ m R^2` confirmed (radius doubling gives 16.000x).
- Screening runs in well under 1 ms/design and never raises.
- **Two real bugs caught by tests, both silent-failure class:**
  1. `to_si()` converted `spoke_curvature_1_per_mm` as a *length* because the field name
     ends in `_mm` — curvature came out 10^6 times too small. Fixed by testing the
     `_1_per_mm` suffix first.
  2. `check_design` raised `ZeroDivisionError` on `n_spokes=0`, violating invariant 3.
     Geometry-dependent checks are now gated behind "no degenerate violations found".
- **One design error corrected:** the analytic inter-spoke gap estimate (arc pitch at the
  hub) was assumed permissive, but it over-reports the gap at low spoke counts and
  *under*-reports at high ones — so it silently rejected feasible designs above ~n=32.
  Removed entirely; the exact geometric check costs ~100 us, so the approximation bought
  nothing and created a second source of truth. Regression test guards its return.

**Interpretation.** The CAD stage is sound where it is tested. The build123d layer is
**unverified** — the development sandbox had no package-index access, so OCCT could not be
installed and no solid has ever actually been built. `scripts/verify_cad.py` exists to
close that gap: 25 checks covering solid validity, watertightness, tessellation
convergence, determinism, monotonic mass responses and export round-trip.

The two bugs found are worth noting as a pattern: both would have produced *plausible*
wrong numbers rather than crashes. That is the failure mode this project is most exposed
to, and it argues for keeping the pure-numpy/OCCT split as the pipeline grows.

**Follow-up.**

1. Run `python scripts/verify_cad.py` on a machine with build123d. Nothing downstream
   should be trusted until it passes.
2. Fill in `configs/robot.yaml` (first-week step 1) — `PlatformLimits` defaults are
   placeholders and a campaign run against them is not a valid campaign.
3. First-week step 3: CalculiX radial compression against a flat plate and a step edge.

---

<!-- New entries below this line -->

## 2026-08-05 — CAD stage executed against a real kernel for the first time

**Status:** complete. `scripts/verify_cad.py` exits 0, 37/37 checks.

**Hypothesis.** The build123d layer, written blind in a sandbox without OCCT, would either
fail outright on API misuse or work. Prior guesses at what would break — the `both=True`
extrude convention halving the width, the 16-spoke fuse returning a Compound rather than a
solid, the 16 × `make_face()` loop being slow or fragile — all turned out to be **wrong**.
The geometry code was correct as written. Everything that was actually broken was in the
*mesh extraction*, which nobody had reason to suspect.

**Config.** conda env `conda3.12`: Python 3.12.13, build123d 0.11.1, cadquery-ocp-novtk
7.9.3.1.1, numpy 2.5.1, CalculiX 2.23 (conda-forge, osx-arm64), gmsh 4.15.2. macOS arm64.
Nominal design unchanged: R=70, W=40, 16 spokes, t=2.0, kappa=0.004, CURVED, TPU_95A @ 40%.

**Result — the geometry is right.** Nominal wheel builds in 3.4 s. Single solid. Bounding
box exactly 140 × 140 × 40 mm, centred on z=0, so `both=True` does extrude `amount` in each
direction. BREP volume 182.87 cm³, mass 210.1 g. All three spoke profiles build; the tread
cutter — which had *no* coverage at all, since `tread_depth_mm` defaults to 0 — builds and
removes 11.2 cm³ without changing the envelope. Euler characteristic of the welded mesh is
V−E+F = 3416−10344+6896 = −32, i.e. **genus 17**: sixteen inter-spoke windows plus the
bore. The topology is exactly what the design intends.

**Three real defects, all in mesh extraction, all silent.**

1. **The tessellation was never watertight.** OCCT triangulates each BREP face
   independently, so vertices on a shared edge are emitted once per adjacent face: 63% of
   the 9332 raw vertices were duplicates and *not one edge* was shared. `is_watertight`
   was correct to reject it. Fixed with `export.weld_vertices()`, now called from
   `tessellate()`. Mass properties were unaffected (the divergence-theorem integral is
   per-triangle and never consults connectivity), which is exactly why this could sit
   undetected behind passing numbers.

2. **`tolerance_mm` did nothing.** build123d calls `BRepMesh_IncrementalMesh` with
   `isRelative=True`, so the linear deflection is scaled by feature size — on this geometry
   0.4 mm and 0.025 mm returned the *identical* 6896-triangle mesh. Compounding it,
   `Shape.mesh()` triangulates only "if none exists", and OCCT's boolean operations leave a
   triangulation behind at a deflection of their own choosing. The mesh being measured was
   a by-product of the union, not anything the caller asked for.

3. **STL export was order-dependent.** Same cause. The exported collision mesh varied with
   whether anything had tessellated the part earlier in the process, and at what tolerance:
   a fresh part gave 6896 triangles at every setting, but after a coarse `tessellate(0.4)`
   the same call gave 5608. Same design, different collision mesh, decided by call order —
   which would have quietly broken the Phase 0 determinism gate.

   Fixed by `export.remesh()`: clean the cached triangulation, then mesh with
   `isRelative=False`. Both `tessellate()` and `export()` now route through it. STL triangle
   counts are now identical regardless of call history and respond to the requested
   tolerance (5608 / 6896 / 12536 at 0.4 / 0.1 / 0.025).

**Consequence for tolerances: the angular tolerance dominates, not the chordal one.** The
volume error lives on the cylindrical surfaces, where facet count tracks angle. Refining
the chordal tolerance alone moves almost nothing; refining both together converges properly:

| tol (mm) / ang (rad) | triangles | volume (cm³) | error vs BREP |
|---|---|---|---|
| 0.4 / 0.4 | 5608 | 181.739 | −0.618% |
| 0.2 / 0.2 | 5944 | 182.502 | −0.201% |
| 0.1 / 0.1 | 6896 | 182.772 | −0.053% |
| 0.05 / 0.05 | 8736 | 182.850 | −0.010% |
| 0.025 / 0.025 | 12536 | 182.865 | −0.003% |

`verify_cad.py` section 8 now sweeps both and asserts the triangle count actually changes —
the old linear-only sweep produced a flat line and passed vacuously.

**One threshold changed deliberately, not patched.** `Izz/(m R²)` measured **0.434**,
just under the old 0.45 floor. This is not a bug: the T3 hub is a *solid* annulus from
r=3 mm to r=25 mm carrying ~42% of the volume close to the axle, which pulls the ratio below
the thin-disc value of 0.5. The bound is now 0.35–1.05, set from the geometry family rather
than the disc formula. If the hub is later webbed to save mass, expect this to rise.

**Interpretation.** The pure-numpy/OCCT split held up: 105 unit tests stayed green
throughout, and every defect was in the thin adapter, which is where the untestable logic
was deliberately concentrated. But the split also hid these — the adapter's *output* was
only ever checked for self-consistency (mesh vs BREP within 1%), and all three defects
preserved self-consistency while corrupting reproducibility. The lesson for the FEA stage:
check that a knob *does something* before trusting a study that varies it. A convergence
study that passes without the mesh changing is worse than no study.

**Follow-up.**

1. First-week step 3: CalculiX radial compression, flat plate and step edge. Environment is
   ready — `ccx -v` reports 2.23.
2. `configs/robot.yaml` still unfrozen; provisional values next, and nothing reads the file
   yet, so `PlatformLimits` remains authoritative in practice.
3. Consider a webbed hub. 210 g on a ~4 kg robot is 5.3% of chassis mass per wheel, and the
   solid hub is the largest single contributor while carrying almost no load.
4. The spoke-to-rim and spoke-to-hub junctions are sharp re-entrant corners. Harmless for
   mass properties, but they will produce mesh-dependent singular stress in FEA — a root
   fillet is likely needed before peak stress means anything.

## 2026-08-06 — CalculiX FEA driver stands up; five silent modelling bugs found

**Status:** complete. `scripts/verify_fea.py --full` passes 20/20 against CalculiX 2.23.
First-week step 3 is done.

**Hypothesis.** A STEP → gmsh → CalculiX → `k_r(δ)` pipeline could be built and made to
produce physically sensible radial-compression behaviour for a compliant wheel against both
a flat plate and a step edge. The interesting question was not whether the code would run —
it was how many of the modelling choices would be silently wrong, since FEA is exactly the
kind of stage where a plausible wrong number survives every check that isn't looking for it.
Answer: five, and every one was silent — no crash, no warning, a believable result.

**Config.** conda `conda3.12`; CalculiX 2.23 (conda-forge, osx-arm64), gmsh 4.15.2,
build123d 0.11.1. Verification design: R=40, W=20, 6 spokes, t=3.0 mm, TPU_95A @ 40%.
Second-order tets (C3D10), displacement-controlled, single `*STEP` sweeping 0 → 3× nominal
→ 0 via `*AMPLITUDE`. The `--tiny` model is ~53k DOF and solves in 2–4 min.

**The result is physics.** Flat-plate sweep: radial stiffness rises 0.9 → 1.3 kN/m
(stiffening, as a compliant wheel must), contact patch grows to 23×14 mm, loaded radius
falls monotonically, and the unloading branch retraces loading to within 1.3% enclosed area
— which is the right answer for a path-independent hyperelastic model. Step edge vs flat at
equal load: **smaller patch (65 vs 168 mm²) and higher peak pressure (1721 vs 1245 kPa)**,
which is the envelopment signature from `docs/plan/06`, §"Sanity checks". The single-element
patch test recovers σ = E·ε to a ratio of 0.99997, which validates the material card syntax,
the coefficient ordering, the SI unit system and the gmsh→Abaqus node ordering at once.

**Five silent bugs, in the order they bit.**

1. **`.dat` parsing dropped the entire load curve.** CalculiX uses *four* different
   leading-column conventions in one file — element output has two id columns (element +
   integration point), nodal output one, `TOTALS=ONLY` force output *zero*, and contact
   output names its id column. Worse, one block is headed `relative contact displacement`,
   and matching `displacement` before `contact` classified it as nodal displacement, where
   it overwrote the reference-node history and `build_load_curve` found nothing. Fixed by
   counting value columns from the header and taking them from the end of the row, and by
   matching contact blocks first. All four variants are now golden-file fixtures copied
   verbatim from real output (`tests/test_fea_parse.py::TestRealBlockVariants`).

2. **`*TIME POINTS` — the last set defined wins for everything.** CalculiX ignores the
   per-request `TIME POINTS=<name>`; a second, sparser set intended only for stress output
   silently thinned the whole load curve from 12 samples to 4. Measured directly (8-point +
   2-point sets → 2 output times for *both* displacement and stress). Now exactly one set.

3. **`D2 = 0` means infinitely stiff, not "off".** The volumetric energy is
   `Σ (1/D_k)(J-1)^{2k}`, so a zero higher-order coefficient is a *reciprocal* of zero.
   CalculiX substitutes ~1e-15 and warns quietly; the effect is 1/D2 ≈ 1e15 Pa and a
   volumetrically locked, far-too-stiff wheel. Fixed by setting D₂, D₃ large (negligible)
   rather than zero. This is the FEA cousin of the `_1_per_mm` bug from 2026-08-05: a
   natural-looking zero that means the opposite of what it reads as.

4. **Peak stress read at the wrong time.** The sweep ends *unloaded*, so reading the last
   stress block reports a wheel carrying nothing (~0 MPa) and every fatigue constraint
   passes. Now taken as the maximum over all sampled times.

5. **Loop area compared branches over different displacement ranges.** The loading and
   unloading branches are not sampled at the same displacements, so integrating each over
   its own range reported a 70% "hysteresis loop" for a curve that retraces perfectly. Now
   both branches are interpolated onto their common range; a retraced curve reports 0.

**Two more that would have killed a campaign, not just skewed it.**

6. **Non-deterministic meshing.** `Mesh.HighOrderOptimize = 2` gave the same STEP different
   node positions run to run (up to 0.13 mm) — which defeats the Phase-0 determinism gate
   and makes the cache key describe a mesh that was not the one solved — *and* intermittently
   aborted the process with an uncatchable C++ exception (`ScaledJac`), which invariant 4
   cannot defend against because it is not a Python exception. Disabled; the cost is slightly
   worse mid-side node placement, which is worth paying for a reproducible, non-fatal mesh.

7. **Curved quadratic tets folded and CalculiX rejected them at t=0.** gmsh curves C3D10
   mid-side nodes onto the bore and spoke fillets, folding elements whose straight-edge
   corner volume is still positive; CalculiX then reports `nonpositive jacobian determinant`
   and diverges before the first increment. Fixed with straight-sided quadratic elements
   (`Mesh.SecondOrderLinear`) plus a quadratic-Jacobian check (`_min_quadratic_jacobian`)
   that samples the shape-function determinant at corners, edge midpoints and centroid and
   raises a typed `MeshFailure` before the solver ever sees the mesh.

**Interpretation.** The pure/impure split paid off exactly as on the CAD stage: 265 unit
tests exercise deck text, parsing, extraction, the cache key and the failure paths without
gmsh or CalculiX installed, and every one of bugs 1–5 became a one-line regression test the
moment it was understood. But the split could not have *found* them — all five were only
visible when real solver output met real geometry, which is the argument for keeping a
`--full` verification tier that actually runs the binary. The recurring shape across this
stage and the CAD stage is a value that is zero, or a default, or a reused mesh, that reads
as innocuous and means something else entirely.

**Honest limitations, recorded so they are not rediscovered as bugs.**
- **No FEA hysteresis loss factor.** A hyperelastic model is path-independent; a real loss
  factor needs `*VISCO` + a Prony series, and `docs/plan/07-materials.md` says the Prony
  data needs DMA that is unavailable. `FeaResult.hysteresis_loss_factor` is `None` in
  `fea-0.1.0` by design, not omission. The 0 → 3× → 0 sweep is still run, as numerical QC
  and to detect buckling via a non-retracing unload.
- **Peak stress at the spoke root is mesh-dependent** (re-entrant corner singularity).
  `p95_von_mises_pa` is the mesh-convergent number a fatigue constraint should use; the raw
  peak is reported but should not be trusted until the CAD geometry gains a root fillet.
- **Hyperelastic coefficients are provisional seeds**, not coupon fits. Because the cache
  key hashes the coefficients themselves, replacing them invalidates prior results
  automatically.
- The material stiffness knock-down uses a Gibson-Ashby `φ^n` law, deliberately *not* the
  linear volume-fraction mix used for density — halving infill roughly halves mass but
  quarters stiffness. Documented in both places so the asymmetry is not "fixed" into a bug.

**Follow-up.**
1. First-week step 4: the 24–48-segment MuJoCo ring, joint stiffness fitted to this
   `k_r(δ)`. The FEA output contract (`LoadCurve`, `ContactPatch`, loaded radius) is what it
   consumes.
2. Add a spoke-root fillet to `cad/compliant_spoke.py` so peak stress becomes meaningful.
   Until then, gate the fatigue constraint on `p95` only.
3. Replace the provisional Mooney-Rivlin seeds with coupon fits per
   `docs/plan/07-materials.md` before any quantitative stiffness claim.
4. Run a mesh-convergence study once for the T3 family (`14-cad-toolchain.md`: once per
   topology family, not per design) to fix the production `MeshSpec`.

---

## 2026-08-07 — Bandless wheel (`T3b`): spoke tips as the running surface

**Phase:** 0
**Status:** complete. CAD and FEA both carry the variant; `verify_cad.py` 18/18 with the new
section 10, unit suite 297/297.
**Config:** conda `conda3.12`; build123d 0.11.1, gmsh 4.15.2, CalculiX 2.23.

**Hypothesis.** Removing the shear band (`rim_thickness_mm = 0`) is a small CAD change that
should make the *first* wheel easier to reason about: one spring in the load path instead of
two in series, so an FEA `k_r(δ)` is a statement about the spokes rather than about spokes
and band jointly. Expected cost: contact stops being axisymmetric. Expected surprise: at
least one place where the pipeline keeps working and quietly answers a different question —
that has been the pattern at every stage so far.

**Setup.** Nominal design R=70, W=40, 16 spokes t=2.0 for the CAD checks. FEA on the `--tiny`
preset (R=40, W=20, 6 spokes t=3.0, TPU_95A @ 40%), flat plate, δ_max = 4 mm, nominal 14 N,
compared like-for-like against the same preset with its usual 3 mm band.

**Result — the wheel is 4.6× stiffer, and then it buckles.**

| `--tiny`, flat plate, δ = 4 mm | banded (3 mm) | bandless, tip phase |
|---|---|---|
| Peak force | 3.78 N | **17.55 N** |
| Buckling | none | **limit point at 16.1 N** |
| Peak contact pressure | ~1.2 MPa | **18.0 MPa** |
| Contact patch length | 23.2 mm | ~0 (line contact, at the mesh's resolution limit) |
| Volume (nominal design) | 182.87 cm³ | 135.03 cm³ |

Not the direction guessed in advance. With a band, load spreads circumferentially and several
spokes bend; without one, the plate lands on a single tip and loads that spoke close to
**axially**, which is far stiffer than bending — until it Euler-buckles, which it does at
16.1 N, below 2.5× nominal, so `fea_buckling` fires as INFEASIBLE. The bandless wheel is not
a softer wheel; it is a strut that holds well and then lets go.

**Interpretation.** Worth keeping as the first article for exactly the reason it was asked
for — one identifiable spring — but it is a different mechanism, not a simplification of the
same one. `04-design-space.md` §`T3b` records the four consequences, of which the one that
matters for the week is that the ring ROM's segment-to-segment coupling *is* the band's
bending stiffness and is now zero: step 4 fits `N_s` independent radial legs, not a ring.

**Three silent defects, all of the house pattern.**

1. **The spoke tip sat outside its own outer radius.** The outline is offset perpendicular
   to the centreline, and a curved centreline is not radial at the tip, so the outboard
   shoulder rode past `outer_radius_mm` — 68 µm at the nominal design, **283 µm** at the
   corner of the design space (S-curve, κ = 0.03, t = 4 mm), and it is the shoulder rather
   than the corner that protrudes furthest. Fatal rather than cosmetic: the FEA tread node
   set is `|r − R| < 0.1 mm`, so the first material to touch the ground would not have been
   in the contact set at all. Invisible with a band, because the band was the running
   surface and the tips were buried inside it. Fixed by clipping the outline radially;
   `tests/test_bandless.py` sweeps profile × radius × curvature × thickness for it.

2. **`--tiny` silently overrode `--rim-thickness 0`.** `apply_tiny` assigned the preset
   unconditionally, so the flag was accepted and discarded, and the run reported the banded
   preset as the bandless design. `apply_tiny` now leaves alone anything the caller moved
   off its default, and prints what it kept.

3. **A load case that never touched the wheel reported `status: ok`.** Phased to the gap
   instead of the tip (−60° for six spokes), the plate descends into a void — the deepest
   material between two tips is 5.4 mm below the running surface, past the 4 mm sweep — and
   the solve converges in 50 increments with no cutbacks, no warning, and `k_r(δ)` identically
   zero. That reads downstream as an infinitely compliant wheel. Now a DEGENERATE
   `fea_no_contact` violation, and it is the *reason* `spoke_phase_deg` and
   `phase_for_tip_contact()` exist rather than a default of 0: with 6 spokes the default
   phase lands in the gap.

**Also changed.** `classify_nodes`/`classify_elements` put the tips in `spokes` rather than
`rim` when there is no band — otherwise the peak-stress output set excludes the highest-stress
elements in the wheel. The gmsh size field no longer applies the coarser rim size to the tips.
`draw_wheel_section` draws the running surface as a dashed circle, since there is no material
on it.

**Follow-up.**
1. Step 4 fits `N_s` legs, not a ring. Decide before fitting whether the ROM validation gate
   is still meaningful in that form, or whether the banded wheel has to be the gate design.
2. 18 MPa of contact pressure on a 3 mm tip is a durability question for printed TPU that
   simulation will not answer. Worth a printed part early.
3. The patch-length extractor reports ~0 mm for line contact. Honest, but resolution-limited
   — the tip carries only a couple of node rows at the `--tiny` mesh size. Re-check on a
   converged mesh before reading anything into it.
4. `k_r` is labelled "(stiffening)" even when both samples are negative, because the label
   compares peak against mid-sweep. Misleading on a post-buckling curve. Not touched here.

---

## 2026-08-07 — The platform was wrong: re-specifying for a 400 × 300 × 200 mm, 10 kg rover

**Phase:** 0
**Status:** complete for CAD; FEA re-baselining in progress.
**Config:** `configs/robot.yaml` rev 2026-08-07.

**What changed and why.** The chassis premise was wrong. Every number in the project had been
derived from a ~4 kg, 300 × 220 × 120 mm robot on 70 mm wheels; the actual target platform is
**400 × 300 × 200 mm**. Given that, and the three choices it does not determine — 10 kg
all-up, four-wheel skid steer, 400 mm in the driving direction — the nominal wheel load goes
**14.0 → 24.5 N**, and the wheel design space moves with it.

This is not a rescaling. `k_r(δ)` is nonlinear and buckling goes as `t³/L²`, so a 75% load
increase does not map onto a 75% thicker spoke, and the previously "nominal" design is not a
small design any more — it is an infeasible one.

**What the load demands.** A closed-form Euler estimate over the space (two loaded spokes,
K = 0.7, effective E from the infill knock-down) against the 2.5× buckling constraint,
61.3 N:

| spoke `t` | 5 mm | 6 mm | 7 mm | 8 mm |
|---|---|---|---|---|
| 2 × P_cr, hub 22 mm | 27.8 N | 42.9 N | **62.3 N** | **86.4 N** |

Nothing below 7 mm survives at the nominal hub radius. The old `spoke_thickness_mm` upper
bound was **4 mm** — the entire feasible region was outside the search space. Thickening also
*lowers* effective E (a thicker feature has a smaller shell fraction, so proportionally more
infill void), which eats part of the gain: E falls 8.99 → 4.02 MPa from t=2 to t=8.

**New nominal design and bounds.** R 85, W 45, 12 spokes at 7 mm, hub 22, 8 mm bore; 364 cm³,
254 g. Bounds: R 60–100 (**set by the 220 mm print bed, not the robot**), W 30–70, `t_spoke`
1.2–8, `t_rim` 1.2–8. `PlatformLimits`, `PARAM_BOUNDS` and `configs/robot.yaml` now agree —
the three-way disagreement that file documented is resolved by hand, though still not by
construction, since nothing loads the YAML yet.

**Three defects the larger wheel exposed.** All three had been latent since the CAD stage was
written, and all three were invisible at 2 mm spokes.

1. **The spokes ran through the shear band and out the other side.** The attachment overlap
   is `0.75 × spoke_thickness`, buried into a rim `rim_thickness` deep. At 2 mm that is
   1.5 mm into a 3 mm rim — exactly half, and fine. At 7 mm it is **5.25 mm into the same
   3 mm rim**, so the spoke tips protruded 2.25 mm past the running surface and the nominal
   wheel measured **175.06 mm across while reporting an 85 mm radius**. Mass, inertia, the
   bounding box and the FEA tread node set were all wrong together. `verify_cad.py`'s
   bounding-box check — added in the 2026-08-05 session precisely because nothing else
   catches a whole-solid scale error — is what caught it. Fixed by
   `centreline.attachment_overlap_mm()`, which caps the overlap at half the member it is
   burying into; at the old 2 mm spoke it returns exactly the old value, so no previously
   verified geometry changed.
2. **A test that compared slicer settings and called it a material comparison.** "PLA is
   heavier than TPU at equal geometry" used the two presets, which differ in infill (25% vs
   40%) as well as base density (1240 vs 1210). On thin spokes shells dominate and base
   density wins; on a 7 mm spoke the infill difference is larger and the sign flips. The
   check had been passing for the wrong reason. Now compares at equal infill and wall count.
3. **gmsh's bare exception escaped `mesh_step`.** A size field coarser than the smallest
   feature — an 18 mm element against a 4 mm bore — makes the surface facets self-intersect,
   and gmsh raises a plain `Exception`. `mesh.py` documents `MeshFailure` as its only failure
   mode and the runner relies on that for invariant 4; instead the whole evaluation crashed.
   Now wrapped, with the element sizes and the bore named in the message, and
   `verify_fea.py` check 3 provokes it deliberately.

**Also moved.** `--tiny` R40/W20 → R60/W30 (the old preset is now out of bounds and would be
rejected before meshing); `hub_bore_radius` 3 → 4 mm for an 8 mm D-shaft; `bed_size` reconciled
to 220 × 220 × 250; `max_material_grams` 200 → 450, which would otherwise reject every design
that survives 24.5 N; print time 10 → 24 h.

**What this invalidates.** Every cached FEA result: `design_hash` changed for the nominal
design and `nominal_load_n` changed for every load case, so nothing in `data/cache/fea` is
reachable. That is the cache working. The qualitative findings survive — stiffening curve,
patch growth, falling loaded radius, retracing unload, the envelopment signature, and the
bandless/banded contrast — but **every absolute number in the 2026-08-06 and the bandless
entries above refers to the superseded R40/W20 preset at 14 N** and must be re-measured
before it is quoted.

**Gates.** `verify_cad.py` 48/48, `verify_fea.py` 11/11 (quick tier), unit suite 297/297.

**Follow-up.**
1. Re-run the flat and step-edge sweeps on the new `--tiny` preset and on the nominal design,
   and replace the superseded numbers in `04-design-space.md` §`T3b`.
2. The buckling estimate above is a straight-column closed form with an assumed end fixity
   and an assumed two loaded spokes. It sized the bounds; it is not evidence. FEA decides.
3. A webbed hub is now clearly worth it: the solid hub is a large share of a 254 g wheel, and
   four of those is 1.0 kg on a 10 kg robot.
4. Freeze the spec, or write the loader. The values agree by hand today and nothing prevents
   them drifting apart again.

## 2026-08-07 — Re-measuring at the new platform: the nominal design does not fit the compute budget

**Hypothesis.** With the platform re-specified, re-run the flat and step-edge sweeps for the
banded and bandless topologies on the new `--tiny` preset and on the nominal design, replace
the superseded numbers in `04-design-space.md` §`T3b`, and close first-week step 1 by making
`configs/robot.yaml` authoritative. Expected: four short sweeps, four long ones, a doc edit
and a loader.

**Config.** CalculiX 2.23, gmsh 4.15.2, TPU 95A at 40% infill / 3 walls, displacement control,
δ swept 0 → δ_max → 0. `--tiny` is R 60 × W 30, 6 spokes, `t_s` 5 mm, hub 20 mm, δ_max 6 mm,
6 points per branch. Nominal is R 85 × W 45, 12 spokes, `t_s` 7 mm, hub 22 mm, δ_max 12 mm,
10 points per branch. Bandless runs use `phase_for_tip_contact()`, i.e. φ = −90°.

### Result 1 — the `--tiny` 2 × 2 completed; §`T3b` now has measured numbers

| | 3 mm shear band | bandless, tip at contact |
|---|---|---|
| Flat, force at δ = 6 mm | 4.36 N | 26.9 N, after a peak of **31.4 N at δ = 3 mm** |
| Flat, buckling | none | **limit point at 31.4 N** |
| Step edge, force at δ = 6 mm | 3.04 N | 18.31 N |
| Step edge, buckling | none | none; `k_r` softens 2.10 → 1.63 kN/m |
| Step edge, unload loop area | 0.29% | 4.19% (QC threshold 5%) |
| Flat, p95 spoke stress | 0.08 MPa | 0.47 MPa |

At equal indentation the bandless wheel is **7.2× stiffer on the flat plate and 6.0× on the
step edge**. The superseded R40/W20 measurement said 4.6×, so the contrast is real and
survived the re-spec. New and load-bearing: the flat case **snaps through at 31.4 N, below
the 61.2 N the buckling constraint demands**, so `fea_buckling` rejects this bandless design
outright. The bandless step edge does not buckle but softens and returns a 4.19% loop area —
just inside the QC threshold, and worth watching rather than trusting.

### Result 2 — the nominal design is a ~20-hour solve, and that is the finding

The nominal flat sweep ran **6 h 08 m and reached step time 0.60 of 2.0** — 16 increments,
~23 min each, 50 779 C3D10 elements, 279 336 DOF. Extrapolated: **~20 h per sweep**, ~3.4 days
for the four planned. Killed rather than finished.

Coarsening does not rescue it. The requested element size is not what sets the mesh — the
geometry is: a 3 mm shear band, a 7 mm spoke and a 4 mm bore all force small elements
regardless. Measured: 0.005/0.006/0.010 m → 50 779 elements; 0.007/0.009/0.012 → 38 277, only
25% fewer; 0.008/0.010/0.014 fails to mesh at all. **The nominal design is inherently
40–50 k quadratic tets.** `12-risks.md` item 2 budgets "10–40 min per sweep"; at the
re-specified platform that is wrong by roughly **30×**, and it got that way for an entirely
mechanical reason — a bigger robot took a bigger wheel, which took a bigger mesh.

This is a Phase-1 problem, not a detail: the FEA stage is offline and cached (invariant 1),
so a handful of 20 h reference runs is affordable, but one *per design* is not, and the ROM
fit needs one per design.

Options, none yet chosen: mid-plane symmetry for the load cases where it is valid (see the
defect below); a shorter δ_max, since the ROM fit may not need the full static-sag range; the
2-D plane-strain slice named as the contingency in the step-3 implementation plan; or
first-order elements,
which is the one to refuse — C3D4 locks under near-incompressibility and would return a
stiffness that is too high and perfectly plausible.

### Defect 1 — `--half-width` was a no-op that only split the cache

Reaching for mid-plane symmetry as the throughput fix is how this surfaced.
`MeshSpec.half_width_symmetry` is declared, documented as "halves the DOF count", exposed as
`run_fea.py --half-width`, and **hashed into the cache key — but never read by `mesh.py` or
`deck.py`.** Setting it produced a full-width model, at full cost, under a different cache
key, and would have been reported in this log as a symmetric run.

`tests/test_fea_cache.py` asserted that flipping it changed the cache key. That was true, and
it is why the no-op survived: splitting the cache was the only thing the flag did, and the
test confirmed it did *something*. Now `MeshSpec.__post_init__` refuses `True` outright, the
CLI reports it as a usage error, and the test asserts the refusal. The field is kept rather
than deleted so existing cache keys stay valid.

Textbook entry for the watch list: a value that is a default, reads as innocuous, and means
something else.

### Defect 2 — the step-edge/flat pressure check compared two different loads

`verify_fea.py` check 5 failed: "step edge gives a higher peak pressure than a flat plate —
864 vs 980 kPa". The physics is not wrong; the check was, twice over.

- **It compared at equal indentation, not equal load.** Both sweeps run to δ = 6 mm, but the
  step edge is the softer indenter — 3.04 N where the flat plate reaches 4.36 N. Comparing
  the last sample of each pits a 3 N patch against a 4.4 N one.
- **It asserted on peak *nodal* pressure.** The slave surface is a node set, so that is the
  load carried by whichever single node is worst placed, and it tracks *how many* nodes are
  in contact rather than how hard they are pressed. On the flat sweep it **falls** from
  1231 kPa to 980 kPa while the load rises from 0.79 N to 4.36 N.

Fixed by adding `ContactPatch.mean_pressure_pa` and `at_force()`, and comparing at the
highest load both sweeps reached: **318 vs 487 mm², 9.6 vs 6.2 kPa at 3.04 N** — smaller patch,
higher pressure, which is the claim. Peak nodal pressure is now printed as a diagnostic and
never asserted on.

`at_force()` clamps outside the sampled range rather than extrapolating, and clamping quietly
is its own trap, so `common_force_n()` returns `None` when two sweeps never overlap in load.
They frequently do not: banded flat covers 0.4–4.4 N while **bandless flat's first contact
sample is already at 13.5 N**, because contact output begins only once nodes touch and a stiff
design can jump from nothing to tens of newtons in one increment. An equal-load comparison of
the two topologies is therefore not available from these sweeps at all, and §`T3b` now says
so instead of quoting a ratio between two states neither solve visited.

### Also — `configs/robot.yaml` is now loaded, not just written

New `wheelopt.platform`: `load_platform()` → `PlatformSpec`, with `platform_limits()`
converting metres to the millimetres `PlatformLimits` screens in, `param_bounds()` for the two
bounds the robot actually pins down (radius and width — the spec has no business claiming
spoke curvature), `require_frozen()`, and `consistency_warnings()` for the cross-checks
between values that are stated independently rather than derived.

Screening still reads the dataclass defaults, deliberately: `cad/constraints.py` promises no
filesystem access and invariant 3 wants an infeasible design to cost milliseconds. What
changed is that `tests/test_platform.py` (33 tests) now asserts the YAML and the defaults
agree — including the metre-to-millimetre conversion, which is where this project would
otherwise put a 0.105 where a 105 belongs. `meta.frozen` stays **false**: the chassis envelope
is a stated requirement, but the mass, motors, battery and inertia are class-typical
estimates, and freezing would assert a confidence nothing has earned.

**Gates.** `verify_cad.py` 48/48, `verify_fea.py --full` **21/21**, unit suite **342/342**,
ruff unchanged at its pre-existing 71.

**Follow-up.**
1. **Decide how the nominal design gets evaluated at all.** Everything downstream — the ROM
   fit, the Phase 1 gate — assumes a per-design FEA that currently costs 20 h. This is the
   blocking question, and it is a budget question, not a bug.
2. Implement mid-plane symmetry properly, or delete the field. It is worth roughly 2× and is
   valid for the flat case; it is *not* valid where lateral buckling matters, which is most
   of what makes these designs interesting.
3. ~~`*EL PRINT, S` on a sparse time grid~~ — **withdrawn, it was already tried.** The comment
   in `deck.py` records the measurement: CalculiX does not honour `TIME POINTS=<name>` per
   output request when more than one set is defined — the last set defined wins for every
   request. A sparse stress grid therefore makes *everything* sparse, including the load
   curve. One set is deliberate, and restricting `*EL PRINT` to `ESPOKES` is the mitigation
   that does work. Reading a follow-up list is not the same as reading the code.
4. Still open from the previous entry: the webbed hub, and the closed-form buckling estimate
   that sized the bounds without being evidence.


## 2026-08-08 — The 2-D plane-strain tier works: 24.5 s against 280 s, and a false negative that nearly buried it

**Hypothesis.** The nominal design costs ~20 h per 3-D sweep. The contingency named in the
implementation plan for first-week step 3 was to drop to a 2-D plane-strain slice of the
cross-section, reusing the deck generator with a different element type. It had never been
run. Build it.

(That contingency lives in the session's implementation plan, not in
`docs/plan/16-first-week.md` — an earlier draft of this entry cited the committed document,
which does not mention it. Worth correcting rather than leaving: a plan quoted from memory
and attributed to a file anyone can open is how a project acquires load-bearing statements
that nothing supports.)

**Result: it works.** `--tiny` banded, flat plate, frictionless, δ 0 → 6 → 0 mm:

| | 3-D, C3D10 | 2-D plane strain, CPE6 |
|---|---|---|
| elements / DOF | 12 168 / 61 611 | 2 636 / 11 886 |
| solver wall time | 272 s | **19 s** |
| peak force at δ = 6 mm | (see the calibration note) | 3.86 N |
| `k_r` mid-sweep → peak | — | 0.64 → 1.45 kN/m, stiffening |
| unload loop area | — | 0.0001% |

**14× faster on the debug preset**, and the saving grows with the model: the 2-D nominal is
20 468 DOF against 279 336 for the 3-D one.

### The methodological failure, which is the more useful result

Between building the tier and reporting it, this session concluded — in an earlier version of
this entry, since deleted — that **CalculiX cannot solve plane-strain elements under
`NLGEOM` at all**, on the strength of a patch-test matrix showing CPE4/CPE6/CPE8/CPE8R all
failing with both linear and hyperelastic materials. That conclusion was wrong, and the whole
apparatus that produced it was one line:

    if echo "$out" | grep -q "no convergence"; then echo "no convergence"

`no convergence` is what CalculiX prints **per Newton iteration** that has not yet converged.
Every successful nonlinear solve prints it, usually many times. The test asked "did this
string ever appear" when the question was "did the step complete", and it returned *failure
for every nonlinear run ever performed*, including the ones that worked perfectly. Re-checked
against `job.sta` reaching the step period, all seven patch tests completed.

The tell was visible and went unread: the failing run reported `largest residual force=
0.000000` and a displacement correction of `3.45e-12`. A model that is not converging does
not have a zero residual. **The evidence contradicting the conclusion was printed in the
output being grepped.**

Worth naming precisely because it is not the usual failure in this log. The recurring one is a
plausible wrong *value* inside the pipeline; this was a wrong *verdict* from a throwaway shell
one-liner outside it, applied to seven experiments at once, and it would have retired a
working option and sent the project down a needless detour. Ad-hoc test harnesses need the
same "what exactly does this assert" scrutiny as the code — more, since nothing reviews them.
A negative result that kills an option deserves a second, differently-shaped check before it
is written down.

### What is actually blocking, and it is friction

The tier converges frictionless and fails with friction, which the false negative had masked
as a general failure:

| `friction_mu` | outcome |
|---|---|
| 0.0 | **ok**, 24.5 s |
| 0.2 | divergence, then "increment size smaller than minimum" |
| 0.8 (the default) | too-slow convergence, same ending |

Both failures return typed results rather than escaping, so invariant 4 holds. But TPU on a
hard surface is a high-friction contact and `friction_mu = 0.8` is the default for good
reason, so a frictionless screening tier is a real fidelity reduction on top of plane strain,
not a free one — tangential traction is part of how a compliant wheel envelops an obstacle.

### Defects found building it

1. **`fuse` silently leaves the section in pieces** — 8 faces in, 8 faces out, no error.
   Meshed, that is a wheel whose spokes are attached to nothing, and it solves happily.
   `fragment` imprints conformally instead; the check is now a union-find over the mesh
   (exactly one connected component), because the face count means nothing either way.
2. **gmsh does not orient triangles consistently** across fragmented faces — both windings in
   one mesh. A clockwise plane-strain triangle expands inside-out and CalculiX stops at t=0
   with "nonpositive jacobian determinant". The area check could not see it: it took `abs()`.
3. **`mesh_step` would label tetrahedra `CPE6`.** `MeshSpec.element_type` returns the plane
   element when `dimension == 2`, and the dry-run path still called the 3-D mesher, so
   `--plane-strain --dry-run` wrote 49 313 C3D10 tets declared as 6-node plane-strain
   triangles. Well-formed deck, real solve, meaningless answer. Both meshers now refuse the
   wrong `dimension`.

### Geometry provenance

`fea/section2d.py` builds the section from `cad/centreline.py` — the same module the 3-D solid
comes from — rather than by sectioning the STEP, so the two are identical by construction.
Independently checked: meshed section area × `width_mm` reproduces the 3-D BREP volume to
**0.05%** (364.7 vs 364.5 cm³ banded, 304.6 vs 304.4 bandless). Two paths sharing no code
below `centreline` agreeing to five parts in ten thousand is the strongest cross-check the CAD
stage has.

### Calibration, at matched settings

Both frictionless, same design, same load case, `verify_fea.py` section 6:

| | 3-D C3D10 | 2-D CPE6 | ratio |
|---|---|---|---|
| DOF | 61 611 | 11 886 | 0.19 |
| solver wall time | 141.6 s | **19.0 s** | 0.13 |
| peak force at δ = 6 mm | 4.29 N | 3.86 N | **0.90** |
| `k_r` at peak | 1.68 kN/m | 1.45 kN/m | **0.86** |
| contact patch length | 34.2 mm | 32.5 mm | 0.95 |

Within 10-14% on stiffness and 5% on patch length, for **7.5× less solver time** on the debug
preset — and the gap widens with size, since the nominal design is 20 468 DOF in 2-D against
279 336 in 3-D. That is a usable screening tier.

**The sign was predicted wrong.** This module's docstring argued from theory that plane strain
over-constrains the free width and would therefore come out *stiffer*. It comes out softer.
The two runs also differ in mesh density — 2.5 mm in the section against 8 mm in the solid,
and a coarse mesh over-stiffens — so part of the gap is convergence rather than
dimensionality, and which dominates is not established. The docstring now says so instead of
predicting. Section 6 asserts the ratio stays inside ±25% rather than assuming it is 1.

**Contact-patch extraction needed a 2-D branch.** Every slave node sits at z = 0, so the 3-D
estimator returned a width of exactly zero and an area of `n_nodes × spacing²` describing
nothing — reported as `32.5 x 0.0 mm at 21.4 kPa`. In plane strain the out-of-plane extent
*is* the section thickness, so area is length × thickness. Now 32.5 × 30.0 mm against the
3-D 34.2 × 30.0.

**Gates.** Unit suite 366/366 (24 new in `test_section2d.py`), ruff at the standing 71,
`verify_cad.py` 48/48, `verify_fea.py --full` **28/28** (7 new in section 6) on a cache fully
invalidated by the new `MeshSpec.dimension` field — and the re-solve reproduced the previous
numbers exactly, which re-confirms determinism across a cache-key change.

### Friction: solved, and it exposed a cache-key bug

The tier's frictional divergence was **conditioning, not physics**. Contact stiffness is
`factor x E / element_size`, so a finer mesh gets a *stiffer* penalty: the section's 1.6 mm
elements give five times the contact stiffness of the solid's 8 mm ones, and the tangential
problem stops conditioning. Softening it fixes it:

| `contact_stiffness_factor` | mu = 0.8 | mu = 0 |
|---|---|---|
| 20 (default) | diverged | ok, 3.86 N |
| 5 | **ok, 3.90 N** | ok, 3.86 N |
| 2 | **ok, 3.88 N** | ok, 3.85 N |

The answer moves 1.3% across a tenfold change in penalty stiffness, so the penalty is not
polluting the result — it was only conditioning. Friction itself is worth ~1% here, which is
expected for normal indentation against a flat plate with little sliding. `--contact-stiffness`
is now a CLI flag and `verify_fea.py` section 6 asserts the frictional 2-D case converges.

**The bug this uncovered.** The first run of that table read "factor 5: ok, 48 s" then
"factor 2: ok, **1 s**" — a cache hit. `SolverSpec` was **not in the cache key at all**.
`cache.py` justified it: *"Deliberately not in the key: output directory, thread count,
timeout. Those change how long the answer takes, not what it is."* True for those three, and
false for everything else in the dataclass — `contact_stiffness_factor` is the contact
compliance, and the increment controls decide whether a limit point is found and whether the
run completes at all. Three different contact models hashed identically, so the second
configuration was served the first one's answer and *looked like confirmation that both
converged*. It had never run.

Fixed by hashing all of `SolverSpec` except a named `SOLVER_TIMING_ONLY = {timeout_s,
n_threads}` — an exclusion list rather than an inclusion list, so a field added later is
hashed by default and the failure mode of forgetting is a redundant re-solve rather than a
shared entry. `test_fea_cache.py` gains three tests, including one that enumerates
`SolverSpec`'s fields and asserts every unexcluded one moves the key.

Same shape as the `--half-width` no-op from the previous entry, and the same shape as the
false negative above: **the check that was supposed to catch it was the thing that was
broken.** Invariant 5 has had a regression test since the FEA driver was written; it passed
throughout, because it only ever tested the arguments the key already took.

**Follow-up.**
1. **Calibrate on more than one design.** One ratio on one geometry is a data point, not a
   calibration curve. It needs at least the bandless topology and the step edge before the
   ROM is fitted against 2-D output.
2. Consider whether `contact_stiffness_factor = 20` is the right default at all, given a
   tenfold reduction barely moves the answer and materially improves conditioning.
3. Unchanged: the 20 h/sweep problem for the *3-D* nominal, and whether the ROM can be fitted
   from a screening tier at all.

## 2026-08-08 — Calibration across topologies, and first-week step 4: the ring fits

### The 2-D tier calibrates, but not to one number

The frictionless 2×2 could not be read: 3-D bandless flat *diverged*, and bandless step_edge
snap-through at 3.3 N where the same case at `mu = 0.8` carried 18.3 N. For a wheel whose
spoke tips **are** the running surface, friction is load-bearing — without it the tips slide
out from under the load. Matching settings was the right instinct for isolating dimensionality
and the wrong choice for this topology. Re-run at `mu = 0.8` on both tiers, all eight
converge:

| design | case | F 3-D | F 2-D | F ratio | k ratio | buckle 3-D | buckle 2-D | 3-D time | 2-D time |
|---|---|---|---|---|---|---|---|---|---|
| banded | flat | 4.36 | 3.90 | **0.89** | 0.86 | none | none | — | — |
| banded | step_edge | 3.04 | 2.90 | **0.95** | 0.91 | none | none | — | 45 s |
| bandless | flat | 31.38 | 35.22 | **1.12** | 1.30 | 31.4 N | 33.8 N | 269 s | 25 s |
| bandless | step_edge | 18.31 | 21.16 | **1.16** | 0.96 | none | none | 229 s | 18 s |

**The offset is topology-dependent and changes sign.** Banded: 0.89-0.95, the 2-D tier softer.
Bandless: 1.12-1.16, the 2-D tier *stiffer*. There is no single constant to divide out, so the
tier has to be calibrated per topology family — which is workable for screening, and would not
have been discovered by measuring one geometry and generalising.

The best result here is one the earlier frictionless run could not show: **buckling agrees to
7.6%** — 31.4 N against 33.8 N on bandless flat. The cheap tier finds the snap-through the
expensive one finds, which is the property that matters most for a screening tier, since
buckling is a hard constraint. Speedups on the cases that actually needed solving: **10.8×**
and **12.7×**.

### Step 4: the ring reproduces the FEA to 0.87%

`wheelopt.rom`, split the way the CAD and FEA layers are: `ring.py` (the analytic
load-deflection response) and `fit.py` (the fit) are pure numpy and test without a simulator;
`mjcf.py` builds the MuJoCo model. `scripts/run_rom.py` runs the three of them against each
other.

Fitting the `--tiny` banded flat curve, per segment count:

| N | RMS error | segments in contact at peak |
|---|---|---|
| 12 | 1.33% | **1** |
| 24 | 2.91% | 3 |
| 36 | 1.58% | 5 |
| 48 | **0.87%** | 7 |

**Fit error alone is not a discretisation check, and the 12-segment row proves it.** It fits
*better* than 24 segments while a single segment carries the entire 34.2 mm contact patch —
a point load wearing the curve's clothes. The script now flags any ring with fewer than three
segments in contact and excludes it from selection rather than letting it win on error. The
measured patch spans 4.4 segment arcs at 48 segments, which is the number to design the ring
around.

Headline validity number for the paper (`06-compliance-rom.md` §4): **RMS 0.038 N, 0.87% of
peak, at 48 segments.** The MuJoCo realisation of that same ring tracks the analytic one to
2.9-8.7%, read from the *floor contact forces* rather than from the spring law — reading the
law back would have compared the formula against itself and agreed even with the contact
geometry wrong.

### Four defects, three of them mine and one instructive

1. **Capsules laid along the rolling direction.** `fromto` spanned ±15 mm in x, so each
   segment was a 30 mm bar in the direction neighbours are 15.7 mm apart: they overlapped
   permanently and the model reported **−37 N** before touching the floor. The capsule spans
   the wheel's *width*, along y.
2. **The capsule radius added to the wheel radius.** Bodies placed at R put the running
   surface at R + 3.9 mm, so contact began 4 mm early and the ring read five to six times the
   analytic force — which looks exactly like a stiffness error and is a geometry one. Bodies
   now sit at `R − r_capsule`.
3. **Prescribing the hub coordinate every step** teleported the body and the run died with
   "Nan, Inf or huge value in QACC". The hub is now welded into the XML at `R − δ` and the
   model rebuilt per δ; a body with no degrees of freedom cannot be teleported.
4. **A guard that could never fire.** `build_mjcf` raised if the segment capsules did not fit
   inside the wheel — but the body radius is `r(1 − π/2n)`, positive for every `n ≥ 2`, and
   `RingSpec` already refuses fewer than three segments. Dead code shaped like a safety check
   is worse than no check, because it invites trust it cannot repay. Removed, and replaced
   with a test asserting the property actually holds.

Also fixed: `ring_force_n` ended in `return out if np.ndim(delta_m) else out`, the same
expression on both branches, so a scalar argument returned a length-1 array. Harmless under
numpy 1, a `TypeError` under numpy 2 — found by a test, not by the code.

**Segment-to-segment coupling is deliberately absent.** §3 of the plan asks for neighbour
joints fitted to the shear band's bending stiffness, and that term is what makes the ring a
ring rather than N independent legs. Leaving it out lets the radial fit be measured alone; it
is also *correct* for the bandless topology, where the coupling is genuinely zero. Capsules
are set not to collide with each other for the same reason — contact between neighbours would
be coupling arriving from the wrong place, and the fit would absorb it as spoke stiffness.

**Gates.** Unit suite **400/400** (31 new in `test_rom.py`), ruff at the standing 71,
`verify_cad.py` 48/48, `verify_fea.py --full` 30/30.

### Correction: the penetration formula was the small-angle limit

Chasing the MuJoCo gap turned up a real approximation in `ring.py`. A segment slides along
its own radius, so its tip touches the plate when ``u = R - (R - δ)/cos θ``. The model used
``u = δ - R(1 - cos θ)``, which is that expression's small-angle limit — exact at the contact
point, **3.4% low at 15° and 7.6% low at 22.5°**, always under-estimating. The whole ring was
therefore softer than its own spring law implied, which is the right sign and about the right
size to explain the disagreement with MuJoCo.

Fixed, and the gap narrowed as predicted: at 48 segments 2.9/5.2/8.2% became 2.5/4.3/5.5%.
The residual ~5% is the segments being capsules on a scalloped surface rather than points on
a circle, which MuJoCo sees and the analytic model does not. The best fit moves 0.87% → 0.90%.

The exact form needs a gate the approximation did not: it divides by ``cos θ``, so a segment
at 105° — facing away, able to touch nothing — is reported as 268 mm penetrated, that being
the distance to the plate's extension behind the hub. The small-angle version went negative
there and was clipped away by luck rather than by intent.

**Follow-up.**
1. **Neighbour coupling**, and re-fitting with it. Until then the banded ring is missing the
   member that carries load between spokes. Note this is not a parameter to add but a
   modelling step up: with coupling, a segment's displacement is no longer given by geometry,
   because a non-contacting segment can be pulled down by its neighbours. It becomes a
   constrained equilibrium — contacting segments pinned between two rigid bodies at
   ``u = g(δ)``, free segments solving ``f_spring + f_couple = 0``, and the contact set found
   by requiring the contact forces stay non-negative.
2. Steps 5 and 6 of the first week: drive the ring at a 50 mm step beside a rigid wheel, then
   check the four qualitative signatures in `16-first-week.md` — envelopment, larger patch,
   climbs better, rolls worse. That is the decision point the whole spike exists for.
3. The ring is fitted to a flat-plate curve only. The step-edge case is the one that matters
   for climbing and it is not in the fit yet.
4. Unchanged: the 20 h/sweep 3-D nominal, and `contact_stiffness_factor`'s default.

## 2026-08-08 — The shear band: neighbour coupling, and the term that was missing from it

**Hypothesis.** Adding the shear band to the ring will move the fitted spring coefficients
(the springs stop carrying the band's share) and should not make the fit much worse.
`ROM_VERSION` goes to `rom-0.2.0`; invariant 5 requires it, since the fitted coefficients of
any banded wheel change.

### What was built

Two stiffnesses, both derived from the band's geometry and its knocked-down modulus — never
chosen, so invariant 2 holds for them like any other stiffness (`ring_for_design`):

| | energy | source | tiny design, N=48 |
|---|---|---|---|
| bending | `(k_b/2) Σ D_i²` | `k_b = EI·dθ/R³` | 0.334 N/m |
| hoop | `(k_h/2)(Σ u_i)²` | `k_h = 2πEA/(R N²)` | 31.8 N/m |

`D_i` is the discrete `w'' + w` — the classical flexible-ring curvature change, a three-term
window that wraps, so closing the ring is free rather than an equality constraint to enforce.
Its coefficient is `α = 1/(2(1 − cos dθ))`, **not** `1/dθ²`: the naive second difference fails
to annihilate rigid translation by 0.57% at N=24, and a band that stores energy when it merely
shifts sideways is a small wrong number of precisely the kind this project keeps finding.

The equilibrium is now solved, not evaluated. Contacting segments are pinned at `u = g(δ)`,
free segments solve `f_spring(u) + Ku = 0` by Newton, and the contact set is corrected until
every contact force is non-negative and no free segment is inside the plate. Two consequences
worth naming: the spring law needed a **linear tension branch**, because the band pulls the
segments beside the patch outward past `R` and a spoke anchored at both ends resists — and a
limp branch is not merely unphysical, it makes the free block singular. And a **very stiff
band sheds contacts**: it translates rather than flattens, drags its neighbours past what the
plate demands, and the patch *shrinks*. That looks like a bug and is not.

In MuJoCo the band is `N + 1` **fixed tendons** and is exact, not approximated. A fixed tendon
has length `L = Σ coef·q` and stores `(k/2)L²`, which is the same object as both energies:
coefficients from row `i` of the operator for bending, all-ones for the hoop. Verified against
the analytic matrix to 1e-9 on a random asymmetric shape.

### The error the internal checks could not see

Bending alone is not a band, and the miss is not small. Every self-consistent test passed
while it was wrong, because the operator *was* a correct discretisation — of an energy that
was missing a term. It took a comparison against something this repo did not produce: squeeze
a bare ring between two opposite radial point loads and it should deflect `0.1488 F R³/EI`
(Roark; the modal sum reproduces it to four digits). The ring deflected **5.28×** that.

The excess is entirely the `n = 0` breathing mode: `2/π = 0.6366` against the `0.1488` of
every other mode combined, and `1 + 0.6366/0.1488 = 5.279` against 5.279 measured. Cause:
inextensionality is what makes bending the whole story for every `n ≥ 1` mode, and the same
assumption *forbids* `n = 0`, because a periodic tangential displacement cannot accommodate a
change of circumference. Model the bending and stop, and the ring breathes against nothing but
its own bending stiffness — wrong by `12(R/t)²`, about 4600 here.

The hoop term is exact rather than a patch, which is worth stating because a radial-only ring
normally cannot carry a membrane term without locking. Circumference change is `∮w dθ = 2πw̄`:
it depends only on the **mean** radial displacement, and the tangential displacement drops out
because it is periodic. So the hoop energy attaches to the mean alone, leaves every `n ≥ 1`
mode untouched, and there is nothing for the missing tangential freedom to lock against. With
it, the squeeze test matches Roark to 2% and converges with segment count.

### The fit had to change too, and the first attempt diverged

Freezing the compressions makes the reaction linear in `(a, b, c)` again, so alternating a
shape solve with a linear coefficient solve looks right. It diverges. Starting from the
uncoupled fit at N=48: first pass `a = −9.6 N/m`, fourth pass `a = −132000`, residual 350% of
peak. The band force on a contact segment is not a constant to be subtracted — move the
coefficients and the shape moves with them, further than the linearisation predicts. It is a
fixed point of the right problem that is not a contraction.

Replaced with damped Gauss-Newton (Levenberg) on the three coefficients, finite-difference
Jacobians of the full nonlinear model, coefficients rescaled to `x_j = coeff_j·u*^(j+1)` so
all three carry force units instead of spanning seven orders of magnitude. Damping is raised
until a step actually lowers the cost, which rejects steps that walk across a contact-set kink.
Pure numpy, no scipy. Converges in 9 iterations on the tiny design.

Before believing the divergence was the fitter's fault rather than the model's, the model was
checked directly: swept `a` with the band in place and confirmed the coupled `F(δ)` brackets
the FEA curve (`a = 100 N/m` gives 0.54→3.77 N against the measured 0.39→4.35 N). A fit
existed; the optimiser was not finding it.

### Result — `scripts/run_rom.py --tiny`, 3-D flat, µ = 0.8

| N | k_bend | k_hoop | a N/m | RMS % uncoupled | RMS % coupled | in contact |
|---|---|---|---|---|---|---|
| 12 | 1.336 | 508.1 | 458.6 | 1.33 | 1.33 | 1 (point load) |
| 24 | 0.668 | 127.0 | 178.3 | 2.94 | **0.69** | 3 |
| 36 | 0.445 | 56.5 | 129.7 | 1.60 | 1.19 | 3 |
| 48 | 0.334 | 31.8 | 99.3 | 0.90 | 1.22 | 3 |

The band helps the coarse ring a great deal (24 segments: 2.94% → 0.69%) and costs the fine
one a little. That is the expected direction — the band is what makes a coarse ring behave
like a continuous one, and at 48 segments the springs alone were already able to imitate it.

**MuJoCo agreement improved by two orders of magnitude** at low load: 0.03–0.05% against the
analytic ring at δ ≤ 4 mm, where the uncoupled model managed 2.5–4.3%. At δ = 5–6 mm the gap
is 6.1–6.6%, and the cause was measured rather than guessed: MuJoCo has 6 segments touching
where the analytic active set has 3, because the capsules are round and bridge the scallops
between segment centres. That is contact discretisation, not a physics disagreement.

### Negative result worth keeping

**The band model does not shear, and a shear band is named for the deformation it carries.**
Pure bending is stiffer against conforming to the ground than the real band, and the symptom
is measurable: against the FEA's 34.2 mm contact patch (4.4 segment arcs at N=48) the coupled
ring puts only 3 segments on the plate where the geometry alone would put 7. Patch length out
of this ROM is a **lower bound** until a shear term exists. It did not stop the load curve
fitting to 0.69%, which is its own warning — `F(δ)` can be right while the patch is not.

**Gates.** Unit suite **440/440** (71 in `test_rom.py`), ruff at the standing 71, `verify_cad.py`
48/48, `verify_fea.py --full` 30/30.

**Follow-up.**
1. Steps 5 and 6 of the first week, unchanged and now unblocked: drive the ring at a 50 mm
   step beside a rigid wheel and check the four signatures in `16-first-week.md`.
2. A shear term for the band, if the patch length turns out to matter for the step case. Do
   not add it speculatively — measure first whether the step-edge fit needs it.
3. The ring is still fitted to a flat-plate curve only.
4. Unchanged: the 20 h/sweep 3-D nominal, and `contact_stiffness_factor`'s default.

## 2026-08-08 — First-week steps 5 and 6: the ring at a step, beside a rigid wheel

**This is the decision point the spike exists for.** `16-first-week.md`: if it looks like
physics, commit to `11-phases.md`; if not, a week has been spent instead of six months.

**Hypothesis, written before the runs.** A compliant wheel of the same radius and the same
mass should envelop the step edge, carry a longer contact patch, climb higher, roll less
efficiently, and sit lower as load rises. Any of those coming out the other way is either the
ROM or the rig, and must be chased before it is believed.

### The rig — `wheelopt.sim.step_climb`, `scripts/run_step.py`

One wheel on a vertical slider, torque at the axle: a single-wheel test rig, not a robot. A
chassis would add weight transfer, a second wheel's traction and suspension geometry — three
more ways to get the right answer for the wrong reason. The compliant wheel *is*
`rom.mjcf`'s ring, imported rather than re-derived, so what is driven at the step is the same
object that was fitted to the FEA.

### Three bugs in the rig, all of which produced plausible output

1. **The axle turned the wrong way.** Axis `0 -1 0` with a positive torque drove the rig 41 m
   backwards, away from the step. Rolling toward `+x` needs `ω > 0` about `+y`, because the
   contact point moves at `−ωR`.
2. **Constant torque has no steady state.** A wheel with nothing resisting it accelerates
   without limit — at 3 s it was 41 m away and still gaining. Every contact metric was
   measuring a wheel in flight and cost of transport was meaningless. Replaced with a motor,
   `τ = τ_stall(1 − ω/ω₀)`, which also stalls at `τ_stall` against the obstacle, i.e. exactly
   the quantity the climbing question is about. Torque is a *tractive coefficient* times
   `m·g·R`, so a 0.12 kg debug wheel is not driven with the robot's 2.8 N·m.
3. **Contact count is not contact patch.** MuJoCo resolves a cylinder on a plane as two points
   at the same `x`, separated along the axle — so a rigid wheel reports two contacts at any
   load and the count said the *rigid* wheel had the bigger patch. Replaced with the span of
   the contact points projected on x–z, which is what the FEA measures and which correctly
   gives zero for a line contact. The first version of that measured floor and step together
   and reported a 66 mm "patch" for a rigid wheel poised on the corner — it was touching the
   lower ground *and* the upper one, and the gap between two point contacts is not a patch.
   Envelopment is now measured against the obstacle geom alone.

### The confound that would have handed the project its answer

Same mass, same radius — **not** the same rotational inertia. A solid cylinder carries half a
ring's inertia about the axle, because a ring keeps its mass at the rim. Less inertia means
harder acceleration *and* less angular momentum arriving at the step, and **both push the
comparison toward the compliant wheel for reasons that are not compliance**. The rigid wheel
now takes an explicit `<inertial>` computed from the ring it is standing in for, checked
against the joint-space mass matrix MuJoCo integrates.

Getting that check right took two goes. Summing `body_inertia` by hand looks equivalent and
is not: MuJoCo stores each body's moments in its own principal frame, sorted, so for a capsule
lying along the axle the axial moment is not element 1. That route reported a 3.2% error
against a formula that was very nearly exact. The remaining 7e-5 was real — the capsule's
hemispherical caps, `½mr²` being a cylinder's answer — and the formula now carries the split.

### Result — `scripts/run_step.py --tiny --sweep`

Tiny design, 24 segments, fit 0.69% RMS. 0.120 kg on the axle (1.18 N, half the fitted
indentation), 0.092 N·m stall, 36 mm step = 0.60 R, loss factor 0.15.

| signature | compliant | rigid | required | |
|---|---|---|---|---|
| patch at the step edge | 14.3 mm | 0.0 mm | compliant longer | PASS |
| mean patch on the flat | 8.0 mm | 0.0 mm | compliant longer | PASS |
| climbed the 36 mm step | yes | no | compliant at least as good | PASS |
| cost of transport, flat | 0.0178 | 0.0082 | compliant higher | PASS |
| loaded radius, 0.3→2.4 N | 59.30 → 55.26 mm | — | must decrease | PASS |

**Tallest step cleared: compliant 50 mm (0.83 R), rigid 20 mm (0.33 R).** A 2.5× difference,
swept rather than bisected because a wheel can bounce over an obstacle it cannot roll over and
bisecting a non-monotone predicate returns whichever side it lands on. The compliant wheel
clears exactly the plan's headline 50 mm, on a 60 mm wheel.

Peak segment compression on the step was 4.99 mm against a fit that reaches 6.0 mm: **0% of
loaded samples beyond the fitted range**, so this is interpolation, not extrapolation.

**It looks like physics.** 5/5, with the two most gameable signatures (climb, cost of
transport) surviving a matched-inertia rerun.

### What this result does not license

**Cost of transport is a statement about the loss factor.** A hyperelastic FEA has no
dissipation (`fea/extract.py`), so damping comes from `TPU_LOSS_FACTOR = 0.15`, a literature
midpoint for TPU with a 0.05–0.30 span and no DMA behind it. The *ranking* is robust — a
rigid wheel has no ring to dissipate in at all — but the 2.2× is not a number to quote.

**It was run on the debug design, not the nominal, and that was forced.** See below.

### Negative result: the cubic spring law cannot fit the nominal design

Fitting the nominal (R = 85 mm, 12 spokes × 7 mm) through the 2-D tier over 0–12 mm gives
**6.8–14.7% RMS at every segment count**, against the 5% threshold. The curve is why:

| δ mm | 1.5 | 3.0 | 4.5 | 6.0 | 7.5 | 9.0 | 10.5 | 12.0 |
|---|---|---|---|---|---|---|---|---|
| F N | 28.5 | 46.6 | 50.0 | 50.2 | 50.3 | 52.1 | 62.3 | 77.3 |
| dF/dδ N/mm | 12.1 | 2.3 | 0.12 | 0.09 | 1.2 | 6.8 | 10.0 | — |

That is a **buckling plateau**: the tangent falls by 140× and comes back. A cubic in `u` cannot
bend that way, and the contact patch tells the same story from the other side — 6.0 mm at
δ = 5 mm, 73.6 mm at δ = 12 mm. The wheel transitions from riding on a stiff arch to
conforming, and *the compliance the project is about lives in the part the law cannot
represent*. Refitting below the plateau (0–5 mm) reaches 4.47% but returns a non-monotone law.

Two further findings fall out of the same curve, both worth their own entries later:

- **`detect_buckling` misses this.** It requires `dF/dδ < 0` strictly; the nominal bottoms out
  at **+0.086 N/mm** and is reported as `buckling_load_n = None`. A plateau is a limit point in
  all but the sign, and mesh or step noise can keep it just positive. Not changed here —
  loosening the criterion moves `fea_violations`, and that is a decision with consequences for
  every cached design, not a fix to make in passing.
- **The nominal design is stiff at its own design load.** 24.5 N puts it at δ ≈ 1.3 mm, 1.5% of
  radius, against 15–20% for a pneumatic tyre, with a contact patch of a few millimetres. It
  only becomes compliant past 4× nominal. That is a finding about the *design point*, not the
  model, and it is the kind of thing the optimiser should be expected to fix.

**Gates.** Unit suite **460/460** (20 new in `test_step_climb.py`), ruff at the standing 71,
`verify_cad.py` 48/48, `verify_fea.py --full` 30/30.

**Follow-up.**
1. **Replace the cubic with a tabulated monotone spring law.** Knot values enter the ring
   response linearly, so the uncoupled deconvolution stays one solve and the coupled one stays
   Gauss-Newton; monotonicity comes free by fitting non-negative interval slopes. This is what
   unblocks the nominal design, and therefore steps 5-6 on a wheel that carries the robot.
2. Decide on `detect_buckling`: a near-zero tangent should be flagged. Needs a threshold, a
   version bump and a re-run of anything cached.
3. Re-run the signatures across the loss-factor span 0.05-0.30 before quoting cost of transport.
4. The ring is still fitted to a flat-plate curve only; the step-edge case is not in the fit.
5. Still open: the band does not shear; the 20 h/sweep 3-D nominal; `contact_stiffness_factor`.

### Looking at it: `scripts/render_step.py`

Every check on the step rig up to here was numeric — contact forces, patch spans, mass
matrices. Two of the three rig bugs found earlier (the axle turning backwards, the wheel
accelerating off to 41 m) would have been obvious in one second of video, so the rig is now
renderable: Pillow-only GIFs and a contact sheet with both wheels at the same instants,
compliant above rigid. Offscreen `mujoco.Renderer` works on this Mac with no extra packages.

**The physics survived being looked at.** At t = 1.76 s the compliant ring is straddling the
corner — visibly flattened against the lower ground on one side and lying along the step's top
surface on the other, hub level with the step edge — and it is 799 mm along by t = 2.96 s. The
rigid wheel stops at x = 295 mm, exactly one radius from the riser at 350 mm, and the last
three frames are identical. Nothing looked wrong that the numbers had called right.

Two render-only defects, worth noting because both made the first attempt useless and neither
is a physics problem: the model had **no lights and no textures**, so the first sheet was six
near-black rectangles; and the camera **tracked the axle**, which keeps the wheel dead centre
in every frame and hides the one thing being looked for, whether it got anywhere. Lights,
a checker floor and coloured materials are now in `build_scenario_mjcf` — visual only, MuJoCo
integrates none of it, so the model rendered is the model measured — and the camera frames the
step instead of the wheel.

**One quantitative check the render prompted.** A wheel that slides rather than rolls would
travel and climb for entirely different reasons, and nothing measured so far would have
noticed. Measured over the flat approach, `1 - Δx/(R·Δθ)`: **compliant 4.1%, rigid 0.0%**. Both
roll. The compliant wheel slipping slightly more is the right sign — its loaded radius is below
`R`, so the same axle rotation carries it less far.

`data/renders/` is gitignored; the artefacts are regenerable and large.

## 2026-08-08 — Direction change: bandless compliant claws (`T7`)

Prompted by looking at the step-climb render. The 24 capsules read as the *wheel* rather than
as the ROM — a fair misreading, and my fault for rendering the simulation before ever
rendering the CAD. The CAD has always built spokes radiating from the hub, and `T3b` bandless
is already fingers-from-the-centre with no band at all. **Render the design before the model
of the design.**

The request that came out of it is real and is a redirection: **every future design is
bandless**, and the family is the compliant claw, named after the "Linear Claw" row of Table I
in the PaTS-Wheel letter (`docs/papers`).

Two things about that table, both easy to get wrong:

- **PaTS-Wheel is not in it.** The paper places its own design *between* "Linkage Claw" and
  "Passive Pad Deform", using pad deformation to actuate the claw. The rows are prior art the
  authors contrast themselves against.
- **Table I's Linear Claw is a rigid mechanism** — bars sliding radially through the hub,
  usually fired by a gear train on wheel stall. Taking it literally would have removed the
  FEA → ROM pipeline from the critical path entirely, since a slider mechanism is exact in
  MuJoCo with nothing to reduce. `T7` borrows the *shape* and keeps compliance in the printed
  material, which keeps the existing pipeline load-bearing.

### What was built

`claw_taper_ratio` on `WheelParams`: tip thickness as a fraction of the root, narrowing
linearly in **arc length** along the centreline. Arc length rather than point index because a
curved centreline is not sampled at uniform distance — on the index the half-thickness station
would land somewhere other than the half-length station, by an amount that moves when the
profile or curvature moves. 1.0 is the default and reproduces the uniform strut exactly, so
nothing predating the claw changes.

**The taper adds a second thickness, and that is a new way to be silently wrong** — the same
shape as every other entry in this log. A 7 mm root at 0.15 taper is a **1.05 mm tip**:
unprintable, while `spoke_thickness_mm` still reads comfortably above the 1.6 mm minimum wall.
Fixed at the source with `WheelParams.tip_thickness_mm`, and two checks now read it:

- `spoke_min_wall` — was passing the design above.
- the `no_shear_band` warning's contact-patch width — was quoting 8.0 mm where the material
  that touches the ground is 2.8 mm.

**One check deliberately left wrong and flagged.** The slenderness proxy still reads the root,
which for a taper is the stiffest section, so it understates slenderness and errs toward
*accepting* a claw that buckles. The right effective section for a tapered cantilever is not
obviously the root, the tip or the mean, and choosing one in a pre-filter would be inventing
buckling physics. Left reading the root, commented, and handed to FEA — which can arbitrate,
since `detect_buckling` runs on every radial sweep.

### Measured while checking the geometry

- The bandless tip clip narrows the contact width **3.4%** below `taper × root`: the clip pulls
  the outboard tip corner radially back onto the running surface. Real, small, and now pinned
  by a test so a change to either mechanism surfaces here.
- **`n_spokes` bottoms out at 6 and the Linear Claw figure has four.** The bound was set for a
  banded wheel where many thin spokes are cheap; fewer, longer, thicker claws are the point of
  the family. Needs re-deriving from the claw load case, not widening by fiat.

### Consequence for the band work earlier today

Bandless means both band stiffnesses are zero, `is_coupled` is False, and the ring takes the
closed-form path — numerically what it did before coupling existed. **The coupling work is
therefore dormant, not wrong**: it is still the correct model for `T3`, which stays as the
measured comparator, and the hoop-term discovery and the Roark check remain the evidence that
a bending-only band is 5.28× too soft. Worth stating plainly rather than quietly: a session's
work moved off the critical path the same day it landed.

**Gates.** Unit suite **470/470** (10 new across `test_centreline.py` and
`test_constraints.py`), ruff at the standing 71, `verify_cad.py` **48/48 against the real OCCT
kernel** — so the tapered claw builds as a solid, not merely as an outline.

**Follow-up.**
1. Per-claw tip load-deflection FEA, and a ROM whose segments *are* the claws. This is the
   large simplification the redirection buys: the spring law stops being a deconvolution of a
   whole-wheel curve and becomes a direct measurement, the segment count stops being a free
   discretisation parameter, and one claw is far cheaper to mesh than a wheel — which may
   retire the 20 h/sweep problem outright.
2. Re-derive the `n_spokes` lower bound for claws.
3. Slenderness for a tapered cantilever.
4. Tangential claw compliance. A claw bends *backwards* under drive torque, and that is much
   of what makes a claw grip; the ring ROM has radial sliders only.

---

## 2026-08-08 — The spring law: it was never the cubic, it was monotonicity

**Hypothesis.** The cubic spring law cannot fit the nominal design because it has too few
degrees of freedom to follow a buckling plateau. Replacing it with a tabulated monotone law —
knot values enter the ring response linearly, monotonicity comes free from fitting non-negative
interval slopes — should bring the fit under the 5% threshold.

**The hypothesis was half right, and the wrong half was the important one.**

### What was built

`TabulatedLaw` in `rom/ring.py`: piecewise linear, stored as knots plus interval slopes, with
`ramp_basis` giving the basis a law is a linear combination of. `fit_tabulated_law` in
`rom/fit.py`, and `nnls` — Lawson-Hanson in thirty lines, because clipping an unconstrained
least-squares answer at zero is *not* the constrained optimum and does not re-fit what
survives. Both laws sit behind a `RadialLaw` protocol, so `ring.py`, `mjcf.py` and
`sim/step_climb.py` take either without knowing which. `_levenberg_marquardt` was generalised
to a plain `(residual_fn, x0)` optimiser with an optional projection, so one damping loop now
serves three coefficients or `n` table slopes.

### The finding

On the nominal design's measured curve (plane strain, frictionless, 12 mm, 20 points), fitted
uncoupled at 24 and 36 segments:

| intervals | monotone table | force ≥ 0 only |
|---|---|---|
| 4 | 17.23% | 10.59% |
| 8 | 13.39% | 8.47% |
| 12 | 12.87% | **2.35%** |

**NNLS is convex**, so the monotone column is not a fit that got stuck — it is the best any
monotone law can do. Monotonicity, not the polynomial degree, was the binding constraint. The
cubic was only the first thing to hit it.

The reason is visible in the data and should have been predicted from it: the nominal design's
tangent stiffness runs **42.8 → −7.0 → +17.3 N/mm** across 0–12 mm. It is genuinely negative
over three samples. No monotone segment law can produce a ring response with a negative
tangent, because every segment's force rises and new segments only ever join the patch.

So the constraint was replaced with the one that is actually physical: **a compressed segment
may not pull.** `TabulatedLaw` validates `f(knot) ≥ 0`, which for a piecewise-linear function
is `f ≥ 0` everywhere. Softening is allowed and *reported* (`is_monotone_nonneg`), not
forbidden. `RingFit.ok` now gates on `is_valid_spring` rather than monotonicity; `SpringLaw`
answers that conservatively, so nothing about the cubic changed. `fit_tabulated_law(monotone=…)`
keeps the strict feasible set available — the two parametrisations are the same table under a
triangular change of variables, so one design matrix serves both.

With that, the nominal design fits for the first time: **3.42% at 36 segments and 12 intervals,
`ok=True`.**

### And the fit error still cannot be trusted alone

The 3.42% law's fitted tangent is
`15.0 → −0.1 → 0.8 → 4.5 → −11.5 → −1.5 → −0.1 → −3.8 → 5.3 → 17.2 → −25.2 → 30.4` N/mm.
A spoke does not do that. Sweeping resolution at 36 segments, unregularised:

| intervals | 4 | 6 | 8 | 10 | 12 |
|---|---|---|---|---|---|
| RMS | 10.60% | 6.19% | 4.67% | 4.01% | 3.42% |
| tangent sign changes | 2 | 3 | 7 | 7 | 6 |

**The error falls monotonically while the law stops being physical.** This is the same trap
already in this log one level down — twelve segments fit the tiny design best while putting a
single segment in contact — and it deserves the same standing answer: the fit error is not the
only number.

A `smoothing` term was added (a penalty on the change of tangent between intervals, appended as
extra rows so the problem stays one convex NNLS, scaled by ‖M‖/‖D‖ so the same value means the
same thing on any design). Default 0.1, chosen from a measured sweep: it costs 0.11 percentage
points of RMS (3.42% → 3.53%) and takes the worst spurious tangent from +63 to −12 N/mm.

**It is a remedy and not a cure, and the residual is diagnostic.** Even at 0.3 the nominal's
law still reverses sign three times. Deconvolving a *banded* wheel's whole-wheel curve into
independent radial segments is ill-posed: the band carries load between segments and the
segment law is being asked to absorb it. The cure is either a coupled fit that converges, or
the direction already taken — a bandless claw wheel where the segments *are* the claws, each
measured directly, and there is no deconvolution to be ill-posed. **This is now an argument
for the claw redirection rather than only a consequence of it.**

### `detect_buckling` sees a plateau

Same session, same curve, the other half of the problem. The old test required `dF/dδ < 0`
strictly, so a curve that flattens to +0.086 N/mm from an initial 12.1 reported `None` — a
structure carrying four extra millimetres at constant load, judged not to be buckling because
a *sign* rather than a *magnitude* was tested. Replaced with a scale-free ratio: buckled where
the tangent falls below `BUCKLING_STIFFNESS_FRACTION` (0.10) of the stiffest tangent reached
*earlier* on the branch. Measured, all three references:

- the plateau case: ratio **0.007** — caught.
- the nominal on the plane-strain tier: ratio **−0.175** — caught, and the old sign test
  caught this one too. Every curve the old rule caught, the new one catches at the same point
  or earlier.
- the tiny design's stiffening sweep, which nobody would call buckled: its tangent never dips,
  minimum ratio **1.157**. More than a decade of margin on both sides of the threshold.

The first sample is excluded from both the test and the reference: before contact closes
`dF/dδ` is meaningless, and a contact-closure spike used as the yardstick would flag every
later sample as collapsed. There is a test for exactly that.

### End to end

`scripts/run_step.py --tiny --law table` — the step rig driven by a tabulated law, in MuJoCo,
with `qfrc_applied` from the table and the damping term read off its tangent at zero:
**5/5 signatures**, 36 mm step cleared against the rigid wheel's failure, 0% of loaded samples
beyond the fitted range. A softening law is a dynamic hazard in principle (the segment snaps
through rather than settling); the tiny design's law does not soften, so this run does not test
that and does not claim to.

**Gates.** Unit suite **498/498** (28 new), ruff at the standing 71.

**Follow-up.**
1. Per-claw tip load-deflection FEA, and a ROM whose segments are the claws — now with a
   second reason: it removes the deconvolution, which is the thing that is ill-posed.
2. A softening law has never been driven in MuJoCo. Build the case deliberately rather than
   waiting for a design to produce one.
3. The coupled tabulated fit stalls: on the nominal at 24 segments it reports 15.57% and
   `converged=False` where the uncoupled fit gets 8.32%. Projected Gauss-Newton over eight
   slopes with finite-difference Jacobians through an active-set solve is the suspect.

---

## 2026-08-08 — Segments are claws: 200× cheaper, and the curve it produces is not yet a spring law

**Hypothesis.** For a bandless `T7` wheel the ring's segments can *be* the claws, one for one.
Then the segment spring law is a direct measurement rather than a deconvolution of a
whole-wheel `F(δ)`, the segment count stops being a free parameter, and one claw is cheap
enough to mesh that the 20 h/sweep 3-D problem may be recoverable.

**Config.** conda `conda3.12`; CalculiX 2.23, gmsh 4.15.2. Nominal claw design: R 85 mm,
W 45 mm, 12 spokes, 7 mm root, `claw_taper_ratio` 0.5, hub 22 mm, bore 4 mm, **bandless**.
Plane strain, CPE6, 2.5 mm spoke / 4.5 mm hub, `--contact-stiffness 5`,
`spoke_phase_deg = phase_for_tip_contact(12)`.

### What was built

`ClawSector` and `mesh_claw_sector` in `fea/section2d.py`; `MeshSpec.claw_sector` and
`claw_hub_span_deg`, so the choice reaches the cache key like every other mesh field and the
*load case* stays an ordinary rigid flat plate — nothing in `deck.py`, `runner.py` or
`extract.py` changed. `ring_from_claw_curve` in `rom/fit.py`, which contains no fit: it wraps
the measured curve in a `TabulatedLaw` with `n_segments = n_spokes`.

### The cost, measured

| | elements | wall clock |
|---|---|---|
| whole wheel | 3155 CPE6 | 41.7 s |
| one claw + full hub | **492** | **0.2 s** |
| one claw + 30° hub wedge | 266 | refused, see below |

### The agreement, and the honest size of it

Claw sector against whole wheel over 1–8 mm: **0.07% of peak**, curves identical to two
decimals. That confirms the sector is the right geometry — but the comparison is weaker than
it looks, and saying so matters more than the number. At these indentations **only one claw
touches the whole wheel either**: the next tip is 30° away and does not reach the plate until
`R(1 − cos 30°) = 11.4 mm`. So this validates "one claw is one claw" and says nothing yet
about the multi-claw regime. A sweep past 11.4 mm is the test that would.

### The hub wedge is refused, not offered

A finite `claw_hub_span_deg` would make cost independent of `n_spokes`. It also cuts the bore
arc the shaft constraint acts on: measured, a 30° wedge on a 4 mm bore keeps **two nodes**.
Two nodes is four constraints in 2-D — enough to stop rigid-body motion, so it *solves*, and
what it describes is a claw pivoting on two pins rather than one clamped to a shaft. Exactly
the recurring failure: it converges and the number is wrong. `mesh_claw_sector` now refuses
fewer than four bore nodes. The default keeps the whole hub annulus, which is exact, and still
gives the 6.4× element saving above.

### The finding: the measured claw curve is dominated by tip slip

The curve is nearly flat — 4.59 N at 1 mm, 4.73 N at 8 mm — which reads as a buckling plateau.
It is not, or not only. Three measurements, same design:

| | force at 1 mm | shape |
|---|---|---|
| taper 0.5, frictionless | 4.59 N | peaks 5.10 N at 0.4 mm, then **falls** to 4.55 |
| taper 0.5, μ = 0.8 | **22.7 N** | peaks 26.8 N at 2 mm, then falls to 24.9 |
| taper 1.0 (uniform strut), frictionless | 7.10 N | **rises** to 8.39 N, no limit point |

**A 5.4× change in force from the friction coefficient alone.** The contact patch is
**0.00 mm at every point** — the tip touches on a single node. Together those say the tapered
tip is *sliding* along the plate, and the vertical reaction is set by whether it sticks or
slips, not by how hard the claw resists being compressed. The uniform strut, whose tip is
twice as wide, does not do it.

`detect_buckling` fires on the tapered claw and reports the limit point at 4.71 N — the ratio
test built earlier today catches it, which is the right behaviour whatever the mechanism.

**Why this blocks the thing it was built for.** The ring gives each segment a *radial slide
and nothing else*. Feeding it this curve encodes tangential slip physics into a radial spring,
and the resulting law moves 5.4× with a friction coefficient nobody has measured on printed
TPU. `docs/plan/TODO.md` #20 (tangential claw compliance) was written as an enhancement — a
claw that cannot bend backwards cannot hook. It is now a **validity condition**: without a
tangential degree of freedom, the per-claw measurement is not a segment law.

The machinery is right and cheap and tested. What it currently measures is not yet the thing
the ROM needs.

**Gates.** Unit suite **507/507** (9 new across `test_section2d.py` and `test_rom.py`), ruff
at the standing 71.

**Follow-up.**
1. Separate slip from structural response before trusting any claw curve: sweep μ, and
   compare against a tip loaded by prescribed radial displacement with no plate at all.
2. #20, promoted to blocking.
3. Sweep past 11.4 mm to test the sector in the multi-claw regime.
4. The 3-D claw sector. The 2-D one cannot see lateral buckling, and a slender tapered claw is
   exactly the geometry that buckles sideways.

---

## 2026-08-08 — A manual playground, and the rounding bug it found in its first hour

**Hypothesis.** Most of a design playground already existed — screening, 2-D FEA with PDF
plots, the ROM fit, the step rig, the renderer — spread across five scripts with no shared
output. One driver plus one report would make it usable by hand, and (the reason for doing it
before more physics) **a visual check finds a different class of bug than a numeric one**.
That claim has paid twice in this log already: Roark's ring caught the 5.28× band error, and
`render_step.py` exists because a rig can pass every numeric check and look obviously wrong.

### What was built

`scripts/explore.py` and `wheelopt/report.py`. One set of geometry flags runs screen → 2-D FEA
→ ROM fit → step climb → render and writes one self-contained HTML page (SVG inline, GIF as a
data URI). `--compare spokes=6,10,14` puts several designs on shared axes. Each stage catches
its own failure and records it on the design, so a run that will not mesh or will not converge
still produces a page saying so — for a tool whose job is exploration, losing that to a
traceback is the worst outcome.

Three things fell out that were not the point but were overdue:

- **`claw_taper_ratio` was unreachable from any CLI.** The parameter the whole `T7` direction
  turns on could not be set from the command line. Added as `--claw-taper` on the shared
  argparse, so every script gets it at once.
- **The five signatures were a print-closure inside `run_step.py`.** Two callers now need
  them, so `Signature`, `judge_signatures` and `loaded_radius_table` moved into
  `sim/step_climb.py`. Two copies of a judgement is how one report comes to pass while
  another fails on the same run.
- Every panel carries its tier and whether that tier screens or decides; `Panel.caution` is a
  banner, not a footnote. Measured on the runs below, four fired without being asked:
  zero-length contact patch, fewer than three segments in contact, extrapolation beyond the
  fitted range, and the standing claw caveat.

### The bug it found

First non-`--tiny` design pointed at it, the rigid comparator failed to load:

    inertia must satisfy A + B >= C

The rigid wheel's `<inertial>` matched the ring's axle inertia with `transverse = 0.5 *
inertia`. For a thin ring that is exactly right — and it puts the model exactly **on** the
triangle-inequality boundary, since `transverse + transverse == inertia`. Whether MuJoCo
accepted it depended on which way `%.12g` rounded the last digit.

Swept afterwards over 54 geometries (6 segment counts × 3 radii × 3 widths): the old formula
is **rejected in 3 of 54**. It passed on `--tiny` and failed on a 10-claw 85 mm wheel, which
is why nine months of `--tiny` runs never saw it.

Fixed with the moment these bodies actually have — a cylindrical shell of finite width,
`I_axle/2 + m·L²/12` — which is strictly inside the boundary rather than on it. The fix is not
a nudge to clear a check; the thin-ring value was simply the wrong formula for capsules with a
width. Regression test sweeps the same 54 configurations, and it was confirmed to fail before
the fix rather than assumed to.

**This is the argument for the playground, made by the playground on its first outing.** The
bug is invisible to every existing test because they all use one geometry.

### What the pages show

- 8 spokes, 6 mm, banded: the fitted table is an **N-shape** — force to 16 N, down to 0, back
  up — at 26.53% RMS. The number alone is a bad fit; the plot is obviously not a spring.
- `--compare spokes=6,10,14`: peak force **2.47 / 51.89 / 60.15 N**. A 20× swing between 6 and
  10 spokes, which is worth knowing before treating spoke count as a smooth knob.
- 10 claws, taper 0.6, bandless: 18.85% RMS with **one** segment in contact, and the run
  extrapolates past the fitted range. Three cautions on one page, all correct.

**Gates.** Unit suite **508/508** (1 new, the 54-geometry inertia sweep), ruff at the standing
71. `data/explore*/` added to `.gitignore`.

**Follow-up.** The report is read-only — a re-run per change. If turning knobs interactively
becomes the bottleneck, the next step is a slider UI over the cached FEA results, not a faster
solver.

---

## 2026-08-08 — The claw curve is not friction-sensitive, it is bimodal: stick or slip

**Hypothesis.** The per-claw measurement was recorded earlier today as untrustworthy because
"the same claw reads 4.59 N frictionless and 22.7 N at mu=0.8, a 5.4x swing set by a friction
coefficient nobody has measured." If that is right, the claw ROM is blocked until TPU friction
is characterised. Sweep `friction_mu` and find out.

**That framing was wrong, and the correction is good news.**

### The sweep

One claw sector, nominal claw design (R 85, 12 spokes, 7 mm root, taper 0.5, bandless), plane
strain, delta to 6 mm:

| mu | claw, F@1mm | claw, F@3mm | strut, F@1mm | strut, F@3mm |
|---|---|---|---|---|
| 0.0 | 4.59 | 4.56 | 7.10 | 7.83 |
| 0.2 | 22.69 | 25.66 | 31.43 | 65.73 |
| 0.4 | 22.69 | 25.66 | 31.43 | 65.73 |
| 0.8 | 22.69 | 25.66 | 31.43 | 65.73 |
| 1.2 | 22.69 | 25.66 | 31.43 | 65.73 |

**Identical to five significant figures for every mu >= 0.2.** There is no friction
sensitivity: the tip either sticks or it slips, and it sticks at any coefficient a real floor
provides. Printed TPU on a hard surface is mu ~ 0.8-1.2, comfortably inside the flat part.

Five identical numbers is exactly the shape of a cache collision, and this project has had one
(`SolverSpec` excluded wholesale as "timing", 2026-08-08). Checked rather than assumed:
`fea_cache_key` gives **five distinct keys**, so these are five separate solves that agree.

### Mesh convergence, because a single-node contact was the other suspect

The contact patch reads 0.00 mm at every sample, so the reaction could have been one node's
worth of penalty force rather than a structural answer. Refined the tip mesh 7x — the tip is
3.5 mm wide, so 0.5 mm elements put roughly seven nodes across it:

| h, mm | elements | mu=0.8, F@1mm | F@3mm | mu=0, F@1mm | F@3mm |
|---|---|---|---|---|---|
| 2.5 | 537 | 22.69 | 25.66 | 4.59 | 4.56 |
| 1.5 | 971 | 22.82 | 25.64 | 4.59 | 4.55 |
| 0.8 | 1935 | 22.87 | 25.63 | 4.59 | 4.55 |
| 0.5 | 3854 | 22.88 | 25.75 | diverged | — |

**Both branches are converged to 0.4-0.8% over a 7x refinement.** Neither is a discretisation
artefact. (The frictionless run diverges at 0.5 mm: the penalty is `factor x E / element_size`,
so a finer mesh stiffens contact — the known issue in TODO #12, arriving on schedule.)

### Which branch the ring wants, and why that settles it

A ring segment slides **radially and only radially**. It has no tangential freedom, so a
segment is kinematically the *stick* case, not the slip case. The stick branch is also the
physically realistic one. The two agree, which is the part that unblocks this:

**Fit the claw ROM to mu >= 0.2, not to the frictionless curve.** The number does not depend
on mu within that branch, and it does not depend on the mesh.

### What was actually wrong, and where it was written

The earlier entry, `CLAUDE.md`, `TODO.md` #24 and `explore.py`'s caution banner all said the
claw response swings 5.4x with an unmeasured parameter. It does not. It takes one of two
values, the relevant one is insensitive to the parameter, and the relevant one is the one the
ring model corresponds to. All four have been corrected.

**One live defect this exposed in a tool shipped an hour earlier**: `scripts/explore.py`
defaulted to `--friction 0`, which is the **slip** branch — right for a banded wheel (the 2-D
tier's calibration against 3-D was done frictionless) and wrong for a claw by a factor of five.
The default is now topology-dependent and printed on every run.

### What remains open

Not friction. The ring still has no tangential degree of freedom, so it cannot represent a
claw bending *backwards* under drive torque — which is much of what makes a claw grip a step
edge. That is TODO #20, and it is now an ordinary enhancement again rather than a validity
condition on the radial law.

Also still open, and unrelated: the contact patch reads 0.00 mm even at seven nodes across the
tip. A bending claw touches on a corner, so a very short patch is expected — but zero at that
resolution wants explaining before patch length is used for anything on a claw.

**Gates.** Unit suite 508/508, ruff at the standing 71.

---

## 2026-08-08 — The ring models a claw's stiffest direction and omits its softest, by 576×

**Hypothesis.** Before building the tangential degree of freedom (TODO #20), scope it: how
much stiffer is a claw radially than tangentially? If the tangential direction is comparably
stiff, the omission is a refinement. Closed-form beam theory, using the *same* knocked-down
modulus the FEA deck is written with, so this is a check against the model rather than a
restatement of it.

**Config.** Nominal claw: R 85 mm, 12 spokes, span 63 mm, 45 mm wide, 7 → 3.5 mm tapered
section, bandless. `for_material(...).initial_youngs_pa` = **8.99 MPa**.

### The two directions

A claw points radially outward from the hub. Load its tip **radially** and it is a *column*
in compression; load it **tangentially** and it is a *cantilever* in bending.

| | formula | value |
|---|---|---|
| radial, axial | `EA/L` | **33.70 N/mm** — what the ring models |
| tangential, bending | `3EI/L³` | **0.0585 N/mm** — what the ring omits |
| ratio | | **576×** |

**The ring gives each segment a radial slide and nothing else. That is the direction 576 times
the stiffer of the two.** For a claw wheel this is not a missing refinement; it is the missing
dominant compliance.

### Two things this confirms from outside the model

The scoping numbers were not the point, but they arrive as external checks on results this log
already contains — the discipline of a number that did not come from the model that produced
the thing being checked:

- **Radial stiffness.** Analytic `EA/L` = 33.7 N/mm against the measured stick-branch
  22.69 N/mm at 1 mm. Same order, analytic high — expected, since the claw carries a 2 mm
  sagitta so it is not a pure column, and the mean-section approximation overstates a taper.
- **The frictionless branch is column buckling.** Fixed-free Euler on the root section gives
  **7.19 N**; the measured frictionless plateau is **4.59 N**. Same order, measured low —
  expected, because a tip free to slide sideways is closer to pinned-free than to fixed-free,
  and pinned-free has the lower critical load. The plateau this log recorded twice as "the
  claw softens" is a **buckling column**, and now has a closed form behind it.

### What it means for what is already recorded

The spike's headline — 50 mm cleared against a rigid wheel's 20 mm — was measured on `--tiny`,
a **banded `T3`** design. A shear band carries tangential load between segments, so the
radial-slide ring is a reasonable model there and that result stands. **It does not transfer
to `T7` claws**, where there is no band and the tangential path is a bare cantilever. Any claw
step-climb number from the current ROM is a lower bound on compliance and therefore, most
likely, a *pessimistic* climb result.

The direction that matters is also the loaded one: flat rolling presses a claw radially, into
the stiff mode, and the ring is fine. A step edge and drive torque load it tangentially, into
the soft mode, which the ring cannot move in at all. Obstacle traversal is the whole point of
the project, so this is loaded exactly where it hurts.

### Consequence

**#20 is re-scoped from "an important enhancement" to "the next substantial piece of work",**
and it is now the thing standing between the claw pipeline and a trustworthy climb number. It
needs: a tangential tip-load FEA case on the claw sector, a second degree of freedom per ring
segment with its own law, the MJCF joint to match, and a `ROM_VERSION` bump. The step-climb
signatures should be re-run across it — the climb result is the one most likely to move, and
it should move *upward*.

**Gates.** No code changed; unit suite 508/508, ruff at the standing 71.

---

## 2026-08-08 — Measuring a claw's tangential stiffness: 134×, and two bugs that read as physics

**Hypothesis.** Yesterday's scoping arithmetic put a claw's radial stiffness 576× above its
tangential one, from `EA/L` against `3EI/L³`. Measure it instead of estimating it, which needs
an FEA case that loads the tip tangentially. First half of TODO #20.

### What was built

Two contact-free load cases, `TIP_RADIAL` and `TIP_TANGENTIAL`, with
`LoadCaseKind.needs_indenter` selecting the deck shape. There is no indenter, no surface
interaction, no friction and no contact pair; instead the tread node set is *itself* the rigid
body, tied to `NREF`, and that node is driven. `NREF` means the same thing in both deck shapes,
so parsing and extraction are untouched.

That is a modelling claim, not a convenience: **a ring segment is a rigid body on a slide**, so
driving the tread rigidly is the ring's own kinematics written out in FEA. It also removes the
contact model from the measurement, which is the only way to tell a structural answer from a
contact one.

### The measurement

Nominal claw, plane strain, delta to 6 mm:

| | at 1 mm | at 6 mm |
|---|---|---|
| tip, radial | **24.81 N/mm** | 37.73 N |
| tip, tangential | **0.1851 N/mm** | 1.77 N |

**Measured ratio 134×**, against the 576× estimated. The estimate was not wrong so much as
answering a different question, and the gap is explainable rather than mysterious: the rigid
tip cannot *rotate*, so the right closed form is a **guided** cantilever `12EI/L³` = 0.234 N/mm,
not a free-tip `3EI/L³` = 0.0585. Measured 0.185 sits just below the guided value, which is
what a section tapering 7 → 3.5 mm toward the tip should do. Radial agrees the same way:
24.81 N/mm rising to a secant near the analytic `EA/L` = 33.70 as the claw straightens.

**134× stands as the finding.** The ring gives each segment the stiff direction and none of the
soft one, and a step edge loads the soft one.

### Two bugs, both of which produced plausible numbers

Neither announced itself. Both were caught by having a closed form to compare against — the
first useful thing the scoping arithmetic did.

1. **The tangential boundary condition was inverted.** The comment said "leave the radial DOF
   free"; the code held it. A claw bending tangentially sweeps its tip along an *arc*, so it
   must come radially inward; forbidding that makes it stretch along its own axis instead, and
   the case reports the **axial** mode. Measured while wrong: 7.35 N/mm — 125× the beam-theory
   value and *constant with displacement*, because nothing was bending.
2. **The extractor hardcoded the y axis.** `build_load_curve` read component 1 for force and
   displacement. The tangential case drives x, so displacement came back **identically zero**
   while the force column filled with rising, believable numbers. A zero-displacement curve
   then made `np.interp` return the last force for every query — which is where the suspiciously
   constant "7.354 N at both 1 mm and 3 mm" came from. The axis is now chosen from the load
   case.

The second is the sharper one: a curve with a **zero** independent variable and a healthy
dependent variable still produced a stiffness that could be quoted. Both are pinned by tests
in `tests/test_fea_deck.py` — the tangential test asserts the radial DOF is *absent* from the
boundary block, which is the assertion that would have failed.

### Where this leaves #20

The FEA half is done and measured. The ROM half is not: `ring.py` still gives each segment one
degree of freedom. That is the next piece — a second DOF with its own law, the MJCF joint, a
`ROM_VERSION` bump, and the step-climb signatures re-run across it.

**Gates.** Unit suite **514/514** (6 new), ruff at the standing 71.

---

## 2026-08-09 — A two-freedom ring, and a question it raises about the one-freedom one

**Hypothesis.** With the tangential stiffness measured (134× below radial), give the ring
segment a second degree of freedom and see what it changes. Second half of TODO #20.

### What was built

`solve_equilibrium_2dof` in `rom/ring.py`, plus `SegmentState2D`, `ring_force_2dof_n` and
`symmetric_force_n`. Deliberately **additive** rather than a rewrite of `solve_equilibrium`:
without a band the segments are independent, so the two-freedom problem factorises into `N`
three-unknown problems instead of one `2N` system. Bandless is every design this project now
builds; a banded spec is refused rather than approximated.

Per segment, with `y(u,v) = (R − δ) − (R − u)cos θ + v sin θ` the height above the plate and a
purely vertical contact force `λ`:

    f_r(u) = λ cos θ,   f_t(v) = λ sin θ,   λ ≥ 0,  y ≥ 0,  λy = 0

`f_r` and `f_t` increasing ⇒ `y` decreasing in `λ` ⇒ a one-dimensional **bisection** on `λ`
that cannot fail to bracket and needs no initial guess. The inner inverse is bisection too,
because a `TabulatedLaw`'s tangent can be exactly zero over an interval — a buckled segment at
constant load — which is a legitimate law and a division by zero for a Newton inverse.

### What it changes

On the nominal claw ring (12 segments, R 85 mm, k_r 24.81 N/mm, k_t 0.1851 N/mm):

| δ, mm | rigid-tangential | 2-dof | ratio | in contact | max splay, mm |
|---|---|---|---|---|---|
| 8 | 198.5 | 198.5 | 1.000 | 1 | 0.00 |
| 11 | 272.9 | 272.9 | 1.000 | 1 | 0.00 |
| 12 | 328.1 | 318.4 | 0.970 | 3 | 1.20 |
| 18 | 774.6 | 670.0 | 0.865 | 3 | 12.9 |
| 25 | 1295.5 | 1080.3 | 0.834 | 3 | 26.6 |

**It is exactly inert until a second claw engages**, at `R(1 − cos 30°) = 11.4 mm` for twelve
claws. A lone segment sits at `θ = 0`, where `sin θ = 0` and there is nothing to splay. Two
consequences worth stating plainly: the flat-plate fit **at design load is unaffected** (24.5 N
is δ ≈ 1 mm, deep in the single-claw regime), and everything this freedom buys is at large
indentation or at angled contact — i.e. at a step, which is where it was wanted.

### The bug the symmetry test caught

`_invert` checked `force_n <= 0` and returned zero **before** the sign handling, so the
symmetric branch was unreachable — and it was marked `# pragma: no cover`, which should have
been read as a warning rather than written as a note. Segments on one side of the contact
point splayed and their mirrors did not, so the ring walked sideways under a symmetric load.
Found by asserting `sum(slip) == 0`, not by reading the code.

### The question this raises about the existing ring

Building the second freedom forced the contact force to be written down explicitly, and the
two models **disagree about how it resolves**. At δ = 18 mm, 12 segments, per segment:

| θ | `f_r` | `f_r·cos θ` (existing) | `f_r/cos θ` (2-dof) |
|---|---|---|---|
| 0° | 446.53 | 446.53 | 446.53 |
| ±30° | 189.40 | 164.03 | 218.70 |

Total 774.58 N against 883.93 N — **a factor of 1.141**, and it grows with how far the patch
spreads. The *kinematics* agree exactly; only the force resolution differs.

The argument for `f_r/cos θ`: a frictionless plate can only push **vertically**, and the
segment's tangential equilibrium is supplied by its own slide joint, which is internal to the
wheel. Virtual work along the joint axis then gives `f_r(u) = λ cos θ`, so the plate sees
`λ = f_r/cos θ`. The argument for `f_r·cos θ` — the one in `ring.py` and in every fit so far —
treats the segment as a strut carrying force along its own axis and takes the vertical
component.

**Evidence on the other side**, and it is why this is not being changed today: the MuJoCo
realisation, which computes real contacts and real joint reactions, agreed with the existing
analytic ring to **0.03–0.05%** below 4 mm (2026-08-08). At that depth the off-axis segments
carry little, so the test is weak — `cos²15° = 0.933` on a small contribution — and the
disagreement did grow to 6.1–6.6% at 5–6 mm, which was attributed to contact discretisation.
Some of that may be this.

**Filed as TODO #26 rather than fixed.** It is a real question with real evidence both ways,
the fits absorb it into the spring law (so `F(δ)` still matches the FEA either way, and it is
the *inferred law* that would be distorted), and settling it needs a deliberate MuJoCo
experiment at a depth where the off-axis segments carry real load — not an argument at the end
of a session. The new tests were written to assert only what is verified: the two solvers'
**compressions** agree to 1e-5 in the rigid-tangential limit, and the softening comparison is
made against the same solver rather than across the disputed quantity.

**Gates.** Unit suite **521/521** (7 new), ruff at the standing 71.

## 2026-08-09 — MuJoCo settles #26: the ring divided where it multiplied

**Hypothesis.** The ring's frictionless contact force resolves as `f_r/cos θ`, not
`f_r·cos θ`. MuJoCo assumes neither, so it can decide. Closes TODO #26.

### The measurement

Not a comparison of totals. MuJoCo's round capsules engage a wider set of segments than the
analytic scallop geometry does — that is a **separate**, already-recorded disagreement, and
comparing sums would have confounded the two. Instead: press the ring, let it settle, and read
back MuJoCo's *own* per-segment joint position `u_i` and its *own* per-contact force `λ_i`.
Then ask which relation holds between them.

Config: bandless ring, R 85 mm, 24 segments, a **linear** law `a = 24.81 kN/m` so nothing is
hidden in a nonlinearity, `condim="1"`, δ = 2–20 mm, settled to `max|q̇| < 1e-13`.

| δ, mm | segment | θ | `u` from MuJoCo, mm | `λ` measured, N | `f_r·cos θ` | `f_r/cos θ` |
|---|---|---|---|---|---|---|
| 6 | 0 | 0° | 5.9442 | 147.475 | 147.475 | 147.475 |
| 6 | ±1 | ±15° | 3.3623 | 86.360 | 80.575 | 86.360 |
| 15 | ±1 | ±15° | 12.6541 | 325.024 | 303.252 | 325.024 |
| 15 | ±2 | ±30° | 4.9679 | 142.321 | 106.741 | 142.321 |

Worst relative error over the whole sweep: **`f_r/cos θ` 6.2e-11, `f_r·cos θ` 2.5e-1**. The
0.25 is not approximate — it is exactly `1 − cos²30°`. The measured *horizontal* contact force
is exactly 0.0 N at every contact, which is the premise the derivation rests on.

**So the incumbent was wrong**, and the virtual-work argument in `vertical_reaction_n` is the
reason: with the plate pushing only along its normal, the generalised force on a radial slide
is `λ cos θ`, so equilibrium of the segment reads `f_r(u) = λ cos θ` and the plate carries
`λ = f_r/cos θ`. `Σ f_r·cos θ` is the vertical part of a force pointing along the segment's own
axis — the segment as a two-force strut — which needs a horizontal `f_r sin θ` the plate has no
way to supply.

### What it actually changes, which is less than it sounds

Fixed in `solve_equilibrium` (both branches) via a new `vertical_reaction_n`; `ROM_VERSION` to
**`rom-0.4.0`**, and this is the first bump that genuinely invalidates prior fits.

**The tiny design barely moves.** Re-fitting its cached FEA curve under both resolutions:

| segments | `a` fixed, N/m | `a` old, N/m | ratio | RMS fixed | RMS old |
|---|---|---|---|---|---|
| 24 | 171.88 | 178.33 | 0.964 | 0.68% | 0.69% |
| 36 | 129.24 | 129.72 | 0.996 | 1.23% | 1.19% |
| 48 | 98.98 | 99.26 | 0.997 | 1.24% | 1.22% |

Because this design's patch is three segments wide, so the correction only ever acts at ±15°
(`cos² = 0.933`) on the two segments carrying least. Note the RMS gets marginally *worse* at 36
and 48 — 0.03pp, well inside the noise of a fit compensating for a geometry mismatch either
way, and not a reason to prefer a wrong model.

Where it is not small is where the patch spreads: **14.1%** on the 12-claw ring at δ = 18 mm
(the discrepancy that raised #26 in the first place), because that patch reaches ±30°. Few
segments, deep indentation and a step edge are exactly the regime the claw redirection is
heading into, so this needed fixing before the claw ROM is built on it, not after.

**The 5–6 mm MuJoCo gap shrank, as predicted, and did not close.** `run_rom.py --tiny --mujoco`,
24 segments, re-fitted:

| δ, mm | ring N | mujoco N | gap now | gap before |
|---|---|---|---|---|
| 1–4 | — | — | 0.03–0.05% | 0.03–0.05% |
| 5.0 | 2.690 | 2.819 | 4.80% | 6.6% |
| 6.0 | 4.341 | 4.514 | 4.00% | 6.1% |

Both halves of that matter. The sub-4 mm agreement is *unchanged*, which it must be — one
segment dominates there and `cos 0° = 1`. The 5–6 mm gap fell by about a third, and the
remainder is the contact-discretisation effect it was originally attributed to: MuJoCo's
capsules bridge the scallops and touch on more segments than the analytic active set. So the
old attribution was **partly** right, and partly cover for this.

### Two things caught while writing the tests

**A capsule on a plane is two contacts, not one.** MuJoCo resolves the line contact at the ends
of the capsule axis, each carrying half the load. The first version of the regression test
asserted per contact and failed by exactly 2.000 on every segment — which reads as a factor-of-two
physics error and is a factor-of-two bookkeeping one. Forces are now accumulated per segment.

**The hand-computed reference had to be computed, not recalled.** The first literal in the test
was 11.3361 N from mental arithmetic; the true value is 11.3354970. Pinning the wrong constant
would have been the same class of failure as the bug being fixed.

### Why no existing test caught this

All 521 passed after the change, which is the finding. Every ring-force test in the suite
checks a *shape* — monotone in δ, stiffer at more segments, zero at zero indentation — and
`cos²θ` is a positive factor that scales a curve without bending it. Nothing pinned a
magnitude, and the fit absorbed the rest into the spring law, so `F(δ)` matched the FEA either
way. This is the third instance of the same failure recorded in this log: **a model checked
only against itself**. Now pinned by `TestVerticalReaction`, which computes the three-term sum
from literal angles rather than through `segment_angles`/`penetrations`, and by the MuJoCo
arbitration above kept as a regression test.

**Gates.** Unit suite **525/525** (4 new), ruff at the standing 71.

## 2026-08-09 — The MJCF tangential joint works statically and folds the wheel under drive

**Hypothesis.** Give the MuJoCo ring the tangential freedom the analytic ring got yesterday,
and the step-climb signatures will move. Second half of TODO #20.

### The joint, and what validates it

One extra slide per segment along `(cos θ, 0, sin θ)` — the in-plane perpendicular to the
segment's own radius, chosen so that moving a segment by `v` raises its tip by `v sin θ`,
exactly the term in the analytic height equation. Opt-in, and **off means absent rather than
locked**, so every result that predates it is still reproducible from the same XML.

**Refused on a banded spec**, matching `solve_equilibrium_2dof`. Not a solver limitation —
MuJoCo would integrate it happily — but `coupling_tendons` couples the *radial* joints only,
so a banded ring with tangential slides would let its segments shear past each other with
nothing resisting, which is the one deformation a shear band exists to carry.

Validated per segment, not on totals, for the same reason as #26: MuJoCo's capsules engage a
different set of segments than the analytic point geometry. Reading back its own `u_i`, `v_i`
and `λ_i` on the static press (12 claws, R 85 mm, k_r 24.81 N/mm, k_t 0.1851 N/mm), both
Kuhn-Tucker conditions hold at **1e-9 or better** at every contact and every depth:

| δ, mm | seg | u, mm | v, mm | λ, N | `f_r − λcos θ` | `f_t − λsin θ` |
|---|---|---|---|---|---|---|
| 18 | 0 | 17.911 | 0.000 | 444.310 | −5.7e-14 | 0 |
| 18 | ±1 | 0.205 | ±15.832 | 5.861 | 3.4e-10 | ∓5.3e-10 |
| 25 | ±1 | 0.381 | ±29.510 | 10.925 | 5.9e-10 | ∓1.0e-9 |

Totals agree too — 456.03 N measured against 456.10 N analytic at 18 mm (0.016%), 639.47
against 639.89 at 25 mm (0.066%). The off-axis claws **fold rather than compress**: 15.8 mm of
splay against 0.2 mm of compression, which is the whole phenomenon and is what a joint on the
wrong axis would not produce.

### Two records corrected

**`R(1 − cos π/n)` is wrong; the pitch is `2π/n`.** The *number* — 11.4 mm for twelve 85 mm
claws — is right, the formula written beside it is not, and `tests/test_rom.py` had encoded the
formula: `SECOND_CLAW_M = 0.085 * (1 - cos(π/12))` = **2.90 mm**, a quarter of the truth.
Nothing failed, because the only test using it halves it first and 1.45 mm is below both
thresholds. A wrong constant that happens to be conservative is still wrong. Now asserted
against the segment grid.

**"3% softer at 12 mm, 17% at 25 mm" does not reproduce.** Against the corrected radial-only
ring the softening is far larger:

| δ, mm | radial-only, N | 2-dof, N | ratio |
|---|---|---|---|
| 11 | 272.88 | 272.88 | 1.000 |
| 12 | 338.18 | 298.57 | 0.883 |
| 18 | 883.93 | 456.10 | 0.516 |
| 25 | 1520.65 | 639.89 | 0.421 |

Part of that is #26 moving the baseline, but the 2-dof numbers moved too (318.4 → 298.57 at
12 mm), which #26 does not touch. The likeliest cause is that yesterday's table was taken
**before** the `_invert` sign bug was fixed — that bug left one side of the ring unable to
splay, which under-reports exactly this. The magnitudes are now pinned in the test rather than
only the direction, so a third set cannot appear silently.

### A timestep bound that was there all along

Driving the joint in the **rolling** rig diverged immediately. It is not the force law: the
joint diverges with *no force applied to it at all*, and it diverges **faster** at higher
stiffness, which is the signature of an explicit-integration limit rather than a sign error.

`qfrc_applied` is an *external* force, so `implicitfast` integrates it explicitly even though
it integrates a joint's own `damping` attribute implicitly. Measured on the bandless R 60 mm
claw ring, k = 19.76 kN/m on 2 g segments (ω = 3143 rad/s):

| ω·h | result |
|---|---|
| 0.377, 0.314 | diverges inside 5 ms |
| 0.251, 0.220, 0.189, 0.157 | clean over 0.6 s |

**The radial-only rig has been running at ω·h = 0.63 and surviving by luck**: an out-of-contact
radial segment sits at exactly `u = 0` where the law returns exactly zero, so nothing excites
the mode. A tangential joint's axis sweeps through gravity every revolution, so it is excited
continuously. Fixed with `stable_timestep_s`, which tightens the step to `ω·h ≤ 0.2` — about
25% under the observed boundary. Two alternatives also work and were rejected: **native joint
damping** (`c ≥ 5 N·s/m`) is dissipation no material supplied, and cost of transport is one of
the five signatures; **heavier segments** (`≥ 20 g` against 2 g) would move the rigid
comparator too, since it matches ring mass. Re-running the radial-only baseline at the tighter
step changed nothing that matters (edge patch 26.9 mm both ways, CoT 0.0197 → 0.0201, peak
compression 7.51 → 8.05 mm, 5/5 either way), which is the check that the bound is a numerical
fix and not a physical one.

### And then the wheel folds over

With the timestep sound, the driven run is still unusable, and now for a reason the model is
entitled to give. At the platform's per-wheel load the tractive force is 21.3 N, and a claw at
`k_t = 0.1475 N/mm` needs

    21.3 / 147.5 = 144 mm of tangential deflection (188 mm at stall)

against a joint range of ±30 mm and a wheel radius of 60 mm. The claws lie flat, the reported
"contact patch" becomes 404 mm, and the signature table prints **4/5** for a wheel that has
collapsed — worth noting on its own, since `run_step` grades any run whose history stays
finite. `fraction_beyond_fit` did report 100%.

**This is a design finding, not only a modelling one.** A tip-loaded claw cantilever at the
measured stiffness cannot transmit drive torque: the tractive force at a planted tip is
perpendicular to the claw, so it bends it, and 134× below radial is far too soft at this load.
Either the claw family needs a much stiffer tangential path than the nominal claw has, or
torque is not transmitted the way this ROM assumes. **Not resolved here** — the 134× was
measured on one claw (R 85, 7 mm root, 0.15 taper) and scaled onto a different design, so the
first thing to do is measure `k_t` on the design actually being driven.

**What stands and what does not.** The joint is built, refused where it would be wrong, and
validated against the analytic ring to 1e-9 in the static press. The rolling rig with it does
**not** produce a usable number, so the five step-climb signatures are still the radial-only
ones. #20 stays open.

**Gates.** Unit suite **535/535** (6 new), ruff at the standing 71.

## 2026-08-09 — A claw's torque capacity and its compliance are the same number, and they trade as `(L/t)²`

**Hypothesis.** The driven rig folded the wheel flat using a `k_t` scaled from the nominal
claw's 134× ratio. Measure `k_t` on the design actually being driven before concluding
anything. TODO #20 step 1.

### The measurement

`TIP_RADIAL` and `TIP_TANGENTIAL` on the driven design's own claw sector — R 60 mm, 12 claws,
6 mm root, taper 0.6, hub 20 mm, TPU_95A at 40% infill, plane strain, δ to 4 mm:

| | measured, first secant | closed form | agreement |
|---|---|---|---|
| radial | **17.20 N/mm** | `EA/L` = 17.03 | +1.0% |
| tangential | **0.2277 N/mm** | `12EI/L³` = 0.2453 | −7.2% |

Both land where a tapered claw should: the radial secant falls from 17.20 to 7.78 N/mm over
4 mm as the column starts to buckle, and the tangential one is flat to 0.6% across the sweep,
which is what a cantilever in small deflection does.

**Ratio 75.5×, not 134×.** The 134× belongs to the *nominal* claw (7 mm root, taper 0.15) and
does not transfer. So the number driven into the rig yesterday, `k_r/134` = 0.1475 N/mm, was
1.5× too soft — and rerunning at the measured 0.2277 N/mm makes no difference: the wheel still
collapses, 691 mm of "contact patch", peak compression 6481 mm.

### Why it could not have been rescued by a better claw

The two closed forms have a ratio with everything cancelled out of it:

    k_r / k_t = (EA/L) / (12EI/L³) = A L² / (12 I) = (L/t)²   for a rectangular section

`L/t = 8.33` here, so `(L/t)² = 69.4` against the measured 75.5 — 9% apart, the taper. **The
stiffness ratio is the slenderness squared, and nothing else.** Not the modulus, not the width,
not the radius.

That closes off the obvious fix. Holding tip deflection to 5 mm under the platform's 21.3 N per
wheel needs `k_t ≥ 4.26 N/mm`, i.e. `k_t/k_r ≥ 0.248`, i.e. **`L/t ≤ 2.01`**. A cantilever twice
as long as it is thick is not a compliant claw; it is a bump on a hub. **A claw is compliant
because it is slender, and slender is exactly what makes it unable to carry tractive load
through tip bending.** The two properties are one parameter.

Measured deflections at the platform load, for the record: 93.5 mm at 21.3 N, 121.7 mm at the
1.661 N·m stall torque — against a 60 mm wheel radius.

### What this does and does not establish

**Does:** for the `T7` family as currently drawn — a straight radial cantilever whose *tip* is
the running surface — compliance and drive-torque capacity cannot be chosen independently, and
at this platform's load they are irreconcilable. That is a design-space result and it belongs
in `04-design-space.md`, not only in the ROM.

**Does not:** it does not say a claw wheel cannot work. It says the tip-loaded cantilever model
of one cannot. Two escapes are visible and neither is tested here. A claw deflecting 90 mm does
not stay a small-deflection cantilever — it folds and then bears along its *side*, which
geometrically stiffens and is plausibly how such wheels actually carry torque. And the load per
claw is shared over however many are in contact, which for a bandless 12-claw wheel is one to
three, not one.

**And it exposes a ROM validity limit, which is the part that matters immediately.** The ring
gives each segment a linear spring on a straight slide. A tangential deflection of order the
wheel radius is outside anything a linearised segment model can represent, so **the ring ROM
cannot be used to evaluate a claw wheel under drive torque at all** — not "gives a pessimistic
number", cannot be used. The 5/5 signatures on this design remain the radial-only ones, and
they are only meaningful because the radial mode stays small (8.05 mm peak, inside its fitted
range).

Filed as **#27**. #20's remaining work is blocked behind it: there is no point adding the
tangential freedom to the signatures until there is a model of the claw that survives the load.

**Gates.** Unit suite 535/535, ruff at the standing 71. No code changed for this entry — it is
a measurement and an identity.

## 2026-08-09 — The claw does stiffen, so the earlier collapse verdict was wrong; the real fault is the segment's kinematics

**Hypothesis.** The linear `k_t` says the wheel folds 93.5 mm under load. Push the claw's own
`TIP_TANGENTIAL` sweep out to a full claw length and see whether it stiffens geometrically
before it gets there. TODO #27.

**Bias declared before running:** the tread node set is a rigid body, so the tip cannot rotate.
A guided tip forces double curvature and *over*-states stiffening, so a null result would have
been strong and a positive one is weakened by the same bias.

### It stiffens, hard

R 60 mm claw, `*STEP, NLGEOM` already on, tangential sweep to 40 mm = one claw length:

| v, mm | F, N | secant, N/mm | tangent, N/mm | secant vs linear |
|---|---|---|---|---|
| 4 | 0.916 | 0.2291 | 0.239 | 1.00× |
| 12 | 2.919 | 0.2433 | 0.281 | 1.06× |
| 20 | 5.590 | 0.2795 | 0.421 | 1.22× |
| 28 | 10.159 | 0.3628 | 0.845 | 1.58× |
| 36 | 21.114 | 0.5865 | 2.343 | 2.56× |
| 40 | 32.992 | 0.8248 | 2.970 | **3.60×** |

Secant 3.6×, **tangent 13×**. The claw rotates toward the load and starts carrying it axially
instead of in bending. It passes the platform's 21.3 N at about **36 mm**, not 93.5 mm.

**So yesterday's verdict was wrong and is withdrawn.** "A structural impossibility" and "the
ring ROM cannot evaluate a claw wheel under drive torque" were both extrapolations of a linear
stiffness through a curve that is anything but linear. The `(L/t)²` identity still holds —
it is a statement about the *small-deflection* stiffnesses and it is still a real design
tension — but it does not settle the design, because the claw does not stay in small
deflection.

Acted on: `law_from_claw_curve` factored out of `ring_from_claw_curve`, and `run_step.py
--tangential` now **measures** the tangential law by a `TIP_TANGENTIAL` sweep on the design's
own claw sector and tabulates it, instead of taking a stiffness on the command line. Measured
rather than chosen, invariant 2, and a table rather than a number because of the row above.

### And the rig still explodes, for a better reason

With the tabulated law the wheel still collapses — 321 mm "contact patch", 6823 mm of radial
compression. That is not the force law any more. It is the **segment's kinematics**.

The two-slide segment translates its tip along a straight line perpendicular to the radius, so
the tip's distance from the hub centre is `√(R² + v²)` and **grows** with splay. A real claw
hinges at its root, so its tip swings on an arc and the distance **shrinks**. On the R 60 mm
claw (root 20 mm, L 40 mm):

| splay, mm | slide model | hinged claw | error | as % of R |
|---|---|---|---|---|
| 2 | 60.03 | 59.98 | +0.05 | +0.1% |
| 10 | 60.83 | 59.58 | +1.25 | +2.1% |
| 20 | 63.25 | 58.19 | +5.06 | +8.4% |
| 36 | 69.97 | 51.94 | **+18.03** | **+30.1%** |

**The sign is the whole problem.** A segment that moves *outward* as it splays presses harder
into the ground, which splays it further. That feedback is built into the element, so no
timestep, joint range or damping fixes it — and at the deflections drive torque produces it is
30% of a wheel radius.

**Where the existing 2-dof numbers stand.** The same check at the flat-plate indentations
already published: **+0.0% of R at δ = 12 mm, +1.5% at 18 mm, +6.4% at 25 mm**. So the static
softening table (11.7% / 48.4% / 57.9%) is sound at 12 and 18 mm and should be read with a
6% geometric caveat at 25 mm. The static press validation against MuJoCo is unaffected — both
models share the same wrong kinematics there, which is exactly why they agreed to 1e-9 and why
that agreement was never evidence about *this*.

### The fix, named

**A hinge at the claw root with a rotational spring, not a slide at the tip.** TODO #20's
original wording offered "a second slide, or a hinge at the root" and the slide was chosen
because it factorises the analytic solve. That was the wrong trade. A hinge gets the arc right
by construction, keeps the claw's length fixed, and is what the measured `TIP_TANGENTIAL`
curve is a moment-rotation curve of anyway.

Both `solve_equilibrium_2dof` and `ring_bodies(tangential=True)` now carry the validity bound
in their docstrings with these numbers, rather than only in this log.

**Gates.** Unit suite 535/535, ruff at the standing 71.
