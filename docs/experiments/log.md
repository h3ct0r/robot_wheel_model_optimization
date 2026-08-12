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

---

## 2026-08-09 — #27: the root hinge, and the damper that was never the element's fault

**Hypothesis.** Replacing the ring's tangential *slide* with a *hinge at the claw root* fixes
the outward-tip feedback recorded above, and the driven claw wheel then survives.

**Result: the hinge is right, and it is not what was breaking the rig.** Both halves are
below. The second is the more useful one.

### The element: built, and checked against a number from outside the ROM

`solve_equilibrium_hinge` gives a bandless ring a rotation at each claw's root, and
`ring_bodies(tangential="hinge")` realises the same thing in MuJoCo. `RingSpec.root_radius_m`
is new and comes from `hub_radius_mm` via `ring_for_design`; the hinge solver **refuses** a
spec without it, because a claw pivoting about the axle sweeps its tip round a circle of
radius `R` and can never indent — a silent rigid wheel.

The solve is one bisection per claw, on the contact angle `ψ = θ + φ` rather than on the
contact force. Written in the force it is a bisection inside a bisection and takes **over 100
seconds** for the table below; in `ψ` the contact condition eliminates the claw's remaining
length, `L - u = c/cos ψ` with `c = (R - δ) - R_root cos θ`, and the whole thing collapses to
`M(ψ - θ) = f_r(u)·c·sin ψ / cos²ψ`. **0.96 s**, and the rigid-hinge limit reproduces
`ring_force_n` to 4e-8 instead of the nested version's 4e-5.

Validated three ways.

1. **Rigid limit.** An infinitely stiff hinge reproduces the radial-only penetrations and
   `ring_force_n` exactly, so the model is a generalisation and not a replacement.
2. **MuJoCo, per segment, reading back its own state** — the #26 pattern. Both KKT conditions
   at 1e-10: the radial residual is 9e-11 of the contact force, and the moment residual
   *divided by* the contact force, which is a length and reads as an error in the lever arm,
   is **6.1e-11 m** on a 40 mm claw. Horizontal contact force exactly 0.0 N (`condim="1"`).
3. **The FEA, on a quantity the ROM does not fit.** This is the one that matters.

That third check exists because the two candidate elements make **opposite predictions** about
something the `TIP_TANGENTIAL` sweep already measures for free: the sweep leaves the radial DOF
of the tread reference node free, so the solver reports where the tip actually went. A hinged
claw must pull its tip *inward* by `L(1 - cos φ)`; a tangential slide pushes it *outward* by
`√(L² + s²) - L`. Measured on the R 60 mm, 12-claw, taper 0.6 design (`LoadCurve.cross_delta_m`
is new, and carries this):

| tip travel, mm | FEA measured, mm | hinge predicts | slide predicts |
|---|---|---|---|
| 1.1 | **+0.133** inward | +0.162 inward | −0.015 outward |
| 20.6 | **+6.207** inward | +6.333 inward | −4.99 outward |
| 36.0 | **+19.673** inward | +22.564 inward | −13.9 outward |

**The claw's tip comes in.** The hinge is within 2% mid-range and 13% at the largest
deflection; the slide has the wrong sign everywhere. #27 is settled by measurement, not by
argument, which is what the CLAUDE.md rule about checking a model against a number from
outside it asks for.

**One correction found while doing it.** On a plane the contact point sits directly under the
capsule's *centre*, so the horizontal lever the floor gets is pivot-to-centre, not
pivot-to-tip. With the hinge at the true root that arm is one capsule radius short — 19.6% at
12 segments, 9.8% at 24 — and short means the modelled claw is *stiffer* in rotation than the
one that was fitted, which is the flattering direction on a fold-over question. Moving the
pivot one capsule radius inboard (`hinge_pivot_radius_m`) makes it exactly `L`. That single
change took the moment residual in check 2 from **6.7e-3 to 6.1e-11**.

The static press then tracks the analytic hinged ring within ±0.7% over 12/24/48 segments and
δ = 2–18 mm, and within ±0.2% everywhere but the shallowest 24-segment point.

**Force barely moves; geometry does.** Against the slide model at the same tip stiffness, the
vertical reaction differs by 0.013% at δ = 12 mm, 0.66% at 18 mm and 1.43% at 25 mm — so every
flat-plate fit taken with the slide stands. The tip radius, as a fraction of `R`: hinge
−0.020% / −0.396% / −1.134%, slide −0.008% / **+0.957%** / **+4.406%**. Same force, opposite
geometry, and a rolling contact depends on the geometry.

### The rig: it was the damper, and the segment mass was the wrong mass

With the hinge in place the driven wheel **still** collapsed, at 2 ms, and the diagnosis is
worth the space because every plausible cause was wrong.

Eliminated, by measurement: the **softening radial law** (a monotone law of the same initial
tangent diverges at the identical microsecond); **friction** (`condim="1"` diverges the same);
**solver iterations** and **noslip** (identical); **segment mass** (2 → 20 g barely moves it,
which also retires "heavier segments" as a remedy here); and **contact altogether** — at the
moment of divergence `ncon == 0` and the wheel is still 0.5 mm in the air, ten milliseconds
from touching anything.

What it was: `qpos` on the hinge joints growing from **1e-22** by a factor of **−8.18 per
step**, seeded by round-off. Turning each applied term off in turn isolated it to the
**damping**, not the spring — spring alone is stable, damping alone is not.

An explicitly integrated dashpot is stable while `c·h < 2·I`. The damping had been scaled by
the segment mass. **That is not the inertia the joint presents.** Every claw's hinge axis is
parallel to the axle, and the axle and carriage are free, so a torque on one claw is reacted by
the other eleven and by the carriage. Measured — unit generalised force in, `qacc` out,
everything else free:

| joint | composite `dof_M0` | effective | ratio |
|---|---|---|---|
| hinge `t0` | 3.2585e-6 kg·m² | **3.027e-7** | 10.8× |
| slide `t0` | 2.0e-3 kg | **3.606e-4** | 5.5× |

and the collective mode across twelve claws is smaller again. The measured amplification of
−8.18 implies `c·h/I ≈ 9.2`, which needs an effective inertia around 2.7e-8 — about 120× below
the segment mass the damping had been scaled by. A textbook instance of this project's
recurring failure: a plausible number standing in for the right one.

**The fix: emit the damping as the joints' native `damping` attribute instead of applying it
through `qfrc_applied`.** `implicitfast` integrates native damping implicitly and
unconditionally. This is *not* the "native joint damping" rejected on 2026-08-09 above — that
was `c ≥ 5 N·s/m` **added** on top of the loss factor to buy stability, which would have been
inventing dissipation. This is the same physically derived number, integrated differently. The
springs stay explicit, because they are nonlinear and tabulated, and `stable_timestep_s` still
bounds them.

### What that unblocks

**The claw wheel runs, and passes 5/5 with the tangential freedom for the first time.**
R 60 mm, 12 claws, taper 0.6, bandless, plane-strain fit, 12 segments, tabulated law,
21.87 N on the axle:

| signature | compliant | rigid |
|---|---|---|
| patch at the step edge, mm | 24.0 | 0.0 |
| mean patch on the flat, mm | 10.9 | 0.0 |
| cost of transport, flat | 0.0676 | 0.0066 |
| loaded radius, N → mm | 5.5→59.56, 21.9→51.86, 43.7→51.19 | — |
| **tallest step cleared** | **30 mm (0.50 R)** | **20 mm (0.33 R)** |

Peak segment compression 8.53 mm against a 6 mm fit, 13% of loaded samples beyond it — an
honest extrapolation, and the number to attack next. The slide element on the same design also
runs now and reports 50.3 mm of step-edge patch against the hinge's 24.0, which is the outward
tip showing up as a longer patch: the reason to prefer the hinge is check 3 above, not this.

**This is the project's first end-to-end claw climb number.** It is *not* the spike's
50-vs-20: that was a banded `T3`, where the band carries tangential load between segments, and
it does not transfer. 30 vs 20 at matched mass, radius and rotational inertia is what a claw
wheel does in this ROM, and it remains a lower bound while the shear-free band and the
tip-loaded contact stand.

**Regression.** `run_step.py --tiny` and `--tiny --law table` are both still 5/5 after moving
the damping into the MJCF, with the same numbers.

**Also fixed in passing.** The static press had the same latent explicit-integration problem
and diverged at δ = 18 mm on 24 and 48 segments once a second freedom existed; it now asks
`stable_timestep_s` too, and `settle_s` is a simulated duration rather than a step count, since
the timestep is no longer fixed. `--tangential-max` defaults to 90% of the claw length rather
than a hard 0.040 m — that default was exactly one claw length on this design, and
`hinge_law_from_tip_curve` correctly refuses a tip that has travelled further sideways than the
claw is long.

**Gates.** Unit suite 566/566, ruff at the standing 71. `ROM_VERSION` bumped: the hinge changes
what a fitted ring does.

---

## 2026-08-09 — #19 and #21: the claw family's two screening gaps, and both answers surprise

Two items filed with the `T7` redirection, closed together because both are about what a
millisecond pre-filter can honestly say about a claw.

### #19 — how few claws? **More than a banded wheel, not fewer**

The item expected to *widen* the bound: `PARAM_BOUNDS["n_spokes"] = (6, 36)` was set for a
banded wheel and rejects the four-claw row of the PaTS-Wheel letter's Table I, and "for `T7`,
fewer, longer and thicker claws are the point of the family". Measured, the answer goes the
other way.

**The quantity nobody had written down: the polygon drop.** With no band the running surface
is `n` discrete tips, so the wheel is a regular polygon — the axle rides at `R` with a tip
straight down and falls to `R cos(π/n)` midway between two, `n` times a revolution. Note
`π/n`, **half** the pitch. The neighbouring `R(1 − cos 2π/n)` is the second-claw *engagement*
threshold and the two differ only by a factor of two inside a cosine, which is precisely the
kind of confusion this project keeps finding; both now have named functions and a test that
states the difference (`polygon_drop_m`, and the 11.4 mm engagement constant).

The rigid drop is not the answer, because compliance changes it in *both* directions. So the
metric is measured on the fitted ring instead: `ride_height_ripple_m` rotates the ring under
the contact point across half a pitch (`segment_angles` gained a `phase_rad`, refused on a
banded spec because the band operator is a circulant on a fixed grid), solves `F(δ, ψ) = 24.5 N`
at each phase, and reports the peak-to-peak axle movement.

On R 85 mm at the platform's 24.5 N, with a linear 13.5 N/mm claw:

| tips | rigid drop mm | ripple mm | ripple/drop | ripple/δ |
|---|---|---|---|---|
| 4 | 24.90 | 23.53 | 0.945 | **12.97** |
| 8 | 6.47 | 5.43 | 0.839 | **2.99** |
| 12 | 2.90 | 1.95 | 0.674 | **1.08** |
| 14 | 2.13 | 1.25 | 0.585 | 0.687 |
| 16 | 1.63 | 0.81 | 0.493 | 0.444 |
| 24 | 0.73 | 0.27 | 0.376 | 0.151 |

**When ripple reaches δ the trailing claw has left the ground entirely.** That crossing is
just past **12 tips**, and repeating it with the two *fitted* claw laws — a 3.7 N/mm taper-0.6
claw and a 13.5 N/mm taper-0.8 one, an order of magnitude apart — puts it at 10 to 12 on both.
The criterion is not sensitive to which claw you ask.

So a passive claw wheel wants `n ≥ 12`. The PaTS-Wheel's four claws are not a counter-example:
that row is gear-driven and the wheel transforms, so its claws are not carrying the load as
passive springs.

Two findings fell out along the way. The soft claw's ripple **exceeds** the rigid polygon drop
by 4× — at deflections of a fifth of the radius the phase changes *how many* claws carry the
load, and that swamps the geometry — so the rigid drop is not a bound in either direction, and
`WheelParams.polygon_drop_mm` says so in its docstring. And the criterion cannot live in the
pre-filter, because it needs a fitted law; what the pre-filter gets is the geometry no
stiffness can rescue, `claw_ride_harshness`, a WARNING at `drop/R > 3.5%` which is `n < 12` on
any radius. **`PARAM_BOUNDS` is unchanged**: six is right for the banded `T3` comparator, which
has a running surface between its spokes, and the claw limit fires only when there is no band.

### #21 — the slenderness proxy, and a complex number hiding behind the commonest design

`slenderness = span / spoke_thickness_mm` read the **root**, the stiffest section of a taper,
so it understated slenderness and erred toward accepting a claw that buckles.

Two candidates were derived rather than chosen. The **compliance-equivalent** thickness is the
uniform cantilever with the same tip deflection under a tip load; the integral
`∫₀ᴸ (L-x)²/(E I(x)) dx` with `I ∝ t(x)³` is elementary and gives

    t_eff = t₀ / Φ(r)^(1/3),   Φ(r) = 3[-ln r + 2r - 3/2 - r²/2] / (1 - r)³

with `Φ(1) = 1`. The **Rayleigh-equivalent** thickness is the buckling-theoretic one: the
fixed-free Euler mode `1 − cos(πx/2L)` weighting `t³` by `cos²`. They are close — 7.08 mm
against 7.11 at `r = 0.6`, 6.47 against 6.71 at 0.4 — so the data decides.

Frictionless claw-sector plate sweep, R 85 mm, L 65 mm, E 4.73 MPa knocked down, measured
plateau load against each predicted Euler load:

| t₀ | taper | plateau N | ÷ Euler(root) | ÷ Euler(compliance) | ÷ Euler(Rayleigh) |
|---|---|---|---|---|---|
| 8.0 | 1.00 | 4.55 | 0.96 | 0.96 | 0.96 |
| 8.0 | 0.60 | 3.10 | 0.66 | 0.95 | 0.94 |
| 8.0 | 0.40 | 2.15 | **0.46** | 0.86 | 0.77 |
| 6.0 | 1.00 | 2.45 | 1.23 | 1.23 | 1.23 |
| 6.0 | 0.60 | 1.60 | 0.80 | 1.16 | 1.14 |
| 6.0 | 0.40 | 1.10 | **0.55** | 1.05 | 0.93 |

A good proxy makes the ratio independent of taper. Across `r = 1.0 → 0.4` the root swings by
**110%**, Rayleigh by 20%, and the compliance form by **10%** — so the closed form is both
cheaper and, on this data, the more faithful. The likely reason it beats the buckling-theoretic
one is that a claw on a plate is not an axially loaded column: its tip slides and rotates, so a
tip-load weighting is nearer the truth than the buckling mode's. `constraints.py` now reads
`WheelParams.effective_thickness_mm`, which is 12% below the root at taper 0.6 and 27% below at
0.25.

**And the test found a bug in the closed form within a minute of being written.** The bracket
is `k³/3 + k⁴/4 + …` with `k = 1 − r`, assembled by cancelling four terms of order one. At
`r = 0.999999` the true value is 3e-19 and the direct expression returns **−166.5** — and
`(-166.5) ** (1/3)` in Python is a *complex number*, silently, for the most common design in
the space (an untapered spoke, where the guard `abs(r - 1) < 1e-9` did not reach). Replaced
with the series `Φ = 1 + Σ 3kⁿ/(n+3)` below `k = 0.1`; the two branches agree to 1e-12 across
the join and both are checked there.

**Open, and deliberately not fixed here: the threshold of 40 is far too permissive for a
claw.** Every design in the sweep above shows a load plateau — root slenderness 8.1 through
26.0 — and every one of those plateaus is *below* the per-claw design load. None is flagged.
That is not a proxy problem, it is a threshold problem, and it interacts with #24's result that
the physical branch is the **stick** one (22.69 N against the frictionless 4.59 N), where the
column mode does not appear. Filed as #28.

**Gates.** Unit suite 584/584, ruff at the standing 71.

---

## 2026-08-09 — #23: a softening segment, driven on purpose, and it is uneventful

**Hypothesis.** `TabulatedLaw` has been able to represent a segment with a negative tangent
since #16 and nothing has ever run one — `run_step.py --tiny --law table` fits a law that
happens not to soften, so it passes 5/5 without testing this at all. Driven deliberately, a
softening segment should snap through dynamically: watch for energy growth, a timestep that has
to fall, and whether `segment_damping_n_s_per_m` is reading the wrong stiffness.

**Result: it runs, and the interesting finding is not the one that was expected.**

Four laws on the tiny design's own 0/2/4/6 mm knot grid, each a legal spring (`TabulatedLaw`
refuses a negative accumulated *force*, not a falling one):

| law | segment tangents, N/mm | signatures | peak compression | axle range |
|---|---|---|---|---|
| monotone (as fitted) | 0.132 / 0.199 / 0.656 | 5/5 | 5.04 mm | 54.96–94.36 mm |
| soft middle | 0.132 / **−0.120** / 0.656 | 5/5 | 5.78 mm | 54.22–93.99 mm |
| plateau then soft | 0.600 / 0.000 / **−0.450** | 5/5 | 5.36 mm | 54.63–94.05 mm |
| peak then collapse | 0.987 / **−0.687** / **−0.275** | 5/5 | 6.15 mm | 55.47–95.05 mm |

No energy growth, no divergence, and **the timestep never had to fall** — `stable_timestep_s`
reads `k(0)`, which a softening branch does not change, and it turns out not to need to.

**Why nothing ran away.** The payload is a dead weight on a free carriage, not a prescribed
displacement. A negative tangent makes the equilibrium statically unstable, so the wheel snaps
through — and then *lands*, because the tabulated law's later intervals and the flat
extrapolation past the last knot always offer a branch to land on. Force control cannot pass a
limit point in a *quasi-static* solve; a dynamic one just falls to the next one.

**The part that was not anticipated: a softening segment need not give a softening wheel.** As
δ grows more segments engage, so the wheel's curve is a sum of a growing number of falling
terms, and on the bandless tiny ring that wins outright for the mild case — the segment tangent
reaches −0.120 N/mm and the **wheel's stays positive at +0.111**. Nothing was unstable at all.
It does not win for the sharp case: −0.687 per segment gives the wheel −1.747, the axle is
crushed from 60 mm to 22.5 mm and a segment compresses 38.7 mm against a 6 mm fit. That is a
collapse and a legitimate result — `fraction_beyond_fit` reports it — but it is worth naming
that the run still graded 5/5, which is the known grading hole recorded under #20.

**The real finding: the damping is ambiguous on a softening law, and it is worth ~8% of cost
of transport.** `c = η k / ω` wants a *storage* stiffness, and a segment on a negative-tangent
branch has none. Two defensible readings remain:

| law | c from `k(0)` | c from the secant | ratio | CoT change |
|---|---|---|---|---|
| monotone | 0.245 | 0.265 N·s/m | 1.08× | +1.7% |
| soft middle | 0.209 | 0.126 | 0.60× | **−7.4%** |
| peak then collapse | 1.187 | 0.783 | 0.66× | **−9.2%** |

**`k(0)` stays**, for two reasons written into the docstring: it is the only reading defined
without knowing the operating point, and the disagreement is an order of magnitude below the
loss factor's own — `TPU_LOSS_FACTOR` is a literature midpoint on a 0.05–0.30 span, a factor of
six, and every cost-of-transport number is already a statement about that.

**And #23's own proposed remedy is refuted.** It suggested deriving the damping from the
*minimum* tangent rather than the initial one. On a softening law that is negative — −0.687
N/mm on the sharpest case here — so `η k / ω` comes out negative and the damper injects energy.
Recorded in the docstring as a refuted suggestion rather than quietly dropped.

**Gates.** Unit suite 588/588, ruff at the standing 71.

## 2026-08-09 — #22 and #12: two deferred items, and in both the suspect list was half right

Both were filed as "real, understood, not on the critical path", and both turned out to have a
decisive one-command answer that had simply never been run. Recorded together because the
shape of the mistake is the same in each: a remedy had been *proposed* in the TODO entry, and
in each case the proposal was right about the mechanism and wrong about what fixing it buys.

### #22 — the coupled tabulated fit stalls

**Hypothesis.** Three suspects were listed, in order: the projection fighting the damping loop;
the piecewise-constant tangent making the inner Newton semismooth, so finite differences are
noise; and eight parameters simply being too many for a finite-difference Gauss-Newton.

**Result: the first, and it is a cost-and-reporting fault, not an accuracy one.** Nominal
design, 24 segments, 8 intervals, seeded from the uncoupled NNLS answer at 15.41%:

| variant | iterations | converged | RMS | residual evaluations |
|---|---|---|---|---|
| as shipped, 60 iterations | 60 | **False** | 14.55% | 604 |
| as shipped, 400 iterations | 400 | **False** | 14.55% | 4004 |
| damping ÷100 on success | 60 | False | 14.55% | 663 |
| damping ÷1000 on success | 60 | False | 14.55% | 722 |
| coarser finite difference, 1e-4 | 60 | False | 14.55% | 604 |
| **free-block step** | **4** | **True** | **14.54%** | **37** |
| free block + damping ÷100 | 4 | True | 14.54% | 37 |
| free block + cost tolerance 1e-6 | 2 | True | 14.54% | 19 |

Three of the eight parameters pin at zero. Clamping the trial point is not enough on its own,
and the reason is worth stating because it looks like slow convergence rather than a bug: a
pinned parameter's Jacobian column is **not** zero — perturbing it upward does move the
residual — so leaving it in the normal equations mixes a direction the projection will
immediately undo into the step computed for every *other* parameter. Solving the step on the
free block only (drop the parameters at zero whose gradient pushes further negative) gives
**4 iterations and 37 evaluations against 400 and 4004**, a 16× reduction, and an honest
`converged=True`.

**But the accuracy does not move: 14.54% against 14.55%.** The TODO framed #22 against the
uncoupled fit's 8.32%, and that comparison is not available to be won — a bandless ring has no
band stiffness for the non-negative table to work around, so it is a different problem, not the
same problem solved better. What was broken was the cost and the flag. Suspects two and three
are **refuted**: coarsening the finite difference and changing the damping schedule both
changed nothing at all.

Tested by a route that does not go through a wheel: on a non-negative *linear* least squares
problem the projected Gauss-Newton is checked against `nnls`, which is convex and therefore
exact. Same cost to 1e-6, 5 iterations, three parameters pinned — two differently-shaped
algorithms agreeing, which is the only kind of check that catches a stall reaching a plausible
wrong place.

### #12 — the contact penalty: the factor moves, and the scaling gets a cap it did not have

**Hypothesis.** Measured on plane strain (2026-08-08), dropping `contact_stiffness_factor` from
20 to 5 to 2 moved the answer 1.3% while turning a diverged frictional run into a converged
one. #12 asked for the same sensitivity on the 3-D tier before the default moved, and asked
separately whether `factor × E / element_size` should be capped, since it grows without bound
as the mesh refines.

**Result, part one: the 3-D tier agrees, and the default moves to 5.** Tiny design, C3D10:

| μ | factor | peak N | k_r at δ_max, N/mm | patch mm | increments | cutbacks |
|---|---|---|---|---|---|---|
| 0.0 | 20 | 4.2896 | 1.6822 | 34.21 | 50 | 0 |
| 0.0 | 5 | 4.2589 (−0.71%) | 1.6504 | 34.21 | 50 | 0 |
| 0.0 | 2 | 4.1793 (−2.57%) | 1.5864 | **38.96** | 50 | 0 |
| 0.6 | 20 | 4.3474 | 1.7126 | 34.21 | 60 | **3** |
| 0.6 | 5 | 4.3144 (−0.76%) | 1.6773 | 34.21 | **50** | **0** |
| 0.6 | 2 | 4.2200 (−2.93%) | 1.6125 | **38.96** | 50 | 0 |

20 → 5 costs **0.7–0.8%** on the reference tier and buys real conditioning: the frictional run
goes from 60 increments with 3 cutbacks to 50 with none. **2 was rejected**, and not on the
force alone — the contact patch jumps 34.2 → 39.0 mm at both friction settings, which is
penetration being reported as conformity. That is the failure this project keeps naming: a
number that moves in a plausible direction for the wrong reason.

**Result, part two: the cap is needed, and the factor cannot substitute for it.** Hold the
factor at 5 and refine the plane-strain mesh at μ = 0.6:

| element | factor | penalty ÷ E, m⁻¹ | outcome |
|---|---|---|---|
| 4.0 mm | 20 | 5000 | diverged |
| 4.0 mm | 5 | 1250 | ok, 3.9960 N |
| 2.5 mm | 20 | 8000 | diverged |
| 2.5 mm | 5 | 2000 | **diverged** |
| 1.5 mm | 20 | 13333 | diverged |
| 1.5 mm | 5 | 3333 | **diverged** |

Lowering the factor does **not** buy fine-mesh robustness — 2.5 and 1.5 mm fail at both. Read
the same rows as absolute penalties and a threshold appears between 1250 and 2000 m⁻¹, so the
prediction is that holding the *penalty* rather than the factor converges at every mesh. It
does:

| element | factor | penalty ÷ E, m⁻¹ | peak N | increments | cutbacks |
|---|---|---|---|---|---|
| 4.0 mm | 5.000 | 1250 | 3.9960 | 57 | 2 |
| 2.5 mm | 3.125 | 1250 | 3.8898 | 84 | 9 |
| 1.5 mm | 1.875 | 1250 | 3.8905 | 94 | 10 |

The two finest agree on the peak force to **0.02%**, and their 2.7% gap to the 4 mm run is mesh
convergence, not penalty. So the divergence tracks the **absolute** penalty and the fix is a
floor on the length in the denominator: `SolverSpec.contact_length_floor_m = 0.004`, which says
"never let the mesh make contact stiffer than a 4 mm element would". Above the floor the
penalty still scales with the element size, which invariant 2 requires — a cap that replaced
the scaling would make soft and stiff designs contact alike.

**Calibrated, not derived**, and the docstring says so: one design, one material, one load
case. 4 mm is relatively finer on a 150 mm wheel than on this 60 mm one, so it needs re-checking
before it is trusted there.

**Both new fields are in the cache key**, automatically — `SOLVER_TIMING_ONLY` names the two
exclusions and everything else is included by default, which is the shape invariant 5 asks for.
Changing the default therefore invalidates cached results taken at 20 rather than silently
serving them, which is the whole reason the factor was put in the key in the first place.

**Two follow-on corrections, both of the same kind — a check that had quietly stopped
checking.** `tests/test_fea_cache.py` probed the key by changing the factor to 5.0, which is
now the default, so it was about to compare a spec against itself. And `verify_fea.py`'s
frictional plane-strain check ran at a hand-softened factor of 5 against a default of 20; with
5 the default it would have been asserting nothing. It now runs the pair — the old uncapped
factor-20 penalty must still diverge, the default must converge — so it asserts the decision
and not just the current number.

The three CLIs that carried a per-tier `factor = 5.0` for plane strain against
`SolverSpec()` for 3-D (`run_step.py`, `run_rom.py`, `render_step.py`) and `explore.py`'s
`CONTACT_STIFFNESS` constant are all gone: there is one penalty for both tiers now and only the
mesh differs.

**Gates.** Unit suite 594/594, ruff at the standing 71.

### Re-running everything the penalty touches, and a defect the re-run exposed

Changing a default that is in the cache key invalidates results rather than corrupting them,
so the question is only which recorded numbers move.

| what | before | after |
|---|---|---|
| `run_rom.py --tiny --mujoco`, best fit at 24 segments | 0.68% RMS | **0.59%** |
| MuJoCo vs analytic ring, δ ≤ 4 mm | 0.03–0.05% | **unchanged** |
| MuJoCo vs analytic ring, δ = 5–6 mm | 4.0–4.8% | **unchanged** |
| `run_step.py --tiny` | 5/5 | **5/5** |
| `verify_fea.py` (non-full) | 11/11 | **11/11** |
| plane-strain `--tiny` flat peak | 3.90 N | **3.88 N** |

The ring-versus-MuJoCo gap not moving is the expected result and worth stating: it is a
comparison of two models of the *same* fitted law, so a change in the law should cancel out of
it, and it does.

**The claw climb moved a bucket, and finding out why exposed a worse problem than the move.**
The headline R 60 mm claw run now reports **60 mm against the rigid wheel's 20**, where the
record said 30-vs-20. `run_step.py --plane-strain` already softened the factor to 5 by hand, so
the *factor* change is a no-op for this design; what reached it is the new floor, since the
2.5 mm section mesh sits below 4 mm. Measured, the same design's fitted law either way:

| | peak N | fit RMS | k(0) N/mm | payload |
|---|---|---|---|---|
| floor at 4 mm | 30.397 | 1.85% | 20.993 | 3.773 kg |
| no floor | 30.493 | 2.13% | 21.254 | 3.793 kg |

**0.3% in peak force and 1.2% in `k(0)`** — and the sweep answers 60 mm on one and **50 mm** on
the other. So the climb metric moves a full 10 mm bucket for a 1% change in the law. That is
the resolution of the sweep behaving as designed, but it means the number must be quoted as a
bucket and a one-bucket gap between two designs is not a ranking. Now written into
`highest_step_climbed`'s docstring and printed under the sweep.

The 30 mm in the earlier record is not reproducible from today's code at either penalty and is
not explained by #12; it predates other changes made the same day. The log said 30 and the log
is the record, so this entry supersedes it rather than editing it.

**The defect.** The profile is monotone — the claw clears 10/20/30/40/50/60 and fails 70/80/90,
the rigid wheel clears 10/20 and fails from 30 — so 60 is a climb and not a bounce over an
obstacle it could not roll over, which the non-monotone predicate does permit. But the sweep's
default range ran to **1.01 R**, and on a 60 mm-radius wheel that ceiling is exactly 60 mm. The
answer was sitting on the top of its own range: correct here only by luck, and for any better
wheel it would have reported `R` and said nothing. That is the failure this project keeps
naming — a default that reads as innocuous and means something else. The range now runs to
**1.5 R**, `default_step_heights_m` is public so a caller can recognise a censored answer, and
`run_step.py` prints `<- AT THE SWEEP CEILING` when the result lands there. On the design that
found it the bound is no longer active: 70 mm fails.

**Gates after all of it.** Unit suite 594/594, ruff at the standing 71, `verify_fea.py` 11/11,
`run_step.py --tiny` 5/5, the claw run 5/5.

## 2026-08-09 — #20's two leftovers: the climb metric gets a profile, and widening the fit makes it worse

Two follow-ups from #20, and the second is a negative result that partly retracts a reading
given earlier the same day.

### The climb metric now reports a profile

`highest_step_climbed` returned one number from a sweep whose own docstring says the predicate
is **not monotone** — a wheel can bounce over an obstacle it cannot roll over. A maximum cannot
tell "cleared everything up to 60 mm" from "failed 40 and flew over 60", and it cannot tell a
real answer from a sweep that ran out of range.

`step_climb_profile` keeps every outcome and `ClimbProfile` exposes `tallest_m`, `censored`
(the tallest cleared height is the top of the swept range) and `monotone` (every height below
the tallest was also cleared). `run_step.py --sweep` prints the pattern:

    compliant     60 mm  [######...] 10-90 mm (1.00 R)
    rigid         20 mm  [##.......] 10-90 mm (0.33 R)

so the claw wheel's 60 mm is visibly a climb rather than a bounce. `highest_step_climbed`
stays as the one-number form and now delegates. Six tests, all pure — the record is a record,
so none of them needs MuJoCo, and the bounce-versus-climb case is asserted directly: two
profiles with the same maximum, one monotone and one not.

### Widening the fitted range removes the extrapolation and breaks the fit

**Hypothesis.** The claw climb ran 13-19% of its loaded samples past a 6 mm fit, so the number
was partly an extrapolation of the radial table. Widen the `RADIAL_FLAT` sweep and re-run; if
the climb moves, the old one was the extrapolation talking.

**It does not move — and that is not the interesting part.** At `--delta-max 0.012` the step
run reports **0% of loaded samples beyond the fitted range** (peak 9.36 mm against a 12 mm fit)
and still **60 mm against the rigid wheel's 20**, same monotone profile. But the fit it rests
on is one the fitter refuses. R 60 mm, 12 claws, taper 0.6, 12 segments:

| δ_max, mm | points | FEA peak N | intervals | fit RMS | `ok` | k(0) N/mm | δ at 24.5 N |
|---|---|---|---|---|---|---|---|
| 6 | 6 | 18.354 | 3 | **3.74%** | **True** | 12.308 | 8.23 mm |
| 6 | 10 | 18.354 | 5 | 5.35% | False | 15.334 | 8.19 mm |
| 9 | 6 | 36.468 | 3 | 13.30% | False | 7.659 | 8.30 mm |
| 9 | 10 | 36.702 | 5 | 10.85% | False | 11.459 | 8.28 mm |
| 12 | 6 | 52.628 | 3 | 10.79% | False | 5.818 | **never** |
| 12 | 10 | 51.298 | 5 | 8.61% | False | 8.027 | **never** |

**Only the original 6 mm / 6-point combination passes the 5% gate.** Everything wider or
better-resolved fails it, and both 12 mm fits produce a law that cannot carry the platform's
own 24.5 N per wheel within 50 mm of indentation — the same thing as the `loaded radius ->
6.00 mm` collapse the widened step run printed and which should have been read as a broken law
rather than a soft wheel.

Two things fall out that matter more than the climb number.

**The design load was already outside the valid fit.** At 24.5 N the ring sits at δ ≈ **8.2 mm**
and the only passing fit was taken to 6 mm. That 8.2 mm is stable across every row while the
fits around it degrade, so it is a property of the design and not of the fit.

**More data makes the fit worse, at fixed parameters-per-datum.** `n_intervals` is
`min(8, len(d)//2)`, so 6 points get 3 intervals and 10 points get 5 — the ratio is unchanged,
and the RMS still rises 3.74% -> 5.35% at the same 6 mm range. That is the deconvolution being
ill-posed, already recorded for *banded* wheels and now showing up on a bandless one at low
segment count, where the patch is one to three claws wide. `--n-points` is not a
post-processing knob: it changes the `*TIME POINTS` grid, so it is a different solve.

**So #20 item 1 does not close.** The honest statement is that the 60 mm climb survives
widening, but no fit of this design over its working range passes the gate, and the narrow fit
that does pass does not reach the design load. Either the ring needs more segments than the
claw count for this topology, or "segments are claws" caps the achievable resolution and the
claw curve needs a different treatment. Filed as #29.

### And an hour lost to a relative path

The sweep above took about an hour when it should have taken minutes on a warm cache. The
scratchpad scripts pass `cache_root="data/cache/fea"` as a **relative** path and were run after
`cd` into the scratchpad, so they solved everything cold into
`scratchpad/data/cache/fea` — 1.5 GB and 52 entries that the repo's own cache cannot see — and
a later probe from the repo root solved two of the same cases again. Nothing the hour bought
was reusable. Not a code defect; a reminder that the cache root is a path and a path is
relative to wherever the process happens to be standing. Scratchpad scripts should pass an
absolute one.

**Gates.** Unit suite 606/606, ruff at the standing 71.

## 2026-08-10 — The whole robot, on rigid wheels, and it climbs three times what one wheel does

**Hypothesis.** Put the platform in `configs/robot.yaml` on four wheels and drive it at a step,
rigid wheels first, to get the chassis rig working and measure what it costs before paying for
four compliant rings. Expect the numbers to resemble the single-wheel rig's.

**They do not, and the gap is the finding.** Same rigid wheel, same friction, same obstacle:

| rig | wheel | tallest step cleared | in radii |
|---|---|---|---|
| single-wheel (`run_step.py`) | R 60 mm | 20 mm | **0.33 R** |
| four-wheel rover (`run_rover.py`) | R 60 mm | 60 mm | **1.00 R** |
| four-wheel rover | R 85 mm | 90–100 mm | **1.06–1.18 R** |

A rigid wheel on the robot climbs **three times** what the same rigid wheel climbs on the test
rig, measured in its own radii. Nothing about the wheel changed. What changed is that three
other driven wheels are pushing while one climbs, and a rigid chassis lets the rear axle lever
the front one up — precisely the effects `step_climb.py` excludes on purpose, and it says so in
its own module docstring: a chassis "would add weight transfer, a second wheel's traction and a
suspension geometry, three more ways for the answer to come out right for the wrong reason."

**Why this matters more than the number.** The spike's headline is a *ratio* — the compliant
claw clears 60 mm against the rigid wheel's 20 on the single-wheel rig, 3x. If the rigid wheel
alone recovers to 1.00 R once it is on a robot, then most of that ratio was the rig's missing
wheels rather than the wheel's compliance. **The comparison has to be re-run on the rover
before the 3x is quoted as a property of compliance.** That is not a retraction of the
single-wheel result, which measures what it says it measures; it is a warning about what it
does *not* transfer to. Recorded as a standing gap and as the reason to finish the compliant
rover rather than stop here.

The rover's own profile is monotone at both radii — `[######......]` at R 60, `[#########....]`
at R 85 — so these are climbs and not bounces. At the failing heights the robot rears: peak
pitch reaches **90°** at a 120 mm step on the 85 mm wheel, the chassis box strikes the riser,
and it ends up behind where it started. Ground clearance is 70 mm and the chassis is a real
contact geom, so bellying out is modelled rather than assumed away.

### What was built

`wheelopt.sim.rover` — a free-jointed chassis box carrying the platform's own mass, dimensions,
centre of mass and inertia, on four hinge axles at `±wheelbase/2 × ±track/2`, each with the
platform's own torque-speed curve. **Nothing dimensional is invented**: the previous rig sized
its drive from `RigSpec.torque_ratio = 1.3`, a per-wheel heuristic that happens to reproduce
this platform's sizing rationale and is not the same statement as `motor.stall_torque = 4.0`.
`scripts/run_rover.py` runs it, sweeps it and films it.

Three enabling pieces, each small:

- **The platform loader was missing every field a vehicle needs.** `wheelbase`,
  `chassis.inertia`, `com_offset`, `ground_clearance_min`, `motor.stall_torque`,
  `motor.no_load_speed` and `operating_point.target_speed` all sat in `robot.yaml` unread.
  They are now required, not defaulted — a default wheelbase would place four wheels somewhere
  plausible and wrong. Four new consistency checks came with them, including one that compares
  the *stated* `chassis.inertia` against the uniform-box formula its own comment cites, since a
  stated value that has drifted from its own provenance is the quiet-wrong-number failure in a
  new place.
- **`ring_bodies` and `coupling_tendons` now take a `prefix`.** MJCF names are global, so four
  wheels nesting the same subtree collide on `seg0`. The tendon builder needs the *same* prefix
  or a banded rover wires every wheel's band to the first wheel's joints — a model that
  compiles and is wrong, which is worse than one that does not.
- **`observe_step` grew a sibling.** `observe_rover` takes the same `observer(k, model, data)`
  hook, so the renderer films the measured run.

### Two scenario bugs, both caught by measurement rather than inspection

**The step was shorter than the run.** A 4 m step box against a robot doing 1.19 m/s for 6 s —
6.9 m of travel — meant the robot climbed the step, crossed it, and drove off the far end. The
final frame then showed it back on the floor at *exactly* its 170 mm ride height, and the climb
predicate read that as "never climbed". Every number in the row was plausible. The step is now
sized from the reachable distance, and a test asserts it outruns the robot.

**The climb predicate passed a leaner.** "Past the step face and above 0.6 of ride height"
is satisfied by a robot nose-up against a 100 mm riser, which is 113 mm above the upper ground
without being on it. It now requires the chassis centre a full half-length beyond the face
*and* within a fifth of its ride height of where it would stand. Both halves were wrong alone.

A third, found by a test rather than a run: `duration_s` below `settle_s` indexed past the end
of the history and died inside the summary with an `IndexError`. `RoverSpec` now refuses it and
says why — no torque is commanded until the robot has settled, so a shorter run measures a
stationary robot.

**Still not modelled, and all three bite the *compliant* rover rather than this one:** the ring
is planar and has no out-of-plane freedom, so a wheel loaded by roll or by dropping off an edge
is rigid; skid steer scrubs and lateral scrub of a segmented capsule ring is validated against
nothing, which is why only straight-line driving is supplied; and the chassis has no suspension
at all, by choice, so the wheel is the whole story.

**Gates.** Unit suite 624/624 (18 new), ruff at the standing 71.

---

## 2026-08-10 — #29: the law was never the problem, and the claw climb is 30 mm, not 60

**Hypothesis.** #29 recorded that no fit of the driven claw design passes the 5% gate over its
working range, and diagnosed it as the deconvolution being ill-posed — the same failure already
established for *banded* wheels, now appearing on a bandless one. Its candidate 2 was to stop
deconvolving: measure **one claw** on the same plate and use that curve as the segment law
directly (`ring_from_claw_curve`, built for #18 and never wired into the step rig). Prediction:
the gate stops failing because nothing is fitted.

**Design throughout:** R 60 mm, width 45 mm, 12 claws, 6 mm root, taper 0.6, hub 22 mm,
bandless, phase −90°, plane strain, μ = 0.8, 12 mm sweep at 10 points. One design, one mesh
setting, both tiers. (Plane-strain force scales linearly with width, so an earlier pass at
30 mm gives forces 1.5× smaller and *identical* relative errors; the numbers below are the
45 mm ones, matching the documented command.)

### The diagnosis was wrong, and the measurement says so twice

**Below second-claw engagement the whole wheel *is* one claw.** Measured, claw sector against
whole wheel:

| δ | whole wheel | one claw | |
|---|---|---|---|
| 1.20 mm | 27.730 N | 27.751 N | +0.1% |
| 2.40 mm | 38.611 N | 38.620 N | +0.0% |
| 3.60 mm | 33.144 N | 33.147 N | +0.0% |
| 4.80 mm | 31.314 N | 31.318 N | +0.0% |
| 6.00 mm | 30.397 N | 30.401 N | +0.0% |

RMS **0.036%** over five points. So over that range there is no deconvolution to be ill-posed
about — one segment carries everything — and whatever the fitter was doing wrong, it was not
that. **#29's stated diagnosis is refuted.**

**What it actually was: under-parameterisation.** `fit_tabulated_law` defaults to
`n_intervals = min(8, len(d) // 2)`. Restricted to the six points below engagement, where the
answer is a single measured curve and the exact law is available for comparison:

| intervals | RMS | passes 5% |
|---|---|---|
| 2 | 22.92% | no |
| 3 (**the default at 6 points**) | 10.42% | no |
| 4 | 1.71% | **yes** |
| 5 | 1.83% | yes |
| no fit at all | **0.0000%** | — |

Three intervals cannot represent a curve that peaks at 2.4 mm and softens for the next ten;
four can. A rule of thumb picked three. That is the whole of the sub-engagement failure.

### But the gate still fails, and now it fails honestly

`ring_from_claw_curve` has no fit error — nothing was fitted, so the honest number is zero and
it means nothing. **`validate_ring` is the replacement**: build the ring from one claw's curve,
then ask it to predict the *whole wheel's*, which is a different experiment on a different mesh
and data the law never saw. The CLAUDE.md watch list asks every model for one check against a
number it did not produce; for a claw ring this is that check, and it is a stronger claim than
the fit error it replaces rather than a weaker one. `RingFit.iterations == 0` marks it.

Held out over the full 12 mm: **20.17% of peak**. The gate fails. Where it fails is the point:

| δ | FEA | radial-only ring | hinged ring |
|---|---|---|---|
| 6.00 mm | 30.397 N | 30.401 N (+0.0%) | 30.401 N (+0.0%) |
| 7.20 mm | 36.644 N | 29.778 N (**−18.7%**) | 29.778 N (−18.7%) |
| 8.40 mm | 52.606 N | 51.582 N (−1.9%) | 30.319 N (−42.4%) |
| 9.60 mm | 64.898 N | 105.566 N (**+62.7%**) | 32.795 N (**−49.5%**) |
| 12.00 mm | 87.063 N | 101.198 N (+16.2%) | 36.261 N (−58.4%) |

**Two idealisations bracketing the truth from opposite sides is a statement about the element,
not about the law.** Both rings run the *same* law, and that law is exact below 6 mm. A radial
slide makes a claw at ±30° a rigid column and overshoots by 63%; a root hinge lets it fold away
and undershoots by 50%. The real claw does both at once, and does something neither models: it
meets a flat plate on its **flank**, not its tip. That shows up independently in the onset —
the FEA has a second claw carrying at **7.20 mm**, 0.84 mm before the geometric threshold of
`R(1 − cos 2π/n)` = 8.04 mm, because contact starts before the tip arrives. CLAUDE.md already
named this as unverified ("a real claw beds onto its side as it folds, and nothing here models
that"); this is the first time it has a number.

### And the headline moves: 30 mm, not 60

`run_step.py --law claw` on the documented design, `--tangential hinge --sweep`:

```
compliant     30 mm  [###......] 10-90 mm (0.50 R)
rigid         20 mm  [##.......] 10-90 mm (0.33 R)
```

5/5 signatures, matched mass, radius and rotational inertia, both profiles monotone, 0% of
loaded samples beyond the law's range, peak segment compression 7.47 mm.

**The 60 mm of 2026-08-09 came from `--law table` on a 6 mm fit; this comes from the exact
measured claw law.** Same design, same rig, same element — a 2× spread in the answer from the
segment law alone. Neither law is validated over the range the wheel uses, so neither number is
better than the other by its own error bar; what argues for this one is that it is *exact*
below 6 mm and the run's peak segment compression is 7.47 mm, so the run spends most of its
time where this law is right and the fitted one never was. **Quote 30-vs-20 and quote the
caveat with it.** The old 60-vs-20 is superseded, and so, again, is the 50-vs-20 banded `T3`.

### Also fixed, and found the same way

A first-crossing search written with `np.interp` — which requires an increasing table and
silently returns a non-crossing when handed a falling one. A claw curve peaks at 2.4 mm and
softens for the next ten, which is exactly why `is_monotone_nonneg` was dropped as a gate in
#16, so this was the one input guaranteed to break it. It did not change the printed answer on
this design, which is the reason to fix it now rather than when it does.

And one of my own, worth recording because the docstring warns about it: the second-claw
threshold is `R(1 − cos 2π/n)`, not twice `polygon_drop_m` = `2R(1 − cos π/n)`. The wrong one
gives 4.09 mm against 8.04 mm — a factor of two, both plausible on a 60 mm wheel. It is now
`ring.second_contact_delta_m` with a test asserting it is *not* the other one, and a second
test that checks it against the ring's own contact set rather than against the same formula.

**Gates.** Unit suite 633/633 (9 new), ruff at the standing 71.

---

## 2026-08-10 — Phase 0, part one: the store, the metric, and S1 driven end to end

**Hypothesis.** Phase 0's remaining bullets are mostly independent, and the store is the one
everything else writes into, so it goes first. The claim to test at the end of it: the Phase 0
gate — *identical θ → identical score* — can be made a **query** rather than a procedure, and
S1 can produce a step-height metric that is continuous rather than bisected.

Both held. What follows is what it cost and the two places a plausible wrong number appeared.

### The store

`wheelopt.store` — append-only Parquet under `runs/`, read through DuckDB. Three calls worth
recording because each rejects an obvious alternative:

**Parquet files, not one `.duckdb`.** A campaign is many workers running for days and expecting
interruption (`13-engineering.md`), and DuckDB takes a single writer lock on its database file.
One file per `append()`, written to `.tmp` and renamed, so a killed worker costs one batch and
a reader globbing `*.parquet` never sees a partial row — the same trick the FEA cache uses.

**`params` / `metrics` / `diagnostics` are JSON columns**, everything else is a real column.
The metric set will change; a rigid schema makes each change a migration, and in practice a
migration means old rows quietly acquire NULLs and nobody can tell which runs predate it.

**`run_id` hashes the inputs and nothing else** — design, scenario, seed, material realisation,
every pipeline version. No metrics, no timestamp. That is the whole determinism gate: two rows
with one `run_id` and different metrics is exactly the failure, and it is *inexpressible* if
the outputs are in the key. `disagreements()` and `repeat_counts()` ship as a pair, because an
empty disagreement list proves nothing if nothing was repeated — a gate that passes because the
campaign never ran a design twice is this project's recurring failure in a new place.

`hashing.py` came out of `fea/cache.py` unchanged rather than being written twice. All 17
cache-key tests still pass, so no key moved.

### The metric, and a 46-metre error bar

`metrics/threshold.py` implements `08-metrics.md`'s threshold fix: a ladder, a logistic fit by
Newton/IRLS in pure numpy, and the height at P = 0.9 with a delta-method standard error.

The first S1 smoke run — three rungs, two seeds — returned **77.2 ± 46029.6 mm**, and `ok` was
**True**. It passed every check that existed: finite, converged, not separated, not censored,
slope the right sign. A 46-metre standard error on a 77-millimetre answer, and nothing about
the number says so. **It is the ladder that says so**, which is why both new guards are scaled
by the ladder rather than by a tuned constant: a standard error wider than the whole ladder
locates the crossing nowhere inside the experiment that was run, and a crossing outside the
rungs is extrapolation from a curve fitted entirely elsewhere. `ThresholdFit.reason` now says
which. That six-run case is pinned as a test.

**And then the fixture turned out to be wrong too, in the same direction.** After adding the
guards, two shape tests failed. The cause was not the guards: my synthetic ladder applied a
flat 15% flip probability at *every* height, which caps success at 0.85, so P = 0.9 is
unreachable and the true crossing is far below the lowest rung. Real terrain noise concentrates
near the cutoff. The fixture now samples from a known logistic, which is both realistic and
better: the fit has an **analytic** answer to recover, so `test_it_recovers_a_known_crossing`
is a check against a number the fit did not produce.

`metrics/aggregate.py` is CVaR at 25% (invariant 7), and `Direction` is a **required** argument.
The worst quartile of a maximised metric is the lowest and of a minimised one the highest; a
default would make the wrong answer the easy one to write, and the wrong answer is a plausible
number in the right units that ranks designs backwards. The boundary sample carries fractional
weight so a design that lost a seed to a diverged run stays comparable with one that did not.

### S1, and the lateral twin of an old bug

`sim/s1_step.py`: ten rungs × eight terrain seeds, constant-throttle controller, one row per
rung. **A terrain seed is a terrain, not a coin flip per run** — the same seed gives the same
friction and approach angle at every rung, so the ladder makes one world progressively harder.
Re-sampling per rung would make eight seeds eighty conditions and the curve would measure the
sampler as much as the wheel.

Each rung gets its own scenario name (`S1_step/h=0.050`). Without that, eighty runs at eight
seeds collide into eight `run_id`s and the determinism gate reads them as one evaluation
repeated ten times with ten different answers — a self-inflicted gate failure.

The rover gained an approach angle for this, and **the quaternion was the easy half**. At 15°
a 6.9 m run drifts 1.8 m off centre, and the step's y half-width was fixed at 1.5 m — so the
robot would have climbed the step and then driven off the *side* of it. That is precisely the
step-shorter-than-the-run bug of the day before, in the axis nobody was looking at. The step is
now sized from `reach·sin(yaw)` and there is a test.

### The result

Full ladder, R 85 mm rigid wheels, 80 runs in **23 s**:

```
   20 mm  8/8      80 mm  3/8       140 mm  0/8
   40 mm  7/8     100 mm  1/8       160 mm  0/8
   60 mm  6/8     120 mm  0/8       180-200 0/8
```

**44.7 ± 9.1 mm at P = 90%**, usable. A clean sigmoid with no artificial noise — the grading
comes from friction 0.30–1.00 and ±15° of approach, which is what `08-metrics.md` asks S1 to
randomise over. Note it is *not* the `run_rover.py --sweep` answer of 100 mm at R 85: that
sweep runs at μ = 1.0 square-on, and this is the robust number.

**The gate, run for real:** `--repeat 2 --gate` gives 160 rows, **80 repeated `run_id`s, zero
disagreements**, in 47 s. Honest limitation — this is one machine, one process. `11-phases.md`
asks for *two machines, two days apart*, and that is untested. What exists is the mechanism and
a demonstration that the pipeline is deterministic under repetition here.

**Gates.** Unit suite 704/704 (46 new), ruff at the standing 71.

---

## 2026-08-10 — The step-climb metric on the rover cannot rank wheels, and flat ground can

**Hypothesis, stated first.** Lowering `PARAM_BOUNDS["n_spokes"]` from 6 to 3 lets a
deliberately bad wheel into the search, and a deliberately bad wheel should lose somewhere
measurable. Which metric it loses on was the open question.

### It does not lose on step climb, and neither does a rigid cylinder

`run_rover.py --sweep`, R 60 mm, taper 0.6, bandless, `--law claw`:

```
   3 claws       60 mm  [######......] 10-120 mm  (1.00 R)   held out RMS  0.02%
   6 claws       60 mm  [######......] 10-120 mm  (1.00 R)   held out RMS  0.03%
  12 claws       60 mm  [######......] 10-120 mm  (1.00 R)   held out RMS 20.17%
  rigid cylinder 60 mm  [######......] 10-120 mm  (1.00 R)
```

**Four different wheels, one answer.** This is not a null result about wheel design; it is the
rover's step-climb metric saturating, and it is the same effect as #30's 3x — three driven
wheels push while one climbs and a rigid chassis levers the front axle up, so the wheel is a
small term. A 10 mm bucket cannot see what is left.

(The RMS column is a free consistency check on #29 rather than the point: at 3 claws second
engagement is `R(1 − cos 120°)` = 90 mm, far outside the 12 mm sweep, so the whole wheel is one
claw throughout and the held-out validation is near-exact. At 12 claws it is 8.04 mm and the
sweep crosses it. Exactly what #29's story predicts, from a direction #29 did not look.)

### Flat ground separates them 4.5x

`RoverSpec.step_height_m = 0` is now a **scenario**, not a degenerate step: no box is emitted
at all, and the run measures objective 3 from `08-metrics.md`, RMS vertical chassis
acceleration. A bandless wheel runs on discrete tips, so it is a polygon and the axle rises and
falls once per tip — the cost compliance is supposed to buy back.

Same designs, 6 s at full throttle, R 60 mm:

| wheel | harshness | polygon drop | loaded ripple | axle work | mean speed |
|---|---|---|---|---|---|
| 3 claws | **22.64** m/s² | 30.0 mm | 29.08 mm | 46.1 J | 0.81 m/s |
| 6 claws | **10.31** m/s² | 8.0 mm | 7.38 mm | 41.7 J | 0.71 m/s |
| 12 claws | **5.00** m/s² | 2.0 mm | 1.50 mm | 12.0 J | 0.83 m/s |
| rigid cylinder | **0.00** m/s² | — | — | 3.9 J | 0.84 m/s |

Cost of transport separates them as well, and harder: the 3-claw wheel spends **12x** the axle
work of the cylinder to cover the same ground at the same speed.

**Three numbers from three places, on purpose.** The harshness is MuJoCo's `qacc` on the
chassis free joint. The polygon drop is closed-form trigonometry on the tip count, with no FEA
and no dynamics in it. The loaded ripple is the ring solving `F(δ, ψ) = 24.5 N` per phase, with
a law but still no dynamics. The standing rule is that a model needs at least one check against
a number it did not produce; these are two, and they track.

They also say something the harshness number alone does not. At 12 claws compliance cuts the
ripple **25%** below the rigid polygon; at 3 claws it cuts it **3%**. A wheel only rides
smoother than its own polygon if it deflects by something comparable to the drop, and at 24.5 N
per wheel this design deflects about 1 mm against a 30 mm drop. The bad wheel is not bad
because it is stiff — it is bad because three tips is a triangle.

### Two things that had to be right for the measurement to mean anything

**The acceleration is read from the solver, not differenced.** `qacc` on the free joint's z
DOF. At a 5e-4 s timestep, second-differencing the height history multiplies contact noise by
4e6 and measures the integrator. The check that it is the right quantity is that a robot
standing on the floor reads ~0 rather than −g: the contact force balances gravity and an
accelerometer bolted to the chassis would agree. Pinned as a test.

**The launch transient is excluded.** The largest vertical acceleration in the whole run is the
squat as the robot leaves rest at stall torque, which is a fact about the motor. Harshness is
quoted over the second half of the driving phase only; including the transient would rank
drivetrains.

And one thing that had to be a scenario rather than a number: **flat ground emits no step
geom**. A zero-height box is a MuJoCo compile error and an epsilon-height box is a lip the
robot bumps over — contributing exactly the acceleration being measured, from the scenario
instead of from the wheel. `climbed` is also forced False there, because both halves of the
climb test pass on flat ground the moment the robot has driven a metre.

### Negative result: the few-clawed numbers are extrapolated, and cannot be un-extrapolated

A 3-tip R 60 wheel has a 30 mm polygon drop. The FEA sweep behind its segment law runs to
`--delta-max`, 12 mm by default, and `TabulatedLaw` extrapolates on its last slope without
complaint — the project's characteristic failure, a plausible number outside the range that
produced it. The run now prints `EXTRAPOLATED` with the ratio.

Widening the sweep **12 → 18 mm** moved the 3-claw answer 24.65 → 22.64 m/s², about 8%, so the
ranking survives the correction. **35 mm cannot be measured at all**: CalculiX stops at t=0.590
after 10 cutbacks, which is itself a statement about a claw pressed half its own radius rather
than a solver setting to tune. So the 3-claw harshness is quotable as a bucket and its sign is
safe; its second digit is not.

### What this metric does not do

It has **no counter-pressure of its own**. Harshness alone ranks 36 claws above 12 above 3,
monotonically and forever, and the rigid comparator — a smooth cylinder — wins outright at
0.00. The floor is there to prove the metric is not measuring the solver, not to propose a
wheel. The pressure back the other way is in the other three objectives, which is exactly the
argument in ADR-0006 for never scalarising them.

Also open: this is one speed on one surface. `08-metrics.md` asks S7 for a washboard swept over
amplitude and wavelength, where a compliant wheel should beat a rigid one rather than merely
lose by less — nothing here demonstrates that, because a smooth cylinder on a smooth plane is
unbeatable. Filed as #33.

**Gates.** Unit suite 778/778 (6 new in `test_rover.py`), ruff at the standing 71.
Also fixed in passing: `ring.polygon_drop_m`'s docstring claimed the rigid drop was an upper
bound on the compliant ripple. `WheelParams.polygon_drop_mm` says the opposite and cites a
measurement — 4x this value on an R 85 mm, 8-claw, 3.7 N/mm design. Two docstrings for the same
formula disagreeing is the same failure one level up again; the measured one is right.

---

## 2026-08-10 — A figure per geometry parameter, and what it turned up about the bounds

`scripts/plot_geometry.py` draws every wheel geometry parameter across its own range, one
figure each, with each design's **screening verdict** printed under it. Twelve figures, a few
seconds, numpy only — the panels come from the same `spoke_outline` the solid is extruded
from, so this runs on a machine with no CAD kernel and no solver.

The point was documentation. `PARAM_BOUNDS` and `check_design` describe the search space in
numbers, and a number does not say what a 0.25 taper looks like or where a design stops being
printable. It turned out to be a test.

### What it found

**`rim_thickness_mm` and `spoke_thickness_mm` are both searched from 1.2 mm, and TPU cannot
print below 1.6.** `PlatformLimits.min_wall_thickness_tpu_mm` is 1.6; 1.2 is the *rigid*
minimum wall. Measured at R 60, both fields set together:

```
   1.2 mm   spoke_min_wall INFEASIBLE, rim_min_wall INFEASIBLE
   1.5 mm   spoke_min_wall INFEASIBLE, rim_min_wall INFEASIBLE
   2.0 mm   clean
```

**And then reading `params.py` changed what this is.** The spoke's bound is *deliberate* and
says so in place: it sits below the wall "so that `spoke_min_wall` stays a live check rather
than being made unreachable by the range". That is a real argument — a constraint no sample can
violate has stopped testing anything — traded against evaluations spent on rejections. So the
figure found a documented trade, not a bug, and the first draft of this entry called it a bug
because the figure was read before the source.

What survives is narrower and still worth an item: **`rim_thickness_mm` carries no such note**.
Same numbers, no stated reason. Filed as **#34**, whose work is to decide and record which it
is rather than to move a number.

Two other red bounds in the same figures are *not* the same finding and are recorded as such:
`n_spokes` 24 and 36 hit `interspoke_gap`, and `tread_depth_mm` 4 exceeds a 3 mm band. Those
depend on radius, thickness and band — a scalar bound cannot know them. The wall one depends
on nothing.

### Two views, because one of them is a projection

`draw_wheel_profile` is new: the axial section, r against z. It exists because `width_mm` is
the direction the mid-plane view projects away and `tread_depth_mm` cuts grooves whose axis is
that direction, so **sweeping either against the mid-plane section alone gives a row of
identical pictures** — which reads as "this parameter does nothing". That is this project's
recurring failure in a figure instead of a value, and the two sweeps that need it now say
`AXIAL section` on their own caption.

**And writing it reproduced the failure it was written to prevent, twice, both caught by
tests.**

- The first version drew tread grooves only on a banded wheel. `_cut_tread` is gated on
  `tread_depth_mm` alone, so a **bandless** design with tread has grooves the drawing did not.
- Fixing that by splitting the block at `rim_inner_radius_mm` was worse: bandless, inner and
  outer radius are equal, so the land rectangles had zero height and the groove-floor
  rectangle had *positive* area below the surface. **Tread added material.** Area went 1440 →
  1474 mm² as the depth rose. The drawing disagreed with the solid in the direction that looks
  fine.

Now one polygon walks the grooved surface from hub to tread, and the test asserts area *falls*
with depth. It also asserts `TREAD_GROOVES` equals the count read back out of
`compliant_spoke.py` by regex — a mirrored constant that cannot be imported (that module needs
OCCT) is a constant that drifts.

### Also corrected

The figures are gitignored. They regenerate in seconds from the centreline layer, and a
committed set is a set that quietly disagrees with the bounds it claims to draw.

`tests/test_cli_help.py`'s script loader did not register the module in `sys.modules` before
executing it, so a script containing a `@dataclass(slots=True)` failed with
`AttributeError: 'NoneType' object has no attribute '__dict__'` — a message about nothing.

**Gates.** Unit suite 783/783 (5 new), ruff at the standing 71.

---

## 2026-08-11 — `T7L`: an L-shaped claw, and the ROM refusing to pretend it can model one

A new topology on request: a claw whose tip turns through a right angle so it lies along the
running surface. `WheelParams.tip_hook_mm` — a radial **leg**, a filleted **bend**, a
tangential **foot**. Zero is the default and reproduces the plain `T7` claw byte for byte, so
no `design_hash` on record moves.

### What it is for, as a closed form

A radial claw touches at a point, so a bandless wheel is a polygon and its axle drops
`R(1 − cos π/n)` once per tip — the ride harshness measured yesterday. A foot spreads contact
over `β = |tip_hook_mm|/R`, so the axle only falls across the gap between feet:

```
   R 60 mm, twelve claws, taper 0.6

   foot   0 mm    polygon drop  2.04 mm      contact patch   3.6 mm (the tip)
          6 mm                  1.34 mm
         12 mm                  0.78 mm                     12.0 mm (the foot)
         20 mm                  0.27 mm
         30 mm                  0.00 mm   <- feet meet: interspoke_gap and hook_reach
                                             both reject it, at a pitch arc of 31.4 mm
```

### Two pieces of geometry that are load-bearing rather than cosmetic

**The bend radius.** An outline is the centreline offset by half the local thickness, and
offsetting a *corner* of centreline radius `ρ` gives an inside face of radius `ρ − h`: at
`ρ = h` it collapses to a point and below it the polygon turns **inside out**. OCCT may refuse
such a face — or accept it into a solid with a reversed patch whose volume is still plausible,
which is this project's standing failure. So `hook_bend_radius_mm = 0.75 t_tip` against a
half-thickness of `0.5 t_tip`, a 1.5x margin, capped at half the foot so a short hook does not
become all fillet. `verify_cad.py` §11 checks the outline for self-intersection with its own
O(n²) segment test, **independently of the kernel**, because a check that asks OCCT whether
OCCT was happy is not a check.

**The foot is built in polar, not in the spoke's local Cartesian frame.** A foot at constant
local `u` is a *chord*, and a 20 mm chord of a 60 mm circle stands **3.2 mm** proud of it — so
the foot would pierce the running surface, `_clip_to_radius` would eat it from outside, and on
a tapered tip the outline would then cross itself. The foot follows the circle because the
ground does. Its centreline sits half a tip thickness inside `outer_radius_mm` so the
*material* lands on the surface: measured reach 59.97957 mm against R 60.

### The mirror test was wrong, at 2.3e-5

`verify_cad.py` §11 asserted that flipping `tip_hook_mm` leaves the volume unchanged. It failed
at a relative 2.25e-5 — and the geometry was right. With a **bowed** leg, `(+bow, +foot)` and
`(+bow, −foot)` are a C and an S: two genuinely different claws. The true mirror flips the
curvature too, and then the volumes agree to **9.9e-16**. On a straight leg, flipping the foot
alone agrees to 7.1e-16.

Both directions are now pinned, and the second is the one worth having: flipping only the foot
must **not** come out equal, or a hook that silently ignored its sign would pass. A 2e-5
discrepancy is exactly the size that gets waved through as tolerance when it is really a wrong
test.

### The contact-patch line, corrected for the third time

The `no_shear_band` warning quotes how wide the ground contact is. It read `spoke_thickness_mm`
until claws tapered, whereupon it overstated the patch by `1/taper`; it was moved to
`tip_thickness_mm`, which is right until a *foot* appears — an L claw touches over its whole
foot, 12.0 mm against the 3.6 mm the tip reports, a factor of 3.3 while looking entirely
reasonable. It is now `WheelParams.contact_patch_mm`, so there is no fourth time.

### What arrived for free, and what did not

Everything downstream reads `spoke_outline`, so the CAD solid, the mid-plane figure and the
**2-D FEA tier** all took the new topology without a line of change. A 12 mm foot on the R 60
twelve-claw design: 6952 elements, 90 increments, 10 cutbacks, 158 s, sweep completed, buckling
limit point at **30.9 N** — below the 61.2 N the constraint asks for, so this particular design
is not a good one, which is a result rather than a problem.

**Noted, one design each, not a finding:** at the same settings the *plain* claw **diverges**
where the L claw completes. If it holds up, a spread contact is easier on the contact solver as
well as on the ride.

**The ring ROM does not take it, and now says so.** Every segment element here — radial slide
and root hinge alike — carries contact at a **point** on the segment's own radius. A foot beds
along an arc and the load travels down it as the wheel rolls. A ring fitted to a `T7L` design
would run, produce a curve, and describe a plain radial claw of the same length: not a crash, a
plausible number about a different wheel. `build_ring` refuses it by name, before any solver
time, and names #35. This is **#31 arriving by design rather than by accident**, which makes
#31 harder to defer: for `T7` the flank contact is an error above second-claw engagement, for
`T7L` it is the first-order behaviour at any load.

**Gates.** `verify_cad.py` **60/60** (12 new checks, section 11). Unit suite **797/797**
(14 new), ruff at the standing 71.

---

## 2026-08-11 — #31: the onset has a closed form, and it is not what was wrong

`TODO.md` #31 opened with a discrepancy and no explanation: the FEA engages its second claw at
**7.20 mm** where the ring's geometry says **8.04 mm**, and above that point the two available
elements straddle the FEA from opposite sides, **+62.7%** (radial slide) and **−49.5%** (root
hinge) at δ = 9.6 mm on the R 60 mm, twelve-claw, taper 0.6 design. The item's instruction to
itself was *measure before choosing*. This is that measurement.

### The onset: a tip is a corner

A claw has thickness. Its deepest material is not the tip *centre* on the segment's own axis
but the tip **corner**, half a thickness off it, whose downward extent is

```
   d(θ) = R cos θ + h sin|θ|          h = half the tip thickness
```

so a claw away from the contact point touches `h sin|θ|` of indentation **early**. On this
design, `h = 1.8 mm` and `θ = 30°`:

```
   tip centre   R cos 30                 = 51.962 mm  ->  engages at 8.038 mm
   tip CORNER   R cos 30 + h sin 30      = 52.862 mm  ->  engages at 7.138 mm
   FEA                                                     7.200 mm
```

**7.138 against 7.200** — one FEA sample apart, on an effect of 0.84 mm. A check against a
number this model did not produce, which is what the watch list asks of any model change.
`RingSpec.tip_half_thickness_m` puts it in both solvers (`rom-0.7.0`); zero reproduces the
point-tipped ring exactly, so only bandless specs built through `ring_for_design` /
`ring_from_claw_curve` move.

For the hinge the corner enters **twice** — the contact condition becomes
`L − u = (c − h sin ψ)/cos ψ`, and the lever of a vertical force about the root becomes
`(L − u) sin ψ − h cos ψ`, both from the same virtual work as the `h = 0` derivation with the
contact point moved onto the corner that actually touches. The bracket's upper end,
`arccos(c/L)` at `h = 0`, becomes `atan2(h, L) + arccos(c/√(L²+h²))`.

### And it does not fix the straddle. That is the result.

```
   delta mm    FEA N    slide  before -> after     hinge  before -> after
     7.20      36.64          -18.7%    -8.4%          -18.7%   -18.2%
     8.40      52.61           -1.9%   +87.7%          -42.4%   -37.2%
     9.60      64.90          +62.7%   +74.7%          -49.5%   -45.6%
    10.80      74.98          +45.8%   +37.1%          -53.7%   -50.6%
    12.00      87.06          +16.2%   +13.7%          -58.4%   -55.8%
```

The hinge improves uniformly, by 3–5 pp, and is still about half the truth. The slide gets
**worse** — and that is the informative half: its −1.9% at 8.40 mm was a too-late onset partly
cancelling a too-stiff element, so the old agreement was luck and removing the first error
exposed the second. **Contact onset was never the cause.** The cause is *bedding*: a claw lies
down along its flank, loaded in bending over a patch that travels down it as the wheel rolls,
and no correction to a point-contact element reaches that.

### A second source of truth, found by the correction failing to do anything

The first version of this change moved the hinge's numbers and left the radial model's
**byte-identical**. `solve_equilibrium` carried its own copy of the interference expression
rather than calling `penetrations`, so only one of the two learned about the corner. A formula
in two places is a formula that will disagree with itself, and this one did, silently, across a
change that moved engagement by 0.9 mm. Both now go through `_interference`, and a test asserts
they agree.

### The bound, measured rather than asserted

`BuiltRing.validity_delta_m` is second-claw engagement, and `RoverResult.multi_contact_fraction`
is the share of a run in which some wheel had more than one claw carrying. Measured on the
R 60 twelve-claw design:

```
   flat ground     >1 claw sharing   9% of the driving phase   peak compression   4.11 mm
   40 mm step      >1 claw sharing   8%                        peak compression  21.98 mm
```

Two different failures, reported separately: sharing is about the **element** (+75%/−46% past
one claw), compression is about the **law** (measured to 12 mm, so the step run extrapolates
1.8x). A flat run is inside both. A step run is comfortably inside the first and well outside
the second, which is the opposite of what the item's title would suggest.

**The first version of that fraction read 70% and was wrong.** The threshold was an absolute
1 µm of compression, which counts a claw still ringing down after it has left the ground. The
static ring says this wheel carries on one claw at every phase but the half-pitch crossing —
checked directly, 1 claw at 0 to 0.4 of a pitch and 2 at 0.5 — so a flat run cannot be 70%. The
criterion is now a **share** of the deepest claw on the same wheel (10%), which is scale-free
and reads 9%. Caught by comparing the sim against the static ring; it would have been quoted as
a headline otherwise.

**Gates.** Unit suite 809/809 (12 new), ruff at the standing 71. `rom-0.7.0`.

---

## 2026-08-11 — S7: on a washboard the sign reverses, and it is not close

`TODO.md` #33 said the harshness objective had no scenario where compliance could *win* — flat
ground scores every compliant wheel against a smooth rigid cylinder's unbeatable 0.00 m/s².
S7's washboard now exists: `RoverSpec.washboard_amplitude_m` / `washboard_wavelength_m`,
`run_rover.py --washboard --wavelength`.

**Hypothesis, stated first:** on a corrugation a rigid wheel must follow the ground and a
compliant one need not, so the compliant wheel should read lower — the reverse of every
harshness comparison so far.

### Result: 4.8-6.9x, everywhere in the sweep

10 mm peak-to-trough, R 60 mm, matched mass, full throttle:

```
   wavelength      60 mm    100 mm    200 mm    400 mm
   rigid           40.56     43.31     36.48     33.83   m/s2 RMS vertical
   12-claw          6.43      6.27      6.20      7.10
   ratio            6.3x      6.9x      5.9x      4.8x
```

Three things make it a result rather than a number. The compliant wheel is **not slower** —
0.67–0.86 m/s against the rigid 0.61–0.78 — so it is not smooth by being slow, which is the
first thing `mean_speed_m_s` exists to rule out. The metric still **ranks within the family**:
the 3-claw wheel reads 10.04 against the 12-claw's 6.27, so bad compliance loses to good
compliance on the same terrain. And the rigid wheel's own flat-to-washboard jump (0.00 → 43)
shows the terrain is doing the forcing, not the solver.

**The #31 machinery earned its keep on day one.** The compliant runs carry 20–53% of the
driving phase with two claws sharing — the element-unvalidated regime — and at 60 mm wavelength
the peak compression grazes 12.57 mm against a law measured to 12. Both are printed on the
run's own output. So: **the sign is the result; the second digit is not.** The 6.9x could be
4x or 10x when #31's element lands; it is very unlikely to be 1x, because the element errors
straddle zero from both sides while the margin is a factor of several.

### Construction notes, both of which are scenario-integrity guards

**Boxes, not a heightfield.** An `hfield` wants its elevation data patched in *after*
compilation through `model.hfield_data`, and `build_rover_mjcf`'s contract is that the string
is the model. Eight boxes per wavelength puts the stair-step sampling error at 3.8% of the
half-amplitude. Sub-half-millimetre slivers are skipped — they flicker in the contact solver.

**The strip enters at a trough.** Starting at a crest puts a full-amplitude face at the entry,
and the transient of hitting it would be charged to the corrugation — the scenario contributing
the acceleration the wheel is being scored on, which is the same failure the flat-ground
scenario guards against with its no-epsilon-step rule. A step and a washboard together are
refused outright: S1 is the step and S7 is the corrugation, and nothing defines the mixture.

Still open under #33: the amplitude x wavelength sweep proper (this is one amplitude), and
terrain seeds over it for the CVaR aggregation.

**Gates.** Unit suite 816/816 (7 new), ruff at the standing 71.

---

## 2026-08-11 — Phase 0 closes, and three small items with it: the gate goes cross-machine, #34, #28, #32

Four pieces in one sitting, each small, none glamorous. Recorded together because they share a
date and a theme: every one is a check being made real rather than asserted.

### CI exists, and the cross-machine determinism gate is now a live experiment

`.github/workflows/ci.yml`, two jobs. `tests` is the ordinary signal: unit suite plus ruff on
every push, no CAD/FEA kernels — the suite is designed to skip those layers, and keeping conda
out is what keeps the workflow under the plan's five-minute budget. `determinism-gate` is the
Phase 0 gate's real claim being tested at last: **identical θ → identical score on two
machines**. `run_s1.py --manifest-out` wrote three designs' S1 ladders (R 60/85/100, 3 rungs ×
4 seeds, rigid) to JSON manifests on this macOS arm64 machine, committed under
`tests/fixtures/ci/`; the runner (Linux x86-64) re-runs the same ladders with `--manifest` and
compares **bit for bit** (`store.manifest_from_records` / `compare_manifests`; floats survive
JSON exactly, since `json` writes float64 with `repr`). If that job fails while `tests`
passes, it is not a broken build — it is the gate *finding something*, namely cross-platform
floating point moving a trajectory, and it goes in this log. The workflow itself is untested
until pushed; the manifest check passes locally against its own references and fails against a
perturbed ladder.

**The gate caught a real bug on its first day.** A ladder run at `--duration 5` produced the
**same run_ids** as the 6-second reference with different metrics inside — because
`S1Config.rung_name` carried only the height, so duration, throttle and the friction range
were all outside the key. Invariant 5, violated quietly since S1 was built, and it read as
non-determinism when it was actually two experiments sharing a name. The rung name now
carries a digest of everything that shapes the run, with two exclusions named per the
invariant's own rule (`n_seeds`, `heights_m` — a row's own seed and height identify it; the
population does not change what it measured). The default design label also gains the width it
was silently omitting. A gate run is now judged by the gate, not by the threshold fit — a
3-rung CI ladder honestly cannot locate P=0.9, and failing the job for that would make the
gate unrunnable at exactly the size CI affords.

**Phase 0's three unbuilt bullets are recorded as #36, not built**: `T0` in CAD (no printer
run needs it before Phase 4, and it does not fit `WheelParams`), CoACD→MJCF (no mesh needs
decomposing until a `T1`/`T2` exists), Hydra (eleven argparse CLIs work; the optimiser's sweep
configs are what Hydra is actually for). Each has a named trigger.

### #34 closed: both thickness floors are deliberate, stated once, with the cost measured

The spoke bound's rationale — the range must express a design `spoke_min_wall` rejects, or the
check can never fire — applies verbatim to the rim, and is now stated once in
`04-design-space.md` §Manufacturing with both `PARAM_BOUNDS` comments pointing at it. The
cost being traded: 0.4/6.8 ≈ **5.9% of a uniform sweep per field**, rejected in milliseconds
by screening. The trigger for revisiting is an optimiser measurably concentrating near the
wall; the remedy then is a material-dependent bound, never a silent raise of the floor.

### #28 closed: the stick branch has a limit point, and slenderness cannot see it

The frictional deep sweep the item asked for, run at last: claw sector, μ = 0.6, 12 mm, tapers
1.0/0.6/0.4 on the R 60 twelve-claw design.

```
   taper   slenderness   stick limit point       vs 61.2 N (2.5x nominal)
   1.0        6.3          105.4 N at 3.8 mm       passes
   0.6        7.2           39.4 N at 2.2 mm       FAILS
   0.4        7.8           22.7 N at 1.5 mm       FAILS, barely 1x nominal
```

A **4.6x collapse** in buckling load across which the slenderness proxy creeps 6.3 → 7.8. No
threshold on an axis that flat can rank the family, so the answer to "is 40 too permissive"
is that **the number is not the problem — the axis is**. The check that does the job is
`fea_buckling`, which measures each design's own limit point and fails all three tapered
claws where the warning stays silent (it fired unprompted on the T7L run yesterday). The
warning is kept for the corner it was written for — a 1.6 mm strut on R 100 reads 48 — and
its comment now carries this measurement. Deeper than ~12 mm the tapered claws diverge at
15–17 mm after 8–10 cutbacks, consistent with snap-back, which CalculiX has no arc-length
solver to traverse: that is the boundary of what this rig can measure, recorded as such.

### #32 advanced: the cross-validation chooser exists, opt-in, default untouched

`fit.n_intervals_by_cv`: leave-one-out over the interior points (endpoints anchor the origin
and the span — a fit must not be scored on its own extrapolation), ties to the coarser table
within 5%. On a curve generated from a known 4-interval law it recovers **4** where the length
rule picks 3 — the exact miss that opened #32. The item stays deferred for the same reason it
was: switching the *default* re-fits every banded result on record, and that re-run has not
been done. The tool now exists for the day it is.

**Gates.** Unit suite 824/824 (15 new), ruff at the standing 71.

---

## 2026-08-11 — The review lands: objectives corrected, ground truth re-decided, two choke points re-scoped

A project review against the restated objective — *a particular skid-steer robot, wheels hard
or soft, maximise obstacle transposition and stability* — found inconsistencies, a missing
objective, and two framing errors. This entry records what changed and why.

### Stability is objective 5, and it was never harshness

"Stability" for this project now means **not tipping over on obstacles and slopes** — rollover
and pitch containment — which the harshness objective (a comfort axis) does not measure.
`PlatformSpec.tipover_angles_rad` derives the static critical angles from the platform's own
CG height, wheelbase and track: **37.4° pitch / 45.8° roll** on the current estimates, pitch
the tighter axis — matching the observed failure mode, a robot rearing to 90° at a step it
cannot climb. `RoverResult.stability_margin` is `1 − max(|pitch|/crit, |roll|/crit)` over the
driving phase (worst moment, not average: a run that tips once has tipped), an S1 **metric**
(designs rank on it) rather than a diagnostic, CVaR-aggregated like everything else. Measured:
+0.99 on the flat, +0.59 at a 60 mm step. Chassis-only CG makes the reference conservative —
the wheels sit lower, so true critical angles are larger. Static reference, stated as such: a
yardstick for comparison, not a tip-over predictor.

### Hardware is the ground truth; Chrono is optional (ADR-0008)

The robot exists and a printer sits beside it, so the thing Chrono was standing in for is
cheaper than Chrono. The Phase 1 gate becomes: ROM within 10% of a **printed wheel's measured
press curve** over the single-claw regime, multi-claw **calibrated against** the same curve,
on ≥3 printed designs. This re-scopes #31: the multi-claw regime no longer waits on an
analytic flank-bedding element — the printed curve becomes the law above second-claw
engagement, and the element gets built only if the calibrated ring still cannot track the
measured shape. The protocol (bench press rig: kitchen scale, screw jack or drill press,
1 mm steps to 20 mm, both phases, two prints per design as material-realisation spread) is
`17-hardware-baseline.md`, alongside the robot-freezing measurement list.

### The scrub gap narrowed to a closed form, and what is left has a number (#38)

The old docstring said lateral behaviour was "not validated against anything". Narrower now:
a claw's out-of-plane stiffness is **(w/t)² ≈ 72×** its tangential one
(`WheelParams.lateral_stiffness_ratio`, 156× at the tip; the taper only widens it, since
in-plane enters cubically and lateral linearly), so the planar ROM is a defensible
*structural* approximation for wide claws and a skid-steer turn is chiefly a **friction**
problem MuJoCo does model. Unvalidated remainder, in #38: the ratio claim (needs a 3-D
`TIP_LATERAL` case — the 2-D tier cannot express it), patch-level tip scrub, and low-`w/t`
designs. Turning scenarios stay out until the first check passes.

### The rigid family is a candidate, not just a baseline (#37)

The pipeline could not have concluded "a hard wheel wins" even if true: no `T0` in CAD, no
`T1`/`T2`, no CoACD path. That is a search biased by construction, now recorded as scheduled
work — `T0`, grousers/lobes, CoACD→MJCF (promoted out of #36), and the first fair
hard-vs-soft suite run. Carried with it: `outer_radius_mm`'s 100 mm cap traces to the dead
220 mm printer; the true ceiling is the chassis wheel well, to be measured off the 3D model.

### Housekeeping with content

The dead printer's ghost was in four places, including the *rationale* for the radius bound
and `wheel_envelope.max_radius` itself. README's status still led with the twice-superseded
50-vs-20; now it leads with the washboard result and says why the old number died. Banded-`T3`
machinery formally **dormant** (a frozen surface; #32's re-fit debt moot until revival). The
slide element retired from both CLIs — it lost to the hinge twice and lives on in the library
as the hinge's regression comparator; `--tangential slide` now errors. CLAUDE.md's command
block subordinated to README's.

**Gates.** Unit suite 828/828 (4 new), ruff at the standing 71.

---

## 2026-08-11 — The robot arrives, and it is not the robot the project assumed

The hardware data landed: four Dynamixel MX-64AT at 12 V, 8.5 kg all-up, 3 cm clearance, and
the 3D model (`configs/robot_piperobot.stl`, 156k triangles). Adopted into `robot.yaml` with
provenance: **stall 6.0 N·m / no-load 6.597 rad/s at the output** (datasheet at 12 V; the
200:1 is internal, so no efficiency factor on top), 12 V power, clearance 0.030, chassis mass
7.3 kg under a stated wheel assumption. `nominal_wheel_load` stays 24.5 N deliberately — the
measured mass implies ~20.9, but that number is baked into every FEA cache key, so it moves
in one commit once the mass ambiguity is settled; the consistency warning that now fires is
that note, enforced (pinned as a test asserting exactly one warning).

### What the measured values already changed

- **Half the speed, 1.5x the torque**: 0.40–0.56 m/s top speed against the old 1.19; the
  scenario boxes shrink automatically (reach is computed), and the old `target_speed: 0.6`
  was unreachable by any design — now 0.35.
- **30 mm of clearance, not 70, and the failure mode flipped**: at an 80 mm step the
  estimated robot reared toward 90°; the measured one **parks its belly on the riser at
  7.2°**. The belly now protects the wheels (peak claw compression identical flat vs 50 mm
  step, because the chassis grounds first). Three rover tests were re-pinned to the measured
  physics; two of their old premises (a step deepens compression; stiffness leaves the
  sharing fraction unchanged) were **measured false** — a 3.8x softer law shares 47 pp more,
  which is physics, not criterion drift.
- **The platform is now part of run identity** (`PlatformSpec.digest()` into
  `RunRecord.versions`) — third instance of the invariant-5 bug: re-measuring the robot
  produced the same run_ids with different metrics inside. CI manifests regenerated.

### Mining the STL, with one cross-check that makes it trustworthy

Vertex analysis of the binary STL (numpy, no mesh library): **overall 426 x 231 x 187 mm;
wheelbase 250 mm; track 157 mm; four contact patches on the floor plane; existing wheels
r ≈ 22.5 mm** (bottom-touch-constrained arc fits, four wheels agreeing 20.5–23.5). The
model's own lowest mid-body material sits at **34 mm** — against the hand-measured 3 cm,
which is the check from outside the extraction that makes the rest credible.

**The assumed robot was fiction in two load-bearing places.** Track is 157, not 350 — the
roll axis just got 2.2x tighter, which the new stability objective will feel directly. And
the existing wheels are r 22.5 against a searched range of **R 60–100**: the entire design
space is 3–4x oversized for the machine it is supposedly for. All four axles show structure
at r 26.5 mm — numerically the MX-64 horn radius (Ø53), which *rotates* and caps nothing —
so the true wheel ceiling is an application question (the 187 mm circular shell says
pipe-fit) that an STL cannot answer.

**Not adopted yet, deliberately**: chassis box, track, and wheel envelope drive every
simulation and cache key, and re-deriving the design space around r ~22–45 invalidates the
existing corpus. That adoption is one commit, after the pipe-bore and bracket questions in
`17-hardware-baseline.md` §A are answered. The YAML carries the measured values as a
prominent pending block so they cannot be lost.

**Gates.** Suite green at 828 after the re-pins, ruff at the standing 71.

---

## 2026-08-11 — The adoption commit: the project now describes the machine that exists

The user answered the four gating questions (no pipe-fit constraint; the r-26.5 structure is
the servo horn, which rotates and caps nothing; wheel width is free outboard; 8.5 kg was
ready-to-run, ~8.0 without wheels) — so the deliberate, cache-invalidating adoption promised
by the pending block happened. Everything below moved in one sitting.

### What moved

- **`robot.yaml` geometry is the measured robot**: chassis 426 x 231 x 157 over a 30 mm belly,
  wheelbase 250, **track 157**, mass 8.0 kg ex-wheels, inertia re-derived. The
  0.400 x 0.300 x 0.200 "requirement" test now pins the measured dims, with the note that the
  requirement is the machine that exists.
- **The interface is the MX-64 horn, not a D-shaft.** Wheels bolt to a D53 rotating disc;
  the 8 mm bore survives as the thrust-boss clearance hole. New screening check
  `hub_seats_horn` (WARNING for now: hub >= 28 mm seats the horn directly, smaller hubs need
  an adapter flange — the whole hub-22 corpus sits there, and hard-failing it the day the
  horn was discovered would call every recorded design unbuildable when it is merely
  un-direct. Promotes to INFEASIBLE when the default hub moves).
- **`PARAM_BOUNDS` radius: (60, 100) → (40, 90).** Floor from the horn seat plus a claw that
  is not a nub; ceiling is judgement (no pipe constraint, horn rotates, bed takes R 120) at
  sane body lift (+67 mm) and top speed (0.59 m/s).
- **`nominal_wheel_load` 24.5 → 20.6 N**, and `LoadCase.nominal_load_n` with it — every FEA
  cache entry deliberately invalidated; the old scale described a fictional 10 kg robot. The
  single-wheel rig's payload followed (2.5 → 2.1 kg).

### The consistency checks earned their keep twice during the adoption itself

The first attempt left 24.5 N in place — the YAML replace silently missed — and the warning
that had been pinned as "the note, enforced" caught it. The second attempt set a target
speed of 0.35 m/s, and the drivetrain check pointed out the smallest wheel in the new range
(R 40) tops out at 0.26: target is now 0.25, reachable by every design. One check was
retired *because the robot contradicts it*: "track narrower than the chassis" is the normal
state of a machine whose wheels tuck under its shell; the check now guards the actual
inconsistency (a track narrower than one wheel).

### Clearance is wheel-dependent, and the constant version was hiding it

The measured 30 mm belongs to the **original r 22.5 wheels**: the belly rides a fixed
**7.5 mm above the axle line** (bracket geometry, `chassis.axle_to_belly`), so real clearance
is `R + 7.5` and a bigger wheel buys belly height one-for-one. The constant-clearance sim had
been sinking an R 85 candidate's axle 55 mm above its own belly — measured symptom: an 80 mm
step "bellied" at 0.1 deg of pitch after 0.59 m, which was the *nose* of a chassis whose
belly the sim had pinned at 30 mm regardless of wheel. `PlatformSpec.ground_clearance_for
(wheel_radius)` is the fix, and the re-measured failure modes are now three regimes:

```
   R 85 fitted -> belly at 92.5 mm; chassis nose overhangs the front axle by 88 mm
   step  25 mm   climbs, pitch  5.7 deg
   step  80 mm   genuine climb attempt: pitch 20 deg, no belly, margin +0.59
   step 100 mm   NOSE-IN: chassis strikes the riser at <1 deg -- the overhang, not the wheels
```

So on the measured robot, obstacle capability is **wheel-limited up to R + 7.5 mm and
nose-limited above it** — wheel radius pays twice (climb reach and belly height), and the
88 mm front overhang is the hard wall no wheel can move. Tip-over also inverted: track 157
against wheelbase 250 makes **roll the tight axis** (35.9 vs 49.0 deg), the reverse of the
fictional wide box — this robot's stability risk is sideways on a slope, not backwards on a
step. The rover test pinning the failure signature is on its third version in one day, and
its docstring keeps all three as the record.

CI manifests regenerated against the measured platform (the platform digest in run identity
made the old ones read as a different experiment, which is what they now are). URDF export
noted as held back per the user (#39 stays open, unscheduled).

**Gates.** Suite 828 green after the re-pins, ruff at the standing 71.

---

## 2026-08-11 — The rover wears its own shell, and the shell disagrees with the box by 17 mm

`run_rover.py` now draws `configs/robot_piperobot.stl` over the chassis by default
(`--no-chassis-stl` for the plain box). Same contract as the wheel overlay, enforced by the
same class of test: zero mass, zero collision, every reported number byte-identical with and
without it (`tests/test_rover.py::TestChassisMesh`). **The box remains the contact geom**,
faded to a ghost rather than removed, for two reasons that are physics rather than taste: the
STL contains the robot's original r 22.5 mm wheels, which must never touch the ground; and
MuJoCo collides a mesh by its convex hull, which would replace the measured flat belly — the
surface the nose-in regime lives on — with the hull of a pipe.

### Placing it: by the axle line, measured twice

The STL's frame is (lateral, vertical, longitudinal) spanning 231 × 187 × 426 mm. Bottom-
touch-constrained arc fits on the four wheels put the axle stations at **110.7 / 360.6 mm**
along the long axis — wheelbase **249.9 mm against the adopted 250**, the cross-check — with
midpoint 235.6 mm; the floor contact patches agree on that midpoint to 0.1 mm (they read the
wheelbase 2.6 mm short from flat-spot bias, which is why the arcs decide). Axle height
24.6 mm above the model's floor (ground plane at 2.07 plus r 22.5). The constants live on
`rover.CHASSIS_MESH_AXLE_MM` / `CHASSIS_MESH_QUAT`; the placement depth is
`−(axle_to_belly + h/2)` below the chassis centre, radius-independent because clearance is
`R + 7.5` on this machine. Verified against the artefact, not the constants: a test loads the
real STL through the compiled geom pose and asserts the four lowest vertex clusters straddle
the sim's own wheel mounts within 10 mm. Rendered and eyeballed: the pipe shell stands on the
sim wheels, horns outboard, old wheels hidden inside any R ≥ 40 design.

### The finding on the way: the overhang is asymmetric, and the box flatters the nose

Placed by axle line, the shell's front overhang measures **~105 mm and the tail ~72**, where
the centred contact box models **88/88**. So the real machine noses into a wall ~17 mm
*earlier* than every nose-in number quoted so far — the 100 mm nose-in regime measured at
R 85 used the box's 88. The render now shows the gap honestly (the white shell leads the
ghost box); whether the box should become an offset box is a platform-model question for the
freeze (com_offset is also still an estimate), not a decoration question, and it is noted in
`docs/run_rover.md` rather than silently absorbed.

**Gates.** `tests/test_rover.py` 67/67 (8 new); full suite below.

---

## 2026-08-11 — The simplified shell arrives with axle stubs, and the track becomes wheel-dependent

The user split the model: `pipebot_detailed.stl` (the 156k-triangle original, renamed from
`robot_piperobot.stl`) and a new `pipebot_simplified.stl` / `.step` (8k triangles) built for
the simulation. The simplified model has **no wheels and explicit axle stubs** — r 7.5 mm
cylinders, confirmed in the STEP's own surfaces — and the sim now draws it by default in
place of the detailed shell. Registration is by the stubs, no arc-fitting needed: stations
y = 103.48/353.51 (**wheelbase 250.03 against the platform's 250**, the cross-check),
midpoint 228.49, height z = 24.50 — identical to the detailed model's measured axle height,
which is the two files agreeing about the machine. Frame is x-lateral/y-long/z-vertical, a
+90° z rotation into the body; `TestChassisMesh` re-pinned, including a test that maps the
real STL through the compiled geom pose and lands all four stubs on the sim's axles.

### The stubs settle where candidate wheels mount, and it is not where the originals are

The stubs emerge at x = +97/−98 about a midline of −0.5 — **±97.5 mm exactly** — and run to
the model's full width. The original r 22.5 wheels tuck UNDER the shell at track 157; no
candidate wheel (R 40–90) can, and the user confirmed external mounting at the plates. So:

- `robot.yaml` gains `drivetrain.wheel_mount_face: 0.0975`; `track_width: 0.157` stays as
  the original wheels' reference measurement.
- `PlatformSpec.track_for(width)` = 2·(0.0975 + w/2) = **195 mm + width** — the same shape
  as `ground_clearance_for`: the second platform quantity that turned out to be
  wheel-dependent, on the second axis, in one day. `wheel_mounts` and `build_rover_mjcf`
  take the width; `tipover_angles_rad` takes the actual track, because an external wheel's
  width genuinely moves the contact line (unlike radius, which stays a fixed yardstick).
- **Roll is still the tight axis, but barely, and the optimiser can now trade on it**:
  46.1° at a 30 mm wheel against 49.0° pitch (was 35.9° at track 157). Width buys roll
  margin at ~0.3°/mm around the default — a real objective-5 coupling that did not exist
  under the fixed track.

### Consequences swallowed deliberately

`wheel_mount_face_m` enters `PlatformSpec.digest()`, so run identity moved: all three CI
manifests regenerated at R 45/65/85 and verified bit-identical (exit 0). Found on the way:
`run_s1.py --manifest-out` was still judged by the threshold fit — unusable on the 3-rung CI
ladder by design — so `write && verify` chains broke on exit 1; manifest-out now counts as a
gate run, same argument as `--manifest`. Every rover number before this entry was measured
at track 157 with the wheels in a place they cannot physically be; the standing re-run of
the corpus (#30/#33 residue) covers this too.

**Gates.** Suite 839 green (platform 3 new, rover re-pinned), ruff at the standing 71,
manifest gates 3/3 exit 0. Renders verified by eye: wheels outboard of the plates, stubs
into the hubs, dome nose forward.

---

## 2026-08-11 — The chassis as primitives: the simplified model turns out to be exact, and the dome changes the nose-in story

`run_rover.py --chassis-collision primitives` replaces the calibrated box with the shapes
read off `pipebot_simplified.stl`: the r 72.5 pipe cylinder, the **r 55 hemispherical nose**
(10 673 mesh vertices within 1 mm of one sphere), and the eight bracket plates/webs below
it. The set is **lossless, not an approximation** — every non-stub vertex of the mesh lies
ON or INSIDE the primitive union, p100 = 0.00 mm — because the user CAD-authored the
simplified model from exactly these solids. A test pins that at 0.5 mm, so a re-exported
model that no longer matches the primitives fails by name. Everything shares the one
axle-stub registration (`CHASSIS_MESH_AXLE_MM`), so physics and picture cannot drift apart.
`chassis_hit_step` now watches the whole chassis geom set rather than a geom literally
named "body".

### Measured on the R 85 ladder, box against primitives

- **60 and 80 mm: byte-identical.** The plates clear the 60 mm step by 2 mm, and at 80 the
  wheels pitch the chassis up before the plates can reach the riser — the chassis never
  touches, and the flat-ground equality property (also pinned by test) holds in the wild.
- **100 mm: the same verdict, a different machine.** The box hard-stops flat against the
  riser at **0.9° pitch, 587 mm**; the dome **rides the step edge** to **24.6° pitch,
  800 mm**, ending 43 mm higher. The box was pessimistic about *how* a nose meets a wall —
  a dome converts a dead stop into a (failed) climb attempt. Neither clears, so no standing
  climb number moves at these heights, but any scenario that reads peak pitch, stability
  margin or travelled distance near the nose-in boundary will read differently.

### What this does NOT settle

The plates' points sit **23 mm below the axle line** (30.5 mm below the box belly) — at the
original r 22.5 wheels that is 0.5 mm off the floor, which is either a real skid the robot
lives on or CAD convenience. Until that is confirmed on hardware, `box` stays the default
and the two are never mixed in one comparison; the flag exists precisely so both can be run
on identical scenarios.

**Gates.** Suite 846 green (7 new), ruff at the standing 71. Filmed: the dome resting on
the 100 mm step corner at full pitch.

---

## 2026-08-11 — "The wheels turn backwards and it will not climb": one illusion, one real de-saturation

A filmed run of the twelve-claw wheel at a 50 mm step (`--render`, default 25 fps) shows the
wheels apparently spinning backwards while the robot drives forward, and the climb failing.
Both were chased to ground.

**The backwards wheels are the camera, measured.** Axles rotate **+6.8 rad/s** while the
chassis moves **+0.41 m/s** — physics forward, both signs checked in the running sim. But
12 claws x 1.08 rev/s = **13.0 claw-passes/s against a 25 fps video**, whose Nyquist rate
for a 12-fold pattern is 12.5 Hz: the pattern advances 0.52 of a claw pitch per frame and
the eye reads it as retreating 0.48 — the wagon-wheel effect, almost exactly on its worst
frequency. At the wall, slip spins the wheels toward no-load and holds them there. Remedy
recorded in `--fps` help: film claw wheels at 60 fps.

**The failed climb is real, and it de-saturates the rover step metric.** On the measured
platform the R 60 rigid cylinder clears **40 mm (0.67 R)** — `[####........]` — not the
1.00 R that made #33 declare the rover sweep unable to rank wheels. That old ceiling
belonged to the fictional platform, which hit the step at 1.19 m/s against the measured
0.40: the saturation was momentum. The claw wheel also clears 40 and also fails 50, so at
50 mm nothing is wrong with the run — the suggested height was stale intuition from the
saturated era (examples in CLAUDE.md / run_rover.py / run_rover.md moved 50 → 40).
Chassis model is irrelevant here: box and primitives give byte-identical numbers at 50 mm
(the chassis never touches).

**How each wheel fails 50 mm differs, and the claw's failure is outside the model's
validity.** Rigid: a genuine attempt — 11.7 deg of pitch, 854 mm travelled, 42.6 J. Claw:
2.4 deg, parks at the riser and grinds **303 J** into slip with **69% multi-claw sharing**
— deep inside the regime where the elements straddle the FEA by +75/−46% (#31) — and the
run's own banner says "the law did not pass its gate; this is a picture, not a number".
Whether a real claw wheel hooks a 50 mm edge is precisely what the ROM cannot yet say and
the bench robot can.

Follow-up worth its own run: with the ladder de-saturated, re-run the #33 four-design
comparison — the rover step sweep may now rank wheels after all.
