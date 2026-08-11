# 04 — Design space

## Topology families

| ID | Family | Material | Notes |
|----|--------|----------|-------|
| `T0` | Smooth rigid cylinder | PLA/PETG | Rigid baseline — needed for the comparison to mean anything |
| `T1` | Grousered rigid | PLA/PETG | Rigid baseline with lugs |
| `T2` | Lobed / star rigid | PLA/PETG | Rigid, discontinuous contact |
| **`T3`** | **Compliant spoke** | **TPU spokes, rigid or TPU hub** | Curved/S-shaped/honeycomb/chevron spokes + thin rim. The NPT archetype |
| **`T4`** | **Monolithic TPU** | **TPU throughout** | Stiffness tuned by infill density/pattern and wall count |
| **`T5`** | **Compliant spoke + lugs** | **TPU** | Conformity plus positive engagement — likely the actual winner for obstacles |
| **`T7`** | **Compliant claw (linear)** | **TPU throughout** | Tapered free-tip fingers from the hub, no band. `T3b` with a taper. **The direction from 2026-08-08** |
| **`T7L`** | **Compliant claw (L)** | **TPU throughout** | `T7` with a tangential **foot** at the tip: a radial leg, a filleted right angle, and a pad lying along the running surface. `tip_hook_mm != 0`. Added 2026-08-11 |
| `T8` | Linkage claw | TPU + pins | Claws driven by a motion-reversing linkage. PaTS-Wheel's own category. Later |
| `T9` | Pivot claw | TPU + pins | Claws hinged at the hub rim. The commonest passive prior art. Later |
| `T6` | Soft tread on rigid hub | PLA hub + TPU tyre | Cheap-to-model comparator; localised deformation only. **Fallback family if the ROM gate fails** |

Start with `T0` and `T3` in Phase 1 — one rigid anchor, one compliant target. Add the rest
progressively.

## Direction, 2026-08-08: bandless claws

**Every future design is bandless.** The shear band is retained in the code and in `T3` as the
thing already measured against, not as a candidate. What replaces it is `T7`, the compliant
claw: tapered fingers cantilevered off the hub with free tips that are themselves the running
surface.

The naming comes from Table I of the PaTS-Wheel letter (`docs/papers`), a taxonomy of
**existing transformable wheels** — Linear Claw, Linkage Claw, Pivot Claw, Passive Pad Deform.
Two things about that table are easy to get wrong and worth writing down:

- **PaTS-Wheel is not in it.** The paper places its own design *between* Linkage Claw and
  Passive Pad Deform, using pad deformation to actuate the claw. The three rows are prior art
  the authors contrast themselves against.
- **The table's Linear Claw is a rigid mechanism** — bars sliding radially through the hub,
  typically fired by a gear train when the wheel stalls. `T7` borrows its *shape*, not its
  mechanism: the compliance is the printed TPU finger bending, which keeps the design inside
  this project's premise and inside the existing FEA → ROM pipeline.

### What the taper buys, and what it breaks

A bandless spoke is a cantilever with a free tip, so the bending moment is largest at the root
and zero at the tip. A uniform section is over-thick everywhere but the root: it wastes mass
and stiffens the tip precisely where conformity is wanted. `claw_taper_ratio` narrows the
outline linearly in **arc length** from root to tip; 1.0 reproduces the uniform strut exactly.

| Parameter | Range | Notes |
|---|---|---|
| `claw_taper_ratio` | 0.25–1.0 | Tip thickness as a fraction of root. 1.0 = uniform strut. Below 0.25 a tapered cantilever stops behaving like a beam and starts behaving like a hinge — a different model, not a thinner one |

It also adds a **second thickness**, and every check has to read the right one. `7 mm` of root
at `0.15` taper is a 1.05 mm tip: unprintable, while `spoke_thickness_mm` still looks
comfortable. `WheelParams.tip_thickness_mm` exists for this, `spoke_min_wall` and the discrete
contact warning now read it, and both are covered by tests. The **slenderness proxy still reads
the root and is knowingly wrong for a taper** — it understates slenderness and errs toward
accepting a claw that buckles. Picking the right effective section for a tapered cantilever is
buckling physics, not a pre-filter tweak; it is flagged in the code and left to FEA.

### `T7L` — the L claw: a foot on the tip

Added 2026-08-11. `tip_hook_mm` bends the last part of the claw through a right angle so it
lies along the running surface: a radial **leg**, a filleted **bend**, and a tangential
**foot**. Zero — the default — is `T7` unchanged, and a zero-length foot is the continuous
limit of a shortening one, so this is a parameter rather than a topology switch (unlike
`rim_thickness_mm`, whose zero really is one).

| Parameter | Range | Notes |
|---|---|---|
| `tip_hook_mm` | −40 to +40 | Arc length of the foot along the running surface. **Signed**: which way the foot points. 0 is the plain radial claw |

**What it is for.** A radial claw touches at a point, so a bandless wheel is a polygon and its
axle drops `R(1 − cos π/n)` once per tip — measured as ride harshness on 2026-08-10, where a
3-claw wheel reads 22.6 m/s² RMS against a 12-claw wheel's 5.0. A foot spreads the contact over
an arc `β = |tip_hook_mm| / R`, so the axle only falls across what is left between two feet and
the closed form becomes `R(1 − cos(π/n − β/2))`. On R 60 with twelve claws a 12 mm foot takes
the drop from **2.04 mm to 0.78 mm**, and `WheelParams.polygon_drop_mm` reads the field.

**Signed, and the sign is a real design variable.** A foot that trails the leg is dragged onto
the ground and folds closed under drive torque; one that leads it is levered open. Nothing in
this project measures that difference yet, which is why both are expressible and neither is a
default. Note that the mirror image of a claw flips **both** the foot and the curvature — the
CAD battery pins this, because flipping the foot alone on a bowed leg gives a C against an S
and the two differ in volume by 2.3e-5, which is exactly the size that gets waved through as
tolerance.

**The bend radius is load-bearing, not cosmetic.** The outline is the centreline offset by half
the local thickness, and offsetting a corner of centreline radius `ρ` makes the inside face a
circle of radius `ρ − h`: at `ρ = h` it collapses to a point and below it the polygon turns
inside out. OCCT may refuse such a face — or accept it into a solid with a reversed patch whose
volume is still plausible, which is the failure mode this project keeps finding. So
`hook_bend_radius_mm` is `0.75 t_tip` against a half-thickness of `0.5 t_tip`, and
`verify_cad.py` §11 checks the outline for self-intersection **independently of the kernel**.

**The foot follows the circle, not a chord.** Built in polar rather than in the spoke's local
Cartesian frame: a 20 mm straight foot on a 60 mm wheel stands 3.2 mm proud of the running
surface, so it would pierce it and the outline clip would then eat it from outside until, on a
tapered tip, the outline crossed itself.

**What is done and what is not.** CAD, screening, the mid-plane figure and the 2-D FEA tier all
handle it — everything reads `spoke_outline`, so it arrived downstream for free, and a 12 mm
foot on the R 60 twelve-claw design meshes and solves. **The ring ROM does not.** Its segments
carry load at a point along their own radius, which is precisely what a foot is not; this is
`TODO.md` #31 — the flank-contact gap — arriving by design rather than by accident, and #35
tracks what it means for `T7L`.

### `n_spokes` bottoms out at 3, and the interesting limit is not the bound

**Closed twice, in opposite directions, and both are worth keeping straight.**

The original gap read: the Linear Claw figure has four spokes and the bound of 6 rejected it,
so the bound needed re-deriving from the claw load case. **`TODO.md` #19 did that measurement
and the answer was the other way round** — a *passive* claw wheel wants **more** tips, not
fewer. Below about twelve it unloads a claw completely once per pitch, whatever the claw's
stiffness, and the letter's four-claw row is only reachable because those claws are gear-driven
rather than passive springs. So the bound was left alone and the claw-specific limit became a
warning, `constraints.claw_ride_harshness`, which fires only when there is no band.

It was lowered to **3** on 2026-08-10 for a different reason: to let the search space express a
**deliberately bad** wheel. A design that a good one has to beat is worth more than a bound
that admits only plausible wheels, and at three tips the axle drops `R(1 − cos 60°)` — half a
radius, 30 mm on a 60 mm wheel — which is a genuinely terrible baseline rather than a
marginally worse one. Nothing about #19's finding changed; it is still reported, loudly.

**Three is the floor because below it the model stops meaning anything**, which is a different
kind of limit from a search bound and is enforced in a different place. `polygon_drop_m` at
one spoke is `R(1 − cos 180°)` = 2R, an axle dropping twice the wheel radius;
`second_contact_delta_m` is `R(1 − cos 360°)` = **0**, which reads as "a second claw engages
immediately" on a wheel that has no second claw. `RingSpec` therefore refuses fewer than three
segments outright, and `check_design` calls one or two spokes `DEGENERATE` — geometry that
cannot be built, not a design that scores badly.

## Shared geometric parameters

| Parameter | Symbol | Range | Notes |
|---|---|---|---|
| Outer radius | `R` | 60–100 mm | **Capped by the 220 mm print bed**, not by the robot |
| Width | `W` | 30–70 mm | Capped by track width |
| Rim (shear band) thickness | `t_rim` | 1.2–8 mm, **or exactly 0** | Lower bound from printability; 0 selects the bandless variant below |
| Hub radius | `R_hub` | 0.2–0.6 · `R` | Sets spoke length |
| Tread depth | `d_tread` | 0–4 mm | Maps to friction bracket |
| Sidewall taper | `α` | 0–20° | Lateral grip, support-free printing |

## Compliant-structure parameters (`T3` / `T5`)

| Parameter | Range | Type |
|---|---|---|
| Spoke count `N_s` | 6–36 | integer |
| Spoke thickness `t_s` | 1.2–8 mm | continuous — **the dominant stiffness knob**. The upper bound is set by buckling at 24.5 N: at a 60 mm span, 6–8 mm is what survives 2.5× nominal |
| Spoke curvature `κ` | −0.03 to +0.03 mm⁻¹ | continuous; sign sets buckling direction |
| Spoke profile | {straight, curved, S-curve, V, honeycomb cell, chevron} | categorical |
| Spanwise pattern | {uniform, tapered, split-pair} | categorical |
| Spoke inclination (out-of-plane) | 0–25° | continuous; couples radial and lateral stiffness |
| Shear band layers | 1–3 | integer |

The space is **conditional**: honeycomb cells introduce cell angle and wall thickness that
other profiles don't have. Ax/BoTorch handles hierarchical spaces; plain scikit-optimize does
not.

### `T3b` — bandless: the spoke tips are the running surface

`t_rim = 0` removes the shear band entirely. It is a **topology switch, not the bottom of
the `t_rim` range** — everything between 0 and 1.2 mm is still rejected as unprintable, and
the screening pre-filter exempts exactly 0 from the bounds check and from the minimum-wall
check. Two extra parameters become live:

| Parameter | Range | Notes |
|---|---|---|
| Spoke phase `φ` | 0–360° | Not searched. Inert with a band; **decisive** without one |

Why it is worth having as the first article: it puts a single spring in the load path
instead of two in series, so an FEA `k_r(δ)` maps onto spoke properties without first
having to separate them from the band's. It is also lighter — 304 cm³ vs 364 cm³ at the
nominal design — and simpler to print.

#### Measured, 2026-08-07

CalculiX 2.23, R 60 × W 30 mm, 6 spokes, `t_s` = 5 mm, hub 20 mm, TPU 95A at 40% infill and
3 walls, δ swept 0 → 6 → 0 mm under displacement control, `φ = −90°` so a tip faces the
indenter. This is the `--tiny` developer preset, **not** the nominal design; it exists to
compare the two topologies at matched geometry, and it reaches only 4.4 N of a 73.5 N target
on the banded side. Treat the ratios as the result and the absolute forces as provisional.

| | 3 mm shear band | bandless, tip at contact |
|---|---|---|
| Flat: force at δ = 6 mm | 4.36 N | 26.9 N, after a peak of **31.4 N at δ = 3 mm** |
| Flat: buckling | none | **limit point at 31.4 N** |
| Flat: `k_r` at peak δ | 1.71 kN/m (stiffening) | −0.55 kN/m (past the limit point) |
| Step edge: force at δ = 6 mm | 3.04 N | 18.31 N |
| Step edge: buckling | none | none; `k_r` softens 2.10 → 1.63 kN/m |
| Step edge: unload loop area | 0.29% | 4.19% (QC threshold is 5%) |
| Flat: p95 spoke stress | 0.08 MPa | 0.47 MPa |

What it costs, and none of it is visible in the CAD:

- **Contact is discrete.** `N_s` tips of width `t_s` replace a cylinder, so the response
  depends on `φ`, and rolling gives `N_s` stiffness cycles per revolution rather than a
  ripple on a continuous curve. Every load case must state its phase.
- **The load path is a strut, not a beam.** The indenter loads one spoke close to axially,
  which is far stiffer than bending a band — and then Euler-buckles it. At equal
  indentation the bandless wheel carries **7.2× the load on the flat plate and 6.0× on the
  step edge**, and the flat case snaps through at 31.4 N. That is **below the 61.2 N the
  buckling constraint demands** (2.5 × 24.5 N), so this particular bandless design is
  infeasible — `fea_buckling` rejects it. Thicker or more numerous spokes are the lever;
  `P_cr ∝ t³/L²`.
- **Contact pressure is an order of magnitude higher**, over `t_s` rather than a patch.
  On the step edge, where the two sweeps come closest to a comparable state, the bandless
  patch is 81 mm² at 226 kPa mean against 318 mm² at 9.6 kPa. A durability question for
  printed TPU, not just a modelling one.
- **The two topologies cannot be compared at equal load on these sweeps.** Banded flat
  covers 0.4–4.4 N; bandless flat's *first* contact sample is already at 13.5 N. The load
  ranges are disjoint, so there is no load at which both were measured — see
  `fea.results.common_force_n`, which returns `None` rather than clamping both to their own
  ends and reporting a ratio between two states neither solve visited. Closing that gap
  means running the banded case to a much larger δ, not re-reading these results.
- **The ring ROM loses its ring.** Segment-to-segment coupling in
  [`06-compliance-rom.md`](06-compliance-rom.md) §3 *is* the shear band's bending
  stiffness, and here it is zero. The model degenerates to `N_s` independent radial legs.
  That is a legitimate reduced-order model and a simpler one, but it is not the FTire ring,
  and "close the ring with an equality constraint" no longer means anything.

## Material and process parameters

| Parameter | Range | Type | Effect |
|---|---|---|---|
| Shore hardness | {TPU 85A, 95A, 98A, 60D} | categorical | Order-of-magnitude stiffness range |
| Infill density | 15–100% | continuous | Effective modulus, mass, damping |
| Infill pattern | {gyroid, grid, honeycomb, concentric} | categorical | Anisotropy and compression behaviour |
| Wall (perimeter) count | 2–6 | integer | Dominates bending stiffness of thin spokes |
| Layer height | {0.15, 0.2, 0.3} mm | categorical | Interlayer bond strength, fatigue |
| Raster angle | {0°, ±45°, 90°} | categorical | **Anisotropy driver** |
| Print orientation | {flat, on-edge} | categorical | Determines which direction sees weak interlayer bonds |

This is a genuinely coupled space: a thin spoke at 100% infill in 95A behaves nothing like a
thick spoke at 20% infill in 85A, and neither reduces to "an effective modulus" without care.

**Effective-property modelling.** Do not mesh the infill. Use homogenisation: represent each
(pattern, density, wall count) combination as an effective orthotropic hyperelastic material,
calibrated once per combination. Standard RVE approach; collapses a meshing nightmare into a
lookup table.

## Constraints

### Geometric
Clearance, track width, watertight manifold mesh, chassis ground clearance under load — the
last now **load-dependent**, since the wheel squashes.

### Manufacturing (FDM, TPU-specific)
- Minimum wall ≥ 1.2 mm (3 × 0.4 mm nozzle); for TPU prefer ≥ 1.6 mm — thin flexible walls
  print poorly
- Maximum unsupported overhang ≤ 45° (tighter than rigid: TPU droops)
- No unsupported bridges > 15 mm — TPU bridges badly
- Minimum gap between spokes ≥ 2 mm so the nozzle can traverse without dragging
- Bounding box fits the 220 × 220 mm bed; print time ≤ 24 h; material ≤ 450 g. A 170 mm
  wheel in TPU is an overnight print — the old 200 g cap predates the larger platform and
  would have rejected every design that survives 24.5 N
- Requires direct-drive extruder — a fixed capability, not a variable

**Why two searched thickness ranges start below the TPU wall** (#34, decided 2026-08-11).
`spoke_thickness_mm` and `rim_thickness_mm` are both searched from **1.2 mm**, and TPU cannot
print below **1.6** — so on every TPU design the bottom of each range is infeasible by
construction. That is deliberate, and the same decision for both fields: the range must be
able to *express* a design the wall check rejects, or `spoke_min_wall` / `rim_min_wall` can
never fire and have stopped testing anything. A constraint that no sample can violate is
indistinguishable from a constraint that was deleted, and this project's watch list is a
catalogue of checks that kept passing after they stopped checking.

The cost being traded: samples spent on rejections. Uniformly over each (1.2, 8.0) range the
unreachable band is 0.4/6.8 ≈ **5.9% per field**, and screening rejects those in milliseconds
(invariant 3) rather than in FEA time. If a future optimiser's proposal distribution turns out
to concentrate near the wall and pay materially more than that, the remedy is a
material-dependent bound — more correct and more machinery — not a silent raise of the floor,
which would quietly retire both checks.

### Actuation
Peak torque ≤ 0.7 · stall; achievable top speed accounting for *loaded* rolling radius, which
is smaller than `R` for a compliant wheel and load-dependent. A real effect on gearing and a
classic omission.

### Structural / compliance-specific
- **Static sag:** loaded radius ≥ 0.85 · `R` at nominal load
- **Buckling:** critical load ≥ 2.5 × nominal wheel load. Compliant spokes fail by
  snap-through, not yield
- **Fatigue:** peak cyclic spoke stress below the material fatigue limit with margin. For one
  characterised TPU the fatigue limit was reported at **10.25 MPa**, with cracks originating
  from micropore aggregation and propagating at ~45°
  ([Polymers, 2023](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9958809/)). Indicative, not
  universal — apply a large safety factor given no in-house material testing
- **Self-contact:** spokes must not collide with each other at maximum deflection. Cheap
  check, easy to forget, produces nonsense FEA when violated

### Implementation
All constraints are a **fast pre-filter returning a violation vector**. An infeasible design
costs 50 ms, not a 6-minute FEA run. See invariant 3 in `CLAUDE.md`.
