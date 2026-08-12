# `scripts/run_rover.py` — the whole robot at an obstacle

Four driven wheels, a chassis box with the platform's own mass and inertia, and a step in
front of it. This page explains what every flag does and, more usefully, **what each one is
allowed to change**: some choose the physics, some choose the picture, and some choose only
how long you wait.

Everything here is also in `--help`. The difference is that this page has the figures.

```bash
python scripts/run_rover.py --help
```

> **Read the climb number next to the single-wheel rig's, not instead of it.**
> `run_step.py` asks what one wheel does with a dead weight on it; this asks what four driven
> wheels and a rigid chassis do together. The second flatters a rigid wheel enormously,
> because three wheels push while one climbs. A rigid wheel clears **0.67 R** here against
> **0.33 R** on the rig (it was 1.00 R on the pre-adoption platform, whose 1.19 m/s of
> approach momentum saturated the ladder) — see `docs/plan/TODO.md` #30.

---

## The scene

Side view, to scale in spirit. Everything dimensional is read from
[`configs/robot.yaml`](../configs/robot.yaml), never invented.

```
                                                    ┌──────────────────────────────
      ·····> drive direction                        │  upper ground (top of step)
                                                    │
     ┌───────────────────────┐                      │      ↑
     │       chassis         │ 157 mm high          │  --obstacle-height
     │   8.0 kg + 4x wheels  │                      │      ↓
     └───┬───────────────┬───┘                      │
     R + 7.5 mm ground clearance                    │
        ( )             ( )   R = --radius     ┌────┴──────────────────────────────
  ══════════════════════════════════════════════════════════════════════════
   floor (plane, friction = --friction)        ^
                                               x = 0.80 m, the step face
     |<-- 250 mm -->|                          (fixed; the robot starts at x = 0)
       --wheelbase, from robot.yaml
```

Two things about that box are deliberate and occasionally surprising:

- **The step is an upper ground, not a platform.** Its length is sized from
  `--duration × no-load speed × --radius`, so the robot cannot climb it, cross it and drive
  off the far end — which used to end with a final frame at exactly the ride height, reading
  as "never climbed" when the truth was the opposite.
- **The chassis is a collision geom, not a decoration.** The *ride height* formula is
  `R + 7.5 mm` (the axle-to-belly bracket geometry), but what actually collides is the
  primitive set by default: its bracket plates reach 23 mm *below* the axle line and its
  nose is a dome that rides step edges. `chassis_hit_step` reports any chassis-terrain
  strike, and watching one happen is most of why this model exists.
- **What you see standing on the wheels is the robot's real shell**
  (`configs/pipebot_simplified.stl`), drawn over the box by default and placed by its
  **axle stubs** — the r 7.5 mm cylinders the external wheels mount on. The contact
  chassis under it (the **primitive set** by default since 2026-08-12, the calibrated box
  via `--chassis-collision box`) fades to a ghost but keeps colliding: MuJoCo would
  collide the mesh itself by its convex hull, bridging the legs and belly into one wrong
  underside. `--no-chassis-stl` drops the shell; either way, every number is identical
  (`tests/test_rover.py::TestChassisMesh`).
- **The wheels mount externally, and the track follows the wheel** (2026-08-11): the inner
  face seats against the side plates at ±97.5 mm, so track = `195 mm + width` —
  `PlatformSpec.track_for` — and a wider wheel widens its own support polygon. The 157 mm
  in `robot.yaml` is the original r 22.5 wheels' tucked-under track, kept as reference.

Plan view, with `--approach` doing the one thing it does:

```
   --approach 0                      --approach 15
   ┌─────────┐    │ step face        ┌ ─ ─ ─ ─ ┐   │ step face
   │  ▲      │    │                    ╲   ▲    ╲  │
   │  │ x    │    │                     ╲  │     ╲ │
   └─────────┘    │                      └ ─ ─ ─ ─┘│
   |<- 225 mm ->|                                   ^ still normal to world x
    track = 195 + wheel width        the heading is yawed; the face is not
    (225 at the default 30 mm)
```

`--approach` rotates the *robot's heading*. It does not steer — four non-steered wheels
cannot turn without lateral scrub, and scrub of a segmented capsule ring is validated against
nothing in this project. Values at or past ±90° are refused: the robot would drive along the
face rather than at it, and every metric below would describe a robot that never met the
obstacle. `docs/plan/08-metrics.md` randomises S1 over ±15°, because a step met square is the
easiest case and a real approach is not.

---

## Four wheel models, and which flags reach them

This is the single most important thing to get right before reading a number, because most of
the geometry group is **silently unused** in the default mode.

```
  --compliant   --stl    what is simulated                        what you see
  ───────────────────────────────────────────────────────────────────────────────
     off         off     solid rigid cylinder                     blue cylinder
                         reads: --radius --width --wheel-mass
                         ↑ the default. The rest of the geometry
                           group describes a design nothing builds.

     off         on      solid rigid cylinder                     cylinder + grey
                         (identical numbers to the row above)     CAD shell

     on          off     N segmented bodies on radial slides,     amber capsules
                         spring law from THIS design's FEA
                         reads: the whole geometry + material group

     on          on      same physics as the row above            amber capsules
                         (byte-identical numbers)                 inside a grey
                                                                  CAD shell
```

The chassis shell is independent of both flags — it is on by default in every row and
governed only by `--chassis-stl` / `--no-chassis-stl`.

The overlay is enforced decoration, not a promise: the mesh geom carries
`contype=0 conaffinity=0 mass=0 density=0`, so it collides with nothing and weighs nothing,
and `tests/test_rover.py` asserts every reported number is identical with and without it.
Handing that mesh to a collision system instead is exactly what
[ADR-0002](decisions/0002-reduced-order-model-over-fem.md) exists to prevent.

### What one compliant wheel actually is

```
              spec.n_segments bodies, each on a radial slide
                        (+ optionally a hinge at its root)
                                  │
              ────────  ╭─╮  ────────
                    ╭─╮ │ │ ╭─╮            each capsule: mass = (wheel_mass - hub) / N
                 ╭─╮    ●    ╭─╮           each slide:   force from the FEA spring law
                    ╰─╯ │ │ ╰─╯            hinge pivot:  one capsule radius inboard of
              ────────  ╰─╯  ────────                    the true claw root
                              ↑
                          ● = hub, on the driven axle hinge
```

**The ring is planar.** Each one lies in its own x–z plane and its segments move radially and
in-plane-tangentially. A rover rolls, yaws, and drops wheels off edges — all of which load a
wheel *out of plane*, where the ring is perfectly rigid. That, plus the unvalidated element
above second-claw engagement (`+62.7%` with a slide, `−49.5%` with a hinge, against the FEA),
is why `--compliant` prints a banner and why this is filed as a **picture, not a
measurement** (TODO #30, #31).

---

## The pipeline a run goes through

Each stage announces itself and reports its own wall time; the long ones carry a progress bar
on **stderr**, so piping stdout to a file leaves clean output and the bar still finds the
terminal.

```
   flags
     │
     ├─ --config ─────────────► load_platform()      mass, dims, inertia, wheelbase,
     │                                               track, clearance, motor curve
     │
     ├─ --compliant ──────────► FEA  (CalculiX)      ── minutes cold, seconds cached
     │    --plane-strain          whole-wheel sweep      the run says which it got
     │    --delta-max             + one-claw sweep
     │    --n-points                     │
     │    --law/--segments               ▼
     │    --cache/--threads         segment spring law
     │    --tangential ───────► tangential FEA sweep ──► hinge / slide element
     │                                    │
     ├─ --stl ────────────────► build123d ──► STL     ── decoration only
     │    --mesh-alpha                     │
     │                                     ▼
     ├─ --obstacle-height ────►  build MJCF  ──►  simulate  ──► RoverResult
     │  --friction --approach                        │
     │  --throttle --duration                        │
     │                                               │
     └─ --render ──────────────────────────────────► frames ─► MP4 / GIF / sheet
          --fps --pixels --out                                  (--no-mp4, --no-gif)
```

A missing optional dependency degrades rather than stops: no OCCT means no overlay and the
simulation runs unchanged; no `ffmpeg` means no MP4 and the GIF still gets written.

---

## The flags, group by group

### The obstacle and the platform

| Flag | Default | What it does |
|---|---|---|
| `--config` | `configs/robot.yaml` | The robot. Chassis mass, dimensions, inertia, wheelbase, track, ground clearance and the motor's torque–speed curve. Nothing in this script invents any of them. |
| `--obstacle-height` | `60.0` mm | Height of the step. Reported back as a fraction of wheel radius, which is the only form worth comparing across radii. **Exactly 0 is a different scenario** — see [flat ground](#flat-ground-the-harshness-scenario). |

`--obstacle-height` was named for the obstacle rather than `--step`, which in a MuJoCo script
also means the integration step and had to be disambiguated by context every single time it
was read.

### Geometry (mm) — a design, only read when something builds it

These are the shared `gen_wheel.py` flags. Each searched one quotes its own screening bound,
read from `PARAM_BOUNDS` rather than typed, so the help and the check cannot drift apart.

| Flag | Bound | Notes |
|---|---|---|
| `--radius` | 60–100 mm | Also capped by the chassis wheel well and the print bed. |
| `--width` | 30–70 mm | Plane-strain FEA force scales linearly with this. |
| `--rim-thickness` | 1.2–8 mm, **or exactly 0** | 0 is a *topology switch*, not the bottom of the range: no shear band, and the spoke tips become the running surface. That is the `T7` claw family. |
| `--hub-radius` | mm | Sets the claw root, so claw length is `radius − hub_radius`. |
| `--bore-radius` | mm | Fixed by the drivetrain, not searched. |
| `--spokes` | 3–36 | **Bandless, this is also the ring's segment count.** |
| `--thickness` | 1.2–8 mm | At the **root**. |
| `--curvature` | ±0.03 1/mm | Signed: which way the spoke bows, and so which direction of drive torque stiffens it. |
| `--profile` | straight / curved / s_curve | Spoke centreline family. |
| `--claw-taper` | 0.25–1 | Tip thickness as a fraction of root. 1.0 is a uniform strut. The minimum-wall check reads the **tip**, so a thick root with an aggressive taper is still rejected. |
| `--tread-depth` | 0–4 mm | 0 is a smooth tread. |
| `--spoke-phase` | degrees | Only matters without a band, where contact is discrete. `-90` puts a tip at the contact point. |

**These are easier to see than to read.** `scripts/plot_geometry.py` draws every one of them
across its own range, with each design's screening verdict under it:

```bash
python scripts/plot_geometry.py --only taper --only spokes --format pdf
```

Two failure modes worth naming, because both read as innocuous:

- **Setting geometry without `--compliant` or `--stl` does nothing at all.** The run is four
  rigid cylinders and only `--radius`, `--width` and `--wheel-mass` are consulted.
- **A rejected design prints a violation list, not a stack trace.** `envelope_radius`,
  `print_bed` and `bounds_outer_radius_mm` all reading `infeasible` means the wheel is too big
  for the chassis well, too big for the print bed, and outside its own search bound —
  typically one root cause, `--radius`, reported three times.

### Material — density *and* stiffness, by two different laws

| Flag | Default | Notes |
|---|---|---|
| `--material` | `TPU_95A` | Filament preset: base density plus hyperelastic coefficients. |
| `--infill` | `0.4` | Knocks mass down **linearly** and stiffness down by a **Gibson–Ashby power law**. Deliberately not the same curve; anyone reading both would otherwise assume one is a bug. |
| `--pattern` | `gyroid` | Sets the packing efficiency behind the knock-down. |
| `--walls` | `3` | A feature thinner than `2 × walls` prints solid, `--infill` then does nothing, and that is reported as a warning. |

### The run

| Flag | Default | Notes |
|---|---|---|
| `--wheel-mass` | `300.0` g | One whole wheel. Held equal between rigid and segmented, so the comparison is at matched mass. |
| `--throttle` | `1.0` | Fraction of stall torque at all four axles. |
| `--duration` | `6.0` s | Also sizes the step box. |
| `--friction` | `1.0` | TPU on concrete. Generous on purpose, so a failed climb is not merely a traction failure. |
| `--approach` | `0.0`° | Heading yaw. Refused at ±90 and beyond. |
| `--washboard` | `0.0` mm | S7: peak-to-trough height of a sinusoidal corrugation. Needs `--obstacle-height 0`; refused alongside a step. |
| `--wavelength` | `100.0` mm | The corrugation's wavelength. Quote every S7 number with both. |
| `--slope` | `0.0` ° | **S2**: uphill gradient, implemented by tilting *gravity* — same physics as an infinite ramp, no entry transient. Metric: sustained speed; sweep gradients for the max sustained gradient. Rigid R 60 holds 10° at 0.38 m/s and backslides at 40° (traction-limited, μ = 1). |
| `--gap` | `0.0` mm | **S3**: a 150 mm-deep trench across the ground, near edge at `step_x`. The floor plane becomes two slabs (a plane cannot have a hole). `crossed` needs the whole body at ride height on the far side; a wheel that drops in stays in. |
| `--rubble` | `0.0` mm | **S4**: tallest rock of a procedural 1.2 m strip; every rock is buried (no floating steps) and none exceeds the asked height, by test. `--rubble-seed` **is** the terrain — same seed, same field, to the bit. One seed is one terrain: sweep seeds before believing a number. Needs `--duration` ≥ ~8 s to reach the far side at this drivetrain's speed. |
| `--spin` | off | **S6 proxy**: left and right sides driven opposite; reports steady yaw rate and scrub energy. **Quarantined by TODO #38** — the ring is laterally quasi-rigid by structure, but tip-level scrub is validated against nothing, so this ranks designs only after #38's checks land. |

Every scenario flag needs `--obstacle-height 0`, and exactly one scenario per run — any
pair is refused by name (`RoverSpec` exclusivity; the suite scores S1–S8 as separate rows).
On any run that travels ≥ 0.2 m the result also carries **`cost_of_transport`** (S5,
objective 2): `E/(m·g·d)` over the driving phase — 0.01 for the rigid cylinder on the flat,
0.18 climbing 10°. On compliant wheels it inherits `TPU_LOSS_FACTOR`'s caveat wholesale.
| `--sweep` | off | Instead of one run, ladder 10 mm to 2.1 R in 10 mm buckets. |

The drive is the platform's own linear torque–speed curve, clipped so a motor never brakes:

```
   torque
     │
  τ_stall ●───╮                τ = throttle · τ_stall · (1 − ω/ω_0),  floored at 0
     │         ╲
     │          ╲              Clipped rather than allowed negative: a motor commanded
     │           ╲             forward and overspeeding would brake, and a braking wheel
     │            ╲            rolling off a step looks like a climb failure caused by
     0 ───────────●──────      the wheel rather than by the drive model.
     0            ω_0    ω
```

The first `settle_s` (0.8 s) of every run commands **no torque at all**. The robot is dropped
a whisker onto the floor and must come to rest first, or the run measures a bounce. This is
why `--duration` must exceed it, and why `distance_m` is measured from the end of settling.

### Compliant wheels

| Flag | Default | Notes |
|---|---|---|
| `--compliant` | off | Four segmented rings instead of four cylinders. Needs CalculiX. |
| `--law` | `claw` | Where the segment law comes from — see below. |
| `--segments` | `24` | Ring resolution. **Ignored by `--law claw`**, where the segments *are* the claws and the count is `--spokes`. |
| `--tangential` | off (bare flag = `hinge`) | A second in-plane freedom per claw. |
| `--plane-strain` | off | The 2-D screening FEA tier: seconds instead of hours. |
| `--delta-max` | `0.012` **METRES** | How deep the FEA presses. Note the unit — every geometry flag above is millimetres. |
| `--n-points` | `10` | Samples per branch. Too few cannot resolve a curve that peaks early. |
| `--cache` | `data/cache/fea` | Content-addressed. A hit is seconds, a miss is minutes; the run says which it got. |
| `--threads` | `4` | Changes how long the answer takes, not what it is — so it is excluded from the cache key. |

**`--law` is the choice that moves the answer most.**

```
  --law claw     press ONE claw in FEA. The measured curve IS the segment law:
                 no fit, no deconvolution. The whole-wheel curve is then spent on
                 a HELD-OUT check instead of being training data.
                 Bandless designs only.  ← the default, and the right one

  --law table    press the WHOLE WHEEL, then deconvolve that curve into N
                 independent segment laws, piecewise linear.

  --law cubic    same deconvolution, fitted to a cubic.
```

The deconvolution is ill-posed when a band carries load between segments, which is an argument
*for* the claw family, where there is no deconvolution to do. Below second-claw engagement the
whole wheel **is** one claw, and the claw law agrees with the whole-wheel curve to **0.036%**.
Above it the ring is not validated at all.

**`--tangential` picks an idealisation, and the two bracket the truth from opposite sides.**

```
  slide                                  hinge  ← the right element
     claw tip slides tangentially           whole claw rotates about its root
     tip moves OUTWARD as it splays         tip moves INWARD as it splays
     FEA at 36 mm travel:  −13.9 mm         FEA at 36 mm travel:  +22.6 mm
                    measured truth: +19.7 mm ────────┘
     kept only as the thing the hinge is compared against
```

Above second-claw engagement, at 9.6 mm, the slide reads **+62.7%** and the hinge **−49.5%**
against the same FEA. Two idealisations straddling the truth from opposite sides is not a law
problem; it is the missing flank contact, filed as TODO #31.

### CAD overlay

| Flag | Default | Notes |
|---|---|---|
| `--stl` | off | Draw the real CAD geometry over each wheel, translucent grey. Needs build123d; the STL is cached under `data/wheels` by design hash. |
| `--mesh-alpha` | `0.40` | 0 invisible, 1 solid. |
| `--chassis-stl` | `configs/pipebot_simplified.stl` | The robot's shell, drawn over the chassis box and placed by its **axle stubs** (r 7.5 cylinders in the model, stations 103.5/353.5 mm — wheelbase 250.03 against the platform's 250, the cross-check). The box stays the contact geom, faded to a ghost. A missing file downgrades to the box with a note. |
| `--chassis-collision` | `primitives` | **A physics switch, not a rendering one.** `primitives` (the default since 2026-08-12) collides the shapes read off the simplified model — the r 72.5 pipe, the r 55 dome nose, and the bracket plates whose points reach **23 mm below the axle line**. Lossless: every non-stub mesh vertex lies on or inside the primitive union, pinned by test. `box` is the flat-bellied box calibrated to the hand-measured clearance (belly at R + 7.5), kept for comparison and pre-flip continuity. Measured on the R 85 ladder: 60/80 mm byte-identical (the chassis never touches), 100 mm a different failure — the box hard-stops at 0.9° pitch where the dome rides the edge to 24.6°. The plates' low points are CAD facts not yet confirmed on hardware; if the hand check contradicts them, the box's measurement wins. Never mix the two in one comparison. |
| `--no-chassis-stl` | off | The plain chassis box, as before. |

The colours are load-bearing, not decorative:

```
   amber capsules  ← the physics. The ring's segments; what the numbers describe.
   grey shell 40%  ← decoration. The shape the physics stands for.
   white shell     ← the robot's real body. Also decoration; the ghost box under
                     it is what actually collides, belly strikes included.
```

Two honesty notes on the shell. Its **axle stubs** run from the side plates into the
simulated wheels' hubs — that is where the external wheels actually mount, and the stub
axes are what the mesh is registered by. And the shell's overhang is **asymmetric** —
~105 mm at the nose against ~71 mm at the tail, where the contact box models a centred
88/88 — so on a nose-in run the shell visibly leads the box. The box is the physics; the
mismatch is the measured gap between the platform model and the machine, drawn rather than
hidden.

Amber was chosen to read against the grey floor, the brown step and the translucent shell.
Before that, `ring_bodies` emitted no `rgba` at all, so the capsules took MuJoCo's built-in
geom colour — a washed olive-green. The one colour in the render that had not been chosen was
the one carrying the result.

### Output

| Flag | Default | Notes |
|---|---|---|
| `--render` | off | Write MP4, GIF and a contact sheet to `--out`. |
| `--no-mp4` | off | Skip the H.264 video (needs `ffmpeg` on PATH). |
| `--no-gif` | off | Skip the GIF. The MP4 is ~13× smaller and full colour, so `--no-gif` is usually what you want. |
| `--fps` | `25` | Used **both** to sample the simulation and to play it back, so the video runs at real speed. |
| `--pixels` | `900` | Frame width; height is 9/16 of it. |
| `--out` | `data/renders` | Where the three files land. |

---

## Reading the result

```
robot: piperobot-426x231, 8.0 kg chassis + 4 x 300 g wheels, 426x231x157 mm
       wheelbase 250 mm, track 157 mm, R 85 mm, ride 171 mm
drive: 6.0 N·m stall x4 at 1.00 throttle, 6.6 rad/s free (0.56 m/s)
       NOTE meta.frozen is false — these are estimates, not a measured robot
-> simulating 6.0 s at a 60 mm obstacle
   0.6s  (climbed)

obstacle 60 mm (0.71 R)
  climbed            True
  travelled          2565 mm
  final clearance    171.0 mm (standing is 171 mm)
  peak pitch / roll  13.9° / 0.33°
  stability margin   +0.72 (1 = level; 0 = CG over the wheel line; crit 49°/36° pitch/roll)
  chassis hit step   False
  axle work          16.6 J
```

That `meta.frozen is false` line is not noise. The chassis envelope is a requirement, but its
mass, motors, battery and inertia are still estimates — so every number above inherits that.

**`climbed` has two conditions and each was wrong alone.** The chassis centre must be a full
half-length past the face *and* within a fifth of its ride height of where it would stand:

```
                              ┌─────────┐  ← climbed: past the face AND at ride height
                    ┌───┐     │         │
   ┌─────┐         ╱│   │     └─────────┘
   │     │        ╱ └───┘  ↑             ┌──────────────────
   └─────┘       ╱         │             │
  ═══════════════──────────┴─────────────┘
   not climbed   reared up:              x > step_x + L/2
                 past the face by x,     and |z − h − ride| < 0.2·ride
                 and 113 mm too high.
                 The x test alone passes it.
```

- **`final clearance`** is height above the *upper* ground. Near the ride height means it is
  standing on the step; near minus the obstacle height means it never left the floor.
- **`peak roll`** should be ~0 on a square approach. Non-zero there means the solver, not the
  terrain.
- **`chassis hit step`** is bellying out (or nosing in) rather than climbing — a real
  outcome for a belly at `R + 7.5 mm`, not an error.

Exit codes: `0` cleared (or `--sweep` finished), `1` did not clear or the run failed, `2` a
dependency or a config file is missing.

### `--sweep`

```
  rigid           40 mm  [####........] 10-120 mm  (0.67 R)
                          │└──────────── did not clear
                          └───────────── cleared
                                          ↑ ladder span, then the answer as a fraction of R
```

(That is an `--radius 60` sweep: `2.1 R` = 126 mm, so the ladder runs 10 → 120 mm.)

**Quote the answer as a bucket, not a millimetre.** The ladder steps in 10 mm and a 1% change
in the segment law can move the answer a whole bucket, so a one-bucket gap between two designs
is not a ranking. The profile is printed rather than just the maximum because the predicate is
not monotone — a bounce reads as a climb otherwise. `AT THE SWEEP CEILING` means the true
value is at least the reported one.

---

## Flat ground: the harshness scenario

`--obstacle-height 0` emits **no obstacle at all** and measures **objective 3** from
`docs/plan/08-metrics.md`: RMS vertical chassis acceleration. It exists because the step-climb
number on this robot discriminates only coarsely — three driven wheels push while one
climbs. On the measured platform the ladder separates compliant from rigid (claws 1.00 R
against the cylinder's 0.67 R, 2026-08-11) but still not claw counts from each other: 6 and
12 claws tie. Within the family, harshness is the axis.

A bandless wheel runs on discrete tips, so it is a **polygon**, and the axle rises and falls
once per tip. That is the cost compliance is supposed to buy back:

```
   rigid polygon, n tips              compliant claw, same n
                                                                axle
      ╭──╮      ╭──╮                     ╭──╮      ╭──╮          height
     ╱    ╲    ╱    ╲                   ╱    ╲    ╱    ╲            │
    ╱      ╲__╱      ╲                 ─        ──       ─          │  ripple
                                                                    ↓
    drop = R(1 − cos π/n)              the claw deflects INTO the
    per half pitch                     gap, so the ripple is smaller
                                       — but only if it deflects by
                                       an amount comparable to the drop
```

Three numbers are printed together, and they come from three different places on purpose:

| Line | Where it comes from | What it is |
|---|---|---|
| `ride harshness` | **MuJoCo**, `qacc` on the chassis free joint | RMS m/s² over the steady window. The measurement. |
| `polygon drop` | **closed-form trigonometry**, `R(1 − cos π/n)` | The rigid limit. No FEA, no dynamics. |
| `loaded ripple` | **the ring**, solving `F(δ, ψ) = load` per phase | Peak-to-peak axle travel with the fitted law and no dynamics at all. |

A model needs at least one check against a number it did not produce. The polygon drop and the
loaded ripple are that check for the harshness measurement — if one moves and the others do
not, stop.

### Measured, R 60 mm, taper 0.6, 6 s at full throttle

| wheel | harshness | polygon drop | loaded ripple | axle work | mean speed |
|---|---|---|---|---|---|
| 3 claws | **22.6** m/s² | 30.0 mm | 29.1 mm | 46.1 J | 0.81 m/s |
| 6 claws | **10.3** m/s² | 8.0 mm | 7.4 mm | 41.7 J | 0.71 m/s |
| 12 claws | **5.0** m/s² | 2.0 mm | 1.5 mm | 12.0 J | 0.83 m/s |
| rigid cylinder | **0.0** m/s² | — | — | 3.9 J | 0.84 m/s |

A **4.5× spread** where the step-climb sweep gave none. Cost of transport separates them too:
the 3-claw wheel spends 12× the axle work of the cylinder to cover the same ground.

Note the ripple against the drop: at 12 claws compliance cuts the ripple 25% below the rigid
polygon, at 3 claws it cuts it 3%. A wheel only rides smoother than its own polygon if it
deflects by something comparable to the drop, and at 24.5 N per wheel this design deflects
about 1 mm.

### Three caveats that travel with these numbers

- **The rigid comparator is a smooth cylinder, so 0.0 is not a wheel anyone could print.**
  It is the floor of the metric, there to prove the metric is not measuring the solver.
- **Fewer claws extrapolate the segment law, and the run says so.** A 3-tip R 60 wheel has a
  30 mm polygon drop; the FEA sweep behind the law runs to `--delta-max`, 12 mm by default,
  and a tabulated law extrapolates on its last slope without complaint. The `EXTRAPOLATED`
  warning fires with the ratio. Widening the sweep 12 → 18 mm moved the 3-claw answer
  24.65 → 22.64 m/s², about 8%, so the *ranking* survives — and **35 mm cannot be measured at
  all**: CalculiX diverges at 10 cutbacks, which is itself a statement about a claw pressed
  half its own radius.
- **This objective has no counter-pressure of its own.** Harshness alone ranks 36 claws above
  12 above 3, monotonically and forever. The pressure back the other way lives in the other
  three objectives — mass, obstacle capability, cost of transport — which is exactly why
  [ADR-0006](decisions/0006-multi-objective-not-scalarised.md) forbids scalarising them into
  one number.

### The washboard (S7): where compliance wins outright

On flat ground the rigid cylinder's 0.00 m/s² is unbeatable. `--washboard` adds a sinusoidal
corrugation — a strip of boxes, entered at a trough so the doorstep is a ramp — and there the
rigid wheel must follow the ground while a compliant one need not:

```
   10 mm peak-to-trough, R 60 mm, full throttle

   wavelength      60 mm    100 mm    200 mm    400 mm
   rigid           40.6      43.3      36.5      33.8   m/s² RMS
   12-claw          6.4       6.3       6.2       7.1
   ratio            6.3x      6.9x      5.9x      4.8x
```

The compliant wheel is also **not slower** (0.67–0.86 m/s against the rigid 0.61–0.78), so it
is not smooth by being slow — and the 3-claw wheel reads 10.0 against the 12-claw's 6.3, so
the metric still ranks within the compliant family. One caveat travels with the magnitudes:
20–53% of those runs have two claws sharing load, where the ROM's element is unvalidated
(TODO #31), and the run says so on its own output. The **sign** is the result.

```bash
python scripts/run_rover.py --obstacle-height 0 --washboard 10 --wavelength 100 \
    --compliant --radius 60 --rim-thickness 0 --spokes 12 --thickness 6 \
    --claw-taper 0.6 --spoke-phase -90 --plane-strain --law claw
```

```bash
# The harshness ladder: a deliberately bad wheel against a good one.
for n in 3 6 12; do
  python scripts/run_rover.py --obstacle-height 0 --compliant --radius 60 \
      --rim-thickness 0 --spokes $n --thickness 6 --claw-taper 0.6 --spoke-phase -90 \
      --plane-strain --law claw
done
```

---

## Worked commands

```bash
# One run against a 60 mm step, rigid wheels, numbers only.
python scripts/run_rover.py --obstacle-height 60
```

```bash
# The tallest obstacle an 85 mm rigid wheel clears, as a profile.
python scripts/run_rover.py --radius 85 --sweep
```

```bash
# Four segmented claw rings from this design's own FEA, the real CAD shape ghosted
# over them, filmed. Needs CalculiX for the ring and build123d for the overlay.
python scripts/run_rover.py --compliant --stl --radius 60 --rim-thickness 0 --spokes 12 \
    --thickness 6 --claw-taper 0.6 --spoke-phase -90 --plane-strain \
    --law claw --tangential hinge --obstacle-height 40 --render --no-gif
```

```bash
# A deliberately bad wheel: three claws, so the axle drops 30 mm between tips.
python scripts/run_rover.py --compliant --radius 60 --rim-thickness 0 --spokes 3 \
    --thickness 6 --claw-taper 0.6 --spoke-phase -90 --plane-strain --law claw --sweep
```

On the pre-adoption platform that last one cleared **exactly what a 12-claw wheel and a
plain rigid cylinder clear** — 1.00 R for all three, the step-climb metric saturating on
approach momentum. The measured platform (2026-08-11) de-saturated it: the rigid cylinder
drops to 0.67 R and the claws hold 1.00 R, so compliant-vs-rigid now separates — but 3, 6
and 12 claws still tie with each other (and the 3-claw number is 2.4× extrapolated past its
law). Ranking *within* the family is still the job of
[flat ground](#flat-ground-the-harshness-scenario) and the washboard, which separate the
same wheels 4.5×. TODO #30 has the full table.

---

## Related

- [`scripts/run_step.py`](../scripts/run_step.py) — the single-wheel rig. One wheel, a dead
  weight, five signatures. The number to read alongside this one.
- [`docs/plan/TODO.md`](plan/TODO.md) — #30 (compliant-vs-rigid on the rover) and #31 (the
  claw's flank contact) both gate what this script's compliant mode is allowed to claim.
- [`docs/plan/08-metrics.md`](plan/08-metrics.md) — the scenario suite this obstacle is one
  rung of.
- [ADR-0002](decisions/0002-reduced-order-model-over-fem.md) — why FEA never runs in the loop,
  and why the CAD mesh here is decoration.
