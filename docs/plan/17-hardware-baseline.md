# 17 — Freezing the platform, and the hardware ground-truth protocol

Two jobs in one file, because they share equipment and an afternoon: **(A)** measure the real
robot so `configs/robot.yaml` can set `meta.frozen: true`, and **(B)** the bench protocol
ADR-0008 promotes to ground truth. Everything downstream carries a "these are estimates"
caveat until (A) is done, and the ROM's multi-claw regime has no calibration path until (B).

## A. Freezing `robot.yaml` — every field, how to get it, what it replaces

**Status 2026-08-11: the measurements arrived, and they changed the robot.** Received: the
motor (4x Dynamixel MX-64AT at 12 V — 6.0 N·m stall, 63 rpm no-load, at the output), 8.5 kg
all-up, 3 cm clearance, and the 3D model (`configs/robot_piperobot.stl`). Mining the STL:
**wheelbase 250 mm, track 157 mm (not the assumed 350), overall 426 x 231 x 187, existing
wheels r ~22.5 mm** — a pipe robot with a circular ~187 mm shell, not the assumed
400x300x200 box. The model's own clearance (34 mm) cross-checks the hand measurement. The
searched wheel range (R 60-100) collides with the real machine; adoption of the measured
geometry is pending the open questions below.

**Open questions the STL cannot answer** (they gate the design-space re-derivation):
1. **Must the robot keep fitting a pipe, and what bore(s)?** The circular shell says
   pipe-fit; a larger wheel lifts the whole body and changes that fit. This sets the wheel
   radius ceiling, which no geometry measurement can.
2. **What rotates at r 26.5 mm from the axles** — servo horn (rotates, caps nothing) or a
   fixed bracket (caps the wheel at ~26 mm)? One photo of a wheel answers it.
3. **How much axial room is there for a wider wheel** outboard/inboard of the current one?
4. The 8.5 kg: with or without the current wheels and battery? (Now minor — the existing
   wheels are tiny — but it settles `nominal_wheel_load`.)


Ordered by leverage: the first three touch every number in the project.

| Field | Current (estimate) | How to measure | Effort |
|---|---|---|---|
| `chassis.mass` | 8.8 kg | Bathroom/kitchen scale, robot ready-to-run **with battery, without wheels**. Weigh wheels separately | 5 min |
| `motor.stall_torque` (at output) | 4.0 N·m | Preferred: **datasheet** from the motor brand + gear ratio (see below). Cross-check: lever arm clamped to one axle pressing on a kitchen scale at a known radius, battery at working charge, brief stall — `τ = m·g·r`. Keep stalls under ~2 s | 30 min |
| `motor.no_load_speed` | 14.0 rad/s | Datasheet, cross-checked by video: tape flag on a wheel, phone slow-mo (240 fps), count frames per revolution at full throttle off the ground | 15 min |
| `chassis.com_offset` | (0, 0, 0) | Balance method: find the tip line by tilting the robot on a straightedge along each axis; or read it from the **3D model** if component masses (battery!) are placed. The z-offset moves both tip-over angles directly — objective 5 depends on it | 30 min |
| `chassis.inertia` | uniform-box formula | From the **3D model** with real component masses assigned (any CAD package reports it). The box formula over-estimates for a robot whose mass concentrates low and central — which flatters nothing but is wrong | via 3D model |
| `wheelbase`, `track` | 260 / 350 mm | Tape measure axle-to-axle, or the 3D model. Trust the model only if the built robot matches it | 5 min |
| `ground_clearance` | 70 mm | Measure under the chassis at ride height with current wheels | 2 min |
| `wheel_envelope.max_radius` | 100 mm (**derived from a dead printer**) | The chassis wheel well: from the 3D model, the largest radius that clears body, fasteners and neighbouring wheel at full articulation, minus 5 mm | via 3D model |
| `shaft` interface | 8 mm D-shaft | Calipers on the actual shaft + flat depth | 5 min |
| `nominal_wheel_load` | 24.5 N | Derived: `(chassis + 4 wheels) · g / 4` once mass is real. Optionally verified per-corner with the scale under one wheel | derived |

**What Claude needs from you to do the desk half:**
1. **The motor brand and model string** (and gearbox ratio if separate) — the datasheet gives
   stall torque, no-load speed, stall current, and the torque constant; the YAML wants
   output-shaft values, so the ratio and its efficiency matter.
2. **The 3D model** (STEP preferred; STL works for measurement) — for the wheel well radius,
   CoM and inertia with masses assigned, and wheelbase/track as-designed.
3. **Three numbers you measure**: all-up mass with battery, per-corner mass on one front and
   one rear wheel (CoM x-offset falls out of the difference), and ground clearance.

With those, the YAML gets filled, the cross-checks in `platform.consistency_warnings` get
re-run, `meta.frozen` flips to true — and `LoadCase.nominal_load_n` plus every cache key
downstream is re-derived once, deliberately, in a single commit.

## B. The bench press test — the ground-truth instrument (ADR-0008)

**What it measures:** the quasi-static whole-wheel load curve `F(δ)` of a printed wheel —
the exact quantity the FEA sweeps produce and the ROM is built from. One rig, three uses:
validate the FEA→print gap, calibrate the ring's multi-claw regime (#31), and provide the
material-realisation spread invariant 7 wants sampled.

**The rig, deliberately primitive:**
- A printed wheel on a stub shaft (print the shaft adapter too), pressed tip-down onto a
  **kitchen scale** (0.1 g class, ≥5 kg range) resting on a hard flat surface.
- Displacement by any of: a drill press or lab stand with a depth stop; a printed screw jack
  (M8 threaded rod = 1.25 mm/turn, quarter-turns give 0.31 mm steps); or shims. Read
  displacement with calipers between two reference faces.
- Protocol: load in ~1 mm steps to 20 mm and back, 10 s settle per point (TPU creeps — the
  settle time is part of the measurement, record it), read force at each. Repeat the sweep
  twice per wheel; print each design **twice** and treat print-to-print spread as material
  realisation data, not error.
- Phase matters on a bandless wheel: press once tip-down and once gap-down
  (`phase_for_tip_contact` says which is which); the difference is the polygon effect the
  harshness metric rides on, measured for free.

**Acceptance uses:** single-claw regime — ROM within 10% of the printed curve (the new
Phase 1 gate); multi-claw regime — the printed curve **is** the calibration target the ring
is fitted to above second-claw engagement. Log every curve to the store like any run.

**Later, same rig family (not needed to start):** spin-in-place on a known surface for the
#38 scrub/friction ground truth (command a slow yaw, film it, compare yaw rate against the
MuJoCo prediction); a step-edge press (a board clamped under half the scale) for the
`RADIAL_STEP_EDGE` case.
