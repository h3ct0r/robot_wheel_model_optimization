# 06 — Modelling compliance without dying

**This is the technical core of the project.** It is the difference between a project that
finishes and one that doesn't.

## The problem

A candidate compliant wheel must be evaluated in closed-loop dynamic simulation — driving at
a step, tracking a path, crossing rubble — thousands of times. Full FEM of the wheel inside
that loop is roughly 10³–10⁵× too slow. Rigid-body simulation cannot represent the effects
that matter at all.

## The solution: reduced-order models generated from FEA

This is a solved problem in the automotive tire world. Copy the solution rather than reinvent
it.

**FTire** is a full 3-D nonlinear in-plane and out-of-plane tire model whose structural
component consists of **80–200 lumped-mass nodes** replacing the tire's cord structure,
connected to the rim, plus a road/contact-pressure/friction component. It runs at **only
5–20× real-time** and is numerically robust. Crucially, **flexible-ring tire models can be
generated *virtually*, from finite element analysis, rather than from physical measurement** —
precisely this project's situation, since FEA is available but test equipment is not. Surrogate
models emulating hysteretic nonlinear tire dynamics have been shown to give good accuracy with
very large simulation-time reductions.

## The pipeline, concretely

### 1. Offline, per design (T2a)

Run a small suite of quasi-static FEA load cases:

- Radial compression against a flat plate, load sweep 0 → 3× nominal → 0 (loading *and*
  unloading, to capture the hysteresis loop)
- **Radial compression against a cleat / edge** (a step corner) — the load case that actually
  matters for obstacle climbing, and the one NPT papers omit
- Lateral (camber) load sweep
- Torsional (drive torque) load sweep
- Buckling / snap-through detection during the radial sweep

### 2. Extract the ROM parameter set

- Radial stiffness curve `k_r(δ)` — nonlinear, typically stiffening
- Contact patch length and width as functions of load
- Loaded rolling radius as a function of load
- Lateral stiffness `k_l`, torsional stiffness `k_θ`
- Hysteresis loss factor (area of the loading/unloading loop) → equivalent damping
- Buckling load and mode
- Peak spoke stress at nominal and at 2× load (feeds the fatigue constraint)

### 3. Build the MuJoCo ring model

Represent the wheel as an `N`-segment ring of rigid bodies (start with `N = 24–48`, in
FTire's spirit), each connected to the hub by a radial prismatic-plus-rotational joint with
stiffness and damping fitted to `k_r(δ)` and the hysteresis factor, and to its neighbours by
joints fitted to the shear-band bending stiffness. Close the ring with an equality constraint.

This is ordinary MuJoCo — fast, batched, no flex, no FEM. Each segment carries a simple
collision primitive (capsule or box).

### 4. Calibrate

Fit the ring model's parameters so its response reproduces the FEA load cases. **Report the
fit error** — this is a headline validity number for the paper.

### 5. Surrogate the FEA away

After ~300 FEA runs across the design space, fit a GP or small neural network mapping
**design parameters → ROM parameters**. New candidates get ROM parameters predicted in
milliseconds and skip FEA entirely. Periodically validate by running true FEA on a random
sample and reporting prediction error.

**This step is what makes a 3,000-design campaign possible, and it is contribution C1.**

## Fidelity ladder

| Level | Model | Cost | Captures | Misses |
|---|---|---|---|---|
| **L0** | Rigid wheel | 1× | nothing compliant | everything |
| **L1** | Rigid wheel + soft contact | 1× | crude contact springiness | patch growth, hysteresis, loaded radius, buckling |
| **L2** | Rigid hub + radial spring-damper per contact point | 1.2× | radial compliance, damping | patch shape, spoke buckling, lateral coupling |
| **L3** | **Segmented ring, FEA-calibrated (recommended)** | 3–10× | patch growth, loaded radius, hysteresis, lateral/torsional coupling, edge envelopment | local stress, true buckling mode, material nonlinearity beyond fit range |
| **L4** | MuJoCo `flex` tetrahedral | 100–1000× | genuine continuum deformation | throughput; too coarse in fast mode |
| **L5** | Chrono ANCF shell FEA tire | 10⁴× | near-ground-truth structural response, tire–soil co-deformation | usable in a loop |

**Plan:** develop at L1, validate at L3 against L5, run the campaign at L3, verify top designs
at L5. Run L0/L1 explicitly as *ablations* — showing that rigid-contact simulation mis-ranks
compliant wheels is a clean result and directly answers RQ3.

## What the ROM will and won't capture — state this in the paper

**Captures well:** static and quasi-static compliance, obstacle envelopment, loaded radius,
contact patch growth, first-order damping, gross ride response.

**Captures poorly:** high-frequency dynamics beyond the ring discretisation, large-strain
material nonlinearity outside the fitted load range, temperature effects, progressive cyclic
softening (`07-materials.md`), local stress concentration, true post-buckling behaviour.
Detect the last by flagging designs whose FEA showed buckling within the operating envelope
and either constraining them out or escalating to L5.

## Sanity checks — run these before trusting anything

- A compliant wheel must have a **larger contact patch** and **lower peak contact pressure**
  than a rigid wheel of the same radius under the same load. If not, the ROM is broken.
- Loaded rolling radius must **decrease** with load.
- Cost of transport on flat must be **higher** for softer wheels (hysteresis). If softer
  wheels are more efficient on flat, the damping term has been lost.
- A compliant wheel should climb a sharp step **better** than a rigid one of equal radius
  (envelopment + patch growth), but be **slower** and **less efficient** on flat.

If that qualitative pattern cannot be reproduced, stop and fix the model before optimising
anything.
