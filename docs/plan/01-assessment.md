# 01 — Honest assessment

## What is good about the idea

The loop — *parametric CAD → automatic asset generation → simulation → metrics → optimiser →
new design* — is sound and well established. All the components exist as mature open-source
software. This is engineering-heavy but tractable.

Wheels remain a smart target: low dimensionality with high leverage, cheap fabrication (a
wheel is one printed part — you can close the sim-to-real loop for €5), and clean measurable
objectives.

**Adding TPU makes the project substantially more interesting, and substantially harder.**
More interesting because compliance introduces a genuine, physically rich trade-off that
rigid-body simulation cannot represent at all: compliance buys contact-patch growth, obstacle
conformity, grip and vibration damping, and it costs energy through hysteretic loss. That
trade-off *is* a research question. Harder because you have just moved from a solved
simulation problem to an unsolved one.

## The prior-work situation

**The rigid loop is not novel.** In February 2026 a group published essentially this
pipeline: Bayesian optimisation co-designing rover wheel geometry and steering PID gains in
full-vehicle closed-loop simulation on deformable terrain, 3,000 simulations in 5–9 days,
with hardware validation showing sim-optimised designs preserved their relative ranking on
the physical rover ([arXiv:2602.01535](https://arxiv.org/pdf/2602.01535)).

**Compliant wheel design is also not virgin territory — you are entering the non-pneumatic
tire (NPT) field.** This matters and should be read into before committing. There is a mature
literature on honeycomb spoke structure optimisation to reduce rolling resistance
([Applied Sciences, 2024](https://doi.org/10.3390/app14135425)), TPMS-based spokes with
adaptive directional stiffness optimised via DoE plus response-surface surrogates
([IJPEM-GT, 2025](https://link.springer.com/article/10.1007/s40684-025-00760-x)), spoke design
and material nonlinearity effects on NPT stiffness and durability, and orthogonal-array
studies of spoke parameters on load capacity. The methods you were going to invent — FEA plus
design of experiments plus surrogate optimisation — are the standard toolkit there.

**Good news:** that literature is almost entirely about full-size road tires, quasi-static or
steady-rolling loading, on flat ground, optimising for rolling resistance and load capacity.
Nobody is doing small-scale FDM-printed compliant wheels, optimised in closed-loop dynamic
simulation, for *obstacle traversal* on a mobile robot. That gap is real.

## The seven things that will kill this project if unplanned

1. **The optimiser will exploit your contact solver, not physics.** When fitness is computed
   in simulation, the search *will* opportunistically exploit inaccuracies in the model to
   achieve high fitness with unrealistic behaviour
   ([Koos et al.](https://hal.science/hal-00687617/file/2012ACLI2214.pdf)). Compliant, spoked
   geometries with many contact events are the worst case. See `10-reality-gap.md`.

2. **Naive soft-body simulation will destroy throughput.** A full FEM wheel is 3–5 orders of
   magnitude more expensive than a rigid one. FEM in the inner loop means roughly 40 designs
   in six months. See `06-compliance-rom.md` — the most important section in the plan.

3. **No material characterisation equipment, and printed TPU is not datasheet TPU.** TPU is
   normally treated as isotropic hyperelastic, but FDM introduces process-inherent
   anisotropy, and raster angle clearly affects properties
   ([Prog. Addit. Manuf., 2024](https://link.springer.com/article/10.1007/s40964-024-00937-x)).
   Infill pattern and density change compression behaviour substantially. The material model
   is the single largest source of uncertainty. See `07-materials.md`.

4. **TPU stiffness drifts with use.** Under cyclic loading there is a steep decline in
   absorbed energy over the initial cycles, then the response becomes increasingly dominated
   by viscous effects with continued softening rather than a stable plateau. **The wheel is
   not the same wheel after 500 revolutions.** Direct consequences for hardware protocol.

5. **Fixed controller + varying morphology = confounded results.** Compliance makes this
   worse: a soft wheel needs a different torque profile than a rigid one. Co-optimise
   controller parameters — see `09-optimiser.md`.

6. **Mass, inertia *and stiffness* must be recomputed from geometry every iteration.** For
   compliant wheels effective stiffness depends on geometry, infill and material in a coupled
   way. A constant-stiffness bug is silent and fatal.

7. **A single obstacle course means a single overfitted wheel.** Evaluate over terrain
   distributions, score with risk-sensitive statistics.

## Verdict

Worth doing, and the TPU angle makes it *more* publishable, not less — provided the
compliance modelling strategy (`06-compliance-rom.md`) is treated as the technical core of
the project rather than an implementation detail. Roughly a third of engineering effort
belongs there.
