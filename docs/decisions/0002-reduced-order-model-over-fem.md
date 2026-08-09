# ADR-0002 — Reduced-order compliance model, not FEM in the loop

**Status:** accepted
**Date:** 2026-08-04
**Depends on:** ADR-0001, ADR-0004, ADR-0005

## Context

Compliant TPU wheels must be evaluated in closed-loop dynamic simulation thousands of times.
Full FEM of the wheel inside that loop is roughly 10³–10⁵× too slow — a realistic campaign
would complete on the order of 40 designs in six months. Rigid-body simulation, at the other
extreme, cannot represent contact-patch growth, load-dependent rolling radius, hysteretic
loss, or edge envelopment — the entire set of effects the project exists to study.

## Decision

Model each compliant wheel as an **`N`-segment ring of rigid bodies** (initially `N = 24–48`)
connected to the hub and to each other by joints whose stiffness and damping are **fitted
offline to quasi-static FEA load cases**. Run that ring model in MuJoCo at full speed.

After ~300 FEA runs, fit a surrogate mapping **design parameters → ROM parameters**, so
subsequent candidates skip FEA entirely.

**FEA never runs inside the optimisation loop.** It is an offline, cached, per-design
pre-processing step. Recorded as invariant 1 in `CLAUDE.md`.

## Alternatives considered

**Full FEM in the loop (Chrono ANCF, MuJoCo flex) — rejected on throughput.** Retained as a
verification tier only (ADR-0004).

**Rigid wheel + soft contact (`solref`/`solimp`) — rejected as insufficient.** It produces a
springy contact point, not a growing contact patch, correct pressure distribution, hysteresis,
or a load-dependent loaded radius. Retained explicitly as an *ablation* (fidelity level L1) to
demonstrate that rigid-contact simulation mis-ranks compliant wheels.

**Rigid hub + per-contact-point radial spring-damper (L2) — rejected as a primary model.**
Captures radial compliance and damping but not patch shape, spoke buckling, or lateral
coupling. Cheap fallback if the ring model proves unstable.

## Precedent

This is not novel in principle and should not be presented as such. The automotive tire field
solved it: **FTire** represents a full 3-D nonlinear tire with **80–200 lumped-mass nodes**
replacing the cord structure, running at **5–20× real-time**. Critically, flexible-ring tire
models have been **generated virtually from FEA rather than from physical measurement** —
exactly this project's situation, given no material test equipment.

The novelty is the application: closed-loop *dynamic obstacle traversal* at robot scale, with
the FEA step surrogated away to enable design *search*.

## Consequences

- The ROM is the project's central technical risk. Phase 1 gate (week 14) exists specifically
  to test it: ROM must reproduce FEA radial stiffness within 10% and Chrono dynamic ranking
  with ρ > 0.8.
- A dedicated batch FEA tier becomes mandatory (ADR-0005).
- Chrono becomes necessary as ground truth for validating the ROM (ADR-0004).
- **Ring discretisation `N` is a modelling parameter that can be exploited by the optimiser.**
  A discretisation sweep is therefore a required part of the solver-perturbation audit — a
  design whose score depends on `N` is an artifact, not a design.
- Every cache key must include the ROM version. Changing discretisation, fitting procedure or
  material homogenisation invalidates all prior results.
- The paper must state plainly what the ROM does *not* capture: high-frequency dynamics beyond
  the discretisation, large-strain nonlinearity outside the fitted range, local stress
  concentration, true post-buckling behaviour, progressive cyclic softening.

## Revisit if

- The Phase 1 gate fails. Fallback is family `T6` (soft tread on rigid hub), a far easier
  modelling problem, with correspondingly narrowed claims.
- A learned dynamics residual trained on FEA/Chrono rollouts outperforms the hand-designed
  ring model — this is extension 6 in `docs/plan/15-extensions.md` and would supersede this
  ADR rather than contradict it.

## References

- [FTire — physically based application-oriented tyre model](https://www.researchgate.net/publication/232851524_FTire_A_physically_based_application-oriented_tyre_model_for_use_with_detailed_MBS_and_finite-element_suspension_models)
- [Virtual Generation of Flexible Ring Tire Models Using Finite Element Analysis](https://www.researchgate.net/publication/355688394_Virtual_Generation_of_Flexible_Ring_Tire_Models_Using_Finite_Element_Analysis_Application_to_Dynamic_Cleat_Simulations)
