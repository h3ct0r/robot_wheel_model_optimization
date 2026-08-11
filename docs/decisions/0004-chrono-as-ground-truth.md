# ADR-0004 — Project Chrono as ground truth, never in the loop

**Status:** superseded in its gating role by ADR-0008 (2026-08-11) — hardware is the ground truth; Chrono remains the documented option for a sim-to-sim cross-check
**Date:** 2026-08-04
**Depends on:** ADR-0002

## Context

The reduced-order compliance model (ADR-0002) is an approximation. Its validity is the central
claim of the project, so it must be checked against something more trustworthy than itself.
Hardware validation happens only in Phase 4; a computational reference is needed from Phase 1.

## Decision

Use **Project Chrono with ANCF (or Reissner) shell FEA tires** as the computational ground
truth. Never in the inner loop.

Three uses:

1. **ROM validation** (Phase 1 gate) — does the ring model reproduce Chrono's dynamic
   response ranking?
2. **Cross-engine rank correlation audit** (Phase 3 gate) — Spearman ρ between ROM and Chrono
   rankings on top-50 + 20 random designs. ρ > 0.7 to proceed; ρ < 0.4 means stop and fix the
   model.
3. **Final verification** of top designs before printing.

## Why Chrono specifically

Chrono::Vehicle offers full finite element tire representations via ANCF or Reissner shell
elements — the most accurate and most expensive tire models available — and can account for
**simultaneous deformation in tire and soil**. There is a documented `ANCFTire` class and
shipped co-simulation test programs (`test_VEH_HMMWV_Cosimulation`,
`test_VEH_tireRig_Cosimulation`) coupling deformable tires to granular terrain via explicit
force–displacement co-simulation on non-blocking parallel threads.

The exact combination needed — a flexible wheel, on rigid or deformable ground, inside a full
vehicle model — already exists, is validated, and is open source. Nothing else in the
candidate set offers this.

## Consequences

- A second simulation stack must be stood up and maintained, with its own asset pipeline from
  the same STEP geometry. Budget for this in Phase 1.
- The Phase 3 cross-engine gate becomes a genuine go/no-go, not a formality.
- Deformable terrain becomes *available* (Chrono CRM/SCM/DEM) without being *in scope*.
  **Hard rule: no deformable terrain in the inner loop before Phase 3** — see risk R11. Note
  also that SCM is only valid for small sinkage, low slip and near-cylindrical wheels without
  lugs, so lugged or compliant wheels on soil require CRM or DEM, not SCM.

## Revisit if

- Chrono ANCF proves too unstable or slow even for tens of designs — fallback would be
  single-wheel FEA co-simulation rigs only, dropping the full-vehicle reference.
- Hardware data becomes plentiful enough to serve as the primary reference, demoting Chrono
  to a secondary check.

## References

- [Chrono::Vehicle tutorial — tire model hierarchy](https://www.projectchrono.org/assets/slides_3_0_0/5_Vehicle/1_ChronoVehicle.pdf)
- [Chrono ANCFTire class reference](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_a_n_c_f_tire.html)
- [High-fidelity vehicle mobility: nonlinear FE tires on granular material](https://www.sciencedirect.com/science/article/abs/pii/S0022489816301173)
