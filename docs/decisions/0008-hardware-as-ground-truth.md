# ADR-0008 — Hardware as ground truth; Chrono demoted to optional

**Status:** accepted
**Date:** 2026-08-11
**Supersedes:** the *gating role* of ADR-0004. Chrono remains the documented option for a
sim-to-sim cross-check; it no longer blocks anything.

## Context

ADR-0004 made Chrono ANCF the computational ground truth: the Phase 1 gate was "ROM
reproduces Chrono dynamic response ranking with ρ > 0.8 on 10 designs", and the plan named
the ROM-vs-Chrono question the project's blocking unknown. That decision was made when the
robot was a paper specification. Two things have changed:

1. **The robot physically exists**, with a printer beside it. Printing a candidate wheel is
   an overnight job costing tens of grams of TPU; pressing it against a scale is an
   afternoon. The thing Chrono was standing in for — reality — is available at less cost
   than Chrono itself.
2. **The ROM's failure modes turned out to be contact-shaped, not dynamics-shaped.** The
   measured gaps (#31: flank bedding, +75%/−46% element straddle above second-claw
   engagement; the law extrapolating past its swept range) are quasi-static contact
   phenomena a bench press test measures *directly*. A Chrono ANCF tire would model them
   with its own discretisation choices — a second opinion, not a measurement.

Standing up PyChrono (conda-only, its own OCCT collision risk noted in the environment plan)
plus an ANCF tire rig calibrated well enough to arbitrate was weeks of the critical path,
spent building a referee we no longer need.

## Decision

**The validation tier is printed hardware.** The Phase 1 gate becomes:

> The ROM reproduces the **measured quasi-static load curve of a printed wheel** within 10%
> over the single-claw regime, and its multi-claw regime is **calibrated against** (not
> predicted ahead of) the same measurement, on at least 3 printed designs spanning the claw
> family.

Dynamic transfer (the step climb, the washboard ranking) is validated on the robot itself in
Phase 4 as already planned; the bench press test is the new intermediate tier. The DIY
characterisation protocol in `07-materials.md` is promoted from "materials week" to the
validation instrument.

**Chrono is optional.** ADR-0004's engine analysis stands; anyone wanting a sim-to-sim
cross-check (for a publication, or if hardware and simulation disagree and a third opinion
would localise the fault) follows it. Nothing waits on it, and PyChrono is not installed
until someone does.

## Consequences

- The "blocking unknown" (CLAUDE.md) is restated: it is now whether the FEA-calibrated ROM
  reproduces a **printed wheel's** measured behaviour — which also collapses two risks into
  one, since the FEA→print gap (infill, anisotropy, the Gibson-Ashby knock-down) was going
  to need this measurement anyway.
- #31's multi-claw regime gets a calibration path that does not require solving the
  flank-bedding element analytically: measure the printed wheel's whole-wheel curve past
  second-claw engagement and fit the ring's multi-claw response to it, with the analytic
  element retained for the single-claw regime where it is exact.
- The Spearman ρ > 0.7 cross-engine check of Phase 3 becomes a hardware rank-correlation
  check on the printed designs.
- Cost: a bench press rig (kitchen scale, calipers or a printed depth gauge, a clamp) and
  print time. Risk: hardware measurements carry their own noise — mitigated by the protocol
  measuring each design at least twice and treating disagreement between prints as data
  (material realisation spread, which invariant 7 wants sampled anyway).
