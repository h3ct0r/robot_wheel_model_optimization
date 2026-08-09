# ADR-0006 — Multi-objective Pareto search, scored on CVaR, never scalarised

**Status:** accepted
**Date:** 2026-08-04

## Context

Wheel design trades off at least four quantities: obstacle capability, energy cost, ride
harshness, and mass. Compliance sharpens the conflict — a soft wheel wins on obstacles and
ride, loses on energy through hysteretic loss.

Simulation is stochastic (terrain seeds) and the material model is uncertain (no
characterisation equipment), so every evaluation is noisy in two independent ways.

## Decision

1. **Keep objectives separate and report the Pareto front.** No weighted-sum scalarisation.
2. **Score each design on CVaR at 25%** (mean of the worst quartile) across
   `k ≥ 8` terrain seeds × `m ≥ 4` material realisations — not the mean.
3. **Optimise with qLogNEHVI** (noisy expected hypervolume improvement) in BoTorch/Ax.
4. **Terminate on measured hypervolume stagnation** with bootstrap confidence intervals — never
   "stop when it stops improving".
5. **Communicate results via three named preference profiles** (obstacle-first,
   efficiency-first, balanced), one champion each.

Recorded as invariants 6 and 7 in `CLAUDE.md`.

## Rationale

**Against scalarisation.** A weighted sum bakes in a preference before the trade-off structure
is known, and cannot recover non-convex regions of the Pareto front. The interesting question
here — *where does compliance stop paying?* — is a question about the shape of the front, and
scalarisation destroys exactly that information.

**For CVaR over mean.** Mean-scoring rewards designs that are excellent on average and
catastrophic in the tail. For hardware transfer the tail is what matters. CVaR also makes the
material-uncertainty handling (RQ4) fall out naturally: a design scoring well under CVaR
across sampled Mooney-Rivlin coefficients is a design robust to *not knowing the material*.

**For qLogNEHVI.** It is designed for the noisy multi-objective setting and is Ax's default
for multi-objective problems.

**Against naive termination.** In a noisy multi-objective setting, "no improvement" is a
statement about sampling noise, not convergence. The original project sketch specified
"repeat until there is no more improvement", which would terminate on noise.

## Consequences

- Per-design cost multiplies by ~4× (material realisations) on top of ~8× (terrain seeds).
  This is precisely why the ROM surrogate matters (ADR-0002).
- Results are a front, not a winner. Communication requires the preference-profile device or
  readers will ask "so which one is best?".
- Baselines must also be evaluated on hypervolume: random search, NSGA-II, and LHS-plus-pick-best
  at equal budget. Without these no claim that the optimiser helps is defensible.
- Topology is handled as a **bandit over families** (one GP per family, allocate by expected
  hypervolume improvement per unit cost) rather than a single mixed kernel — simpler to debug,
  and it produces the per-family comparison for RQ1 for free.

## Revisit if

- The front collapses to effectively one dimension (objectives turn out highly correlated), in
  which case a reduced objective set is more honest than a nominally 4-D front.
- Evaluation noise turns out negligible, weakening the case for CVaR over mean.

## References

- [BoTorch Multi-Objective Bayesian Optimization](https://botorch.org/docs/multi_objective)
- [Constrained multi-objective optimization with qNEHVI and qParEGO](https://botorch.org/docs/v0.14.0/tutorials/constrained_multi_objective_bo)
