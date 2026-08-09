# ADR-0007 — CoACD for convex decomposition, not V-HACD

**Status:** accepted
**Date:** 2026-08-04

## Context

MuJoCo requires convex collision geometry. Wheel geometries in this project are strongly
concave: lugs, spokes, open sectors between spokes. The decomposition quality directly
determines whether contact behaviour is physically meaningful — and a bad decomposition is a
prime source of the solver artifacts the optimiser will exploit.

## Decision

Use **CoACD** for convex decomposition. Enforce MuJoCo's vertex budgets as a hard check:
≤ ~200 vertices per hull for mesh–primitive collision, ≤ 32 for convex–convex. Log hull count
and vertex budget per design.

## Rationale

CoACD uses a **collision-aware concavity metric** based on both the shape's boundary and its
interior volume, rather than a generic concavity threshold. In practice it preserves fine
concave features that V-HACD fills in, and emits **fewer hulls for the same fidelity**
(typically 4–24).

The trade is: slower to decompose, faster and more accurate to simulate. For this project each
mesh is decomposed **once** and simulated **thousands** of times, so the trade is
overwhelmingly favourable.

## Alternatives considered

**V-HACD — rejected.** Fast, deterministic, shipped in nearly every physics-adjacent tool. But
it fills concave features, which for a spoked wheel means the open sectors between spokes get
partially bridged — silently converting a compliant open structure into something closer to a
solid disc for collision purposes. That is exactly the kind of geometry-model divergence that
produces unfalsifiable simulation results.

**Manual hulls — rejected.** Not viable when geometry is generated programmatically across
thousands of designs.

## Consequences

- Decomposition settings become part of the pipeline version and therefore part of every cache
  key.
- Hull budget is a **hard constraint**: a design whose decomposition exceeds the budget is
  marked infeasible rather than simulated with degraded geometry.
- Decomposition settings must be included in the solver-perturbation audit — re-running with a
  different hull budget is one of the perturbations used to detect artifact-exploiting designs.
- Visual and collision meshes stay separate; the visual mesh keeps full resolution.

## Revisit if

- Decomposition time becomes a bottleneck at campaign scale (unlikely — it is cached and
  amortised over thousands of rollouts).
- MuJoCo gains efficient native concave collision support.

## References

- [CoACD: Approximate Convex Decomposition with Collision-Aware Concavity (SIGGRAPH 2022)](https://arxiv.org/pdf/2205.02961)
- [CoACD code](https://github.com/SarahWeiii/CoACD)
