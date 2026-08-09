# ADR-0005 — CalculiX for the batch quasi-static FEA tier

**Status:** accepted
**Date:** 2026-08-04
**Depends on:** ADR-0002

## Context

The ROM pipeline needs quasi-static nonlinear FEA with hyperelastic materials and contact,
run unattended on hundreds of designs, producing stiffness curves, contact patch data,
hysteresis loops, buckling detection and peak stress.

Requirements: nonlinear large-deformation, hyperelastic material models, contact,
scriptable/batch, restartable, free.

## Decision

**CalculiX** as the default batch FEA driver. **FEniCSx** as the escape hatch if proper
viscoelasticity becomes necessary.

## Rationale

CalculiX handles nonlinear structural and contact problems with material nonlinearity and
large deformation, uses a file-driven solver with Abaqus-like input decks, and is explicitly
suited to batch automation. It is robust and boring — the right properties for something that
must run 500 times unattended.

## Alternatives considered

**FEniCSx — strong second, kept as escape hatch.** Cross-validated against Abaqus on
hyperelastic large-deformation problems with sub-percent agreement, and a large-deformation
viscoelasticity theory for elastomeric materials has been implemented in it. Choose this if
hysteretic rolling resistance needs to be quantitative rather than qualitative. Cost: writing
variational forms, and more implementation effort per load case.

**Chrono::FEA — viable.** One codebase for FEA and multibody, ANCF shells suit thin rims and
spokes. Rejected as the batch tier because Chrono is already the ground-truth tier
(ADR-0004), and using the same code for both the reference and the thing being referenced
would undermine the cross-engine audit's independence.

**Abaqus / ANSYS — use if a licence exists.** The NPT literature is almost entirely Abaqus, so
published results would be easier to reproduce and compare against. Not assumed available.

**Elmer — rejected.** Strength is multiphysics coupling, which is not the need here.

## Consequences

- FEA is a subprocess with file I/O, not an in-process library call. Acceptable because it is
  offline and cached.
- Input decks are generated from STEP geometry plus a material card. Requires a meshing step —
  Gmsh is the natural companion.
- **Convergence failures must be a typed infeasibility, never a crash** (invariant 4). Soft,
  buckling geometry converges badly; expect a meaningful failure rate and log it as a pipeline
  health metric (risk R3).
- Mesh convergence study runs once per topology family, not per design.
- Keeping the ground-truth tier (Chrono) and the batch tier (CalculiX) as independent codebases
  preserves the independence of the cross-engine audit.

## Revisit if

- Quantitative hysteresis becomes a required result rather than a qualitative ranking — switch
  to FEniCSx.
- Batch convergence rate proves unacceptably low on compliant geometry.

## References

- [Free FEA programs — open-source comparison](https://caeflow.com/fea/free-fea-program/)
- [Large deformation viscoelasticity for elastomers in FEniCSx](https://www.sciencedirect.com/science/article/abs/pii/S0020768324003822)
