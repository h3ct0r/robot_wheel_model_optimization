# ADR-0003 — build123d for parametric CAD, not OpenSCAD

**Status:** accepted
**Date:** 2026-08-04

## Context

The project was originally conceived with OpenSCAD as the parametric CAD layer. The addition
of an FEA tier (ADR-0005) and multi-material TPU wheels changed the requirements on the
geometry kernel.

## Decision

Author all wheel geometry in **build123d**. Export **STEP** for FEA and archival, **STL** for
simulation collision and visuals. STL is never the source of truth.

## Rationale

1. **STEP export is mandatory.** The FEA solver needs BREP geometry. Meshing an STL for
   nonlinear hyperelastic analysis with contact is painful and produces low-quality elements.
   OpenSCAD (CGAL/Manifold, CSG) cannot produce STEP; build123d (OCCT) can.

2. **Region tagging for multi-material.** The FEA solver must be told which faces are TPU 95A
   at 40% gyroid infill and which are PETG. That requires named, queryable topological
   entities — a BREP concept, absent from CSG triangle output.

3. **Queryable topology makes manufacturability constraints checkable** rather than heuristic:
   overhang angles, minimum inter-spoke gaps, wall thickness. In OpenSCAD these would be
   inferred from triangle soup.

4. **In-process Python.** No subprocess round-trip, no file parsing, no stdout error handling,
   in a loop that will run thousands of times.

5. **OCCT over CGAL:** NURBS, splines, surface sewing, STL repair, STEP I/O; faster
   STL/STEP/3MF export than OpenSCAD.

## Alternatives considered

**OpenSCAD (original plan) — rejected.** Even with the Manifold backend (non-experimental
since late 2024, 5–30× faster than CGAL fast-csg, itself 30–150× faster than baseline Nef
routines) and SolidPython2 for programmatic generation, a **separate BREP path would still be
required for FEA**. Maintaining two geometry definitions of the same wheel is a reliable
source of silent divergence bugs.

**CadQuery — viable, not chosen.** Same OCCT kernel, same STEP capability. build123d replaces
CadQuery's fluent method-chaining API with stateful context managers, giving normal Python
control flow — loops, sorting, filtering over topology. For generating `N` spokes with varying
curvature under conditional parameters, that matters. They share the OCP wrapper, so objects
are interchangeable; switching later is cheap.

## Consequences

- ~3–5 days of migration cost if any OpenSCAD wheel definitions already exist.
- A build123d/OCCT dependency, which is heavier to install than OpenSCAD.
- Mass properties come from BREP volume directly, which is more accurate than mesh-volume
  integration and supports the invariant that mass and inertia are always derived, never
  hard-coded.
- The STEP output is directly usable for machining, external FEA, or handing to a
  collaborator — useful beyond this project.

## Revisit if

- A hard blocker appears in build123d that CadQuery does not share (switch is cheap — shared
  OCP wrapper).
- The project drops the FEA tier entirely, at which point OpenSCAD becomes viable again.

## References

- [build123d — external tools and libraries](https://build123d.readthedocs.io/en/latest/external.html)
- [CadQuery](https://github.com/cadquery/cadquery)
- [OpenSCAD Manifold backend no longer experimental](https://lists.openscad.org/empathy/thread/D6KV3ZLXHLBHSITSQ5GPUZUKHURU4ABE)
