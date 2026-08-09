# 14 — CAD and asset pipeline

## Why build123d, not OpenSCAD

With compliance in scope this stops being a preference and becomes close to a requirement.
See ADR-0003.

- **STEP export is mandatory.** The FEA solver needs BREP geometry, not triangle soup. Meshing
  an STL for nonlinear hyperelastic analysis with contact is painful and low-quality. OpenSCAD
  cannot produce STEP; build123d/CadQuery can, via OCCT.
- **Region tagging for multi-material.** The FEA solver must be told "these faces are TPU 95A
  at 40% gyroid infill, these are PETG." That requires named, queryable topological entities —
  a BREP concept, absent in CSG triangle output.
- **Python-native, in-process.** No subprocess round-trip in a loop run thousands of times.
- **OCCT over CGAL:** NURBS, splines, surface sewing, STL repair, STEP I/O. Faster STL/STEP/3MF
  export than OpenSCAD.
- **Queryable topology** makes manufacturability constraints (overhang angles, minimum
  inter-spoke gaps, wall thickness) *checkable* rather than heuristic.
- **build123d vs CadQuery:** build123d replaces CadQuery's fluent method chaining with stateful
  context managers, giving normal Python control flow — loops, sorting, filtering over
  topology. For generating `N` spokes with varying curvature under conditional parameters,
  that matters. They share the OCP wrapper, so objects are interchangeable.

**If OpenSCAD must be kept:** use the Manifold backend (`--backend=manifold`, non-experimental
since late 2024, 5–30× faster than CGAL fast-csg, which was itself 30–150× faster than
baseline Nef routines), drive it via `openscad -D` from Python, and use SolidPython2 to
generate SCAD programmatically. But a separate BREP path for FEA is still required, and
maintaining two geometry definitions is a reliable source of bugs.

## Asset pipeline

1. **Watertightness check** on every export. Reject non-manifold meshes before they reach any
   solver — a leaky mesh produces silently wrong contact.

2. **Convex decomposition with CoACD**, not V-HACD. CoACD uses a collision-aware concavity
   metric based on boundary *and* interior volume, preserves fine concave features V-HACD
   fills in, and emits fewer hulls for the same fidelity (typically 4–24). Slower to
   decompose, faster and more accurate to simulate — the right trade when each mesh is
   decomposed once and simulated thousands of times. Respect MuJoCo's guidance: ≤ ~200
   vertices per hull for mesh–primitive collision, ≤ 32 for convex–convex. Enforce as a hard
   check; log hull count and vertex budget per design. See ADR-0007.

3. **Separate visual and collision meshes.** Visual can be full resolution; collision must be
   the decomposition.

4. **Mass properties from BREP volume × effective density**, never hand-specified:
   `ρ_eff = ρ_material × (shell_fraction + infill_fraction × infill_density)`.

5. **FEA mesh from STEP**, not STL. Second-order tetrahedra, or shells for thin spokes. Mesh
   convergence study once per topology family, not per design.

6. **Version the pipeline** and include the version in every cache key.
