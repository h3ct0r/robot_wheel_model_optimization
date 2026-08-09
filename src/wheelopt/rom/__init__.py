"""Reduced-order compliance model — ADR-0002. The technical core.

An N-segment ring of rigid bodies whose joint stiffness and damping are fitted offline to
quasi-static FEA load cases, then run in MuJoCo at full speed.

After ~300 FEA runs, a surrogate maps design parameters -> ROM parameters directly, so
later candidates skip FEA entirely.

The ROM version must appear in every cache key: changing ring discretisation, the fitting
procedure, or material homogenisation invalidates all prior results.

See docs/plan/06-compliance-rom.md.
"""

#: Version of the reduced-order model. Bump on any change that can move the numbers: ring
#: discretisation, the coupling model, the spring law's functional form, the fitting
#: procedure, material homogenisation. Invariant 5 — it belongs in every cache key that
#: covers a ROM result. Nothing caches one yet; the constant exists so that when something
#: does, it is not invented on the spot with no history behind it.
#:
#: - ``rom-0.1.0`` — uncoupled ring: N radial springs, compressions read off the geometry.
#: - ``rom-0.2.0`` — neighbour coupling from the shear band's bending stiffness; the spring
#:   law gained a linear tension branch; compressions now come from a constrained
#:   equilibrium. Uncoupled specs are numerically unchanged, but the version moves anyway
#:   because the fitted coefficients of a *banded* wheel do change.
#: - ``rom-0.3.0`` — a second spring law: ``TabulatedLaw``, piecewise linear over knots, fitted
#:   by non-negative least squares. The constraint on a fitted law changed with it, from "never
#:   softens" to "never pulls" — a design whose segments buckle is now fittable and was not,
#:   so the same curve can produce a different law than it did at 0.2.0. The cubic path is
#:   numerically unchanged; the version moves because which law a caller gets is now a choice.
ROM_VERSION = "rom-0.3.0"

__all__ = ["ROM_VERSION"]
