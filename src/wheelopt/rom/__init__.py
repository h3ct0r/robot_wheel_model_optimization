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
#: - ``rom-0.4.0`` — the ring's frictionless contact force was resolved wrongly. The plate's
#:   normal force on a segment is ``f_r / cos θ``, not ``f_r · cos θ``; MuJoCo, which assumes
#:   neither, matches the former to 6e-11 and the latter to 25% (see ``vertical_reaction_n``).
#:   **Every ring force and every fitted law moves**, the more so the wider the contact patch,
#:   so this invalidates prior results in a way none of the earlier bumps did. Also adds a
#:   tangential degree of freedom for bandless rings (``solve_equilibrium_2dof``), which is
#:   inert until a second segment engages and therefore changes no flat-plate fit.
#: - ``rom-0.5.0`` — the second freedom becomes a **hinge at the claw root**
#:   (``solve_equilibrium_hinge``) rather than a slide at the tip, because a slide lengthens
#:   the claw as it splays and the FEA measures the tip moving the other way (``TODO.md`` #27,
#:   the 2026-08-09 log entry). ``RingSpec`` gains ``root_radius_m``, so a spec built before
#:   this is not a spec built after it. The vertical reaction moves by under 1.5% out to
#:   δ = 25 mm and the flat-plate fits are effectively unchanged; what moves is where the tips
#:   are, which only a rolling contact sees. The slide is kept and reachable, as the thing the
#:   hinge is compared against.
ROM_VERSION = "rom-0.5.0"

__all__ = ["ROM_VERSION"]
