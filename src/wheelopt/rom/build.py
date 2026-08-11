"""Design -> FEA -> ring, in one place, so that two CLIs cannot disagree about it.

`scripts/run_step.py` and `scripts/run_rover.py` both need the same chain — press the design
against a plate, turn the curve into a segment law, optionally measure the claw's tangential
response — and the failure this module exists to prevent is the two of them drifting into
building *different rings from the same flags*. That is not hypothetical here: the same
argument already forced `cad/cli.py` into existence so two CLIs could not produce different
``design_hash`` values, and `hashing.py` for the same reason a level down.

Everything below returns a typed result and never raises for a design's own behaviour
(invariant 4). A missing solver, a diverged solve and a curve that cannot be fitted are all
messages, not tracebacks.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np

from ..cad.materials import MaterialSpec
from ..cad.params import WheelParams
from ..fea.loadcase import LoadCase, LoadCaseKind, MeshSpec, SolverSpec
from .fit import RingFit, ring_from_claw_curve, validate_ring
from .ring import RadialLaw, RingSpec, ring_for_design, second_contact_delta_m

__all__ = ["BuiltRing", "LawKind", "build_ring", "measure_tangential_law"]

#: Where the segment law comes from. ``"cubic"`` and ``"table"`` deconvolve the whole-wheel
#: curve; ``"claw"`` measures one claw and does no fit at all (``TODO.md`` #18, #29).
LawKind = Literal["cubic", "table", "claw"]

#: The plane-strain screening mesh. One setting, quoted once.
_MESH_2D = {"size_spoke_m": 0.0025, "size_rim_m": 0.003, "size_hub_m": 0.002}
_MESH_3D = {"size_spoke_m": 0.008, "size_rim_m": 0.010, "size_hub_m": 0.010}


@dataclass(frozen=True, slots=True)
class BuiltRing:
    """A ring and its law, or the reason there isn't one."""

    spec: RingSpec | None = None
    fit: RingFit | None = None
    message: str = ""
    #: True when every sweep behind this ring came out of the cache. Worth surfacing rather
    #: than inferring from the clock: the difference between a cache hit and a cold CalculiX
    #: solve is seconds against minutes, and a caller waiting on the second one should be told
    #: that is what it is waiting for.
    cached: bool = False
    #: Wall time the solver reported, summed over the sweeps. Zero for a fully cached build.
    solver_seconds: float = 0.0

    @property
    def ok(self) -> bool:
        return self.spec is not None and self.fit is not None

    @property
    def validity_delta_m(self) -> float:
        """Indentation beyond which this ring is **not validated**, metres.

        Second-claw engagement (:func:`~wheelopt.rom.ring.second_contact_delta_m`). Below it a
        bandless claw ring reproduces the FEA to 0.036% — the whole wheel *is* one claw, so
        there is nothing to get wrong. Above it the two available elements straddle the FEA
        from opposite sides, +74.7% with a radial slide and −45.6% with a root hinge, and no
        choice of law repairs that because it is the element (``TODO.md`` #31).

        Exposed as a number rather than left in a docstring so that callers can say how much
        of a run was outside it. A limit nobody checks is a limit nobody keeps.
        """
        if self.spec is None or self.spec.is_coupled:
            return float("inf")   # a banded ring is a different model with a different gap
        return second_contact_delta_m(self.spec)

    def validity_note(self, peak_delta_m: float | None = None) -> str:
        """One line about where this ring is trustworthy, and whether a run left that range."""
        limit = self.validity_delta_m
        if not np.isfinite(limit):
            return ""
        line = f"valid to delta {limit * 1e3:.2f} mm (second-claw engagement)"
        if peak_delta_m is None:
            return line
        if peak_delta_m <= limit:
            return f"{line}; this run peaked at {peak_delta_m * 1e3:.2f} mm — inside it"
        return (f"{line}; this run peaked at {peak_delta_m * 1e3:.2f} mm, "
                f"{peak_delta_m / limit:.1f}x BEYOND it — the elements straddle the FEA by "
                "+75%/-46% there (TODO #31)")

    @property
    def law(self) -> RadialLaw | None:
        return self.fit.law if self.fit is not None else None

    @property
    def missing_solver(self) -> bool:
        """True when the failure was the environment rather than the design."""
        return "solver_missing" in self.message


def _was_cached(result) -> bool:
    """Whether the runner served this from cache. ``_load_cached`` marks it in the log tail."""
    return "(cached)" in (result.diagnostics.log_tail or "")


def _plate_curve(
    params: WheelParams, material: MaterialSpec, *, claw_sector: bool, plane_strain: bool,
    delta_max_m: float, n_points: int, cache_root: Path, n_threads: int,
):
    """One RADIAL_FLAT sweep. Returns ``((delta, force, result), "")`` or ``(None, message)``."""
    mesh = (MeshSpec(dimension=2, claw_sector=claw_sector, **_MESH_2D)
            if plane_strain or claw_sector else MeshSpec(**_MESH_3D))
    from ..fea.runner import run_load_case

    case = LoadCase(kind=LoadCaseKind.RADIAL_FLAT, delta_max_m=delta_max_m,
                    n_points_per_branch=n_points)
    result = run_load_case(params, material, case, mesh_spec=mesh,
                           solver=SolverSpec(n_threads=n_threads), cache_root=cache_root)
    if not result.ok:
        return None, f"{result.status.value}: {result.message}"
    loading = result.curve.loading
    return (result.curve.delta_m[loading], result.curve.force_n[loading], result), ""


def build_ring(
    params: WheelParams,
    material: MaterialSpec,
    *,
    law: LawKind = "cubic",
    n_segments: int = 24,
    plane_strain: bool = False,
    delta_max_m: float = 0.006,
    n_points: int = 6,
    cache_root: Path,
    n_threads: int = 4,
) -> BuiltRing:
    """Press the design against a plate and return a ring fitted — or *measured* — from it.

    ``"cubic"`` and ``"table"`` deconvolve the whole-wheel curve into a segment law, with
    ``n_segments`` a free discretisation parameter. ``"claw"`` does no fit: it presses **one
    claw** on the same plate, takes that curve as the segment law directly, and spends the
    whole-wheel curve on :func:`~wheelopt.rom.fit.validate_ring` — a held-out check rather than
    training data. Bandless designs only, and the segment count is then ``n_spokes``, not a
    choice.
    """
    if law == "claw" and params.has_shear_band:
        return BuiltRing(message=(
            "--law claw needs a bandless design: with a band the claws share load through it "
            f"and one claw's curve is not a segment law. rim_thickness_mm is "
            f"{params.rim_thickness_mm:g}"
        ))
    if params.is_l_claw:
        # Refused rather than fitted (`TODO.md` #35). Every ring element this project has —
        # the radial slide and the root hinge alike — carries contact at a *point* on the
        # segment's own radius. An L claw's foot beds along an arc and the load travels down
        # it as the wheel rolls, so a ring built from this design would run, produce a curve,
        # and describe a radial claw of the same length. That is the failure this repo keeps
        # finding: not a crash, a plausible number about a different wheel. The geometry, the
        # screening and the FEA tiers all handle T7L; only the reduced-order model does not.
        return BuiltRing(message=(
            f"the ring ROM cannot represent an L claw: tip_hook_mm is "
            f"{params.tip_hook_mm:+g}, and a {abs(params.tip_hook_mm):g} mm foot carries "
            "contact along an arc while every segment element here carries it at a point on "
            "the segment's own radius. Fitting one anyway would describe a plain radial claw "
            "of the same length. Set --tip-hook 0 for the T7 claw, or see TODO #35"
        ))
    common = {"plane_strain": plane_strain, "delta_max_m": delta_max_m,
              "n_points": n_points, "cache_root": cache_root, "n_threads": n_threads}
    curve, message = _plate_curve(params, material, claw_sector=False, **common)
    if curve is None:
        return BuiltRing(message=message)
    delta, force, wheel_result = curve
    cached = _was_cached(wheel_result)
    seconds = wheel_result.diagnostics.wall_seconds

    try:
        if law == "claw":
            claw, message = _plate_curve(params, material, claw_sector=True, **common)
            if claw is None:
                return BuiltRing(message=f"claw sector: {message}")
            claw_delta, claw_force, claw_result = claw
            cached = cached and _was_cached(claw_result)
            seconds += claw_result.diagnostics.wall_seconds
            spec, segment_law = ring_from_claw_curve(params, claw_delta, claw_force)
            return BuiltRing(spec=spec,
                             fit=validate_ring(spec, segment_law, delta, force),
                             cached=cached, solver_seconds=seconds)
        from .fit import fit_spring_law, fit_tabulated_law

        spec = ring_for_design(params, material, n_segments=n_segments)
        fitted = (fit_spring_law(spec, delta, force) if law == "cubic"
                  else fit_tabulated_law(spec, delta, force))
        return BuiltRing(spec=spec, fit=fitted, cached=cached, solver_seconds=seconds)
    except Exception as exc:  # noqa: BLE001 - a bad curve is a result, not a crash
        return BuiltRing(message=f"{type(exc).__name__}: {exc}")


def measure_tangential_law(
    params: WheelParams,
    material: MaterialSpec,
    spec: RingSpec,
    *,
    element: Literal["hinge", "slide"] = "hinge",
    sweep_max_m: float | None = None,
    cache_root: Path,
    n_threads: int = 4,
) -> tuple[RadialLaw | None, tuple[np.ndarray, np.ndarray] | None, str]:
    """The claw's own tangential curve, as a law for ``element``.

    One sweep, two readings: as a *slide* law it is force against tip travel straight off the
    solver; as a *hinge* law it is moment against root rotation, the same points in different
    coordinates. Tabulated rather than a stiffness because the claw stiffens 3.6x in secant
    between 4 mm and one claw length as it rotates toward the load.

    Returns ``(law, kinematics, message)``. ``kinematics`` is the measured versus predicted
    inward tip travel — the check from outside the ROM that says whether the hinge idealisation
    describes this claw — or ``None`` if the solver did not report the free axis.
    """
    from ..fea.runner import run_load_case
    from .fit import hinge_kinematics_check, hinge_law_from_tip_curve, law_from_claw_curve

    sweep_max_m = sweep_max_m if sweep_max_m is not None else 0.9 * spec.claw_length_m
    mesh = MeshSpec(dimension=2, claw_sector=True, **_MESH_2D)
    case = LoadCase(kind=LoadCaseKind.TIP_TANGENTIAL, delta_max_m=sweep_max_m,
                    n_points_per_branch=10, friction_mu=0.0)
    result = run_load_case(params, material, case, mesh_spec=mesh,
                           solver=SolverSpec(n_threads=n_threads), cache_root=cache_root)
    if not result.ok:
        return None, None, f"{result.status.value}: {result.message}"
    load = result.curve.loading
    delta, force = result.curve.delta_m[load], result.curve.force_n[load]
    try:
        law = (hinge_law_from_tip_curve(delta, force, spec.claw_length_m) if element == "hinge"
               else law_from_claw_curve(delta, force))
    except Exception as exc:  # noqa: BLE001 - a bad curve is a result, not a crash
        return None, None, f"{type(exc).__name__}: {exc}"

    kinematics = None
    if result.curve.cross_delta_m is not None:
        keep = delta > 0.0
        kinematics = hinge_kinematics_check(
            delta[keep], result.curve.cross_delta_m[load][keep], spec.claw_length_m
        )
    return law, kinematics, ""
