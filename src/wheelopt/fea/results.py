"""Typed results for the FEA stage. Pure — no numpy dependency beyond arrays passed in.

Invariant 4 (CLAUDE.md): *nothing kills a campaign*. A diverged solve, a failed mesh, a
missing binary and a truncated result file are all ordinary outcomes of running FEA on
hundreds of soft, buckling-prone structures. Every one of them must arrive here as a value,
not an exception. ADR-0005 predicts a meaningful failure rate and asks for it to be logged
as a pipeline health metric, which requires the failures to be *typed* rather than merely
absent.

Two vocabularies meet in this module, deliberately kept apart:

* :class:`FeaStatus` describes what happened to the *process*. Its payload is a log tail
  and an increment count. It has no margin, no limit, and no meaningful ordering.
* :class:`wheelopt.cad.constraints.Violation` describes what the result *means* for the
  design — peak stress against the fatigue limit, buckling load against nominal. These are
  scalar margins, they compose with the existing ``is_feasible``, and so they are reused
  rather than reinvented. Forcing process failures into that shape would mean inventing a
  ``value``/``limit``/``margin`` triple for "the solver timed out", which is worse than
  having two types.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from ..cad.constraints import Violation
from .loadcase import LoadCase

__all__ = [
    "FeaStatus",
    "SolverDiagnostics",
    "LoadCurve",
    "ContactPatch",
    "FeaResult",
    "common_force_n",
]


class FeaStatus(str, Enum):
    """What happened to this evaluation. Only ``OK`` carries physical results."""

    #: Solved to completion and parsed.
    OK = "ok"
    #: Rejected by the analytic pre-filter; no geometry was ever built. Not a failure.
    SCREENED_OUT = "screened_out"
    #: The CAD stage could not produce a solid.
    CAD_FAILED = "cad_failed"
    #: gmsh could not mesh the STEP, or produced a degenerate mesh.
    MESH_FAILED = "mesh_failed"
    #: The deck could not be assembled — missing node set, empty surface, bad material.
    DECK_INVALID = "deck_invalid"
    #: No ``ccx`` binary was found. An environment problem, not a design problem.
    SOLVER_MISSING = "solver_missing"
    #: CalculiX ran but did not reach the end of the step: cutbacks exhausted, or the
    #: increment size fell below the minimum. The expected failure for a buckling design.
    SOLVER_DIVERGED = "solver_diverged"
    #: Wall-clock budget exceeded.
    SOLVER_TIMEOUT = "solver_timeout"
    #: Non-zero exit with no recognisable CalculiX error — a crash, a signal, a full disk.
    SOLVER_CRASHED = "solver_crashed"
    #: The solve finished but its output could not be read.
    PARSE_FAILED = "parse_failed"


@dataclass(frozen=True, slots=True)
class SolverDiagnostics:
    """Pipeline health, not physics. Aggregated across a campaign to track risk R3."""

    n_increments: int = 0
    n_cutbacks: int = 0
    wall_seconds: float = 0.0
    n_nodes: int = 0
    n_elements: int = 0
    #: Final step time reached, as a fraction of the step period. 1.0 means completion.
    completed_fraction: float = 0.0
    #: Worst slave-node penetration observed, metres. A contact-quality check: large values
    #: mean the contact stiffness is too soft and the stiffness curve is not trustworthy.
    max_penetration_m: float | None = None
    #: Tail of solver output, kept for triage when something fails.
    log_tail: str = ""

    def summary(self) -> str:  # pragma: no cover - display only
        return (
            f"{self.n_increments} increments ({self.n_cutbacks} cutbacks), "
            f"{self.n_elements} elements, {self.wall_seconds:.1f} s, "
            f"{self.completed_fraction:.0%} of step"
        )


@dataclass(frozen=True, slots=True)
class LoadCurve:
    """Radial force against imposed displacement, over the whole load/unload sweep.

    Displacement-controlled, so ``delta_m`` is the independent variable and is exact;
    ``force_n`` is the reaction at the indenter reference node.
    """

    #: Indenter displacement into the wheel, metres, positive into contact.
    delta_m: np.ndarray
    #: Reaction force magnitude, newtons.
    force_n: np.ndarray
    #: True on the loading branch, False on unloading.
    loading: np.ndarray
    #: Displacement of the reference node along the in-plane axis that is **not** driven,
    #: metres, signed **positive inward** (toward the hub). ``None`` for every case that holds
    #: that axis, which is all of them except ``TIP_TANGENTIAL``.
    #:
    #: It is here because it is the one thing in this pipeline that can falsify the ring's
    #: choice of tangential element: a claw hinged at its root must pull its tip *in* as it
    #: bends, by ``L(1 - cos φ)``, where a tip on a tangential slide pushes it *out*. The
    #: sweep leaves the axis free and measures which happens. See
    #: :func:`~wheelopt.rom.fit.hinge_kinematics_check` and ``docs/plan/TODO.md`` #27.
    cross_delta_m: np.ndarray | None = None

    @property
    def peak_force_n(self) -> float:
        return float(np.max(self.force_n)) if len(self.force_n) else 0.0

    @property
    def peak_delta_m(self) -> float:
        return float(np.max(self.delta_m)) if len(self.delta_m) else 0.0

    def secant_stiffness_n_per_m(self) -> np.ndarray:
        """``F/delta`` on the loading branch. Undefined at zero displacement."""
        d = self.delta_m[self.loading]
        f = self.force_n[self.loading]
        with np.errstate(divide="ignore", invalid="ignore"):
            k = np.where(d > 0, f / np.where(d > 0, d, 1.0), np.nan)
        return k

    def tangent_stiffness_n_per_m(self) -> np.ndarray:
        """``dF/ddelta`` on the loading branch. Negative values indicate a limit point."""
        d = self.delta_m[self.loading]
        f = self.force_n[self.loading]
        if len(d) < 2:
            return np.zeros(len(d), dtype=np.float64)
        return np.gradient(f, d)


@dataclass(frozen=True, slots=True)
class ContactPatch:
    """Contact geometry as a function of load, sampled on the loading branch."""

    force_n: np.ndarray
    #: Extent along the rolling direction, metres.
    length_m: np.ndarray
    #: Extent across the wheel width, metres.
    width_m: np.ndarray
    #: Summed area of facets in contact, m^2.
    area_m2: np.ndarray
    #: Largest *nodal* normal contact pressure, Pa. **Not mesh-convergent — do not build a
    #: constraint or a comparison on it.** The slave surface is a node set (see the note in
    #: :mod:`wheelopt.fea.extract`), so this is the load carried by whichever single node
    #: happens to be worst placed against the master facets, divided by that node's share of
    #: the area. It is dominated by how many nodes are in contact rather than by how hard
    #: they are pressed: on the verification wheel it *falls* from 1231 kPa to 980 kPa while
    #: the load rises from 0.8 N to 4.4 N, purely because the patch grows from 2 nodes to 23.
    #: Use :attr:`mean_pressure_pa` for anything quantitative and keep this as a diagnostic.
    peak_pressure_pa: np.ndarray
    #: Number of slave nodes carrying load.
    n_nodes: np.ndarray

    @property
    def mean_pressure_pa(self) -> np.ndarray:
        """Normal force over contact area. The comparable pressure measure.

        Still inherits the area estimate's node-count quantisation, so it is coarse at low
        load — but it is driven by the total force rather than by one node, so it is the one
        that behaves like a pressure.
        """
        return np.divide(
            self.force_n,
            self.area_m2,
            out=np.zeros_like(self.force_n, dtype=np.float64),
            where=self.area_m2 > 0.0,
        )

    @property
    def force_range_n(self) -> tuple[float, float]:
        """Lowest and highest load at which contact was actually sampled.

        Note the lower bound is rarely zero. Contact output only exists once nodes touch,
        and a stiff design can jump from no contact to tens of newtons in one increment —
        the bandless verification wheel's first contact sample is at 13.5 N. That gap is why
        :func:`common_force_n` exists.
        """
        if len(self.force_n) == 0:
            return (0.0, 0.0)
        return (float(self.force_n.min()), float(self.force_n.max()))

    def at_force(self, force_n: float) -> tuple[float, float]:
        """Linearly interpolated ``(area_m2, mean_pressure_pa)`` at a given load.

        Comparing two load cases means comparing them at the same *load*, not at the same
        indentation: a step edge is softer than a flat plate, so at equal indentation it
        carries ~30% less force, and a smaller patch under a smaller load says nothing about
        pressure.

        A load outside :attr:`force_range_n` **clamps** to the nearest sampled end rather
        than extrapolating, because extrapolating a contact patch past the last converged
        increment would invent geometry. Clamping silently is its own trap — it returns a
        plausible number for a load the solve never reached — so pick the comparison load
        with :func:`common_force_n` rather than assuming any two sweeps overlap.
        """
        if len(self.force_n) == 0:
            return 0.0, 0.0
        order = np.argsort(self.force_n)
        f = self.force_n[order]
        area = float(np.interp(force_n, f, self.area_m2[order]))
        pressure = float(np.interp(force_n, f, self.mean_pressure_pa[order]))
        return area, pressure


def common_force_n(a: ContactPatch, b: ContactPatch) -> float | None:
    """Highest load both patches actually reached, or ``None`` if they never overlap.

    Two sweeps run to the same indentation can cover disjoint load ranges — a bandless
    wheel and a banded one at the same delta differ by nearly an order of magnitude in
    force, so there is no load at which both were measured and no honest comparison to
    make. Returning ``None`` says so instead of quietly clamping both to their own ends and
    reporting a ratio between two unrelated states.
    """
    if len(a.force_n) == 0 or len(b.force_n) == 0:
        return None
    lo = max(float(a.force_n.min()), float(b.force_n.min()))
    hi = min(float(a.force_n.max()), float(b.force_n.max()))
    return hi if hi >= lo else None


@dataclass(frozen=True, slots=True)
class FeaResult:
    """The complete outcome of one load case on one design.

    Constructed by :func:`wheelopt.fea.runner.run_load_case`, which never raises. Use
    :attr:`ok` before reading any physical field; all of them are ``None`` unless the solve
    completed.
    """

    status: FeaStatus
    load_case: LoadCase
    cache_key: str
    message: str = ""

    curve: LoadCurve | None = None
    patch: ContactPatch | None = None

    #: Mesh-dependent and physically meaningless at re-entrant corners — see the note in
    #: :mod:`wheelopt.fea.extract`. Reported for completeness; do not build a constraint
    #: on it without a spoke-root fillet.
    peak_von_mises_pa: float | None = None
    #: 95th percentile of spoke stress. This is the mesh-convergent one.
    p95_von_mises_pa: float | None = None

    #: Hub-centre to contact-plane distance under load, metres. Must decrease with load.
    loaded_radius_m: np.ndarray | None = None
    #: True if the tangent stiffness went negative, or the branches failed to retrace.
    buckling_detected: bool | None = None
    #: Force at the limit point, newtons, if one was found.
    buckling_load_n: float | None = None

    #: Always ``None`` in fea-0.1.0. A hyperelastic constitutive model is path-independent,
    #: so loading and unloading coincide by construction and the enclosed area is zero by
    #: assumption, not by measurement. A real loss factor needs ``*VISCO`` with a Prony
    #: series, and docs/plan/07-materials.md states the Prony data requires DMA that is not
    #: available. Reporting a fitted zero here would be fabricating a result.
    hysteresis_loss_factor: float | None = None
    #: Enclosed loop area as a fraction of the loading-branch work. Expected ~0; anything
    #: larger is a *numerical* artefact (contact chatter, friction locking) or a sign the
    #: structure found a different equilibrium path on unloading, i.e. it buckled.
    loop_area_fraction: float | None = None

    #: Design-level consequences of this result, in the vocabulary the CAD stage already
    #: uses, so they compose with ``wheelopt.cad.constraints.is_feasible``.
    violations: list[Violation] = field(default_factory=list)
    diagnostics: SolverDiagnostics = field(default_factory=SolverDiagnostics)

    @property
    def ok(self) -> bool:
        return self.status is FeaStatus.OK

    @property
    def is_environment_failure(self) -> bool:
        """Distinguish "this machine is misconfigured" from "this design is hard".

        Only the latter belongs in a campaign's failure-rate statistic.
        """
        return self.status is FeaStatus.SOLVER_MISSING

    def summary(self) -> str:  # pragma: no cover - display only
        if not self.ok:
            return f"{self.status.value}: {self.message}"
        peak = self.curve.peak_force_n if self.curve else float("nan")
        return (
            f"ok: {peak:.1f} N at {self.curve.peak_delta_m * 1e3:.1f} mm, "
            f"{self.diagnostics.summary()}"
        )


def failure(
    status: FeaStatus,
    load_case: LoadCase,
    cache_key: str,
    message: str,
    diagnostics: SolverDiagnostics | None = None,
) -> FeaResult:
    """Build a typed failure. Convenience so callers never construct a half-filled result."""
    return FeaResult(
        status=status,
        load_case=load_case,
        cache_key=cache_key,
        message=message,
        diagnostics=diagnostics or SolverDiagnostics(),
    )
