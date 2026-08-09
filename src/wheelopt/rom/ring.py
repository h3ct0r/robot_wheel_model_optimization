"""The segmented ring: geometry, and its quasi-static load-deflection response.

Pure numpy. Nothing here imports MuJoCo — the point of the split is that the *physics of the
reduced-order model* can be tested, and its fit error measured, on a machine with no
simulator. :mod:`wheelopt.rom.mjcf` builds the MuJoCo model from the same parameters, and
``scripts/run_rom.py`` checks the two agree. If they disagree, the bug is in the MJCF, not in
the model.

The model
---------
``n_segments`` rigid segments evenly spaced around the hub, each on a radial spring
(``docs/plan/06-compliance-rom.md`` §3, FTire's structure at a smaller node count). Pressing
the wheel onto a flat plate by ``δ`` compresses the segment at angle ``θ`` from the contact
point by ``u(θ) = R - (R - δ)/cos θ`` where that is positive (:func:`penetrations`), and the
plate reacts each compressed segment with

    F(δ) = Σ_i f(u_i) / cos θ_i

**Divided, not multiplied** — see :func:`vertical_reaction_n`, which is where the reasoning
and the measurement that settled it are written down.

Note what that means for the *inverse* problem: a measured ``F(δ)`` does not give ``f(u)``
directly, because every δ mixes many different ``u`` values. Fitting is a deconvolution, which
is why :mod:`wheelopt.rom.fit` exists rather than a division.

The shear band
--------------
§3 of the plan also asks for neighbour joints fitted to the shear band, and that term is what
makes the ring a *ring* rather than ``N`` independent legs. It arrives as two stiffnesses on
:class:`RingSpec` — :attr:`~RingSpec.band_bending_n_per_m` against a change of curvature and
:attr:`~RingSpec.band_hoop_n_per_m` against a change of circumference — and it changes the
model's character rather than adding a parameter to it: a segment's compression is no longer
*given* by geometry, because a segment the plate does not touch can be dragged inward by its
neighbours or pushed outward past ``R``. The sum above is then only the bandless case, and
everything else goes through :func:`solve_equilibrium`, a constrained equilibrium whose
contact set is found rather than assumed. For the **bandless** topology both stiffnesses are
genuinely zero (``docs/plan/04-design-space.md`` §``T3b``), the closed form is exact, and it
is the default so that a ring nobody has given a band to behaves as it did before.

What the band model still misses: it bends and it stretches, but it does not **shear**, and a
non-pneumatic shear band is named for the deformation it is designed to carry. A pure bending
band is stiffer against conforming to the ground than the real one, and the symptom is visible
— against the tiny design's measured 34 mm contact patch this ring puts only three segments on
the plate at 48 segments where the geometry alone would put seven. Treat the patch length from
this ROM as a lower bound until a shear term exists.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import numpy as np

if TYPE_CHECKING:  # Keeps this module importable, and testable, with nothing else installed.
    from ..cad.materials import MaterialSpec
    from ..cad.params import WheelParams

__all__ = [
    "RadialLaw",
    "RingSpec",
    "RingState",
    "SegmentState2D",
    "SegmentStateHinge",
    "SpringLaw",
    "TabulatedLaw",
    "TipEquivalentLaw",
    "bending_coupling_n_per_m",
    "coupling_matrix",
    "curvature_operator",
    "hoop_coupling_n_per_m",
    "penetrations",
    "polygon_drop_m",
    "ramp_basis",
    "ride_height_ripple_m",
    "ring_for_design",
    "ring_force_2dof_n",
    "ring_force_hinge_n",
    "ring_force_n",
    "segment_angles",
    "solve_equilibrium",
    "solve_equilibrium_2dof",
    "solve_equilibrium_hinge",
    "symmetric_force_n",
    "tip_radius_hinge_m",
    "tip_radius_slide_m",
    "uniform_knots",
    "vertical_reaction_n",
]

#: Segment counts the plan calls for. Not enforced — `verify` sweeps outside it deliberately —
#: but a ring this coarse stops resolving the contact patch, and one this fine costs
#: simulation time for no fidelity.
SEGMENT_RANGE = (24, 48)


@dataclass(frozen=True, slots=True)
class RingSpec:
    """Discretisation and geometry of one ring model."""

    #: Undeformed outer radius, metres. The FEA wheel's ``outer_radius_mm`` in SI.
    radius_m: float
    n_segments: int = 24
    #: The shear band's resistance to a change of *curvature*, N/m — the three-term neighbour
    #: coupling. Zero means the segments are independent legs, which is the *correct* model
    #: for the bandless topology and an approximation for every other one.
    band_bending_n_per_m: float = 0.0
    #: The shear band's resistance to a change of *circumference*, N/m. Not a neighbour term
    #: at all: it couples every segment to every other, because stretching a hoop is a global
    #: thing. Omitting it is not a small error — see :func:`hoop_coupling_n_per_m`.
    band_hoop_n_per_m: float = 0.0
    #: Radius at which a segment is attached to the hub, metres — the claw root. Only the
    #: **hinge** model reads it (:func:`solve_equilibrium_hinge`), because only that model has
    #: an element with a length: a radial slide and a tangential slide both act at the tip and
    #: never ask where the other end is. Zero means "not stated", and the hinge solver refuses
    #: it rather than silently hinging at the wheel's centre — a claw pivoting about the axle
    #: sweeps its tip along a circle of radius ``R`` and can therefore never indent at all.
    root_radius_m: float = 0.0

    def __post_init__(self) -> None:
        if self.radius_m <= 0:
            raise ValueError("radius_m must be positive")
        if self.n_segments < 3:
            raise ValueError("a ring needs at least three segments")
        if min(self.band_bending_n_per_m, self.band_hoop_n_per_m) < 0:
            raise ValueError("band stiffnesses must be non-negative")
        if not 0.0 <= self.root_radius_m < self.radius_m:
            raise ValueError(
                f"root_radius_m must be in [0, radius_m); got {self.root_radius_m} "
                f"against a radius of {self.radius_m}"
            )

    @property
    def segment_arc_m(self) -> float:
        """Arc length one segment occupies on the undeformed ring."""
        return 2.0 * np.pi * self.radius_m / self.n_segments

    @property
    def claw_length_m(self) -> float:
        """Root-to-tip length of one segment, metres. ``R - root_radius_m``.

        Equals ``radius_m`` when the root radius was never stated, which is the degenerate
        reading and the reason :func:`solve_equilibrium_hinge` refuses that case instead of
        using this value.
        """
        return self.radius_m - self.root_radius_m

    @property
    def is_coupled(self) -> bool:
        """Whether the band is present at all. Selects the solver, so it is not cosmetic."""
        return max(self.band_bending_n_per_m, self.band_hoop_n_per_m) > 0.0


@runtime_checkable
class RadialLaw(Protocol):
    """What the ring, the fit, the MJCF and the scenario runners need from a spring law.

    Two implementations: :class:`SpringLaw`, a cubic, and :class:`TabulatedLaw`, a monotone
    piecewise-linear table. Everything downstream is written against this protocol so that
    changing which one a design uses is a decision made once, at the fit, and not a decision
    every consumer has to re-make.
    """

    def force_n(self, u_m: np.ndarray | float) -> np.ndarray:
        """Radial force at compression ``u``, newtons. ``f(0) = 0``; ``u < 0`` is extension."""
        ...

    def stiffness_n_per_m(self, u_m: np.ndarray | float) -> np.ndarray:
        """Tangent ``df/du``, N/m."""
        ...

    @property
    def is_valid_spring(self) -> bool:
        """Whether the law never *pulls* while compressed. The gate on :attr:`RingFit.ok`.

        Deliberately not "never softens". A monotone requirement was the original gate and it
        rules out buckling, which is the phenomenon under study — see :class:`TabulatedLaw`.
        A cubic answers this conservatively (it has no way to soften and recover safely
        without also folding), a table answers it exactly.
        """
        ...

    @property
    def is_monotone_nonneg(self) -> bool:
        """Whether the law additionally never softens. Reported, not required."""
        ...


@dataclass(frozen=True, slots=True)
class SpringLaw:
    """Radial force of one segment against its own compression, newtons.

    Cubic in the compression: ``f(u) = a·u + b·u² + c·u³``, with no constant term because a
    segment at zero compression carries zero load. Cubic because the FEA curves are
    *stiffening* — every measured ``k_r(δ)`` in the log rises with load — and three terms is
    the fewest that can bend that way without the freedom to oscillate.

    Coefficients are per segment, in SI: ``a`` is N/m, ``b`` N/m², ``c`` N/m³.

    In tension the law is **linear**, ``f(u) = a·u`` for ``u < 0`` — the tangent at the
    origin, continued. Extension is unreachable in the uncoupled model, where ``u`` is a
    geometric interference and never negative, so this changes no result that predates
    coupling. It matters once the band is there: a segment just outside the contact patch is
    pulled *outward* past ``R`` by its neighbours, and the spoke holding it resists, because
    a spoke is anchored at both ends. The earlier reading — that a segment "cannot pull", so
    ``f = 0`` for ``u < 0`` — is not merely unphysical here, it is singular. A limp segment
    contributes no diagonal stiffness, and the coupling matrix on its own has the two
    rigid-translation modes in its null space (see :func:`curvature_operator`), so the
    equilibrium solve has no unique answer. The cubic terms are deliberately *not* continued:
    they are fitted to compression data and a cubic extrapolated backwards turns over.
    """

    a: float
    b: float = 0.0
    c: float = 0.0

    def force_n(self, u_m: np.ndarray | float) -> np.ndarray:
        """Force at compression ``u``, newtons. Negative ``u`` is extension; see the class."""
        u = np.asarray(u_m, dtype=np.float64)
        compressed = np.where(u > 0.0, u, 0.0)
        stretched = np.where(u < 0.0, u, 0.0)
        return (self.a * compressed + self.b * compressed**2 + self.c * compressed**3
                + self.a * stretched)

    def stiffness_n_per_m(self, u_m: np.ndarray | float) -> np.ndarray:
        """Tangent stiffness ``df/du``, for reporting, for the MJCF, and for Newton.

        The clip to zero gives ``a`` in extension, which is the tangent of the linear
        tension branch in :meth:`force_n` — the two agree, and the Newton solve in
        :func:`solve_equilibrium` needs them to.
        """
        u = np.asarray(u_m, dtype=np.float64)
        u = np.where(u > 0.0, u, 0.0)
        return self.a + 2.0 * self.b * u + 3.0 * self.c * u**2

    @property
    def is_monotone_nonneg(self) -> bool:
        """True if the law never softens or pulls over a plausible compression range.

        A least-squares fit is free to return coefficients that make ``f`` fold back on
        itself, which is not a spring and would make the MuJoCo model unstable in a way that
        looks like a contact bug. Checked over 0-20 mm, well past any δ this project uses.
        """
        u = np.linspace(0.0, 0.020, 201)
        return bool(np.all(self.force_n(u) >= -1e-12)
                    and np.all(self.stiffness_n_per_m(u) >= -1e-12))

    @property
    def is_valid_spring(self) -> bool:
        """For a cubic, the same conservative test as :attr:`is_monotone_nonneg`.

        A cubic that dips and recovers is representable, but the coefficients that do it also
        make the extrapolation past the fitted range turn over, and there is no way to clamp
        that without leaving the cubic. :class:`TabulatedLaw` is the law to reach for when a
        design needs a softening branch; this one stays monotone or is rejected.
        """
        return self.is_monotone_nonneg


def uniform_knots(u_max_m: float, n_intervals: int) -> np.ndarray:
    """``n_intervals + 1`` knots evenly spaced over ``[0, u_max]``, starting at zero."""
    if u_max_m <= 0:
        raise ValueError("u_max_m must be positive")
    if n_intervals < 1:
        raise ValueError("need at least one interval")
    return np.linspace(0.0, float(u_max_m), n_intervals + 1)


def ramp_basis(knots_m: np.ndarray, u_m: np.ndarray | float) -> np.ndarray:
    """Basis of the piecewise-linear laws on ``knots_m``, as ``(len(u), n_intervals)``.

    Column ``j`` is the *integral of the indicator of interval j* — a ramp that is flat at 0
    below ``k_j``, rises with unit slope across the interval, and is flat at the interval
    width above ``k_{j+1}``. A law is then ``f(u) = Σ_j s_j · w_j(u)`` with ``s_j`` the slope
    on interval ``j``, and that is the property the whole thing is built on: **``f`` is linear
    in the slopes**, so the uncoupled ring response is too, so the fit stays a single solve
    and monotonicity is exactly ``s ≥ 0``. Fitting the knot *values* instead would make
    monotonicity an ordering constraint between parameters, which is the same feasible set
    described in a way no ordinary solver takes.

    Every column is **bounded**, so this basis is flat beyond the last knot and describes the
    law only on ``[0, knots[-1]]``. Extrapolation is deliberately not part of the basis:
    :class:`TabulatedLaw` continues past the table at ``max(last slope, 0)``, and that clamp
    is not a linear function of the slopes. Keeping it out here is what lets the fit treat
    this matrix as an exact Jacobian over the range where the data actually lives.

    The **first** column is the exception at the other end: it carries ``min(u, 0)``,
    continuing the first slope into extension. Same reasoning as :class:`SpringLaw`'s linear
    tension branch, and the same necessity — without it a segment pulled outward by the band
    contributes no diagonal stiffness and :func:`solve_equilibrium`'s Jacobian is singular.
    """
    knots = np.asarray(knots_m, dtype=np.float64)
    u = np.atleast_1d(np.asarray(u_m, dtype=np.float64)).ravel()
    lower, upper = knots[:-1], knots[1:]
    columns = np.clip(u[:, None] - lower[None, :], 0.0, (upper - lower)[None, :])
    columns[:, 0] += np.minimum(u, 0.0)
    return columns


@dataclass(frozen=True, slots=True)
class TabulatedLaw:
    """A piecewise-linear radial force law, stored as knots and interval slopes.

    Why this exists: a cubic **cannot fit the nominal design**. Measured ``F(δ)`` on the
    plane-strain tier has a tangent running 42.8 → −7.0 → 17.3 N/mm across 0–12 mm, and
    ``a·u + b·u² + c·u³`` has one inflection to spend, so it follows the rise or the collapse
    and not both: 8.7–14.4% RMS against a 5% threshold. That collapse is not an artefact to be
    fitted around — **it is the compliance this project exists to study** — so the law has to
    be able to represent it rather than the design being excluded for having it.

    **What the law may *not* do is pull.** That is the constraint, and it is a weaker one than
    it first looks: the obvious guard is to require the force to be non-decreasing, which is
    what this class enforced when it was written, and it is *wrong for this project*. A
    buckling spoke genuinely softens — a negative tangent is what buckling is — and forbidding
    it excludes exactly the designs the search is meant to find. Measured on the nominal
    design at 24 segments and 12 intervals: a monotone table reaches **12.87%** RMS, a table
    constrained only to non-negative force reaches **2.35%**. Monotonicity, not the cubic, was
    the binding constraint; the cubic was only the first thing to hit it.

    So validation is ``f(knot) >= 0`` at every knot, which for a piecewise-linear function is
    exactly ``f >= 0`` on the whole table. Compressing a segment can never make it pull the
    ground. Whether the law is additionally monotone is *reported*
    (:attr:`is_monotone_nonneg`) rather than required, because a softening segment is a
    physical result worth seeing and a numerical hazard worth naming.

    Attributes:
        knots_m: ``n+1`` ascending compressions, metres, with ``knots_m[0] == 0``. Zero is
            not a convention — the law must pass through the origin, because a segment at its
            undeformed radius carries no load.
        slopes_n_per_m: ``n`` tangent stiffnesses, one per interval. May be negative;
            the accumulated force may not.
    """

    knots_m: np.ndarray
    slopes_n_per_m: np.ndarray

    def __post_init__(self) -> None:
        knots = np.asarray(self.knots_m, dtype=np.float64).ravel()
        slopes = np.asarray(self.slopes_n_per_m, dtype=np.float64).ravel()
        if knots.size < 2:
            raise ValueError("a table needs at least two knots")
        if slopes.size != knots.size - 1:
            raise ValueError(
                f"{slopes.size} slopes for {knots.size} knots; want one per interval"
            )
        if knots[0] != 0.0:
            raise ValueError("the first knot must be 0: f(0) = 0 is not optional")
        if np.any(np.diff(knots) <= 0.0):
            raise ValueError("knots must be strictly ascending")
        # Tolerance rather than a bare >= 0: the knot forces come out of a least-squares
        # solve whose constraint was exactly this, so the binding ones land at zero give or
        # take rounding, and rejecting a -1e-16 would fail the fits that obeyed the rule.
        forces = np.concatenate([[0.0], np.cumsum(slopes * np.diff(knots))])
        floor = -1e-9 * max(float(np.max(np.abs(forces))), 1.0)
        if np.any(forces < floor):
            raise ValueError(
                "a compressed segment cannot pull: the table goes negative at "
                f"{knots[int(np.argmin(forces))] * 1e3:.2f} mm"
            )
        # Frozen means the *fields* cannot be rebound, which does nothing for an array whose
        # contents are mutable. Make them genuinely immutable so a shared law cannot be
        # edited through one holder's reference.
        knots.setflags(write=False)
        slopes.setflags(write=False)
        object.__setattr__(self, "knots_m", knots)
        object.__setattr__(self, "slopes_n_per_m", slopes)

    @property
    def extrapolation_slope_n_per_m(self) -> float:
        """Tangent used beyond the last knot: the last interval's, but never negative.

        Two bad extrapolations, and this picks the less bad one. Continuing a *negative* final
        slope drives the force to zero and then through it, so a wheel pressed past the range
        the FEA covered would be pulled into the obstacle — a law that produces its own
        runaway. Clamping to flat understates the force instead, which is wrong in the safe
        direction and is why :class:`wheelopt.sim.step_climb.StepResult` reports the fraction
        of a run spent beyond the fitted range rather than the law pretending to know.
        """
        return max(float(self.slopes_n_per_m[-1]), 0.0)

    @classmethod
    def from_forces(cls, knots_m: np.ndarray, forces_n: np.ndarray) -> TabulatedLaw:
        """Build from forces *at* the knots. ``forces_n[0]`` must be 0 and the rest ascending."""
        knots = np.asarray(knots_m, dtype=np.float64).ravel()
        forces = np.asarray(forces_n, dtype=np.float64).ravel()
        if forces.size != knots.size:
            raise ValueError("need one force per knot")
        if forces.size and forces[0] != 0.0:
            raise ValueError("f(0) must be 0")
        return cls(knots_m=knots, slopes_n_per_m=np.diff(forces) / np.diff(knots))

    @property
    def forces_n(self) -> np.ndarray:
        """Force at each knot, newtons — the table as a person would read it."""
        widths = np.diff(self.knots_m)
        return np.concatenate([[0.0], np.cumsum(self.slopes_n_per_m * widths)])

    @property
    def n_intervals(self) -> int:
        return int(self.slopes_n_per_m.size)

    def force_n(self, u_m: np.ndarray | float) -> np.ndarray:
        u = np.atleast_1d(np.asarray(u_m, dtype=np.float64)).ravel()
        beyond = np.maximum(u - self.knots_m[-1], 0.0)
        out = (ramp_basis(self.knots_m, u) @ self.slopes_n_per_m
               + self.extrapolation_slope_n_per_m * beyond)
        return out if np.ndim(u_m) else out[0]

    def stiffness_n_per_m(self, u_m: np.ndarray | float) -> np.ndarray:
        """Tangent stiffness, N/m — **piecewise constant, and discontinuous at the knots.**

        That discontinuity is real and has one consequence worth naming: the Newton loop in
        :func:`solve_equilibrium` becomes semismooth, and can in principle chatter across a
        knot instead of settling. It does not silently produce a wrong answer if it does —
        the loop reports ``converged=False`` and :class:`RingFit` propagates it. Only the
        *coupled* solve runs Newton at all; the bandless topology every current design uses
        reads its compressions straight off the geometry and never touches this path.
        """
        u = np.atleast_1d(np.asarray(u_m, dtype=np.float64)).ravel()
        index = np.clip(np.searchsorted(self.knots_m, u, side="right") - 1,
                        0, self.n_intervals - 1)
        out = np.where(u > self.knots_m[-1],
                       self.extrapolation_slope_n_per_m,
                       self.slopes_n_per_m[index])
        return out if np.ndim(u_m) else out[0]

    @property
    def is_valid_spring(self) -> bool:
        """Always True: a constructed table has already been checked not to pull."""
        return True

    @property
    def is_monotone_nonneg(self) -> bool:
        """Whether the law also never softens. **Reported, not required.**

        False is a finding, not a fault: it says this segment has a limit point, which is what
        a buckling spoke has. Two things follow that a caller may care about. The equilibrium
        Jacobian ``K + diag(df/du)`` can lose positive definiteness, so
        :func:`solve_equilibrium` leans on :func:`_solve_spd`'s least-squares fallback. And in
        MuJoCo the segment will snap through dynamically rather than settle — which is the
        real behaviour, but it needs the damping term to be present and the timestep small.
        """
        return bool(np.all(self.slopes_n_per_m >= 0.0))

    def summary(self) -> str:  # pragma: no cover - display only
        stiffness = self.slopes_n_per_m * 1e-3
        tail = "" if self.is_monotone_nonneg else "  <- softens"
        return (f"table, {self.n_intervals} intervals to "
                f"{self.knots_m[-1] * 1e3:.1f} mm, tangent "
                + " -> ".join(f"{k:.2f}" for k in stiffness) + f" N/mm{tail}")


def segment_angles(spec: RingSpec, phase_rad: float = 0.0) -> np.ndarray:
    """Angle of each segment from the contact point, radians, in ``[-π, π)``.

    With ``phase_rad = 0`` — the default everywhere except the harshness metric — segment 0
    sits at the contact point, the ring is symmetric about it, and the flat-plate response is
    a function of ``δ`` alone.

    ``phase_rad`` rotates the whole ring under a stationary contact point. For a **banded**
    wheel that is a rotation of a nearly circular thing and changes almost nothing; for a
    **bandless** one it is the difference between standing on a tip and standing between two,
    which is the entire polygon effect. Half a segment pitch is the worst case. See
    :func:`ride_height_ripple_m`.
    """
    i = np.arange(spec.n_segments)
    theta = (2.0 * np.pi * i / spec.n_segments + phase_rad) % (2.0 * np.pi)
    return np.where(theta >= np.pi, theta - 2.0 * np.pi, theta)


def penetrations(spec: RingSpec, delta_m: float,
                 phase_rad: float = 0.0) -> np.ndarray:
    """Radial compression of each segment at hub indentation ``δ``, metres.

    A segment slides along **its own radius**, so its tip sits at height
    ``y = (R - δ) - (R - u) cos θ`` and touches the plate when ``y = 0``:

        u = R - (R - δ) / cos θ,    for cos θ > 0

    Not ``δ - R(1 - cos θ)``, which is that expression's small-angle limit and what this
    function used to return. The two agree at the contact point and diverge away from it —
    at δ = 6 mm on a 60 mm wheel the approximation is 3.4% low at 15° and 7.6% low at 22.5°,
    always *under*-estimating, so the whole ring came out softer than its own spring law
    implied. That is the right sign and roughly the right size to explain the gap against
    MuJoCo, which does the exact geometry.

    Segments with ``cos θ <= 0`` face away from the plate and can never reach it. They need
    an explicit gate: the exact expression divides by ``cos θ``, so at 105° it returns a
    cheerful 268 mm of "penetration" — the distance to the plate's *extension* behind the
    hub. The small-angle form went negative there and was clipped away by accident.
    """
    theta = segment_angles(spec)
    cos_theta = np.cos(theta)
    facing = cos_theta > 0.0
    u = np.zeros_like(cos_theta)
    u[facing] = spec.radius_m - (spec.radius_m - delta_m) / cos_theta[facing]
    return np.where(u > 0.0, u, 0.0)


def bending_coupling_n_per_m(
    *,
    youngs_pa: float,
    band_width_m: float,
    band_thickness_m: float,
    radius_m: float,
    n_segments: int,
) -> float:
    """Neighbour coupling stiffness from the shear band's bending stiffness, N/m.

    An inextensional ring of bending stiffness ``EI`` deflecting radially by ``w(θ)`` stores

        U = EI / (2R³) ∫ (w'' + w)² dθ

    — the classical flexible-ring energy, and the reason :func:`curvature_operator` is the
    discrete form of ``w'' + w`` rather than a plain second difference. Discretising the
    integral at ``N`` equally spaced segments with ``dθ = 2π/N`` gives
    ``U = (k/2) Σ (w'' + w)_i²`` with

        k = EI · dθ / R³,      I = width · thickness³ / 12

    Both ends of that are honest about their assumptions. **Inextensionality** is the standard
    flexible-ring assumption and the reason no circumferential membrane term appears; it is
    good for a thin band and gets worse as the band thickens. **``I`` for a flat strip** treats
    the band as a rectangular section, ignoring the spoke roots that thicken it locally. And
    ``R`` here is the band's *mid-surface* radius, not the outer radius: they differ by
    ``t/2``, which is 2.5% on the tiny design and 7.6% once cubed, so the distinction is worth
    the argument name being explicit about which one it wants.

    Args:
        youngs_pa: modulus of the printed band — pass the knocked-down value from
            ``fea.hyperelastic.for_material(...).initial_youngs_pa``, not the solid-TPU one,
            so that the band the ring bends is the band the FEA compressed.
        band_thickness_m: the shear band's radial thickness, ``rim_thickness_mm`` in SI.
            **Zero is meaningful**: the bandless topology, coupling exactly zero.
        radius_m: mid-surface radius of the band, ``R_outer - t/2``.

    Returns:
        Coupling stiffness in N/m, ready for :attr:`RingSpec.coupling_n_per_m`.
    """
    if min(youngs_pa, band_width_m, radius_m) <= 0:
        raise ValueError("modulus, band width and radius must be positive")
    if band_thickness_m < 0:
        raise ValueError("band_thickness_m must be non-negative")
    if n_segments < 3:
        raise ValueError("a ring needs at least three segments")
    second_moment_m4 = band_width_m * band_thickness_m**3 / 12.0
    d_theta = 2.0 * np.pi / n_segments
    return float(youngs_pa * second_moment_m4 * d_theta / radius_m**3)


def hoop_coupling_n_per_m(
    *,
    youngs_pa: float,
    band_width_m: float,
    band_thickness_m: float,
    radius_m: float,
    n_segments: int,
) -> float:
    """The band's resistance to changing its own circumference, N/m.

    Bending alone is not the shear band. Inextensionality — the assumption that makes
    :func:`bending_coupling_n_per_m` the whole story for every ``n >= 1`` mode — also
    *forbids* the ``n = 0`` mode, uniform inflation, because a periodic tangential
    displacement cannot accommodate a change of circumference. Model bending and stop there
    and the ring is free to breathe against nothing but its own bending stiffness, which is
    the wrong stiffness by a factor of ``12(R/t)²`` — about 4600 on the tiny design.

    It is not a subtle error. Squeezing a bare ring between two opposite radial point loads
    should deflect ``0.1488 F R³ / EI`` (Roark, and the modal sum reproduces it). With
    bending only, this ring deflects **5.28×** that, and the excess is entirely the ``n = 0``
    mode: ``2/π = 0.6366`` against the ``0.1488`` of every other mode combined. The check is
    in ``tests/test_rom.py`` and it is the reason this function exists.

    The rest is exact rather than approximate, which is worth saying because a radial-only
    ring usually *cannot* carry a membrane term without locking. The circumference change is
    ``∮ w dθ = 2π w̄``: it depends only on the **mean** radial displacement, and the
    tangential displacement drops out of it because it is periodic. So the hoop energy
    ``π E A w̄² / R`` attaches to the mean alone, leaves every ``n >= 1`` mode untouched, and
    there is nothing for the missing tangential freedom to lock against.

    Returns:
        ``k_h`` in N/m for an energy ``(k_h/2)(Σ u)²`` — a single all-to-all term, not a
        neighbour one, because a hoop is not a chain of local springs.
    """
    if min(youngs_pa, band_width_m, radius_m) <= 0:
        raise ValueError("modulus, band width and radius must be positive")
    if band_thickness_m < 0:
        raise ValueError("band_thickness_m must be non-negative")
    if n_segments < 3:
        raise ValueError("a ring needs at least three segments")
    area_m2 = band_width_m * band_thickness_m
    return float(2.0 * np.pi * youngs_pa * area_m2 / (radius_m * n_segments**2))


def ring_for_design(
    params: WheelParams,
    material: MaterialSpec,
    n_segments: int = 24,
) -> RingSpec:
    """The ring a given wheel design and material imply — radius and coupling both derived.

    Invariant 2 in the form it takes for the ROM: the band stiffness is a *consequence* of
    ``rim_thickness_mm``, ``width_mm`` and the printed material, so a sweep over band
    thickness moves it and no caller can hold it constant by accident. The modulus is the
    knocked-down one from :mod:`wheelopt.fea.hyperelastic`, the same value the FEA deck was
    written with, so the ring bends the band the solver compressed.

    The **bandless** topology (``rim_thickness_mm == 0``) returns coupling exactly zero, and
    short-circuits before asking for a hyperelastic model of a feature with no thickness.
    That is the topology switch, not the bottom of the ``t_rim`` range.
    """
    from ..fea.hyperelastic import for_material

    radius_m = params.outer_radius_mm * 1e-3
    # The claw root, for the hinge model. Derived here rather than defaulted, so that a ring
    # built from a design always knows where its segments are attached and only a hand-built
    # RingSpec can be missing it.
    root_radius_m = params.hub_radius_mm * 1e-3
    if params.rim_thickness_mm <= 0.0:
        return RingSpec(radius_m=radius_m, n_segments=n_segments,
                        root_radius_m=root_radius_m)

    band = for_material(material, feature_thickness_mm=params.rim_thickness_mm)
    geometry = {
        "youngs_pa": band.initial_youngs_pa,
        "band_width_m": params.width_mm * 1e-3,
        "band_thickness_m": params.rim_thickness_mm * 1e-3,
        # Mid-surface of the band, not the outer radius: the bending energy goes as 1/R^3.
        "radius_m": (params.outer_radius_mm - 0.5 * params.rim_thickness_mm) * 1e-3,
        "n_segments": n_segments,
    }
    return RingSpec(
        radius_m=radius_m,
        n_segments=n_segments,
        band_bending_n_per_m=bending_coupling_n_per_m(**geometry),
        band_hoop_n_per_m=hoop_coupling_n_per_m(**geometry),
        root_radius_m=root_radius_m,
    )


def curvature_operator(spec: RingSpec) -> np.ndarray:
    """The discrete ``w'' + w``: an ``N×N`` circulant mapping compressions to curvature change.

    Row ``i`` is ``α(u_{i+1} - 2u_i + u_{i-1}) + u_i``, wrapping at the ends because the ring
    closes — that wrap is the equality constraint §3 of the plan asks for, and it is free in a
    circulant rather than something to enforce.

    ``α`` is **not** ``1/dθ²``, which is what a naive discretisation of the second derivative
    gives. It is ``1 / (2(1 - cos dθ))``, the value that makes the operator annihilate
    ``u_i = cos θ_i`` and ``u_i = sin θ_i`` *exactly at this N* rather than only in the limit.
    Those two modes are the band translating rigidly sideways and vertically, which must cost
    no bending energy at any discretisation; with ``1/dθ²`` they cost a spurious 0.57% of the
    stiffness at ``N = 24``, a small wrong number of exactly the kind this project keeps
    finding. The two agree to O(dθ²) — ``α → 1/dθ²`` as ``N → ∞``.

    Uniform inflation ``u_i = const`` is *not* annihilated, and should not be: growing a ring
    changes its curvature.
    """
    n = spec.n_segments
    d_theta = 2.0 * np.pi / n
    alpha = 1.0 / (2.0 * (1.0 - np.cos(d_theta)))
    operator = np.zeros((n, n), dtype=np.float64)
    index = np.arange(n)
    operator[index, index] = 1.0 - 2.0 * alpha
    operator[index, (index + 1) % n] += alpha
    operator[index, (index - 1) % n] += alpha
    return operator


def coupling_matrix(spec: RingSpec) -> np.ndarray:
    """The band's stiffness matrix, N/m, such that the force it applies is ``-K u``.

    Two terms, and they are different shapes. Bending is ``k_b·AᵀA`` — banded, local, three
    segments either side. Hoop is ``k_h·11ᵀ`` — dense, global, every segment against every
    other, because a change of circumference is not something a neighbour can resist alone.

    Symmetric positive semi-definite by construction. Rank is ``N - 2`` from bending alone
    (:func:`curvature_operator` annihilates the two rigid translations, and the hoop term does
    not restore them — a translated ring has the same circumference). So the band *alone*
    cannot determine the ring's shape, and the equilibrium solve leans on the radial springs,
    including in tension, to be well posed.
    """
    operator = curvature_operator(spec)
    ones = np.ones((spec.n_segments, spec.n_segments), dtype=np.float64)
    return (spec.band_bending_n_per_m * (operator.T @ operator)
            + spec.band_hoop_n_per_m * ones)


def _solve_spd(matrix: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    """Solve, falling back to least squares if the Newton Jacobian is singular.

    The Jacobian is ``K + diag(tangent stiffness)``, positive definite whenever the tangent
    is — which holds for any law a fit would *accept*, since :attr:`SpringLaw.is_monotone_nonneg`
    requires it. It does not hold for every law a fit *tries*: the coupled fit iterates, and
    an intermediate least-squares solution is free to come back with ``a <= 0``. That is a
    reachable branch, not a safety net for something that cannot happen.
    """
    try:
        return np.linalg.solve(matrix, rhs)
    except np.linalg.LinAlgError:
        return np.linalg.lstsq(matrix, rhs, rcond=None)[0]


def vertical_reaction_n(spec: RingSpec, contact_force_n: np.ndarray,
                        phase_rad: float = 0.0) -> float:
    """Total vertical load on the plate, newtons, given each segment's *radial* force.

    ``Σ f_r / cos θ``. The division is the whole of ADR-free issue #26 and it was wrong here
    until 2026-08-09, so the reasoning is worth keeping next to the arithmetic.

    ``f_r`` is the force conjugate to the segment's radial coordinate ``u`` — what the spring
    law returns, and what the coupled solve's constraint multiplier is. The *contact* force is
    a different vector. A frictionless plate can only push along its own normal, so it applies
    ``λ ĵ`` with no horizontal part; virtual work against the slide joint's motion
    ``∂p/∂u = (-sin θ, cos θ)`` gives the generalised force ``λ cos θ``, and equilibrium of
    the segment is ``f_r(u) = λ cos θ``. Hence ``λ = f_r / cos θ``, and the plate carries
    ``Σ λ``. The horizontal ``λ tan θ`` needed to hold the segment on its radius is supplied
    by the slide joint, which is internal to the wheel and cancels in the sum.

    The old ``Σ f_r · cos θ`` is the answer to a different question: it is the vertical part
    of a contact force pointing **along the segment's own axis**, i.e. the segment treated as
    a two-force strut. That needs the plate to supply ``f_r sin θ`` horizontally, which a
    frictionless plate has no way to do. The two differ by ``cos²θ`` per segment: 6.7% one
    segment off-centre on a 24-ring, 25% two off, and it grows with the patch.

    **Measured, not argued.** MuJoCo's ring (``condim="1"``, so genuinely frictionless — the
    measured horizontal contact force is exactly 0.0 N) settles at its own ``u_i`` and reports
    its own ``λ_i``. Over δ = 2–20 mm on a 24-segment R 85 mm ring with a linear law,
    ``f_r(u_i)/cos θ_i`` reproduces every measured ``λ_i`` to **6.2e-11** relative, and
    ``f_r(u_i)·cos θ_i`` is off by up to **25%** — exactly ``1 - cos²30°``. See the 2026-08-09
    log entry.

    Segments facing away from the plate are excluded rather than divided: ``cos θ`` reaches
    exactly zero whenever ``n_segments`` is a multiple of four, and there the quotient is
    ``0/0``. They carry no contact force in any case.
    """
    cos_theta = np.cos(segment_angles(spec, phase_rad))
    facing = cos_theta > 0.0
    return float(np.sum(contact_force_n[facing] / cos_theta[facing]))


@dataclass(frozen=True, slots=True)
class RingState:
    """The solved state of a ring at one indentation."""

    #: Radial compression of every segment, metres. Negative where the band pulled a segment
    #: outward past the undeformed radius.
    compression_m: np.ndarray
    #: Force the plate applies to each segment along its radius, newtons. Zero off the patch,
    #: non-negative on it — a plate pushes, and the active-set loop exists to guarantee that.
    contact_force_n: np.ndarray
    #: Which segments the plate touches.
    in_contact: np.ndarray
    #: Vertical reaction on the plate, newtons: ``Σ contact_force / cos θ``. Divided — see
    #: :func:`vertical_reaction_n`; :attr:`contact_force_n` is radial, the reaction is not.
    force_n: float
    #: Active-set iterations used. One means the geometric guess was already right.
    iterations: int
    #: False if either loop hit its cap. The state is still the best one reached — an
    #: evaluation returns rather than raises (invariant 4) — but a fit built on it is not
    #: trustworthy, so :mod:`wheelopt.rom.fit` propagates this rather than averaging it away.
    converged: bool = True


def solve_equilibrium(
    spec: RingSpec,
    law: RadialLaw,
    delta_m: float,
    *,
    max_active_set_iters: int = 64,
    newton_iters: int = 40,
    tol_m: float = 1e-14,
    phase_rad: float = 0.0,
) -> RingState:
    """Equilibrium of the ring pressed onto a flat plate by ``δ``, with coupling.

    Without coupling every compression is read straight off the geometry. With it, that stops
    being true, and the difference is the whole point of the band: a segment the plate does
    not touch is dragged inward by the segments it does touch, and pushes back on them
    through the band. So this is a constrained minimisation of

        Π(u) = Σ U_spring(u_i) + ½ uᵀ K u      subject to     u_i ≥ g_i on reachable segments

    where ``g_i`` is :func:`penetrations`' interference — a segment may be compressed *more*
    than geometry demands, never less, because less would put it inside the plate. The
    multiplier on the active constraints is the contact force, and it must come out
    non-negative: a plate cannot pull a segment down to hold it in the patch.

    Solved by an active set, seeded with the geometric contact set and corrected until both
    conditions hold. Each iteration is a Newton solve on the free block — nonlinear because
    the spring law is cubic, well conditioned because the block is
    ``diag(tangent stiffness) + K`` with the tangent strictly positive.

    Two failure modes are handled rather than asserted away. **Non-convergence** of either
    loop returns the best state reached with the iteration count set to the cap, because a
    ring is evaluated thousands of times inside a fit and invariant 4 says an evaluation
    returns rather than raises. **Cycling** of the active set is prevented by only ever
    releasing the single most-negative contact and only ever adding the single deepest
    violation, so each step strictly improves one measure.

    ``phase_rad`` rotates the ring under the contact point (:func:`segment_angles`). It is a
    bandless quantity — the band operator is a circulant on a fixed segment grid — and
    :func:`ring_force_n` refuses it on a coupled spec for that reason.
    """
    theta = segment_angles(spec, phase_rad)
    cos_theta = np.cos(theta)
    reachable = cos_theta > 0.0
    gap = np.full(spec.n_segments, -np.inf, dtype=np.float64)
    gap[reachable] = spec.radius_m - (spec.radius_m - delta_m) / cos_theta[reachable]

    if not spec.is_coupled:
        # Uncoupled: the constraint is active exactly where geometry says it is, every free
        # segment relaxes to u = 0, and the multiplier is the spring force. Identical to the
        # active-set answer below and cheap enough to matter — the fit calls this per delta
        # per basis function per iteration.
        compression = np.where(np.isfinite(gap) & (gap > 0.0), gap, 0.0)
        active = compression > 0.0
        contact = np.where(active, law.force_n(compression), 0.0)
        return RingState(
            compression_m=compression,
            contact_force_n=contact,
            in_contact=active,
            force_n=vertical_reaction_n(spec, contact, phase_rad),
            iterations=1,
        )

    stiffness = coupling_matrix(spec)
    active = np.isfinite(gap) & (gap > 0.0)
    compression = np.zeros(spec.n_segments, dtype=np.float64)
    iterations = 0
    converged = False

    for iterations in range(1, max_active_set_iters + 1):
        free = ~active
        compression[active] = gap[active]
        if np.any(free):
            block = stiffness[np.ix_(free, free)]
            coupled_in = stiffness[np.ix_(free, active)] @ compression[active]
            guess = compression[free]
            settled = False
            for _ in range(newton_iters):
                residual = law.force_n(guess) + block @ guess + coupled_in
                jacobian = block + np.diag(law.stiffness_n_per_m(guess))
                step = _solve_spd(jacobian, -residual)
                guess = guess + step
                if np.max(np.abs(step)) < tol_m:
                    settled = True
                    break
            compression[free] = guess
            if not settled:
                break

        multiplier = law.force_n(compression) + stiffness @ compression

        # A contact pulling downward is not a contact. Release the worst one and re-solve.
        held = np.where(active, multiplier, np.inf)
        if np.min(held) < 0.0:
            active[int(np.argmin(held))] = False
            continue
        # A free segment below the plate surface must join the patch. Add the deepest.
        depth = np.where(free & reachable, gap - compression, -np.inf)
        if np.max(depth) > tol_m:
            active[int(np.argmax(depth))] = True
            continue
        converged = True
        break

    contact = np.where(active, law.force_n(compression) + stiffness @ compression, 0.0)
    return RingState(
        compression_m=compression,
        contact_force_n=contact,
        in_contact=active,
        force_n=vertical_reaction_n(spec, contact, phase_rad),
        iterations=iterations,
        converged=converged,
    )


def ring_force_n(spec: RingSpec, law: RadialLaw, delta_m: np.ndarray | float,
                 phase_rad: float = 0.0) -> np.ndarray:
    """Vertical reaction of the ring against a flat plate, newtons.

    The plate's normal force on a segment is ``f_r / cos θ``, not ``f_r · cos θ``; see
    :func:`vertical_reaction_n`. Segments past ±90° face away from the plate and are excluded
    there, so they cannot contribute a negative term.

    ``phase_rad`` rotates the ring under the contact point. **Bandless only**: the coupled
    solve is written about a fixed segment grid, so a phase there would rotate the contact set
    without rotating the band, and it raises rather than quietly answering the wrong question.
    """
    if phase_rad and spec.is_coupled:
        raise ValueError(
            "phase_rad is a bandless-ring quantity: solve_equilibrium's band operator is "
            "written on a fixed segment grid, so rotating the contact point alone would "
            "shear the band for free"
        )
    deltas = np.atleast_1d(np.asarray(delta_m, dtype=np.float64))
    out = np.array([solve_equilibrium(spec, law, float(d), phase_rad=phase_rad).force_n
                    for d in deltas])
    # Scalar in, scalar out. The earlier version wrote `out if np.ndim(delta_m) else out`,
    # which is the same expression twice and so always returned a length-1 array; under
    # numpy 2 `float()` on that raises rather than quietly working.
    return out if np.ndim(delta_m) else out[0]


def polygon_drop_m(radius_m: float, n_segments: int) -> float:
    """Ride-height drop of a **rigid** ``n``-tip wheel over one segment pitch, metres.

    ``R(1 - cos(π/n))``. A bandless wheel runs on ``n`` discrete tips, so it is a regular
    polygon: the axle rides at ``R`` with a tip straight down and falls to ``R cos(π/n)``
    midway between two, ``n`` times a revolution.

    **Half the pitch, not the whole one.** The neighbouring quantity ``R(1 - cos(2π/n))`` is
    the *second-claw engagement* threshold — how deep the wheel must indent before the next
    tip reaches the ground plane — and the two get confused because they differ only in a
    factor of two inside a cosine. This one is about where the axle sits as the wheel turns;
    that one is about how many claws share a static load.

    The rigid limit, and therefore an upper bound on the ripple a compliant wheel of the same
    tip count shows: see :func:`ride_height_ripple_m` for what the compliance does to it.
    """
    if radius_m <= 0.0 or n_segments < 3:
        raise ValueError("need a positive radius and at least three tips")
    return float(radius_m * (1.0 - np.cos(np.pi / n_segments)))


def ride_height_ripple_m(
    spec: RingSpec, law: RadialLaw, load_n: float, *, samples: int = 13,
    max_delta_m: float | None = None,
) -> tuple[float, float, float]:
    """How far the axle rises and falls over one segment pitch at constant load, metres.

    The harshness metric ``docs/plan/TODO.md`` #19 needs, and the reason a bandless wheel
    cannot have arbitrarily few claws: with no band the running surface is a polygon, and a
    wheel that does not deflect by more than the polygon drop leaves the ground between tips.

    Measured rather than reasoned: rotate the ring under the contact point across half a pitch
    (symmetry makes the other half a mirror), solve ``F(δ, ψ) = load_n`` for ``δ`` at each
    phase, and report the axle height ``R - δ`` at each. Loaded compliance shrinks the ripple
    below :func:`polygon_drop_m`, and by how much is the answer.

    Returns:
        ``(ripple_m, delta_min_m, delta_max_m)`` — the peak-to-peak axle movement, and the
        indentations at the two extremes. ``ripple_m`` is ``inf`` if the ring cannot carry
        ``load_n`` at some phase within ``max_delta_m``, which is itself the answer: that wheel
        bottoms out once a pitch.

    Raises:
        ValueError: on a coupled spec, where a phase is not defined (see :func:`ring_force_n`).
    """
    if spec.is_coupled:
        raise ValueError("ride_height_ripple_m is a bandless quantity; this spec has a band")
    if load_n <= 0.0:
        raise ValueError("load_n must be positive")
    ceiling = max_delta_m if max_delta_m is not None else 0.95 * spec.radius_m
    pitch = 2.0 * np.pi / spec.n_segments
    deltas = []
    for phase in np.linspace(0.0, 0.5 * pitch, samples):
        if float(ring_force_n(spec, law, ceiling, phase_rad=float(phase))) < load_n:
            return float("inf"), float("nan"), float("nan")
        lo, hi = 0.0, ceiling
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if float(ring_force_n(spec, law, mid, phase_rad=float(phase))) < load_n:
                lo = mid
            else:
                hi = mid
        deltas.append(0.5 * (lo + hi))
    lo_d, hi_d = float(np.min(deltas)), float(np.max(deltas))
    return hi_d - lo_d, lo_d, hi_d


# --------------------------------------------------------------------------------------
# The tangential degree of freedom.


@dataclass(frozen=True, slots=True)
class SegmentState2D:
    """A two-degree-of-freedom ring solved at one indentation."""

    #: Radial compression of every segment, metres. Positive inward.
    compression_m: np.ndarray
    #: Tangential displacement of every segment, metres. Signed: a segment on the leading
    #: side splays one way, its mirror the other, and the ring's total is zero by symmetry.
    slip_m: np.ndarray
    #: Vertical force the plate applies to each segment, newtons. Non-negative.
    contact_force_n: np.ndarray
    in_contact: np.ndarray
    #: Total vertical reaction, newtons — the plain sum of the per-segment multipliers, which
    #: *are* the vertical forces here. The radial-only :class:`RingState` reaches the same
    #: quantity from its radial force by :func:`vertical_reaction_n`, so the two now agree in
    #: the rigid-tangential limit rather than differing by ``cos²θ``.
    force_n: float


def symmetric_force_n(law: RadialLaw, x_m: np.ndarray | float) -> np.ndarray:
    """A radial law applied symmetrically about zero: ``sign(x) · f(|x|)``.

    The tangential spring resists displacement in *either* direction with the same
    magnitude, which a :class:`RadialLaw` does not do on its own — it is written for a
    compression that is normally one-signed, with an asymmetric tension branch. A claw bends
    as readily backwards as forwards, so reusing the radial law directly would make one
    direction of splay stiffer than the other for no physical reason.
    """
    x = np.atleast_1d(np.asarray(x_m, dtype=np.float64)).ravel()
    out = np.sign(x) * np.asarray(law.force_n(np.abs(x)), dtype=np.float64)
    return out if np.ndim(x_m) else out[0]


def solve_equilibrium_2dof(
    spec: RingSpec,
    radial_law: RadialLaw,
    tangential_law: RadialLaw,
    delta_m: float,
    *,
    bisection_iters: int = 80,
) -> SegmentState2D:
    """Equilibrium of a **bandless** ring whose segments also move tangentially.

    Why this is separate from :func:`solve_equilibrium` rather than replacing it: without a
    band the segments are *independent*, so the two-freedom problem factorises into ``N``
    identical three-unknown problems instead of one ``2N`` system. That is both far cheaper
    and far more robust, and bandless is every design this project now builds
    (``docs/plan/04-design-space.md`` §Direction). The coupled solver keeps the radial-only
    model, which is the correct one for a banded wheel.

    The geometry. Segment ``i`` sits on a radius at angle ``θ_i`` from the contact point,
    with outward unit vector ``e_r`` and tangential ``e_t``. Let ``u`` be its radial
    compression and ``v`` its tangential displacement. Its height above the plate is

        y(u, v) = (R - δ) - (R - u) cos θ + v sin θ

    **Valid only while ``v`` is small against the claw length, and this is not a refinement
    to note later.** The tip here translates along a straight line, so its distance from the
    hub centre *grows* as ``√(R² + v²)``; a real claw hinges at its root, so its tip swings on
    an arc and the distance *shrinks*. Measured on the R 60 mm claw (root 20 mm, L 40 mm), the
    two disagree by +0.1% of R at 2 mm of splay, +2.1% at 10 mm, +8.4% at 20 mm and **+30% at
    36 mm** — and the sign is the dangerous one: a segment that moves outward as it splays
    presses harder into the ground, which splays it further. Under drive torque, where
    deflections reach a claw length, that feedback tears the rolling model apart.
    On the flat-plate sweeps this stays small: 1.5% of R at δ = 18 mm and 6.4% at 25 mm.
    See ``docs/plan/TODO.md`` #27 — the fix is a hinge at the root, not a slide at the tip.

    and it may not go below the plate. On a frictionless plate the contact force is purely
    **vertical**, so it drives both freedoms, weighted by how much each changes the height:
    ``∂y/∂u = cos θ`` and ``∂y/∂v = sin θ``. Stationarity of
    ``U_r(u) + U_t(v) - λ y`` then gives

        f_r(u) = λ cos θ,      f_t(v) = λ sin θ,      λ ≥ 0,  y ≥ 0,  λ y = 0

    Three equations, three unknowns, per segment. Because ``f_r`` and ``f_t`` are increasing,
    ``u`` and ``v`` increase with ``λ`` and ``y`` therefore *decreases* with ``λ`` — so the
    active case is a one-dimensional bisection on ``λ``, which cannot fail to bracket and
    needs no initial guess. That monotonicity is the whole reason this is not a Newton solve
    with the usual failure modes.

    **What it changes physically.** A segment away from the contact point is pushed up by a
    vertical force with a component along its own tangent, and the radial-only model has
    nowhere for that to go — it is reacted rigidly. Here the segment splays, so the wheel is
    softer and the contact patch spreads. On the nominal claw the two stiffnesses are
    24.81 N/mm radial against 0.1851 N/mm tangential, a factor of 134, so "softer" is not a
    small correction away from the contact point.

    Raises:
        ValueError: if the spec has a band. A banded ring's segments are not independent and
            this factorisation does not hold; use :func:`solve_equilibrium`.
    """
    if spec.is_coupled:
        raise ValueError(
            "solve_equilibrium_2dof factorises per segment, which needs independent "
            "segments; this spec has a band. Use solve_equilibrium for the radial-only "
            "coupled model"
        )
    theta = segment_angles(spec)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    n = spec.n_segments
    radius, hub = spec.radius_m, spec.radius_m - delta_m

    compression = np.zeros(n, dtype=np.float64)
    slip = np.zeros(n, dtype=np.float64)
    contact = np.zeros(n, dtype=np.float64)
    active = np.zeros(n, dtype=bool)

    def height(index: int, u: float, v: float) -> float:
        return hub - (radius - u) * cos_t[index] + v * sin_t[index]

    for i in range(n):
        # A segment facing away from the plate can never reach it, whatever it does
        # tangentially: its radius carries it further away and the tangential term is
        # bounded by the splay the spring permits. Excluded outright, as in the radial model.
        if cos_t[i] <= 0.0 or height(i, 0.0, 0.0) >= 0.0:
            continue

        # Bracket lambda. y is decreasing in lambda, so grow the upper bound until the
        # segment is at or above the plate, then bisect.
        lo, hi = 0.0, 1.0
        for _ in range(200):
            u_hi = _invert(radial_law, hi * cos_t[i])
            v_hi = _invert(tangential_law, hi * sin_t[i], symmetric=True)
            if height(i, u_hi, v_hi) >= 0.0:
                break
            hi *= 2.0
        for _ in range(bisection_iters):
            mid = 0.5 * (lo + hi)
            u_m = _invert(radial_law, mid * cos_t[i])
            v_m = _invert(tangential_law, mid * sin_t[i], symmetric=True)
            if height(i, u_m, v_m) < 0.0:
                lo = mid
            else:
                hi = mid
        lam = 0.5 * (lo + hi)
        compression[i] = _invert(radial_law, lam * cos_t[i])
        slip[i] = _invert(tangential_law, lam * sin_t[i], symmetric=True)
        contact[i] = lam
        active[i] = True

    return SegmentState2D(
        compression_m=compression,
        slip_m=slip,
        contact_force_n=contact,
        in_contact=active,
        force_n=float(np.sum(contact)),
    )


def _invert(law: RadialLaw, force_n: float, *, symmetric: bool = False,
            iters: int = 60) -> float:
    """Displacement at which ``law`` carries ``force_n``. Bisection, no derivative needed.

    Deliberately derivative-free: a :class:`TabulatedLaw`'s tangent is piecewise constant and
    can be **zero** over an interval, which is a legitimate law (a buckled segment carrying a
    constant load) and a division by zero for a Newton inverse. Bisection does not care, and
    on a flat interval it returns the low end of it — the smallest displacement consistent
    with the force, which is the right choice for a contact problem.
    """
    # Sign first, then the zero check. Written the other way round the early return fired on
    # every negative force and the symmetric branch was unreachable — so the segments on one
    # side of the contact point splayed and their mirrors did not, and the ring walked
    # sideways under a symmetric load. Caught by the antisymmetry test, not by inspection.
    sign = 1.0
    if force_n < 0.0:
        if not symmetric:
            return 0.0
        sign, force_n = -1.0, -force_n
    if force_n == 0.0:
        return 0.0
    lo, hi = 0.0, 1e-4
    for _ in range(200):
        if float(law.force_n(hi)) >= force_n:
            break
        hi *= 2.0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if float(law.force_n(mid)) < force_n:
            lo = mid
        else:
            hi = mid
    return sign * 0.5 * (lo + hi)


def ring_force_2dof_n(
    spec: RingSpec,
    radial_law: RadialLaw,
    tangential_law: RadialLaw,
    delta_m: np.ndarray | float,
) -> np.ndarray:
    """Vertical reaction of the two-freedom ring. Scalar in, scalar out."""
    deltas = np.atleast_1d(np.asarray(delta_m, dtype=np.float64))
    out = np.array([
        solve_equilibrium_2dof(spec, radial_law, tangential_law, float(d)).force_n
        for d in deltas
    ])
    return out if np.ndim(delta_m) else out[0]


# --------------------------------------------------------------------------------------
# The hinge: the same second freedom, with the kinematics a claw actually has.


@dataclass(frozen=True, slots=True)
class SegmentStateHinge:
    """A ring of hinged claws solved at one indentation."""

    #: Axial shortening of every claw, metres. Positive inward, as in the radial-only model.
    compression_m: np.ndarray
    #: Rotation of every claw about its own root, radians. Signed: positive carries the tip
    #: toward ``+e_t``, so a claw ahead of the contact point and its mirror behind it rotate
    #: opposite ways and the ring's total is zero by symmetry.
    rotation_rad: np.ndarray
    #: Vertical force the plate applies to each claw tip, newtons. Non-negative.
    contact_force_n: np.ndarray
    in_contact: np.ndarray
    #: Total vertical reaction, newtons — the plain sum of the multipliers, which *are* the
    #: vertical forces. Directly comparable with :attr:`SegmentState2D.force_n`.
    #:
    #: For where the tips ended up — the one quantity that distinguishes this model from
    #: :class:`SegmentState2D`, and the whole of ``TODO.md`` #27 — pass
    #: :attr:`compression_m` and :attr:`rotation_rad` to :func:`tip_radius_hinge_m`.
    force_n: float


def tip_radius_hinge_m(
    spec: RingSpec, compression_m: np.ndarray | float, rotation_rad: np.ndarray | float
) -> np.ndarray:
    """Distance of a hinged claw's tip from the hub centre, metres.

    ``√(R_root² + ℓ² + 2 R_root ℓ cos φ)`` with ``ℓ = L - u`` — the law of cosines on the
    triangle root/centre/tip. **Decreasing in ``|φ|``**, which is the point.

    Compare the two-slide segment of :func:`solve_equilibrium_2dof`, whose tip sits at
    ``√((R - u)² + v²)`` and therefore moves *outward* as it splays. On the R 60 mm claw
    (root 20 mm, ℓ 40 mm) the two disagree by +2.1% of ``R`` at 10 mm of tip travel, +8.4% at
    20 mm and +30.1% at 36 mm, and the sign is the destabilising one: a segment that grows
    longer as it splays presses harder into the ground, which splays it further.
    """
    ell = spec.claw_length_m - np.asarray(compression_m, dtype=np.float64)
    root = spec.root_radius_m
    cos_phi = np.cos(np.asarray(rotation_rad, dtype=np.float64))
    return np.sqrt(np.maximum(root * root + ell * ell + 2.0 * root * ell * cos_phi, 0.0))


def tip_radius_slide_m(
    spec: RingSpec, compression_m: np.ndarray | float, slip_m: np.ndarray | float
) -> np.ndarray:
    """The same quantity for the two-slide segment: ``√((R - u)² + v²)``. **Increasing in
    ``|v|``.** Here so the two kinematics can be compared in one expression rather than in
    prose; see :func:`tip_radius_hinge_m`."""
    radial = spec.radius_m - np.asarray(compression_m, dtype=np.float64)
    slip = np.asarray(slip_m, dtype=np.float64)
    return np.sqrt(radial * radial + slip * slip)


@dataclass(frozen=True, slots=True)
class TipEquivalentLaw:
    """A hinge's moment-rotation law, restated as the force-deflection law of its own tip.

    ``M(φ)`` at arm ``a`` is ``F(s) = M(s/a)/a`` for small rotations, and the tangent scales by
    ``a²``. That is a change of coordinates, not a model: it exists so that every rule already
    written for a **linear** segment law — the explicit-integration timestep bound, the
    hysteretic damping equivalence — can be applied to a hinge without being rewritten in
    angular form and without a second chance to get a factor of ``a`` wrong. Convert the
    damping the rule returns back with :func:`~wheelopt.rom.mjcf.tangential_damping`.

    Deliberately **not** what drives the joint: MuJoCo needs the real moment, and
    ``solve_equilibrium_hinge`` needs the real moment, so both take ``hinge_law`` itself.
    """

    hinge_law: RadialLaw
    arm_m: float

    def __post_init__(self) -> None:
        if self.arm_m <= 0.0:
            raise ValueError(f"arm_m must be positive; got {self.arm_m}")

    def force_n(self, u_m: np.ndarray | float) -> np.ndarray:
        return np.asarray(
            self.hinge_law.force_n(np.asarray(u_m, dtype=np.float64) / self.arm_m)
        ) / self.arm_m

    def stiffness_n_per_m(self, u_m: np.ndarray | float) -> np.ndarray:
        return np.asarray(
            self.hinge_law.stiffness_n_per_m(np.asarray(u_m, dtype=np.float64) / self.arm_m)
        ) / (self.arm_m * self.arm_m)

    @property
    def is_valid_spring(self) -> bool:
        return self.hinge_law.is_valid_spring

    @property
    def is_monotone_nonneg(self) -> bool:
        return self.hinge_law.is_monotone_nonneg


def solve_equilibrium_hinge(
    spec: RingSpec,
    radial_law: RadialLaw,
    hinge_law: RadialLaw,
    delta_m: float,
    *,
    bisection_iters: int = 60,
) -> SegmentStateHinge:
    """Equilibrium of a **bandless** ring of claws hinged at the hub, on a flat plate.

    The replacement for :func:`solve_equilibrium_2dof` demanded by ``docs/plan/TODO.md`` #27.
    Same physics, same contact law, same per-segment factorisation — a different *element*.
    There, the second freedom is a slide at the tip; here it is a rotation at the root, and a
    claw is a cantilever off a hub, so the rotation is the honest one. What changes is not the
    stiffness but the kinematics: a hinged tip swings on an arc of fixed radius and therefore
    comes *inward* as it rotates, where a sliding one goes outward.
    :func:`tip_radius_hinge_m` carries the measurement.

    The geometry. Claw ``i`` is rooted at radius ``R_root`` on the ray at angle ``θ_i`` from
    the contact point, and reaches its tip a further ``ℓ = L - u`` along that ray, where
    ``L = R - R_root`` and ``u`` is the claw's axial shortening. Rotating the claw about its
    root by ``φ`` swings the tip through the same angle, so the tip's downward extent is

        d(u, φ) = R_root cos θ + (L - u) cos(θ + φ)

    — the two terms are the two sides of the triangle, and ``θ + φ`` appearing as a sum is the
    whole simplification: rotating the claw is indistinguishable from *moving the claw round
    the wheel*, as far as this segment's own contact is concerned. Height above the plate is
    ``y = (R - δ) - d``, and at ``φ = 0`` it collapses to ``(R - δ) - R cos θ``, the
    radial-only model, exactly.

    The forces. A frictionless plate pushes straight up with ``λ ≥ 0``, so stationarity of
    ``U_r(u) + U_φ(φ) - λ y`` against each freedom gives

        f_r(u) = λ cos(θ + φ),    M(φ) = λ (L - u) sin(θ + φ),    λ ≥ 0, y ≥ 0, λ y = 0

    Note the second: ``(L - u) sin(θ + φ)`` is the moment arm of a *vertical* force about the
    root, which is a lever arm that shortens as the claw folds — and that is the stabilising
    feedback the slide model lacked. Note also that ``M`` is a **moment**, N·m against a
    rotation in radians, so ``hinge_law`` is a :class:`RadialLaw` only in the duck-typed sense;
    :func:`~wheelopt.rom.fit.hinge_law_from_tip_curve` builds one from a measured tip curve and
    is the only intended source.

    Solving: **one bisection per claw**, on the contact angle ``ψ = θ + φ`` rather than on
    ``λ``. Written in ``λ`` this is two coupled equations needing a bisection inside a
    bisection, which costs about a thousand times more work and stacks two tolerances; in
    ``ψ`` it collapses. A claw in contact has ``y = 0``, so with ``c = (R - δ) - R_root cos θ``
    fixed by the geometry,

        L - u = c / cos ψ,     λ = f_r(u) / cos ψ,     M(ψ - θ) = f_r(u) · c · sin ψ / cos²ψ

    — the first is the contact condition solved for the claw's remaining length, the second is
    the radial stationarity (and is ``f_r/cos``, the #26 result, arriving a third way), and
    substituting both into the moment condition leaves one scalar equation in ``ψ``.

    It is bracketed by construction. The lower end is ``ψ = |θ|``, the unrotated claw, where
    the residual is ``-f_r c tan θ / cos θ ≤ 0``. The upper end is ``ψ_max = arccos(c/L)``,
    where ``u = 0`` — the claw has stood back up to its full length — and beyond which it would
    have to stretch; there ``f_r(0) = 0`` and the residual is ``M(ψ_max - θ) > 0``. A claw in
    contact always has ``|θ| < ψ_max``, so the sign change is guaranteed and no iteration can
    run away. Claws behind the contact point are the mirror image, solved as ``|θ|`` with the
    sign restored, which is why ``hinge_law`` is applied symmetrically.

    Raises:
        ValueError: if the spec has a band (the factorisation needs independent segments), or
            if ``root_radius_m`` is zero — a claw hinged at the wheel's centre sweeps its tip
            round a circle of radius ``R`` and can never indent, so the model would silently
            report a rigid wheel.
    """
    if spec.is_coupled:
        raise ValueError(
            "solve_equilibrium_hinge factorises per segment, which needs independent "
            "segments; this spec has a band. Use solve_equilibrium for the radial-only "
            "coupled model"
        )
    if spec.root_radius_m <= 0.0:
        raise ValueError(
            "a root hinge needs a root: RingSpec.root_radius_m is 0, which puts the pivot at "
            "the wheel's centre, where rotating a claw moves its tip along a circle of "
            "radius R and cannot indent at all. Build the spec with ring_for_design, which "
            "takes it from hub_radius_mm"
        )

    theta = segment_angles(spec)
    n = spec.n_segments
    length, root = spec.claw_length_m, spec.root_radius_m
    hub = spec.radius_m - delta_m

    compression = np.zeros(n, dtype=np.float64)
    rotation = np.zeros(n, dtype=np.float64)
    contact = np.zeros(n, dtype=np.float64)
    active = np.zeros(n, dtype=bool)

    for i in range(n):
        abs_theta = abs(float(theta[i]))
        sign = 1.0 if theta[i] >= 0.0 else -1.0
        cos_theta = np.cos(abs_theta)
        # A claw facing away from the plate can never reach it. Rotating only ever raises the
        # tip — the downward extent is R_root cos θ + (L - u) cos ψ and ψ only grows — so a
        # claw that misses the plate unrotated misses it however it moves. Excluded outright,
        # as in the other two models.
        if cos_theta <= 0.0 or hub >= spec.radius_m * cos_theta:
            continue

        c = hub - root * cos_theta

        def residual(psi: float, c: float = c, abs_theta: float = abs_theta) -> float:
            cos_psi = np.cos(psi)
            u = length - c / cos_psi
            radial = float(radial_law.force_n(u)) if u > 0.0 else 0.0
            return (float(hinge_law.force_n(psi - abs_theta))
                    - radial * c * np.sin(psi) / (cos_psi * cos_psi))

        lo = abs_theta
        hi = float(np.arccos(min(max(c / length, -1.0), 1.0)))
        if residual(lo) >= 0.0 or hi <= lo:
            # θ = 0, where the vertical force is along the claw and there is nothing to rotate
            # about. Not a fallback: it is the exact answer, and the branch exists because
            # bisecting an interval with no sign change would return its midpoint instead.
            psi = lo
        else:
            for _ in range(bisection_iters):
                mid = 0.5 * (lo + hi)
                if residual(mid) < 0.0:
                    lo = mid
                else:
                    hi = mid
            psi = 0.5 * (lo + hi)

        cos_psi = np.cos(psi)
        u = length - c / cos_psi
        compression[i] = u
        rotation[i] = sign * (psi - abs_theta)
        contact[i] = float(radial_law.force_n(u)) / cos_psi if u > 0.0 else 0.0
        active[i] = True

    return SegmentStateHinge(
        compression_m=compression,
        rotation_rad=rotation,
        contact_force_n=contact,
        in_contact=active,
        force_n=float(np.sum(contact)),
    )


def ring_force_hinge_n(
    spec: RingSpec,
    radial_law: RadialLaw,
    hinge_law: RadialLaw,
    delta_m: np.ndarray | float,
) -> np.ndarray:
    """Vertical reaction of the hinged-claw ring. Scalar in, scalar out."""
    deltas = np.atleast_1d(np.asarray(delta_m, dtype=np.float64))
    out = np.array([
        solve_equilibrium_hinge(spec, radial_law, hinge_law, float(d)).force_n
        for d in deltas
    ])
    return out if np.ndim(delta_m) else out[0]
