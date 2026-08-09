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
    "SpringLaw",
    "TabulatedLaw",
    "bending_coupling_n_per_m",
    "coupling_matrix",
    "curvature_operator",
    "hoop_coupling_n_per_m",
    "penetrations",
    "ramp_basis",
    "ring_for_design",
    "ring_force_2dof_n",
    "ring_force_n",
    "segment_angles",
    "solve_equilibrium",
    "solve_equilibrium_2dof",
    "symmetric_force_n",
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

    def __post_init__(self) -> None:
        if self.radius_m <= 0:
            raise ValueError("radius_m must be positive")
        if self.n_segments < 3:
            raise ValueError("a ring needs at least three segments")
        if min(self.band_bending_n_per_m, self.band_hoop_n_per_m) < 0:
            raise ValueError("band stiffnesses must be non-negative")

    @property
    def segment_arc_m(self) -> float:
        """Arc length one segment occupies on the undeformed ring."""
        return 2.0 * np.pi * self.radius_m / self.n_segments

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


def segment_angles(spec: RingSpec) -> np.ndarray:
    """Angle of each segment from the contact point, radians, in ``[-π, π)``.

    Segment 0 sits at the contact point. The ring is symmetric about it, which is what makes
    the flat-plate response a function of ``δ`` alone.
    """
    i = np.arange(spec.n_segments)
    theta = 2.0 * np.pi * i / spec.n_segments
    return np.where(theta >= np.pi, theta - 2.0 * np.pi, theta)


def penetrations(spec: RingSpec, delta_m: float) -> np.ndarray:
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
    if params.rim_thickness_mm <= 0.0:
        return RingSpec(radius_m=radius_m, n_segments=n_segments)

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


def vertical_reaction_n(spec: RingSpec, contact_force_n: np.ndarray) -> float:
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
    cos_theta = np.cos(segment_angles(spec))
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
    """
    theta = segment_angles(spec)
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
            force_n=vertical_reaction_n(spec, contact),
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
        force_n=vertical_reaction_n(spec, contact),
        iterations=iterations,
        converged=converged,
    )


def ring_force_n(spec: RingSpec, law: RadialLaw, delta_m: np.ndarray | float) -> np.ndarray:
    """Vertical reaction of the ring against a flat plate, newtons.

    The plate's normal force on a segment is ``f_r / cos θ``, not ``f_r · cos θ``; see
    :func:`vertical_reaction_n`. Segments past ±90° face away from the plate and are excluded
    there, so they cannot contribute a negative term.
    """
    deltas = np.atleast_1d(np.asarray(delta_m, dtype=np.float64))
    out = np.array([solve_equilibrium(spec, law, float(d)).force_n for d in deltas])
    # Scalar in, scalar out. The earlier version wrote `out if np.ndim(delta_m) else out`,
    # which is the same expression twice and so always returned a length-1 array; under
    # numpy 2 `float()` on that raises rather than quietly working.
    return out if np.ndim(delta_m) else out[0]


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
