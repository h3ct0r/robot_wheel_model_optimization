"""Fit a ring spring law to an FEA load curve, and report the error honestly.

``docs/plan/06-compliance-rom.md`` §4: *"Fit the ring model's parameters so its response
reproduces the FEA load cases. **Report the fit error** — this is a headline validity number
for the paper."* So the fit returns a :class:`RingFit` carrying the error, and there is no way
to get the coefficients without also getting it.

Why this is a fit and not a division
------------------------------------
At hub indentation ``δ`` the segments are compressed by a *spread* of amounts — the one at the
contact point by ``δ``, its neighbours by less, following ``δ - R(1 - cos θ)``. So each
measured ``F(δ)`` is a weighted sum over many points of the unknown ``f(u)``, and recovering
``f`` from ``F`` is a deconvolution.

Without neighbour coupling it is a *linear* deconvolution: the compressions are fixed by
geometry, so the ring response is linear in the spring coefficients even though it is
nonlinear in ``u``, and the whole thing is one least-squares solve with no iteration and no
initial guess. Coupling breaks that, because the compressions now depend on the coefficients
being fitted, and the contact set does too. The coupled fit is therefore a small nonlinear
least squares — damped Gauss-Newton, seeded from the uncoupled answer
(:func:`_levenberg_marquardt`). Still no scipy; the Jacobians are finite differences of a
model that costs a 48x48 Newton solve, and :func:`nnls` is thirty lines of Lawson-Hanson.

Two laws, and the choice is a real one
--------------------------------------
:func:`fit_spring_law` fits a cubic; :func:`fit_tabulated_law` fits a piecewise-linear table.
They are not "simple" and "flexible" versions of the same thing — they differ in what they can
*say*. The cubic cannot represent a segment that softens; the table can, and on the nominal
design that is the difference between a 14.25% fit and a 3.42% one, because the design's
tangent stiffness genuinely goes negative. See :class:`~wheelopt.rom.ring.TabulatedLaw`.

**And a warning that applies to both.** The deconvolution is ill-posed on a banded wheel, so
the fit error alone cannot tell you the law is right: on the nominal it falls monotonically as
the table gets finer, while the fitted tangent starts swinging between −52 and +63 N/mm.
Always look at the tangents, not only at :attr:`RingFit.rms_error_fraction`.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import numpy as np

from .ring import (
    RadialLaw,
    RingSpec,
    SpringLaw,
    TabulatedLaw,
    penetrations,
    ring_force_n,
    solve_equilibrium,
    uniform_knots,
)

if TYPE_CHECKING:  # Keeps this module importable, and testable, with nothing else installed.
    from ..cad.params import WheelParams

__all__ = [
    "FitFailure",
    "RingFit",
    "fit_spring_law",
    "fit_tabulated_law",
    "nnls",
    "ring_from_claw_curve",
]

#: One law per fitted coefficient, each with that coefficient set to 1 and the rest to 0.
#: Evaluating the real force law with these gives the design-matrix columns, so the fit can
#: never drift out of agreement with :mod:`wheelopt.rom.ring` about what the law *is*.
_BASIS_LAWS = (SpringLaw(a=1.0), SpringLaw(a=0.0, b=1.0), SpringLaw(a=0.0, c=1.0))

#: How far Levenberg damping may be raised before a step is abandoned. Ten decades: past that
#: the step is smaller than the finite-difference noise it was computed from.
_MAX_DAMPING_STEPS = 12


class FitFailure(ValueError):
    """The load curve cannot be fitted.

    Raised, not returned: unlike an FEA evaluation, this is not a property of the design
    being screened — it means the curve handed in was empty, degenerate, or not a loading
    branch, which is a programming error upstream. Invariant 4 governs evaluations; the
    runner catches this and types it like anything else.
    """


@dataclass(frozen=True, slots=True)
class RingFit:
    """A fitted spring law, with the error it achieved. Never one without the other."""

    spec: RingSpec
    law: RadialLaw
    #: Root-mean-square force error over the fitted points, newtons.
    rms_error_n: float
    #: RMS error as a fraction of the peak force in the data. The number to quote.
    rms_error_fraction: float
    #: Largest single-point force error, newtons.
    max_error_n: float
    #: The points fitted, for plotting and for re-checking without re-fitting.
    delta_m: np.ndarray
    force_n: np.ndarray
    fitted_force_n: np.ndarray
    #: Gauss-Newton iterations. Always 1 without coupling, where the fit is a single exact
    #: least-squares solve.
    iterations: int = 1
    #: False if the optimiser hit its cap, or if any equilibrium solve behind it did. The
    #: coefficients and the error are still the honest ones for the state reached.
    converged: bool = True

    @property
    def ok(self) -> bool:
        """Whether the fit is good enough to build a ring on.

        5% of peak force, a law that never pulls the ground, and a fit that converged.
        Note what is *not* here: monotonicity. A softening segment is what a buckling spoke
        does, and gating on it excluded the nominal design — see
        :class:`~wheelopt.rom.ring.TabulatedLaw`.
        The threshold is a judgement, not a derivation — it is tight enough that a ring
        reproducing the FEA to within it will not change the ranking of two designs that
        differ by more, and loose enough to survive the kink a buckling curve puts in the
        data. Convergence is not a judgement: an optimiser stopped mid-flight can land on a
        small error by accident, and that number means nothing.
        """
        return (self.rms_error_fraction <= 0.05
                and self.law.is_valid_spring
                and self.converged)

    def summary(self) -> str:  # pragma: no cover - display only
        tail = "" if self.converged else ", DID NOT CONVERGE"
        return (
            f"{self.spec.n_segments} segments, "
            f"RMS {self.rms_error_n:.3f} N ({self.rms_error_fraction:.2%} of peak), "
            f"max {self.max_error_n:.3f} N, {self.iterations} iteration(s){tail}"
        )


def _design_matrix(spec: RingSpec, deltas: np.ndarray) -> np.ndarray:
    """Ring response per unit of each spring coefficient: columns a, b, c.

    Valid only for an **uncoupled** spec, where ``ring_force_n`` is linear in (a, b, c).
    Evaluating it with each coefficient set to 1 and the others to 0 gives the columns
    directly. Building it this way rather than re-deriving the sums keeps one definition of
    the ring response — if `ring.py` changes, the fit follows automatically instead of
    drifting out of agreement with it.
    """
    cols = [ring_force_n(spec, basis, deltas) for basis in _BASIS_LAWS]
    return np.column_stack(cols)


def _levenberg_marquardt(
    residual_at: Callable[[np.ndarray], np.ndarray],
    x0: np.ndarray,
    max_iterations: int,
    tol: float,
    project: Callable[[np.ndarray], np.ndarray] | None = None,
) -> tuple[np.ndarray, int, bool]:
    """Damped Gauss-Newton on ``x``. Returns ``(x, iterations, converged)``.

    Generic over the parameter vector because there are now two of them: three cubic
    coefficients (:func:`fit_spring_law`) and ``n`` table slopes
    (:func:`fit_tabulated_law`). One optimiser, one set of damping rules, one place where the
    convergence criterion lives.

    ``project`` clamps a trial point onto the feasible set — used for the table, whose slopes
    must stay non-negative. It is applied to the *trial* before the cost is measured, so an
    infeasible Gauss-Newton step is evaluated at the point actually taken and accepted only if
    that point is genuinely better. Nothing enforces feasibility of ``x0``; the callers seed
    from a feasible point.

    The caller is responsible for scaling: this compares ``|Δx|`` against ``|x|``
    componentwise, which is meaningless if the components carry different units and differ by
    orders of magnitude. Cubic coefficients do (N/m against N/m³) and are rescaled by the
    caller; table slopes are all N/m and are not.

    Why not the obvious thing. Freezing the compressions makes the reaction linear in the
    coefficients again — ``Σ_contact [f_spring(u_i) + (K u)_i] cos θ_i`` with ``u`` held —
    so alternating a shape solve with a linear coefficient solve looks like it should work,
    and it is how this was first written. It diverges. Measured on the tiny design at 48
    segments, starting from the uncoupled fit: the first pass returns ``a = -9.6 N/m``, the
    fourth ``a = -132000``, and the residual runs to 350% of peak. The reason is that the
    band force on a contact segment is not a constant to be subtracted — move the
    coefficients and the shape moves with them, and at these stiffnesses it moves more than
    the linearisation predicts. The frozen-shape step is a fixed point of the right problem
    that happens not to be a contraction.

    Gauss-Newton with Levenberg damping is the boring answer and it works. Jacobians are
    finite differences of the *full* nonlinear model, which is affordable — one solve is a
    48×48 Newton loop and the whole fit is a few hundred of them. Damping is raised until a
    step actually lowers the cost, so a step that walks into a different contact set is
    rejected rather than followed; the active set makes the residual piecewise smooth, and
    that is the term that would otherwise make a plain Gauss-Newton overshoot at a kink.

    """
    keep = project if project is not None else (lambda x: x)
    x = keep(np.asarray(x0, dtype=np.float64).copy())
    residual = residual_at(x)
    cost = float(residual @ residual)
    damping = 1e-3
    n = x.size

    for iteration in range(1, max_iterations + 1):
        jacobian = np.empty((residual.size, n), dtype=np.float64)
        for column in range(n):
            step = 1e-6 * max(abs(x[column]), 1.0)
            probe = x.copy()
            probe[column] += step
            jacobian[:, column] = (residual_at(probe) - residual) / step
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ residual

        for _ in range(_MAX_DAMPING_STEPS):
            damped = normal + damping * np.diag(np.maximum(np.diag(normal), 1e-12))
            delta = np.linalg.lstsq(damped, -gradient, rcond=None)[0]
            trial = keep(x + delta)
            trial_residual = residual_at(trial)
            trial_cost = float(trial_residual @ trial_residual)
            if trial_cost < cost:
                break
            damping *= 10.0
        else:
            # No amount of damping improves on this point, so it is a local minimum of the
            # residual. Converged in the only sense available; whether it is a *good* minimum
            # is what rms_error_fraction is for.
            return x, iteration, True

        # Measure the step actually taken, which after projection is not ``delta``.
        moved = float(np.max(np.abs(trial - x) / np.maximum(np.abs(x), 1e-12)))
        x, residual, cost = trial, trial_residual, trial_cost
        damping = max(damping * 0.1, 1e-12)
        if moved <= tol:
            return x, iteration, True

    return x, max_iterations, False


def nnls(matrix: np.ndarray, rhs: np.ndarray, *, tol: float = 1e-12,
         max_iterations: int | None = None) -> np.ndarray:
    """Non-negative least squares: ``argmin ||A x - b||`` subject to ``x >= 0``.

    Lawson-Hanson, in about thirty lines, because there is no scipy in this project and the
    only alternative — an unconstrained solve followed by clipping the negatives to zero — is
    **not** the same answer. Clipping does not re-fit the surviving coefficients, so it lands
    on a point that is neither the constrained optimum nor a good approximation of it, and on
    a deconvolution like this one the difference is large: the negative slope it removes was
    compensating for a neighbour that is now left too big.

    The one thing to know about the algorithm: the outer loop admits one variable at a time by
    steepest gradient, and the inner loop drops any admitted variable that goes non-positive,
    walking to the boundary rather than jumping past it. Both loops are capped — this is
    called inside a fit, and invariant 4's spirit is that nothing here spins forever on a
    pathological curve. Hitting a cap returns the best feasible point reached, which is a
    valid law, just not the optimal one.
    """
    a = np.asarray(matrix, dtype=np.float64)
    b = np.asarray(rhs, dtype=np.float64).ravel()
    n = a.shape[1]
    cap = max_iterations if max_iterations is not None else 3 * n
    x = np.zeros(n, dtype=np.float64)
    passive = np.zeros(n, dtype=bool)
    gradient = a.T @ (b - a @ x)

    for _ in range(cap):
        candidates = ~passive & (gradient > tol)
        if not np.any(candidates):
            break
        passive[int(np.argmax(np.where(candidates, gradient, -np.inf)))] = True

        for _ in range(cap):
            trial = np.zeros(n, dtype=np.float64)
            trial[passive] = np.linalg.lstsq(a[:, passive], b, rcond=None)[0]
            if np.all(trial[passive] > 0.0):
                x = trial
                break
            blocking = passive & (trial <= 0.0)
            ratios = np.where(blocking, x / np.maximum(x - trial, 1e-300), np.inf)
            x = x + float(np.min(ratios)) * (trial - x)
            passive &= x > tol
            x = np.where(passive, x, 0.0)
            if not np.any(passive):
                break
        else:  # pragma: no cover - inner loop exhausts only on a degenerate matrix
            break

        gradient = a.T @ (b - a @ x)

    return x


def fit_spring_law(
    spec: RingSpec,
    delta_m: np.ndarray,
    force_n: np.ndarray,
    *,
    order: int = 3,
    max_iterations: int = 60,
    tol: float = 1e-8,
) -> RingFit:
    """Least-squares fit of the segment spring law to a measured load curve.

    Without coupling this is one exact least-squares solve. With coupling it is damped
    Gauss-Newton on the three coefficients (:func:`_levenberg_marquardt`), seeded from the
    uncoupled fit. The reported error is **not** the optimiser's last residual — it is
    recomputed by running the full nonlinear coupled model at the final law, so a fit that
    stopped early still reports the error it actually achieves.

    Args:
        spec: ring geometry, segment count and neighbour coupling.
        delta_m: hub indentation, metres. **Loading branch only** — pass
            ``curve.delta_m[curve.loading]``. Feeding both branches of a hyperelastic sweep
            in just duplicates every point, and feeding a buckled unloading branch in fits
            the average of two different equilibrium paths.
        force_n: measured reaction at those indentations, newtons.
        order: 1, 2 or 3 terms of the cubic. Lower it when the data is too short to support
            three — a two-point curve fitted with three coefficients is exact and meaningless.
        max_iterations: cap on the Gauss-Newton iterations. Ignored without coupling.
        tol: relative coefficient change that counts as settled. Finite-difference Jacobians
            bottom out around 1e-8, so tightening this past that just spends iterations.

    Returns:
        A :class:`RingFit`. Check :attr:`RingFit.ok` before using the law.

    Raises:
        FitFailure: on empty, mismatched, or degenerate input.
    """
    if not 1 <= order <= 3:
        raise FitFailure("order must be 1, 2 or 3")
    d, f = _clean(delta_m, force_n, order, "coefficients being fitted")

    # Seed from the uncoupled problem even when the spec is coupled: it is the same fit with
    # the band's share of the load wrongly attributed to the springs, which is a close enough
    # starting point that the Gauss-Newton below converges in single-figure passes.
    uncoupled = replace(spec, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0)
    matrix = _design_matrix(uncoupled, d)[:, :order]
    if not np.all(np.isfinite(matrix)):  # pragma: no cover - defensive
        raise FitFailure("ring response is not finite; check radius and segment count")
    law = _law_from(np.linalg.lstsq(matrix, f, rcond=None)[0], order)

    iterations, converged = 1, True
    if spec.is_coupled:
        # Rescale to ``x_j = coeff_j · u*^(j+1)`` with ``u*`` the deepest indentation, so all
        # three carry force units and differ by ones rather than by the seven orders of
        # magnitude between ``a`` in N/m and ``c`` in N/m³. Without that the optimiser's
        # finite differences and its relative-step test are meaningless for two of the three.
        reference_m = float(np.max(d))
        scale = np.array([reference_m, reference_m**2, reference_m**3])[:order]
        seed = np.array([law.a, law.b, law.c])[:order] * scale

        def residual_at(x: np.ndarray) -> np.ndarray:
            return ring_force_n(spec, _law_from(x / scale, order), d) - f

        x, iterations, converged = _levenberg_marquardt(
            residual_at, seed, max_iterations, tol
        )
        law = _law_from(x / scale, order)
        if not all(solve_equilibrium(spec, law, float(delta)).converged for delta in d):
            converged = False

    return _result(spec, law, d, f, iterations, converged)


def fit_tabulated_law(
    spec: RingSpec,
    delta_m: np.ndarray,
    force_n: np.ndarray,
    *,
    n_intervals: int | None = None,
    monotone: bool = False,
    smoothing: float = 0.1,
    max_iterations: int = 60,
    tol: float = 1e-8,
) -> RingFit:
    """Fit a :class:`TabulatedLaw` to a measured load curve.

    The same deconvolution as :func:`fit_spring_law` against a basis that can bend any way the
    data does. Uncoupled it is one non-negative least squares solve — convex, so its answer is
    *the* optimum over the feasible set and not a local one — with no iteration and no initial
    guess, because the ring response is linear in the table
    (:func:`~wheelopt.rom.ring.ramp_basis`). Coupled it is the projected Gauss-Newton the cubic
    uses, seeded from the uncoupled answer.

    **The constraint is that the law may not pull, not that it may not soften.** The two
    parametrisations of the same table differ in exactly that, and the choice is
    ``monotone``:

    * ``False`` (default) fits the **knot forces**, constrained non-negative. The law can have
      a negative tangent — it can buckle — and still never pulls the ground.
    * ``True`` fits the **interval slopes**, constrained non-negative, giving a law that never
      softens. Both are plain NNLS; ``s = D v`` is a triangular change of variables, so the
      same design matrix serves both.

    The default is the permissive one because the strict one **cannot fit the nominal
    design**, and that was not obvious in advance: on its measured curve at 24 segments and 12
    intervals, monotone reaches 12.87% RMS and non-negative-force reaches 2.35%. Monotonicity
    was the binding constraint all along; the cubic was just the first thing to hit it. Use
    ``monotone=True`` when a softening segment is known to be a fitting artefact rather than a
    design's behaviour — and expect the error to rise if it is not.

    Resolution, and why fit error cannot choose it
    ----------------------------------------------
    Too few intervals and the table is a cubic with extra steps and fewer parameters:
    measured on the tiny design's six-point curve, two intervals gives 4.93% RMS against the
    cubic's 2.94%, three gives 3.25%, four gives 2.26%. Too many and the deconvolution is
    under-determined, and **the error keeps falling while the law stops being physical.**
    Measured on the nominal design at 36 segments, unregularised, the RMS runs 10.60% → 6.19%
    → 4.67% → 4.01% → 3.42% as the table goes from 4 intervals to 12, while the fitted tangent
    goes from ``14.1 → −8.1 → −0.8 → 9.4`` N/mm to a sequence swinging between −52 and +63.
    A spoke does not do that. This is the same trap the segment-count sweep already contains —
    twelve segments fit the tiny design best while putting one segment in contact — and the
    same answer applies: **the fit error is not the only number.** Look at the tangents.

    ``smoothing`` is the partial remedy: a penalty on the *change* in tangent between
    intervals, appended as extra rows so the problem stays one convex NNLS. It is scaled by
    ``‖M‖/‖D‖`` so the same value means the same thing on any design. The default of 0.1 was
    picked from a measured sweep on the nominal: it costs 0.11 percentage points of RMS
    (3.42% → 3.53%) and takes the worst spurious tangent from +63 N/mm to −12.

    It is a remedy and **not a cure**, which is worth being plain about. Even at 0.3 the
    nominal's fitted law still reverses sign three times. The oscillation is telling the truth
    about the model: deconvolving a *banded* wheel's whole-wheel curve into independent radial
    segments is ill-posed, because the band carries load between segments and the segment law
    is being asked to absorb that. The real fix is either a coupled fit that converges or —
    the direction this project took — a bandless claw wheel where the segments *are* the
    claws, each measured directly, and there is no deconvolution to be ill-posed.

    Args:
        spec: ring geometry, segment count and neighbour coupling.
        delta_m: hub indentation, metres. Loading branch only — same reasoning as
            :func:`fit_spring_law`.
        force_n: measured reaction at those indentations, newtons.
        n_intervals: table resolution. ``None`` picks it from the data length.
        monotone: forbid a softening branch. See above; the default is not to.
        smoothing: weight on the tangent-change penalty, relative to the data term. 0
            disables it, which is what a round-trip test against a known law wants.
        max_iterations: cap on the Gauss-Newton iterations. Ignored without coupling.
        tol: relative parameter change that counts as settled.

    Returns:
        A :class:`RingFit` whose ``law`` is a :class:`TabulatedLaw`.

    Raises:
        FitFailure: on empty, mismatched, or degenerate input.
    """
    if n_intervals is not None and n_intervals < 1:
        raise FitFailure("n_intervals must be at least 1")
    if smoothing < 0:
        raise FitFailure("smoothing must be non-negative")
    d, f = _clean(delta_m, force_n, n_intervals or 1, "table intervals")
    if n_intervals is None:
        n_intervals = max(1, min(8, len(d) // 2))

    # The deepest segment is compressed by exactly δ, so the table spans the same range as the
    # data; the law extrapolates beyond it at a clamped slope, which is the table's business
    # and not the fit's.
    knots = uniform_knots(float(np.max(d)), n_intervals)
    widths = np.diff(knots)
    # x -> slopes. Under `monotone` the parameters *are* the slopes; otherwise they are the
    # knot forces v_1..v_K and s_j = (v_{j+1} - v_j)/w_j with v_0 = 0 — a triangular change of
    # variables, so one design matrix in slope space serves both feasible sets.
    to_slopes = (np.eye(n_intervals) if monotone
                 else (np.eye(n_intervals) - np.eye(n_intervals, k=-1)) / widths[:, None])

    def law_at(x: np.ndarray) -> TabulatedLaw:
        slopes = to_slopes @ np.maximum(x, 0.0)
        return TabulatedLaw(knots_m=knots, slopes_n_per_m=slopes)

    uncoupled = replace(spec, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0)
    slope_basis = [
        TabulatedLaw(knots_m=knots, slopes_n_per_m=np.eye(n_intervals)[j])
        for j in range(n_intervals)
    ]
    in_slopes = np.column_stack([ring_force_n(uncoupled, law, d) for law in slope_basis])
    matrix = in_slopes @ to_slopes
    if not np.all(np.isfinite(matrix)):  # pragma: no cover - defensive
        raise FitFailure("ring response is not finite; check radius and segment count")

    # Tikhonov rows appended to the data rows: the augmented problem is still one convex NNLS
    # over the same feasible set, so nothing about the solve's character changes.
    penalty = np.zeros((0, n_intervals), dtype=np.float64)
    if smoothing > 0 and n_intervals > 1:
        difference = np.eye(n_intervals - 1, n_intervals, k=1) - np.eye(n_intervals - 1,
                                                                       n_intervals)
        penalty = difference @ to_slopes
        norm = np.linalg.norm(penalty)
        if norm > 0:
            penalty *= smoothing * np.linalg.norm(matrix) / norm
    x = nnls(np.vstack([matrix, penalty]),
             np.concatenate([f, np.zeros(len(penalty))]))

    iterations, converged = 1, True
    if spec.is_coupled:
        def residual_at(probe: np.ndarray) -> np.ndarray:
            return ring_force_n(spec, law_at(probe), d) - f

        x, iterations, converged = _levenberg_marquardt(
            residual_at, x, max_iterations, tol, project=lambda p: np.maximum(p, 0.0),
        )
        if not all(solve_equilibrium(spec, law_at(x), float(delta)).converged for delta in d):
            converged = False

    return _result(spec, law_at(x), d, f, iterations, converged)


def ring_from_claw_curve(
    params: WheelParams,
    delta_m: np.ndarray,
    force_n: np.ndarray,
) -> tuple[RingSpec, TabulatedLaw]:
    """Build a ring straight from **one claw's** measured tip load-deflection curve.

    There is no fit here, and that is the entire point. For a bandless claw wheel the ring's
    segments *are* the claws — one segment per claw, no band, so no coupling — and a segment's
    compression is the tip deflection of the claw it stands for. The measured ``F(δ)`` of a
    single claw pressed onto a plate is therefore the segment spring law **as measured**, not
    something to be recovered from a whole-wheel curve.

    Compare what that removes. The whole-wheel path (:func:`fit_spring_law`,
    :func:`fit_tabulated_law`) is a deconvolution: every δ mixes many different segment
    compressions, so the law has to be inferred, the segment count is a free discretisation
    parameter swept over 12/24/36/48, and on a banded wheel the inverse problem is ill-posed
    enough that the fitted tangent oscillates between −52 and +63 N/mm while the error falls.
    None of that exists here. There is one segment count and it is ``n_spokes``; there is one
    law and it is the data; there is no fit error to report because nothing was fitted.

    What it does *not* remove is the ring's own limitation. A segment slides radially and only
    radially, so this law describes the claw's radial response and says nothing about the
    backward bend under drive torque that is much of what makes a claw grip. That is a
    separate degree of freedom and a separate measurement — see ``docs/plan/TODO.md`` #20.

    Args:
        params: the design. Must be bandless; ``n_spokes`` becomes the segment count.
        delta_m: radial tip deflection, metres, ascending, loading branch only.
        force_n: radial reaction at those deflections, newtons.

    Returns:
        ``(spec, law)`` ready for :func:`~wheelopt.rom.ring.ring_force_n` and the MJCF.

    Raises:
        FitFailure: if the design has a band, or the curve is not a usable loading branch.
    """
    if params.has_shear_band:
        raise FitFailure(
            "segments-are-claws needs a bandless design: with a band the claws share load "
            f"through it and one claw's curve is not a segment law. rim_thickness_mm is "
            f"{params.rim_thickness_mm:g}"
        )
    d = np.asarray(delta_m, dtype=np.float64).ravel()
    f = np.asarray(force_n, dtype=np.float64).ravel()
    if d.shape != f.shape:
        raise FitFailure(f"delta and force have different shapes: {d.shape} vs {f.shape}")
    keep = d > 0.0
    d, f = d[keep], f[keep]
    if len(d) < 1:
        raise FitFailure("no positive deflection in the claw curve")
    if np.any(np.diff(d) <= 0.0):
        raise FitFailure("claw deflections must be strictly ascending; sort the curve first")
    if np.any(f < 0.0):
        raise FitFailure(
            "the claw curve pulls: a tip pressed onto a plate cannot react negatively, so "
            "this is a sign convention or an extraction bug, not a soft claw"
        )
    # The law must pass through the origin, and the measured curve starts at the first
    # sampled deflection rather than at zero. Prepending (0, 0) is not an assumption — a claw
    # at its undeformed radius carries no load — but it does make the first interval's slope
    # an extrapolation back to the origin from the first two samples' worth of data.
    knots = np.concatenate([[0.0], d])
    forces = np.concatenate([[0.0], f])
    spec = RingSpec(radius_m=params.outer_radius_mm * 1e-3, n_segments=params.n_spokes)
    return spec, TabulatedLaw.from_forces(knots, forces)


def _clean(delta_m: np.ndarray, force_n: np.ndarray, n_parameters: int,
           what: str) -> tuple[np.ndarray, np.ndarray]:
    """Validate a load curve and drop the δ = 0 point. Shared by both fitters."""
    d = np.asarray(delta_m, dtype=np.float64).ravel()
    f = np.asarray(force_n, dtype=np.float64).ravel()
    if d.shape != f.shape:
        raise FitFailure(f"delta and force have different shapes: {d.shape} vs {f.shape}")
    keep = d > 0.0
    d, f = d[keep], f[keep]
    if len(d) < n_parameters:
        raise FitFailure(
            f"{len(d)} usable points is fewer than the {n_parameters} {what}; "
            "a fit with more freedom than data is exact and says nothing"
        )
    if not np.any(f > 0):
        raise FitFailure("no positive force in the curve; nothing was in contact")
    return d, f


def _result(spec: RingSpec, law: RadialLaw, d: np.ndarray, f: np.ndarray,
            iterations: int, converged: bool) -> RingFit:
    """Assemble a :class:`RingFit`, measuring the error against the *full* nonlinear model.

    Deliberately not the optimiser's last residual: a fit that stopped early, or one whose
    slopes were clamped by the projection after the last cost evaluation, would otherwise
    report an error it does not achieve.
    """
    predicted = ring_force_n(spec, law, d)
    residual = predicted - f
    peak = float(np.max(np.abs(f)))
    rms = float(np.sqrt(np.mean(residual**2)))
    return RingFit(
        spec=spec,
        law=law,
        rms_error_n=rms,
        rms_error_fraction=rms / peak if peak > 0 else float("inf"),
        max_error_n=float(np.max(np.abs(residual))),
        delta_m=d,
        force_n=f,
        fitted_force_n=predicted,
        iterations=iterations,
        converged=converged,
    )


def _law_from(coeffs: np.ndarray, order: int) -> SpringLaw:
    """Pad a truncated coefficient vector back out to the full cubic."""
    padded = list(coeffs) + [0.0] * (3 - order)
    return SpringLaw(a=float(padded[0]), b=float(padded[1]), c=float(padded[2]))



def contact_segments(spec: RingSpec, delta_m: float, law: RadialLaw | None = None) -> int:
    """How many segments carry load at this indentation.

    The discretisation check. A ring resolving a contact patch with two or three segments is
    not modelling a patch, it is modelling three point loads, and its ``F(δ)`` will be
    visibly stepped as segments engage one at a time. Compare against the FEA contact patch
    length before trusting the fit.

    Args:
        law: needed only for a coupled ring, where the patch is an outcome of the equilibrium
            rather than a property of the geometry — the band can drag a segment the
            undeformed circle would clear down onto the plate. Without it the count falls
            back to the geometric interference, which is a *lower* bound in that case; the
            caller has to decide whether it wants the bound or the answer.
    """
    if law is not None and spec.is_coupled:
        return int(np.count_nonzero(solve_equilibrium(spec, law, delta_m).in_contact))
    return int(np.count_nonzero(penetrations(spec, delta_m) > 0.0))
