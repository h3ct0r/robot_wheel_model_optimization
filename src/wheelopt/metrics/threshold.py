"""The threshold-metric fix: a success ladder, a logistic fit, and a continuous height.

`docs/plan/08-metrics.md`: *"Maximum step height cleared" is a discontinuous, noisy threshold.
Bisection on a stochastic simulator gives a jittery signal that poisons a GP surrogate.*
Instead, evaluate a fixed ladder of heights across seeds, fit a logistic success curve, and
report the height at which success probability crosses 0.9 as a continuous quantity with an
uncertainty estimate.

This module is that, and it is mostly about the ways it can fail. A logistic fit to a success
ladder has three degenerate cases and every one of them returns a plausible number if it is
not checked for:

- **Perfect separation** — every run below some height succeeded and every run above it
  failed, which is what a *good* ladder on a low-noise simulator looks like. The maximum
  likelihood estimate does not exist: the slope runs to infinity and the likelihood keeps
  improving, so an optimiser stops on whatever iteration cap it was given and reports a
  finite, confident, meaningless slope. Detected explicitly and reported as
  :attr:`ThresholdFit.separated`, with the threshold bracketed by the ladder instead.
- **No crossing** — all successes, or all failures. The answer is outside the ladder and the
  honest report is a bound, not a number. :attr:`censored` says which end.
- **A negative slope** — success probability *rising* with obstacle height. Physically
  nonsense, so it means the ladder or the predicate is wrong, and quietly extrapolating a
  crossing from it would produce a step height with no relationship to anything.

Pure numpy: the fit is Newton-Raphson on the logistic log-likelihood (IRLS), about fifteen
lines, so this stays importable and testable with nothing installed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["SUCCESS_LEVEL", "ThresholdFit", "fit_threshold", "logistic"]

#: The success probability the reported height is quoted at. `08-metrics.md` fixes it at 0.9.
SUCCESS_LEVEL = 0.9

#: Newton steps before giving up. Logistic IRLS on well-posed data converges in under ten;
#: hitting this cap means the data are separated, which is tested for directly rather than
#: inferred from the iteration count.
_MAX_ITERATIONS = 50


def logistic(x: np.ndarray | float) -> np.ndarray:
    """``1 / (1 + exp(-x))``, computed without overflowing on large negative ``x``."""
    x = np.asarray(x, dtype=np.float64)
    out = np.empty_like(x)
    positive = x >= 0
    out[positive] = 1.0 / (1.0 + np.exp(-x[positive]))
    exp_x = np.exp(x[~positive])
    out[~positive] = exp_x / (1.0 + exp_x)
    return out


@dataclass(frozen=True, slots=True)
class ThresholdFit:
    """A fitted success curve and the height at :data:`SUCCESS_LEVEL`.

    ``ok`` is the only thing a caller should gate on. A fit that is separated or censored
    still carries its best available answer — that is useful for a plot and for a log — but
    it is not a number to put in an objective vector.
    """

    #: Height at which the fitted success probability crosses :data:`SUCCESS_LEVEL`, metres.
    #: On a separated or censored fit this is the ladder-based bracket, not an extrapolation.
    height_m: float
    #: One standard error on ``height_m`` by the delta method, metres. ``inf`` when the fit
    #: is separated or censored, because there is no finite information about the slope.
    stderr_m: float
    intercept: float
    slope_per_m: float
    #: True when success and failure are perfectly separated by height and the MLE does not
    #: exist. Common on a low-noise simulator, so not an error — but not a fit either.
    separated: bool = False
    #: ``"below"`` if every run succeeded (the true height is above the ladder), ``"above"``
    #: if every run failed, ``""`` when the ladder brackets the crossing.
    censored: str = ""
    #: Set when the fitted slope is non-negative — success *rising* with obstacle height. The
    #: healthy slope is negative: a wheel clears fewer obstacles as they get taller.
    inverted: bool = False
    n_heights: int = 0
    n_runs: int = 0
    #: Lowest and highest rung actually run, metres. The scale everything else is judged
    #: against, which is what makes the checks in `reason` derived rather than tuned.
    low_rung_m: float = 0.0
    high_rung_m: float = 0.0

    @property
    def ladder_span_m(self) -> float:
        """Highest rung minus lowest, metres."""
        return self.high_rung_m - self.low_rung_m

    @property
    def reason(self) -> str:
        """Why this fit is not usable, or ``""`` when it is. The thing to print.

        The last two clauses were added after a fit reported **77.2 ± 46029.6 mm** and passed
        every earlier check: finite, converged, not separated, not censored, slope the right
        sign. A 46-metre standard error on a 77-millimetre answer is not a measurement, and
        nothing about the number itself says so — it is the ladder that says so. Both tests
        are therefore scaled by the ladder rather than by a constant:

        * a standard error wider than the whole ladder means the data locate the crossing
          nowhere within the experiment that was run;
        * a crossing outside the rungs is an extrapolation from a curve fitted entirely
          elsewhere, and the honest response is a better ladder, not a smaller font.
        """
        if self.censored:
            return f"censored {self.censored} the ladder"
        if self.separated:
            return "perfectly separated; the MLE does not exist"
        if self.inverted:
            return "success rises with obstacle height"
        if not np.isfinite(self.height_m) or not np.isfinite(self.stderr_m):
            return "non-finite fit"
        if self.ladder_span_m > 0.0 and self.stderr_m > self.ladder_span_m:
            return (f"standard error {self.stderr_m * 1e3:.0f} mm exceeds the "
                    f"{self.ladder_span_m * 1e3:.0f} mm ladder; add seeds or rungs")
        if self.ladder_span_m > 0.0 and not (self.low_rung_m <= self.height_m
                                             <= self.high_rung_m):
            return "the crossing falls outside the rungs; re-centre the ladder"
        return ""

    @property
    def ok(self) -> bool:
        """Whether ``height_m`` is a number worth putting in an objective vector."""
        return not self.reason

    def summary(self) -> str:  # pragma: no cover - display only
        body = (f"{self.height_m * 1e3:.1f} +/- {self.stderr_m * 1e3:.1f} mm at "
                f"P={SUCCESS_LEVEL:.0%} ({self.n_runs} runs over {self.n_heights} heights)")
        return body if self.ok else f"{body}  <- NOT USABLE: {self.reason}"


def fit_threshold(
    heights_m: np.ndarray,
    successes: np.ndarray,
    *,
    level: float = SUCCESS_LEVEL,
) -> ThresholdFit:
    """Fit `P(success | height)` and report the height at ``level``.

    Args:
        heights_m: one entry per run, metres. Repeated values are expected — the ladder is
            10 heights × 8 seeds, so each height appears eight times.
        successes: one entry per run, truthy for a clear.
        level: success probability to quote the height at. 0.9 per `08-metrics.md`.

    Returns:
        A :class:`ThresholdFit`. **Never raises for degenerate data** — separated, censored
        and inverted are outcomes a real ladder produces, and invariant 4 makes them results.
        It does raise for *malformed* data, which is a caller bug rather than a design's
        behaviour.
    """
    h = np.asarray(heights_m, dtype=np.float64).ravel()
    y = np.asarray(successes).ravel().astype(np.float64)
    if h.shape != y.shape:
        raise ValueError(f"heights and successes differ in shape: {h.shape} vs {y.shape}")
    if h.size == 0:
        raise ValueError("no runs to fit")
    if not np.all(np.isfinite(h)):
        raise ValueError("non-finite height in the ladder")
    if not np.all((y == 0.0) | (y == 1.0)):
        raise ValueError("successes must be boolean-like: a run cleared the step or it did not")
    if not 0.0 < level < 1.0:
        raise ValueError(f"level must be in (0, 1); got {level}")

    ladder = np.unique(h)
    common = {"n_heights": int(ladder.size), "n_runs": int(h.size),
              "low_rung_m": float(ladder.min()), "high_rung_m": float(ladder.max())}

    if np.all(y == 1.0):
        return ThresholdFit(height_m=float(ladder.max()), stderr_m=float("inf"),
                            intercept=float("inf"), slope_per_m=0.0, censored="below", **common)
    if np.all(y == 0.0):
        return ThresholdFit(height_m=float(ladder.min()), stderr_m=float("inf"),
                            intercept=float("-inf"), slope_per_m=0.0, censored="above", **common)

    # Separation: no run contradicts a step function, so no finite slope is maximal and the
    # MLE does not exist. Checked in **both** orientations. The first is what a clean
    # simulator produces and the honest answer is the gap the ladder resolved; the second is
    # separated *and* nonsense, and saying only "separated" would let a caller read the
    # bracket as a step height when success was rising with obstacle height.
    cleared, failed = h[y == 1.0], h[y == 0.0]
    if float(cleared.max()) < float(failed.min()):
        return ThresholdFit(
            height_m=0.5 * (float(cleared.max()) + float(failed.min())),
            stderr_m=float("inf"), intercept=float("nan"), slope_per_m=float("-inf"),
            separated=True, **common,
        )
    if float(cleared.min()) > float(failed.max()):
        return ThresholdFit(
            height_m=float("nan"), stderr_m=float("inf"), intercept=float("nan"),
            slope_per_m=float("inf"), separated=True, inverted=True, **common,
        )

    # Centre and scale the heights before fitting. On raw metres a ladder spans 0.01-0.20 and
    # the slope is O(100/m), which makes the Newton system badly conditioned for no reason.
    centre, scale = float(np.mean(h)), float(np.std(h)) or 1.0
    z = (h - centre) / scale
    design = np.column_stack([np.ones_like(z), z])
    beta = np.zeros(2)
    for _ in range(_MAX_ITERATIONS):
        p = logistic(design @ beta)
        weights = np.clip(p * (1.0 - p), 1e-12, None)
        gradient = design.T @ (y - p)
        hessian = design.T @ (design * weights[:, None])
        try:
            step = np.linalg.solve(hessian, gradient)
        except np.linalg.LinAlgError:  # pragma: no cover - guarded by the separation check
            break
        beta = beta + step
        if np.max(np.abs(step)) < 1e-10:
            break

    intercept_z, slope_z = float(beta[0]), float(beta[1])
    slope = slope_z / scale
    intercept = intercept_z - slope_z * centre / scale
    # A step wheel clears *fewer* obstacles as they get taller, so the fitted slope is
    # negative and that is the healthy case. A non-negative one says success rose with
    # height, which is not a soft wheel — it is a broken predicate or a mislabelled ladder.
    if slope >= 0.0:
        return ThresholdFit(height_m=float("nan"), stderr_m=float("inf"), intercept=intercept,
                            slope_per_m=slope, inverted=True, **common)

    target = float(np.log(level / (1.0 - level)))
    height = (target - intercept) / slope

    # Delta method on the *scaled* parameters, then one linear change of variables. The
    # threshold in z is (target - b0)/b1, so d/d(b0) = -1/b1 and d/d(b1) = -(target-b0)/b1^2.
    p = logistic(design @ beta)
    weights = np.clip(p * (1.0 - p), 1e-12, None)
    try:
        covariance = np.linalg.inv(design.T @ (design * weights[:, None]))
    except np.linalg.LinAlgError:  # pragma: no cover
        return ThresholdFit(height_m=height, stderr_m=float("inf"), intercept=intercept,
                            slope_per_m=slope, **common)
    z_star = (target - intercept_z) / slope_z
    jacobian = np.array([-1.0 / slope_z, -z_star / slope_z])
    variance_z = float(jacobian @ covariance @ jacobian)
    stderr = scale * float(np.sqrt(max(variance_z, 0.0)))
    return ThresholdFit(height_m=height, stderr_m=stderr, intercept=intercept,
                        slope_per_m=slope, **common)
