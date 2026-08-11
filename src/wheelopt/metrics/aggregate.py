"""Robust aggregation over terrain seeds × material realisations. Invariant 7.

Every design is scored over `k ≥ 8` terrain seeds **times** `m ≥ 4` material realisations and
aggregated with **CVaR at 25%** — the mean of the worst quartile — never the mean.
`docs/plan/08-metrics.md`. The reason is hardware transfer: a design whose *average* run is
excellent and whose worst quartile collapses is a design that fails on the bench, and the
mean cannot see the difference.

**The direction is a required argument, and that is the whole safety property of this module.**
CVaR is the mean of the *worst* tail, and which tail is worst depends entirely on whether the
metric is being maximised or minimised — the lowest quartile of step heights, the highest
quartile of cost of transport. Defaulting it would make the wrong answer the easy one to
write, and the wrong answer here is a perfectly plausible number of the right magnitude and
units that ranks designs backwards. It is the exact shape of failure this project keeps
recording, so :func:`cvar` refuses to guess.
"""

from __future__ import annotations

from enum import Enum

import numpy as np

__all__ = ["CVAR_ALPHA", "Direction", "cvar", "cvar_table", "tail_size"]

#: The tail fraction invariant 7 fixes. Not a tuning knob — changing it changes what every
#: recorded score means, so it lives here as a named constant and callers pass it on purpose.
CVAR_ALPHA = 0.25


class Direction(str, Enum):
    """Which way is better for a metric. There is no sensible default."""

    #: Higher is better — step height cleared, gap crossed, max gradient.
    MAXIMISE = "maximise"
    #: Lower is better — cost of transport, ride harshness, mass, sag.
    MINIMISE = "minimise"


def tail_size(n_samples: int, alpha: float = CVAR_ALPHA) -> float:
    """How many samples the ``alpha`` tail contains. Fractional on purpose.

    ``n·α``, not ``floor(n·α)``. With the 8 × 4 = 32 samples `08-metrics.md` asks for the
    distinction is invisible — 32 × 0.25 is exactly 8 — but a scenario that loses a seed to a
    diverged run leaves 31, and rounding 7.75 down to 7 quietly changes what the number means
    between one design and the next. :func:`cvar` weights the boundary sample instead.
    """
    if n_samples < 1:
        raise ValueError("need at least one sample")
    if not 0.0 < alpha <= 1.0:
        raise ValueError(f"alpha must be in (0, 1]; got {alpha}")
    return n_samples * alpha


def cvar(values: np.ndarray, direction: Direction, alpha: float = CVAR_ALPHA) -> float:
    """Conditional value at risk: the mean of the worst ``alpha`` fraction of ``values``.

    The estimator is the standard one, with the boundary sample carried at fractional weight:
    sort worst-first, take the first ``⌊nα⌋`` outright, and give the next one the leftover
    ``nα − ⌊nα⌋``. That makes the result continuous in ``alpha`` and in the sample count, so
    two designs scored over different numbers of surviving seeds stay comparable.

    With ``alpha = 1`` this is the plain mean, which is the honest degenerate case rather than
    an error — a caller sweeping ``alpha`` to show what robustness costs needs that endpoint.

    Args:
        values: one metric, one value per (seed, material realisation). Must all be finite:
            a diverged run is a *missing* sample, not a NaN to be averaged over, and mixing
            the two is how a design that failed half its runs scores well.
        direction: which tail is the bad one. Required — see the module docstring.
        alpha: tail fraction. Defaults to invariant 7's 0.25.

    Raises:
        ValueError: on an empty input, a non-finite value, or an out-of-range ``alpha``.
    """
    x = np.asarray(values, dtype=np.float64).ravel()
    if x.size == 0:
        raise ValueError("no samples to aggregate")
    if not np.all(np.isfinite(x)):
        raise ValueError(
            "non-finite value in the sample: a diverged or failed run is a sample that is "
            "absent, not one worth NaN. Drop it and record the failure as its own row."
        )
    count = tail_size(x.size, alpha)
    # Worst first. For a maximised metric that is ascending; for a minimised one, descending.
    worst = np.sort(x) if direction is Direction.MAXIMISE else np.sort(x)[::-1]
    whole = int(np.floor(count))
    remainder = count - whole
    if whole >= x.size:
        return float(np.mean(worst))
    total = float(np.sum(worst[:whole])) + remainder * float(worst[whole])
    return total / count


def cvar_table(
    samples: dict[str, np.ndarray],
    directions: dict[str, Direction],
    alpha: float = CVAR_ALPHA,
) -> dict[str, float]:
    """CVaR for several metrics at once. Every metric must state its direction.

    Raises rather than skipping a metric with no direction: a metric silently dropped from an
    aggregation is a metric silently dropped from the objective vector, and the campaign that
    notices is the one that has already run.
    """
    missing = sorted(set(samples) - set(directions))
    if missing:
        raise ValueError(
            f"no direction given for {missing}; CVaR cannot know which tail is the bad one"
        )
    return {name: cvar(values, directions[name], alpha) for name, values in samples.items()}
