"""Metric extraction and robust aggregation. `docs/plan/08-metrics.md`.

Two pieces so far, both pure numpy and both about the same thing — not letting a plausible
number stand in for a real one:

- :mod:`~wheelopt.metrics.aggregate` — **CVaR at 25%**, the mean of the worst quartile over
  terrain seeds × material realisations (invariant 7). Requires the metric's direction; there
  is no default, because the wrong tail gives a number of the right units that ranks designs
  backwards.
- :mod:`~wheelopt.metrics.threshold` — the **logistic success curve** that turns "tallest step
  cleared" from a jittery bisected threshold into a continuous height at P = 0.9 with an
  uncertainty. Reports separation, censoring and inverted slopes rather than extrapolating
  through them.
"""

from .aggregate import CVAR_ALPHA, Direction, cvar, cvar_table, tail_size
from .threshold import SUCCESS_LEVEL, ThresholdFit, fit_threshold, logistic

__all__ = [
    "CVAR_ALPHA",
    "SUCCESS_LEVEL",
    "Direction",
    "ThresholdFit",
    "cvar",
    "cvar_table",
    "fit_threshold",
    "logistic",
    "tail_size",
]
