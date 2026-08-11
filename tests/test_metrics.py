"""CVaR and the logistic threshold metric. Both pure numpy, so all of this always runs.

The tests that matter are the ones checking these refuse to produce a plausible number from
data that does not support one — the wrong CVaR tail, and a logistic fit to perfectly
separated data. Both are silent by nature: they return a value of the right magnitude and
units, and only ranking or hardware finds out.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.metrics import (
    CVAR_ALPHA,
    SUCCESS_LEVEL,
    Direction,
    cvar,
    cvar_table,
    fit_threshold,
    logistic,
    tail_size,
)


class TestCvar(unittest.TestCase):
    def test_the_worst_quartile_of_a_maximised_metric_is_the_lowest(self):
        values = np.arange(1.0, 9.0)          # 1..8, so the worst two are 1 and 2
        self.assertAlmostEqual(cvar(values, Direction.MAXIMISE), 1.5)

    def test_the_worst_quartile_of_a_minimised_metric_is_the_highest(self):
        values = np.arange(1.0, 9.0)          # worst two are 8 and 7
        self.assertAlmostEqual(cvar(values, Direction.MINIMISE), 7.5)

    def test_the_two_directions_disagree_and_both_look_reasonable(self):
        """The point of making direction required. Both answers are in range, in the right
        units, and only one of them ranks designs the right way round."""
        values = np.array([0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.30])
        low = cvar(values, Direction.MAXIMISE)
        high = cvar(values, Direction.MINIMISE)
        self.assertLess(low, np.mean(values))
        self.assertGreater(high, np.mean(values))

    def test_it_is_never_better_than_the_mean(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            values = rng.normal(size=32)
            self.assertLessEqual(cvar(values, Direction.MAXIMISE), np.mean(values) + 1e-12)
            self.assertGreaterEqual(cvar(values, Direction.MINIMISE), np.mean(values) - 1e-12)

    def test_alpha_of_one_is_the_mean(self):
        values = np.array([1.0, 5.0, 9.0, 11.0])
        for direction in Direction:
            self.assertAlmostEqual(cvar(values, direction, alpha=1.0), float(np.mean(values)))

    def test_a_lost_seed_moves_the_answer_smoothly(self):
        """31 samples means a tail of 7.75, and flooring that to 7 would make the score jump
        between two designs for no reason but a diverged run."""
        self.assertAlmostEqual(tail_size(32), 8.0)
        self.assertAlmostEqual(tail_size(31), 7.75)
        full = cvar(np.arange(32.0), Direction.MAXIMISE)
        short = cvar(np.arange(31.0), Direction.MAXIMISE)
        self.assertLess(abs(full - short), 0.2)

    def test_the_boundary_sample_carries_fractional_weight(self):
        """Five samples at alpha=0.25 is a tail of 1.25: the worst outright, plus a quarter
        of the next, divided by 1.25."""
        values = np.array([0.0, 4.0, 8.0, 12.0, 16.0])
        expected = (0.0 + 0.25 * 4.0) / 1.25
        self.assertAlmostEqual(cvar(values, Direction.MAXIMISE), expected)

    def test_a_nan_is_refused_rather_than_averaged_over(self):
        """A diverged run is a sample that is absent. Treating it as a number is how a design
        that failed half its runs scores well."""
        with self.assertRaises(ValueError) as ctx:
            cvar(np.array([1.0, 2.0, np.nan, 4.0]), Direction.MAXIMISE)
        self.assertIn("non-finite", str(ctx.exception))

    def test_an_empty_sample_is_refused(self):
        with self.assertRaises(ValueError):
            cvar(np.array([]), Direction.MAXIMISE)

    def test_the_invariant_7_default(self):
        self.assertEqual(CVAR_ALPHA, 0.25)

    def test_a_metric_without_a_direction_is_refused_not_skipped(self):
        with self.assertRaises(ValueError) as ctx:
            cvar_table({"climb": np.arange(8.0), "cot": np.arange(8.0)},
                       {"climb": Direction.MAXIMISE})
        self.assertIn("cot", str(ctx.exception))

    def test_the_table_agrees_with_the_scalar(self):
        values = np.arange(1.0, 9.0)
        table = cvar_table({"a": values}, {"a": Direction.MINIMISE})
        self.assertAlmostEqual(table["a"], cvar(values, Direction.MINIMISE))


class TestLogistic(unittest.TestCase):
    def test_it_does_not_overflow_on_large_magnitudes(self):
        out = logistic(np.array([-800.0, 0.0, 800.0]))
        np.testing.assert_allclose(out, [0.0, 0.5, 1.0], atol=1e-12)
        self.assertTrue(np.all(np.isfinite(out)))


def ladder(heights_mm, per_height, cutoff_mm, rng=None, sharpness_per_mm=0.15):
    """A synthetic ladder sampled from a **known** logistic curve.

    `P(success | h) = logistic(sharpness * (cutoff - h))`, so the uncertainty concentrates
    near the cutoff and runs far either side are nearly certain — which is what terrain
    variation does to a rover, and what a flat per-run flip probability does not. A uniform
    15% flip caps success at 0.85 at *every* height, so P=0.9 is unreachable, the crossing is
    an extrapolation far below the lowest rung, and the fixture rather than the code is what
    produces the degenerate fit. That mistake is preserved in
    `test_a_ladder_that_never_reaches_the_level_is_refused`.

    A large `sharpness_per_mm` gives the clean step a low-noise simulator produces.
    """
    rng = rng or np.random.default_rng(0)
    heights, successes = [], []
    for mm in heights_mm:
        p = float(logistic(np.array(sharpness_per_mm * (cutoff_mm - mm))))
        for _ in range(per_height):
            heights.append(mm * 1e-3)
            successes.append(bool(rng.random() < p))
    return np.array(heights), np.array(successes)


def true_height_m(cutoff_mm, sharpness_per_mm=0.15, level=SUCCESS_LEVEL):
    """Where the fixture's own curve crosses ``level``. The answer a fit has to recover."""
    return (cutoff_mm - np.log(level / (1.0 - level)) / sharpness_per_mm) * 1e-3


class TestThresholdFit(unittest.TestCase):
    HEIGHTS = tuple(range(10, 110, 10))
    CLEAN = 10.0          # sharpness that makes the ladder effectively a step function

    def test_it_recovers_a_known_crossing(self):
        """The check from outside the fit: the fixture's curve has an analytic P=0.9 height,
        and the fit has to land on it. Everything else here is a shape test; this one has a
        right answer."""
        h, y = ladder(self.HEIGHTS, 40, 55.0, rng=np.random.default_rng(7))
        fit = fit_threshold(h, y)
        self.assertTrue(fit.ok, fit.summary())
        self.assertAlmostEqual(fit.height_m, true_height_m(55.0), delta=0.006)
        self.assertLess(fit.stderr_m, 0.010)
        self.assertEqual(fit.n_runs, 400)
        self.assertEqual(fit.n_heights, 10)

    def test_the_truth_is_inside_the_error_bar(self):
        """Across independent draws, roughly two standard errors should cover it. Loose on
        purpose -- this asserts the uncertainty is the right order, not that it is exact."""
        covered = 0
        for seed in range(20):
            fit = fit_threshold(*ladder(self.HEIGHTS, 16, 55.0,
                                        rng=np.random.default_rng(seed)))
            if fit.ok and abs(fit.height_m - true_height_m(55.0)) < 2.5 * fit.stderr_m:
                covered += 1
        self.assertGreaterEqual(covered, 16)

    def test_it_is_continuous_where_a_bisection_would_jump(self):
        """The whole reason this metric exists. One flipped run out of four hundred moves the
        reported height by a fraction of a ladder step, not by a whole one."""
        h, y = ladder(self.HEIGHTS, 40, 55.0, rng=np.random.default_rng(3))
        before = fit_threshold(h, y).height_m
        y_flipped = y.copy()
        y_flipped[200] = not y_flipped[200]
        after = fit_threshold(h, y_flipped).height_m
        self.assertLess(abs(after - before), 0.005)
        self.assertNotAlmostEqual(after, before)

    def test_perfect_separation_is_reported_and_not_fitted(self):
        """A clean simulator produces exactly this, and it is where an unguarded fit returns
        a finite confident slope that is an artefact of the iteration cap."""
        h, y = ladder(self.HEIGHTS, 8, 55.0, sharpness_per_mm=self.CLEAN)
        fit = fit_threshold(h, y)
        self.assertTrue(fit.separated)
        self.assertFalse(fit.ok)
        self.assertEqual(fit.stderr_m, float("inf"))
        # The bracket is the gap the ladder actually resolved: 50 mm cleared, 60 mm not.
        self.assertAlmostEqual(fit.height_m, 0.055)

    def test_all_successes_is_censored_below(self):
        h, y = ladder(self.HEIGHTS, 4, 1000.0, sharpness_per_mm=self.CLEAN)
        fit = fit_threshold(h, y)
        self.assertEqual(fit.censored, "below")
        self.assertFalse(fit.ok)
        self.assertAlmostEqual(fit.height_m, 0.100)

    def test_all_failures_is_censored_above(self):
        h, y = ladder(self.HEIGHTS, 4, 0.0, sharpness_per_mm=self.CLEAN)
        fit = fit_threshold(h, y)
        self.assertEqual(fit.censored, "above")
        self.assertFalse(fit.ok)
        self.assertAlmostEqual(fit.height_m, 0.010)

    def test_success_rising_with_height_is_flagged_not_extrapolated(self):
        """Physically nonsense, so it means the predicate or the ladder is wrong. Reporting
        a crossing from it would give a step height unrelated to anything."""
        h, y = ladder(self.HEIGHTS, 8, 55.0, sharpness_per_mm=self.CLEAN)
        fit = fit_threshold(h, ~y.astype(bool))
        self.assertTrue(fit.separated)
        self.assertTrue(fit.inverted)
        self.assertFalse(fit.ok)

    def test_a_gently_inverted_ladder_is_flagged_by_the_slope(self):
        """Not separated -- there are contradicting runs -- so this reaches the fit and has
        to be caught by the sign of the slope rather than by the separation check."""
        h, y = ladder(self.HEIGHTS, 40, 55.0, rng=np.random.default_rng(4))
        fit = fit_threshold(h, ~y.astype(bool))
        self.assertFalse(fit.separated)
        self.assertTrue(fit.inverted)
        self.assertFalse(fit.ok)

    def test_a_ladder_too_thin_to_locate_a_crossing_is_refused(self):
        """The literal reproducer for **77.2 +/- 46029.6 mm** — a 46-metre standard error on
        a 77-millimetre answer, reported as a clean fit.

        Three rungs and two seeds, from the first S1 smoke test. Every earlier check passes:
        finite, converged, not separated, not censored, slope the right sign. Nothing about
        the number says it is worthless; only its size against the ladder does.
        """
        h = np.array([0.04, 0.08, 0.12, 0.04, 0.08, 0.12])
        y = np.array([True, True, False, True, False, False])
        fit = fit_threshold(h, y)
        self.assertFalse(fit.separated)
        self.assertFalse(fit.censored)
        self.assertFalse(fit.inverted)
        self.assertLess(fit.slope_per_m, 0.0)
        self.assertTrue(np.isfinite(fit.height_m))
        # ...and it is still not a measurement.
        self.assertGreater(fit.stderr_m, fit.ladder_span_m)
        self.assertFalse(fit.ok)
        self.assertIn("standard error", fit.reason)

    def test_a_crossing_outside_the_rungs_is_refused(self):
        """Extrapolation from a curve fitted entirely elsewhere. The honest response is a
        better ladder, and `reason` says so rather than the fit quietly reporting a height
        no run ever visited."""
        h, y = ladder(self.HEIGHTS, 200, 5.0, rng=np.random.default_rng(0))
        fit = fit_threshold(h, y)
        self.assertLess(fit.height_m, fit.low_rung_m)
        self.assertFalse(fit.ok)
        self.assertIn("outside the rungs", fit.reason)

    def test_a_taller_capable_wheel_reports_a_taller_height(self):
        """Monotonicity in the thing being measured — the property an optimiser needs."""
        weak = fit_threshold(*ladder(self.HEIGHTS, 40, 35.0, rng=np.random.default_rng(11)))
        strong = fit_threshold(*ladder(self.HEIGHTS, 40, 75.0, rng=np.random.default_rng(11)))
        self.assertTrue(weak.ok, weak.summary())
        self.assertTrue(strong.ok, strong.summary())
        self.assertGreater(strong.height_m, weak.height_m)

    def test_more_runs_shrink_the_error_bar(self):
        wide = fit_threshold(*ladder(self.HEIGHTS, 8, 55.0, rng=np.random.default_rng(5)))
        narrow = fit_threshold(*ladder(self.HEIGHTS, 128, 55.0, rng=np.random.default_rng(5)))
        self.assertTrue(narrow.ok, narrow.summary())
        self.assertLess(narrow.stderr_m, wide.stderr_m)

    def test_the_fitted_curve_passes_through_the_reported_point(self):
        """A check on the arithmetic rather than on the shape: evaluate the fitted logistic
        at the reported height and it must give back exactly the level asked for."""
        h, y = ladder(self.HEIGHTS, 40, 55.0, rng=np.random.default_rng(2))
        fit = fit_threshold(h, y)
        p = float(logistic(np.array(fit.intercept + fit.slope_per_m * fit.height_m)))
        self.assertAlmostEqual(p, SUCCESS_LEVEL, places=9)

    def test_the_level_is_configurable_and_ordered(self):
        h, y = ladder(self.HEIGHTS, 40, 55.0, rng=np.random.default_rng(2))
        lenient = fit_threshold(h, y, level=0.5).height_m
        strict = fit_threshold(h, y, level=0.99).height_m
        self.assertGreater(lenient, strict)

    def test_malformed_input_raises_but_degenerate_data_does_not(self):
        h, y = ladder(self.HEIGHTS, 4, 55.0, sharpness_per_mm=self.CLEAN)
        with self.assertRaises(ValueError):
            fit_threshold(h[:-1], y)
        with self.assertRaises(ValueError):
            fit_threshold(np.array([]), np.array([]))
        with self.assertRaises(ValueError):
            fit_threshold(h, np.full(h.shape, 0.5))
        with self.assertRaises(ValueError):
            fit_threshold(h, y, level=1.0)
        # Degenerate but real: returns a result.
        self.assertFalse(fit_threshold(h, np.ones_like(y)).ok)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
