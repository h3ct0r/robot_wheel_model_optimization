"""Constraint pre-filter behaviour.

The behavioural contract (invariant 3): screening never raises, and always returns a typed
violation vector. These tests exist mainly to defend that contract — a constraint check
that raises will kill a 40-hour campaign at hour 39.
"""

from __future__ import annotations

import math
import unittest
from dataclasses import replace

from wheelopt.cad.constraints import (
    PlatformLimits,
    Severity,
    check_design,
    is_feasible,
)
from wheelopt.cad.materials import PLA, TPU95A, MaterialSpec
from wheelopt.cad.params import SpokeProfile, WheelParams

LIMITS = PlatformLimits()
NOMINAL = WheelParams()
TPU = TPU95A


def names(violations) -> set[str]:
    return {v.name for v in violations}


class TestNominalDesign(unittest.TestCase):
    def test_default_design_is_feasible(self):
        v = check_design(WheelParams(), TPU95A, LIMITS)
        self.assertTrue(is_feasible(v), f"unexpected violations: {[str(x) for x in v]}")

    def test_default_design_has_no_hard_violations(self):
        v = check_design(WheelParams(), TPU95A, LIMITS)
        hard = [x for x in v if x.severity is not Severity.WARNING]
        self.assertEqual(hard, [])


class TestDegenerateGeometry(unittest.TestCase):
    def test_hub_reaching_rim_is_degenerate(self):
        p = WheelParams(outer_radius_mm=50.0, rim_thickness_mm=3.0, hub_radius_mm=48.0)
        v = check_design(p, TPU95A, LIMITS)
        self.assertIn("spoke_span", names(v))
        self.assertFalse(is_feasible(v))

    def test_bore_larger_than_hub_is_degenerate(self):
        p = WheelParams(hub_radius_mm=5.0, hub_bore_radius_mm=6.0)
        self.assertIn("hub_bore", names(check_design(p, TPU95A, LIMITS)))

    def test_too_few_spokes_is_degenerate(self):
        self.assertIn("n_spokes", names(check_design(WheelParams(n_spokes=2), TPU95A, LIMITS)))

    def test_extreme_curvature_folds_spoke(self):
        """Sagitta grows as span squared, so a long spoke folds at modest curvature.

        Fold condition is ``kappa * L / 4 > 1``: at the curvature bound of 0.03 /mm that
        needs a span over ~133 mm, which only the largest wheels in the space can reach.
        """
        p = WheelParams(
            outer_radius_mm=150.0,
            rim_thickness_mm=1.2,
            hub_radius_mm=5.0,
            hub_bore_radius_mm=3.0,
            spoke_curvature_1_per_mm=0.03,
        )
        self.assertGreater(p.spoke_span_mm, 133.0)
        self.assertIn("spoke_sagitta", names(check_design(p, TPU95A, LIMITS)))

    def test_degenerate_design_skips_geometry_dependent_checks(self):
        """Zero spokes must be rejected, not crash the geometric gap query.

        Regression test for invariant 3: a constraint check that raises kills the whole
        campaign rather than rejecting one design.
        """
        v = check_design(WheelParams(n_spokes=0), TPU95A, LIMITS)
        self.assertIn("n_spokes", names(v))
        self.assertNotIn("interspoke_gap", names(v))

    def test_degenerate_severity_is_marked(self):
        # Hub swallows the shear band, so there is no span left for spokes. Derived from
        # the default rather than hard-coded, so it stays degenerate if the platform moves.
        p = WheelParams(hub_radius_mm=WheelParams().rim_inner_radius_mm + 2.0)
        v = check_design(p, TPU95A, LIMITS)
        self.assertTrue(any(x.severity is Severity.DEGENERATE for x in v))


class TestManufacturability(unittest.TestCase):
    def test_tpu_enforces_thicker_walls_than_rigid(self):
        """1.4 mm passes for PLA and fails for TPU — flexible thin walls print badly."""
        p = WheelParams(spoke_thickness_mm=1.4, rim_thickness_mm=2.0)
        self.assertNotIn("spoke_min_wall", names(check_design(p, PLA, LIMITS)))
        self.assertIn("spoke_min_wall", names(check_design(p, TPU95A, LIMITS)))

    def test_crowded_spokes_violate_nozzle_clearance(self):
        p = WheelParams(n_spokes=36, spoke_thickness_mm=4.0, hub_radius_mm=25.0)
        v = check_design(p, TPU95A, LIMITS)
        self.assertIn("interspoke_gap", names(v))

    def test_gap_violation_reports_the_true_geometric_value(self):
        """The reported value must be the measured clearance, not an approximation."""
        from wheelopt.cad.centreline import min_gap_between_spokes

        p = WheelParams(n_spokes=34, spoke_thickness_mm=3.8)
        v = [x for x in check_design(p, TPU95A, LIMITS) if x.name == "interspoke_gap"]
        self.assertEqual(len(v), 1)
        self.assertAlmostEqual(v[0].value, min_gap_between_spokes(p), places=9)

    def test_tread_cutting_through_shear_band(self):
        p = WheelParams(rim_thickness_mm=2.0, tread_depth_mm=2.5)
        self.assertIn("tread_depth", names(check_design(p, TPU95A, LIMITS)))

    def test_oversized_wheel_fails_bed_and_envelope(self):
        p = WheelParams(outer_radius_mm=149.0)
        n = names(check_design(p, TPU95A, PlatformLimits(bed_size_mm=(180.0, 180.0, 180.0))))
        self.assertIn("print_bed", n)


class TestEnvelope(unittest.TestCase):
    def test_wheel_too_large_for_well(self):
        lim = PlatformLimits(wheel_well_radius_mm=60.0)
        self.assertIn("envelope_radius", names(check_design(WheelParams(), TPU95A, lim)))

    def test_wheel_too_wide(self):
        lim = PlatformLimits(max_width_mm=30.0)
        self.assertIn("envelope_width", names(check_design(WheelParams(), TPU95A, lim)))

    def test_bore_must_match_shaft(self):
        lim = PlatformLimits(shaft_radius_mm=5.0)
        self.assertIn(
            "shaft_fit", names(check_design(WheelParams(hub_bore_radius_mm=3.0), TPU95A, lim))
        )


class TestSearchBounds(unittest.TestCase):
    def test_radius_below_bound(self):
        p = WheelParams(outer_radius_mm=20.0, hub_radius_mm=8.0, rim_thickness_mm=2.0)
        self.assertIn("bounds_outer_radius_mm", names(check_design(p, TPU95A, LIMITS)))

    def test_curvature_outside_bound(self):
        p = WheelParams(spoke_curvature_1_per_mm=0.5)
        self.assertIn("bounds_spoke_curvature_1_per_mm", names(check_design(p, TPU95A, LIMITS)))


class TestWarnings(unittest.TestCase):
    def test_straight_spoke_warns_but_stays_feasible(self):
        p = WheelParams(spoke_profile=SpokeProfile.STRAIGHT)
        v = check_design(p, TPU95A, LIMITS)
        self.assertIn("straight_spoke_buckling", names(v))
        self.assertTrue(is_feasible(v), "a warning must not make a design infeasible")

    def test_slender_spoke_warns(self):
        p = WheelParams(spoke_thickness_mm=1.6, hub_radius_mm=12.0, outer_radius_mm=100.0)
        self.assertIn("spoke_slenderness", names(check_design(p, TPU95A, LIMITS)))

    def test_ineffective_infill_warns(self):
        """A 2.0 mm spoke with 3 walls prints solid; infill_density is a no-op."""
        mat = MaterialSpec(name="TPU_95A", infill_density=0.2, wall_count=3)
        p = WheelParams(spoke_thickness_mm=2.0)
        self.assertIn("infill_ineffective", names(check_design(p, mat, LIMITS)))

    def test_thick_spoke_does_not_warn_about_infill(self):
        mat = MaterialSpec(name="TPU_95A", infill_density=0.2, wall_count=2)
        p = WheelParams(spoke_thickness_mm=4.0)
        self.assertNotIn("infill_ineffective", names(check_design(p, mat, LIMITS)))


class TestContract(unittest.TestCase):
    """Invariant 3: screening never raises, whatever it is handed."""

    def test_never_raises_on_pathological_inputs(self):
        pathological = [
            WheelParams(outer_radius_mm=0.0),
            WheelParams(width_mm=0.0),
            WheelParams(n_spokes=0),
            WheelParams(n_spokes=1),
            WheelParams(spoke_thickness_mm=0.0),
            WheelParams(hub_radius_mm=0.0, hub_bore_radius_mm=0.0),
            WheelParams(rim_thickness_mm=1e6),
            WheelParams(outer_radius_mm=-10.0),
            WheelParams(spoke_curvature_1_per_mm=1e6),
            WheelParams(hub_radius_mm=1e-9, spoke_samples=3),
        ]
        for i, p in enumerate(pathological):
            with self.subTest(case=i):
                try:
                    result = check_design(p, TPU95A, LIMITS)
                except Exception as exc:  # noqa: BLE001 - that is the point of the test
                    self.fail(f"check_design raised {type(exc).__name__}: {exc}")
                self.assertIsInstance(result, list)

    def test_violation_margin_is_negative_when_violated(self):
        lim = PlatformLimits(wheel_well_radius_mm=50.0)
        v = [x for x in check_design(WheelParams(), TPU95A, lim) if x.name == "envelope_radius"]
        self.assertEqual(len(v), 1)
        self.assertLess(v[0].margin, 0.0)

    def test_is_feasible_ignores_warnings_only(self):
        self.assertTrue(is_feasible([]))
        v = check_design(WheelParams(spoke_profile=SpokeProfile.STRAIGHT), TPU95A, LIMITS)
        self.assertTrue(all(x.severity is Severity.WARNING for x in v))
        self.assertTrue(is_feasible(v))

    def test_screening_is_fast(self):
        import time

        p = WheelParams()
        check_design(p, TPU95A, LIMITS)
        start = time.perf_counter()
        for _ in range(200):
            check_design(p, TPU95A, LIMITS)
        per_call_ms = (time.perf_counter() - start) / 200 * 1e3
        self.assertLess(per_call_ms, 5.0, f"screening took {per_call_ms:.2f} ms/design")


if __name__ == "__main__":
    unittest.main()


class TestClawTaperScreening(unittest.TestCase):
    """A taper adds a second thickness, and every check has to read the right one."""

    def _named(self, params, name):
        return [v for v in check_design(params, TPU) if v.name == name]

    def test_the_min_wall_check_reads_the_tip_not_the_root(self):
        # 7 mm of root looks comfortable against a 1.6 mm minimum wall; at 0.15 taper the
        # material that actually gets printed at the tip is 1.05 mm. A check written against
        # spoke_thickness_mm passes this design.
        thin_tip = replace(NOMINAL, rim_thickness_mm=0.0, spoke_thickness_mm=7.0,
                           claw_taper_ratio=0.15)
        self.assertEqual(thin_tip.spoke_thickness_mm, 7.0)
        self.assertAlmostEqual(thin_tip.tip_thickness_mm, 1.05, places=9)
        found = self._named(thin_tip, "spoke_min_wall")
        self.assertTrue(found, "an unprintable claw tip was not rejected")
        self.assertAlmostEqual(found[0].value, 1.05, places=9)

    def test_an_untapered_spoke_screens_exactly_as_before(self):
        uniform = replace(NOMINAL, claw_taper_ratio=1.0)
        self.assertEqual(uniform.tip_thickness_mm, uniform.spoke_thickness_mm)
        self.assertFalse(self._named(uniform, "spoke_min_wall"))

    def test_the_discrete_contact_warning_quotes_the_tip_width(self):
        # The patch is as wide as the material touching the ground. Quoting the root would
        # overstate it by 1/taper.
        claw = replace(NOMINAL, rim_thickness_mm=0.0, spoke_thickness_mm=8.0,
                       claw_taper_ratio=0.35)
        message = self._named(claw, "no_shear_band")[0].message
        self.assertIn("2.8 mm patches", message)
        self.assertNotIn("8.0 mm patches", message)

    def test_the_taper_bound_is_enforced(self):
        over = replace(NOMINAL, claw_taper_ratio=1.4)
        self.assertTrue(self._named(over, "bounds_claw_taper_ratio"))


class TestClawRideHarshness(unittest.TestCase):
    """TODO #19: how few tips a bandless wheel may have, and why the answer went up.

    Derived 2026-08-09 by measuring the ride-height ripple of a fitted ring at the platform's
    24.5 N (`wheelopt.rom.ring.ride_height_ripple_m`), on two claws an order of magnitude apart
    in stiffness. Both unload a claw completely once per pitch below 10-12 tips. So the
    pre-filter's job here is to flag the geometry no stiffness can rescue, and it is a warning
    rather than an infeasibility because the real criterion needs the fitted law.
    """

    def _named(self, params, name):
        return [v for v in check_design(params, TPU) if v.name == name]

    def test_the_polygon_drop_is_half_the_pitch_not_the_whole_one(self):
        """The two formulas differ by a factor of two inside a cosine and are easy to swap.
        This one is the axle's ride height as the wheel turns; ``R(1 - cos 2π/n)`` is how deep
        the wheel must indent for a *second* tip to reach the ground plane.
        """
        wheel = replace(NOMINAL, outer_radius_mm=85.0, n_spokes=12)
        self.assertAlmostEqual(wheel.polygon_drop_mm, 2.8963, places=4)
        engagement = 85.0 * (1.0 - math.cos(2.0 * math.pi / 12))
        self.assertAlmostEqual(engagement, 11.3878, places=4)
        self.assertGreater(engagement, 3.0 * wheel.polygon_drop_mm)

    def test_a_coarse_bandless_wheel_is_flagged(self):
        coarse = replace(NOMINAL, rim_thickness_mm=0.0, outer_radius_mm=85.0, n_spokes=6,
                         spoke_thickness_mm=6.0, claw_taper_ratio=0.6)
        found = self._named(coarse, "claw_ride_harshness")
        self.assertTrue(found)
        self.assertEqual(found[0].severity, Severity.WARNING)
        self.assertAlmostEqual(found[0].value, 0.1340, places=4)
        self.assertIn("11.4 mm", found[0].message)

    def test_twelve_tips_is_the_boundary(self):
        """n = 12 on any radius is a 3.4% drop, just inside the 3.5% threshold; n = 11 is
        4.1% and outside. The threshold is radius-free because the drop is a fraction of R."""
        for radius in (60.0, 85.0, 100.0):
            fine = replace(NOMINAL, rim_thickness_mm=0.0, outer_radius_mm=radius,
                           n_spokes=12, spoke_thickness_mm=6.0, claw_taper_ratio=0.6)
            coarse = replace(fine, n_spokes=11)
            with self.subTest(radius=radius):
                self.assertFalse(self._named(fine, "claw_ride_harshness"))
                self.assertTrue(self._named(coarse, "claw_ride_harshness"))

    def test_a_banded_wheel_is_never_flagged(self):
        """The check is about a polygon running surface. A band makes the wheel round, so six
        spokes is a perfectly good `T3` and must not inherit the claw family's limit."""
        banded = replace(NOMINAL, rim_thickness_mm=3.0, outer_radius_mm=85.0, n_spokes=6)
        self.assertTrue(banded.has_shear_band)
        self.assertFalse(self._named(banded, "claw_ride_harshness"))

    def test_it_is_a_warning_and_does_not_make_a_design_infeasible(self):
        coarse = replace(NOMINAL, rim_thickness_mm=0.0, outer_radius_mm=85.0, n_spokes=6,
                         spoke_thickness_mm=6.0, claw_taper_ratio=0.6)
        self.assertTrue(is_feasible(check_design(coarse, TPU)))


class TestTaperedSlenderness(unittest.TestCase):
    """TODO #21: the proxy must not read the root of a tapered claw.

    The root is the *stiffest* section, so a proxy reading it understates slenderness and
    errs toward accepting a claw that buckles — the non-conservative direction, and the one
    this project's watch list is about.
    """

    def _named(self, params, name):
        return [v for v in check_design(params, TPU) if v.name == name]

    def test_a_uniform_strut_is_unchanged(self):
        """`Φ(1) = 1` is a 0/0 limit in the closed form and must be taken explicitly, or
        every untapered spoke — most of the design space — gets a NaN thickness."""
        uniform = replace(NOMINAL, claw_taper_ratio=1.0, spoke_thickness_mm=6.0)
        self.assertEqual(uniform.effective_thickness_mm, 6.0)
        near = replace(uniform, claw_taper_ratio=0.999999)
        self.assertAlmostEqual(near.effective_thickness_mm, 6.0, places=5)

    def test_the_two_branches_agree_across_the_join(self):
        """The series is used below ``k = 0.1`` and the closed form above it, because the
        closed form is catastrophic cancellation near ``r = 1``: at ``r = 0.999999`` the true
        value is 1.00000075 and the direct expression returns **-166.5**, whose cube root in
        Python is complex. That is a NaN-shaped bug hiding behind the most common design in
        the space — an untapered spoke — so the branches are checked against each other here.
        """
        from wheelopt.cad.params import _taper_compliance_factor

        for r in (0.9 - 1e-12, 0.9, 0.9 + 1e-12):
            with self.subTest(r=r):
                self.assertAlmostEqual(_taper_compliance_factor(r), 1.0815469735, places=9)
        self.assertEqual(_taper_compliance_factor(1.0), 1.0)
        self.assertAlmostEqual(_taper_compliance_factor(0.999999), 1.00000075, places=8)
        # Strictly decreasing in r: less taper, less softening. A sign slip breaks this.
        values = [_taper_compliance_factor(r)
                  for r in (0.25, 0.4, 0.6, 0.8, 0.95, 0.99, 1.0)]
        self.assertEqual(values, sorted(values, reverse=True))

    def test_the_effective_thickness_lies_between_tip_and_root(self):
        """A weighted average of the section must be inside the sections it averages. It is
        not the arithmetic mean, which is the tempting wrong answer: at taper 0.4 the mean is
        0.70 of the root and this is 0.808."""
        for taper in (0.8, 0.6, 0.4, 0.25):
            claw = replace(NOMINAL, spoke_thickness_mm=8.0, claw_taper_ratio=taper)
            with self.subTest(taper=taper):
                self.assertLess(claw.effective_thickness_mm, claw.spoke_thickness_mm)
                self.assertGreater(claw.effective_thickness_mm, claw.tip_thickness_mm)
        mid = replace(NOMINAL, spoke_thickness_mm=8.0, claw_taper_ratio=0.4)
        self.assertAlmostEqual(mid.effective_thickness_mm / 8.0, 0.8084, places=4)
        self.assertNotAlmostEqual(mid.effective_thickness_mm, 0.5 * (8.0 + 3.2), places=2)

    def test_it_matches_the_derivation_at_the_tapers_that_were_measured(self):
        """Pinned against the closed form evaluated by hand, so a transcription slip in
        `Φ(r) = 3[-ln r + 2r - 3/2 - r²/2]/(1-r)³` cannot pass."""
        for taper, expected in ((0.6, 7.07629), (0.4, 6.46714), (0.25, 5.87512)):
            claw = replace(NOMINAL, spoke_thickness_mm=8.0, claw_taper_ratio=taper)
            with self.subTest(taper=taper):
                self.assertAlmostEqual(claw.effective_thickness_mm, expected, places=4)

    def test_a_taper_now_raises_the_reported_slenderness(self):
        """The behaviour change. Same root, same span; the tapered claw must screen as more
        slender than the uniform one, and previously screened as exactly the same."""
        span = 65.0
        uniform = replace(NOMINAL, rim_thickness_mm=0.0, outer_radius_mm=85.0,
                          hub_radius_mm=20.0, spoke_thickness_mm=1.7, claw_taper_ratio=1.0)
        self.assertAlmostEqual(uniform.spoke_span_mm, span, places=9)
        tapered = replace(uniform, claw_taper_ratio=0.6)
        self.assertAlmostEqual(uniform.spoke_span_mm / uniform.effective_thickness_mm,
                               38.24, places=2)
        self.assertAlmostEqual(tapered.spoke_span_mm / tapered.effective_thickness_mm,
                               43.23, places=2)
        # 40 is the threshold, so the taper is what tips this pair across it.
        self.assertFalse(self._named(uniform, "spoke_slenderness"))
        found = self._named(tapered, "spoke_slenderness")
        self.assertTrue(found)
        self.assertIn("root", found[0].message)
