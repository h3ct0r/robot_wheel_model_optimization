"""Constraint pre-filter behaviour.

The behavioural contract (invariant 3): screening never raises, and always returns a typed
violation vector. These tests exist mainly to defend that contract — a constraint check
that raises will kill a 40-hour campaign at hour 39.
"""

from __future__ import annotations

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
