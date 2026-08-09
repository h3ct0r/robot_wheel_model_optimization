"""Turning solver output into ROM parameters, against synthetic curves with known answers.

Every test here builds a load curve whose stiffness, limit point or loop area is known in
closed form, so a failure points at the extraction and not at the physics.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.cad.constraints import Severity
from wheelopt.cad.params import WheelParams
from wheelopt.fea.extract import (
    detect_buckling,
    fea_violations,
    loaded_radius,
    loop_area_fraction,
    spoke_stress,
)
from wheelopt.fea.loadcase import LoadCase
from wheelopt.fea.parse import parse_dat
from wheelopt.fea.results import ContactPatch, LoadCurve, common_force_n

PARAMS = WheelParams()


def curve_from(delta_load: np.ndarray, force_load: np.ndarray,
               force_unload: np.ndarray | None = None) -> LoadCurve:
    """Assemble a load/unload curve; unloading retraces unless told otherwise."""
    unload_d = delta_load[::-1]
    unload_f = force_load[::-1] if force_unload is None else force_unload
    return LoadCurve(
        delta_m=np.concatenate([delta_load, unload_d]),
        force_n=np.concatenate([force_load, unload_f]),
        loading=np.concatenate([
            np.ones(len(delta_load), bool), np.zeros(len(unload_d), bool)
        ]),
    )


class TestStiffness(unittest.TestCase):
    def test_linear_spring_gives_constant_stiffness(self):
        d = np.linspace(0.001, 0.010, 10)
        k_true = 5000.0
        c = curve_from(d, k_true * d)
        np.testing.assert_allclose(c.tangent_stiffness_n_per_m(), k_true, rtol=1e-9)
        np.testing.assert_allclose(c.secant_stiffness_n_per_m(), k_true, rtol=1e-9)

    def test_cubic_stiffening_matches_the_derivative(self):
        d = np.linspace(0.001, 0.010, 40)
        f = 1e4 * d + 5e7 * d**3
        c = curve_from(d, f)
        expected = 1e4 + 1.5e8 * d**2
        got = c.tangent_stiffness_n_per_m()
        # `np.gradient` is second-order accurate in the interior but only first-order at
        # the two ends. Worth knowing rather than smoothing over: the last sample is the
        # peak-load stiffness, which is a reported quantity, so it carries roughly an order
        # of magnitude more error than the rest of the curve.
        np.testing.assert_allclose(got[1:-1], expected[1:-1], rtol=2e-3)
        np.testing.assert_allclose(got, expected, rtol=2e-2)

    def test_endpoint_stiffness_is_less_accurate_than_the_interior(self):
        d = np.linspace(0.001, 0.010, 40)
        c = curve_from(d, 1e4 * d + 5e7 * d**3)
        expected = 1e4 + 1.5e8 * d**2
        error = np.abs(c.tangent_stiffness_n_per_m() - expected) / expected
        self.assertGreater(error[-1], error[len(error) // 2])

    def test_secant_and_tangent_differ_for_a_nonlinear_spring(self):
        d = np.linspace(0.001, 0.010, 20)
        c = curve_from(d, 5e7 * d**3)
        self.assertGreater(
            c.tangent_stiffness_n_per_m()[-1], c.secant_stiffness_n_per_m()[-1]
        )

    def test_peak_values(self):
        d = np.linspace(0.001, 0.010, 10)
        c = curve_from(d, 1000.0 * d)
        self.assertAlmostEqual(c.peak_delta_m, 0.010)
        self.assertAlmostEqual(c.peak_force_n, 10.0)


class TestBuckling(unittest.TestCase):
    def test_monotonic_curve_shows_no_limit_point(self):
        d = np.linspace(0.001, 0.010, 20)
        detected, load = detect_buckling(curve_from(d, 5000.0 * d))
        self.assertFalse(detected)
        self.assertIsNone(load)

    def test_negative_tangent_stiffness_is_a_limit_point(self):
        d = np.linspace(0.001, 0.010, 20)
        f = 5000.0 * d
        f[12:] = f[12] - 2000.0 * (d[12:] - d[12])  # softening branch
        detected, load = detect_buckling(curve_from(d, f))
        self.assertTrue(detected)
        self.assertIsNotNone(load)
        self.assertLessEqual(load, f.max() * 1.05)

    def test_short_curve_does_not_crash(self):
        d = np.array([0.001, 0.002])
        self.assertEqual(detect_buckling(curve_from(d, 1000.0 * d)), (False, None))

    def test_a_plateau_is_buckling_even_with_a_positive_tangent(self):
        """The case the sign test missed and the reason this is a ratio.

        Tangent 12.1 N/mm, collapsing to +0.086, recovering to 10.0 — the nominal design's
        shape from the log. Strictly positive throughout, so ``dF/ddelta < 0`` reports
        nothing, while the structure is carrying 4 mm of extra indentation at constant load.
        """
        d = np.linspace(0.001, 0.012, 12)
        tangent = np.array([12.1, 12.1, 8.0, 0.086, 0.086, 0.086, 0.086,
                            2.0, 6.0, 10.0, 10.0, 10.0]) * 1e3
        f = np.concatenate([[0.0], np.cumsum(tangent[1:] * np.diff(d))])
        self.assertTrue(bool(np.all(np.gradient(f, d) > 0.0)))
        detected, load = detect_buckling(curve_from(d, f))
        self.assertTrue(detected)
        self.assertIsNotNone(load)

    def test_a_stiffening_curve_is_never_flagged_however_it_starts(self):
        """The other side. A curve whose tangent only ever rises has no dip to find, so the
        ratio test must not invent one — this is the tiny design's measured shape."""
        d = np.linspace(0.001, 0.006, 6)
        tangent = np.array([0.70, 0.70, 0.81, 1.22, 2.21, 2.91]) * 1e3
        f = np.concatenate([[0.0], np.cumsum(tangent[1:] * np.diff(d))])
        self.assertEqual(detect_buckling(curve_from(d, f)), (False, None))

    def test_the_first_sample_is_neither_tested_nor_used_as_the_reference(self):
        """Before contact closes dF/ddelta is meaningless. A spuriously huge first tangent
        would otherwise become the yardstick and flag every later sample as collapsed."""
        d = np.linspace(0.001, 0.010, 10)
        f = 1000.0 * d
        f[0] *= 40.0  # a contact-closure spike at the first point
        self.assertEqual(detect_buckling(curve_from(d, f)), (False, None))


class TestLoopArea(unittest.TestCase):
    def test_retraced_curve_encloses_nothing(self):
        d = np.linspace(0.001, 0.010, 20)
        self.assertAlmostEqual(loop_area_fraction(curve_from(d, 5000.0 * d)), 0.0, places=9)

    def test_a_lower_unloading_branch_encloses_area(self):
        d = np.linspace(0.001, 0.010, 20)
        f = 5000.0 * d
        c = curve_from(d, f, force_unload=0.8 * f[::-1])
        self.assertGreater(loop_area_fraction(c), 0.15)

    def test_zero_work_is_handled(self):
        d = np.linspace(0.001, 0.010, 10)
        self.assertEqual(loop_area_fraction(curve_from(d, np.zeros_like(d))), 0.0)

    def test_branches_sampled_over_different_ranges_still_retrace(self):
        """The branches rarely share sample points: loading may start at 2 mm while
        unloading runs down to 0. Integrating each over its own range compares areas under
        different intervals and reports a huge loop for a curve that retraces exactly."""
        load_d = np.array([0.002, 0.004])
        unload_d = np.array([0.002, 0.0])
        k = 1000.0
        curve = LoadCurve(
            delta_m=np.concatenate([load_d, unload_d]),
            force_n=np.concatenate([k * load_d, k * unload_d]),
            loading=np.array([True, True, False, False]),
        )
        self.assertLess(loop_area_fraction(curve), 1e-6)


class TestLoadedRadius(unittest.TestCase):
    def test_decreases_with_indentation(self):
        d = np.linspace(0.001, 0.010, 10)
        r = loaded_radius(curve_from(d, 1000.0 * d), PARAMS)
        loading = r[: len(d)]
        self.assertTrue(bool(np.all(np.diff(loading) < 0)))

    def test_equals_free_radius_minus_indentation(self):
        d = np.array([0.005])
        r = loaded_radius(curve_from(d, np.array([10.0])), PARAMS)
        self.assertAlmostEqual(float(r[0]), PARAMS.outer_radius_mm * 1e-3 - 0.005)


class TestSpokeStress(unittest.TestCase):
    def test_von_mises_of_uniaxial_stress_is_the_axial_stress(self):
        dat = (
            "stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set ESPOKES"
            " and time  0.1000000E+01\n"
            "   1  1  1.000000E+06  0.0  0.0  0.0  0.0  0.0\n"
        )
        peak, p95 = spoke_stress(parse_dat(dat))
        self.assertAlmostEqual(peak, 1.0e6, places=3)
        self.assertAlmostEqual(p95, 1.0e6, places=3)

    def test_pure_shear_von_mises_is_sqrt3_tau(self):
        dat = (
            "stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set ESPOKES"
            " and time  0.1000000E+01\n"
            "   1  1  0.0 0.0 0.0  1.000000E+06  0.0  0.0\n"
        )
        peak, _ = spoke_stress(parse_dat(dat))
        self.assertAlmostEqual(peak, np.sqrt(3.0) * 1.0e6, places=1)

    def test_hydrostatic_stress_has_no_von_mises(self):
        dat = (
            "stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set ESPOKES"
            " and time  0.1000000E+01\n"
            "   1  1  5.0E5 5.0E5 5.0E5 0.0 0.0 0.0\n"
        )
        peak, _ = spoke_stress(parse_dat(dat))
        self.assertAlmostEqual(peak, 0.0, places=3)

    def test_peak_exceeds_p95_when_one_point_is_singular(self):
        rows = "".join(
            f"   1  {i}  1.000000E+06 0.0 0.0 0.0 0.0 0.0\n" for i in range(1, 40)
        )
        rows += "   1  40  9.000000E+07 0.0 0.0 0.0 0.0 0.0\n"
        header = (
            "stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set ESPOKES"
            " and time  0.1000000E+01\n"
        )
        peak, p95 = spoke_stress(parse_dat(header + rows))
        self.assertGreater(peak, p95 * 10)

    def test_missing_stress_block_returns_none(self):
        self.assertEqual(spoke_stress([]), (None, None))

    def test_stress_is_taken_at_peak_load_not_at_the_last_sample(self):
        """The sweep ends unloaded, so the final block is a wheel carrying nothing.

        Reading it reports ~0 MPa and every fatigue constraint passes.
        """
        header = (
            "stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set ESPOKES"
            " and time  {t}\n"
        )
        dat = (
            header.format(t="0.5000000E+00") + "   1  1  5.0E5 0.0 0.0 0.0 0.0 0.0\n"
            + header.format(t="0.1000000E+01") + "   1  1  4.0E6 0.0 0.0 0.0 0.0 0.0\n"
            + header.format(t="0.2000000E+01") + "   1  1  1.0E2 0.0 0.0 0.0 0.0 0.0\n"
        )
        peak, _ = spoke_stress(parse_dat(dat))
        self.assertAlmostEqual(peak, 4.0e6, places=1)


class TestViolations(unittest.TestCase):
    def _curve(self, k=5000.0, dmax=0.012):
        d = np.linspace(0.0005, dmax, 25)
        return curve_from(d, k * d)

    def test_a_healthy_design_produces_none(self):
        case = LoadCase(nominal_load_n=14.0, max_load_multiple=3.0)
        v = fea_violations(self._curve(k=20000.0), case, PARAMS, 1.0e6, None)
        self.assertEqual(v, [])

    def test_a_sweep_that_never_touched_is_degenerate(self):
        """Measured, not hypothetical: a bandless wheel phased so the indenter descends
        into the gap between two tips converges cleanly and reports 0.00 N over the whole
        sweep. Without a typed violation that reads as an infinitely compliant wheel."""
        case = LoadCase(nominal_load_n=14.0)
        v = fea_violations(self._curve(k=0.0), case, PARAMS, 1.0e6, None)
        names = [x.name for x in v]
        self.assertIn("fea_no_contact", names)
        self.assertIs(
            next(x for x in v if x.name == "fea_no_contact").severity, Severity.DEGENERATE
        )
        # A missed load case says nothing about sag, stress or buckling, so it must not
        # also emit the derived checks — they would all be reading zeros.
        self.assertEqual(names, ["fea_no_contact"])

    def test_a_grazing_touch_is_not_reported_as_a_miss(self):
        case = LoadCase(nominal_load_n=14.0)
        names = [x.name for x in fea_violations(self._curve(k=1.0), case, PARAMS, 1.0e6, None)]
        self.assertNotIn("fea_no_contact", names)
        self.assertIn("fea_load_range", names)

    def test_short_sweep_warns_about_extrapolation(self):
        case = LoadCase(nominal_load_n=14.0, max_load_multiple=3.0)
        v = fea_violations(self._curve(k=100.0), case, PARAMS, 1.0e6, None)
        names = [x.name for x in v]
        self.assertIn("fea_load_range", names)
        warn = next(x for x in v if x.name == "fea_load_range")
        self.assertIs(warn.severity, Severity.WARNING)

    def test_excess_sag_is_infeasible(self):
        """A very soft wheel squashes past 85% of its free radius at nominal load."""
        case = LoadCase(nominal_load_n=14.0)
        curve = self._curve(k=800.0, dmax=0.030)
        v = fea_violations(curve, case, PARAMS, 1.0e6, None)
        self.assertIn("fea_static_sag", [x.name for x in v])

    def test_stress_above_the_fatigue_limit_is_infeasible(self):
        case = LoadCase(nominal_load_n=14.0)
        v = fea_violations(self._curve(k=20000.0), case, PARAMS, 9.0e6, None)
        found = next(x for x in v if x.name == "fea_peak_stress")
        self.assertIs(found.severity, Severity.INFEASIBLE)
        self.assertLess(found.margin, 0)

    def test_early_buckling_is_infeasible(self):
        case = LoadCase(nominal_load_n=14.0)
        v = fea_violations(self._curve(k=20000.0), case, PARAMS, 1.0e6, buckling_load_n=20.0)
        self.assertIn("fea_buckling", [x.name for x in v])

    def test_late_buckling_is_acceptable(self):
        case = LoadCase(nominal_load_n=14.0)
        v = fea_violations(self._curve(k=20000.0), case, PARAMS, 1.0e6, buckling_load_n=200.0)
        self.assertNotIn("fea_buckling", [x.name for x in v])

    def test_violations_compose_with_the_cad_feasibility_check(self):
        """The reason for reusing `Violation` rather than inventing an FEA-specific type."""
        from wheelopt.cad.constraints import is_feasible

        case = LoadCase(nominal_load_n=14.0)
        healthy = fea_violations(self._curve(k=20000.0), case, PARAMS, 1.0e6, None)
        broken = fea_violations(self._curve(k=20000.0), case, PARAMS, 9.0e6, None)
        self.assertTrue(is_feasible(healthy))
        self.assertFalse(is_feasible(broken))

    def test_margins_are_negative_exactly_when_violated(self):
        case = LoadCase(nominal_load_n=14.0)
        for v in fea_violations(self._curve(k=20000.0), case, PARAMS, 9.0e6, 20.0):
            with self.subTest(name=v.name):
                if v.severity is Severity.INFEASIBLE:
                    self.assertLess(v.margin, 0)


class TestContactPatchPressure(unittest.TestCase):
    """Mean pressure and the equal-load comparison.

    ``peak_pressure_pa`` is the largest nodal value on a node-set slave surface, so it
    tracks how many nodes are in contact rather than how hard they are pressed. The
    verification battery compared two load cases on it and produced a wrong verdict; these
    tests pin down the measure that replaced it.
    """

    @staticmethod
    def patch(force_n, area_m2, peak_pa=None) -> ContactPatch:
        force_n = np.asarray(force_n, dtype=float)
        area_m2 = np.asarray(area_m2, dtype=float)
        return ContactPatch(
            force_n=force_n,
            length_m=np.sqrt(area_m2),
            width_m=np.sqrt(area_m2),
            area_m2=area_m2,
            peak_pressure_pa=np.asarray(
                peak_pa if peak_pa is not None else np.zeros_like(force_n), dtype=float
            ),
            n_nodes=np.ones_like(force_n, dtype=np.int64),
        )

    def test_mean_pressure_is_force_over_area(self):
        p = self.patch([2.0, 6.0], [1e-4, 2e-4])
        np.testing.assert_allclose(p.mean_pressure_pa, [2e4, 3e4])

    def test_zero_area_does_not_divide_by_zero(self):
        # A time point where contact was detected but no node cleared the hot threshold.
        p = self.patch([1.0, 2.0], [0.0, 1e-4])
        np.testing.assert_allclose(p.mean_pressure_pa, [0.0, 2e4])

    def test_at_force_interpolates(self):
        p = self.patch([1.0, 3.0], [1e-4, 3e-4])
        area, pressure = p.at_force(2.0)
        self.assertAlmostEqual(area, 2e-4)
        self.assertAlmostEqual(pressure, 1e4)  # both ends are exactly 10 kPa

    def test_at_force_clamps_rather_than_extrapolates(self):
        # Extrapolating past the last converged increment would invent a contact patch.
        p = self.patch([1.0, 3.0], [1e-4, 3e-4])
        self.assertAlmostEqual(p.at_force(99.0)[0], 3e-4)
        self.assertAlmostEqual(p.at_force(0.0)[0], 1e-4)

    def test_at_force_survives_an_empty_patch(self):
        self.assertEqual(self.patch([], [])
                         .at_force(1.0), (0.0, 0.0))

    def test_common_force_finds_the_shared_ceiling(self):
        a = self.patch([1.0, 5.0], [1e-4, 5e-4])
        b = self.patch([2.0, 3.0], [2e-4, 3e-4])
        self.assertAlmostEqual(common_force_n(a, b), 3.0)

    def test_disjoint_sweeps_have_no_common_load(self):
        # The case that matters: contact output starts only once nodes touch, so a stiff
        # design's first sample can sit above a soft design's last one. Comparing them by
        # clamping would report a ratio between two states neither solve ever visited.
        soft = self.patch([0.4, 4.4], [5e-5, 5.6e-4])
        stiff = self.patch([13.5, 26.9], [1.8e-4, 4.6e-5])
        self.assertIsNone(common_force_n(soft, stiff))

    def test_common_force_of_an_empty_patch(self):
        self.assertIsNone(common_force_n(self.patch([], []), self.patch([1.0], [1e-4])))

    def test_at_force_clamping_is_why_common_force_exists(self):
        # Demonstrates the trap the guard prevents: asking for a load below the sweep
        # returns the sweep's own first sample, which looks like a legitimate answer.
        stiff = self.patch([13.5, 26.9], [1.8e-4, 4.6e-5])
        area, _ = stiff.at_force(4.4)
        self.assertAlmostEqual(area, 1.8e-4)  # the 13.5 N value, not a 4.4 N value

    def test_force_range_reports_the_sampled_span(self):
        self.assertEqual(self.patch([13.5, 26.9], [1e-4, 2e-4]).force_range_n, (13.5, 26.9))
        self.assertEqual(self.patch([], []).force_range_n, (0.0, 0.0))

    def test_the_equal_load_comparison_reverses_the_equal_delta_one(self):
        # The real numbers from the verification wheel, --tiny preset. At equal
        # indentation the flat plate is at 4.36 N and the step edge at 3.04 N; comparing
        # the last sample of each made the step edge look *lower* pressure. At the common
        # load it is higher, which is the physical claim.
        flat = self.patch([2.642, 4.356], [464.5e-6, 562.3e-6], [1020.6e3, 979.6e3])
        edge = self.patch([2.078, 3.041], [244.5e-6, 317.8e-6], [821.5e3, 863.8e3])

        self.assertLess(edge.peak_pressure_pa[-1], flat.peak_pressure_pa[-1])

        common = min(float(flat.force_n.max()), float(edge.force_n.max()))
        flat_area, flat_pressure = flat.at_force(common)
        edge_area, edge_pressure = edge.at_force(common)
        self.assertLess(edge_area, flat_area)
        self.assertGreater(edge_pressure, flat_pressure)


if __name__ == "__main__":
    unittest.main()
