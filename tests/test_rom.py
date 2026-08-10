"""The segmented ring: geometry, the fit, and the MuJoCo realisation.

The fit is a deconvolution, so the test that matters most is the round trip —
:class:`TestRoundTrip` generates a load curve from a known spring law and checks the fitter
recovers it. Everything else is guarding a specific way the model can be plausibly wrong.

MuJoCo tests are skipped without the simulator; the rest are pure numpy, which is the reason
`ring.py` and `fit.py` do not import it.
"""

from __future__ import annotations

import unittest
from dataclasses import replace
from types import MappingProxyType

import numpy as np

from wheelopt.cad.materials import MaterialSpec
from wheelopt.cad.params import WheelParams
from wheelopt.rom.fit import (
    FitFailure,
    contact_segments,
    fit_spring_law,
    fit_tabulated_law,
    hinge_kinematics_check,
    hinge_law_from_tip_curve,
    nnls,
    ring_from_claw_curve,
    validate_ring,
)
from wheelopt.rom.ring import (
    RadialLaw,
    RingSpec,
    SpringLaw,
    TabulatedLaw,
    TipEquivalentLaw,
    bending_coupling_n_per_m,
    coupling_matrix,
    curvature_operator,
    hoop_coupling_n_per_m,
    penetrations,
    polygon_drop_m,
    ramp_basis,
    ride_height_ripple_m,
    ring_for_design,
    ring_force_2dof_n,
    ring_force_hinge_n,
    ring_force_n,
    second_contact_delta_m,
    segment_angles,
    solve_equilibrium,
    solve_equilibrium_2dof,
    solve_equilibrium_hinge,
    symmetric_force_n,
    tip_radius_hinge_m,
    tip_radius_slide_m,
    uniform_knots,
)

try:
    import mujoco

    HAVE_MUJOCO = True
except ImportError:  # pragma: no cover - environment dependent
    HAVE_MUJOCO = False

SPEC = RingSpec(radius_m=0.060, n_segments=24)
#: The same ring with a band. 0.8 N/m is the order the tiny design actually produces, so the
#: coupled tests exercise the regime the project is in rather than a stiffness chosen to make
#: an effect visible.
COUPLED = replace(SPEC, band_bending_n_per_m=0.8, band_hoop_n_per_m=90.0)
LAW = SpringLaw(a=4000.0, b=2.0e5, c=1.0e8)
#: The `--tiny` design from `scripts/run_rom.py`, so the derived-coupling tests are anchored
#: to a wheel the FEA tier has actually solved rather than to invented dimensions.
TINY_PARAMS = WheelParams(outer_radius_mm=60.0, width_mm=30.0, n_spokes=6,
                          spoke_thickness_mm=5.0, rim_thickness_mm=3.0,
                          hub_radius_mm=20.0)
TPU = MaterialSpec(name="TPU_95A")


class TestRingSpec(unittest.TestCase):
    def test_rejects_degenerate_geometry(self):
        with self.assertRaises(ValueError):
            RingSpec(radius_m=0.0)
        with self.assertRaises(ValueError):
            RingSpec(radius_m=0.06, n_segments=2)

    def test_segment_arc_closes_the_ring(self):
        self.assertAlmostEqual(
            SPEC.segment_arc_m * SPEC.n_segments, 2 * np.pi * SPEC.radius_m
        )


class TestGeometry(unittest.TestCase):
    def test_segment_zero_is_at_the_contact_point(self):
        self.assertAlmostEqual(segment_angles(SPEC)[0], 0.0)

    def test_angles_are_wrapped_and_symmetric(self):
        theta = segment_angles(SPEC)
        self.assertTrue(bool(np.all(theta >= -np.pi)))
        self.assertTrue(bool(np.all(theta < np.pi)))
        # A ring is symmetric about the contact point, which is what makes F a function of
        # delta alone; if it were not, the flat-plate response would depend on phase.
        np.testing.assert_allclose(np.sort(theta), -np.sort(-theta)[::-1], atol=1e-12)

    def test_penetration_is_deepest_at_the_contact_point(self):
        u = penetrations(SPEC, 0.004)
        self.assertEqual(int(np.argmax(u)), 0)
        self.assertAlmostEqual(u[0], 0.004)

    def test_segments_out_of_reach_do_not_touch(self):
        u = penetrations(SPEC, 0.001)
        self.assertTrue(bool(np.all(u[3:-3] == 0.0)))

    def test_no_contact_at_zero_indentation(self):
        self.assertTrue(bool(np.all(penetrations(SPEC, 0.0) == 0.0)))

    def test_more_segments_engage_as_the_wheel_is_pressed(self):
        counts = [contact_segments(SPEC, d) for d in (0.001, 0.003, 0.006, 0.010)]
        self.assertEqual(counts, sorted(counts))


class TestSpringLaw(unittest.TestCase):
    def test_extension_is_linear_and_uses_the_tangent_at_the_origin(self):
        # This used to assert force_n(-u) == 0 -- "a segment cannot pull". That reading was
        # unreachable before coupling (penetrations never goes negative) and wrong after it:
        # the band pulls the segments beside the patch outward past R, and the spoke holding
        # one is anchored at both ends, so it resists. A limp branch is also singular -- see
        # the SpringLaw docstring.
        law = SpringLaw(a=1000.0, b=2.0e5, c=1.0e8)
        self.assertEqual(float(law.force_n(-0.005)), -5.0)
        self.assertEqual(float(law.force_n(0.0)), 0.0)

    def test_the_cubic_terms_are_not_continued_into_extension(self):
        # A cubic fitted to compression turns over when extrapolated backwards; continuing it
        # would put a fold in the law exactly where the coupled solve operates.
        soft, stiff = SpringLaw(a=1000.0), SpringLaw(a=1000.0, b=9.0e5, c=9.0e9)
        self.assertEqual(float(soft.force_n(-0.004)), float(stiff.force_n(-0.004)))

    def test_force_and_stiffness_agree_in_extension(self):
        law = SpringLaw(a=500.0, b=2.0e5, c=1.0e7)
        u = -0.003
        numeric = (law.force_n(u + 1e-9) - law.force_n(u - 1e-9)) / 2e-9
        self.assertAlmostEqual(float(law.stiffness_n_per_m(u)) / float(numeric), 1.0, places=5)

    def test_force_and_stiffness_agree(self):
        law = SpringLaw(a=500.0, b=2.0e5, c=1.0e7)
        u = 0.003
        numeric = (law.force_n(u + 1e-9) - law.force_n(u - 1e-9)) / 2e-9
        self.assertAlmostEqual(float(law.stiffness_n_per_m(u)) / float(numeric), 1.0, places=5)

    def test_a_folded_law_is_rejected(self):
        # A least-squares fit is free to return this; it is not a spring, and in MuJoCo it
        # would look like a contact bug rather than a fitting one.
        self.assertFalse(SpringLaw(a=100.0, b=-1.0e7, c=0.0).is_monotone_nonneg)

    def test_a_stiffening_law_is_accepted(self):
        self.assertTrue(SpringLaw(a=500.0, b=2.0e5, c=1.0e7).is_monotone_nonneg)


class TestRingForce(unittest.TestCase):
    def test_zero_indentation_carries_no_load(self):
        self.assertAlmostEqual(float(ring_force_n(SPEC, SpringLaw(a=1e3), 0.0)), 0.0)

    def test_force_rises_with_indentation(self):
        f = ring_force_n(SPEC, SpringLaw(a=1e3), np.linspace(0.0005, 0.008, 12))
        self.assertTrue(bool(np.all(np.diff(f) > 0)))

    def test_a_finer_ring_is_stiffer_at_the_same_spring_law(self):
        # Per-segment springs, so more segments share the patch and carry more in total.
        # This is why the fitted coefficients fall as n rises, and why a spring law is
        # meaningless without the segment count it was fitted at.
        coarse = ring_force_n(RingSpec(0.060, 12), SpringLaw(a=1e3), 0.005)
        fine = ring_force_n(RingSpec(0.060, 48), SpringLaw(a=1e3), 0.005)
        self.assertGreater(float(fine), float(coarse))


class TestVerticalReaction(unittest.TestCase):
    """How a segment's radial force becomes load on the plate — issue #26.

    Everything else in this file tests the ring's *shape*: monotone in δ, stiffer at more
    segments, zero at zero. All of it passed for months with the resolution inverted, because
    ``cos²θ`` is a positive factor and scales a curve without bending it. So these tests pin a
    magnitude, and they do it by a route that does not call the ring's own machinery.
    """

    #: R 60 mm, 24 segments, a linear 1 kN/m segment spring, pressed 5 mm. Chosen so the whole
    #: sum is three terms and can be written out by hand.
    SPEC = RingSpec(radius_m=0.060, n_segments=24)
    LAW = SpringLaw(a=1.0e3)
    DELTA_M = 0.005

    @staticmethod
    def _by_hand() -> tuple[float, float]:
        """The reaction, computed from literal angles rather than from ``segment_angles``.

        Returns both resolutions so the test can assert the code matches one and is nowhere
        near the other. At 5 mm on this ring only θ = 0 and θ = ±15° reach the plate: the
        segments at ±30° would need ``u = 60 - 55/cos 30° = -3.5 mm``, i.e. the plate is
        3.5 mm short of them.
        """
        r_mm, delta_mm, k_n_per_mm = 60.0, 5.0, 1.0
        total_sec = total_cos = 0.0
        for deg in (-15.0, 0.0, 15.0):
            cos = np.cos(np.radians(deg))
            u_mm = r_mm - (r_mm - delta_mm) / cos
            assert u_mm > 0.0
            total_sec += k_n_per_mm * u_mm / cos
            total_cos += k_n_per_mm * u_mm * cos
        return total_sec, total_cos

    def test_the_reaction_matches_a_hand_computed_sum(self):
        divided, multiplied = self._by_hand()
        # 11.3355 N against 10.9111 N. Only 3.9% apart, because a three-segment patch reaches
        # just +/-15 deg and the two outer segments carry little; the gap is 8.8% by 15 mm on
        # a 24-ring and 14% at +/-45 deg. A shallow test would not have caught this.
        self.assertAlmostEqual(divided, 11.3354970, places=6)
        self.assertAlmostEqual(multiplied, 10.9110992, places=6)
        got = float(ring_force_n(self.SPEC, self.LAW, self.DELTA_M))
        self.assertAlmostEqual(got, divided, places=9)

    def test_the_helper_divides_by_cos_theta(self):
        """Stated as a per-segment identity, so a future refactor that reintroduces the
        multiplication fails here with the reason rather than three tests away with a number.
        """
        state = solve_equilibrium(self.SPEC, self.LAW, self.DELTA_M)
        cos = np.cos(segment_angles(self.SPEC))
        active = state.in_contact
        expected = float(np.sum(state.contact_force_n[active] / cos[active]))
        self.assertAlmostEqual(state.force_n, expected, places=12)
        self.assertGreater(state.force_n,
                           float(np.sum(state.contact_force_n[active] * cos[active])))

    def test_segments_facing_away_are_excluded_rather_than_divided(self):
        """``cos θ`` is exactly zero at ±90°, which every segment count divisible by four
        reaches. Dividing there would be 0/0 and would poison the sum with a NaN."""
        for n in (4, 8, 12, 24, 48):
            spec = RingSpec(radius_m=0.060, n_segments=n)
            with self.subTest(n=n):
                self.assertTrue(np.any(np.isclose(np.cos(segment_angles(spec)), 0.0)))
                self.assertTrue(np.isfinite(float(ring_force_n(spec, self.LAW, 0.005))))


class TestCurvatureOperator(unittest.TestCase):
    """The discrete ``w'' + w``. Everything the coupling does rests on this being right."""

    def test_rigid_translation_costs_no_bending_energy(self):
        # The whole reason alpha is 1/(2(1-cos dtheta)) and not 1/dtheta^2. A band that
        # shifts sideways or downwards without changing shape must store nothing, at every
        # segment count -- not merely in the limit.
        for n in (6, 12, 24, 48):
            operator = curvature_operator(RingSpec(0.060, n))
            theta = segment_angles(RingSpec(0.060, n))
            with self.subTest(n=n):
                self.assertLess(float(np.abs(operator @ np.cos(theta)).max()), 1e-12)
                self.assertLess(float(np.abs(operator @ np.sin(theta)).max()), 1e-12)

    def test_the_naive_second_difference_would_not(self):
        # Records what the rejected discretisation costs, so nobody "simplifies" alpha back.
        d_theta = 2.0 * np.pi / 24
        naive = 1.0 + 2.0 * (np.cos(d_theta) - 1.0) / d_theta**2
        self.assertAlmostEqual(naive, 0.0057, places=4)

    def test_uniform_inflation_does_cost_energy(self):
        # Growing a ring changes its curvature. If this were also annihilated the operator
        # would be a plain second difference wearing a hat.
        operator = curvature_operator(SPEC)
        self.assertTrue(bool(np.allclose(operator @ np.ones(SPEC.n_segments), 1.0)))

    def test_only_neighbours_are_coupled(self):
        operator = curvature_operator(SPEC)
        self.assertEqual(int(np.count_nonzero(operator[5])), 3)
        self.assertNotEqual(operator[0, SPEC.n_segments - 1], 0.0)  # the ring closes

    def test_the_stiffness_matrix_is_symmetric_and_psd_with_rank_n_minus_two(self):
        stiffness = coupling_matrix(COUPLED)
        self.assertTrue(bool(np.allclose(stiffness, stiffness.T)))
        eigenvalues = np.linalg.eigvalsh(stiffness)
        self.assertGreater(float(eigenvalues.min()), -1e-9)
        self.assertEqual(np.linalg.matrix_rank(stiffness), COUPLED.n_segments - 2)

    def test_an_uncoupled_spec_has_no_stiffness(self):
        self.assertTrue(bool(np.all(coupling_matrix(SPEC) == 0.0)))


#: A 3 mm x 30 mm band on the tiny wheel, at the modulus the knocked-down TPU comes out near.
BAND = MappingProxyType({"youngs_pa": 2.0e7, "band_width_m": 0.030,
                         "band_thickness_m": 0.003, "radius_m": 0.0585, "n_segments": 48})


class TestBendingCoupling(unittest.TestCase):
    KWARGS = BAND

    def test_scales_with_the_cube_of_band_thickness(self):
        thin = bending_coupling_n_per_m(**{**self.KWARGS, "band_thickness_m": 0.002})
        thick = bending_coupling_n_per_m(**{**self.KWARGS, "band_thickness_m": 0.004})
        self.assertAlmostEqual(thick / thin, 8.0, places=9)

    def test_a_bandless_wheel_has_exactly_no_coupling(self):
        self.assertEqual(bending_coupling_n_per_m(**{**self.KWARGS,
                                                     "band_thickness_m": 0.0}), 0.0)

    def test_rejects_a_degenerate_band(self):
        with self.assertRaises(ValueError):
            bending_coupling_n_per_m(**{**self.KWARGS, "youngs_pa": 0.0})
        with self.assertRaises(ValueError):
            bending_coupling_n_per_m(**{**self.KWARGS, "band_thickness_m": -0.001})

    def test_the_tiny_design_lands_in_the_same_order_as_its_radial_springs(self):
        # A guard on the derivation, not on the number: coupling that came out a thousand
        # times too small would be invisible in every force comparison, and a thousand times
        # too large would flatten the ring into one rigid body. Either would look like "the
        # band does nothing" or "the band does everything" rather than like an arithmetic
        # slip. The measured value on the tiny design is ~0.8 N/m at N=48.
        coupling = bending_coupling_n_per_m(**self.KWARGS)
        self.assertGreater(coupling, 0.1)
        self.assertLess(coupling, 10.0)

    def test_derived_from_a_design_and_never_chosen(self):
        params = replace(TINY_PARAMS, rim_thickness_mm=3.0)
        spec = ring_for_design(params, TPU, n_segments=48)
        self.assertAlmostEqual(spec.radius_m, 0.060, places=12)
        self.assertTrue(spec.is_coupled)
        # Invariant 2: a thicker band must move the stiffness, not leave it alone.
        thicker = ring_for_design(replace(params, rim_thickness_mm=4.0), TPU, 48)
        self.assertGreater(thicker.band_bending_n_per_m, spec.band_bending_n_per_m)
        self.assertGreater(thicker.band_hoop_n_per_m, spec.band_hoop_n_per_m)

    def test_a_bandless_design_short_circuits_to_an_uncoupled_ring(self):
        spec = ring_for_design(replace(TINY_PARAMS, rim_thickness_mm=0.0), TPU, 48)
        self.assertEqual(spec.band_bending_n_per_m, 0.0)
        self.assertEqual(spec.band_hoop_n_per_m, 0.0)
        self.assertFalse(spec.is_coupled)

    def test_hoop_scales_linearly_with_band_thickness(self):
        # Membrane, not bending: area, not second moment. If this ever came out cubic the two
        # derivations have been copied from one another.
        thin = hoop_coupling_n_per_m(**{**self.KWARGS, "band_thickness_m": 0.002})
        thick = hoop_coupling_n_per_m(**{**self.KWARGS, "band_thickness_m": 0.004})
        self.assertAlmostEqual(thick / thin, 2.0, places=9)

    def test_hoop_dwarfs_bending_in_the_breathing_mode(self):
        # The comparison that makes omitting the hoop term a factor-of-thousands error rather
        # than a refinement. Against uniform inflation the bending term stores k_b*N and the
        # hoop term k_h*N^2, so their ratio is k_h*N/k_b = 12(R/t)^2 -- independent of the
        # segment count, which is the sign that it is a property of the band and not of the
        # discretisation.
        n = self.KWARGS["n_segments"]
        ratio = (hoop_coupling_n_per_m(**self.KWARGS) * n
                 / bending_coupling_n_per_m(**self.KWARGS))
        expected = 12.0 * (self.KWARGS["radius_m"] / self.KWARGS["band_thickness_m"]) ** 2
        self.assertAlmostEqual(ratio / expected, 1.0, places=9)
        self.assertGreater(ratio, 4000.0)


class TestBareRingAgainstRoark(unittest.TestCase):
    """Squeeze a bare ring between two opposite radial loads and check it against the book.

    The one test here that is not self-referential: it compares the discrete band against a
    closed-form result nothing in this repo produced. It is also the test that found the
    model's real defect. With bending alone the ring came out **5.28x** too soft, and the
    excess was entirely the n = 0 breathing mode -- 2/pi against the 0.1488 of every other
    mode combined -- because inextensionality forbids that mode and bending stiffness alone
    does not. Every internal check passed while this was wrong: the operator was a correct
    discretisation of an energy that was missing a term.
    """

    E, WIDTH, THICKNESS, R = 5.0e6, 0.030, 0.003, 0.0585

    def _squeeze(self, spec: RingSpec) -> float:
        """Diameter change under unit opposite radial loads, metres per newton."""
        stiffness = coupling_matrix(spec)
        load = np.zeros(spec.n_segments)
        load[0] = load[spec.n_segments // 2] = 1.0
        # Least squares because the band alone is singular in the two translation modes. The
        # load is self-equilibrated, so a solution exists and the diameter change is the same
        # for all of them -- adding c*cos(theta) lifts one load point and drops the other.
        displacement = np.linalg.lstsq(stiffness, load, rcond=None)[0]
        return float(displacement[0] + displacement[spec.n_segments // 2])

    def _band(self, n: int) -> RingSpec:
        geometry = {"youngs_pa": self.E, "band_width_m": self.WIDTH,
                    "band_thickness_m": self.THICKNESS, "radius_m": self.R, "n_segments": n}
        return RingSpec(radius_m=self.R, n_segments=n,
                        band_bending_n_per_m=bending_coupling_n_per_m(**geometry),
                        band_hoop_n_per_m=hoop_coupling_n_per_m(**geometry))

    @property
    def _roark(self) -> float:
        second_moment = self.WIDTH * self.THICKNESS**3 / 12.0
        return 0.1488 * self.R**3 / (self.E * second_moment)

    def test_matches_the_closed_form_within_two_percent(self):
        for n in (48, 96, 192):
            with self.subTest(n=n):
                self.assertAlmostEqual(self._squeeze(self._band(n)) / self._roark,
                                       1.0, delta=0.02)

    def test_it_converges_with_segment_count(self):
        errors = [abs(self._squeeze(self._band(n)) / self._roark - 1.0)
                  for n in (24, 48, 96, 192)]
        self.assertEqual(errors, sorted(errors, reverse=True))

    def test_dropping_the_hoop_term_is_off_by_the_documented_factor(self):
        # Pinned, not merely asserted to be wrong: 1 + (2/pi)/0.1488 = 5.279. If someone
        # deletes the hoop term this fails with a number that says exactly what is missing.
        bending_only = replace(self._band(192), band_hoop_n_per_m=0.0)
        self.assertAlmostEqual(self._squeeze(bending_only) / self._roark, 5.279, delta=0.01)


class TestEquilibrium(unittest.TestCase):
    def test_without_coupling_it_reproduces_the_closed_form_exactly(self):
        # The two code paths must not be two models. If this drifts, every pre-coupling
        # number in the log silently stops meaning what it said.
        for delta in (0.0005, 0.002, 0.006, 0.012):
            state = solve_equilibrium(SPEC, LAW, delta)
            with self.subTest(delta=delta):
                self.assertTrue(bool(np.array_equal(state.compression_m,
                                                    penetrations(SPEC, delta))))
                self.assertEqual(state.force_n,
                                 float(ring_force_n(SPEC, LAW, delta)))

    def test_a_vanishing_band_converges_to_the_uncoupled_answer(self):
        loose = replace(SPEC, band_bending_n_per_m=1e-9)
        self.assertAlmostEqual(solve_equilibrium(loose, LAW, 0.006).force_n,
                               solve_equilibrium(SPEC, LAW, 0.006).force_n, places=6)

    def test_contact_forces_are_never_negative(self):
        # The active set exists for this: a plate pushes. A multiplier allowed to go negative
        # would be the plate holding a segment down inside the patch, which would inflate the
        # reported reaction with load that is not there.
        for coupling in (0.1, 0.8, 5.0, 50.0):
            for delta in (0.001, 0.003, 0.006):
                state = solve_equilibrium(replace(SPEC, band_bending_n_per_m=coupling),
                                          LAW, delta)
                with self.subTest(coupling=coupling, delta=delta):
                    self.assertTrue(state.converged)
                    self.assertGreaterEqual(float(state.contact_force_n.min()), -1e-9)

    def test_no_segment_is_left_inside_the_plate(self):
        for delta in (0.001, 0.003, 0.006):
            state = solve_equilibrium(COUPLED, LAW, delta)
            theta = segment_angles(COUPLED)
            height = (COUPLED.radius_m - delta) - (COUPLED.radius_m
                                                   - state.compression_m) * np.cos(theta)
            with self.subTest(delta=delta):
                self.assertGreater(float(height.min()), -1e-9)

    def test_the_band_pulls_segments_beside_the_patch_outward(self):
        # The behaviour the whole exercise was for: with a band, a segment's compression is
        # no longer its geometric interference. Some segments end up *past* R, which is why
        # the spring law needed a tension branch.
        state = solve_equilibrium(COUPLED, LAW, 0.006)
        self.assertLess(float(state.compression_m.min()), -1e-6)
        self.assertFalse(bool(np.array_equal(state.compression_m,
                                             penetrations(COUPLED, 0.006))))

    def test_a_band_stiffens_the_ring(self):
        soft = solve_equilibrium(SPEC, LAW, 0.006).force_n
        firm = solve_equilibrium(COUPLED, LAW, 0.006).force_n
        self.assertGreater(firm, soft)

    def test_a_very_stiff_band_refuses_to_conform_and_sheds_contacts(self):
        # Not a bug, and worth pinning down because it looks like one. A band far stiffer
        # than the spokes translates rather than flattens: it drags the segments beside the
        # contact point inward past what the plate demands, so they lift off and the patch
        # *shrinks*. The uncoupled model cannot express this at all.
        rigid = replace(SPEC, band_bending_n_per_m=500.0)
        self.assertLess(int(solve_equilibrium(rigid, LAW, 0.003).in_contact.sum()),
                        int(solve_equilibrium(SPEC, LAW, 0.003).in_contact.sum()))

    def test_segments_facing_away_never_touch_the_plate(self):
        state = solve_equilibrium(COUPLED, LAW, 0.006)
        facing_away = np.cos(segment_angles(COUPLED)) <= 0.0
        self.assertFalse(bool(np.any(state.in_contact & facing_away)))


class TestRoundTrip(unittest.TestCase):
    """Generate from a known law, fit, and check the law comes back."""

    TRUTH = SpringLaw(a=8.0e3, b=1.2e6, c=4.0e8)

    def test_recovers_the_coefficients(self):
        d = np.linspace(0.001, 0.006, 6)
        f = ring_force_n(SPEC, self.TRUTH, d)
        fit = fit_spring_law(SPEC, d, f)
        for name in ("a", "b", "c"):
            with self.subTest(name):
                self.assertAlmostEqual(
                    getattr(fit.law, name) / getattr(self.TRUTH, name), 1.0, places=6
                )

    def test_reports_a_negligible_error_on_exact_data(self):
        d = np.linspace(0.001, 0.006, 6)
        fit = fit_spring_law(SPEC, d, ring_force_n(SPEC, self.TRUTH, d))
        self.assertLess(fit.rms_error_fraction, 1e-9)
        self.assertTrue(fit.ok)

    def test_error_is_reported_on_data_the_model_cannot_match(self):
        # A curve that *falls* with indentation cannot come from a monotone spring ring.
        # The fit must not silently return something and call it fine.
        d = np.linspace(0.001, 0.006, 6)
        f = np.array([5.0, 4.0, 3.0, 2.0, 1.0, 0.5])
        fit = fit_spring_law(SPEC, d, f)
        self.assertFalse(fit.ok)

    def test_the_fit_holds_the_error_it_achieved(self):
        d = np.linspace(0.001, 0.006, 6)
        fit = fit_spring_law(SPEC, d, ring_force_n(SPEC, self.TRUTH, d))
        # There is deliberately no way to get coefficients without the error alongside them.
        self.assertEqual(fit.fitted_force_n.shape, fit.force_n.shape)
        self.assertGreaterEqual(fit.max_error_n, fit.rms_error_n)


class TestFitFailures(unittest.TestCase):
    def test_mismatched_shapes(self):
        with self.assertRaises(FitFailure):
            fit_spring_law(SPEC, np.array([1.0, 2.0]), np.array([1.0]))

    def test_too_few_points_for_the_coefficients(self):
        with self.assertRaises(FitFailure):
            fit_spring_law(SPEC, np.array([0.001, 0.002]), np.array([1.0, 2.0]), order=3)

    def test_a_lower_order_fits_a_short_curve(self):
        fit = fit_spring_law(SPEC, np.array([0.001, 0.002]), np.array([1.0, 2.0]), order=1)
        self.assertEqual(fit.law.b, 0.0)
        self.assertEqual(fit.law.c, 0.0)

    def test_nothing_in_contact(self):
        with self.assertRaises(FitFailure):
            fit_spring_law(SPEC, np.linspace(0.001, 0.006, 6), np.zeros(6))

    def test_zero_indentation_points_are_dropped(self):
        # delta = 0 carries no information and would drag the fit toward the origin twice.
        d = np.concatenate([[0.0], np.linspace(0.001, 0.006, 6)])
        f = np.concatenate([[0.0], ring_force_n(SPEC, TestRoundTrip.TRUTH,
                                                np.linspace(0.001, 0.006, 6))])
        fit = fit_spring_law(SPEC, d, f)
        self.assertEqual(len(fit.delta_m), 6)

    def test_bad_order(self):
        with self.assertRaises(FitFailure):
            fit_spring_law(SPEC, np.linspace(0.001, 0.006, 6), np.ones(6), order=7)


class TestCoupledFit(unittest.TestCase):
    """The fit stops being one linear solve once the band is there. It must still be honest."""

    TRUTH = SpringLaw(a=8.0e3, b=1.2e6, c=4.0e8)
    DELTAS = np.linspace(0.001, 0.006, 6)

    def test_recovers_a_known_law_through_the_coupling(self):
        # The alternation's real test: generate with the coupled model, fit with it, and see
        # whether the coefficients come back. If the frozen-shape linearisation were wrong,
        # this would settle somewhere plausible and wrong rather than failing loudly.
        force = ring_force_n(COUPLED, self.TRUTH, self.DELTAS)
        fit = fit_spring_law(COUPLED, self.DELTAS, force)
        self.assertTrue(fit.converged)
        for name in ("a", "b", "c"):
            with self.subTest(name):
                self.assertAlmostEqual(
                    getattr(fit.law, name) / getattr(self.TRUTH, name), 1.0, places=4
                )

    def test_it_converges_in_a_handful_of_passes(self):
        force = ring_force_n(COUPLED, self.TRUTH, self.DELTAS)
        fit = fit_spring_law(COUPLED, self.DELTAS, force)
        self.assertLessEqual(fit.iterations, 12)
        self.assertGreater(fit.iterations, 1)  # a coupled fit that took one pass did nothing

    def test_an_uncoupled_fit_still_takes_exactly_one_solve(self):
        force = ring_force_n(SPEC, self.TRUTH, self.DELTAS)
        fit = fit_spring_law(SPEC, self.DELTAS, force)
        self.assertEqual(fit.iterations, 1)
        self.assertTrue(fit.converged)

    def test_the_reported_error_comes_from_the_nonlinear_model(self):
        # Not from the last linearised least-squares residual, which would flatter the fit
        # exactly when the frozen shape is furthest from the true one.
        force = ring_force_n(COUPLED, self.TRUTH, self.DELTAS)
        fit = fit_spring_law(COUPLED, self.DELTAS, force)
        np.testing.assert_allclose(fit.fitted_force_n,
                                   ring_force_n(COUPLED, fit.law, self.DELTAS), rtol=1e-12)

    def test_coupling_shifts_the_load_off_the_radial_springs(self):
        # The point of doing this at all. Fit the same curve with and without a band: with
        # one, the springs no longer have to carry the band's share, so they come out softer.
        # An uncoupled fit of a banded wheel is not wrong by a little, it is attributing
        # stiffness to the wrong member.
        force = ring_force_n(COUPLED, self.TRUTH, self.DELTAS)
        with_band = fit_spring_law(COUPLED, self.DELTAS, force)
        without = fit_spring_law(SPEC, self.DELTAS, force)
        self.assertGreater(without.law.a, with_band.law.a)

    def test_a_fit_that_did_not_converge_is_not_ok(self):
        force = ring_force_n(COUPLED, self.TRUTH, self.DELTAS)
        fit = fit_spring_law(COUPLED, self.DELTAS, force, max_iterations=1)
        self.assertFalse(fit.converged)
        self.assertFalse(fit.ok)  # even if the error happens to look fine
        self.assertIn("DID NOT CONVERGE", fit.summary())

    def test_the_projected_step_reaches_the_nnls_answer_and_stops(self):
        """``TODO.md`` #22, as a unit test of the optimiser rather than of a wheel.

        A non-negative *linear* least squares problem is convex, so `nnls` gives the exact
        constrained optimum — a second, differently-shaped algorithm to check the projected
        Gauss-Newton against, which is the only kind of check that catches a stall reaching a
        plausible wrong place. The matrix is chosen so the unconstrained solution has negative
        components and three of the eight parameters pin at zero; that is the configuration
        the coupled tabulated fit hits, and the one that ran 400 iterations without
        converging before the step was solved on the free block only.
        """
        from wheelopt.rom.fit import _levenberg_marquardt, nnls

        rng = np.random.default_rng(20260809)
        matrix = rng.normal(size=(24, 8))
        truth = np.array([2.0, -1.0, 1.5, -0.8, 3.0, -2.0, 0.5, 1.0])
        target = matrix @ truth + 0.01 * rng.normal(size=24)
        exact = nnls(matrix, target)
        self.assertEqual(int(np.count_nonzero(exact <= 1e-12)), 3, "no parameters pinned")

        start = np.full(8, 0.5)
        found, iterations, converged = _levenberg_marquardt(
            lambda x: matrix @ x - target, start, 400, 1e-10, non_negative=True
        )
        self.assertTrue(converged, "the projected step still runs to the iteration cap")
        self.assertLess(iterations, 40)
        self.assertTrue(np.all(found >= 0.0))
        # Same cost, to the finite-difference floor. Compared on the residual rather than on
        # the parameters: pinned parameters are degenerate directions and two solvers may sit
        # at different points of the same flat.
        def cost(x: np.ndarray) -> float:
            return float(np.sum((matrix @ x - target) ** 2))

        self.assertLess(cost(found), cost(exact) * (1.0 + 1e-6))

    def test_the_patch_count_needs_the_law_once_the_ring_is_coupled(self):
        # Without a law the count falls back to the geometric interference, which is only a
        # lower bound on a coupled ring. Callers get the bound, not a wrong answer dressed up
        # as the right one.
        law = fit_spring_law(COUPLED, self.DELTAS,
                             ring_force_n(COUPLED, self.TRUTH, self.DELTAS)).law
        solved = contact_segments(COUPLED, 0.006, law)
        geometric = contact_segments(COUPLED, 0.006)
        self.assertEqual(geometric, contact_segments(SPEC, 0.006))
        self.assertEqual(solved,
                         int(solve_equilibrium(COUPLED, law, 0.006).in_contact.sum()))


@unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
class TestMjcf(unittest.TestCase):
    LAW = SpringLaw(a=573.8, b=-2.334e5, c=3.573e7)

    def test_the_model_compiles(self):
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW))
        self.assertEqual(model.njnt, SPEC.n_segments)  # the hub is welded, not jointed

    def test_the_running_surface_sits_at_the_wheel_radius(self):
        """The capsule radius must not be added to the wheel radius.

        It was: bodies were placed at R, putting the contact surface at R + r_capsule, so
        contact began ~4 mm early and the model reported five times the analytic force.
        """
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        segs = [g for g in range(model.ngeom)
                if g != floor and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE]
        lowest = min(
            float(data.geom_xpos[g, 2] - model.geom_size[g, 0]) for g in segs
        )
        self.assertAlmostEqual(lowest, 0.0, places=6)

    def test_segments_do_not_collide_with_each_other(self):
        """Capsule-to-capsule contact would be a coupling arriving from the wrong place.

        The only thing joining adjacent segments in this model is the shear band, and it
        arrives as fixed tendons with a known stiffness. If capsules touched, the fit would
        absorb that contact stiffness as if it were spoke stiffness.
        """
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        for c in range(data.ncon):
            pair = (data.contact[c].geom1, data.contact[c].geom2)
            self.assertIn(floor, pair, "two segments are in contact with each other")

    def test_mujoco_resolves_the_contact_force_as_f_over_cos_theta(self):
        """The arbitration of issue #26, kept as a regression test.

        MuJoCo assumes neither resolution: ``condim="1"`` makes the floor genuinely
        frictionless, and the segments settle wherever the constraint solver puts them. So we
        read back *its* compressions and *its* per-contact forces and ask which relation holds
        between them. This is a per-segment test on purpose — comparing totals would confound
        the answer with contact discretisation, since round capsules engage a wider set of
        segments than the analytic scallop geometry does, which is a separate known gap.
        """
        from wheelopt.rom.mjcf import build_mjcf

        law, delta = SpringLaw(a=1.0e4), 0.010
        model = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, law, indentation_m=delta))
        data = mujoco.MjData(model)
        qpos, dofs = model.jnt_qposadr[:], model.jnt_dofadr[:]
        for _ in range(4000):
            data.qfrc_applied[dofs] = (law.force_n(-data.qpos[qpos])
                                       - 4.0 * data.qvel[dofs])
            mujoco.mj_step(model, data)
        self.assertLess(float(np.max(np.abs(data.qvel))), 1e-8, "not settled")

        cos = np.cos(segment_angles(SPEC))
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        wrench = np.zeros(6)
        # Accumulated per segment, not per contact: a capsule resting on a plane is a line
        # contact, and MuJoCo resolves it as *two* points at the ends of the axis, each
        # carrying half the load. Asserting per contact fails by exactly a factor of two.
        vertical = np.zeros(SPEC.n_segments)
        horizontal = np.zeros(SPEC.n_segments)
        for c in range(data.ncon):
            con = data.contact[c]
            mujoco.mj_contactForce(model, data, c, wrench)
            world = con.frame.reshape(3, 3).T @ wrench[:3]
            i = int(mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM,
                int(con.geom2 if con.geom1 == floor else con.geom1))[1:])
            vertical[i] += float(world[2])
            horizontal[i] += abs(float(world[0]))

        touching = np.flatnonzero(vertical > 1e-9)
        for i in touching:
            f_r = float(law.force_n(-data.qpos[qpos][i]))
            with self.subTest(segment=int(i)):
                # Frictionless: the plate pushes straight up and nothing sideways. If this
                # ever fails the rest of the test is meaningless, not merely wrong.
                self.assertAlmostEqual(horizontal[i], 0.0, places=9)
                self.assertAlmostEqual(vertical[i], f_r / cos[i], delta=1e-6 * vertical[i])
                if cos[i] < 1.0:  # at theta = 0 the two resolutions are the same statement
                    self.assertNotAlmostEqual(vertical[i], f_r * cos[i],
                                              delta=1e-3 * vertical[i])
        # Five segments carry load here, out to +/-30 deg where the two resolutions differ by
        # 1/cos^2(30) = 1.333. A test that only ever touched one segment would pass either way.
        self.assertEqual(len(touching), 5)

    def test_the_tangential_joint_is_absent_unless_asked_for(self):
        """Off means absent, not locked. Every result that predates the joint must still be
        reproducible by the same XML, so a radial-only ring is byte-identical to what it was.
        """
        from wheelopt.rom.mjcf import build_mjcf

        plain = build_mjcf(SPEC, self.LAW)
        self.assertNotIn('name="t0"', plain)
        model = mujoco.MjModel.from_xml_string(plain)
        self.assertEqual(model.njnt, SPEC.n_segments)

        both = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW, tangential="slide"))
        self.assertEqual(both.njnt, 2 * SPEC.n_segments)

    def test_the_tangential_axis_raises_the_tip_by_v_sin_theta(self):
        """The axis is what makes the MJCF ring and the analytic ring the *same* model.

        `solve_equilibrium_2dof` writes the tip height as ``(R-δ) - (R-u)cos θ + v sin θ``.
        So displacing a segment along its tangential joint by ``v`` must raise it by exactly
        ``v sin θ`` and move it horizontally by ``v cos θ``. Checked by moving the joint and
        reading the body position, which tests the compiled model rather than the string.
        """
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW, tangential="slide"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        theta = segment_angles(SPEC)
        before = np.array(data.xpos[1:], copy=True)  # body 0 is world, body 1 the hub
        v = 0.004
        for i in range(SPEC.n_segments):
            data.qpos[model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"t{i}")]] = v
        mujoco.mj_forward(model, data)
        after = np.array(data.xpos[1:], copy=True)
        moved = after - before
        for i in range(SPEC.n_segments):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"seg{i}") - 1
            with self.subTest(segment=i):
                self.assertAlmostEqual(moved[body, 2], v * np.sin(theta[i]), places=9)
                self.assertAlmostEqual(moved[body, 0], v * np.cos(theta[i]), places=9)
                self.assertAlmostEqual(moved[body, 1], 0.0, places=12)

    def test_the_two_freedom_ring_satisfies_the_analytic_equilibrium(self):
        """MuJoCo's two-freedom ring against the conditions `solve_equilibrium_2dof` imposes.

        Per segment, not on the total, for the same reason as the #26 arbitration: MuJoCo's
        capsules engage a different set of segments than the analytic point geometry, so a
        comparison of sums would confound the joint with the discretisation. The conditions
        are ``f_r(u) = λ cos θ`` and ``f_t(v) = λ sin θ``, and MuJoCo is told neither — it is
        given two force laws on two joints and left to find the equilibrium.
        """
        from wheelopt.rom.mjcf import build_mjcf

        spec = RingSpec(radius_m=0.085, n_segments=12)
        radial, tangential = SpringLaw(a=24807.0), SpringLaw(a=185.1)
        model = mujoco.MjModel.from_xml_string(
            build_mjcf(spec, radial, indentation_m=0.018, tangential="slide")
        )
        data = mujoco.MjData(model)

        def addr(prefix):
            ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}{i}")
                   for i in range(spec.n_segments)]
            return model.jnt_qposadr[ids], model.jnt_dofadr[ids]

        rq, rd = addr("j")
        tq, td = addr("t")
        for _ in range(20000):
            data.qfrc_applied[rd] = radial.force_n(-data.qpos[rq]) - 2.0 * data.qvel[rd]
            data.qfrc_applied[td] = (-symmetric_force_n(tangential, data.qpos[tq])
                                     - 2.0 * data.qvel[td])
            mujoco.mj_step(model, data)
        self.assertLess(float(np.max(np.abs(data.qvel))), 1e-8, "not settled")

        theta = segment_angles(spec)
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        lam = np.zeros(spec.n_segments)
        wrench = np.zeros(6)
        for c in range(data.ncon):
            con = data.contact[c]
            mujoco.mj_contactForce(model, data, c, wrench)
            i = int(mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM,
                int(con.geom2 if con.geom1 == floor else con.geom1))[1:])
            lam[i] += float((con.frame.reshape(3, 3).T @ wrench[:3])[2])

        touching = np.flatnonzero(lam > 1e-6)
        self.assertGreater(len(touching), 1, "no off-axis segment carries load")
        u, v = -data.qpos[rq], data.qpos[tq]
        for i in touching:
            with self.subTest(segment=int(i)):
                self.assertAlmostEqual(float(radial.force_n(u[i])),
                                       lam[i] * np.cos(theta[i]), delta=1e-6 * lam[i])
                self.assertAlmostEqual(float(symmetric_force_n(tangential, v[i])),
                                       lam[i] * np.sin(theta[i]), delta=1e-6 * lam[i])
        # A claw at 30 deg is 134x softer sideways than radially, so it folds rather than
        # compresses: 15.8 mm of splay against 0.2 mm of compression. That is the whole
        # phenomenon, and a joint on the wrong axis would not produce it.
        off_axis = [i for i in touching if abs(theta[i]) > 1e-9]
        for i in off_axis:
            self.assertGreater(abs(v[i]), 20.0 * abs(u[i]))

    def test_indentation_lowers_the_hub(self):
        from wheelopt.rom.mjcf import build_mjcf

        high = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW, indentation_m=0.0))
        low = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW, indentation_m=0.005))
        self.assertAlmostEqual(high.body_pos[1, 2] - low.body_pos[1, 2], 0.005)

    def test_an_uncoupled_ring_has_no_tendons(self):
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(SPEC, self.LAW))
        self.assertEqual(model.ntendon, 0)
        self.assertNotIn("<tendon>", build_mjcf(SPEC, self.LAW))

    def test_one_tendon_per_segment_spanning_three_joints(self):
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(COUPLED, self.LAW))
        # N bending tendons plus the single hoop, which spans every joint.
        self.assertEqual(model.ntendon, COUPLED.n_segments + 1)
        self.assertTrue(bool(np.all(model.tendon_num[:-1] == 3)))
        self.assertEqual(int(model.tendon_num[-1]), COUPLED.n_segments)

    def test_the_tendons_reproduce_the_analytic_coupling_matrix_exactly(self):
        """MuJoCo's band must be the same operator `ring.py` fits against, not a cousin.

        A fixed tendon stores ``(k/2)L²`` with ``L = Σ coef·q``, so setting the coefficients
        to a row of the curvature operator makes the passive force ``K u`` identically --
        no linearisation, no small-angle assumption. Checked against a shape with no symmetry
        so that a transposed or mis-wrapped row cannot pass by cancelling.
        """
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(COUPLED, self.LAW))
        data = mujoco.MjData(model)
        rng = np.random.default_rng(0)
        compression = rng.normal(scale=0.002, size=COUPLED.n_segments)
        data.qpos[:] = -compression  # positive q is outward; the joint reads q = -u
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(
            data.qfrc_passive, coupling_matrix(COUPLED) @ compression, atol=1e-9
        )

    def test_a_rigidly_translated_band_stores_nothing_in_mujoco_either(self):
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(COUPLED, self.LAW))
        data = mujoco.MjData(model)
        data.qpos[:] = -0.002 * np.cos(segment_angles(COUPLED))
        mujoco.mj_forward(model, data)
        self.assertLess(float(np.abs(data.qfrc_passive).max()), 1e-9)

    def test_the_band_rest_length_is_not_inherited_from_the_pose(self):
        # springlength="0" is explicit on purpose: MuJoCo's default is the length at compile
        # time, which would make the band's rest state depend on how the model happened to be
        # posed rather than on the geometry.
        from wheelopt.rom.mjcf import build_mjcf

        self.assertIn('springlength="0"', build_mjcf(COUPLED, self.LAW))
        model = mujoco.MjModel.from_xml_string(build_mjcf(COUPLED, self.LAW))
        self.assertTrue(bool(np.all(model.tendon_lengthspring == 0.0)))

    def test_the_joint_range_does_not_cap_the_bulge(self):
        # With a band the segments beside the patch move outward. A range that clipped that
        # would read as a stiffness disagreement rather than as the constraint it is.
        from wheelopt.rom.mjcf import build_mjcf

        model = mujoco.MjModel.from_xml_string(build_mjcf(COUPLED, self.LAW))
        compression = solve_equilibrium(COUPLED, self.LAW, 0.006).compression_m
        # q = -u, so the deepest compression tests the lower bound and the bulge the upper.
        self.assertLess(float(compression.max()), 0.2 * abs(float(model.jnt_range[:, 0].max())))
        self.assertLess(-float(compression.min()), 0.2 * float(model.jnt_range[:, 1].min()))

    def test_capsules_always_fit_inside_the_wheel(self):
        """The body radius is r(1 - pi/2n), so it is positive for any legal ring.

        Asserted rather than guarded against: an earlier version raised on
        ``body_radius <= 0``, a branch that cannot be reached because RingSpec already
        requires three segments. Dead code that looks like a safety check is worse than
        none, because it invites trust it cannot repay.
        """
        for n in (3, 4, 24, 200):
            with self.subTest(n=n):
                spec = RingSpec(radius_m=0.060, n_segments=n)
                self.assertGreater(spec.radius_m - 0.5 * spec.segment_arc_m * 0.5, 0.0)

    def test_simulated_response_tracks_the_analytic_one(self):
        """The MuJoCo ring against the analytic ring it was built from.

        Not expected to match exactly — the analytic model treats each segment as a point on
        a circle, MuJoCo as a capsule on a scalloped surface — so this asserts the loose
        agreement actually measured (within ~10%) rather than an equality that would fail
        for a good reason.
        """
        from wheelopt.rom.mjcf import static_load_deflection

        d = np.array([0.003, 0.006])
        simulated = static_load_deflection(SPEC, self.LAW, d, settle_s=0.75)
        analytic = ring_force_n(SPEC, self.LAW, d)
        self.assertTrue(bool(np.all(simulated > 0)))
        np.testing.assert_allclose(simulated, analytic, rtol=0.25)


class TestRampBasis(unittest.TestCase):
    """The piecewise-linear basis, checked against forces worked out by hand."""

    KNOTS = np.array([0.0, 0.0025, 0.005, 0.0075, 0.010])
    SLOPES = np.array([1000.0, 100.0, 100.0, 2000.0])

    def law(self) -> TabulatedLaw:
        return TabulatedLaw(knots_m=self.KNOTS, slopes_n_per_m=self.SLOPES)

    def test_forces_at_and_between_knots(self):
        u = np.array([0.0, 0.0025, 0.00375, 0.005, 0.0075, 0.010])
        # 1000*0.0025 = 2.5 at the first knot, then 100 N/m for two intervals, then 2000.
        expected = [0.0, 2.5, 2.625, 2.75, 3.0, 8.0]
        np.testing.assert_allclose(self.law().force_n(u), expected)

    def test_beyond_the_last_knot_it_extrapolates_at_the_last_slope(self):
        """Not flat. A law that stops resisting past the fitted range is wrong the dangerous
        way — it would let a wheel sink through an obstacle it has never been tested against."""
        law = self.law()
        beyond = 0.015
        expected = law.force_n(0.010) + self.SLOPES[-1] * (beyond - 0.010)
        self.assertAlmostEqual(float(law.force_n(beyond)), expected)
        self.assertAlmostEqual(float(law.stiffness_n_per_m(beyond)), self.SLOPES[-1])

    def test_tension_continues_the_first_slope(self):
        law = self.law()
        self.assertAlmostEqual(float(law.force_n(-0.002)), -2.0)
        self.assertAlmostEqual(float(law.stiffness_n_per_m(-0.002)), self.SLOPES[0])

    def test_response_is_linear_in_the_slopes_over_the_table(self):
        """The property everything else rests on: superposition over the slope vector.

        If this fails, the uncoupled fit is no longer one solve and the design matrix built
        from unit basis laws is not the model's Jacobian. Asserted over the table's own range
        plus the tension branch — beyond the last knot the law clamps the extrapolation slope
        at zero, which is deliberately *not* linear in the slopes.
        """
        u = np.linspace(-0.002, self.KNOTS[-1], 25)
        columns = ramp_basis(self.KNOTS, u)
        np.testing.assert_allclose(columns @ self.SLOPES, self.law().force_n(u))

    def test_scalar_in_scalar_out(self):
        self.assertIsInstance(float(self.law().force_n(0.003)), float)
        self.assertEqual(np.ndim(self.law().force_n(0.003)), 0)
        self.assertEqual(np.ndim(self.law().force_n([0.003])), 1)


class TestTabulatedLaw(unittest.TestCase):
    def test_accepts_a_softening_table(self):
        """A negative tangent is buckling, which is the phenomenon under study. Forbidding it
        is what kept the nominal design unfittable, so it must be constructible."""
        law = TabulatedLaw(knots_m=np.array([0.0, 0.005, 0.010]),
                           slopes_n_per_m=np.array([1000.0, -100.0]))
        self.assertFalse(law.is_monotone_nonneg)
        self.assertTrue(law.is_valid_spring)
        self.assertAlmostEqual(float(law.force_n(0.010)), 4.5)

    def test_rejects_a_table_that_pulls(self):
        """The constraint that *is* enforced: compressing a segment cannot make it pull."""
        with self.assertRaises(ValueError):
            TabulatedLaw(knots_m=np.array([0.0, 0.005, 0.010]),
                         slopes_n_per_m=np.array([1000.0, -2000.0]))

    def test_a_downward_final_slope_is_clamped_rather_than_extrapolated(self):
        """Continuing a negative tail would drive the force through zero and pull a wheel
        into an obstacle it was never fitted against. Flat understates instead."""
        law = TabulatedLaw(knots_m=np.array([0.0, 0.005, 0.010]),
                           slopes_n_per_m=np.array([1000.0, -100.0]))
        self.assertEqual(law.extrapolation_slope_n_per_m, 0.0)
        self.assertAlmostEqual(float(law.force_n(0.020)), float(law.force_n(0.010)))
        self.assertEqual(float(law.stiffness_n_per_m(0.020)), 0.0)

    def test_rejects_a_table_that_does_not_start_at_zero(self):
        with self.assertRaises(ValueError):
            TabulatedLaw(knots_m=np.array([0.001, 0.005]), slopes_n_per_m=np.array([1000.0]))

    def test_rejects_non_ascending_knots_and_a_slope_count_mismatch(self):
        with self.assertRaises(ValueError):
            TabulatedLaw(knots_m=np.array([0.0, 0.005, 0.005]),
                         slopes_n_per_m=np.array([1.0, 1.0]))
        with self.assertRaises(ValueError):
            TabulatedLaw(knots_m=np.array([0.0, 0.005, 0.010]),
                         slopes_n_per_m=np.array([1.0]))

    def test_from_forces_round_trips(self):
        knots = uniform_knots(0.012, 4)
        forces = np.array([0.0, 5.0, 5.5, 9.0, 20.0])
        law = TabulatedLaw.from_forces(knots, forces)
        np.testing.assert_allclose(law.forces_n, forces)
        np.testing.assert_allclose(law.force_n(knots), forces)

    def test_arrays_cannot_be_mutated_through_the_law(self):
        """Frozen rebinds the field, not the buffer. A shared law must stay shared safely."""
        law = TabulatedLaw(knots_m=uniform_knots(0.01, 2),
                           slopes_n_per_m=np.array([100.0, 200.0]))
        with self.assertRaises(ValueError):
            law.slopes_n_per_m[0] = -1.0

    def test_satisfies_the_radial_law_protocol(self):
        law = TabulatedLaw(knots_m=uniform_knots(0.01, 2),
                           slopes_n_per_m=np.array([100.0, 200.0]))
        self.assertIsInstance(law, RadialLaw)
        self.assertIsInstance(SpringLaw(a=1.0), RadialLaw)
        self.assertTrue(law.is_monotone_nonneg)


class TestNnls(unittest.TestCase):
    def test_agrees_with_lstsq_when_the_answer_is_already_feasible(self):
        rng = np.random.default_rng(0)
        a = rng.random((12, 4))
        x_true = np.array([1.0, 2.0, 0.5, 3.0])
        b = a @ x_true
        np.testing.assert_allclose(nnls(a, b), x_true, atol=1e-9)

    def test_is_not_the_same_as_clipping_the_unconstrained_answer(self):
        """The reason this function exists rather than a `np.maximum(lstsq, 0)` one-liner.

        Here the unconstrained fit is (4, -1). Clipping gives (4, 0) with residual norm
        3.74; the constrained optimum is (2, 0) with residual norm 1.41. Clipping does not
        re-fit what survives.
        """
        a = np.array([[1.0, 1.0], [1.0, 2.0], [1.0, 3.0]])
        b = np.array([3.0, 2.0, 1.0])
        unconstrained = np.linalg.lstsq(a, b, rcond=None)[0]
        clipped = np.maximum(unconstrained, 0.0)
        constrained = nnls(a, b)
        self.assertLess(unconstrained[1], 0.0)
        np.testing.assert_allclose(constrained, [2.0, 0.0], atol=1e-9)
        self.assertLess(np.linalg.norm(a @ constrained - b),
                        np.linalg.norm(a @ clipped - b))

    def test_all_directions_uphill_gives_zero(self):
        a = np.eye(3)
        np.testing.assert_allclose(nnls(a, -np.ones(3)), np.zeros(3))


class TestTabulatedFit(unittest.TestCase):
    #: A plateau: stiff, then almost nothing, then stiff again. This is the *shape* of the
    #: nominal design's measured curve (tangent 12.1 -> 0.09 -> 10.0 N/mm across 0-12 mm),
    #: which is what a cubic cannot bend around and why this law exists.
    PLATEAU = TabulatedLaw(knots_m=uniform_knots(0.006, 4),
                           slopes_n_per_m=np.array([12000.0, 300.0, 300.0, 9000.0]))
    #: The same shape but with a genuine limit point, which is what the nominal design's
    #: measured curve has (tangent 42.8 -> -7.0 -> 17.3 N/mm). A monotone law cannot go here.
    SOFTENING = TabulatedLaw(knots_m=uniform_knots(0.006, 4),
                             slopes_n_per_m=np.array([12000.0, -4000.0, 300.0, 9000.0]))

    def curve(self, spec=SPEC, law=None):
        deltas = np.linspace(0.0005, 0.006, 20)
        return deltas, ring_force_n(spec, law or self.PLATEAU, deltas)

    def test_recovers_the_table_it_was_generated_from(self):
        deltas, force = self.curve()
        fit = fit_tabulated_law(SPEC, deltas, force, n_intervals=4, smoothing=0.0)
        np.testing.assert_allclose(fit.law.slopes_n_per_m, self.PLATEAU.slopes_n_per_m,
                                   rtol=1e-6)
        self.assertLess(fit.rms_error_fraction, 1e-9)
        self.assertTrue(fit.ok)

    def test_beats_the_cubic_on_a_plateau(self):
        """The claim the redirection rests on, as an inequality rather than an anecdote."""
        deltas, force = self.curve()
        table = fit_tabulated_law(SPEC, deltas, force, n_intervals=4, smoothing=0.0)
        cubic = fit_spring_law(SPEC, deltas, force)
        self.assertGreater(cubic.rms_error_fraction, 0.01)
        self.assertLess(table.rms_error_fraction, 0.1 * cubic.rms_error_fraction)

    def test_matches_the_cubic_on_a_curve_the_cubic_can_represent(self):
        """The other half: a table must not be *worse* where three coefficients suffice."""
        deltas, force = self.curve(law=LAW)
        table = fit_tabulated_law(SPEC, deltas, force, n_intervals=4, smoothing=0.0)
        cubic = fit_spring_law(SPEC, deltas, force)
        self.assertLess(cubic.rms_error_fraction, 1e-6)
        self.assertLess(table.rms_error_fraction, 0.01)

    def test_the_fitted_law_never_pulls_even_on_noisy_data(self):
        """Non-negative force is the feasible set, not a check applied afterwards."""
        rng = np.random.default_rng(7)
        deltas, force = self.curve()
        noisy = force * (1.0 + 0.15 * rng.standard_normal(force.shape))
        fit = fit_tabulated_law(SPEC, deltas, noisy, n_intervals=6)
        self.assertTrue(bool(np.all(fit.law.forces_n >= -1e-9)))
        self.assertTrue(fit.law.is_valid_spring)

    def test_monotone_true_forbids_softening_and_costs_accuracy(self):
        """Both halves of the flag, on data that genuinely softens.

        The permissive fit is exact here because the truth is a monotone table; what this
        pins is that `monotone=True` is a real restriction of the same solve, and that the
        default is the one that can follow a curve down.
        """
        deltas, force = self.curve(law=self.SOFTENING)
        strict = fit_tabulated_law(SPEC, deltas, force, n_intervals=4, monotone=True,
                                   smoothing=0.0)
        loose = fit_tabulated_law(SPEC, deltas, force, n_intervals=4, smoothing=0.0)
        self.assertTrue(strict.law.is_monotone_nonneg)
        self.assertFalse(loose.law.is_monotone_nonneg)
        self.assertLess(loose.rms_error_fraction, 0.1 * strict.rms_error_fraction)

    def test_default_interval_count_follows_the_data_length(self):
        deltas, force = self.curve()
        self.assertEqual(fit_tabulated_law(SPEC, deltas, force).law.n_intervals, 8)
        short = slice(0, 6)
        self.assertEqual(
            fit_tabulated_law(SPEC, deltas[short], force[short]).law.n_intervals, 3
        )

    def test_smoothing_trades_error_for_a_less_jagged_law(self):
        """Both directions of the trade, so neither can silently stop happening."""
        rng = np.random.default_rng(3)
        deltas, force = self.curve()
        noisy = force * (1.0 + 0.10 * rng.standard_normal(force.shape))
        rough = fit_tabulated_law(SPEC, deltas, noisy, n_intervals=8, smoothing=0.0)
        smooth = fit_tabulated_law(SPEC, deltas, noisy, n_intervals=8, smoothing=0.5)
        jag = lambda fit: float(np.abs(np.diff(fit.law.slopes_n_per_m)).sum())
        self.assertLess(jag(smooth), jag(rough))
        self.assertGreaterEqual(smooth.rms_error_fraction, rough.rms_error_fraction)
        with self.assertRaises(FitFailure):
            fit_tabulated_law(SPEC, deltas, force, smoothing=-1.0)

    def test_refuses_more_intervals_than_data(self):
        deltas, force = self.curve()
        with self.assertRaises(FitFailure):
            fit_tabulated_law(SPEC, deltas[:3], force[:3], n_intervals=5)
        with self.assertRaises(FitFailure):
            fit_tabulated_law(SPEC, deltas, force, n_intervals=0)

    def test_a_coupled_ring_fits_and_converges(self):
        """The band path. Bandless is the direction, but T3 is still the comparator, and a
        piecewise-constant tangent makes the coupled Newton semismooth — so it is checked."""
        deltas, force = self.curve(spec=COUPLED)
        fit = fit_tabulated_law(COUPLED, deltas, force, n_intervals=4, smoothing=0.0)
        self.assertTrue(fit.converged)
        self.assertLess(fit.rms_error_fraction, 0.05)
        self.assertTrue(bool(np.all(fit.law.slopes_n_per_m >= 0.0)))


class TestRingFromClawCurve(unittest.TestCase):
    """Segments-are-claws: the path with no fit in it at all."""

    CLAW = WheelParams(outer_radius_mm=85.0, width_mm=45.0, n_spokes=12,
                       spoke_thickness_mm=7.0, rim_thickness_mm=0.0, hub_radius_mm=22.0,
                       claw_taper_ratio=0.5)
    DELTA = np.array([0.001, 0.002, 0.004, 0.006, 0.008])
    FORCE = np.array([4.59, 4.55, 4.58, 4.65, 4.73])

    def test_the_law_is_the_measurement(self):
        """No deconvolution, so the curve comes back out of the law exactly. This is the
        whole reason the claw redirection is worth anything to the ROM."""
        spec, law = ring_from_claw_curve(self.CLAW, self.DELTA, self.FORCE)
        self.assertEqual(spec.n_segments, self.CLAW.n_spokes)
        self.assertAlmostEqual(spec.radius_m, 0.085)
        self.assertFalse(spec.is_coupled)
        np.testing.assert_allclose(law.force_n(self.DELTA), self.FORCE)
        self.assertEqual(float(law.force_n(0.0)), 0.0)

    def test_a_measured_claw_that_softens_is_accepted(self):
        """The measured claw does soften — 4.59 to 4.55 N over the first millimetre — and
        that must survive, because it is the behaviour the design has."""
        _, law = ring_from_claw_curve(self.CLAW, self.DELTA, self.FORCE)
        self.assertFalse(law.is_monotone_nonneg)
        self.assertTrue(law.is_valid_spring)

    def test_refuses_a_banded_design(self):
        banded = replace(self.CLAW, rim_thickness_mm=3.0)
        with self.assertRaises(FitFailure):
            ring_from_claw_curve(banded, self.DELTA, self.FORCE)

    def test_refuses_a_curve_that_pulls_or_is_out_of_order(self):
        with self.assertRaises(FitFailure):
            ring_from_claw_curve(self.CLAW, self.DELTA, self.FORCE * -1.0)
        with self.assertRaises(FitFailure):
            ring_from_claw_curve(self.CLAW, self.DELTA[::-1], self.FORCE)

    def test_the_ring_reproduces_a_single_claw_in_contact(self):
        """At these indentations only the claw at the contact point touches — the next tip
        is 30 deg away and does not reach until R(1-cos 30) = 11.4 mm — so the ring's whole
        response is that one claw, and it must equal the measured curve."""
        spec, law = ring_from_claw_curve(self.CLAW, self.DELTA, self.FORCE)
        self.assertEqual(contact_segments(spec, float(self.DELTA.max())), 1)
        np.testing.assert_allclose(ring_force_n(spec, law, self.DELTA), self.FORCE,
                                   rtol=1e-9)


class TestSecondContactDelta(unittest.TestCase):
    """R(1-cos 2pi/n), and the reason it is a named function rather than an expression.

    It is one factor of two inside a cosine away from `polygon_drop_m`, and doubling the
    drop is the natural wrong guess: on the R 60 mm, 12-claw design that gives 4.09 mm
    against the true 8.04 mm. Both are plausible millimetre-scale numbers on a 60 mm wheel.
    """

    SPEC = RingSpec(radius_m=0.060, n_segments=12, root_radius_m=0.020)

    def test_it_is_the_closed_form(self):
        self.assertAlmostEqual(second_contact_delta_m(self.SPEC),
                               0.060 * (1.0 - np.cos(np.pi / 6.0)))
        self.assertAlmostEqual(second_contact_delta_m(self.SPEC) * 1e3, 8.038, places=3)

    def test_it_is_not_twice_the_polygon_drop(self):
        """The confusion the docstring warns about, pinned as a test so it stays wrong."""
        self.assertNotAlmostEqual(second_contact_delta_m(self.SPEC),
                                  2.0 * polygon_drop_m(0.060, 12), places=4)

    def test_the_ring_really_does_engage_a_second_segment_there(self):
        """Against the ring's own contact set rather than against the same formula twice."""
        engage = second_contact_delta_m(self.SPEC)
        self.assertEqual(contact_segments(self.SPEC, 0.999 * engage), 1)
        self.assertEqual(contact_segments(self.SPEC, 1.001 * engage), 3)

    def test_more_segments_engage_sooner(self):
        fine = replace(self.SPEC, n_segments=36)
        self.assertLess(second_contact_delta_m(fine), second_contact_delta_m(self.SPEC))


class TestValidateRing(unittest.TestCase):
    """A held-out error, not a fit error. `iterations == 0` is how a caller tells them apart."""

    SPEC = RingSpec(radius_m=0.060, n_segments=12, root_radius_m=0.020)
    LAW = SpringLaw(a=2.0e4)

    def _curve(self, deltas):
        return deltas, ring_force_n(self.SPEC, self.LAW, deltas)

    def test_a_ring_measured_against_its_own_prediction_has_no_error(self):
        deltas = np.array([0.002, 0.004, 0.006, 0.008])
        fit = validate_ring(self.SPEC, self.LAW, *self._curve(deltas))
        self.assertAlmostEqual(fit.rms_error_fraction, 0.0, places=12)
        self.assertTrue(fit.ok)

    def test_nothing_was_fitted_and_it_says_so(self):
        deltas = np.array([0.002, 0.004, 0.006, 0.008])
        fit = validate_ring(self.SPEC, self.LAW, *self._curve(deltas))
        self.assertEqual(fit.iterations, 0)
        self.assertTrue(fit.converged)

    def test_a_wrong_law_is_reported_as_wrong(self):
        """The point of a held-out check: it has to be able to fail. A fit error cannot.

        The size is pinned against the closed form rather than a guessed threshold. A law
        1.5x too stiff on a single-segment contact overpredicts every point by exactly half
        its force, so the reported fraction is `0.5 * rms(f) / max(f)` — 34.2% here, not the
        50% the stiffness error would suggest, because the denominator is the peak.
        """
        deltas = np.array([0.002, 0.004, 0.006, 0.008])
        _, force = self._curve(deltas)
        self.assertEqual(contact_segments(self.SPEC, float(deltas.max())), 1)
        fit = validate_ring(self.SPEC, SpringLaw(a=3.0e4), deltas, force)
        expected = 0.5 * float(np.sqrt(np.mean(force**2)) / np.max(force))
        self.assertAlmostEqual(fit.rms_error_fraction, expected, places=12)
        self.assertFalse(fit.ok)

    def test_the_hinge_element_is_validated_when_asked_for(self):
        """Validating the radial-only ring when the scenario will run a hinge would check a
        ring nobody drives. Measured on the claw design, the two differ by more than 100% of
        each other above second-claw engagement, so this is not a formality."""
        deltas = np.array([0.010, 0.012, 0.014])
        _, force = self._curve(deltas)
        hinge = SpringLaw(a=5.0)          # N.m/rad — a nearly free root
        radial_only = validate_ring(self.SPEC, self.LAW, deltas, force)
        hinged = validate_ring(self.SPEC, self.LAW, deltas, force, hinge_law=hinge)
        self.assertAlmostEqual(radial_only.rms_error_fraction, 0.0, places=12)
        self.assertGreater(hinged.rms_error_fraction, 0.05)

    def test_an_unusable_curve_raises_rather_than_returning_a_flattering_number(self):
        with self.assertRaises(FitFailure):
            validate_ring(self.SPEC, self.LAW, np.array([0.002, 0.004]), np.array([1.0]))
        with self.assertRaises(FitFailure):
            validate_ring(self.SPEC, self.LAW, np.array([0.002, 0.004]), np.zeros(2))


class TestTwoDegreeOfFreedomRing(unittest.TestCase):
    """The tangential freedom. Measured on the nominal claw: 24.81 N/mm radial against
    0.1851 N/mm tangential, a factor of 134."""

    SPEC = RingSpec(radius_m=0.085, n_segments=12)
    RADIAL = SpringLaw(a=24807.0)
    TANGENTIAL = SpringLaw(a=185.1)
    #: 12 claws on an 85 mm wheel: the second tip reaches the plate at R(1 - cos 30) = 11.4 mm.
    #: The pitch is ``2π/n``, not ``π/n`` — this constant read ``np.pi / 12`` until 2026-08-09
    #: and evaluated to 2.90 mm, a quarter of the truth. Nothing failed, because the only test
    #: using it halves it first and 1.45 mm is below both thresholds. A wrong constant that
    #: happens to be conservative is still wrong; asserted against the segment grid below so
    #: it cannot drift again.
    SECOND_CLAW_M = 0.085 * (1.0 - np.cos(2.0 * np.pi / 12))

    def test_the_engagement_threshold_matches_the_segment_grid(self):
        """``SECOND_CLAW_M`` restated from the model rather than from a formula in a comment.

        Below it exactly one segment reaches the plate; above it, three.
        """
        self.assertAlmostEqual(self.SECOND_CLAW_M, 0.0113878, places=7)
        neighbour = float(np.abs(segment_angles(self.SPEC))[1])
        self.assertAlmostEqual(neighbour, 2.0 * np.pi / self.SPEC.n_segments, places=12)
        self.assertEqual(contact_segments(self.SPEC, 0.999 * self.SECOND_CLAW_M), 1)
        self.assertEqual(contact_segments(self.SPEC, 1.001 * self.SECOND_CLAW_M), 3)

    def test_a_rigid_tangential_spring_recovers_the_radial_kinematics(self):
        """The limit that makes this a generalisation: infinitely stiff tangentially *is* a
        radial slide, so every compression must equal the geometric penetration exactly.

        Asserted on the kinematics *and*, since #26 closed, on the force: both solvers now
        resolve the plate's normal force as ``f_r/cos θ``, so the rigid-tangential limit must
        reproduce :func:`ring_force_n` and not merely resemble it.
        """
        for delta in (0.006, 0.014, 0.020):
            state = solve_equilibrium_2dof(
                self.SPEC, self.RADIAL, SpringLaw(a=1e6 * self.RADIAL.a), delta
            )
            geometric = penetrations(self.SPEC, delta)
            active = state.in_contact
            with self.subTest(delta=delta):
                self.assertTrue(bool(np.any(active)))
                # rtol is 1e-5, not machine epsilon, and the reason is worth stating: a
                # 1e6 stiffness ratio is stiff, not rigid. The residual splay is ~2 nm,
                # which perturbs the height equation at ~3e-7 relative — so a tighter
                # tolerance would be testing the ratio chosen here, not the solver.
                np.testing.assert_allclose(
                    state.compression_m[active], geometric[active], rtol=1e-5
                )
                self.assertLess(float(np.max(np.abs(state.slip_m))), 1e-8)
                np.testing.assert_allclose(
                    state.force_n,
                    float(ring_force_n(self.SPEC, self.RADIAL, delta)),
                    rtol=1e-5,
                )

    def test_it_is_inert_while_only_one_claw_touches(self):
        """A lone segment sits at theta = 0, where the contact force is purely radial and
        sin(theta) is zero. Nothing to splay, so the answer must be bit-for-bit the radial
        one — this is why the flat-plate fit at design load is unaffected."""
        delta = 0.5 * self.SECOND_CLAW_M
        state = solve_equilibrium_2dof(self.SPEC, self.RADIAL, self.TANGENTIAL, delta)
        self.assertEqual(int(state.in_contact.sum()), 1)
        self.assertEqual(float(np.max(np.abs(state.slip_m))), 0.0)
        self.assertAlmostEqual(
            state.force_n, float(ring_force_n(self.SPEC, self.RADIAL, delta)), places=9
        )

    def test_it_softens_the_ring_once_a_second_claw_engages(self):
        """Past the threshold the off-centre claws splay, so the wheel carries less at the
        same indentation.

        Measured 2026-08-09: **11.7% softer at 12 mm, 48.4% at 18 mm, 57.9% at 25 mm**. An
        earlier record of "3% and 17%" does not reproduce and was almost certainly taken
        before the `_invert` sign bug was fixed — that bug left one side of the ring unable to
        splay, which under-reports exactly this effect. The magnitudes are pinned here, not
        just the direction, so a third set of numbers cannot appear without a test failing.
        """
        # Compared against the *same* solver with a rigid tangential spring, so the comparison
        # isolates the new freedom rather than mixing in any other difference between solvers.
        expected = {0.012: 0.883, 0.018: 0.516, 0.025: 0.421}
        for delta, ratio in expected.items():
            soft = float(ring_force_2dof_n(self.SPEC, self.RADIAL, self.TANGENTIAL, delta))
            stiff = float(ring_force_2dof_n(
                self.SPEC, self.RADIAL, SpringLaw(a=1e6 * self.RADIAL.a), delta
            ))
            with self.subTest(delta=delta):
                self.assertLess(soft, stiff)
                self.assertAlmostEqual(soft / stiff, ratio, places=3)
        delta = 0.018
        state = solve_equilibrium_2dof(self.SPEC, self.RADIAL, self.TANGENTIAL, delta)
        self.assertGreater(int(state.in_contact.sum()), 1)
        self.assertGreater(float(np.max(np.abs(state.slip_m))), 0.0)

    def test_splay_is_antisymmetric_about_the_contact_point(self):
        """A ring is symmetric about the contact point, so the two sides splay opposite ways
        and the total tangential displacement cancels. A non-zero sum would mean the wheel
        was walking sideways under a symmetric load."""
        state = solve_equilibrium_2dof(self.SPEC, self.RADIAL, self.TANGENTIAL, 0.018)
        self.assertAlmostEqual(float(np.sum(state.slip_m)), 0.0, places=12)

    def test_a_banded_ring_is_refused(self):
        with self.assertRaises(ValueError):
            solve_equilibrium_2dof(COUPLED, self.RADIAL, self.TANGENTIAL, 0.004)

    def test_the_tangential_law_resists_both_directions_equally(self):
        law = SpringLaw(a=500.0, b=2.0e5, c=1.0e7)
        for x in (0.001, 0.004):
            self.assertAlmostEqual(
                float(symmetric_force_n(law, x)), -float(symmetric_force_n(law, -x))
            )
        self.assertEqual(float(symmetric_force_n(law, 0.0)), 0.0)

    def test_it_inverts_a_table_with_a_flat_interval(self):
        """A buckled segment carrying constant load has zero tangent over an interval. That
        is a legitimate law and a division by zero for a Newton inverse, which is why the
        solver bisects."""
        flat = TabulatedLaw(knots_m=uniform_knots(0.008, 4),
                            slopes_n_per_m=np.array([12000.0, 0.0, 0.0, 9000.0]))
        state = solve_equilibrium_2dof(self.SPEC, flat, self.TANGENTIAL, 0.018)
        self.assertTrue(np.all(np.isfinite(state.compression_m)))
        self.assertTrue(np.all(np.isfinite(state.slip_m)))
        self.assertGreater(state.force_n, 0.0)


@unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
class TestMjcfHinge(unittest.TestCase):
    """The MJCF realisation of ``solve_equilibrium_hinge``. ``TODO.md`` #27."""

    SPEC = RingSpec(radius_m=0.060, n_segments=12, root_radius_m=0.020)
    RADIAL = SpringLaw(a=24807.0)

    def hinge_law(self):
        from wheelopt.rom.mjcf import hinge_arm_m

        arm = hinge_arm_m(self.SPEC)
        return SpringLaw(a=185.1 * arm * arm), arm

    def test_the_pivot_is_inboard_so_the_moment_arm_is_the_claw_length(self):
        """The correction that took the moment residual from 7e-3 to 6e-11.

        On a plane the contact point is directly under the capsule *centre*, so the horizontal
        lever the floor gets is pivot-to-centre. Pivot at the true root and that is one capsule
        radius short — 19.6% here at 12 segments, 9.8% at 24 — which makes the modelled claw
        stiffer in rotation than the one that was fitted.
        """
        from wheelopt.rom.mjcf import (
            hinge_arm_m,
            hinge_pivot_radius_m,
            segment_body_radius_m,
        )

        capsule = 0.25 * self.SPEC.segment_arc_m
        self.assertAlmostEqual(hinge_pivot_radius_m(self.SPEC),
                               self.SPEC.root_radius_m - capsule, places=12)
        self.assertAlmostEqual(hinge_arm_m(self.SPEC), self.SPEC.claw_length_m, places=12)
        # And the running surface is still exactly at R, which is what must not move.
        self.assertAlmostEqual(segment_body_radius_m(self.SPEC) + capsule,
                               self.SPEC.radius_m, places=12)

    def test_too_few_segments_for_the_hub_is_refused(self):
        """A capsule wider than the hub would put the pivot at or through the axle."""
        from wheelopt.rom.mjcf import hinge_pivot_radius_m

        coarse = RingSpec(radius_m=0.060, n_segments=4, root_radius_m=0.020)
        self.assertGreater(0.25 * coarse.segment_arc_m, coarse.root_radius_m)
        with self.assertRaisesRegex(ValueError, "too few"):
            hinge_pivot_radius_m(coarse)

    def test_a_rootless_or_banded_spec_is_refused(self):
        from wheelopt.rom.mjcf import ring_bodies

        with self.assertRaisesRegex(ValueError, "root"):
            ring_bodies(RingSpec(radius_m=0.060, n_segments=12), tangential="hinge")
        banded = replace(self.SPEC, band_bending_n_per_m=0.8)
        with self.assertRaisesRegex(ValueError, "shear"):
            ring_bodies(banded, tangential="hinge")
        with self.assertRaisesRegex(ValueError, "unknown tangential element"):
            ring_bodies(self.SPEC, tangential="pivot")

    def test_the_joints_compose_so_the_slide_stays_on_the_claw_axis(self):
        """Hinge **before** slide, and MuJoCo carries the slide's axis round with it.

        Get the order backwards and the slide is a fixed direction in the hub frame, which is
        the two-slide element again wearing a hinge. Asserted on the compiled kinematics: at
        90° of rotation the radial joint must move segment 0's capsule along +x, and at any
        rotation the pivot-to-capsule distance must not change.
        """
        from wheelopt.rom.mjcf import build_mjcf, hinge_pivot_radius_m

        model = mujoco.MjModel.from_xml_string(
            build_mjcf(self.SPEC, self.RADIAL, tangential="hinge"))
        data = mujoco.MjData(model)
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "g0")
        pivot = np.array([0.0, 0.0, self.SPEC.radius_m - hinge_pivot_radius_m(self.SPEC)])
        h = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "t0")]
        j = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j0")]
        self.assertLess(h, j, "the hinge must be listed before the slide")

        for phi in (0.0, 0.4, 0.5 * np.pi):
            data.qpos[:] = 0.0
            data.qpos[h] = phi
            mujoco.mj_kinematics(model, data)
            reach = float(np.linalg.norm(data.geom_xpos[geom] - pivot))
            with self.subTest(phi=phi):
                self.assertAlmostEqual(reach, self.SPEC.claw_length_m, places=9)
        data.qpos[h], data.qpos[j] = 0.5 * np.pi, 0.01
        mujoco.mj_kinematics(model, data)
        moved = data.geom_xpos[geom] - pivot
        self.assertAlmostEqual(float(moved[0]), self.SPEC.claw_length_m + 0.01, places=9)
        self.assertAlmostEqual(float(moved[2]), 0.0, places=9)

    def test_the_hinged_tip_stays_inside_the_wheel_where_the_slide_leaves_it(self):
        """The compiled-model form of the #27 discriminator, so the claim is not resting on
        the analytic kinematics alone. Same tangential tip travel, opposite radial answer."""
        from wheelopt.rom.mjcf import build_mjcf

        travel = 0.020
        radius = {}
        for element in ("slide", "hinge"):
            model = mujoco.MjModel.from_xml_string(
                build_mjcf(self.SPEC, self.RADIAL, tangential=element))
            data = mujoco.MjData(model)
            hub = np.array([0.0, 0.0, self.SPEC.radius_m])
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "g0")
            t = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "t0")]
            data.qpos[t] = (travel if element == "slide"
                            else np.arcsin(travel / self.SPEC.claw_length_m))
            mujoco.mj_kinematics(model, data)
            pos = data.geom_xpos[geom] - hub
            self.assertAlmostEqual(float(pos[0]), travel, places=9)
            capsule = 0.25 * self.SPEC.segment_arc_m
            radius[element] = float(np.linalg.norm(pos)) + capsule
        self.assertLess(radius["hinge"], self.SPEC.radius_m)
        self.assertGreater(radius["slide"], self.SPEC.radius_m)

    def test_the_hinged_ring_satisfies_the_analytic_equilibrium(self):
        """MuJoCo's hinged ring against the conditions ``solve_equilibrium_hinge`` imposes.

        The #26 pattern on the new element: read back MuJoCo's **own** ``u_i``, ``φ_i`` and
        ``λ_i`` and check the two stationarity conditions, rather than comparing totals — a
        total confounds the element with the capsule-versus-point contact geometry. MuJoCo is
        told neither condition; it is given two force laws on two joints and a floor.

        Measured 2026-08-09: the radial residual is 9e-11 of the contact force, and the
        moment residual **divided by** the contact force — which is a length, and reads as the
        error in the lever arm — is 6.1e-11 m. Before the pivot was moved inboard that second
        number was 6.7e-3 of the arm, which is how the correction was found.
        """
        from wheelopt.rom.mjcf import build_mjcf

        law, arm = self.hinge_law()
        model = mujoco.MjModel.from_xml_string(
            build_mjcf(self.SPEC, self.RADIAL, indentation_m=0.018, tangential="hinge"))
        data = mujoco.MjData(model)

        def addr(prefix):
            ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}{i}")
                   for i in range(self.SPEC.n_segments)]
            return model.jnt_qposadr[ids], model.jnt_dofadr[ids]

        rq, rd = addr("j")
        tq, td = addr("t")
        for _ in range(40000):
            data.qfrc_applied[rd] = self.RADIAL.force_n(-data.qpos[rq]) - 2.0 * data.qvel[rd]
            data.qfrc_applied[td] = (-symmetric_force_n(law, data.qpos[tq])
                                     - 2.0 * arm * arm * data.qvel[td])
            mujoco.mj_step(model, data)
        self.assertLess(float(np.max(np.abs(data.qvel))), 1e-8, "not settled")

        theta = segment_angles(self.SPEC)
        floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        lam = np.zeros(self.SPEC.n_segments)
        horizontal = 0.0
        wrench = np.zeros(6)
        for c in range(data.ncon):
            con = data.contact[c]
            mujoco.mj_contactForce(model, data, c, wrench)
            world = con.frame.reshape(3, 3).T @ wrench[:3]
            i = int(mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM,
                int(con.geom2 if con.geom1 == floor else con.geom1))[1:])
            lam[i] += float(world[2])
            horizontal += abs(float(world[0]))
        # condim="1", so the plate is genuinely frictionless and the multiplier is vertical.
        self.assertEqual(horizontal, 0.0)

        u, phi = -data.qpos[rq], data.qpos[tq]
        touching = np.flatnonzero(lam > 1e-6)
        self.assertGreater(len(touching), 1, "no off-axis claw carries load")
        for i in touching:
            psi = theta[i] + phi[i]
            with self.subTest(segment=int(i)):
                self.assertAlmostEqual(float(self.RADIAL.force_n(u[i])),
                                       lam[i] * np.cos(psi), delta=1e-9 * lam[i])
                # Normalised by lam alone, so the tolerance is an error in the lever arm:
                # 1e-9 m against a 40 mm claw.
                self.assertAlmostEqual(float(symmetric_force_n(law, phi[i])),
                                       lam[i] * (arm - u[i]) * np.sin(psi),
                                       delta=1e-9 * lam[i])
        # And the claw folds rather than compresses, which is the phenomenon: 24 deg of
        # rotation against 0.09 mm of shortening on the off-axis claws.
        off_axis = [i for i in touching if abs(theta[i]) > 1e-9]
        self.assertTrue(off_axis)
        for i in off_axis:
            self.assertGreater(abs(phi[i]) * arm, 20.0 * abs(u[i]))

    def test_the_static_press_tracks_the_analytic_hinged_ring(self):
        """Totals, where the remaining gap is the capsule-versus-point contact geometry.

        Measured 2026-08-09 across 12/24/48 segments and δ = 2-18 mm: within ±0.7%, and
        within ±0.2% everywhere except the shallowest 24-segment point. The two runs that
        diverged before :func:`stable_timestep_s` was applied to the press are the 18 mm
        points at 24 and 48 segments.
        """
        from wheelopt.rom.mjcf import static_load_deflection

        law, _arm = self.hinge_law()
        deltas = np.array([0.006, 0.012, 0.018])
        simulated = static_load_deflection(self.SPEC, self.RADIAL, deltas,
                                           tangential_law=law, tangential_element="hinge",
                                           settle_s=1.0)
        analytic = ring_force_hinge_n(self.SPEC, self.RADIAL, law, deltas)
        self.assertTrue(bool(np.all(np.isfinite(simulated))))
        np.testing.assert_allclose(simulated, analytic, rtol=0.01)


class TestHingedClawRing(unittest.TestCase):
    """``TODO.md`` #27: the second freedom as a rotation at the root, not a slide at the tip.

    Same wheel and same stiffnesses as :class:`TestTwoDegreeOfFreedomRing`, so the two models
    are compared rather than merely both tested. The hinge law is the tip stiffness referred
    to the root, ``M = k_t L² φ``, which is what makes them the same claw.
    """

    SPEC = RingSpec(radius_m=0.085, n_segments=12, root_radius_m=0.020)
    RADIAL = SpringLaw(a=24807.0)
    TANGENTIAL = SpringLaw(a=185.1)
    HINGE = SpringLaw(a=185.1 * 0.065**2)
    RIGID_HINGE = SpringLaw(a=1e9 * 185.1 * 0.065**2)

    def test_the_claw_length_is_the_radius_less_the_root(self):
        self.assertAlmostEqual(self.SPEC.claw_length_m, 0.065, places=12)
        self.assertAlmostEqual(float(self.HINGE.a), 0.7820475, places=7)

    def test_a_rootless_spec_is_refused(self):
        """A claw hinged at the axle sweeps its tip round a circle of radius ``R``, so it can
        never indent — the model would report a rigid wheel and look like a stiff result."""
        rootless = RingSpec(radius_m=0.085, n_segments=12)
        self.assertEqual(rootless.root_radius_m, 0.0)
        with self.assertRaisesRegex(ValueError, "root"):
            solve_equilibrium_hinge(rootless, self.RADIAL, self.HINGE, 0.018)

    def test_a_banded_ring_is_refused(self):
        banded = replace(self.SPEC, band_bending_n_per_m=0.8, band_hoop_n_per_m=90.0)
        with self.assertRaises(ValueError):
            solve_equilibrium_hinge(banded, self.RADIAL, self.HINGE, 0.004)

    def test_the_root_radius_must_lie_inside_the_wheel(self):
        for bad in (-0.001, 0.085, 0.1):
            with self.subTest(root=bad), self.assertRaises(ValueError):
                RingSpec(radius_m=0.085, n_segments=12, root_radius_m=bad)

    def test_a_rigid_hinge_recovers_the_radial_kinematics(self):
        """The limit that makes this a generalisation: a claw that cannot rotate is a claw on
        a radial slide, so every compression must be the geometric penetration and the total
        must be :func:`ring_force_n` — including its ``f_r/cos θ`` resolution (#26).

        Tolerances are 1e-6, not machine epsilon, because 1e9 times stiffer is stiff and not
        rigid; the residual rotation is 1e-8 rad. Tightening them would be testing the ratio
        chosen here rather than the solver.
        """
        for delta in (0.006, 0.014, 0.020):
            state = solve_equilibrium_hinge(self.SPEC, self.RADIAL, self.RIGID_HINGE, delta)
            geometric = penetrations(self.SPEC, delta)
            active = state.in_contact
            with self.subTest(delta=delta):
                self.assertTrue(bool(np.any(active)))
                np.testing.assert_allclose(state.compression_m[active], geometric[active],
                                           rtol=1e-6)
                self.assertLess(float(np.max(np.abs(state.rotation_rad))), 1e-6)
                self.assertAlmostEqual(
                    state.force_n / float(ring_force_n(self.SPEC, self.RADIAL, delta)),
                    1.0, places=6,
                )

    def test_it_is_inert_while_only_one_claw_touches(self):
        """The lone claw sits at θ = 0, where the vertical force runs straight down its own
        axis and there is no moment to rotate it. Exactly zero, not nearly — which is why the
        flat-plate fit at design load is untouched by any of this."""
        delta = 0.5 * 0.085 * (1.0 - np.cos(2.0 * np.pi / 12))
        state = solve_equilibrium_hinge(self.SPEC, self.RADIAL, self.HINGE, delta)
        self.assertEqual(int(state.in_contact.sum()), 1)
        self.assertEqual(float(np.max(np.abs(state.rotation_rad))), 0.0)
        self.assertAlmostEqual(
            state.force_n, float(ring_force_n(self.SPEC, self.RADIAL, delta)), places=9
        )

    def test_rotation_is_antisymmetric_about_the_contact_point(self):
        """The two sides fold opposite ways and cancel. A non-zero sum is a wheel walking
        sideways under a symmetric load, which is how the ``_invert`` sign bug showed up in
        the slide model."""
        state = solve_equilibrium_hinge(self.SPEC, self.RADIAL, self.HINGE, 0.018)
        self.assertAlmostEqual(float(np.sum(state.rotation_rad)), 0.0, places=12)
        self.assertGreater(float(np.max(np.abs(state.rotation_rad))), 0.0)

    def test_a_hinged_tip_comes_inward_where_a_sliding_one_goes_out(self):
        """**This is #27.** The two elements make opposite predictions about the same number.

        A hinged claw's tip swings on an arc of fixed length, so its distance from the hub
        centre can only *shrink*; a two-slide segment's is ``√((R-u)² + v²)`` and *grows*.
        Outward is the destabilising sign — a claw that lengthens as it splays presses harder
        into the ground, which splays it further — so this is not a tie broken by taste.

        Measured 2026-08-09 on the R 85 mm, 12-claw ring, as a fraction of ``R``:

            δ = 12 mm   hinge -0.0202%   slide -0.0083%
            δ = 18 mm   hinge -0.3956%   slide +0.9568%
            δ = 25 mm   hinge -1.1337%   slide +4.4059%

        At 12 mm the *slide* is inward too, and that is not a counter-example: barely past the
        second claw's engagement the compression still dominates its own outward term. It is
        already the larger of the two, and it crosses zero and keeps going. So the assertion
        that holds at every δ is the **ordering**, and the sign is asserted where the splay is
        real. A test that demanded an outward slide everywhere would be pinning an artefact of
        which δ it happened to sample.
        """
        expected = {0.012: (-0.00020194, -0.00008282),
                    0.018: (-0.00395564, +0.00956792),
                    0.025: (-0.01133712, +0.04405904)}
        for delta, (want_hinge, want_slide) in expected.items():
            hinge = solve_equilibrium_hinge(self.SPEC, self.RADIAL, self.HINGE, delta)
            slide = solve_equilibrium_2dof(self.SPEC, self.RADIAL, self.TANGENTIAL, delta)
            r_h = tip_radius_hinge_m(self.SPEC, hinge.compression_m, hinge.rotation_rad)
            r_s = tip_radius_slide_m(self.SPEC, slide.compression_m, slide.slip_m)
            got_hinge = float(np.max(r_h[hinge.in_contact])) / self.SPEC.radius_m - 1.0
            got_slide = float(np.max(r_s[slide.in_contact])) / self.SPEC.radius_m - 1.0
            with self.subTest(delta=delta):
                # No hinged tip may EVER exceed R -- that one is by construction and holds for
                # every segment, in or out of contact.
                self.assertLessEqual(float(np.max(r_h)), self.SPEC.radius_m + 1e-12)
                self.assertLess(got_hinge, 0.0)
                self.assertGreater(got_slide, got_hinge)
                self.assertAlmostEqual(got_hinge, want_hinge, places=7)
                self.assertAlmostEqual(got_slide, want_slide, places=7)
        # And where the splay is real, the sliding tip is outside the wheel it belongs to.
        for delta in (0.018, 0.025):
            slide = solve_equilibrium_2dof(self.SPEC, self.RADIAL, self.TANGENTIAL, delta)
            r_s = tip_radius_slide_m(self.SPEC, slide.compression_m, slide.slip_m)
            self.assertGreater(float(np.max(r_s)), self.SPEC.radius_m)

    def test_the_two_elements_agree_on_force_and_disagree_on_geometry(self):
        """Why the flat-plate work survives #27 and the driven work does not.

        The vertical reaction differs by 0.013% at 12 mm, 0.66% at 18 mm and 1.43% at 25 mm —
        so every fit, every ``F(δ)`` and every static number taken with the slide stands, and
        the flat-plate work does not have to be redone. What differs is where the tips are,
        which is what a rolling contact depends on and a plate does not.
        """
        expected = {0.012: 0.999870, 0.018: 0.993406, 0.025: 0.985655}
        for delta, ratio in expected.items():
            hinge = float(ring_force_hinge_n(self.SPEC, self.RADIAL, self.HINGE, delta))
            slide = float(ring_force_2dof_n(self.SPEC, self.RADIAL, self.TANGENTIAL, delta))
            with self.subTest(delta=delta):
                self.assertAlmostEqual(hinge / slide, ratio, places=6)

    def test_it_softens_the_ring_once_a_second_claw_engages(self):
        """Magnitudes pinned, as for the slide model, so a third set cannot appear quietly.

        Measured 2026-08-09 against the same solver with a rigid hinge, so the comparison
        isolates the freedom rather than mixing in a difference between two solvers.
        """
        expected = {0.012: 0.8828, 0.018: 0.5126, 0.025: 0.4148}
        for delta, ratio in expected.items():
            soft = float(ring_force_hinge_n(self.SPEC, self.RADIAL, self.HINGE, delta))
            stiff = float(ring_force_hinge_n(self.SPEC, self.RADIAL, self.RIGID_HINGE, delta))
            with self.subTest(delta=delta):
                self.assertLess(soft, stiff)
                self.assertAlmostEqual(soft / stiff, ratio, places=4)

    def test_the_claw_never_rotates_past_the_plate(self):
        """``ψ = θ + φ`` is bounded by ``arccos(c/L)``, where the claw has stood back up to
        full length. A rotation past that would mean the claw stretching to stay in contact,
        which the radial law has no branch for."""
        for delta in (0.012, 0.018, 0.025, 0.035):
            state = solve_equilibrium_hinge(self.SPEC, self.RADIAL, self.HINGE, delta)
            active = state.in_contact
            psi = np.abs(segment_angles(self.SPEC) + state.rotation_rad)[active]
            with self.subTest(delta=delta):
                self.assertTrue(bool(np.all(psi < 0.5 * np.pi)))
                self.assertTrue(bool(np.all(state.compression_m[active] >= -1e-15)))
                self.assertTrue(bool(np.all(state.contact_force_n >= 0.0)))

    def test_it_inverts_a_table_with_a_flat_interval(self):
        """A buckled claw carrying constant load has zero tangent over an interval — a
        legitimate law, and a division by zero for anything Newton-shaped."""
        flat = TabulatedLaw(knots_m=uniform_knots(0.008, 4),
                            slopes_n_per_m=np.array([12000.0, 0.0, 0.0, 9000.0]))
        state = solve_equilibrium_hinge(self.SPEC, flat, self.HINGE, 0.018)
        self.assertTrue(np.all(np.isfinite(state.compression_m)))
        self.assertTrue(np.all(np.isfinite(state.rotation_rad)))
        self.assertGreater(state.force_n, 0.0)

    def test_ring_for_design_supplies_the_root(self):
        """Derived from ``hub_radius_mm``, so only a hand-built spec can be missing it."""
        spec = ring_for_design(replace(TINY_PARAMS, rim_thickness_mm=0.0), TPU, n_segments=12)
        self.assertAlmostEqual(spec.root_radius_m, 0.020, places=12)
        self.assertAlmostEqual(spec.claw_length_m, 0.040, places=12)
        banded = ring_for_design(TINY_PARAMS, TPU, n_segments=12)
        self.assertAlmostEqual(banded.root_radius_m, 0.020, places=12)


class TestTipEquivalentLaw(unittest.TestCase):
    """A hinge law in tip coordinates, so the linear rules can be reused without rewriting."""

    def test_it_is_the_change_of_variables_and_nothing_else(self):
        hinge = SpringLaw(a=0.78, b=3.0, c=11.0)
        arm = 0.065
        tip = TipEquivalentLaw(hinge, arm)
        for s in (0.001, 0.010, 0.030):
            with self.subTest(s=s):
                self.assertAlmostEqual(float(tip.force_n(s)),
                                       float(hinge.force_n(s / arm)) / arm, places=12)
                self.assertAlmostEqual(float(tip.stiffness_n_per_m(s)),
                                       float(hinge.stiffness_n_per_m(s / arm)) / arm**2,
                                       places=9)

    def test_a_linear_hinge_gives_back_the_tip_stiffness_it_was_built_from(self):
        """The round trip that matters: ``M = k_t a² φ`` must come back as ``k_t``."""
        arm, k_t = 0.065, 185.1
        tip = TipEquivalentLaw(SpringLaw(a=k_t * arm * arm), arm)
        self.assertAlmostEqual(float(tip.stiffness_n_per_m(0.0)), k_t, places=9)

    def test_a_zero_arm_is_refused(self):
        with self.assertRaises(ValueError):
            TipEquivalentLaw(SpringLaw(a=1.0), 0.0)


class TestHingeLawFromTipCurve(unittest.TestCase):
    """Turning a measured ``TIP_TANGENTIAL`` sweep into a moment-rotation law."""

    L = 0.040

    def test_a_rigid_bar_round_trips(self):
        """Build the law from a curve, then check a rigid bar on that spring sits exactly
        where the curve says under exactly the force the curve says.

        This is the definition of the law restated as an assertion, and it is worth asserting
        because both corrections — ``arcsin`` for the arc and ``cos φ`` for the shortening
        moment arm — are second order and therefore easy to drop without anything looking
        wrong until the deflections get large.
        """
        s = np.array([0.004, 0.010, 0.020, 0.030, 0.036])
        f = np.array([0.85, 2.4, 6.1, 13.0, 21.3])
        law = hinge_law_from_tip_curve(s, f, self.L)
        for si, fi in zip(s, f, strict=True):
            phi = np.arcsin(si / self.L)
            with self.subTest(s=si):
                # Moment balance on the bar: M(phi) = F * L cos(phi).
                self.assertAlmostEqual(float(law.force_n(phi)), fi * self.L * np.cos(phi),
                                       places=12)

    def test_the_corrections_are_not_negligible_at_a_claw_length(self):
        """If both were dropped the law would be ``M = F·s`` at ``φ = s/L``. At 36 mm on a
        40 mm claw that is 44% wrong in the moment and 42% wrong in the rotation, so a small
        sweep cannot tell you whether the code is right."""
        s, f = 0.036, 21.3
        law = hinge_law_from_tip_curve(np.array([s]), np.array([f]), self.L)
        phi = float(np.arcsin(s / self.L))
        naive_phi = s / self.L
        self.assertAlmostEqual(np.degrees(phi), 64.158, places=3)
        self.assertGreater(phi / naive_phi, 1.24)
        self.assertLess(float(law.force_n(phi)) / (f * s), 0.57)

    def test_it_refuses_a_sweep_longer_than_the_claw(self):
        with self.assertRaisesRegex(FitFailure, "rigid bar|arcsin"):
            hinge_law_from_tip_curve(np.array([0.02, 0.045]), np.array([1.0, 2.0]), self.L)

    def test_it_refuses_a_non_positive_claw_length(self):
        with self.assertRaises(FitFailure):
            hinge_law_from_tip_curve(np.array([0.01]), np.array([1.0]), 0.0)

    def test_the_kinematics_check_predicts_inward_travel(self):
        """The falsifiable consequence: a hinged bar's tip must come in by ``L(1 - cos φ)``.
        The sweep leaves that DOF free and measures it, so the check has something to be
        wrong against — which is the point of it existing at all."""
        s = np.array([0.010, 0.020, 0.036])
        measured, predicted = hinge_kinematics_check(s, np.zeros_like(s), self.L)
        np.testing.assert_allclose(measured, 0.0)
        np.testing.assert_allclose(
            predicted, self.L * (1.0 - np.cos(np.arcsin(s / self.L))), rtol=1e-12
        )
        # A slide would go the other way, by sqrt(L^2 + s^2) - L. Both are second order; they
        # are within 3% of each other at 10 mm and 1.63x apart at 36 mm, so the *sign* is the
        # discriminator at every deflection and the size only becomes one near the tip's limit.
        outward = np.hypot(self.L, s) - self.L
        self.assertTrue(bool(np.all(outward > 0.0)))
        np.testing.assert_allclose(predicted / outward, [1.03177, 1.13505, 1.63339], rtol=1e-5)

    def test_shape_mismatch_is_a_fit_failure(self):
        with self.assertRaises(FitFailure):
            hinge_kinematics_check(np.array([0.01, 0.02]), np.array([0.001]), self.L)


class TestRideHarshness(unittest.TestCase):
    """TODO #19: how few claws a bandless wheel may have, derived rather than inherited."""

    RADIUS = 0.085
    LOAD_N = 24.5

    def law(self, k_n_per_mm: float) -> SpringLaw:
        return SpringLaw(a=k_n_per_mm * 1e3)

    def test_the_polygon_drop_is_the_rigid_limit_and_uses_half_the_pitch(self):
        self.assertAlmostEqual(polygon_drop_m(0.085, 12), 0.0028963, places=7)
        self.assertAlmostEqual(polygon_drop_m(0.085, 4), 0.0248959, places=7)
        # Falls as 1/n^2 in the limit -- doubling the tips quarters the drop, which is why
        # the criterion bites so hard at low n and disappears at high n. Approached from
        # below: 3.932 at n = 6, 3.983 at 12, 3.996 at 24.
        for n, ratio in ((6, 3.9319), (12, 3.9829), (24, 3.9957)):
            with self.subTest(n=n):
                self.assertAlmostEqual(
                    polygon_drop_m(0.085, n) / polygon_drop_m(0.085, 2 * n), ratio, places=4
                )

    def test_it_refuses_a_degenerate_wheel(self):
        for radius, n in ((0.0, 12), (-0.1, 12), (0.085, 2)):
            with self.subTest(radius=radius, n=n), self.assertRaises(ValueError):
                polygon_drop_m(radius, n)

    def test_a_phase_rotates_the_ring_under_the_contact_point(self):
        """Half a pitch puts the contact point between two segments rather than on one, which
        is the whole polygon effect. At a full pitch the ring is back where it started."""
        spec = RingSpec(radius_m=self.RADIUS, n_segments=12, root_radius_m=0.020)
        pitch = 2.0 * np.pi / 12
        np.testing.assert_allclose(np.sort(segment_angles(spec, pitch)),
                                   np.sort(segment_angles(spec)), atol=1e-12)
        half = segment_angles(spec, 0.5 * pitch)
        self.assertFalse(np.any(np.isclose(half, 0.0, atol=1e-9)),
                         "half a pitch must put no segment at the contact point")
        self.assertAlmostEqual(float(np.min(np.abs(half))), 0.5 * pitch, places=12)

    def test_a_phase_is_refused_on_a_banded_ring(self):
        with self.assertRaisesRegex(ValueError, "bandless"):
            ring_force_n(COUPLED, LAW, 0.004, phase_rad=0.1)
        # Zero phase on a banded ring is the ordinary call and must still work.
        self.assertGreater(float(ring_force_n(COUPLED, LAW, 0.004, phase_rad=0.0)), 0.0)

    def test_compliance_smooths_the_polygon_and_more_tips_smooth_it_further(self):
        """A stiff claw: the ripple sits below the rigid drop and falls with ``n``.

        Measured on R 85 mm at 24.5 N with a linear 13.5 N/mm claw: ripple/drop 0.945 at
        4 tips, 0.839 at 8, 0.674 at 12, 0.493 at 16. The wheel deflects into the gap, so it
        never sees the whole polygon.
        """
        law = self.law(13.5)
        previous = float("inf")
        for n in (4, 8, 12, 16):
            spec = RingSpec(radius_m=self.RADIUS, n_segments=n, root_radius_m=0.020)
            ripple, lo, hi = ride_height_ripple_m(spec, law, self.LOAD_N)
            with self.subTest(n=n):
                self.assertLess(ripple, polygon_drop_m(self.RADIUS, n))
                self.assertLess(ripple, previous)
                self.assertAlmostEqual(hi - lo, ripple, places=12)
            previous = ripple

    def test_a_coarse_wheel_unloads_a_claw_once_a_pitch(self):
        """The criterion, and the reason the claw count went **up** rather than down.

        When the peak-to-peak axle movement reaches the wheel's own static deflection, the
        trailing claw has left the ground entirely. On a linear 13.5 N/mm claw the crossing is
        just past 12 tips — ripple/δ is 12.97 at 4, 2.99 at 8, 1.08 at 12, 0.687 at 14 and
        0.444 at 16. The measured claws (fitted tabulated laws, 3.7 and 13.5 N/mm) cross at
        10-12, so the family's answer is **n >= 12**, and it is not sensitive to which of them
        you ask.
        """
        law = self.law(13.5)
        crossing = {}
        for n in (4, 8, 12, 14, 16):
            spec = RingSpec(radius_m=self.RADIUS, n_segments=n, root_radius_m=0.020)
            ripple, _lo, _hi = ride_height_ripple_m(spec, law, self.LOAD_N)
            lo, hi = 0.0, 0.9 * self.RADIUS
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if float(ring_force_n(spec, law, mid)) < self.LOAD_N:
                    lo = mid
                else:
                    hi = mid
            crossing[n] = ripple / (0.5 * (lo + hi))
        self.assertAlmostEqual(crossing[4], 12.968, places=3)
        self.assertAlmostEqual(crossing[8], 2.992, places=3)
        self.assertGreater(crossing[12], 1.0)
        self.assertLess(crossing[14], 1.0)
        self.assertLess(crossing[16], 0.5)

    def test_a_wheel_that_cannot_carry_the_load_reports_infinite_ripple(self):
        """Not an exception and not a quiet zero: a wheel that bottoms out at some phase is a
        result, and ``inf`` is the honest one."""
        spec = RingSpec(radius_m=self.RADIUS, n_segments=4, root_radius_m=0.020)
        ripple, lo, hi = ride_height_ripple_m(spec, self.law(0.05), self.LOAD_N)
        self.assertEqual(ripple, float("inf"))
        self.assertTrue(np.isnan(lo) and np.isnan(hi))

    def test_it_refuses_a_banded_spec_and_a_non_positive_load(self):
        with self.assertRaises(ValueError):
            ride_height_ripple_m(COUPLED, LAW, 24.5)
        spec = RingSpec(radius_m=self.RADIUS, n_segments=12, root_radius_m=0.020)
        with self.assertRaises(ValueError):
            ride_height_ripple_m(spec, self.law(13.5), 0.0)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
