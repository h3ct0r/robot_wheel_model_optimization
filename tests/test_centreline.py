"""Spoke centreline and outline geometry — pure numpy, no OCCT required."""

from __future__ import annotations

import unittest
from dataclasses import replace

import numpy as np

from wheelopt.cad.centreline import (
    min_gap_between_spokes,
    spoke_centreline,
    spoke_outline,
)
from wheelopt.cad.params import SpokeProfile, WheelParams


class TestCentrelineEndpoints(unittest.TestCase):
    def setUp(self):
        self.p = WheelParams()

    def test_starts_at_hub_and_ends_at_rim(self):
        for profile in SpokeProfile:
            with self.subTest(profile=profile.value):
                p = WheelParams(spoke_profile=profile)
                c = spoke_centreline(p, 0)
                self.assertAlmostEqual(
                    float(np.linalg.norm(c[0])), p.hub_radius_mm, places=9
                )
                self.assertAlmostEqual(
                    float(np.linalg.norm(c[-1])), p.rim_inner_radius_mm, places=9
                )

    def test_overlap_extends_both_ends(self):
        overlap = 1.5
        c = spoke_centreline(self.p, 0, overlap_mm=overlap)
        self.assertAlmostEqual(
            float(np.linalg.norm(c[0])), self.p.hub_radius_mm - overlap, places=9
        )
        self.assertAlmostEqual(
            float(np.linalg.norm(c[-1])), self.p.rim_inner_radius_mm + overlap, places=9
        )

    def test_sample_count_matches_params(self):
        p = WheelParams(spoke_samples=17)
        self.assertEqual(spoke_centreline(p, 0).shape, (17, 2))


class TestProfiles(unittest.TestCase):
    def test_straight_spoke_is_radial(self):
        """Every point of a straight spoke lies on the same ray from the centre."""
        p = WheelParams(spoke_profile=SpokeProfile.STRAIGHT)
        c = spoke_centreline(p, 3)
        angles = np.arctan2(c[:, 1], c[:, 0])
        np.testing.assert_allclose(angles, angles[0], atol=1e-12)

    def test_straight_spoke_ignores_curvature(self):
        a = spoke_centreline(WheelParams(spoke_profile=SpokeProfile.STRAIGHT), 0)
        b = spoke_centreline(
            WheelParams(spoke_profile=SpokeProfile.STRAIGHT, spoke_curvature_1_per_mm=0.02), 0
        )
        np.testing.assert_allclose(a, b, atol=1e-15)

    def test_curved_spoke_bulges_one_way(self):
        """A CURVED spoke's tangential offset is single-signed and peaks at mid-span."""
        p = WheelParams(spoke_profile=SpokeProfile.CURVED, spoke_curvature_1_per_mm=0.004)
        c = spoke_centreline(p, 0)
        theta = 0.0
        tangential = np.array([-np.sin(theta), np.cos(theta)])
        offsets = c @ tangential
        self.assertTrue(np.all(offsets >= -1e-12), "curved spoke changed sign")
        self.assertEqual(int(np.argmax(offsets)), len(offsets) // 2)
        self.assertAlmostEqual(float(np.max(offsets)), p.spoke_sagitta_mm, places=9)

    def test_s_curve_changes_sign(self):
        p = WheelParams(spoke_profile=SpokeProfile.S_CURVE, spoke_curvature_1_per_mm=0.004)
        c = spoke_centreline(p, 0)
        tangential = np.array([0.0, 1.0])
        offsets = c @ tangential
        self.assertGreater(float(np.max(offsets)), 0.0)
        self.assertLess(float(np.min(offsets)), 0.0)

    def test_curvature_sign_flips_bulge_direction(self):
        pos = spoke_centreline(WheelParams(spoke_curvature_1_per_mm=+0.004), 0)
        neg = spoke_centreline(WheelParams(spoke_curvature_1_per_mm=-0.004), 0)
        tangential = np.array([0.0, 1.0])
        self.assertGreater(float(np.max(pos @ tangential)), 0.0)
        self.assertLess(float(np.min(neg @ tangential)), 0.0)

    def test_curved_is_longer_than_straight(self):
        """Curvature buys compliance by adding arc length at fixed span."""

        def arc_length(c):
            return float(np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1)))

        straight = arc_length(
            spoke_centreline(WheelParams(spoke_profile=SpokeProfile.STRAIGHT), 0)
        )
        curved = arc_length(
            spoke_centreline(
                WheelParams(spoke_profile=SpokeProfile.CURVED, spoke_curvature_1_per_mm=0.01), 0
            )
        )
        self.assertGreater(curved, straight)


class TestRotationalSymmetry(unittest.TestCase):
    def test_spokes_are_rotated_copies(self):
        p = WheelParams(n_spokes=12)
        a = spoke_centreline(p, 0)
        b = spoke_centreline(p, 5)
        phi = 5 * p.spoke_pitch_angle_rad
        rot = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
        np.testing.assert_allclose(b, a @ rot.T, atol=1e-12)

    def test_full_revolution_wraps(self):
        p = WheelParams(n_spokes=8)
        np.testing.assert_allclose(
            spoke_centreline(p, 8), spoke_centreline(p, 0), atol=1e-12
        )


class TestOutline(unittest.TestCase):
    def test_outline_is_closed_pair_of_offsets(self):
        p = WheelParams()
        outline = spoke_outline(p, 0)
        self.assertEqual(outline.shape, (2 * p.spoke_samples, 2))

    def test_outline_width_equals_spoke_thickness(self):
        """Opposite points across the centreline are one thickness apart."""
        p = WheelParams(spoke_thickness_mm=2.4)
        n = p.spoke_samples
        outline = spoke_outline(p, 0)
        left, right = outline[:n], outline[n:][::-1]
        widths = np.linalg.norm(left - right, axis=1)
        np.testing.assert_allclose(widths, p.spoke_thickness_mm, atol=1e-9)

    def test_outline_does_not_self_intersect_at_nominal_curvature(self):
        """Consecutive outline points must advance monotonically, not fold back."""
        p = WheelParams(spoke_profile=SpokeProfile.CURVED, spoke_curvature_1_per_mm=0.004)
        n = p.spoke_samples
        left = spoke_outline(p, 0)[:n]
        steps = np.linalg.norm(np.diff(left, axis=0), axis=1)
        self.assertTrue(np.all(steps > 1e-6), "outline folded back on itself")


class TestInterspokeGap(unittest.TestCase):
    def test_true_gap_is_positive_for_default_design(self):
        self.assertGreater(min_gap_between_spokes(WheelParams()), 0.0)

    def test_more_spokes_reduces_gap(self):
        few = min_gap_between_spokes(WheelParams(n_spokes=8))
        many = min_gap_between_spokes(WheelParams(n_spokes=28))
        self.assertGreater(few, many)

    def test_thicker_spokes_reduce_gap(self):
        thin = min_gap_between_spokes(WheelParams(spoke_thickness_mm=1.6))
        thick = min_gap_between_spokes(WheelParams(spoke_thickness_mm=3.5))
        self.assertGreater(thin, thick)

    def test_matches_chord_formula_for_straight_spokes(self):
        """For straight radial spokes the gap is closed-form, so check against it.

        Adjacent centrelines are rays separated by the pitch angle; their closest approach
        over the spoke span is at the hub, a chord of ``2 r sin(d/2)``. Each outline is
        offset half a thickness toward the other, and that offset is tangential, so it
        projects onto the chord by ``cos(d/2)``.

        The thickness is pinned rather than left at the default: the closed form describes
        two *disjoint* outlines, and at 32 spokes of the default 7 mm they overlap, where
        it predicts a negative gap and the measured minimum distance is zero.
        """
        for n in (8, 12, 16, 24, 32):
            p = WheelParams(
                n_spokes=n, spoke_thickness_mm=2.0, spoke_profile=SpokeProfile.STRAIGHT
            )
            half_pitch = p.spoke_pitch_angle_rad / 2.0
            expected = (
                2.0 * p.hub_radius_mm * np.sin(half_pitch)
                - p.spoke_thickness_mm * np.cos(half_pitch)
            )
            with self.subTest(n=n):
                self.assertAlmostEqual(min_gap_between_spokes(p), expected, places=6)

    def test_no_analytic_approximation_survives_on_params(self):
        """Regression guard: the removed approximation must not come back.

        It over-reported the gap at low spoke counts and under-reported it at high ones,
        so it was permissive in one region of the design space and silently rejected
        feasible designs in another. See the note in params.py.
        """
        self.assertFalse(
            hasattr(WheelParams(), "min_interspoke_gap_mm"),
            "analytic gap approximation reintroduced; use min_gap_between_spokes instead",
        )

    def test_gap_check_is_fast_enough_for_screening(self):
        """The exact check must stay in the microsecond range to belong in a pre-filter."""
        import time

        p = WheelParams()
        min_gap_between_spokes(p)  # warm up
        start = time.perf_counter()
        for _ in range(200):
            min_gap_between_spokes(p)
        per_call_us = (time.perf_counter() - start) / 200 * 1e6
        self.assertLess(per_call_us, 2000.0, f"gap check took {per_call_us:.0f} us/call")


if __name__ == "__main__":
    unittest.main()


class TestClawTaper(unittest.TestCase):
    """The taper is what makes a claw a claw. It is also a new way to be silently wrong."""

    ROOT_MM = 8.0
    #: Banded, so the tip is *not* clipped to the running surface. The clip is a separate,
    #: real effect — it narrows a bandless tip by 3.4% on this design — and measuring the
    #: taper through it would conflate the two.
    CLAW = WheelParams(outer_radius_mm=85.0, width_mm=45.0, rim_thickness_mm=3.0,
                       hub_radius_mm=22.0, n_spokes=6, spoke_thickness_mm=ROOT_MM,
                       claw_taper_ratio=0.35, spoke_curvature_1_per_mm=0.012)
    STRUT = replace(CLAW, claw_taper_ratio=1.0)

    def _widths(self, params):
        """Section width along the span, from the outline itself rather than the formula."""
        outline = spoke_outline(params, 0)
        half = len(outline) // 2
        left, right = outline[:half], outline[half:][::-1]
        return np.linalg.norm(left - right, axis=1)

    def _arc(self, params):
        centre = spoke_centreline(params, 0)
        steps = np.linalg.norm(np.diff(centre, axis=0), axis=1)
        return np.concatenate([[0.0], np.cumsum(steps)])

    def test_an_untapered_spoke_is_unchanged(self):
        # The default must reproduce the uniform strut exactly, or every result that
        # predates the claw work silently moves. Interior points only: the outline uses
        # one-sided normals at the two ends, so the end sections are not perpendicular.
        widths = self._widths(self.STRUT)[1:-1]
        np.testing.assert_allclose(widths, self.ROOT_MM, rtol=2e-3)

    def test_the_section_narrows_monotonically_toward_the_tip(self):
        widths = self._widths(self.CLAW)
        self.assertTrue(bool(np.all(np.diff(widths) <= 1e-9)))
        self.assertAlmostEqual(widths[0], self.ROOT_MM, delta=0.02)
        self.assertAlmostEqual(widths[-1], self.ROOT_MM * 0.35, delta=0.02)

    def test_the_taper_is_linear_in_arc_length(self):
        """Not linear in point index: a curved centreline is not sampled at uniform distance.

        Interpolating on the index would put the half-thickness station somewhere other than
        the half-length station, by an amount that changes with the profile and the
        curvature — a wrong number that moves when an unrelated parameter moves.
        """
        arc = self._arc(self.CLAW)
        widths = self._widths(self.CLAW)
        expected = self.ROOT_MM * (1.0 + (0.35 - 1.0) * arc / arc[-1])
        np.testing.assert_allclose(widths[1:-1], expected[1:-1], rtol=3e-3)

    def test_a_claw_carries_less_material_than_the_strut_it_came_from(self):
        self.assertLess(self._widths(self.CLAW).sum(), self._widths(self.STRUT).sum())

    def test_a_bandless_tip_stays_on_the_running_surface(self):
        # Bandless, so the tips are the running surface and the FEA tread node set is
        # |r - R| < 0.1 mm. A tip pushed outside it would never be offered to the contact
        # search — the failure the clip in spoke_outline exists to prevent. Tapering must
        # not reintroduce it.
        bandless = replace(self.CLAW, rim_thickness_mm=0.0)
        outline = spoke_outline(bandless, 0)
        self.assertLessEqual(float(np.linalg.norm(outline, axis=1).max()),
                             bandless.outer_radius_mm + 1e-9)

    def test_the_clip_narrows_a_bandless_tip_and_that_is_recorded(self):
        # Measured, not assumed: the clip pulls the outboard tip corner radially back onto
        # the running surface, so a bandless claw's contact width is a few percent under
        # taper x root. Pinned so that a change to either mechanism shows up here.
        bandless = replace(self.CLAW, rim_thickness_mm=0.0)
        clipped = self._widths(bandless)[-1]
        self.assertLess(clipped, self.ROOT_MM * 0.35)
        self.assertGreater(clipped, 0.90 * self.ROOT_MM * 0.35)


class TestLClaw(unittest.TestCase):
    """The tangential foot at the tip — family ``T7L``, ``tip_hook_mm != 0``."""

    BASE = WheelParams(outer_radius_mm=60.0, width_mm=45.0, rim_thickness_mm=0.0,
                       n_spokes=12, spoke_thickness_mm=6.0, claw_taper_ratio=0.6,
                       spoke_phase_deg=-90.0)

    def hooked(self, hook_mm: float = 12.0, **kwargs) -> WheelParams:
        return replace(self.BASE, tip_hook_mm=hook_mm, **kwargs)

    def test_zero_hook_is_byte_for_byte_the_plain_claw(self):
        """The default, and the thing that lets this land without moving any existing result.
        A new field that perturbs designs which do not use it would invalidate every fit on
        record through `design_hash`."""
        plain = spoke_centreline(self.BASE, 0)
        still_plain = spoke_centreline(replace(self.BASE, tip_hook_mm=0.0), 0)
        np.testing.assert_array_equal(plain, still_plain)

    def test_the_foot_follows_the_circle_rather_than_a_chord(self):
        """The reason the foot is built in polar rather than in the spoke's local Cartesian
        frame. A straight foot at constant local ``u`` is a chord, and a 12 mm chord of a
        60 mm circle stands 1.2 mm proud of it — so the foot would pierce the running surface
        and the outline clip would then eat it from outside until, on a tapered tip, the
        outline crossed itself."""
        centre = spoke_centreline(self.hooked(20.0), 0)
        radii = np.linalg.norm(centre, axis=1)
        foot = radii[radii > radii.max() - 1e-9]
        self.assertGreaterEqual(len(foot), 2, "there should be a run of constant radius")
        # Every point of the foot at one radius, to machine precision.
        self.assertLess(float(np.ptp(radii[-8:])), 1e-9)

    def test_the_outer_face_of_the_foot_lands_on_the_running_surface(self):
        """Not the centreline: the centreline sits half a tip thickness inside, so that the
        material — which is what touches the ground — ends exactly at ``outer_radius_mm``."""
        params = self.hooked(12.0)
        outline = spoke_outline(params, 0)
        reach = float(np.linalg.norm(outline, axis=1).max())
        self.assertLessEqual(reach, params.outer_radius_mm + 1e-9)
        self.assertGreater(reach, params.outer_radius_mm - 0.05)

    def test_the_bend_is_wide_enough_that_the_offset_cannot_invert(self):
        """The one geometric condition an offset right angle has to satisfy. Inside a bend of
        centreline radius rho the offset face has radius ``rho - h``; at ``rho <= h`` the
        outline turns inside out, which OCCT may accept into a solid with a reversed patch."""
        for taper in (0.25, 0.6, 1.0):
            params = self.hooked(12.0, claw_taper_ratio=taper)
            with self.subTest(taper=taper):
                self.assertGreater(params.hook_bend_radius_mm,
                                   0.5 * params.tip_thickness_mm)

    def test_a_short_hook_is_all_fillet_and_the_bend_is_capped(self):
        """Otherwise the bend would be wider than the foot it is bending into, and the arc
        would double back past the tip."""
        params = self.hooked(1.0)
        self.assertLessEqual(params.hook_bend_radius_mm, 0.5 * abs(params.tip_hook_mm))

    def test_the_sign_reflects_the_claw_about_its_own_ray(self):
        """A mirror, and exactly a mirror — so the curvature has to flip with it. Flipping the
        foot alone on a bowed leg gives a C against an S, which are two different claws; the
        CAD battery has the volume difference (2.3e-5) that makes that concrete."""
        straight = replace(self.BASE, spoke_curvature_1_per_mm=0.0)
        left = spoke_centreline(replace(straight, tip_hook_mm=+14.0), 0)
        right = spoke_centreline(replace(straight, tip_hook_mm=-14.0), 0)
        # Spoke 0 at phase -90 lies along -y, so its ray is the y axis and the mirror is x.
        np.testing.assert_allclose(left[:, 0], -right[:, 0], atol=1e-9)
        np.testing.assert_allclose(left[:, 1], right[:, 1], atol=1e-9)

    def test_the_hook_lengthens_the_claw_by_about_what_was_asked_for(self):
        """`tip_hook_mm` is an arc length along the running surface, so the claw's total
        centreline should grow by roughly that much — not by the chord, and not by twice it."""
        short = spoke_centreline(self.BASE, 0)
        long = spoke_centreline(self.hooked(15.0), 0)
        grew = (np.linalg.norm(np.diff(long, axis=0), axis=1).sum()
                - np.linalg.norm(np.diff(short, axis=0), axis=1).sum())
        self.assertAlmostEqual(grew, 15.0, delta=1.5)

    def test_the_taper_continues_into_the_foot(self):
        """Thickness is linear in arc length from the root, and the foot is arc length like
        any other, so the thinnest material is the end of the foot. That is what makes it
        conform — a foot at root thickness would be a rigid paddle."""
        from wheelopt.cad.centreline import _thickness_profile_mm

        params = self.hooked(15.0)
        thickness = _thickness_profile_mm(params, spoke_centreline(params, 0))
        self.assertLess(thickness[-1], thickness[0])
        self.assertAlmostEqual(thickness[-1] / thickness[0], params.claw_taper_ratio,
                               places=9)

    def test_feet_do_not_run_into_the_next_claw_at_a_screened_length(self):
        """`hook_reach` and `interspoke_gap` are meant to agree. This checks the geometry the
        second one measures rather than the formula the first one uses."""
        self.assertGreater(min_gap_between_spokes(self.hooked(12.0)), 0.5)

    def test_a_foot_shrinks_the_polygon_drop_towards_zero(self):
        """The point of the topology, as a closed form: contact over an arc means the axle
        only falls across the gap between feet."""
        drops = [self.hooked(h).polygon_drop_mm if h else self.BASE.polygon_drop_mm
                 for h in (0.0, 6.0, 12.0, 20.0)]
        self.assertEqual(drops, sorted(drops, reverse=True))
        self.assertLess(drops[-1], 0.2 * drops[0])

    def test_feet_that_meet_leave_no_drop_at_all(self):
        """`R(1 - cos(pi/n - beta/2))` goes negative inside the cosine once the arcs overlap,
        and a negative half-gap would give a *negative* drop — a plausible number, in the
        right units, describing a wheel whose axle rises between claws."""
        touching = self.hooked(2.0 * np.pi * 60.0 / 12.0)
        self.assertEqual(touching.polygon_drop_mm, 0.0)
