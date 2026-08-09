"""The bandless topology: ``rim_thickness_mm == 0``, spoke tips as the running surface.

Pure numpy — no OCCT. The solid-level checks (single solid, bounding box, watertightness)
live in ``scripts/verify_cad.py`` section 1, which needs a real kernel.

Everything here guards a *silent* failure. A bandless wheel builds, screens, meshes and
solves whether or not any of these hold; what changes is only the number that comes out.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.cad.centreline import spoke_centreline, spoke_outline
from wheelopt.cad.constraints import PlatformLimits, Severity, check_design, is_feasible
from wheelopt.cad.materials import MaterialSpec
from wheelopt.cad.params import PARAM_BOUNDS, SpokeProfile, WheelParams
from wheelopt.fea.loadcase import CONTACT_ANGLE_DEG, phase_for_tip_contact
from wheelopt.fea.mesh import MeshSpec, classify_elements, classify_nodes

TPU95A = MaterialSpec(name="TPU_95A")
LIMITS = PlatformLimits()

BANDLESS = WheelParams(rim_thickness_mm=0.0, spoke_phase_deg=-90.0)
BANDED = WheelParams(rim_thickness_mm=3.0)


class TestTopologySwitch(unittest.TestCase):
    def test_has_shear_band_distinguishes_the_two(self):
        self.assertTrue(BANDED.has_shear_band)
        self.assertFalse(BANDLESS.has_shear_band)

    def test_rim_inner_radius_collapses_to_the_running_surface(self):
        self.assertEqual(BANDLESS.rim_inner_radius_mm, BANDLESS.outer_radius_mm)

    def test_only_exactly_zero_is_the_switch(self):
        """A sliver of a band is still a band, and is still unprintable."""
        lo, _ = PARAM_BOUNDS["rim_thickness_mm"]
        sliver = WheelParams(rim_thickness_mm=lo / 2.0)
        self.assertTrue(sliver.has_shear_band)
        names = {v.name for v in check_design(sliver, TPU95A, LIMITS)}
        self.assertIn("rim_min_wall", names)
        self.assertIn("bounds_rim_thickness_mm", names)


class TestScreening(unittest.TestCase):
    def test_bandless_is_feasible(self):
        v = check_design(BANDLESS, TPU95A, LIMITS)
        self.assertTrue(is_feasible(v), [str(x) for x in v])

    def test_bandless_warns_that_contact_is_discrete(self):
        names = {x.name for x in check_design(BANDLESS, TPU95A, LIMITS)}
        self.assertIn("no_shear_band", names)
        self.assertNotIn("no_shear_band", {x.name for x in check_design(BANDED, TPU95A, LIMITS)})

    def test_untreaded_bandless_is_not_rejected_for_cutting_through_a_band(self):
        """The old single comparison read ``0 >= 0`` and rejected a perfectly good wheel."""
        names = {x.name for x in check_design(BANDLESS, TPU95A, LIMITS)}
        self.assertNotIn("tread_depth", names)

    def test_tread_on_a_bandless_wheel_is_still_rejected(self):
        p = WheelParams(rim_thickness_mm=0.0, tread_depth_mm=1.0)
        bad = [x for x in check_design(p, TPU95A, LIMITS) if x.name == "tread_depth"]
        self.assertEqual(len(bad), 1)
        self.assertIs(bad[0].severity, Severity.INFEASIBLE)

    def test_zero_is_exempt_from_the_search_bounds_but_nothing_else_is(self):
        names = {x.name for x in check_design(BANDLESS, TPU95A, LIMITS)}
        self.assertNotIn("bounds_rim_thickness_mm", names)
        over = WheelParams(rim_thickness_mm=PARAM_BOUNDS["rim_thickness_mm"][1] + 1.0)
        self.assertIn(
            "bounds_rim_thickness_mm", {x.name for x in check_design(over, TPU95A, LIMITS)}
        )


class TestTipGeometry(unittest.TestCase):
    def test_tips_reach_the_running_surface(self):
        for profile in SpokeProfile:
            with self.subTest(profile=profile.value):
                p = WheelParams(rim_thickness_mm=0.0, spoke_profile=profile)
                c = spoke_centreline(p, 0, overlap_mm=1.5)
                self.assertAlmostEqual(
                    float(np.linalg.norm(c[-1])), p.outer_radius_mm, places=9,
                    msg="outer overlap applied with nothing to overlap into",
                )

    def test_no_outline_point_escapes_the_running_surface(self):
        """The regression this file exists for.

        Offsetting perpendicular to a centreline that is not radial at the tip pushes the
        outboard shoulder past ``outer_radius_mm`` — 389 um at the nominal design and
        3.3 mm at the corner of the design space. The FEA tread node set is
        ``|r - R| < 0.1 mm``, so the first material to touch the ground would not be in the
        contact set at all. The thicker spokes the 24.5 N platform needs made this an order
        of magnitude worse than when it was found.
        """
        worst = -np.inf
        for profile in SpokeProfile:
            for radius in (60.0, 85.0, 100.0):
                for curvature in np.linspace(-0.03, 0.03, 13):
                    for thickness in (1.2, 4.0, 8.0):
                        p = WheelParams(
                            outer_radius_mm=radius,
                            rim_thickness_mm=0.0,
                            hub_radius_mm=0.35 * radius,
                            spoke_thickness_mm=thickness,
                            spoke_curvature_1_per_mm=float(curvature),
                            spoke_profile=profile,
                        )
                        if abs(p.spoke_sagitta_mm) > 0.5 * p.spoke_span_mm:
                            continue
                        outline = spoke_outline(p, 0, overlap_mm=0.75 * thickness)
                        worst = max(
                            worst, float(np.max(np.linalg.norm(outline, axis=1))) - radius
                        )
        # 1 nm, to allow for the round-trip through the radial projection. The failure this
        # guards was five orders of magnitude larger.
        self.assertLessEqual(
            worst, 1e-6, f"tip escapes the running surface by {worst * 1e3:.1f} um"
        )

    def test_truncation_stays_within_the_chord_sagitta(self):
        """Clipping must trim the tip, not blunt it: the shortfall is (t/2)^2 / 2R."""
        p = WheelParams(rim_thickness_mm=0.0)
        outline = spoke_outline(p, 0, overlap_mm=0.75 * p.spoke_thickness_mm)
        reach = float(np.max(np.linalg.norm(outline, axis=1)))
        chord_sagitta = (0.5 * p.spoke_thickness_mm) ** 2 / (2.0 * p.outer_radius_mm)
        self.assertGreaterEqual(reach, p.outer_radius_mm - 2.0 * chord_sagitta)

    def test_banded_outlines_are_untouched_by_the_clip(self):
        outline = spoke_outline(BANDED, 0, overlap_mm=1.5)
        self.assertGreater(
            float(np.max(np.linalg.norm(outline, axis=1))),
            BANDED.rim_inner_radius_mm,
            "the attachment overlap into the shear band was clipped away",
        )


class TestPhase(unittest.TestCase):
    def test_phase_rotates_the_whole_pattern(self):
        rotated = WheelParams(spoke_phase_deg=37.0)
        base = spoke_centreline(WheelParams(), 0)
        turned = spoke_centreline(rotated, 0)
        angle = np.degrees(
            np.arctan2(turned[-1][1], turned[-1][0]) - np.arctan2(base[-1][1], base[-1][0])
        )
        self.assertAlmostEqual(angle, 37.0, places=6)

    def test_phase_changes_the_design_hash(self):
        self.assertNotEqual(
            WheelParams().design_hash(), WheelParams(spoke_phase_deg=-90.0).design_hash()
        )

    def test_tip_phase_aims_a_spoke_at_the_contact_point(self):
        for n in (6, 9, 16, 17):
            with self.subTest(n_spokes=n):
                p = WheelParams(
                    rim_thickness_mm=0.0, n_spokes=n,
                    spoke_phase_deg=phase_for_tip_contact(n),
                )
                angles = [
                    np.degrees(np.arctan2(*spoke_centreline(p, i)[-1][::-1]))
                    for i in range(n)
                ]
                self.assertTrue(
                    any(abs(a - CONTACT_ANGLE_DEG) < 1e-9 for a in angles),
                    f"no tip at the contact point; got {sorted(round(a, 2) for a in angles)}",
                )

    def test_gap_phase_aims_the_midpoint_between_two_tips(self):
        """The default phase of 0 gives this for a six-spoke wheel — by accident."""
        n = 6
        gap = phase_for_tip_contact(n, on_tip=False)
        self.assertAlmostEqual(gap - phase_for_tip_contact(n), 180.0 / n, places=9)
        self.assertAlmostEqual(WheelParams().spoke_phase_deg % (360.0 / n),
                               gap % (360.0 / n), places=9)


class TestMeshTagging(unittest.TestCase):
    """The tips must land in the sets that carry contact and stress output."""

    def setUp(self):
        # A ring of nodes at each interesting radius, in metres.
        self.p = WheelParams(rim_thickness_mm=0.0)
        # bore, inside the hub, the hub surface, mid-spoke, just inside the tread, the tread
        radii_mm = [4.0, 12.0, 22.0, 50.0, 84.5, 85.0]
        theta = np.linspace(0.0, 2.0 * np.pi, 12, endpoint=False)
        self.nodes = np.array(
            [[r * np.cos(t) * 1e-3, r * np.sin(t) * 1e-3, 0.0]
             for r in radii_mm for t in theta]
        )
        self.radius_of = np.repeat(radii_mm, len(theta))

    def _set_radii(self, sets, name):
        return sorted(set(np.round(self.radius_of[sets[name] - 1], 3)))

    def test_tip_nodes_are_tread_and_spoke_not_rim(self):
        sets = classify_nodes(self.nodes, self.p, MeshSpec())
        self.assertEqual(self._set_radii(sets, "tread"), [85.0])
        self.assertEqual(len(sets["rim"]), 0)
        self.assertIn(85.0, self._set_radii(sets, "spokes"))

    def test_banded_wheel_still_separates_rim_from_spokes(self):
        sets = classify_nodes(self.nodes, BANDED, MeshSpec())
        self.assertGreater(len(sets["rim"]), 0)
        self.assertNotIn(85.0, self._set_radii(sets, "spokes"))

    def test_tip_elements_stay_in_the_stress_output_set(self):
        """They are the peak-stress location on a bandless wheel, so dropping them would
        report a peak stress from somewhere else entirely."""
        elements = np.array([[i + 1] * 4 for i in range(len(self.nodes))], dtype=np.int64)
        sets = classify_elements(self.nodes, elements, self.p)
        radii = sorted(set(np.round(self.radius_of[sets["spokes"] - 1], 3)))
        self.assertIn(85.0, radii)
        self.assertEqual(len(sets["rim"]), 0)


if __name__ == "__main__":
    unittest.main()
