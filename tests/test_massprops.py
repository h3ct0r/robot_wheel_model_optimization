"""Validate mass properties against solids with known closed-form inertia.

This is the load-bearing test in the CAD stage. If inertia is wrong, every dynamic result
downstream is wrong in a way that no simulation check will catch — see invariant 2.
"""

from __future__ import annotations

import math
import unittest

import numpy as np

from wheelopt.cad.massprops import check_against_brep_volume, mass_properties


def box_mesh(a: float, b: float, c: float) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned box centred at the origin, outward-facing triangles."""
    x, y, z = a / 2, b / 2, c / 2
    v = np.array(
        [
            [-x, -y, -z], [+x, -y, -z], [+x, +y, -z], [-x, +y, -z],
            [-x, -y, +z], [+x, -y, +z], [+x, +y, +z], [-x, +y, +z],
        ],
        dtype=np.float64,
    )
    f = np.array(
        [
            [0, 2, 1], [0, 3, 2],  # -z
            [4, 5, 6], [4, 6, 7],  # +z
            [0, 1, 5], [0, 5, 4],  # -y
            [2, 3, 7], [2, 7, 6],  # +y
            [1, 2, 6], [1, 6, 5],  # +x
            [0, 4, 7], [0, 7, 3],  # -x
        ],
        dtype=np.int64,
    )
    return v, f


def cylinder_mesh(radius: float, height: float, segments: int = 720):
    """Solid cylinder about the z axis, centred at the origin."""
    ang = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    bottom = np.hstack([ring, np.full((segments, 1), -height / 2)])
    top = np.hstack([ring, np.full((segments, 1), +height / 2)])
    centre_bot = np.array([[0.0, 0.0, -height / 2]])
    centre_top = np.array([[0.0, 0.0, +height / 2]])
    v = np.vstack([bottom, top, centre_bot, centre_top])

    ib, it = segments * 2, segments * 2 + 1
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        faces.append([ib, j, i])                          # bottom cap, outward = -z
        faces.append([it, segments + i, segments + j])    # top cap, outward = +z
        faces.append([i, j, segments + j])                # side
        faces.append([i, segments + j, segments + i])
    return v, np.array(faces, dtype=np.int64)


def annulus_mesh(r_outer: float, r_inner: float, height: float, segments: int = 720):
    """Hollow cylinder — the shear-band analogue. Inner surface wound inward."""
    ang = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    cos, sin = np.cos(ang), np.sin(ang)

    def ring(r, z):
        return np.stack([r * cos, r * sin, np.full(segments, z)], axis=1)

    zb, zt = -height / 2, +height / 2
    ob, ot = ring(r_outer, zb), ring(r_outer, zt)
    ib, it = ring(r_inner, zb), ring(r_inner, zt)
    v = np.vstack([ob, ot, ib, it])
    n = segments
    OB, OT, IB, IT = 0, n, 2 * n, 3 * n

    faces = []
    for i in range(n):
        j = (i + 1) % n
        # outer wall (normals point outward, +r)
        faces.append([OB + i, OB + j, OT + j])
        faces.append([OB + i, OT + j, OT + i])
        # inner wall (normals point inward, -r)
        faces.append([IB + i, IT + j, IB + j])
        faces.append([IB + i, IT + i, IT + j])
        # bottom annular cap (normal -z)
        faces.append([OB + i, IB + j, OB + j])
        faces.append([OB + i, IB + i, IB + j])
        # top annular cap (normal +z)
        faces.append([OT + i, OT + j, IT + j])
        faces.append([OT + i, IT + j, IT + i])
    return v, np.array(faces, dtype=np.int64)


class TestBox(unittest.TestCase):
    """A box is exactly representable, so agreement must be to machine precision."""

    def setUp(self):
        self.a, self.b, self.c = 0.030, 0.050, 0.070  # metres
        self.rho = 1210.0
        self.v, self.f = box_mesh(self.a, self.b, self.c)
        self.mp = mass_properties(self.v, self.f, self.rho)

    def test_volume_exact(self):
        self.assertAlmostEqual(self.mp.volume_m3, self.a * self.b * self.c, places=15)

    def test_mass_exact(self):
        self.assertAlmostEqual(self.mp.mass_kg, self.rho * self.a * self.b * self.c, places=12)

    def test_com_at_origin(self):
        np.testing.assert_allclose(self.mp.com_m, np.zeros(3), atol=1e-15)

    def test_inertia_exact(self):
        m = self.mp.mass_kg
        expected = np.diag(
            [
                m * (self.b**2 + self.c**2) / 12.0,
                m * (self.a**2 + self.c**2) / 12.0,
                m * (self.a**2 + self.b**2) / 12.0,
            ]
        )
        np.testing.assert_allclose(self.mp.inertia_kg_m2, expected, rtol=1e-12, atol=1e-18)

    def test_products_of_inertia_vanish(self):
        off = self.mp.inertia_kg_m2 - np.diag(np.diag(self.mp.inertia_kg_m2))
        np.testing.assert_allclose(off, np.zeros((3, 3)), atol=1e-18)

    def test_offset_box_recovers_com_and_same_inertia(self):
        """Translating the mesh must move the CoM and leave inertia about it unchanged.

        The parallel-axis shift subtracts large nearly-equal quantities, so the recovered
        off-diagonals are cancellation noise rather than exact zeros. The meaningful
        assertion is that they are negligible *relative to the tensor's own scale*, not
        that they are zero in absolute terms.
        """
        shift = np.array([0.1, -0.2, 0.35])
        mp = mass_properties(self.v + shift, self.f, self.rho)
        np.testing.assert_allclose(mp.com_m, shift, atol=1e-14)

        scale = np.max(np.abs(np.diag(self.mp.inertia_kg_m2)))
        np.testing.assert_allclose(
            mp.inertia_kg_m2, self.mp.inertia_kg_m2, rtol=1e-9, atol=1e-9 * scale
        )
        # And the noise really is noise: 10+ orders of magnitude below the diagonal.
        off = mp.inertia_kg_m2 - np.diag(np.diag(mp.inertia_kg_m2))
        self.assertLess(np.max(np.abs(off)) / scale, 1e-10)

    def test_inverted_winding_gives_same_result(self):
        flipped = self.f[:, ::-1].copy()
        mp = mass_properties(self.v, flipped, self.rho)
        self.assertAlmostEqual(mp.volume_m3, self.mp.volume_m3, places=15)
        np.testing.assert_allclose(mp.inertia_kg_m2, self.mp.inertia_kg_m2, rtol=1e-12)


class TestCylinder(unittest.TestCase):
    """Curved surface: agreement is limited by tessellation, so tolerances are relative."""

    def setUp(self):
        self.r, self.h = 0.070, 0.040
        self.rho = 1210.0
        self.v, self.f = cylinder_mesh(self.r, self.h, segments=2000)
        self.mp = mass_properties(self.v, self.f, self.rho)

    def test_volume(self):
        expected = math.pi * self.r**2 * self.h
        self.assertAlmostEqual(self.mp.volume_m3 / expected, 1.0, places=5)

    def test_polar_moment(self):
        """Izz = m r^2 / 2 — the moment that dominates wheel spin-up torque."""
        m = self.mp.mass_kg
        expected = m * self.r**2 / 2.0
        self.assertAlmostEqual(self.mp.inertia_kg_m2[2, 2] / expected, 1.0, places=4)

    def test_transverse_moments(self):
        m = self.mp.mass_kg
        expected = m * (3 * self.r**2 + self.h**2) / 12.0
        for axis in (0, 1):
            self.assertAlmostEqual(self.mp.inertia_kg_m2[axis, axis] / expected, 1.0, places=4)

    def test_tessellation_under_reports_volume(self):
        """An inscribed polygon is always smaller than the circle it approximates."""
        exact = math.pi * self.r**2 * self.h
        self.assertLess(self.mp.volume_m3, exact)


class TestAnnulus(unittest.TestCase):
    """Hollow cylinder — checks that internal voids are handled by winding alone."""

    def setUp(self):
        self.ro, self.ri, self.h = 0.070, 0.067, 0.040
        self.rho = 1210.0
        v, f = annulus_mesh(self.ro, self.ri, self.h, segments=2000)
        self.mp = mass_properties(v, f, self.rho)

    def test_volume(self):
        expected = math.pi * (self.ro**2 - self.ri**2) * self.h
        self.assertAlmostEqual(self.mp.volume_m3 / expected, 1.0, places=4)

    def test_polar_moment(self):
        m = self.mp.mass_kg
        expected = m * (self.ro**2 + self.ri**2) / 2.0
        self.assertAlmostEqual(self.mp.inertia_kg_m2[2, 2] / expected, 1.0, places=4)

    def test_thin_rim_has_higher_specific_inertia_than_solid(self):
        """Sanity: mass concentrated at the rim spins up harder per unit mass."""
        solid_v, solid_f = cylinder_mesh(self.ro, self.h, segments=2000)
        solid = mass_properties(solid_v, solid_f, self.rho)
        self.assertGreater(
            self.mp.inertia_kg_m2[2, 2] / self.mp.mass_kg,
            solid.inertia_kg_m2[2, 2] / solid.mass_kg,
        )


class TestScalingLaws(unittest.TestCase):
    def test_inertia_scales_with_radius_squared(self):
        """Doubling radius at fixed density must raise Izz by 2^4 = 16 (m x r^2)."""
        rho = 1210.0
        small = mass_properties(*cylinder_mesh(0.05, 0.04, 512), rho)
        large = mass_properties(*cylinder_mesh(0.10, 0.04, 512), rho)
        ratio = large.inertia_kg_m2[2, 2] / small.inertia_kg_m2[2, 2]
        self.assertAlmostEqual(ratio, 16.0, places=3)

    def test_mass_is_linear_in_density(self):
        v, f = cylinder_mesh(0.07, 0.04, 256)
        a = mass_properties(v, f, 1000.0)
        b = mass_properties(v, f, 2000.0)
        self.assertAlmostEqual(b.mass_kg / a.mass_kg, 2.0, places=12)
        self.assertAlmostEqual(a.volume_m3, b.volume_m3, places=15)


class TestFailureModes(unittest.TestCase):
    def test_empty_mesh_raises(self):
        with self.assertRaises(ValueError):
            mass_properties(np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64), 1000.0)

    def test_degenerate_flat_mesh_raises(self):
        """A zero-volume mesh is a pipeline bug, not an infeasible design — it must raise."""
        v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float64)
        f = np.array([[0, 1, 2], [0, 2, 1]], dtype=np.int64)
        with self.assertRaises(ValueError):
            mass_properties(v, f, 1000.0)

    def test_bad_shapes_raise(self):
        with self.assertRaises(ValueError):
            mass_properties(np.zeros((5, 2)), np.zeros((1, 3), dtype=np.int64), 1000.0)


class TestBrepCrossCheck(unittest.TestCase):
    def test_accepts_small_under_report(self):
        ok, rel = check_against_brep_volume(0.995, 1.0, tolerance=0.01)
        self.assertTrue(ok)
        self.assertLess(rel, 0.0)

    def test_rejects_coarse_tessellation(self):
        ok, rel = check_against_brep_volume(0.90, 1.0, tolerance=0.01)
        self.assertFalse(ok)
        self.assertAlmostEqual(rel, -0.10, places=12)

    def test_rejects_nonpositive_brep_volume(self):
        ok, _ = check_against_brep_volume(1.0, 0.0)
        self.assertFalse(ok)

    def test_real_cylinder_passes_at_reasonable_tessellation(self):
        r, h = 0.07, 0.04
        mp = mass_properties(*cylinder_mesh(r, h, segments=512), 1210.0)
        ok, rel = check_against_brep_volume(mp.volume_m3, math.pi * r**2 * h, tolerance=0.01)
        self.assertTrue(ok, f"relative error {rel:.4%} exceeded 1%")


if __name__ == "__main__":
    unittest.main()
