"""The cross-section: pure-geometry checks, plus meshing when gmsh is available.

The section exists to be a cheap stand-in for the 3-D solid, so the test that matters most
is that it *is* the same geometry: :class:`TestAgreesWithTheSolid` compares the meshed area
against the analytic one and against the extruded volume. Two paths that share no code below
`cad.centreline` agreeing is the only reason to trust either.

CalculiX does solve this tier. An earlier session reported otherwise; that was a false
negative in the test harness, not a solver limitation — see docs/experiments/log.md,
2026-08-08.
"""

from __future__ import annotations

import unittest
from dataclasses import replace

import numpy as np

from wheelopt.cad.materials import TPU95A
from wheelopt.cad.params import WheelParams
from wheelopt.fea.loadcase import LoadCase, MeshSpec, SolverSpec, phase_for_tip_contact
from wheelopt.fea.mesh import MeshFailure
from wheelopt.fea.section2d import (
    _n_connected_components,
    _orient_ccw,
    _tri_areas,
    _tri_signed_areas,
    mesh_claw_sector,
    mesh_section,
    section_area_mm2,
    section_polygons,
)

try:  # gmsh is an optional extra; the pure checks below must run without it
    import gmsh  # noqa: F401

    HAVE_GMSH = True
except ImportError:  # pragma: no cover - environment dependent
    HAVE_GMSH = False

TINY = dict(
    outer_radius_mm=60.0, width_mm=30.0, n_spokes=6,
    spoke_thickness_mm=5.0, hub_radius_mm=20.0,
)
BANDED = WheelParams(**TINY, rim_thickness_mm=3.0)
BANDLESS = WheelParams(
    **TINY, rim_thickness_mm=0.0, spoke_phase_deg=phase_for_tip_contact(6)
)
SPEC = MeshSpec(dimension=2, size_spoke_m=0.0025, size_rim_m=0.003, size_hub_m=0.002)


class TestSectionPolygons(unittest.TestCase):
    def test_one_polygon_per_spoke(self):
        self.assertEqual(len(section_polygons(BANDED)), BANDED.n_spokes)

    def test_polygons_are_closed_loops_without_a_repeated_point(self):
        for poly in section_polygons(BANDED):
            self.assertEqual(poly.shape[1], 2)
            self.assertFalse(np.allclose(poly[0], poly[-1]))

    def test_bandless_tips_stay_inside_the_running_surface(self):
        # The 3-D generator clips spoke outlines to outer_radius_mm; the section is built
        # from the same call, so it inherits that. If it ever stopped inheriting it, the
        # section would be larger than the wheel it claims to represent.
        for poly in section_polygons(BANDLESS):
            r = np.hypot(poly[:, 0], poly[:, 1])
            self.assertLessEqual(r.max(), BANDLESS.outer_radius_mm + 1e-6)

    def test_analytic_area_is_an_upper_bound_not_an_identity(self):
        # The spokes overlap the hub and band, and the analytic sum double-counts that.
        # Stated as a test so nobody later "fixes" the mesh to match it exactly.
        analytic = section_area_mm2(BANDED)
        parts = (
            np.pi * (BANDED.hub_radius_mm**2 - BANDED.hub_bore_radius_mm**2)
            + np.pi * (BANDED.outer_radius_mm**2 - BANDED.rim_inner_radius_mm**2)
        )
        self.assertGreater(analytic, parts)


class TestTriangleOrientation(unittest.TestCase):
    """The guard that stops CalculiX dying at t=0 with a nonpositive Jacobian."""

    CCW = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                    [0.5, 0.0, 0.0], [0.5, 0.5, 0.0], [0.0, 0.5, 0.0]])

    def tri(self, clockwise: bool) -> tuple[np.ndarray, np.ndarray]:
        nodes = self.CCW.copy()
        conn = np.array([[1, 2, 3, 4, 5, 6]], dtype=np.int64)
        if clockwise:
            conn = conn[:, [1, 0, 2, 3, 5, 4]]
        return nodes, conn

    def test_signed_area_detects_winding(self):
        self.assertGreater(_tri_signed_areas(*self.tri(False))[0], 0)
        self.assertLess(_tri_signed_areas(*self.tri(True))[0], 0)

    def test_unsigned_area_cannot(self):
        # Why the original check missed it: abs() reports both windings as healthy.
        self.assertAlmostEqual(
            _tri_areas(*self.tri(False))[0], _tri_areas(*self.tri(True))[0]
        )

    def test_rewinding_fixes_a_clockwise_triangle(self):
        nodes, conn = self.tri(True)
        fixed = _orient_ccw(nodes, conn)
        self.assertGreater(_tri_signed_areas(nodes, fixed)[0], 0)

    def test_rewinding_preserves_the_element(self):
        # The reversal must keep each mid-side node on its own edge, or the element is
        # quietly deformed rather than reoriented.
        nodes, conn = self.tri(True)
        fixed = _orient_ccw(nodes, conn)[0]
        for corner_a, corner_b, mid in ((0, 1, 3), (1, 2, 4), (2, 0, 5)):
            expected = 0.5 * (nodes[fixed[corner_a] - 1] + nodes[fixed[corner_b] - 1])
            np.testing.assert_allclose(nodes[fixed[mid] - 1], expected, atol=1e-12)

    def test_a_ccw_triangle_is_left_alone(self):
        nodes, conn = self.tri(False)
        np.testing.assert_array_equal(_orient_ccw(nodes, conn), conn)


class TestConnectivity(unittest.TestCase):
    """`fuse` returned the section as 8 separate faces without complaining; this is how
    that gets caught."""

    def test_one_body(self):
        conn = np.array([[1, 2, 3, 4, 5, 6], [2, 7, 3, 8, 9, 5]], dtype=np.int64)
        self.assertEqual(_n_connected_components(conn, 9), 1)

    def test_two_bodies(self):
        conn = np.array([[1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12]], dtype=np.int64)
        self.assertEqual(_n_connected_components(conn, 12), 2)

    def test_touching_at_a_single_node_still_counts_as_joined(self):
        # Honest limitation: two triangles meeting at one node are one component here, and
        # mechanically they are a hinge. The check exists to catch parts that are not
        # connected at all, which is the failure `fuse` produced.
        conn = np.array([[1, 2, 3, 4, 5, 6], [3, 7, 8, 9, 10, 11]], dtype=np.int64)
        self.assertEqual(_n_connected_components(conn, 11), 1)


@unittest.skipUnless(HAVE_GMSH, "gmsh is not installed")
class TestAgreesWithTheSolid(unittest.TestCase):
    """The section must be the same geometry the 3-D tier meshes."""

    def mesh(self, params):
        from wheelopt.fea.section2d import mesh_section

        return mesh_section(params, SPEC)

    def test_meshes_as_one_connected_body(self):
        for name, params in (("banded", BANDED), ("bandless", BANDLESS)):
            with self.subTest(name):
                m = self.mesh(params)
                self.assertEqual(_n_connected_components(m.elements, m.n_nodes), 1)

    def test_every_triangle_is_counter_clockwise(self):
        for name, params in (("banded", BANDED), ("bandless", BANDLESS)):
            with self.subTest(name):
                m = self.mesh(params)
                self.assertTrue(bool(np.all(_tri_signed_areas(m.nodes_m, m.elements) > 0)))

    def test_area_is_close_to_but_below_the_analytic_bound(self):
        for name, params in (("banded", BANDED), ("bandless", BANDLESS)):
            with self.subTest(name):
                m = self.mesh(params)
                meshed = float(_tri_areas(m.nodes_m, m.elements).sum()) * 1e6
                analytic = section_area_mm2(params)
                self.assertLess(meshed, analytic)
                self.assertGreater(meshed, 0.9 * analytic)

    def test_nodes_are_exactly_planar(self):
        # A stray 1e-19 in z is a real out-of-plane coordinate to CalculiX's expansion.
        m = self.mesh(BANDED)
        self.assertTrue(bool(np.all(m.nodes_m[:, 2] == 0.0)))

    def test_the_sets_that_define_the_problem_are_populated(self):
        m = self.mesh(BANDED)
        for name in ("bore", "tread", "hub", "spokes"):
            with self.subTest(name):
                self.assertGreater(len(m.node_sets[name]), 0)

    def test_bandless_contact_is_discrete(self):
        # Six tips instead of a cylinder: far fewer tread nodes than the banded wheel, and
        # that difference is the whole point of the topology.
        self.assertLess(
            len(self.mesh(BANDLESS).node_sets["tread"]),
            0.5 * len(self.mesh(BANDED).node_sets["tread"]),
        )

    def test_the_3d_mesher_refuses_a_2d_spec(self):
        from wheelopt.fea.mesh import MeshFailure, mesh_step

        with self.assertRaises(MeshFailure):
            mesh_step("unused.step", BANDED, SPEC)

    def test_the_2d_mesher_refuses_a_3d_spec(self):
        from wheelopt.fea.mesh import MeshFailure
        from wheelopt.fea.section2d import mesh_section

        with self.assertRaises(MeshFailure):
            mesh_section(BANDED, MeshSpec())


@unittest.skipUnless(HAVE_GMSH, "gmsh is not installed")
class TestPlaneStrainDeck(unittest.TestCase):
    """The deck a plane-strain mesh produces. CalculiX solves it; see the log for the
    false negative that briefly said otherwise."""

    def deck(self):
        from wheelopt.fea.deck import build_deck
        from wheelopt.fea.hyperelastic import for_material
        from wheelopt.fea.indenter import build_indenter
        from wheelopt.fea.section2d import mesh_section

        mesh = mesh_section(BANDED, SPEC)
        case = LoadCase()
        indenter = build_indenter(
            case.kind, case.indenter,
            BANDED.outer_radius_mm * 1e-3, BANDED.width_mm * 1e-3, dimension=2,
        )
        return build_deck(
            mesh, indenter, BANDED, TPU95A,
            for_material(TPU95A, BANDED.spoke_thickness_mm), case, SolverSpec(),
        ).text

    def test_uses_plane_strain_element_types(self):
        text = self.deck()
        self.assertIn("*ELEMENT, TYPE=CPE6, ELSET=EWHEEL", text)
        self.assertIn("*ELEMENT, TYPE=CPE4, ELSET=EINDENT", text)

    def test_sections_carry_the_wheel_width_as_thickness(self):
        # Omitted, CalculiX defaults the thickness to 1 m and every force is ~22x too
        # large on a 45 mm wheel — plausible, and wrong.
        text = self.deck().splitlines()
        for i, ln in enumerate(text):
            if ln.startswith("*SOLID SECTION"):
                with self.subTest(ln):
                    self.assertAlmostEqual(float(text[i + 1]), BANDED.width_mm * 1e-3)

    def test_the_bore_is_restrained_in_plane_only(self):
        # Plane-strain nodes have no third DOF of their own; asking for one is rejected.
        self.assertIn(" NBORE, 1, 2, 0.0", self.deck())

    def test_the_master_surface_is_the_wheel_facing_edge(self):
        # S3 after the orientation flip, not the hexahedron's S3-by-coincidence. A wrong
        # face declares the indenter's back edge as the master and nothing ever touches.
        from wheelopt.fea.indenter import build_indenter

        ind = build_indenter(
            LoadCase().kind, LoadCase().indenter, 0.060, 0.030, dimension=2
        )
        edge = {1: (0, 1), 3: (2, 3)}[ind.contact_face]
        ys = ind.nodes_m[ind.elements[0] - 1][:, 1]
        other = [i for i in range(4) if i not in edge]
        self.assertGreater(ys[list(edge)].min(), ys[other].max())


class TestClawSector(unittest.TestCase):
    """One claw and its hub, instead of the whole wheel — see `ClawSector`."""

    SPEC = MeshSpec(dimension=2, size_spoke_m=0.0025, size_rim_m=0.003, size_hub_m=0.0045)
    CLAW = WheelParams(outer_radius_mm=85.0, width_mm=45.0, n_spokes=12,
                       spoke_thickness_mm=7.0, rim_thickness_mm=0.0, hub_radius_mm=22.0,
                       claw_taper_ratio=0.5, spoke_phase_deg=-90.0)

    def test_refuses_a_banded_design(self):
        """One claw of a banded wheel is not an independent structure, so measuring it as
        one would report a stiffness the wheel does not have."""
        banded = replace(self.CLAW, rim_thickness_mm=3.0)
        with self.assertRaises(MeshFailure):
            mesh_claw_sector(banded, self.SPEC)

    @unittest.skipUnless(HAVE_GMSH, "gmsh is not installed")
    def test_refuses_a_wedge_that_pins_rather_than_clamps(self):
        """A 30 deg wedge on a 4 mm bore keeps ~2 mm of arc and two nodes. That solves, and
        describes a claw pivoting on two pins rather than clamped to a shaft."""
        with self.assertRaises(MeshFailure) as caught:
            mesh_claw_sector(self.CLAW, self.SPEC, hub_span_deg=30.0)
        self.assertIn("bore", str(caught.exception))

    @unittest.skipUnless(HAVE_GMSH, "gmsh is not installed")
    def test_is_far_smaller_than_the_whole_wheel_and_keeps_one_tip(self):
        whole = mesh_section(self.CLAW, self.SPEC)
        sector = mesh_claw_sector(self.CLAW, self.SPEC)
        self.assertLess(sector.stats.n_elements, 0.35 * whole.stats.n_elements)
        # The full bore is kept, so the shaft constraint is exactly the wheel's.
        self.assertEqual(len(sector.node_sets["bore"]), len(whole.node_sets["bore"]))
        # One tip on the running surface instead of twelve.
        self.assertGreater(len(whole.node_sets["tread"]), 3 * len(sector.node_sets["tread"]))
        self.assertGreater(len(sector.node_sets["tread"]), 0)

    @unittest.skipUnless(HAVE_GMSH, "gmsh is not installed")
    def test_the_kept_claw_is_the_one_the_indenter_aims_at(self):
        """`spoke_phase_deg` positions spoke 0, `phase_for_tip_contact` aims the indenter at
        the same angle, and the sector keeps index 0. If those three ever disagree, the mesh
        has a claw and the load case presses somewhere else — and it would still solve."""
        params = replace(self.CLAW, spoke_phase_deg=phase_for_tip_contact(12))
        sector = mesh_claw_sector(params, self.SPEC)
        tread = sector.nodes_m[np.asarray(sector.node_sets["tread"]) - 1]
        # The contact point is at -90 deg, straight down: y negative, x near zero.
        self.assertTrue(bool(np.all(tread[:, 1] < 0.0)))
        self.assertLess(float(np.abs(tread[:, 0]).max()), 0.01)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
