"""Rigid contact bodies: element validity, placement and face numbering.

A negative-Jacobian hexahedron is rejected by CalculiX outright, and a contact face
declared on the wrong local face number produces a model where the wheel never touches
anything — which converges instantly to zero force and looks like an infinitely soft wheel.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.fea.indenter import CONTACT_FACE, _hex_volume, build_indenter
from wheelopt.fea.loadcase import IndenterSpec, LoadCaseKind

RADIUS = 0.070
WIDTH = 0.040


#: Only the contact cases have an indenter at all. `TIP_RADIAL` / `TIP_TANGENTIAL` prescribe
#: the tread directly and `build_indenter` rightly refuses them, so sweeping every member of
#: the enum here would be testing that a function fails.
CONTACT_KINDS = tuple(k for k in LoadCaseKind if k.needs_indenter)


def build(kind: LoadCaseKind, **spec_kwargs):
    return build_indenter(kind, IndenterSpec(**spec_kwargs), RADIUS, WIDTH)


class TestElementValidity(unittest.TestCase):
    def test_all_elements_have_positive_volume(self):
        for kind in CONTACT_KINDS:
            for fillet in (0.0, 0.0005, 0.001, 0.003):
                with self.subTest(kind=kind.value, fillet=fillet):
                    m = build(kind, edge_fillet_m=fillet)
                    vols = np.array([_hex_volume(m.nodes_m[h - 1]) for h in m.elements])
                    self.assertTrue(
                        bool((vols > 0).all()),
                        f"{int((vols <= 0).sum())} inverted elements",
                    )

    def test_thick_block_around_a_tight_fillet_does_not_invert(self):
        """The offset must be capped by local curvature or the corner folds through itself."""
        m = build(LoadCaseKind.RADIAL_STEP_EDGE, thickness_m=0.020, edge_fillet_m=0.0005)
        vols = np.array([_hex_volume(m.nodes_m[h - 1]) for h in m.elements])
        self.assertTrue(bool((vols > 0).all()))

    def test_connectivity_is_within_range(self):
        for kind in CONTACT_KINDS:
            with self.subTest(kind=kind.value):
                m = build(kind)
                self.assertEqual(m.elements.shape[1], 8)
                self.assertGreaterEqual(int(m.elements.min()), 1)
                self.assertLessEqual(int(m.elements.max()), m.n_nodes)

    def test_no_element_reuses_a_node(self):
        m = build(LoadCaseKind.RADIAL_FLAT)
        for h in m.elements[:50]:
            self.assertEqual(len(set(h.tolist())), 8)


class TestPlacement(unittest.TestCase):
    def test_sits_below_the_wheel_with_a_gap(self):
        for kind in CONTACT_KINDS:
            with self.subTest(kind=kind.value):
                m = build(kind)
                top = float(m.nodes_m[:, 1].max())
                self.assertLess(top, -RADIUS, "indenter must start clear of the tread")
                self.assertAlmostEqual(top, -RADIUS - 1e-4, places=9)

    def test_overhangs_the_wheel_width(self):
        """The patch must never run off the master surface."""
        for kind in CONTACT_KINDS:
            with self.subTest(kind=kind.value):
                m = build(kind)
                self.assertGreater(float(m.nodes_m[:, 2].max()), WIDTH / 2)
                self.assertLess(float(m.nodes_m[:, 2].min()), -WIDTH / 2)

    def test_flat_plate_spans_both_sides_of_the_contact_point(self):
        m = build(LoadCaseKind.RADIAL_FLAT)
        self.assertLess(float(m.nodes_m[:, 0].min()), 0.0)
        self.assertGreater(float(m.nodes_m[:, 0].max()), 0.0)

    def test_step_edge_material_lies_on_one_side_only(self):
        """A step has a cliff: nothing to the +x side of the corner."""
        m = build(LoadCaseKind.RADIAL_STEP_EDGE)
        self.assertLessEqual(float(m.nodes_m[:, 0].max()), 1e-12)

    def test_step_edge_drops_by_the_step_height(self):
        m = build(LoadCaseKind.RADIAL_STEP_EDGE, step_height_m=0.05)
        span = float(m.nodes_m[:, 1].max() - m.nodes_m[:, 1].min())
        self.assertGreater(span, 0.045)

    def test_reference_point_is_on_the_wheel_centreline(self):
        for kind in CONTACT_KINDS:
            with self.subTest(kind=kind.value):
                m = build(kind)
                self.assertAlmostEqual(float(m.ref_point_m[2]), 0.0)


class TestFillet(unittest.TestCase):
    def test_fillet_rounds_the_corner(self):
        sharp = build(LoadCaseKind.RADIAL_STEP_EDGE, edge_fillet_m=0.0)
        round_ = build(LoadCaseKind.RADIAL_STEP_EDGE, edge_fillet_m=0.002)
        self.assertGreater(round_.n_nodes, sharp.n_nodes)

    def test_fillet_segments_control_resolution(self):
        few = build(LoadCaseKind.RADIAL_STEP_EDGE, fillet_segments=3)
        many = build(LoadCaseKind.RADIAL_STEP_EDGE, fillet_segments=12)
        self.assertGreater(many.n_nodes, few.n_nodes)

    def test_profile_has_no_duplicate_points(self):
        m = build(LoadCaseKind.RADIAL_STEP_EDGE, edge_fillet_m=0.001)
        slice_ = m.nodes_m[np.isclose(m.nodes_m[:, 2], m.nodes_m[0, 2])]
        unique = np.unique(slice_.round(12), axis=0)
        self.assertEqual(len(unique), len(slice_))


class TestSpecValidation(unittest.TestCase):
    def test_rejects_non_positive_extents(self):
        for kwargs in ({"half_length_m": 0.0}, {"half_width_m": -1.0},
                       {"thickness_m": 0.0}):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                IndenterSpec(**kwargs)

    def test_rejects_negative_fillet(self):
        with self.assertRaises(ValueError):
            IndenterSpec(edge_fillet_m=-0.001)

    def test_rejects_zero_fillet_segments(self):
        with self.assertRaises(ValueError):
            IndenterSpec(fillet_segments=0)


class TestContactFace(unittest.TestCase):
    def test_contact_face_is_s3(self):
        self.assertEqual(CONTACT_FACE, 3)

    def test_contact_face_nodes_lie_on_the_profile(self):
        """C3D8 face S3 is nodes 1-5-6-2, which must be the wheel-facing surface.

        Checked geometrically: those four nodes must be no deeper than the rest of the
        element, i.e. they are the outermost face toward the wheel.
        """
        for kind in CONTACT_KINDS:
            with self.subTest(kind=kind.value):
                m = build(kind)
                for h in m.elements[:40]:
                    coords = m.nodes_m[h - 1]
                    face = coords[[0, 4, 5, 1]]
                    self.assertGreaterEqual(
                        float(face[:, 1].mean()) + 1e-12, float(coords[:, 1].mean())
                    )

    def test_every_element_is_offered_to_the_contact_search(self):
        m = build(LoadCaseKind.RADIAL_FLAT)
        self.assertEqual(len(m.contact_elements), m.n_elements)


if __name__ == "__main__":
    unittest.main()
