"""Mesh utilities in the export layer: vertex welding and the manifold check.

These are pure numpy and run without OCCT. They matter because a BREP tessellator emits
each face independently, so the raw mesh has no shared edges at all — the welding step is
what turns a geometrically closed surface into a topologically manifold one, and a
non-manifold collision mesh produces plausible-looking wrong contact rather than an error.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.cad.export import is_watertight, weld_vertices

from .test_massprops import box_mesh


def explode(vertices: np.ndarray, faces: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Give every triangle its own copy of its vertices.

    This is what OCCT's per-face tessellation does at face boundaries, taken to the limit:
    no two triangles share an index, so no edge is shared.
    """
    v = vertices[faces.reshape(-1)]
    f = np.arange(len(v), dtype=np.int64).reshape(-1, 3)
    return v, f


class TestWeldVertices(unittest.TestCase):
    def test_exploded_box_welds_back_to_eight_vertices(self):
        v, f = box_mesh(2.0, 3.0, 4.0)
        ev, ef = explode(v, f)
        self.assertEqual(len(ev), 36)

        wv, wf = weld_vertices(ev, ef)
        self.assertEqual(len(wv), 8)
        self.assertEqual(wf.shape, f.shape)

    def test_welding_makes_an_exploded_mesh_watertight(self):
        v, f = box_mesh(1.0, 1.0, 1.0)
        ev, ef = explode(v, f)
        self.assertFalse(is_watertight(ef)[0])

        _, wf = weld_vertices(ev, ef)
        self.assertTrue(is_watertight(wf)[0])

    def test_welding_preserves_geometry(self):
        """Welding may reindex, but it must never move a point."""
        v, f = box_mesh(2.0, 3.0, 4.0)
        ev, ef = explode(v, f)
        wv, wf = weld_vertices(ev, ef)

        before = np.sort(ev[ef.reshape(-1)].round(12), axis=0)
        after = np.sort(wv[wf.reshape(-1)].round(12), axis=0)
        np.testing.assert_allclose(before, after, atol=0.0)

    def test_welding_preserves_winding(self):
        """Face orientation must survive reindexing, or volume flips sign."""
        v, f = box_mesh(2.0, 2.0, 2.0)
        ev, ef = explode(v, f)
        wv, wf = weld_vertices(ev, ef)

        def signed_volume(vv, ff):
            a, b, c = vv[ff[:, 0]], vv[ff[:, 1]], vv[ff[:, 2]]
            return float(np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) / 6.0)

        self.assertAlmostEqual(signed_volume(wv, wf), signed_volume(v, f), places=12)

    def test_points_further_apart_than_the_tolerance_are_not_merged(self):
        v = np.array([[0.0, 0.0, 0.0], [1e-6, 0.0, 0.0]], dtype=np.float64)
        f = np.array([[0, 1, 0]], dtype=np.int64)
        wv, _ = weld_vertices(v, f, tolerance_m=1e-9)
        self.assertEqual(len(wv), 2)

    def test_points_within_the_tolerance_are_merged(self):
        v = np.array([[0.0, 0.0, 0.0], [1e-12, 0.0, 0.0]], dtype=np.float64)
        f = np.array([[0, 1, 0]], dtype=np.int64)
        wv, _ = weld_vertices(v, f, tolerance_m=1e-9)
        self.assertEqual(len(wv), 1)

    def test_already_welded_mesh_keeps_its_vertex_count_and_geometry(self):
        """Welding is idempotent in effect. Indices may be permuted — `np.unique` orders
        the output lexicographically — so the invariant is geometric, not index-wise."""
        v, f = box_mesh(2.0, 3.0, 4.0)
        wv, wf = weld_vertices(v, f)
        self.assertEqual(len(wv), len(v))
        self.assertTrue(is_watertight(wf)[0])

        before = np.sort(v[f.reshape(-1)].round(12), axis=0)
        after = np.sort(wv[wf.reshape(-1)].round(12), axis=0)
        np.testing.assert_allclose(before, after, atol=0.0)

    def test_is_deterministic(self):
        v, f = box_mesh(2.0, 3.0, 4.0)
        ev, ef = explode(v, f)
        a_v, a_f = weld_vertices(ev, ef)
        b_v, b_f = weld_vertices(ev, ef)
        np.testing.assert_array_equal(a_v, b_v)
        np.testing.assert_array_equal(a_f, b_f)

    def test_empty_mesh_is_returned_unchanged(self):
        v = np.zeros((0, 3), dtype=np.float64)
        f = np.zeros((0, 3), dtype=np.int64)
        wv, wf = weld_vertices(v, f)
        self.assertEqual(len(wv), 0)
        self.assertEqual(len(wf), 0)


class TestIsWatertight(unittest.TestCase):
    def test_closed_box_is_watertight(self):
        _, f = box_mesh(1.0, 1.0, 1.0)
        ok, n_bad = is_watertight(f)
        self.assertTrue(ok)
        self.assertEqual(n_bad, 0)

    def test_box_with_a_missing_face_is_not_watertight(self):
        _, f = box_mesh(1.0, 1.0, 1.0)
        ok, n_bad = is_watertight(f[:-1])
        self.assertFalse(ok)
        self.assertGreater(n_bad, 0)

    def test_empty_mesh_is_not_watertight(self):
        ok, n_bad = is_watertight(np.zeros((0, 3), dtype=np.int64))
        self.assertFalse(ok)
        self.assertEqual(n_bad, 0)


if __name__ == "__main__":
    unittest.main()
