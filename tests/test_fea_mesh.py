"""Mesh-layer checks that do not need gmsh.

The quadratic-Jacobian test is pure numpy against hand-built C3D10 elements, so it runs
anywhere. It guards the check that catches a folded second-order tet before CalculiX does —
without it, a curved mid-side node produces "nonpositive jacobian determinant" and the
whole solve dies at t=0 with nothing typed to explain why.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.fea.mesh import (
    GMSH_TO_ABAQUS_TET10,
    _min_quadratic_jacobian,
    classify_nodes,
)
from wheelopt.cad.params import WheelParams
from wheelopt.fea.loadcase import MeshSpec


def straight_tet10(corners: np.ndarray) -> np.ndarray:
    """A C3D10 with mid-side nodes at the edge midpoints, Abaqus order."""
    pairs = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]
    mids = np.array([(corners[a] + corners[b]) / 2 for a, b in pairs])
    return np.vstack([corners, mids])


UNIT = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)


class TestQuadraticJacobian(unittest.TestCase):
    def test_a_healthy_element_has_positive_jacobian(self):
        nodes = straight_tet10(UNIT)
        elems = np.arange(1, 11, dtype=np.int64).reshape(1, 10)
        det = _min_quadratic_jacobian(nodes, elems)
        self.assertTrue(bool((det > 0).all()))

    def test_a_folded_mid_side_node_is_caught(self):
        """Push one mid-side node far past the far corner; the element inverts locally
        even though the four corners still enclose a positive volume."""
        nodes = straight_tet10(UNIT)
        nodes[4] = np.array([5.0, 5.0, 5.0])  # midpoint of edge (0,1), yanked away
        elems = np.arange(1, 11, dtype=np.int64).reshape(1, 10)
        det = _min_quadratic_jacobian(nodes, elems)
        self.assertTrue(bool((det <= 0).any()))

    def test_scales_with_element_volume_not_sign(self):
        big = _min_quadratic_jacobian(straight_tet10(UNIT * 2.0),
                                      np.arange(1, 11).reshape(1, 10))
        small = _min_quadratic_jacobian(straight_tet10(UNIT),
                                        np.arange(1, 11).reshape(1, 10))
        self.assertGreater(big.min(), small.min())
        self.assertGreater(small.min(), 0)


class TestNodeOrdering(unittest.TestCase):
    def test_gmsh_to_abaqus_swaps_only_the_last_two_mid_side_nodes(self):
        """The one difference between the two conventions, isolated."""
        self.assertEqual(GMSH_TO_ABAQUS_TET10[:8], (0, 1, 2, 3, 4, 5, 6, 7))
        self.assertEqual(GMSH_TO_ABAQUS_TET10[8:], (9, 8))
        self.assertEqual(sorted(GMSH_TO_ABAQUS_TET10), list(range(10)))


class TestClassifyNodes(unittest.TestCase):
    def setUp(self):
        self.params = WheelParams()
        self.spec = MeshSpec()

    def _ring(self, radius_m: float, n: int = 8) -> np.ndarray:
        ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
        return np.stack([radius_m * np.cos(ang), radius_m * np.sin(ang),
                         np.zeros(n)], axis=1)

    def test_bore_and_tread_are_found_by_radius(self):
        bore_r = self.params.hub_bore_radius_mm * 1e-3
        tread_r = self.params.outer_radius_mm * 1e-3
        nodes = np.vstack([self._ring(bore_r), self._ring(tread_r)])
        sets = classify_nodes(nodes, self.params, self.spec)
        self.assertEqual(len(sets["bore"]), 8)
        self.assertEqual(len(sets["tread"]), 8)

    def test_groove_floor_nodes_are_not_classified_as_tread(self):
        """With a tread groove, only the outer cylinder contacts a flat plate — the floors
        at R - depth must be excluded from the slave surface."""
        params = WheelParams(tread_depth_mm=2.0)
        bore_r = params.hub_bore_radius_mm * 1e-3
        tread_r = params.outer_radius_mm * 1e-3
        floor_r = tread_r - params.tread_depth_mm * 1e-3
        nodes = np.vstack([self._ring(bore_r), self._ring(tread_r), self._ring(floor_r)])
        sets = classify_nodes(nodes, params, self.spec)
        self.assertEqual(len(sets["tread"]), 8)

    def test_missing_bore_nodes_is_a_mesh_failure(self):
        from wheelopt.fea.mesh import MeshFailure

        nodes = self._ring(self.params.outer_radius_mm * 1e-3)  # tread only
        with self.assertRaises(MeshFailure):
            classify_nodes(nodes, self.params, self.spec)


if __name__ == "__main__":
    unittest.main()
