"""Deck assembly against a hand-built mesh.

Pure text generation, so this runs with neither gmsh nor CalculiX installed. The fixture
mesh is two tetrahedra placed at radii that put one in the hub and one in the spokes, which
is enough to exercise set membership, numbering and every card the real deck emits.

The unit guard matters most: the CAD layer works in millimetres and the deck in metres, and
a deck written in millimetres solves happily while reporting stiffnesses off by 10^3.
"""

from __future__ import annotations

import unittest

import numpy as np

from wheelopt.cad.materials import MaterialSpec
from wheelopt.cad.params import WheelParams
from wheelopt.fea.deck import DeckError, build_deck
from wheelopt.fea.hyperelastic import for_material
from wheelopt.fea.indenter import build_indenter
from wheelopt.fea.loadcase import LoadCase, LoadCaseKind, SolverSpec
from wheelopt.fea.mesh import FeaMesh

PARAMS = WheelParams()
MATERIAL = MaterialSpec(name="TPU_95A", infill_density=0.4)
HYPER = for_material(MATERIAL, PARAMS.spoke_thickness_mm)


def fixture_mesh() -> FeaMesh:
    """Two C3D10 tets: one near the axle, one out among the spokes."""
    def tet(cx: float) -> np.ndarray:
        a = 0.004
        corners = np.array(
            [[cx, 0, 0], [cx + a, 0, 0], [cx, a, 0], [cx, 0, a]], dtype=np.float64
        )
        pairs = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]
        mids = np.array([(corners[i] + corners[j]) / 2 for i, j in pairs])
        return np.vstack([corners, mids])

    nodes = np.vstack([tet(0.010), tet(0.045)])
    elements = np.array(
        [list(range(1, 11)), list(range(11, 21))], dtype=np.int64
    )
    return FeaMesh(
        nodes_m=nodes,
        elements=elements,
        element_type="C3D10",
        node_sets={
            "bore": np.array([1, 2, 3], dtype=np.int64),
            "tread": np.array([11, 12, 13, 14], dtype=np.int64),
            "hub": np.arange(1, 11, dtype=np.int64),
            "rim": np.arange(11, 21, dtype=np.int64),
            "spokes": np.arange(11, 21, dtype=np.int64),
        },
        element_sets={
            "hub": np.array([1], dtype=np.int64),
            "rim": np.array([], dtype=np.int64),
            "spokes": np.array([2], dtype=np.int64),
        },
    )


def build(load_case: LoadCase | None = None, mesh: FeaMesh | None = None):
    case = load_case or LoadCase()
    m = mesh or fixture_mesh()
    ind = build_indenter(
        case.kind, case.indenter, PARAMS.outer_radius_mm * 1e-3, PARAMS.width_mm * 1e-3
    )
    return build_deck(
        m, ind, PARAMS, MATERIAL, HYPER, case, SolverSpec(),
        design_hash="deadbeef", cache_key="cafe1234",
    )


class TestStructure(unittest.TestCase):
    def setUp(self):
        self.deck = build().text

    def test_has_the_required_cards(self):
        for card in (
            "*NODE, NSET=NALL",
            "*ELEMENT, TYPE=C3D10, ELSET=EWHEEL",
            "*ELEMENT, TYPE=C3D8, ELSET=EINDENT",
            "*HYPERELASTIC, POLYNOMIAL",
            "*SOLID SECTION, ELSET=EWHEEL",
            "*RIGID BODY, ELSET=EINDENT",
            "*SURFACE INTERACTION, NAME=SI1",
            "*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR",
            "*CONTACT PAIR, INTERACTION=SI1, TYPE=NODE TO SURFACE",
            "*AMPLITUDE, NAME=SWEEP",
            "*TIME POINTS, NAME=TP",
            "*STEP, NLGEOM",
            "*STATIC",
            "*END STEP",
        ):
            with self.subTest(card=card):
                self.assertIn(card, self.deck)

    def test_requests_the_outputs_extraction_needs(self):
        for request in (
            "*NODE PRINT, NSET=NREF",
            "*CONTACT PRINT",
            "*EL PRINT, ELSET=ESPOKES",
        ):
            with self.subTest(request=request):
                self.assertIn(request, self.deck)
        self.assertIn("CSTR", self.deck)
        self.assertIn(" RF", self.deck)

    def test_bore_is_fully_restrained(self):
        self.assertIn("NBORE, 1, 3, 0.0", self.deck)

    def test_loading_is_displacement_controlled(self):
        """A prescribed displacement on the reference node, not a force."""
        self.assertIn("*BOUNDARY, AMPLITUDE=SWEEP", self.deck)
        self.assertNotIn("*CLOAD", self.deck)
        self.assertNotIn("*DLOAD", self.deck)

    def test_amplitude_loads_then_unloads(self):
        self.assertIn("0.0, 0.0, 1.0, 1.0, 2.0, 0.0", self.deck)

    def test_provenance_is_recorded(self):
        self.assertIn("deadbeef", self.deck)
        self.assertIn("cafe1234", self.deck)
        self.assertIn("fea-0.1.0", self.deck)

    def test_ends_with_a_newline(self):
        self.assertTrue(self.deck.endswith("\n"))


class TestNumbering(unittest.TestCase):
    def test_node_ids_are_contiguous_from_one(self):
        deck = build()
        ids = []
        for line in deck.text.splitlines():
            if line.startswith("*"):
                if ids:
                    break
                continue
            if ids or line[0].isdigit():
                ids.append(int(line.split(",")[0]))
        self.assertEqual(ids[0], 1)
        self.assertEqual(ids, list(range(1, len(ids) + 1)))

    def test_reference_node_is_the_last_two_nodes(self):
        deck = build()
        self.assertEqual(deck.ref_node, deck.n_nodes - 1)

    def test_indenter_elements_follow_the_wheel(self):
        deck = build()
        self.assertEqual(deck.n_elements, 2 + build_indenter(
            LoadCaseKind.RADIAL_FLAT, LoadCase().indenter, 0.070, 0.040).n_elements)

    def test_slave_nodes_match_the_tread_set(self):
        deck = build()
        np.testing.assert_array_equal(deck.slave_nodes, fixture_mesh().node_sets["tread"])
        self.assertEqual(len(deck.slave_coords_m), len(deck.slave_nodes))


class TestUnits(unittest.TestCase):
    def test_coordinates_are_metres_not_millimetres(self):
        """The mm/m guard. A wheel is O(0.1) m; in mm every coordinate would be O(100)."""
        deck = build()
        coords = []
        for line in deck.text.splitlines():
            if line.startswith("*NODE"):
                continue
            if line.startswith("*"):
                if coords:
                    break
                continue
            parts = line.split(",")
            if len(parts) == 4:
                coords.extend(abs(float(p)) for p in parts[1:])
        self.assertGreater(len(coords), 0)
        self.assertLess(max(coords), 1.0, "coordinates look like millimetres")

    def test_moduli_are_pascals(self):
        deck = build().text
        self.assertIn("2.100000000e+11", deck)  # rigid indenter E


class TestDeterminism(unittest.TestCase):
    def test_identical_inputs_give_byte_identical_output(self):
        """If this fails, the cache is serving results for a deck it did not produce."""
        self.assertEqual(build().text, build().text)

    def test_load_case_changes_the_deck(self):
        flat = build(LoadCase(kind=LoadCaseKind.RADIAL_FLAT)).text
        step = build(LoadCase(kind=LoadCaseKind.RADIAL_STEP_EDGE)).text
        self.assertNotEqual(flat, step)


class TestValidation(unittest.TestCase):
    def test_rejects_an_empty_mesh(self):
        mesh = fixture_mesh()
        empty = FeaMesh(
            nodes_m=mesh.nodes_m,
            elements=np.zeros((0, 10), dtype=np.int64),
            element_type="C3D10",
            node_sets=mesh.node_sets,
            element_sets=mesh.element_sets,
        )
        with self.assertRaises(DeckError):
            build(mesh=empty)

    def test_rejects_an_empty_bore_set(self):
        mesh = fixture_mesh()
        mesh.node_sets["bore"] = np.array([], dtype=np.int64)
        with self.assertRaises(DeckError):
            build(mesh=mesh)

    def test_rejects_an_empty_tread_set(self):
        mesh = fixture_mesh()
        mesh.node_sets["tread"] = np.array([], dtype=np.int64)
        with self.assertRaises(DeckError):
            build(mesh=mesh)

    def test_rejects_an_empty_spoke_element_set(self):
        mesh = fixture_mesh()
        mesh.element_sets["spokes"] = np.array([], dtype=np.int64)
        with self.assertRaises(DeckError):
            build(mesh=mesh)


class TestTimePoints(unittest.TestCase):
    def test_output_grid_is_explicit_not_frequency_based(self):
        """`FREQUENCY=n` would give a solver-dependent grid and an irreproducible k_r."""
        deck = build().text
        self.assertIn("TIME POINTS=TP", deck)
        self.assertNotIn("FREQUENCY", deck)

    def test_grid_covers_both_branches(self):
        case = LoadCase(n_points_per_branch=5)
        points = case.time_points()
        self.assertEqual(len(points), 10)
        self.assertAlmostEqual(points[-1], 2.0)
        self.assertIn(1.0, points)

    def test_exactly_one_time_points_set_is_defined(self):
        """CalculiX ignores the per-request `TIME POINTS=<name>` when several sets exist —
        the last set defined wins for every output. A second, sparser set intended just for
        stress output therefore thins out the load curve too, which is how a 12-point sweep
        silently became a 4-point one."""
        deck = build().text
        self.assertEqual(deck.count("*TIME POINTS"), 1)

    def test_every_output_request_uses_that_one_set(self):
        deck = build().text
        for line in deck.splitlines():
            if "TIME POINTS=" in line and not line.startswith("*TIME POINTS"):
                with self.subTest(line=line):
                    self.assertIn("TIME POINTS=TP", line)


if __name__ == "__main__":
    unittest.main()
