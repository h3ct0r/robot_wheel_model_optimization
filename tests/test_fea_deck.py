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
    # The tip cases prescribe the tread directly and take no indenter at all.
    ind = (
        build_indenter(
            case.kind, case.indenter, PARAMS.outer_radius_mm * 1e-3, PARAMS.width_mm * 1e-3
        )
        if case.kind.needs_indenter
        else None
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


class TestContactPenalty(unittest.TestCase):
    """``TODO.md`` #12: the penalty scales with the mesh, and the scaling is capped."""

    @staticmethod
    def penalty(text: str) -> float:
        """The ``K`` on the line after ``*SURFACE INTERACTION``'s overclosure card."""
        lines = text.splitlines()
        i = next(k for k, line in enumerate(lines)
                 if line.startswith("*SURFACE BEHAVIOR"))
        return float(lines[i + 1].split(",")[0])

    def _mesh_scaled(self, scale: float) -> FeaMesh:
        """The fixture mesh with every coordinate multiplied, so the elements change size."""
        mesh = fixture_mesh()
        return FeaMesh(
            nodes_m=mesh.nodes_m * scale,
            elements=mesh.elements,
            element_type=mesh.element_type,
            node_sets=mesh.node_sets,
            element_sets=mesh.element_sets,
        )

    def test_the_penalty_is_proportional_to_the_factor(self):
        one = self.penalty(build().text)
        solver = SolverSpec(contact_stiffness_factor=2.0 * SolverSpec()
                            .contact_stiffness_factor)
        two = self.penalty(build_deck(
            fixture_mesh(),
            build_indenter(LoadCaseKind.RADIAL_FLAT, LoadCase().indenter,
                           PARAMS.outer_radius_mm * 1e-3, PARAMS.width_mm * 1e-3),
            PARAMS, MATERIAL, HYPER, LoadCase(), solver,
            design_hash="deadbeef", cache_key="cafe1234",
        ).text)
        self.assertAlmostEqual(two / one, 2.0, places=6)

    def test_a_coarse_mesh_is_below_the_floor_and_still_scales(self):
        """Above the floor the penalty must still track the element size, or the cap has
        replaced the scaling instead of bounding it — and a soft design would then contact
        like a stiff one, which is invariant 2."""
        coarse = self.penalty(build(mesh=self._mesh_scaled(4.0)).text)
        coarser = self.penalty(build(mesh=self._mesh_scaled(8.0)).text)
        self.assertAlmostEqual(coarse / coarser, 2.0, places=6)

    def test_refining_past_the_floor_stops_stiffening_the_penalty(self):
        """The cap itself. Unfloored these two differ by 4x, and the finer one diverges:
        measured 2026-08-09, factor 5 converges at a 4 mm element and does not at 2.5 or
        1.5 mm, while holding the penalty at the 4 mm value converges at all three."""
        fine = self.penalty(build(mesh=self._mesh_scaled(0.5)).text)
        finer = self.penalty(build(mesh=self._mesh_scaled(0.125)).text)
        self.assertEqual(fine, finer)

    def test_the_floor_can_be_switched_off(self):
        solver = SolverSpec(contact_length_floor_m=0.0)
        def penalty_at(scale: float) -> float:
            return self.penalty(build_deck(
                self._mesh_scaled(scale),
                build_indenter(LoadCaseKind.RADIAL_FLAT, LoadCase().indenter,
                               PARAMS.outer_radius_mm * 1e-3, PARAMS.width_mm * 1e-3),
                PARAMS, MATERIAL, HYPER, LoadCase(), solver,
                design_hash="deadbeef", cache_key="cafe1234",
            ).text)
        self.assertAlmostEqual(penalty_at(0.125) / penalty_at(0.5), 4.0, places=6)

    def test_the_default_is_the_softened_penalty(self):
        """A regression guard on the decision, not on the number: #12 moved the default from
        20 to 5 because 20 diverged on the fine section mesh. A silent drift back would
        reintroduce a failure that reads as "this contact problem is unsolvable"."""
        self.assertEqual(SolverSpec().contact_stiffness_factor, 5.0)
        self.assertGreater(SolverSpec().contact_length_floor_m, 0.0)


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


class TestPrescribedTipDeck(unittest.TestCase):
    """The contact-free cases: no indenter, the tread itself driven as a rigid body."""

    def deck(self, kind: LoadCaseKind) -> str:
        return build(LoadCase(kind=kind, delta_max_m=0.006)).text

    def test_there_is_no_contact_at_all(self):
        """The point of these cases: nothing about the answer can come from the contact
        model, because there is not one."""
        text = self.deck(LoadCaseKind.TIP_RADIAL)
        for keyword in ("*CONTACT PAIR", "*SURFACE INTERACTION", "*FRICTION",
                        "*CONTACT PRINT", "EINDENT"):
            self.assertNotIn(keyword, text)

    def test_the_tread_is_the_rigid_body(self):
        text = self.deck(LoadCaseKind.TIP_RADIAL)
        self.assertIn("*RIGID BODY, NSET=NTREAD", text)

    def test_radial_drives_y_and_holds_x(self):
        text = self.deck(LoadCaseKind.TIP_RADIAL)
        ref = _ref_node(text)
        self.assertIn(f" {ref}, 2, 2, 6.000000000e-03", text)   # driven
        self.assertIn(f" {ref}, 1, 1, 0.0", text)               # tangential held

    def test_tangential_drives_x_and_leaves_y_free(self):
        """The bug this pins: holding y as well forces the claw to *stretch* instead of
        bend, because a bending tip sweeps an arc and must come radially inward. Measured
        while it was wrong — 7.35 N/mm against a beam-theory 0.06, and constant with
        displacement because nothing was bending."""
        text = self.deck(LoadCaseKind.TIP_TANGENTIAL)
        ref = _ref_node(text)
        self.assertIn(f" {ref}, 1, 1, 6.000000000e-03", text)   # driven
        self.assertNotIn(f" {ref}, 2, 2, 0.0", text)            # radial must stay free

    def test_an_indenter_is_refused_rather_than_ignored(self):
        case = LoadCase(kind=LoadCaseKind.TIP_RADIAL)
        ind = build_indenter(
            LoadCaseKind.RADIAL_FLAT, case.indenter,
            PARAMS.outer_radius_mm * 1e-3, PARAMS.width_mm * 1e-3,
        )
        with self.assertRaises(DeckError):
            build_deck(fixture_mesh(), ind, PARAMS, MATERIAL, HYPER, case, SolverSpec())

    def test_a_contact_case_without_an_indenter_is_refused(self):
        with self.assertRaises(DeckError):
            build_deck(fixture_mesh(), None, PARAMS, MATERIAL, HYPER,
                       LoadCase(kind=LoadCaseKind.RADIAL_FLAT), SolverSpec())


def _ref_node(text: str) -> int:
    """The id declared by ``*NSET, NSET=NREF``."""
    lines = text.splitlines()
    return int(lines[lines.index("*NSET, NSET=NREF") + 1].strip())


if __name__ == "__main__":
    unittest.main()
