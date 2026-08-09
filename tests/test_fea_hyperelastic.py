"""Material model and the printed-TPU stiffness knock-down.

The CalculiX card format is the load-bearing part: a wrong constant count or ordering is
accepted by the solver and produces a wrong-but-plausible stiffness for every design.
"""

from __future__ import annotations

import unittest

from wheelopt.cad.materials import InfillPattern, MaterialSpec
from wheelopt.fea.hyperelastic import (
    PATTERN_EXPONENT,
    SOLID_TPU_FITS,
    HyperelasticModel,
    UnknownMaterial,
    for_material,
    stiffness_knockdown,
)


def model(order: int = 2) -> HyperelasticModel:
    return HyperelasticModel(
        c=(1.0e6, 2.0e5, 3.0e4, -4.0e3, 5.0e2, 6.0e1, 7.0, 8.0, 9.0),
        d=(1.0e-8, 2.0e-9, 3.0e-10),
        order=order,
        source="test",
    )


class TestCard(unittest.TestCase):
    def test_order_one_emits_three_constants(self):
        card = model(1).calculix_card("M")
        self.assertIn("*HYPERELASTIC, POLYNOMIAL, N=1", card)
        values = card.splitlines()[2].split(",")
        self.assertEqual(len(values), 3)  # C10, C01, D1

    def test_order_two_emits_seven_constants(self):
        card = model(2).calculix_card("M")
        self.assertIn("N=2", card)
        values = [v for line in card.splitlines()[2:] for v in line.split(",")]
        self.assertEqual(len(values), 7)  # C10 C01 C20 C11 C02 D1 D2

    def test_order_three_emits_twelve_constants_over_two_lines(self):
        card = model(3).calculix_card("M")
        self.assertIn("N=3", card)
        body = card.splitlines()[2:]
        self.assertEqual(len(body), 2, "8 constants per line, so 12 needs two lines")
        values = [v for line in body for v in line.split(",")]
        self.assertEqual(len(values), 12)

    def test_constants_are_emitted_in_calculix_order(self):
        card = model(3).calculix_card("M")
        values = [
            float(v) for line in card.splitlines()[2:] for v in line.split(",") if v.strip()
        ]
        # C10 C01 C20 C11 C02 C30 C21 C12 C03 then D1 D2 D3
        self.assertEqual(values[:9], list(model(3).c))
        self.assertEqual(values[9:], list(model(3).d))

    def test_material_name_appears(self):
        self.assertIn("*MATERIAL, NAME=TPU_TEST", model().calculix_card("TPU_TEST"))

    def test_rejects_unsupported_order(self):
        with self.assertRaises(ValueError):
            HyperelasticModel(c=(1e6,) + (0.0,) * 8, d=(0.0, 0.0, 0.0), order=4, source="x")

    def test_rejects_non_physical_stiffness(self):
        with self.assertRaises(ValueError):
            HyperelasticModel(c=(0.0,) * 9, d=(0.0, 0.0, 0.0), order=1, source="x")


class TestModuli(unittest.TestCase):
    def test_initial_shear_modulus(self):
        self.assertAlmostEqual(model().initial_shear_modulus_pa, 2 * (1.0e6 + 2.0e5))

    def test_incompressible_youngs_is_three_mu(self):
        m = HyperelasticModel(
            c=(1e6, 0.0, 0, 0, 0, 0, 0, 0, 0), d=(0.0, 0.0, 0.0), order=1, source="x"
        )
        self.assertAlmostEqual(m.initial_youngs_pa, 3 * m.initial_shear_modulus_pa)
        self.assertAlmostEqual(m.poisson_effective, 0.5)

    def test_with_poisson_round_trips(self):
        for nu in (0.3, 0.45, 0.46, 0.499):
            with self.subTest(nu=nu):
                m = model().with_poisson(nu)
                self.assertAlmostEqual(m.poisson_effective, nu, places=9)

    def test_with_poisson_rejects_incompressible_limit(self):
        for bad in (0.5, 0.6, -1.0):
            with self.subTest(nu=bad), self.assertRaises(ValueError):
                model().with_poisson(bad)

    def test_higher_order_volumetric_terms_are_large_not_zero(self):
        """D2 = 0 does not mean "no second-order term" — it means an infinitely stiff one.

        The volumetric energy is sum_k (1/D_k)(J-1)^(2k), so the coefficient is the
        reciprocal of D. CalculiX accepts D2 = 0, warns that it substituted a default of
        ~1e-15, and proceeds with 1/D2 ~ 1e15 Pa; the wheel comes out volumetrically locked
        and far too stiff with nothing in the results to show why.
        """
        m = model().with_poisson(0.46)
        self.assertGreater(m.d[0], 0.0)
        self.assertGreater(m.d[1], m.d[0] * 1e3)
        self.assertGreater(m.d[2], m.d[1])

    def test_higher_order_volumetric_terms_are_negligible(self):
        """Large D means small 1/D, i.e. the term contributes essentially nothing."""
        m = model().with_poisson(0.46)
        self.assertLess(1.0 / m.d[1], 1e-3 * (1.0 / m.d[0]))

    def test_scaling_multiplies_stiffness_and_preserves_poisson(self):
        base = model().with_poisson(0.46)
        scaled = base.scaled(0.25)
        self.assertAlmostEqual(
            scaled.initial_shear_modulus_pa, 0.25 * base.initial_shear_modulus_pa
        )
        self.assertAlmostEqual(scaled.poisson_effective, base.poisson_effective, places=9)

    def test_scaling_rejects_non_positive(self):
        for bad in (0.0, -1.0):
            with self.subTest(factor=bad), self.assertRaises(ValueError):
                model().scaled(bad)


class TestDigest(unittest.TestCase):
    def test_is_stable(self):
        self.assertEqual(model().coefficient_digest(), model().coefficient_digest())

    def test_changes_when_a_single_coefficient_changes(self):
        other = HyperelasticModel(
            c=model().c[:3] + (model().c[3] * 1.000001,) + model().c[4:],
            d=model().d,
            order=2,
            source="test",
        )
        self.assertNotEqual(model().coefficient_digest(), other.coefficient_digest())

    def test_ignores_the_source_string(self):
        """The source is provenance, not physics — it must not split the cache."""
        other = HyperelasticModel(c=model().c, d=model().d, order=2, source="different")
        self.assertEqual(model().coefficient_digest(), other.coefficient_digest())


class TestKnockdown(unittest.TestCase):
    def test_full_infill_gives_packing_efficiency_only(self):
        m = MaterialSpec(name="TPU_95A", infill_density=1.0)
        self.assertAlmostEqual(stiffness_knockdown(m, 8.0), m.packing_efficiency)

    def test_monotonic_in_infill_density(self):
        values = [
            stiffness_knockdown(MaterialSpec(name="TPU_95A", infill_density=phi), 8.0)
            for phi in (0.2, 0.4, 0.6, 0.8, 1.0)
        ]
        self.assertEqual(values, sorted(values))

    def test_thin_feature_prints_solid_so_infill_is_irrelevant(self):
        """Mirrors the 'infill_ineffective' warning in cad.constraints."""
        thin = 1.0  # below 2 x wall thickness
        a = stiffness_knockdown(MaterialSpec(name="TPU_95A", infill_density=0.2), thin)
        b = stiffness_knockdown(MaterialSpec(name="TPU_95A", infill_density=0.9), thin)
        self.assertAlmostEqual(a, b)

    def test_stiffness_falls_faster_than_mass(self):
        """Gibson-Ashby, n > 1: halving infill roughly halves mass but quarters stiffness.

        The asymmetry with `effective_density_kg_m3` is deliberate; this pins it down so a
        future 'consistency' fix cannot quietly linearise it.
        """
        thick = 8.0
        half = MaterialSpec(name="TPU_95A", infill_density=0.5)
        full = MaterialSpec(name="TPU_95A", infill_density=1.0)
        mass_ratio = half.effective_density_kg_m3(thick) / full.effective_density_kg_m3(thick)
        stiff_ratio = stiffness_knockdown(half, thick) / stiffness_knockdown(full, thick)
        self.assertLess(stiff_ratio, mass_ratio)

    def test_every_pattern_has_an_exponent(self):
        for pattern in InfillPattern:
            self.assertIn(pattern, PATTERN_EXPONENT)

    def test_concentric_is_stiffer_than_grid_at_equal_density(self):
        thick = 8.0
        grid = MaterialSpec(name="TPU_95A", infill_density=0.4,
                            infill_pattern=InfillPattern.GRID)
        conc = MaterialSpec(name="TPU_95A", infill_density=0.4,
                            infill_pattern=InfillPattern.CONCENTRIC)
        self.assertGreater(stiffness_knockdown(conc, thick), stiffness_knockdown(grid, thick))


class TestForMaterial(unittest.TestCase):
    def test_known_material_resolves(self):
        m = for_material(MaterialSpec(name="TPU_95A"), 2.0)
        self.assertGreater(m.initial_shear_modulus_pa, 0)

    def test_unknown_material_raises_loudly(self):
        """Never silently default: a wrong modulus would be invisible in every plot."""
        with self.assertRaises(UnknownMaterial):
            for_material(MaterialSpec(name="PLA"), 2.0)

    def test_harder_shore_is_stiffer(self):
        order = ["TPU_85A", "TPU_95A", "TPU_98A", "TPU_60D"]
        moduli = [SOLID_TPU_FITS[n].initial_shear_modulus_pa for n in order]
        self.assertEqual(moduli, sorted(moduli))

    def test_poisson_is_applied(self):
        m = for_material(MaterialSpec(name="TPU_95A"), 2.0, poisson_effective=0.42)
        self.assertAlmostEqual(m.poisson_effective, 0.42, places=9)


if __name__ == "__main__":
    unittest.main()
