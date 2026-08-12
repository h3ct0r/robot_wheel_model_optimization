"""Material homogenisation and parameter bookkeeping."""

from __future__ import annotations

import unittest

from wheelopt.cad.materials import BASE_DENSITIES_KG_M3, PLA, TPU95A, MaterialSpec
from wheelopt.cad.params import SpokeProfile, WheelParams


class TestShellFraction(unittest.TestCase):
    def test_thin_feature_prints_solid(self):
        """Anything up to twice the wall thickness is all perimeter, no infill."""
        m = MaterialSpec(name="TPU_95A", wall_count=3, nozzle_diameter_mm=0.4)
        self.assertAlmostEqual(m.wall_thickness_mm, 1.2, places=12)
        self.assertEqual(m.shell_fraction(1.2), 1.0)
        self.assertEqual(m.shell_fraction(2.4), 1.0)

    def test_thick_feature_has_infill_core(self):
        m = MaterialSpec(name="TPU_95A", wall_count=2, nozzle_diameter_mm=0.4)
        self.assertAlmostEqual(m.shell_fraction(8.0), 1.6 / 8.0, places=12)

    def test_shell_fraction_decreases_with_thickness(self):
        m = MaterialSpec(name="TPU_95A", wall_count=2)
        values = [m.shell_fraction(t) for t in (2.0, 4.0, 8.0, 16.0)]
        self.assertEqual(values, sorted(values, reverse=True))

    def test_degenerate_thickness_is_solid(self):
        self.assertEqual(MaterialSpec(name="PLA").shell_fraction(0.0), 1.0)
        self.assertEqual(MaterialSpec(name="PLA").shell_fraction(-1.0), 1.0)


class TestEffectiveDensity(unittest.TestCase):
    def test_solid_feature_ignores_infill_setting(self):
        """The trap this guards: believing infill reduces the mass of a thin spoke."""
        sparse = MaterialSpec(name="TPU_95A", infill_density=0.15, wall_count=3)
        dense = MaterialSpec(name="TPU_95A", infill_density=1.0, wall_count=3)
        self.assertAlmostEqual(
            sparse.effective_density_kg_m3(2.0),
            dense.effective_density_kg_m3(2.0),
            places=12,
        )

    def test_infill_matters_for_thick_features(self):
        sparse = MaterialSpec(name="TPU_95A", infill_density=0.15, wall_count=2)
        dense = MaterialSpec(name="TPU_95A", infill_density=1.0, wall_count=2)
        self.assertLess(
            sparse.effective_density_kg_m3(20.0), dense.effective_density_kg_m3(20.0)
        )

    def test_full_infill_reduces_to_base_times_packing(self):
        m = MaterialSpec(name="PLA", infill_density=1.0, packing_efficiency=0.95)
        self.assertAlmostEqual(
            m.effective_density_kg_m3(20.0),
            BASE_DENSITIES_KG_M3["PLA"] * 0.95,
            places=9,
        )

    def test_effective_density_never_exceeds_base(self):
        for name in BASE_DENSITIES_KG_M3:
            for fill in (0.0, 0.25, 0.5, 1.0):
                m = MaterialSpec(name=name, infill_density=fill)
                for t in (1.0, 3.0, 10.0):
                    with self.subTest(name=name, fill=fill, t=t):
                        self.assertLessEqual(
                            m.effective_density_kg_m3(t), m.base_density_kg_m3
                        )

    def test_pla_is_denser_than_tpu(self):
        """PLA (~1240) is denser than TPU (~1210), not the other way round.

        Worth pinning: the intuition that "rubbery means heavier" is wrong here, and a
        sign error in the density table would bias every compliant-versus-rigid mass
        comparison in the campaign.
        """
        solid_tpu = MaterialSpec(name="TPU_95A", infill_density=1.0)
        solid_pla = MaterialSpec(name="PLA", infill_density=1.0)
        self.assertGreater(
            solid_pla.effective_density_kg_m3(10.0), solid_tpu.effective_density_kg_m3(10.0)
        )

    def test_shore_hardness_ordering_of_densities(self):
        """Harder TPU grades are slightly denser; the table must stay monotonic."""
        grades = ["TPU_85A", "TPU_95A", "TPU_98A", "TPU_60D"]
        densities = [BASE_DENSITIES_KG_M3[g] for g in grades]
        self.assertEqual(densities, sorted(densities))


class TestMaterialValidation(unittest.TestCase):
    """A bad material spec is a configuration error and must fail loudly."""

    def test_rejects_out_of_range_infill(self):
        for bad in (-0.1, 1.5):
            with self.subTest(value=bad), self.assertRaises(ValueError):
                MaterialSpec(name="PLA", infill_density=bad)

    def test_rejects_zero_walls(self):
        with self.assertRaises(ValueError):
            MaterialSpec(name="PLA", wall_count=0)

    def test_rejects_unknown_material(self):
        with self.assertRaises(KeyError):
            MaterialSpec(name="unobtainium")

    def test_rejects_bad_packing_efficiency(self):
        with self.assertRaises(ValueError):
            MaterialSpec(name="PLA", packing_efficiency=0.0)

    def test_elastomer_detection(self):
        self.assertTrue(TPU95A.is_elastomer)
        self.assertFalse(PLA.is_elastomer)


class TestParams(unittest.TestCase):
    def test_derived_radii(self):
        p = WheelParams(outer_radius_mm=70.0, rim_thickness_mm=3.0, hub_radius_mm=25.0)
        self.assertAlmostEqual(p.rim_inner_radius_mm, 67.0)
        self.assertAlmostEqual(p.spoke_span_mm, 42.0)

    def test_straight_profile_has_zero_sagitta(self):
        p = WheelParams(spoke_profile=SpokeProfile.STRAIGHT, spoke_curvature_1_per_mm=0.02)
        self.assertEqual(p.spoke_sagitta_mm, 0.0)

    def test_sagitta_scales_with_span_squared(self):
        rim_inner = WheelParams().rim_inner_radius_mm  # 82 mm at the nominal design
        a = WheelParams(hub_radius_mm=rim_inner - 60.0)  # span 60
        b = WheelParams(hub_radius_mm=rim_inner - 30.0)  # span 30
        self.assertAlmostEqual(a.spoke_sagitta_mm / b.spoke_sagitta_mm, 4.0, places=9)

    def test_to_si_converts_lengths_and_curvature(self):
        si = WheelParams(outer_radius_mm=70.0, spoke_curvature_1_per_mm=0.004).to_si()
        self.assertAlmostEqual(si["outer_radius_m"], 0.070, places=12)
        self.assertAlmostEqual(si["spoke_curvature_1_per_m"], 4.0, places=12)
        self.assertNotIn("outer_radius_mm", si)

    def test_to_si_preserves_non_length_fields(self):
        si = WheelParams(n_spokes=19).to_si()
        self.assertEqual(si["n_spokes"], 19)

    def test_design_hash_is_stable_and_sensitive(self):
        a = WheelParams()
        self.assertEqual(a.design_hash(), WheelParams().design_hash())
        self.assertNotEqual(a.design_hash(), WheelParams(n_spokes=17).design_hash())
        self.assertNotEqual(
            a.design_hash(), WheelParams(spoke_profile=SpokeProfile.S_CURVE).design_hash()
        )

    def test_design_hash_detects_tiny_changes(self):
        a = WheelParams(spoke_thickness_mm=2.0)
        b = WheelParams(spoke_thickness_mm=2.0000001)
        self.assertNotEqual(a.design_hash(), b.design_hash())

    def test_params_are_immutable(self):
        import dataclasses

        with self.assertRaises(dataclasses.FrozenInstanceError):
            WheelParams().outer_radius_mm = 99.0  # type: ignore[misc]

    def test_bounding_box(self):
        bx, by, bz = WheelParams(outer_radius_mm=70.0, width_mm=40.0).bounding_box_mm
        self.assertEqual((bx, by, bz), (140.0, 140.0, 40.0))


if __name__ == "__main__":
    unittest.main()
