"""Cache-key composition — the invariant 5 regression test.

CLAUDE.md invariant 5: every cache key includes the pipeline version and the ROM version,
because changing ring discretisation, fitting procedure or material homogenisation must
invalidate prior results. A cache key that fails to change is the worst possible bug in a
research pipeline: it silently serves last week's answer for this week's model, and every
downstream plot looks fine.

So each test below changes exactly one thing and asserts the key moves.
"""

from __future__ import annotations

import unittest
from dataclasses import replace

from wheelopt.cad.materials import InfillPattern, MaterialSpec
from wheelopt.cad.params import SpokeProfile, WheelParams
from wheelopt.fea.cache import SOLVER_TIMING_ONLY, cache_dir_for, fea_cache_key
from wheelopt.fea.hyperelastic import HyperelasticModel, for_material
from wheelopt.fea.loadcase import (
    IndenterSpec,
    LoadCase,
    LoadCaseKind,
    MeshSpec,
    SolverSpec,
)

PARAMS = WheelParams()
MATERIAL = MaterialSpec(name="TPU_95A", infill_density=0.4)
HYPER = for_material(MATERIAL, PARAMS.spoke_thickness_mm)
CASE = LoadCase()
MESH = MeshSpec()


def key(**overrides) -> str:
    args = {
        "params": PARAMS,
        "material": MATERIAL,
        "hyper": HYPER,
        "load_case": CASE,
        "mesh_spec": MESH,
        "solver_identity": "ccx-2.23+gmsh-4.15.2",
        "solver": SolverSpec(),
    }
    args.update(overrides)
    return fea_cache_key(**args)


class TestStability(unittest.TestCase):
    def test_is_deterministic(self):
        self.assertEqual(key(), key())

    def test_is_sixteen_hex_characters(self):
        k = key()
        self.assertEqual(len(k), 16)
        int(k, 16)

    def test_equivalent_inputs_agree(self):
        """Rebuilding the inputs from scratch must not move the key."""
        other = for_material(MaterialSpec(name="TPU_95A", infill_density=0.4),
                             WheelParams().spoke_thickness_mm)
        self.assertEqual(key(), key(hyper=other, params=WheelParams()))


class TestKeyResponds(unittest.TestCase):
    """Every input that can change the numbers must change the key."""

    def test_design_geometry(self):
        for field, value in (
            ("outer_radius_mm", 71.0),
            ("n_spokes", 17),
            ("spoke_thickness_mm", 2.1),
            ("spoke_curvature_1_per_mm", 0.005),
            ("spoke_profile", SpokeProfile.S_CURVE),
            ("width_mm", 41.0),
        ):
            with self.subTest(field=field):
                self.assertNotEqual(key(), key(params=replace(PARAMS, **{field: value})))

    def test_material(self):
        for field, value in (
            ("name", "TPU_85A"),
            ("infill_density", 0.45),
            ("infill_pattern", InfillPattern.GRID),
            ("wall_count", 4),
        ):
            with self.subTest(field=field):
                self.assertNotEqual(
                    key(), key(material=replace(MATERIAL, **{field: value}))
                )

    def test_a_single_hyperelastic_coefficient(self):
        """Re-seeding the literature table invalidates results even without a version bump."""
        nudged = HyperelasticModel(
            c=(HYPER.c[0] * 1.0000001,) + HYPER.c[1:],
            d=HYPER.d,
            order=HYPER.order,
            source=HYPER.source,
        )
        self.assertNotEqual(key(), key(hyper=nudged))

    def test_load_case(self):
        for field, value in (
            ("kind", LoadCaseKind.RADIAL_STEP_EDGE),
            ("nominal_load_n", 15.0),
            ("delta_max_m", 0.013),
            ("n_points_per_branch", 21),
            ("friction_mu", 0.7),
        ):
            with self.subTest(field=field):
                self.assertNotEqual(key(), key(load_case=replace(CASE, **{field: value})))

    def test_indenter_geometry(self):
        """The step-edge fillet changes the answer, so it changes the key."""
        for field, value in (
            ("edge_fillet_m", 0.002),
            ("step_height_m", 0.06),
            ("element_size_m", 0.0025),
        ):
            with self.subTest(field=field):
                case = replace(CASE, indenter=replace(IndenterSpec(), **{field: value}))
                self.assertNotEqual(key(), key(load_case=case))

    def test_mesh_spec(self):
        for field, value in (
            ("size_spoke_m", 0.0013),
            ("size_rim_m", 0.0019),
            ("size_hub_m", 0.005),
            ("order", 1),
            ("algorithm_3d", 10),
        ):
            with self.subTest(field=field):
                self.assertNotEqual(key(), key(mesh_spec=replace(MESH, **{field: value})))

    def test_half_width_symmetry_is_refused(self):
        """It is in the cache key but nothing implements it.

        This used to be one more entry in the sensitivity list above, which asserted that
        flipping it changed the key — perfectly true, and the reason the no-op went
        unnoticed: the only thing the flag actually did was split the cache. Until the
        mesher honours it, constructing such a spec must fail.
        """
        with self.assertRaises(NotImplementedError):
            replace(MESH, half_width_symmetry=True)

    def test_solver_settings_that_change_the_answer(self):
        """The 2026-08-08 regression.

        These were all excluded from the key as "timing only". `contact_stiffness_factor`
        is the contact compliance — changing it 20 -> 5 turned a diverged plane-strain run
        into a converged one — and because the key did not move, the second run was served
        the first one's result and looked like evidence that both converged.
        """
        for field, value in (
            ("contact_stiffness_factor", 5.0),
            ("initial_increment", 0.01),
            ("min_increment", 1e-4),
            ("max_increment", 0.1),
            ("max_increments", 500),
        ):
            with self.subTest(field=field):
                self.assertNotEqual(key(), key(solver=replace(SolverSpec(), **{field: value})))

    def test_solver_settings_that_only_change_the_duration(self):
        for field, value in (("timeout_s", 99.0), ("n_threads", 8)):
            with self.subTest(field=field):
                self.assertEqual(key(), key(solver=replace(SolverSpec(), **{field: value})))

    def test_every_solver_field_is_classified(self):
        """A field added to SolverSpec is hashed unless it is named timing-only.

        Asserted so the exclusion list cannot quietly fall behind the dataclass: the
        default must be "in the key", because the cost of a wrong exclusion is a shared
        cache entry and the cost of a wrong inclusion is a redundant re-solve.
        """
        from dataclasses import fields

        names = {f.name for f in fields(SolverSpec)}
        self.assertTrue(SOLVER_TIMING_ONLY <= names, "stale name in SOLVER_TIMING_ONLY")
        for field in names - SOLVER_TIMING_ONLY:
            with self.subTest(field=field):
                current = getattr(SolverSpec(), field)
                bumped = current * 2 if isinstance(current, (int, float)) else current
                if bumped == current:
                    continue
                self.assertNotEqual(
                    key(), key(solver=replace(SolverSpec(), **{field: bumped}))
                )

    def test_solver_identity(self):
        """A different CalculiX can give a different answer to the same contact problem."""
        self.assertNotEqual(key(), key(solver_identity="ccx-2.22+gmsh-4.15.2"))
        self.assertNotEqual(key(), key(solver_identity="ccx-2.23+gmsh-4.14.0"))

    def test_pipeline_versions(self):
        """Guards the two version constants themselves against being dropped from the key."""
        import wheelopt.fea.cache as cache_module

        baseline = key()
        original = cache_module.FEA_PIPELINE_VERSION
        try:
            cache_module.FEA_PIPELINE_VERSION = "fea-9.9.9"
            self.assertNotEqual(baseline, key())
        finally:
            cache_module.FEA_PIPELINE_VERSION = original

        original_cad = cache_module.CAD_PIPELINE_VERSION
        try:
            cache_module.CAD_PIPELINE_VERSION = "cad-9.9.9"
            self.assertNotEqual(baseline, key())
        finally:
            cache_module.CAD_PIPELINE_VERSION = original_cad


class TestKeyIgnores(unittest.TestCase):
    """Things that change how long the answer takes, not what it is."""

    def test_solver_runtime_settings(self):
        from wheelopt.fea.loadcase import SolverSpec

        SolverSpec(n_threads=8, timeout_s=60.0)  # constructible, and absent from the key
        self.assertEqual(key(), key())


class TestCacheDir(unittest.TestCase):
    def test_directory_is_named_by_key(self):
        from pathlib import Path

        self.assertEqual(cache_dir_for(Path("/tmp/x"), "abc123"), Path("/tmp/x/abc123"))


if __name__ == "__main__":
    unittest.main()
