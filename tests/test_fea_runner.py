"""Runner failure handling and result typing, using a stub solver.

Invariant 4: *nothing kills a campaign*. Every way CalculiX can fail must arrive as a typed
:class:`FeaResult`, never as an exception. That whole surface is testable without the real
binary by pointing ``find_ccx`` at a shell script that misbehaves on demand — which is what
these tests do, so they run anywhere.
"""

from __future__ import annotations

import os
import stat
import tempfile
import unittest
from pathlib import Path

from wheelopt.cad.materials import MaterialSpec
from wheelopt.cad.params import WheelParams
from wheelopt.fea.loadcase import LoadCase, LoadCaseKind
from wheelopt.fea.results import FeaResult, FeaStatus, SolverDiagnostics, failure
from wheelopt.fea.runner import find_ccx, run_load_case, solver_identity, summarise

PARAMS = WheelParams()
TPU = MaterialSpec(name="TPU_95A", infill_density=0.4)


def stub_solver(directory: Path, body: str) -> Path:
    """Write an executable stand-in for ccx."""
    path = directory / "ccx"
    path.write_text("#!/bin/sh\n" + body)
    path.chmod(path.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
    return path


class TestFindCcx(unittest.TestCase):
    def test_missing_binary_returns_none_rather_than_raising(self):
        self.assertIsNone(find_ccx(Path("/nonexistent/ccx")))

    def test_explicit_path_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = stub_solver(Path(tmp), "exit 0\n")
            self.assertEqual(find_ccx(path), path)

    def test_env_var_is_honoured(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = stub_solver(Path(tmp), "exit 0\n")
            original = os.environ.get("WHEELOPT_CCX")
            os.environ["WHEELOPT_CCX"] = str(path)
            try:
                self.assertEqual(find_ccx(), path)
            finally:
                if original is None:
                    del os.environ["WHEELOPT_CCX"]
                else:
                    os.environ["WHEELOPT_CCX"] = original

    def test_a_non_executable_file_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ccx"
            path.write_text("not executable")
            self.assertIsNone(find_ccx(path))

    def test_a_bad_explicit_path_does_not_fall_through_to_path(self):
        """An explicit path is final. If it names a solver that is not usable, the answer
        is None — never "some other ccx on PATH". Falling through would run a different
        solver than the caller pinned, and file the results under the wrong cache key,
        since the solver identity is part of the key."""
        # Even with a working stub installed on PATH, a bad explicit path yields None.
        with tempfile.TemporaryDirectory() as tmp:
            good = stub_solver(Path(tmp), "exit 0\n")
            original = os.environ.get("PATH", "")
            os.environ["PATH"] = str(good.parent) + os.pathsep + original
            try:
                self.assertIsNone(find_ccx(Path("/nonexistent/ccx")))
            finally:
                os.environ["PATH"] = original

    def test_identity_without_a_binary_is_the_sentinel(self):
        self.assertEqual(solver_identity(None), "solver-unknown")


class TestFailurePaths(unittest.TestCase):
    """Each of these would be an exception in a naive implementation."""

    def run_with(self, body: str, **kwargs) -> FeaResult:
        with tempfile.TemporaryDirectory() as tmp:
            ccx = stub_solver(Path(tmp), body)
            return run_load_case(
                PARAMS, TPU, LoadCase(),
                cache_root=Path(tmp) / "cache",
                ccx_path=ccx,
                use_cache=False,
                step_path=Path(tmp) / "missing.step",
                **kwargs,
            )

    def test_missing_solver_is_typed(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = run_load_case(
                PARAMS, TPU, LoadCase(),
                cache_root=Path(tmp),
                ccx_path=Path("/nonexistent/ccx"),
                use_cache=False,
            )
        self.assertIs(result.status, FeaStatus.SOLVER_MISSING)
        self.assertFalse(result.ok)
        self.assertTrue(result.is_environment_failure)

    def test_missing_step_file_is_typed_not_raised(self):
        result = self.run_with("exit 0\n")
        self.assertIn(
            result.status, {FeaStatus.MESH_FAILED, FeaStatus.CAD_FAILED}
        )
        self.assertFalse(result.ok)

    def test_unknown_material_is_a_deck_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = run_load_case(
                PARAMS, MaterialSpec(name="PLA"), LoadCase(),
                cache_root=Path(tmp), use_cache=False,
            )
        self.assertIs(result.status, FeaStatus.DECK_INVALID)
        self.assertIn("PLA", result.message)

    def test_every_status_is_constructible(self):
        for status in FeaStatus:
            with self.subTest(status=status.value):
                r = failure(status, LoadCase(), "key", "message")
                self.assertIs(r.status, status)
                self.assertEqual(r.ok, status is FeaStatus.OK)


class TestResultShape(unittest.TestCase):
    def test_failed_results_expose_no_physics(self):
        r = failure(FeaStatus.SOLVER_DIVERGED, LoadCase(), "k", "diverged")
        self.assertIsNone(r.curve)
        self.assertIsNone(r.patch)
        self.assertIsNone(r.peak_von_mises_pa)
        self.assertEqual(r.violations, [])

    def test_hysteresis_is_never_reported_in_this_version(self):
        """A hyperelastic model cannot produce a loss factor; reporting one would be
        fabricating a result. See FeaResult and docs/plan/07-materials.md."""
        r = failure(FeaStatus.OK, LoadCase(), "k", "")
        self.assertIsNone(r.hysteresis_loss_factor)

    def test_environment_failure_is_distinguished_from_a_hard_design(self):
        """Only design failures belong in a campaign's failure-rate statistic."""
        env = failure(FeaStatus.SOLVER_MISSING, LoadCase(), "k", "")
        design = failure(FeaStatus.SOLVER_DIVERGED, LoadCase(), "k", "")
        self.assertTrue(env.is_environment_failure)
        self.assertFalse(design.is_environment_failure)

    def test_diagnostics_default_to_zero_not_none(self):
        d = SolverDiagnostics()
        self.assertEqual(d.n_increments, 0)
        self.assertEqual(d.completed_fraction, 0.0)

    def test_summarise_counts_statuses(self):
        results = [
            failure(FeaStatus.OK, LoadCase(), "a", ""),
            failure(FeaStatus.SOLVER_DIVERGED, LoadCase(), "b", ""),
            failure(FeaStatus.SOLVER_DIVERGED, LoadCase(), "c", ""),
        ]
        self.assertEqual(summarise(results), {"ok": 1, "solver_diverged": 2})


class TestLoadCaseValidation(unittest.TestCase):
    def test_rejects_non_positive_displacement(self):
        with self.assertRaises(ValueError):
            LoadCase(delta_max_m=0.0)

    def test_rejects_too_few_sample_points(self):
        with self.assertRaises(ValueError):
            LoadCase(n_points_per_branch=1)

    def test_rejects_negative_friction(self):
        with self.assertRaises(ValueError):
            LoadCase(friction_mu=-0.1)

    def test_step_period_covers_load_and_unload(self):
        self.assertEqual(LoadCase().step_period, 2.0)

    def test_target_load_is_the_multiple_of_nominal(self):
        case = LoadCase(nominal_load_n=10.0, max_load_multiple=3.0)
        self.assertAlmostEqual(case.target_load_n, 30.0)

    def test_both_kinds_are_available(self):
        self.assertEqual(
            {k.value for k in LoadCaseKind}, {"radial_flat", "radial_step_edge"}
        )


if __name__ == "__main__":
    unittest.main()
