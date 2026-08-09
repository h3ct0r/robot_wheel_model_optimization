"""Report plots.

The suite must pass without matplotlib installed, so the plotting assertions are skipped
when it is absent — but the import-hygiene tests below always run, because the point of
``wheelopt.viz`` is that importing it costs nothing.
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from wheelopt.cad.materials import MaterialSpec
from wheelopt.cad.params import WheelParams
from wheelopt.fea.loadcase import LoadCase, LoadCaseKind
from wheelopt.fea.results import ContactPatch, FeaResult, FeaStatus, LoadCurve, failure

try:
    import matplotlib  # noqa: F401

    HAVE_MPL = True
except ImportError:  # pragma: no cover - depends on environment
    HAVE_MPL = False

needs_mpl = unittest.skipUnless(HAVE_MPL, "matplotlib not installed")

PARAMS = WheelParams(outer_radius_mm=40.0, width_mm=20.0, n_spokes=6,
                     spoke_thickness_mm=3.0, hub_radius_mm=14.0)
MATERIAL = MaterialSpec(name="TPU_95A", infill_density=0.4)


def synthetic_result(kind: LoadCaseKind = LoadCaseKind.RADIAL_FLAT) -> FeaResult:
    delta = np.linspace(0.0005, 0.004, 6)
    force = 900.0 * delta + 4.0e7 * delta**3
    curve = LoadCurve(
        delta_m=np.concatenate([delta, delta[::-1]]),
        force_n=np.concatenate([force, force[::-1]]),
        loading=np.concatenate([np.ones(6, bool), np.zeros(6, bool)]),
    )
    patch = ContactPatch(
        force_n=force,
        length_m=np.linspace(0.002, 0.023, 6),
        width_m=np.full(6, 0.0136),
        area_m2=np.linspace(2e-5, 1.7e-4, 6),
        peak_pressure_pa=np.linspace(5e4, 1.2e6, 6),
        n_nodes=np.arange(2, 14, 2),
    )
    return FeaResult(
        status=FeaStatus.OK,
        load_case=LoadCase(kind=kind),
        cache_key="cafe1234",
        curve=curve,
        patch=patch,
        peak_von_mises_pa=6.1e5,
        p95_von_mises_pa=1.6e5,
        loaded_radius_m=PARAMS.outer_radius_mm * 1e-3 - curve.delta_m,
        buckling_detected=False,
        loop_area_fraction=0.013,
    )


class TestImportHygiene(unittest.TestCase):
    """Importing the plotting module must not drag in heavy dependencies."""

    def test_importing_viz_does_not_import_matplotlib_or_occt(self):
        import subprocess
        import sys

        code = (
            "import sys; sys.path.insert(0, 'src');"
            "import wheelopt.viz;"
            "bad = [m for m in ('matplotlib', 'build123d', 'OCP', 'gmsh') "
            "       if m in sys.modules];"
            "print(','.join(bad))"
        )
        out = subprocess.run(
            [sys.executable, "-c", code],
            capture_output=True, text=True,
            cwd=str(Path(__file__).resolve().parents[1]),
        )
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(out.stdout.strip(), "", "viz pulled in a heavy dependency at import")

    def test_case_colours_cover_every_load_case(self):
        from wheelopt.viz import CASE_COLOURS

        for kind in LoadCaseKind:
            self.assertIn(kind.value, CASE_COLOURS)


@needs_mpl
class TestDesignPdf(unittest.TestCase):
    def test_writes_a_real_pdf(self):
        from wheelopt.viz import write_design_pdf

        with tempfile.TemporaryDirectory() as tmp:
            out = write_design_pdf(Path(tmp) / "d.pdf", PARAMS, MATERIAL)
            data = out.read_bytes()
            self.assertTrue(data.startswith(b"%PDF"))
            self.assertGreater(len(data), 5000)

    def test_needs_no_cad_kernel(self):
        """The section is drawn from the centreline, so a design can be plotted on a
        machine that cannot build a solid — that is what makes this a screening aid."""
        import subprocess
        import sys

        with tempfile.TemporaryDirectory() as tmp:
            code = (
                "import sys; sys.path.insert(0, 'src');"
                "from wheelopt.viz import write_design_pdf;"
                "from wheelopt.cad.params import WheelParams;"
                f"write_design_pdf(r'{tmp}/d.pdf', WheelParams());"
                "import sys as s;"
                "print('OCP' in s.modules or 'build123d' in s.modules)"
            )
            out = subprocess.run(
                [sys.executable, "-c", code], capture_output=True, text=True,
                cwd=str(Path(__file__).resolve().parents[1]),
            )
            self.assertEqual(out.returncode, 0, out.stderr)
            self.assertIn("False", out.stdout)

    def test_creates_missing_parent_directories(self):
        from wheelopt.viz import write_design_pdf

        with tempfile.TemporaryDirectory() as tmp:
            out = write_design_pdf(Path(tmp) / "deep" / "nested" / "d.pdf", PARAMS)
            self.assertTrue(out.exists())


@needs_mpl
class TestReportPdf(unittest.TestCase):
    def test_writes_a_multi_page_report(self):
        from wheelopt.viz import write_report_pdf

        results = [synthetic_result(k) for k in LoadCaseKind]
        with tempfile.TemporaryDirectory() as tmp:
            out = write_report_pdf(Path(tmp) / "r.pdf", PARAMS, MATERIAL, results)
            data = out.read_bytes()
            self.assertTrue(data.startswith(b"%PDF"))
            # design page + metrics page + one page per case
            n_pages = data.count(b"/Type /Page") - data.count(b"/Type /Pages")
            self.assertGreaterEqual(n_pages, 4)

    def test_failed_results_still_get_a_page(self):
        """A report that silently contains fewer cases than were asked for is worse than
        one that says why a case diverged."""
        from wheelopt.viz import write_report_pdf

        results = [
            synthetic_result(),
            failure(FeaStatus.SOLVER_DIVERGED, LoadCase(kind=LoadCaseKind.RADIAL_STEP_EDGE),
                    "key", "stopped at t=0.42 after 9 cutbacks"),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            out = write_report_pdf(Path(tmp) / "r.pdf", PARAMS, MATERIAL, results)
            self.assertTrue(out.read_bytes().startswith(b"%PDF"))

    def test_report_with_no_solved_cases_still_writes(self):
        from wheelopt.viz import write_report_pdf

        results = [failure(FeaStatus.SOLVER_MISSING, LoadCase(), "k", "no ccx")]
        with tempfile.TemporaryDirectory() as tmp:
            out = write_report_pdf(Path(tmp) / "r.pdf", PARAMS, MATERIAL, results)
            self.assertTrue(out.exists())

    def test_result_without_contact_data_does_not_crash(self):
        from wheelopt.viz import write_report_pdf

        base = synthetic_result()
        no_patch = FeaResult(
            status=base.status, load_case=base.load_case, cache_key=base.cache_key,
            curve=base.curve, patch=None, loaded_radius_m=base.loaded_radius_m,
        )
        with tempfile.TemporaryDirectory() as tmp:
            out = write_report_pdf(Path(tmp) / "r.pdf", PARAMS, MATERIAL, [no_patch])
            self.assertTrue(out.exists())


@needs_mpl
class TestSectionDrawing(unittest.TestCase):
    def test_draws_one_polygon_per_spoke(self):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from wheelopt.viz import draw_wheel_section

        for n in (6, 12, 20):
            with self.subTest(spokes=n):
                fig, ax = plt.subplots()
                draw_wheel_section(ax, WheelParams(n_spokes=n), annotate=False)
                spokes = [p for p in ax.patches
                          if (p.get_gid() or "").startswith("spoke-")]
                self.assertEqual(len(spokes), n)
                plt.close(fig)

    def test_section_spans_the_full_diameter(self):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        from wheelopt.viz import draw_wheel_section

        fig, ax = plt.subplots()
        draw_wheel_section(ax, PARAMS, annotate=False)
        lo, hi = ax.get_xlim()
        self.assertLessEqual(lo, -PARAMS.outer_radius_mm)
        self.assertGreaterEqual(hi, PARAMS.outer_radius_mm)
        plt.close(fig)


if __name__ == "__main__":
    unittest.main()
