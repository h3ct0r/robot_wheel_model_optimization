"""Report plots.

The suite must pass without matplotlib installed, so the plotting assertions are skipped
when it is absent — but the import-hygiene tests below always run, because the point of
``wheelopt.viz`` is that importing it costs nothing.
"""

from __future__ import annotations

import tempfile
import unittest
from dataclasses import replace
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
        check=False, )
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
            check=False, )
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


@needs_mpl
class TestProfileDrawing(unittest.TestCase):
    """The axial section. It exists because two parameters are invisible in the other view."""

    def axes(self):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        return plt, *plt.subplots()

    @staticmethod
    def corners(patch) -> np.ndarray:
        """Data-space vertices of any patch — the drawing mixes Rectangle and Polygon, and
        `get_x`/`get_width` exist only on the first."""
        return patch.get_path().transformed(patch.get_patch_transform()).vertices

    def extent(self, ax) -> tuple[float, float]:
        """Widest and tallest patch extent actually drawn, mm."""
        points = np.vstack([self.corners(p) for p in ax.patches])
        low, high = points.min(axis=0), points.max(axis=0)
        return float(high[0] - low[0]), float(high[1] - low[1])

    def area(self, ax) -> float:
        """Total drawn area by the shoelace formula, mm². Works on either patch type, and on
        a grooved outline, which is one polygon rather than a stack of boxes."""
        total = 0.0
        for patch in ax.patches:
            x, y = self.corners(patch).T
            total += 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
        return total

    def test_width_changes_the_drawing_and_the_mid_plane_view_does_not(self):
        """The whole reason this function exists. If a width sweep were drawn with
        `draw_wheel_section` every panel would be identical, which reads as "this parameter
        does nothing" — the project's recurring failure, in a figure instead of a value."""
        from wheelopt.viz import draw_wheel_profile, draw_wheel_section

        narrow = replace(PARAMS, width_mm=20.0)
        wide = replace(PARAMS, width_mm=60.0)

        plt, fig, ax = self.axes()
        draw_wheel_profile(ax, narrow, annotate=False)
        thin_w, thin_h = self.extent(ax)
        plt.close(fig)

        plt, fig, ax = self.axes()
        draw_wheel_profile(ax, wide, annotate=False)
        wide_w, wide_h = self.extent(ax)
        plt.close(fig)

        self.assertAlmostEqual(wide_w / thin_w, 3.0, places=6)
        self.assertAlmostEqual(wide_h, thin_h, places=9, msg="radius must not move")

        # ...and the mid-plane view is genuinely blind to it, which is the premise.
        spans = []
        for params in (narrow, wide):
            plt, fig, ax = self.axes()
            draw_wheel_section(ax, params, annotate=False)
            spans.append(ax.get_xlim())
            plt.close(fig)
        self.assertEqual(spans[0], spans[1])

    def test_the_tread_grooves_are_the_ones_the_solid_actually_gets(self):
        """`TREAD_GROOVES` mirrors a count inside `compliant_spoke._cut_tread`, which needs
        OCCT and so cannot be imported here. A mirrored constant drifts; this reads the number
        back out of the source rather than trusting the copy."""
        import re

        from wheelopt.viz import TREAD_GROOVES

        source = (Path(__file__).resolve().parents[1]
                  / "src" / "wheelopt" / "cad" / "compliant_spoke.py").read_text()
        found = re.search(r"n_grooves\s*=\s*(\d+)", source)
        self.assertIsNotNone(found, "the groove count moved; update TREAD_GROOVES")
        self.assertEqual(int(found.group(1)), TREAD_GROOVES)

    def test_a_deeper_groove_removes_material_rather_than_adding_a_shape(self):
        """A groove is a cut. The drawn area must fall as the depth rises — the opposite
        would be a picture of a tread that stands proud of the tyre."""
        from wheelopt.viz import draw_wheel_profile

        areas = []
        for depth in (0.0, 1.0, 3.0):
            plt, fig, ax = self.axes()
            draw_wheel_profile(ax, replace(PARAMS, tread_depth_mm=depth), annotate=False)
            areas.append(self.area(ax))
            plt.close(fig)
        self.assertGreater(areas[0], areas[1])
        self.assertGreater(areas[1], areas[2])

    def test_a_bandless_wheel_still_gets_its_tread_cut(self):
        """Caught by this test, and it was wrong in the first version. `_cut_tread` is gated on
        `tread_depth_mm` alone, not on the band, so a bandless design with tread has grooves —
        and the drawing did not. A picture missing a feature the part has is the same failure
        as a value that is quietly zero."""
        from wheelopt.viz import draw_wheel_profile

        areas = []
        for depth in (0.0, 2.0):
            plt, fig, ax = self.axes()
            draw_wheel_profile(ax, replace(PARAMS, rim_thickness_mm=0.0,
                                           tread_depth_mm=depth), annotate=False)
            areas.append(self.area(ax))
            plt.close(fig)
        self.assertGreater(areas[0], areas[1])

    def test_the_running_surface_is_marked_where_there_is_no_material_on_it(self):
        """Bandless, the block ends at the tips and the ground plane is a dashed line, exactly
        as the mid-plane view draws a dashed circle. Banded, the material reaches the surface
        and no such line is drawn."""
        from wheelopt.viz import draw_wheel_profile

        dashed = []
        for rim in (0.0, 3.0):
            plt, fig, ax = self.axes()
            draw_wheel_profile(ax, replace(PARAMS, rim_thickness_mm=rim), annotate=False)
            at_surface = [line for line in ax.lines
                          if abs(abs(line.get_ydata()[0]) - PARAMS.outer_radius_mm) < 1e-9]
            dashed.append(len(at_surface))
            plt.close(fig)
        self.assertEqual(dashed, [2, 0], "one line per side, bandless only")


if __name__ == "__main__":
    unittest.main()
