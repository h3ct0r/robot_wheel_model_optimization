"""Vector PDF plots of the CAD design and the FEA metrics.

matplotlib is an optional dependency (``pip install -e '.[viz]'``) and is imported lazily,
so it never sits on the import path of a screening worker or a solver batch.

**The design page needs no CAD kernel.** It is drawn from
:mod:`wheelopt.cad.centreline`, which is pure numpy, so a design can be plotted without
OCCT installed and without building a solid — which makes ``--plot-pdf`` usable as a
screening aid, not only as post-processing. The section it draws is the true
spoke outline, including the attachment overlap the solid is built with, so the picture and
the geometry cannot drift apart.

PDF rather than PNG because these end up in the report: text stays selectable, curves stay
vector, and a reviewer can zoom into the spoke root without it turning to mush.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from .cad.centreline import attachment_overlap_mm, spoke_outline
from .cad.materials import MaterialSpec
from .cad.params import WheelParams
from .fea.loadcase import LoadCaseKind

if TYPE_CHECKING:  # pragma: no cover
    from .fea.results import FeaResult

__all__ = [
    "CASE_COLOURS",
    "TREAD_GROOVES",
    "MissingPlotting",
    "draw_wheel_profile",
    "draw_wheel_section",
    "write_design_pdf",
    "write_report_pdf",
]


class MissingPlotting(ImportError):
    """matplotlib is unavailable. An environment problem, like ``MissingCadKernel``."""


#: One colour per load case, used consistently across every figure so a reader can follow a
#: case between plots without re-reading the legend.
CASE_COLOURS: dict[str, str] = {
    LoadCaseKind.RADIAL_FLAT.value: "#0f7d86",
    LoadCaseKind.RADIAL_STEP_EDGE.value: "#c4661a",
    # The contact-free tip cases. Paler than the contact pair on purpose: they measure one
    # claw against a prescribed displacement, not a wheel against the ground, and a plot
    # mixing the two should not invite reading them off the same axis.
    LoadCaseKind.TIP_RADIAL.value: "#5a8fa8",
    LoadCaseKind.TIP_TANGENTIAL.value: "#a87f5a",
}
_INK = "#1a2226"
_MUTED = "#5d6b72"
_GRID = "#dde3e6"
_FILL = "#0f7d86"


def _pyplot() -> Any:
    """Import matplotlib in headless mode, or raise a message that says what to install."""
    try:
        import matplotlib
    except ImportError as exc:  # pragma: no cover - environment
        raise MissingPlotting(
            "matplotlib is not installed; run `pip install -e '.[viz]'` to enable --plot-pdf"
        ) from exc
    # Agg before pyplot: these run unattended on machines with no display.
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.size": 8.5,
            "axes.titlesize": 9.5,
            "axes.labelsize": 8.5,
            "axes.edgecolor": _MUTED,
            "axes.labelcolor": _INK,
            "axes.titleweight": "bold",
            "text.color": _INK,
            "xtick.color": _MUTED,
            "ytick.color": _MUTED,
            "xtick.labelsize": 7.5,
            "ytick.labelsize": 7.5,
            "grid.color": _GRID,
            "grid.linewidth": 0.6,
            "legend.frameon": False,
            "legend.fontsize": 8,
            "figure.dpi": 150,
            "savefig.bbox": "tight",
            "pdf.fonttype": 42,  # embed TrueType so text stays selectable, not outlined
        }
    )
    return plt


def _style(ax: Any) -> None:
    ax.grid(True, linewidth=0.6, alpha=0.9)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def _colour(result: FeaResult) -> str:
    return CASE_COLOURS.get(result.load_case.kind.value, _FILL)


def _label(result: FeaResult) -> str:
    return result.load_case.kind.value.replace("_", " ")


# --------------------------------------------------------------------------------------
# design


def draw_wheel_section(
    ax: Any,
    params: WheelParams,
    material: MaterialSpec | None = None,
    *,
    annotate: bool = True,
) -> None:
    """Draw the T3 mid-plane cross-section: shear band, spokes, hub, bore.

    Geometry comes from the same ``spoke_outline`` the solid is extruded from, with the
    same attachment overlap, so this is the design rather than an impression of it.
    """
    from matplotlib.patches import Circle, Polygon

    theta = np.linspace(0.0, 2.0 * np.pi, 721)
    outer_r = params.outer_radius_mm
    rim_inner_r = params.rim_inner_radius_mm

    # Shear band as a filled annulus: outer disc minus the inner one. With no band, draw
    # the running surface as a dashed circle instead — the spoke tips sit on it but there
    # is no material there, and a solid ring would show a wheel that was not built.
    if params.has_shear_band:
        ax.fill(
            outer_r * np.cos(theta), outer_r * np.sin(theta),
            facecolor=_FILL, alpha=0.14, edgecolor=_FILL, linewidth=1.0, zorder=1,
        )
        ax.fill(
            rim_inner_r * np.cos(theta), rim_inner_r * np.sin(theta),
            facecolor="white", edgecolor=_GRID, linewidth=0.7, zorder=2,
        )
    else:
        ax.plot(
            outer_r * np.cos(theta), outer_r * np.sin(theta),
            color=_MUTED, linewidth=0.7, linestyle=(0, (4, 4)), zorder=1,
        )

    overlap = attachment_overlap_mm(params)
    for index in range(params.n_spokes):
        outline = spoke_outline(params, index, overlap_mm=overlap)
        ax.add_patch(
            Polygon(
                outline, closed=True, facecolor=_FILL, alpha=0.22,
                edgecolor=_FILL, linewidth=0.9, zorder=3,
                # Tagged so the spokes can be told apart from the shear-band fills, which
                # are Polygons too.
                gid=f"spoke-{index}",
            )
        )

    ax.add_patch(
        Circle((0, 0), params.hub_radius_mm, facecolor="white",
               edgecolor=_FILL, linewidth=1.1, zorder=4)
    )
    if params.hub_bore_radius_mm > 0:
        ax.add_patch(
            Circle((0, 0), params.hub_bore_radius_mm, facecolor="#eef1f3",
                   edgecolor=_MUTED, linewidth=0.9, zorder=5)
        )

    if annotate:
        # Put the radius dimension in an inter-spoke gap. Spoke i is attached at
        # i * (2*pi/n), so half a pitch lands the line squarely between two of them and it
        # never crosses a spoke or the bore.
        phi = np.pi / max(params.n_spokes, 1)
        direction = np.array([np.cos(phi), np.sin(phi)])
        ax.annotate(
            "", xy=tuple(direction * outer_r), xytext=tuple(direction * params.hub_radius_mm),
            arrowprops={"arrowstyle": "<->", "color": _MUTED, "linewidth": 0.8,
                        "shrinkA": 0, "shrinkB": 0},
            zorder=6,
        )
        label_at = direction * (params.hub_radius_mm + outer_r) * 0.5
        offset = np.array([-np.sin(phi), np.cos(phi)]) * outer_r * 0.09
        ax.text(*(label_at + offset), f"R {outer_r:g}", color=_MUTED, fontsize=7.5,
                ha="center", va="center", zorder=6)
        ax.text(0, -outer_r * 1.13, f"{params.n_spokes} × {params.spoke_profile.value} "
                f"spokes, t {params.spoke_thickness_mm:g} mm",
                color=_MUTED, fontsize=7.5, ha="center")

    limit = outer_r * 1.2
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal")
    ax.axis("off")


#: Circumferential tread grooves cut by ``cad.compliant_spoke._cut_tread``. Mirrored here
#: rather than imported because that module needs OCCT and this one must draw without it —
#: which makes the count a number in two places, so ``tests/test_viz.py`` asserts they agree.
TREAD_GROOVES = 3


def draw_wheel_profile(
    ax: Any,
    params: WheelParams,
    *,
    annotate: bool = True,
) -> None:
    """Draw the **axial** section — the r-z plane, cut through a spoke.

    The companion to :func:`draw_wheel_section`, and it exists because two real parameters are
    invisible in the mid-plane view: ``width_mm`` is the direction that view projects away,
    and ``tread_depth_mm`` cuts grooves whose axis is that direction. A figure sweeping either
    of them against the mid-plane section shows a column of identical pictures, which reads as
    "this parameter does nothing" — the exact shape of mistake this project's watch list is
    about.

    Radius runs up the page and the full diameter is drawn, mirrored about the axle, so the
    section reads as a wheel rather than as a quadrant. The tread grooves are the same three
    that ``_cut_tread`` cuts, at the same widths and offsets, and they are cut **whether or
    not there is a band** — which is what the solid does, `_cut_tread` being gated on
    ``tread_depth_mm`` alone. Drawing them only on a banded wheel was the first version and
    it was wrong: a bandless design with tread would have shown grooves the part has and the
    picture did not.

    **What this view cannot show, in exchange.** A spoke's *thickness* is in-plane, so it is
    invisible here; and cut through a spoke, a banded wheel and a bandless one are the same
    solid block from hub to outer radius. Banded against bandless is a mid-plane question.
    """
    from matplotlib.patches import Polygon, Rectangle

    half_w = 0.5 * params.width_mm
    outer_r = params.outer_radius_mm
    inner_r = params.rim_inner_radius_mm

    def band(r_lo: float, r_hi: float, z_lo: float, z_hi: float, **kwargs) -> None:
        """One rectangle and its mirror image below the axle."""
        for sign in (+1, -1):
            lo, hi = sorted((sign * r_lo, sign * r_hi))
            ax.add_patch(Rectangle((z_lo, lo), z_hi - z_lo, hi - lo, **kwargs))

    solid = {"facecolor": _FILL, "alpha": 0.22, "edgecolor": _FILL,
             "linewidth": 0.9, "zorder": 3}
    # The spokes run the full width — `extrude(amount=half, both=True)` — so in this view a
    # spoke is a full-width block, and so is the band above it.
    band(params.hub_bore_radius_mm, params.hub_radius_mm, -half_w, half_w, **solid)

    # One block from the hub to the running surface, with the grooves taken out of its top —
    # rather than a spoke block plus a separate banded ring. The grooves must cut whatever is
    # at the surface, and bandless that is the spoke tip, not a band. Splitting at `inner_r`
    # first made a bandless groove a zero-height rectangle plus a *positive*-area one below
    # it, so tread ADDED material: the drawing disagreed with the solid in the direction that
    # looks fine.
    groove_w = params.width_mm / (2 * TREAD_GROOVES + 1)
    cuts = ([(-half_w + groove_w * (2 * i + 1), -half_w + groove_w * (2 * i + 2))
             for i in range(TREAD_GROOVES)] if params.tread_depth_mm > 0 else [])
    # One polygon walking the grooved surface, not a row of rectangles per land. Stacked
    # rectangles each draw their own edge, so the seams between lands appear as vertical
    # hairlines through a part that is one solid — structure the wheel does not have.
    floor = outer_r - params.tread_depth_mm
    top = [(-half_w, outer_r)]
    for start, end in cuts:
        top += [(start, outer_r), (start, floor), (end, floor), (end, outer_r)]
    top.append((half_w, outer_r))
    outline = [(-half_w, params.hub_radius_mm), *top, (half_w, params.hub_radius_mm)]
    for sign in (+1, -1):
        ax.add_patch(Polygon([(z, sign * r) for z, r in outline], closed=True, **solid))

    if params.has_shear_band:
        # Where the band starts. A hairline rather than an edge: it is one solid, and the
        # boundary is a fact about how it was authored, not a face.
        for sign in (+1, -1):
            ax.plot([-half_w, half_w], [sign * inner_r] * 2, color=_FILL,
                    linewidth=0.7, alpha=0.55, zorder=4)
    else:
        # `inner_r == outer_r` here, so the loop above drew nothing and the block already ends
        # at the tips. Mark the running surface the way the mid-plane view does — a dashed
        # circle there, a dashed line here — because it is where the wheel touches the ground
        # and not where its material ends.
        for sign in (+1, -1):
            ax.plot([-half_w, half_w], [sign * outer_r] * 2, color=_MUTED,
                    linewidth=0.9, linestyle=(0, (4, 4)), zorder=4)

    # The axle, drawn as a segment rather than an `axhline`: a line spanning the whole axes
    # runs far outside the wheel and, in a row of panels, reads as one rule through all of them.
    axle = half_w * 1.35
    ax.plot([-axle, axle], [0.0, 0.0], color=_MUTED, linewidth=0.7,
            linestyle=(0, (6, 3)), zorder=2)

    if annotate:
        ax.annotate("", xy=(half_w, -outer_r * 1.08), xytext=(-half_w, -outer_r * 1.08),
                    arrowprops={"arrowstyle": "<->", "color": _MUTED, "linewidth": 0.8,
                                "shrinkA": 0, "shrinkB": 0}, zorder=6)
        ax.text(0, -outer_r * 1.22, f"w {params.width_mm:g} mm", color=_MUTED,
                fontsize=7.5, ha="center", va="center")
        if params.tread_depth_mm > 0:
            ax.text(0, outer_r * 1.10, f"tread {params.tread_depth_mm:g} mm deep, "
                    f"{TREAD_GROOVES} grooves", color=_MUTED, fontsize=7.5, ha="center")

    ax.set_xlim(-outer_r * 1.25, outer_r * 1.25)   # equal aspect, squared on the radius
    ax.set_ylim(-outer_r * 1.32, outer_r * 1.25)
    ax.set_aspect("equal")
    ax.axis("off")


def _design_summary(params: WheelParams, material: MaterialSpec | None) -> list[tuple[str, str]]:
    rows = [
        ("design hash", params.design_hash()),
        ("outer radius", f"{params.outer_radius_mm:g} mm"),
        ("width", f"{params.width_mm:g} mm"),
        ("rim thickness", f"{params.rim_thickness_mm:g} mm" if params.has_shear_band
         else "none — spoke tips run on the ground"),
        ("hub radius / bore", f"{params.hub_radius_mm:g} / {params.hub_bore_radius_mm:g} mm"),
        ("spokes", f"{params.n_spokes} × {params.spoke_profile.value}"),
        ("spoke thickness", f"{params.spoke_thickness_mm:g} mm"),
        ("spoke curvature", f"{params.spoke_curvature_1_per_mm:g} /mm"),
        ("spoke span / sagitta", f"{params.spoke_span_mm:.1f} / {params.spoke_sagitta_mm:.2f} mm"),
    ]
    if not params.has_shear_band:
        # Inert with a band, decisive without one — so it is only worth a row when it
        # actually changes the answer.
        rows.append(("spoke phase", f"{params.spoke_phase_deg:g}°"))
    if params.tread_depth_mm > 0:
        rows.append(("tread depth", f"{params.tread_depth_mm:g} mm"))
    if material is not None:
        rows += [
            ("material", material.name),
            ("infill", f"{material.infill_density:.0%} {material.infill_pattern.value}"),
            ("walls", f"{material.wall_count} × {material.nozzle_diameter_mm:g} mm"),
            (
                "effective density",
                f"{material.effective_density_kg_m3(params.spoke_thickness_mm):.0f} kg/m³",
            ),
        ]
    return rows


def _draw_table(
    ax: Any, rows: Sequence[tuple[str, str]], title: str, *, wrap_over: int = 30
) -> None:
    """Two-column key/value block.

    Values longer than ``wrap_over`` characters drop onto their own line: right-aligning a
    long string on the same row as its label makes the two overlap, which is exactly what
    the solver-diagnostics row does otherwise.
    """
    ax.axis("off")
    ax.set_title(title, loc="left", pad=8)

    lines: list[tuple[str | None, str | None]] = []
    for key, value in rows:
        if len(str(value)) > wrap_over:
            lines.append((key, None))
            lines.append((None, str(value)))
        else:
            lines.append((key, value))

    y = 1.0
    step = 1.0 / max(len(lines) + 1, 10)
    for key, value in lines:
        if key is not None:
            ax.text(0.0, y, key, color=_MUTED, fontsize=8, va="top")
        if value is not None:
            if key is None:
                ax.text(0.03, y, value, color=_INK, fontsize=7, va="top",
                        family="monospace")
            else:
                ax.text(1.0, y, value, color=_INK, fontsize=8, va="top", ha="right",
                        family="monospace")
        y -= step


# --------------------------------------------------------------------------------------
# metrics


def _plot_load_curve(ax: Any, results: Sequence[FeaResult]) -> None:
    for result in results:
        curve = result.curve
        if curve is None:
            continue
        colour = _colour(result)
        load = curve.loading
        # Prepend the origin: the sweep is sampled from the first output time, not from
        # zero, and a line that starts mid-air reads as a missing measurement.
        ax.plot(
            np.concatenate([[0.0], curve.delta_m[load] * 1e3]),
            np.concatenate([[0.0], curve.force_n[load]]),
            "-o", color=colour, linewidth=1.6, markersize=3.2,
            markerfacecolor="white", markeredgewidth=1.1, label=f"{_label(result)} · load",
        )
        if (~load).any():
            ax.plot(
                np.concatenate([curve.delta_m[~load] * 1e3, [0.0]]),
                np.concatenate([curve.force_n[~load], [0.0]]),
                "--", color=colour, linewidth=1.1, alpha=0.85,
                label=f"{_label(result)} · unload",
            )
    ax.set_xlabel("indentation δ  (mm)")
    ax.set_ylabel("reaction force  (N)")
    ax.set_title("Force – deflection", loc="left")
    ax.legend(loc="upper left")
    _style(ax)


def _plot_stiffness(ax: Any, results: Sequence[FeaResult]) -> None:
    for result in results:
        curve = result.curve
        if curve is None:
            continue
        delta = curve.delta_m[curve.loading] * 1e3
        ax.plot(delta, curve.tangent_stiffness_n_per_m() / 1e3, "-",
                color=_colour(result), linewidth=1.6, label=f"{_label(result)} · tangent")
        ax.plot(delta, curve.secant_stiffness_n_per_m() / 1e3, ":",
                color=_colour(result), linewidth=1.2, alpha=0.9,
                label=f"{_label(result)} · secant")
    ax.set_xlabel("indentation δ  (mm)")
    ax.set_ylabel("radial stiffness $k_r$  (kN/m)")
    ax.set_title("Stiffness — rising means stiffening", loc="left")
    ax.legend(loc="best")
    _style(ax)


def _plot_contact(ax: Any, results: Sequence[FeaResult]) -> None:
    plotted = False
    for result in results:
        patch = result.patch
        if patch is None or len(patch.force_n) == 0:
            continue
        ax.plot(patch.force_n, patch.peak_pressure_pa / 1e3, "-o",
                color=_colour(result), linewidth=1.5, markersize=3.2,
                markerfacecolor="white", markeredgewidth=1.0, label=_label(result))
        plotted = True
    ax.set_xlabel("reaction force  (N)")
    ax.set_ylabel("peak contact pressure  (kPa)")
    ax.set_title("Contact pressure vs load", loc="left")
    if plotted:
        ax.legend(loc="best")
    else:
        ax.text(0.5, 0.5, "no contact output", transform=ax.transAxes,
                ha="center", va="center", color=_MUTED, fontsize=8)
    _style(ax)


def _plot_loaded_radius(ax: Any, results: Sequence[FeaResult]) -> None:
    for result in results:
        curve, radius = result.curve, result.loaded_radius_m
        if curve is None or radius is None:
            continue
        load = curve.loading
        ax.plot(curve.force_n[load], radius[load] * 1e3, "-o",
                color=_colour(result), linewidth=1.5, markersize=3.2,
                markerfacecolor="white", markeredgewidth=1.0, label=_label(result))
    ax.set_xlabel("reaction force  (N)")
    ax.set_ylabel("loaded rolling radius  (mm)")
    ax.set_title("Loaded radius — must fall with load", loc="left")
    ax.legend(loc="best")
    _style(ax)


def _result_summary(result: FeaResult) -> list[tuple[str, str]]:
    curve = result.curve
    rows: list[tuple[str, str]] = [("status", result.status.value)]
    if curve is not None:
        k = curve.tangent_stiffness_n_per_m()
        rows += [
            ("peak force", f"{curve.peak_force_n:.2f} N"),
            ("at indentation", f"{curve.peak_delta_m * 1e3:.2f} mm"),
            ("k_r first → last", f"{k[1] / 1e3:.2f} → {k[-1] / 1e3:.2f} kN/m"),
        ]
    if result.p95_von_mises_pa is not None:
        rows.append(("spoke stress p95", f"{result.p95_von_mises_pa / 1e6:.2f} MPa"))
    if result.peak_von_mises_pa is not None:
        rows.append(("peak (mesh-dependent)", f"{result.peak_von_mises_pa / 1e6:.2f} MPa"))
    if result.patch is not None and len(result.patch.force_n):
        rows.append(
            ("patch at peak",
             f"{result.patch.length_m[-1] * 1e3:.1f} × {result.patch.width_m[-1] * 1e3:.1f} mm")
        )
        rows.append(
            ("peak pressure", f"{result.patch.peak_pressure_pa[-1] / 1e3:.0f} kPa")
        )
    rows.append(("buckling", "detected" if result.buckling_detected else "none"))
    if result.loop_area_fraction is not None:
        rows.append(("loop area (QC)", f"{result.loop_area_fraction:.2%}"))
    # Stated rather than left blank: a reader should not have to wonder whether the number
    # is missing or unobtainable. See FeaResult.hysteresis_loss_factor.
    rows.append(("hysteresis loss", "n/a — hyperelastic"))
    rows.append(("solver", result.diagnostics.summary()))
    return rows


# --------------------------------------------------------------------------------------
# documents


def _title_block(fig: Any, title: str, subtitle: str) -> None:
    fig.text(0.02, 0.975, title, fontsize=13, fontweight="bold", color=_INK, va="top")
    fig.text(0.02, 0.945, subtitle, fontsize=8, color=_MUTED, va="top")


def write_design_pdf(
    path: Path | str,
    params: WheelParams,
    material: MaterialSpec | None = None,
    *,
    violations: Sequence[Any] = (),
) -> Path:
    """One page: the wheel section and its parameters. No solver, no CAD kernel needed."""
    plt = _pyplot()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(8.27, 5.4))
    _title_block(fig, "Compliant wheel — design",
                 f"family T3 · design {params.design_hash()}")
    grid = fig.add_gridspec(1, 2, width_ratios=(1.15, 1.0), left=0.02, right=0.97,
                            top=0.88, bottom=0.05, wspace=0.12)
    draw_wheel_section(fig.add_subplot(grid[0, 0]), params, material)

    ax = fig.add_subplot(grid[0, 1])
    rows = _design_summary(params, material)
    if violations:
        rows.append(("screening", f"{len(violations)} note(s)"))
        for violation in violations:
            rows.append((f"  {violation.severity.value}", violation.name))
    _draw_table(ax, rows, "Parameters")

    fig.savefig(path, format="pdf")
    plt.close(fig)
    return path


def write_report_pdf(
    path: Path | str,
    params: WheelParams,
    material: MaterialSpec,
    results: Sequence[FeaResult],
) -> Path:
    """Multi-page report: the design, then the metrics, then a per-case summary.

    Failed results are included rather than dropped — a page saying *why* a case diverged is
    more useful than a report that quietly contains one fewer case than was asked for.
    """
    plt = _pyplot()
    from matplotlib.backends.backend_pdf import PdfPages

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    solved = [r for r in results if r.ok and r.curve is not None]

    with PdfPages(path) as pdf:
        # --- page 1: design -----------------------------------------------------------
        fig = plt.figure(figsize=(8.27, 5.4))
        cases = ", ".join(sorted({r.load_case.kind.value.replace("_", " ") for r in results}))
        _title_block(fig, "Compliant wheel — FEA report",
                     f"design {params.design_hash()} · {material.name} · cases: {cases}")
        grid = fig.add_gridspec(1, 2, width_ratios=(1.15, 1.0), left=0.02, right=0.97,
                                top=0.88, bottom=0.05, wspace=0.12)
        draw_wheel_section(fig.add_subplot(grid[0, 0]), params, material)
        _draw_table(fig.add_subplot(grid[0, 1]), _design_summary(params, material),
                    "Parameters")
        pdf.savefig(fig)
        plt.close(fig)

        # --- page 2: metrics ----------------------------------------------------------
        if solved:
            fig = plt.figure(figsize=(8.27, 6.6))
            _title_block(fig, "Radial response and contact",
                         "the curves the segmented-ring ROM is fitted to (step 4)")
            grid = fig.add_gridspec(2, 2, left=0.09, right=0.97, top=0.88, bottom=0.08,
                                    hspace=0.38, wspace=0.28)
            _plot_load_curve(fig.add_subplot(grid[0, 0]), solved)
            _plot_stiffness(fig.add_subplot(grid[0, 1]), solved)
            _plot_contact(fig.add_subplot(grid[1, 0]), solved)
            _plot_loaded_radius(fig.add_subplot(grid[1, 1]), solved)
            pdf.savefig(fig)
            plt.close(fig)

        # --- page 3+: one summary per case --------------------------------------------
        for result in results:
            fig = plt.figure(figsize=(8.27, 5.4))
            _title_block(fig, f"Case — {_label(result)}",
                         f"cache key {result.cache_key or 'n/a'}")
            grid = fig.add_gridspec(1, 2, width_ratios=(1.0, 1.0), left=0.06, right=0.97,
                                    top=0.88, bottom=0.10, wspace=0.24)
            ax = fig.add_subplot(grid[0, 0])
            if result.ok and result.curve is not None:
                _plot_load_curve(ax, [result])
            else:
                ax.axis("off")
                ax.text(0.0, 0.9, result.status.value, color=CASE_COLOURS.get(
                    result.load_case.kind.value, _INK), fontsize=11, va="top",
                    fontweight="bold")
                ax.text(0.0, 0.78, result.message or "", color=_MUTED, fontsize=8,
                        va="top", wrap=True)

            rows = _result_summary(result)
            if result.violations:
                rows.append(("—", ""))
                for violation in result.violations:
                    rows.append((f"{violation.severity.value}", violation.name))
            _draw_table(fig.add_subplot(grid[0, 1]), rows, "Extracted metrics")
            pdf.savefig(fig)
            plt.close(fig)

        info = pdf.infodict()
        info["Title"] = f"wheelopt FEA report — {params.design_hash()}"
        info["Subject"] = "Radial compression of a compliant TPU wheel"
        info["Creator"] = "wheelopt"

    return path
