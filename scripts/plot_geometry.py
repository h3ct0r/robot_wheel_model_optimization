#!/usr/bin/env python3
"""Draw every wheel geometry parameter across its own range, as a figure per parameter.

    python scripts/plot_geometry.py                    # every sweep, PNG, into data/figures
    python scripts/plot_geometry.py --only spokes      # one of them
    python scripts/plot_geometry.py --format pdf       # vector, for the write-up
    python scripts/plot_geometry.py --contact-sheet    # all of them on one page

The point is not decoration. `PARAM_BOUNDS` and `check_design` describe the search space in
numbers, and a number does not say what a 0.25 taper *looks* like or where a design stops
being printable. Every panel is the geometry the solid is extruded from — the same
`spoke_outline` — and carries its own screening verdict, so the figures are a map of the
design space rather than an illustration of it.

**Two views, because one of them is a projection.** The mid-plane section shows everything
in-plane: spokes, band, hub, taper, phase. It cannot show `width_mm` at all, and it cannot
show `tread_depth_mm`, whose grooves run around the circumference. Sweeping either against the
mid-plane view alone gives a row of identical pictures, which reads as "this parameter does
nothing". Those two sweeps are drawn as **axial** sections instead, and the figure says which
view it is using.

Needs matplotlib (`pip install -e '.[viz]'`). It does **not** need OCCT: everything drawn here
comes from the numpy centreline layer, so the figures build on a machine with no CAD kernel.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from wheelopt.cad.constraints import Severity, check_design
from wheelopt.cad.materials import MaterialSpec
from wheelopt.cad.params import PARAM_BOUNDS, SpokeProfile, WheelParams

#: The `T3` banded design the plan calls nominal, and the base every in-plane sweep varies
#: one field of. Radius 60 rather than the 85 mm default so a 3-spoke wheel and a 36-spoke
#: wheel are legible at the same figure size.
NOMINAL = WheelParams(outer_radius_mm=60.0, width_mm=45.0, spoke_thickness_mm=6.0)

#: The `T7` claw design of the 2026-08-10 log entry: bandless, tapered, a tip at the contact
#: point. Sweeps of taper and phase are meaningless on a banded wheel, so they use this.
CLAW = replace(NOMINAL, rim_thickness_mm=0.0, n_spokes=12, claw_taper_ratio=0.6,
               spoke_phase_deg=-90.0)

TPU = MaterialSpec(name="TPU_95A", infill_density=0.4)

#: Verdict colours. Green is deliberately absent — a design that merely passes screening is
#: not endorsed, it is only not rejected, and a green tick would say more than the check does.
_VERDICT = {
    None: ("#5d6b72", "feasible"),
    Severity.WARNING: ("#c4661a", "warning"),
    Severity.INFEASIBLE: ("#b3341f", "infeasible"),
    Severity.DEGENERATE: ("#7a1f12", "degenerate"),
}
_ORDER = [Severity.DEGENERATE, Severity.INFEASIBLE, Severity.WARNING]


@dataclass(frozen=True, slots=True)
class Sweep:
    """One parameter, the values to draw it at, and how to draw them."""

    key: str
    field: str
    title: str
    values: tuple
    base: WheelParams = NOMINAL
    #: "section" is the mid-plane cut, "profile" the axial one. Named per sweep because the
    #: choice is a statement about which view can see the parameter at all.
    view: str = "section"
    note: str = ""

    def label(self, value) -> str:
        if isinstance(value, SpokeProfile):
            return value.value
        return f"{value:g}"


def _bounds(field: str) -> str:
    low, high = PARAM_BOUNDS[field]
    return f"searched {low:g} to {high:g}"


def sweeps() -> list[Sweep]:
    """Every geometry parameter, at values chosen to include both bounds and the interesting
    interior. Where a value is deliberately outside the bound it is drawn anyway, because a
    map whose edges are cropped does not show where the edges are."""
    return [
        Sweep("spokes", "n_spokes", "Spoke count",
              (1, 3, 6, 12, 24, 36),
              note=f"{_bounds('n_spokes')}. 1 and 2 are drawn to show what the floor is for: "
                   "below three, polygon_drop_m returns 2R and second_contact_delta_m returns "
                   "0 — plausible numbers about nothing"),
        Sweep("thickness", "spoke_thickness_mm", "Spoke thickness at the root",
              (0.8, 1.2, 3.0, 6.0, 8.0, 10.0),
              note=f"{_bounds('spoke_thickness_mm')}. The minimum-wall check reads the TIP, "
                   "so a tapered design is screened on a number this sweep does not vary"),
        Sweep("curvature", "spoke_curvature_1_per_mm", "Spoke curvature (signed)",
              (-0.03, -0.015, 0.0, 0.004, 0.015, 0.03),
              note=f"{_bounds('spoke_curvature_1_per_mm')}, 1/mm. The sign decides which way "
                   "the spoke bows, and so which direction of drive torque stiffens it"),
        Sweep("profile", "spoke_profile", "Spoke centreline family",
              tuple(SpokeProfile),
              note="not a searched scalar — a discrete family. Drawn at the nominal curvature"),
        Sweep("rim", "rim_thickness_mm", "Shear band thickness",
              (0.0, 1.2, 3.0, 5.0, 8.0),
              note=f"{_bounds('rim_thickness_mm')}, and exactly 0 is a TOPOLOGY SWITCH rather "
                   "than the bottom of it: no band, the spoke tips become the running "
                   "surface, and screening exempts 0 from the bound"),
        Sweep("taper", "claw_taper_ratio", "Claw taper (tip / root thickness)",
              (0.2, 0.25, 0.4, 0.6, 0.8, 1.0), base=CLAW,
              note=f"{_bounds('claw_taper_ratio')}. Drawn bandless, where it is the T7 claw "
                   "shape; 1.0 is a uniform strut. Tapering is linear in ARC LENGTH"),
        Sweep("hook", "tip_hook_mm", "L-claw foot (tangential hook at the tip)",
              (0.0, 4.0, 8.0, 16.0, 24.0, -16.0), base=CLAW,
              note=f"{_bounds('tip_hook_mm')} mm, SIGNED like the curvature: the last "
                   "panel is the same foot pointing the other way, which is a different "
                   "wheel once it is driven. 0 is the plain radial claw. The foot lies along "
                   "the running surface, so contact is an arc rather than a point and "
                   "polygon_drop_mm falls with it"),
        Sweep("phase", "spoke_phase_deg", "Spoke phase",
              (0.0, -30.0, -45.0, -90.0), base=CLAW,
              note="degrees, not searched. Only matters without a band, where contact is "
                   "discrete: -90 puts a tip at the contact point, which is what every "
                   "bandless load case must state"),
        Sweep("radius", "outer_radius_mm", "Outer radius",
              (40.0, 60.0, 85.0, 100.0, 120.0),
              note=f"{_bounds('outer_radius_mm')} mm, and also capped by the chassis wheel "
                   "well and the print bed — both from configs/robot.yaml, so the verdicts "
                   "here are about this platform"),
        Sweep("hub", "hub_radius_mm", "Hub radius",
              (10.0, 16.0, 22.0, 30.0, 40.0),
              note="mm, not searched. Sets the claw root, so claw length is radius minus "
                   "this — and a hub too small refuses the MJCF root hinge, whose pivot sits "
                   "one capsule radius inboard"),
        Sweep("bore", "hub_bore_radius_mm", "Shaft bore radius",
              (2.0, 4.0, 6.0, 8.0),
              note="mm, fixed by the drivetrain rather than searched"),
        Sweep("width", "width_mm", "Tread width",
              (20.0, 30.0, 45.0, 70.0, 90.0), view="profile",
              note=f"{_bounds('width_mm')} mm. AXIAL section: the mid-plane view projects "
                   "this direction away entirely. Plane-strain FEA force scales linearly "
                   "with it"),
        Sweep("tread", "tread_depth_mm", "Tread groove depth",
              (0.0, 1.0, 2.0, 4.0, 6.0), view="profile",
              note=f"{_bounds('tread_depth_mm')} mm. AXIAL section, for the same reason: the "
                   "grooves run around the circumference, so a mid-plane cut shows none of "
                   "them. Three grooves, straight and circumferential — printable flat, and "
                   "no sharp radial edge for the contact solver to trip on"),
    ]


def verdict(params: WheelParams) -> tuple[str, str]:
    """Colour and text for one design's screening outcome. Never raises — invariant 3."""
    violations = check_design(params, TPU)
    worst = next((s for s in _ORDER if any(v.severity is s for v in violations)), None)
    colour, word = _VERDICT[worst]
    if worst is None:
        return colour, word
    # One name and a count, not a list. Panels sit shoulder to shoulder, so a two-name label
    # runs under its neighbour and the reader cannot tell which design it belongs to — the
    # figure then says the wrong thing about a design rather than nothing about it.
    names = [v.name for v in violations if v.severity is worst]
    tail = f" +{len(names) - 1}" if len(names) > 1 else ""
    return colour, f"{word}\n{names[0]}{tail}"


def draw_panel(ax, params: WheelParams, sweep: Sweep, value, plt) -> None:
    """One design in one axes, titled by its value and labelled by its verdict."""
    from wheelopt.viz import draw_wheel_profile, draw_wheel_section

    try:
        (draw_wheel_profile if sweep.view == "profile" else draw_wheel_section)(
            ax, params, annotate=False)
    except (ValueError, ZeroDivisionError) as exc:
        # A degenerate design has no outline to draw, and that is information rather than an
        # error: the panel says so and the figure keeps its shape.
        ax.text(0.5, 0.5, f"cannot be drawn\n{type(exc).__name__}", transform=ax.transAxes,
                ha="center", va="center", fontsize=7.5, color=_VERDICT[Severity.DEGENERATE][0])
        ax.set_aspect("equal")
        ax.axis("off")
    colour, word = verdict(params)
    ax.set_title(sweep.label(value), fontsize=9.5, pad=4)
    ax.text(0.5, -0.02, word, transform=ax.transAxes, ha="center", va="top",
            fontsize=7.0, color=colour, wrap=True)


def _wrap(text: str, width: int = 96) -> str:
    import textwrap

    return "\n".join(textwrap.wrap(text, width)) if text else ""


def figure_for(sweep: Sweep, plt):
    """One figure: a row of designs, a title, and the caption that says what to read."""
    n = len(sweep.values)
    fig, axes = plt.subplots(1, n, figsize=(2.1 * n, 3.0))
    axes = [axes] if n == 1 else list(axes)
    for ax, value in zip(axes, sweep.values):
        draw_panel(ax, replace(sweep.base, **{sweep.field: value}), sweep, value, plt)
    view = "axial section" if sweep.view == "profile" else "mid-plane section"
    base = "T7 claw, bandless" if sweep.base is CLAW else "T3, banded"
    fig.suptitle(f"{sweep.title}  —  {sweep.field}", fontsize=11, y=0.99)
    fig.text(0.5, 0.90, f"{view} · base design: {base}, R {sweep.base.outer_radius_mm:g} mm",
             ha="center", fontsize=8, color="#5d6b72")
    if sweep.note:
        fig.text(0.5, 0.035, _wrap(sweep.note), ha="center", va="top", fontsize=7.5,
                 color="#5d6b72")
    fig.subplots_adjust(top=0.80, bottom=0.20, wspace=0.05)
    return fig


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--only", action="append", metavar="KEY",
                   help="draw one sweep instead of all of them; repeatable. Keys are "
                        + ", ".join(s.key for s in sweeps()))
    p.add_argument("--format", choices=("png", "pdf", "svg"), default="png",
                   help="PDF and SVG are vector, for a write-up; PNG is for a browser")
    p.add_argument("--dpi", type=int, default=170,
                   help="raster resolution. Ignored by the vector formats")
    p.add_argument("--contact-sheet", action="store_true",
                   help="also write one multi-page PDF holding every sweep, in order")
    p.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "figures",
                   help="directory for the figures")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        from wheelopt.viz import MissingPlotting, _pyplot
    except ImportError as exc:      # pragma: no cover - environment
        print(f"cannot import the plotting layer: {exc}")
        return 2
    try:
        plt = _pyplot()
    except MissingPlotting as exc:
        print(exc)
        return 2

    chosen = sweeps()
    if args.only:
        by_key = {s.key: s for s in chosen}
        unknown = [k for k in args.only if k not in by_key]
        if unknown:
            print(f"unknown sweep(s): {', '.join(unknown)}")
            print(f"available: {', '.join(by_key)}")
            return 1
        chosen = [by_key[k] for k in args.only]

    args.out.mkdir(parents=True, exist_ok=True)
    figures = []
    for sweep in chosen:
        fig = figure_for(sweep, plt)
        path = args.out / f"geometry_{sweep.key}.{args.format}"
        fig.savefig(path, dpi=args.dpi)
        figures.append((fig, path, sweep))
        print(f"  {path}  ({len(sweep.values)} designs)")

    if args.contact_sheet:
        from matplotlib.backends.backend_pdf import PdfPages

        sheet = args.out / "geometry_all.pdf"
        with PdfPages(sheet) as pdf:
            for fig, _, _ in figures:
                pdf.savefig(fig)
        print(f"  {sheet}  ({len(figures)} pages)")

    for fig, _, _ in figures:
        plt.close(fig)
    print(f"\n{len(figures)} figure(s) in {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
