#!/usr/bin/env python3
"""Turn a knob, see what moved. One design (or a few) end to end, into one HTML page.

    python scripts/explore.py --spokes 8 --thickness 6                  # one design
    python scripts/explore.py --compare spokes=6,8,12                   # three, shared axes
    python scripts/explore.py --rim-thickness 0 --claw-taper 0.5 --spoke-phase -90
    python scripts/explore.py --spokes 8 --no-sim                       # skip MuJoCo, faster

Runs the whole chain — screen, plane-strain FEA, ROM fit, step climb, render — and writes a
single self-contained HTML file with the section drawing, the load curve, the contact patch,
the fitted segment law and the simulation, each labelled with the tier that produced it.

**This is the screening tier and it is a playground, not a verdict.** Plane strain cannot see
lateral spoke buckling, and every claw number carries the open question in `docs/plan/TODO.md`
#24. The page says so on the panels it applies to; do not quote a number off it without
reading the banner above the number.

Nothing here raises on a failed stage. A design that will not mesh, will not converge, or
will not fit still produces a page saying so — that is the interesting case, and losing it to
a traceback would be the worst outcome for a tool whose job is exploration.

Exit 0 if the page was written, 2 if a dependency is missing.
"""

from __future__ import annotations

import argparse
import shlex
import sys
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))


from wheelopt.cad.cli import (
    add_geometry_args,
    add_material_args,
    material_from_args,
    params_from_args,
)
from wheelopt.cad.constraints import check_design, is_feasible
from wheelopt.cad.params import WheelParams
from wheelopt.fea.loadcase import LoadCase, LoadCaseKind, MeshSpec, SolverSpec
from wheelopt.report import (
    Panel,
    figure_svg,
    gif_data_uri,
    new_figure,
    table,
    write_report,
)

#: The plane-strain tier's settings, from CLAUDE.md's command block. The fine section mesh
#: over-stiffens penalty contact, so the default factor of 20 diverges under friction.
SECTION_MESH = {"size_spoke_m": 0.0025, "size_rim_m": 0.003, "size_hub_m": 0.002}
CONTACT_STIFFNESS = 5.0

#: Which flags `--compare` may sweep. Restricted on purpose: sweeping something the pipeline
#: keys its cache on but the plots do not label would silently overlay unlike things.
COMPARABLE = {
    "spokes": ("n_spokes", int),
    "thickness": ("spoke_thickness_mm", float),
    "radius": ("outer_radius_mm", float),
    "rim_thickness": ("rim_thickness_mm", float),
    "claw_taper": ("claw_taper_ratio", float),
    "curvature": ("spoke_curvature_1_per_mm", float),
    "infill": ("__material__infill_density", float),
}

PLANE_STRAIN_NOTE = (
    "2-D plane strain (CPE6), the screening tier. Measured against the 3-D solid at matched "
    "frictionless settings: force ratio 0.90, k_r ratio 0.86, patch length 0.95. It cannot "
    "see lateral spoke buckling at all, so it screens and the 3-D tier decides."
)
ROM_NOTE = (
    "Segmented ring fitted to the curve above. Look at the tangents, not only the RMS: on a "
    "banded wheel the deconvolution is ill-posed, and the error keeps falling as the table "
    "gets finer while the law stops being physical."
)
SIM_NOTE = (
    "MuJoCo, ring ROM against a rigid wheel matched in mass, radius and rotational inertia. "
    "Cost of transport is a statement about sim.step_climb.TPU_LOSS_FACTOR (0.15, a "
    "literature midpoint on a 0.05-0.30 span, no DMA behind it)."
)
CLAW_CAUTION = (
    "A claw tip either sticks to the ground or slides on it, and the two give very different "
    "forces: 22.7 N against 4.59 N at 1 mm on the nominal claw. It is a switch, not a "
    "gradient — every mu from 0.2 to 1.2 gives the same answer to five figures, and both "
    "branches are mesh-converged to under 1%. The stick branch is the physical one and is "
    "what a radial-slide ring segment corresponds to, so that is what these runs use. What "
    "the ring still cannot represent is the claw bending backwards under drive torque; see "
    "docs/plan/TODO.md #20."
)


@dataclass
class Design:
    """One design and whatever each stage managed to produce for it."""

    label: str
    params: WheelParams
    material: Any
    violations: list = field(default_factory=list)
    curve: Any = None
    patch: Any = None
    buckling: tuple = (False, None)
    fit: Any = None
    n_contact: int = 0
    signatures: list = field(default_factory=list)
    step: Any = None
    rigid: Any = None
    gif: Path | None = None
    messages: list[str] = field(default_factory=list)

    @property
    def feasible(self) -> bool:
        """Warnings do not stop a run. A bandless wheel always warns; it is still a design."""
        return is_feasible(self.violations)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_geometry_args(p)
    add_material_args(p)
    p.add_argument("--compare", metavar="KEY=V1,V2,...",
                   help=f"sweep one parameter. One of: {', '.join(sorted(COMPARABLE))}")
    p.add_argument("--delta-max", type=float, default=0.010,
                   help="deepest indentation in the FEA sweep, metres")
    p.add_argument("--n-points", type=int, default=12)
    p.add_argument("--friction", type=float, default=None,
                   help="tread/ground friction in the FEA. Default depends on topology: 0 "
                        "for a banded wheel, which is what the 2-D tier was calibrated at, "
                        "and 0.8 for a bandless claw, whose tip must stick rather than slide "
                        "(frictionless understates a claw fivefold). Printed on every run")
    p.add_argument("--segments", type=int, default=None,
                   help="ring segments. Default: n_spokes without a band (segments are "
                        "claws), 24 with one")
    p.add_argument("--law", choices=("cubic", "table"), default="table")
    p.add_argument("--step-height", type=float, default=None, help="mm; default 0.6 R")
    p.add_argument("--no-sim", action="store_true", help="skip MuJoCo entirely")
    p.add_argument("--no-render", action="store_true", help="metrics but no animation")
    p.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "explore")
    p.add_argument("--cache", type=Path, default=REPO_ROOT / "data" / "cache" / "fea")
    p.add_argument("--threads", type=int, default=4)
    return p


# --------------------------------------------------------------------------------------
# stages. Each catches its own failure and records it on the Design.


def stage_screen(design: Design) -> None:
    design.violations = list(check_design(design.params, design.material))


def stage_fea(design: Design, args: argparse.Namespace) -> None:
    from wheelopt.fea.extract import detect_buckling
    from wheelopt.fea.runner import run_load_case

    mesh = MeshSpec(dimension=2, **SECTION_MESH)
    case = LoadCase(kind=LoadCaseKind.RADIAL_FLAT, delta_max_m=args.delta_max,
                    n_points_per_branch=args.n_points, friction_mu=args.friction)
    result = run_load_case(
        design.params, design.material, case, mesh_spec=mesh,
        solver=SolverSpec(n_threads=args.threads,
                          contact_stiffness_factor=CONTACT_STIFFNESS),
        cache_root=args.cache,
    )
    if not result.ok:
        design.messages.append(f"FEA: {result.status.value}: {result.message}")
        return
    design.curve, design.patch = result.curve, result.patch
    design.buckling = detect_buckling(result.curve)


def stage_rom(design: Design, args: argparse.Namespace) -> None:
    from wheelopt.rom.fit import contact_segments, fit_spring_law, fit_tabulated_law
    from wheelopt.rom.ring import ring_for_design

    loading = design.curve.loading
    delta = design.curve.delta_m[loading]
    force = design.curve.force_n[loading]
    n = args.segments
    if n is None:
        # Segments are claws when there is nothing coupling them. That is not a default so
        # much as the only discretisation that means anything for a bandless wheel.
        n = design.params.n_spokes if not design.params.has_shear_band else 24
    spec = ring_for_design(design.params, design.material, n_segments=n)
    try:
        design.fit = (fit_spring_law(spec, delta, force) if args.law == "cubic"
                      else fit_tabulated_law(spec, delta, force))
    except Exception as exc:  # noqa: BLE001 - a playground never dies on one stage
        design.messages.append(f"ROM: {type(exc).__name__}: {exc}")
        return
    design.n_contact = contact_segments(spec, float(delta.max()), design.fit.law)


def stage_sim(design: Design, args: argparse.Namespace) -> None:
    from wheelopt.rom.ring import solve_equilibrium
    from wheelopt.sim.step_climb import RigSpec, judge_signatures, run_flat, run_step

    spec, law = design.fit.spec, design.fit.law
    fit_max = float(np.max(design.fit.delta_m))
    payload = float(solve_equilibrium(spec, law, 0.5 * fit_max).force_n) / 9.81
    height = (args.step_height * 1e-3 if args.step_height is not None
              else round(0.6 * spec.radius_m, 3))
    rig = RigSpec(payload_kg=max(payload, 0.05), step_height_m=height)
    runs = {
        (name, phase): runner(spec, law, rig, rigid=rigid, fit_max_m=fit_max)
        for name, rigid in (("compliant", False), ("rigid", True))
        for phase, runner in (("flat", run_flat), ("step", run_step))
    }
    for (name, phase), result in runs.items():
        if not result.ok:
            design.messages.append(f"sim {name} {phase}: {result.message}")
            return
    design.step = runs[("compliant", "step")]
    design.rigid = runs[("rigid", "step")]
    design.signatures = judge_signatures(
        spec, law,
        compliant_flat=runs[("compliant", "flat")], compliant_step=design.step,
        rigid_flat=runs[("rigid", "flat")], rigid_step=design.rigid,
        step_height_m=height, static_load_n=payload * 9.81,
    )


def stage_render(design: Design, args: argparse.Namespace, out: Path) -> None:
    import subprocess

    cmd = [
        sys.executable, str(REPO_ROOT / "scripts" / "render_step.py"),
        "--radius", str(design.params.outer_radius_mm),
        "--width", str(design.params.width_mm),
        "--spokes", str(design.params.n_spokes),
        "--thickness", str(design.params.spoke_thickness_mm),
        "--rim-thickness", str(design.params.rim_thickness_mm),
        "--hub-radius", str(design.params.hub_radius_mm),
        "--claw-taper", str(design.params.claw_taper_ratio),
        "--spoke-phase", str(design.params.spoke_phase_deg),
        "--plane-strain", "--delta-max", str(args.delta_max),
        "--n-points", str(args.n_points),
        "--segments", str(design.fit.spec.n_segments),
        "--out", str(out), "--cache", str(args.cache),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=1800,
                          check=False)
    if proc.returncode != 0:
        design.messages.append(f"render: exit {proc.returncode}: {proc.stderr.strip()[-300:]}")
        return
    gifs = sorted(out.glob("step_compliant_*.gif"), key=lambda p: p.stat().st_mtime)
    design.gif = gifs[-1] if gifs else None


# --------------------------------------------------------------------------------------
# panels


def panel_design(designs: list[Design]) -> Panel:
    from wheelopt.viz import _pyplot, draw_wheel_section

    plt = _pyplot()
    fig, axes = plt.subplots(1, len(designs), figsize=(3.4 * len(designs), 3.6), dpi=100)
    for ax, design in zip(np.atleast_1d(axes), designs):
        draw_wheel_section(ax, design.params, design.material, annotate=len(designs) == 1)
        ax.set_aspect("equal")
        ax.set_title(design.label, fontsize=12)
        ax.axis("off")
    fig.tight_layout()
    body = figure_svg(fig)
    plt.close(fig)

    rows = []
    for design in designs:
        p = design.params
        verdict = "feasible" if design.feasible else "REJECTED"
        worst = "; ".join(v.name for v in design.violations) or "none"
        rows.append((design.label, f"{p.outer_radius_mm:g} x {p.width_mm:g} mm",
                     f"{p.n_spokes}", f"{p.spoke_thickness_mm:g} / {p.tip_thickness_mm:.2f}",
                     f"{p.rim_thickness_mm:g}", verdict, worst))
    body += table(
        rows,
        header=("design", "R x W", "spokes", "root / tip mm", "band mm", "screen", "flags"),
    )
    return Panel(
        title="Design",
        body=body,
        provenance="Geometry from cad.centreline — the same module the 3-D solid is "
                   "extruded from, so this is the design, not an impression of it. "
                   "Screening is the millisecond pre-filter, not FEA.",
    )


def panel_fea(designs: list[Design]) -> Panel | None:
    solved = [d for d in designs if d.curve is not None]
    if not solved:
        return None
    from wheelopt.viz import _pyplot

    plt = _pyplot()
    fig, axes = new_figure(plt, ncols=3, width=13.0, height=3.6)

    for design in solved:
        loading = design.curve.loading
        d = design.curve.delta_m[loading] * 1e3
        f = design.curve.force_n[loading]
        axes[0].plot(d, f, marker="o", ms=2.5, lw=1.4, label=design.label)
        axes[1].plot(d, design.curve.tangent_stiffness_n_per_m()[: len(d)] * 1e-3,
                     marker="o", ms=2.5, lw=1.4, label=design.label)
        if design.patch is not None:
            axes[2].plot(design.patch.force_n, design.patch.length_m * 1e3,
                         marker="o", ms=2.5, lw=1.4, label=design.label)
    axes[0].set_xlabel("indentation δ, mm"); axes[0].set_ylabel("force, N")
    axes[1].set_xlabel("indentation δ, mm"); axes[1].set_ylabel("tangent dF/dδ, N/mm")
    axes[1].axhline(0.0, color="#b03030", lw=0.8, ls=(0, (4, 3)))
    axes[2].set_xlabel("load, N"); axes[2].set_ylabel("contact patch length, mm")
    if len(solved) > 1:
        axes[0].legend(fontsize=10, frameon=False)
    fig.tight_layout()
    body = figure_svg(fig)
    plt.close(fig)

    rows = []
    for design in solved:
        loading = design.curve.loading
        f = design.curve.force_n[loading]
        buckled, load = design.buckling
        patch = ("n/a" if design.patch is None
                 else f"{design.patch.length_m[-1] * 1e3:.1f}")
        rows.append((design.label, f"{f.max():.2f}", patch,
                     f"{buckled}" + (f" at {load:.1f} N" if load else ""),
                     "; ".join(design.messages) or "-"))
    body += table(rows, header=("design", "peak N", "patch mm at peak",
                                "buckling", "notes"))

    caution = None
    if any(d.patch is not None and float(np.max(d.patch.length_m)) <= 1e-9 for d in solved):
        caution = (
            "A contact patch of 0.0 mm means the tread touched on a single node. The force "
            "is then a point reaction whose magnitude is set by whether that node sticks or "
            "slips, not by how hard the structure resists — check the friction sensitivity "
            "before reading anything into the load curve."
        )
    return Panel(title="FEA — load curve, tangent stiffness, contact patch",
                 body=body, provenance=PLANE_STRAIN_NOTE, caution=caution)


def panel_rom(designs: list[Design]) -> Panel | None:
    fitted = [d for d in designs if d.fit is not None]
    if not fitted:
        return None
    from wheelopt.rom.ring import TabulatedLaw
    from wheelopt.viz import _pyplot

    plt = _pyplot()
    fig, axes = new_figure(plt, ncols=2, width=10.0, height=3.6)

    for design in fitted:
        fit = design.fit
        u = np.linspace(0.0, float(np.max(fit.delta_m)), 250)
        axes[0].plot(u * 1e3, fit.law.force_n(u), lw=1.5, label=design.label)
        axes[1].plot(u * 1e3, np.asarray(fit.law.stiffness_n_per_m(u)) * 1e-3, lw=1.5,
                     label=design.label)
    axes[0].set_xlabel("segment compression u, mm"); axes[0].set_ylabel("segment force, N")
    axes[1].set_xlabel("segment compression u, mm"); axes[1].set_ylabel("df/du, N/mm")
    axes[1].axhline(0.0, color="#b03030", lw=0.8, ls=(0, (4, 3)))
    if len(fitted) > 1:
        axes[0].legend(fontsize=10, frameon=False)
    fig.tight_layout()
    body = figure_svg(fig)
    plt.close(fig)

    rows = []
    for design in fitted:
        fit = design.fit
        kind = "table" if isinstance(fit.law, TabulatedLaw) else "cubic"
        rows.append((
            design.label, kind, f"{fit.spec.n_segments}",
            f"{fit.rms_error_fraction:.2%}", f"{design.n_contact}",
            "yes" if fit.law.is_monotone_nonneg else "no (has a limit point)",
            "yes" if fit.ok else "NO",
        ))
    body += table(rows, header=("design", "law", "segments", "RMS", "segments in contact",
                                "monotone", "usable"))
    caution = None
    if any(d.n_contact < 3 for d in fitted):
        caution = (
            "Fewer than three segments carry load. A ring like that is modelling point loads "
            "rather than a patch, and it can still fit the curve beautifully — fit error "
            "cannot see this, which is why the count is in the table."
        )
    return Panel(title="ROM — the fitted segment spring law", body=body,
                 provenance=ROM_NOTE, caution=caution)


def panel_sim(designs: list[Design]) -> Panel | None:
    simulated = [d for d in designs if d.step is not None]
    if not simulated:
        return None
    rows = []
    for design in simulated:
        for sig in design.signatures:
            rows.append((design.label, sig.name, sig.compliant, sig.rigid,
                         "PASS" if sig.passed else "FAIL"))
    body = table(rows, header=("design", "signature", "compliant", "rigid", "verdict"))

    extra = []
    for design in simulated:
        extra.append((
            design.label,
            f"{design.step.distance_m * 1e3:.0f}",
            f"{design.step.cost_of_transport:.4f}",
            f"{design.step.peak_compression_m * 1e3:.2f}",
            f"{design.step.fraction_beyond_fit:.0%}",
        ))
    body += table(extra, header=("design", "travelled mm", "cost of transport",
                                 "peak segment compression mm", "beyond fitted range"))

    for design in simulated:
        if design.gif is not None:
            body += (f'<img class="frames" alt="{design.label} step climb" '
                     f'src="{gif_data_uri(design.gif)}">')

    caution = None
    if any(d.step.fraction_beyond_fit > 0.0 for d in simulated):
        caution = (
            "Part of this run pressed the segments deeper than the FEA curve the law was "
            "fitted to. Beyond the last knot a tabulated law continues at a clamped slope, "
            "which understates the force — the run is an extrapolation, not a measurement."
        )
    return Panel(title="Simulation — step climb against a rigid wheel", body=body,
                 provenance=SIM_NOTE, caution=caution)


# --------------------------------------------------------------------------------------


def parse_compare(spec: str, args: argparse.Namespace) -> list[tuple[str, dict]]:
    """``spokes=6,8,12`` -> a label and an override dict per value."""
    if "=" not in spec:
        raise SystemExit(f"--compare wants KEY=V1,V2,...; got {spec!r}")
    key, values = spec.split("=", 1)
    key = key.strip().replace("-", "_")
    if key not in COMPARABLE:
        raise SystemExit(
            f"cannot compare {key!r}. One of: {', '.join(sorted(COMPARABLE))}"
        )
    field_name, cast = COMPARABLE[key]
    out = []
    for raw in values.split(","):
        raw = raw.strip()
        if not raw:
            continue
        out.append((f"{key}={raw}", {field_name: cast(raw)}))
    if not out:
        raise SystemExit(f"--compare {spec!r} lists no values")
    return out


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        from wheelopt.viz import _pyplot

        _pyplot()
    except Exception as exc:  # noqa: BLE001
        print(f"the report needs matplotlib: {exc}")
        return 2

    base_params = params_from_args(args)
    base_material = material_from_args(args)
    if args.friction is None:
        # Not a tuning knob and not a default to inherit silently. A banded wheel rolls on
        # its band and the 2-D tier's calibration against 3-D was done frictionless; a claw
        # runs on a tip that either sticks or slides, and frictionless picks the slide
        # branch — 4.59 N against 22.7 N on the nominal claw. Chosen from the topology and
        # echoed below so the run says which branch it took.
        args.friction = 0.0 if base_params.has_shear_band else 0.8
    variants = (parse_compare(args.compare, args) if args.compare
                else [(base_params.design_hash()[:8], {})])

    designs = []
    for label, overrides in variants:
        material = base_material
        geometry = dict(overrides)
        infill = geometry.pop("__material__infill_density", None)
        if infill is not None:
            material = replace(base_material, infill_density=infill)
        designs.append(Design(label=label,
                              params=replace(base_params, **geometry),
                              material=material))

    print(f"friction mu={args.friction:g} "
          f"({'banded, the calibrated frictionless setting' if base_params.has_shear_band else 'bandless: the tip must stick, not slide'})")
    for design in designs:
        started = time.time()
        stage_screen(design)
        note = "feasible" if design.feasible else "REJECTED by screening"
        print(f"[{design.label}] {note}")
        if design.feasible:
            stage_fea(design, args)
            if design.curve is not None:
                stage_rom(design, args)
            if design.fit is not None and not args.no_sim:
                try:
                    stage_sim(design, args)
                except Exception as exc:  # noqa: BLE001
                    design.messages.append(f"sim: {type(exc).__name__}: {exc}")
            if design.step is not None and not args.no_render:
                try:
                    stage_render(design, args, args.out)
                except Exception as exc:  # noqa: BLE001
                    design.messages.append(f"render: {type(exc).__name__}: {exc}")
        for message in design.messages:
            print(f"  {message}")
        print(f"  {time.time() - started:.1f} s")

    panels = [panel_design(designs)]
    for builder in (panel_fea, panel_rom, panel_sim):
        panel = builder(designs)
        if panel is not None:
            panels.append(panel)

    if any(d.params.is_claw for d in designs):
        panels.insert(1, Panel(
            title="Read this before quoting a claw number",
            body="<p>The claw pipeline is complete and the numbers below are real solver "
                 "output. What they <em>mean</em> is the open question.</p>",
            provenance="docs/plan/TODO.md #24, #20",
            caution=CLAW_CAUTION,
        ))

    path = write_report(
        args.out / "explore.html",
        title="Wheel design exploration",
        subtitle=(f"{len(designs)} design(s), plane-strain screening tier, "
                  f"delta to {args.delta_max * 1e3:g} mm, friction mu={args.friction:g}"),
        panels=panels,
        command="python " + " ".join(shlex.quote(a) for a in sys.argv[1:]),
    )
    print(f"\n{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
