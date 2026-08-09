#!/usr/bin/env python3
"""Run one FEA load case on one design, or just generate the deck.

    # deck + mesh only, no solver needed. The developer loop.
    python scripts/run_fea.py --dry-run --tiny

    # the real thing
    python scripts/run_fea.py --case flat --case step_edge

Exit codes: 0 solved (or deck written), 1 the evaluation failed, 2 a dependency is missing.

The failure path is deliberately boring: `run_load_case` never raises, so everything here
is reporting. See invariant 4.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from wheelopt.cad.cli import (  # noqa: E402
    add_geometry_args,
    add_material_args,
    material_from_args,
    params_from_args,
)
from wheelopt.cad.constraints import check_design, is_feasible  # noqa: E402
from wheelopt.fea.loadcase import (  # noqa: E402
    IndenterSpec,
    LoadCase,
    LoadCaseKind,
    MeshSpec,
    SolverSpec,
)

_CASES = {"flat": LoadCaseKind.RADIAL_FLAT, "step_edge": LoadCaseKind.RADIAL_STEP_EDGE}

#: A model small enough to iterate on. Radius and width sit at the bottom of the searched
#: range rather than below it, so the design still passes screening and the run stays
#: representative. It grew with the platform (2026-08-07): the old R40/W20 preset is now
#: out of bounds and would be rejected before it ever reached the mesher. Element sizes
#: were coarsened to compensate, so the solve time is roughly unchanged.
TINY = {
    "radius": 60.0, "width": 30.0, "spokes": 6, "thickness": 5.0,
    "rim_thickness": 3.0, "hub_radius": 20.0,
    "size_spoke": 0.008, "size_rim": 0.010, "size_hub": 0.010,
    "delta_max": 0.006, "n_points": 6,
}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_geometry_args(p)
    add_material_args(p)

    c = p.add_argument_group("load case")
    c.add_argument("--case", action="append", choices=sorted(_CASES),
                   help="repeatable; defaults to flat")
    c.add_argument("--nominal-load", type=float, default=LoadCase().nominal_load_n,
                   help="static load per wheel, N")
    c.add_argument("--delta-max", type=float, default=LoadCase().delta_max_m,
                   help="peak indentation, m")
    c.add_argument("--n-points", type=int, default=LoadCase().n_points_per_branch)
    c.add_argument("--friction", type=float, default=LoadCase().friction_mu)
    c.add_argument("--step-height", type=float, default=IndenterSpec().step_height_m)
    c.add_argument("--step-edge-fillet", type=float, default=IndenterSpec().edge_fillet_m)

    m = p.add_argument_group("mesh (metres)")
    m.add_argument("--size-spoke", type=float, default=MeshSpec().size_spoke_m)
    m.add_argument("--size-rim", type=float, default=MeshSpec().size_rim_m)
    m.add_argument("--size-hub", type=float, default=MeshSpec().size_hub_m)
    m.add_argument("--order", type=int, choices=(1, 2), default=MeshSpec().order)
    m.add_argument("--plane-strain", action="store_true",
                   help="2-D plane-strain cross-section (CPE6) instead of the solid: "
                        "seconds instead of hours, at the cost of every out-of-plane "
                        "effect including lateral spoke buckling. A screening tier, and "
                        "it reports a stiffness that is too high; calibrate against 3-D")
    m.add_argument("--half-width", action="store_true",
                   help="NOT IMPLEMENTED — mid-plane symmetry is ignored by the mesher, so "
                        "this only changes the cache key. Rejected rather than accepted "
                        "silently; see MeshSpec.half_width_symmetry")

    s = p.add_argument_group("solver")
    s.add_argument("--ccx", type=Path, default=None)
    s.add_argument("--threads", type=int, default=SolverSpec().n_threads)
    s.add_argument("--timeout", type=float, default=SolverSpec().timeout_s)
    s.add_argument("--contact-stiffness", type=float,
                   default=SolverSpec().contact_stiffness_factor,
                   help="penalty stiffness multiplier (x E / element size). Lower it if a "
                        "frictional contact diverges — a fine mesh gets a stiff penalty. "
                        "The plane-strain tier needs <= 5 with friction; 20 diverges")

    io = p.add_argument_group("io")
    io.add_argument("--step", type=Path, default=None, help="reuse an existing STEP")
    io.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "cache" / "fea")
    io.add_argument("--no-cache", action="store_true")
    io.add_argument("--dry-run", action="store_true",
                    help="mesh and write the deck, then stop. Needs gmsh, not CalculiX")
    io.add_argument("--tiny", action="store_true",
                    help="override geometry and mesh for a fast debug model")
    io.add_argument("--plot-pdf", nargs="?", const=True, default=None, metavar="PATH",
                    help="write a vector PDF of the design and the extracted metrics. "
                         "Bare flag picks <out>/report_<design hash>.pdf. Needs matplotlib "
                         "(pip install -e '.[viz]'); with --dry-run it writes the design "
                         "page alone, which needs no solver.")
    return p


def _plot_path(args, params, suffix: str = "report") -> Path:
    """Resolve --plot-pdf into a path, defaulting to a hash-named file under --out."""
    if args.plot_pdf is not True:
        return Path(args.plot_pdf)
    return Path(args.out) / f"{suffix}_{params.design_hash()}.pdf"


#: ``--tiny`` key -> the argparse destination it overrides.
_TINY_DESTS = {
    "radius": "radius", "width": "width", "spokes": "spokes", "thickness": "thickness",
    "rim_thickness": "rim_thickness", "hub_radius": "hub_radius",
    "size_spoke": "size_spoke", "size_rim": "size_rim", "size_hub": "size_hub",
    "delta_max": "delta_max", "n_points": "n_points",
}


def apply_tiny(args: argparse.Namespace, parser: argparse.ArgumentParser) -> list[str]:
    """Shrink the model for a fast debug loop. **Explicit flags win.**

    A flag the parser accepts and then ignores is this project's recurring failure mode, and
    ``--tiny --rim-thickness 0`` is exactly that shape: it would run the banded preset and
    report it as the bandless design the user asked for. Anything the caller moved off its
    default is left alone; the rest is overridden and named in the returned list.
    """
    overridden = []
    for key, dest in _TINY_DESTS.items():
        if getattr(args, dest) != parser.get_default(dest):
            continue  # the caller said otherwise
        setattr(args, dest, TINY[key])
        overridden.append(dest)
    return overridden


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.tiny:
        kept = sorted(set(_TINY_DESTS.values()) - set(apply_tiny(args, parser)))
        if kept:
            print(f"--tiny: keeping your {', '.join('--' + k.replace('_', '-') for k in kept)}")

    params = params_from_args(args)
    material = material_from_args(args)

    violations = check_design(params, material)
    for v in violations:
        print(f"  [{v.severity.value}] {v.name}: {v.message}")
    if not is_feasible(violations):
        print("\nDesign rejected by screening; no geometry built.")
        return 1

    # Spec construction validates; a bad combination is a usage error, so report it as one
    # rather than as a traceback. This is config time, before any compute is committed.
    try:
        mesh_spec = MeshSpec(
            size_spoke_m=args.size_spoke,
            size_rim_m=args.size_rim,
            size_hub_m=args.size_hub,
            order=args.order,
            dimension=2 if args.plane_strain else 3,
            half_width_symmetry=args.half_width,
        )
        solver = SolverSpec(
            timeout_s=args.timeout,
            n_threads=args.threads,
            contact_stiffness_factor=args.contact_stiffness,
        )
        indenter = IndenterSpec(
            step_height_m=args.step_height, edge_fillet_m=args.step_edge_fillet
        )
    except (ValueError, NotImplementedError) as exc:
        print(f"\n{type(exc).__name__}: {exc}")
        return 2
    kinds = [_CASES[c] for c in (args.case or ["flat"])]

    if args.dry_run:
        return _dry_run(args, params, material, mesh_spec, solver, indenter, kinds)

    from wheelopt.fea.runner import run_load_case

    worst = 0
    results = []
    for kind in kinds:
        case = LoadCase(
            kind=kind,
            nominal_load_n=args.nominal_load,
            delta_max_m=args.delta_max,
            n_points_per_branch=args.n_points,
            friction_mu=args.friction,
            indenter=indenter,
        )
        print(f"\n=== {kind.value} ===")
        started = time.perf_counter()
        result = run_load_case(
            params, material, case,
            mesh_spec=mesh_spec, solver=solver, step_path=args.step,
            cache_root=args.out, ccx_path=args.ccx, use_cache=not args.no_cache,
        )
        results.append(result)
        print(f"status : {result.status.value}  ({time.perf_counter() - started:.1f} s)")
        if result.message:
            print(f"detail : {result.message}")
        print(f"solver : {result.diagnostics.summary()}")

        if not result.ok:
            if result.diagnostics.log_tail:
                print("--- solver output (tail) ---")
                print(result.diagnostics.log_tail)
            worst = 2 if result.is_environment_failure else 1
            continue

        _report(result)

    if args.plot_pdf is not None:
        from wheelopt.viz import MissingPlotting, write_report_pdf

        try:
            written = write_report_pdf(
                _plot_path(args, params), params, material, results
            )
        except MissingPlotting as exc:
            print(f"\n{exc}")
            return 2
        print(f"\nplot   : {written}")
    return worst


def _dry_run(args, params, material, mesh_spec, solver, indenter, kinds) -> int:
    """Mesh, build the deck, write it. Exercises everything except the solver."""
    from wheelopt.cad.compliant_spoke import build_wheel
    from wheelopt.cad.export import export
    from wheelopt.fea.deck import DeckError, build_deck
    from wheelopt.fea.hyperelastic import UnknownMaterial, for_material
    from wheelopt.fea.indenter import build_indenter
    from wheelopt.fea.mesh import MeshFailure, mesh_step

    out = Path(args.out) / "dry-run"
    out.mkdir(parents=True, exist_ok=True)

    try:
        hyper = for_material(material, params.spoke_thickness_mm)
    except UnknownMaterial as exc:
        print(f"\n{exc}")
        return 1
    print(f"\nmaterial: mu0 = {hyper.initial_shear_modulus_pa / 1e6:.3f} MPa, "
          f"E = {hyper.initial_youngs_pa / 1e6:.3f} MPa, "
          f"nu_eff = {hyper.poisson_effective:.3f}")

    step = Path(args.step) if args.step else None
    if step is None:
        result = build_wheel(params, material)
        if not result.ok or result.part is None:
            print("geometry build failed")
            return 1
        step = export(result.part, params, out).step
    print(f"step    : {step.name} ({step.stat().st_size // 1024} kB)")

    try:
        started = time.perf_counter()
        if mesh_spec.dimension == 2:
            from wheelopt.fea.section2d import mesh_section

            mesh = mesh_section(params, mesh_spec)
        else:
            mesh = mesh_step(step, params, mesh_spec)
    except MeshFailure as exc:
        print(f"mesh failed: {exc}")
        return 1
    except ImportError:
        print("gmsh is not installed; pip install -e '.[fea]'")
        return 2
    dof_per_node = 2 if mesh_spec.dimension == 2 else 3
    print(f"mesh    : {mesh.stats.summary()} in {time.perf_counter() - started:.1f} s")
    print(f"          {mesh.n_nodes * dof_per_node} DOF, "
          f"max aspect {mesh.stats.max_aspect_ratio:.1f}")
    print("          sets " + ", ".join(f"{k}={len(v)}" for k, v in mesh.node_sets.items()))

    for kind in kinds:
        case = LoadCase(
            kind=kind, nominal_load_n=args.nominal_load, delta_max_m=args.delta_max,
            n_points_per_branch=args.n_points, friction_mu=args.friction,
            indenter=indenter,
        )
        ind = build_indenter(
            kind, indenter, params.outer_radius_mm * 1e-3, params.width_mm * 1e-3,
            dimension=mesh_spec.dimension,
        )
        try:
            bundle = build_deck(
                mesh, ind, params, material, hyper, case, solver,
                design_hash=params.design_hash(),
            )
        except DeckError as exc:
            # A dry run exists to surface exactly this before any solver time is spent.
            print(f"deck rejected: {exc}")
            return 1
        path = out / f"job_{kind.value}.inp"
        path.write_text(bundle.text)
        print(f"deck    : {path.name}  {len(bundle.text) // 1024} kB, "
              f"{bundle.n_nodes} nodes, {bundle.n_elements} elements, "
              f"{len(bundle.slave_nodes)} slave nodes")

    if args.plot_pdf is not None:
        # Design page only: a dry run has no metrics to plot yet.
        from wheelopt.cad.constraints import check_design as _check
        from wheelopt.viz import MissingPlotting, write_design_pdf

        try:
            written = write_design_pdf(
                _plot_path(args, params, "design"), params, material,
                violations=_check(params, material),
            )
        except MissingPlotting as exc:
            print(f"\n{exc}")
            return 2
        print(f"plot    : {written}")
    return 0


def _report(result) -> None:
    curve = result.curve
    print(f"peak   : {curve.peak_force_n:.2f} N at {curve.peak_delta_m * 1e3:.2f} mm")
    k = curve.tangent_stiffness_n_per_m()
    print(f"k_r    : {k[len(k) // 2] / 1e3:.2f} kN/m at mid-sweep, "
          f"{k[-1] / 1e3:.2f} kN/m at peak "
          f"({'stiffening' if k[-1] > k[len(k) // 2] else 'softening'})")
    if result.p95_von_mises_pa:
        print(f"stress : p95 {result.p95_von_mises_pa / 1e6:.2f} MPa, "
              f"peak {result.peak_von_mises_pa / 1e6:.2f} MPa "
              "(peak is mesh-dependent at the spoke root)")
    if result.patch is not None and len(result.patch.force_n):
        i = -1
        print(f"patch  : {result.patch.length_m[i] * 1e3:.1f} x "
              f"{result.patch.width_m[i] * 1e3:.1f} mm at "
              f"{result.patch.force_n[i]:.1f} N, peak pressure "
              f"{result.patch.peak_pressure_pa[i] / 1e3:.1f} kPa")
    if result.buckling_detected and result.buckling_load_n is not None:
        print(f"buckle : DETECTED, limit point at {result.buckling_load_n:.1f} N")
    else:
        print("buckle : none")
    print(f"loop   : {result.loop_area_fraction:.4%} of loading work "
          "(numerical QC; hyperelasticity has no true hysteresis)")
    for v in result.violations:
        print(f"  [{v.severity.value}] {v.name}: {v.message}")


if __name__ == "__main__":
    raise SystemExit(main())
