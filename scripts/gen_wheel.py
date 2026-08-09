#!/usr/bin/env python3
"""Generate one compliant-spoke wheel: screen, build, export, report mass properties.

    python scripts/gen_wheel.py --radius 70 --spokes 16 --thickness 2.0 --profile curved

Requires build123d. Constraint screening runs first and does not, so an infeasible design
is reported without ever loading the CAD kernel.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from wheelopt.cad.cli import (  # noqa: E402
    add_geometry_args,
    add_material_args,
    material_from_args,
    params_from_args,
)
from wheelopt.cad.constraints import (  # noqa: E402
    PlatformLimits,
    Severity,
    check_design,
    is_feasible,
)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    # Shared with run_fea.py so the two entry points cannot compute different design
    # hashes for the same flags and silently split the cache.
    add_geometry_args(p)
    add_material_args(p)

    o = p.add_argument_group("output")
    o.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "wheels")
    o.add_argument("--stl-tolerance", type=float, default=0.05, help="mm chordal deviation")
    o.add_argument("--screen-only", action="store_true", help="skip geometry, just screen")
    o.add_argument("--plot-pdf", nargs="?", const=True, default=None, metavar="PATH",
                   help="write a vector PDF of the design. Bare flag picks "
                        "<out>/design_<hash>.pdf. Needs matplotlib (pip install -e "
                        "'.[viz]'); works with --screen-only, since the section is drawn "
                        "from the centreline rather than from a built solid.")
    return p


def write_plot(args, params, material, violations) -> int:
    """Write the design PDF. Returns a process exit code."""
    from wheelopt.viz import MissingPlotting, write_design_pdf

    target = (
        Path(args.plot_pdf) if args.plot_pdf is not True
        else Path(args.out) / f"design_{params.design_hash()}.pdf"
    )
    try:
        written = write_design_pdf(target, params, material, violations=violations)
    except MissingPlotting as exc:
        print(f"\n{exc}")
        return 2
    print(f"\nplot {written}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    params = params_from_args(args)
    material = material_from_args(args)

    print(f"design {params.design_hash()}  profile={params.spoke_profile.value}")
    print(
        f"  R={params.outer_radius_mm:.1f}  W={params.width_mm:.1f}  "
        f"span={params.spoke_span_mm:.1f}  sagitta={params.spoke_sagitta_mm:.2f} mm"
    )

    violations = check_design(params, material, PlatformLimits())
    for v in violations:
        print(f"  {v}")

    if not is_feasible(violations):
        hard = [v for v in violations if v.severity is not Severity.WARNING]
        print(f"\nREJECTED by screening ({len(hard)} violation(s)). No geometry built.")
        return 1

    if args.screen_only:
        print("\nfeasible (screen-only, no geometry built)")
        return write_plot(args, params, material, violations) if args.plot_pdf else 0

    from wheelopt.cad.compliant_spoke import MissingCadKernel, build_wheel, tessellate
    from wheelopt.cad.export import export, is_watertight
    from wheelopt.cad.massprops import check_against_brep_volume, mass_properties

    try:
        result = build_wheel(params, material)
    except MissingCadKernel as exc:
        print(f"\n{exc}", file=sys.stderr)
        return 2

    if not result.ok:
        print("\nREJECTED during build.")
        return 1

    vertices, faces = tessellate(result.part, tolerance_mm=args.stl_tolerance)

    watertight, n_bad = is_watertight(faces)
    if not watertight:
        print(f"\nWARNING: mesh is not watertight ({n_bad} bad edge(s)).")
        print("A leaky collision mesh produces silently wrong contact — do not simulate this.")

    rho = material.effective_density_kg_m3(params.spoke_thickness_mm)
    mp = mass_properties(vertices, faces, rho)

    ok_vol, rel = check_against_brep_volume(mp.volume_m3, result.brep_volume_m3)
    paths = export(result.part, params, args.out, stl_tolerance_mm=args.stl_tolerance)

    print(f"\neffective density {rho:.1f} kg/m^3")
    print(mp.summary())
    print(f"mesh-vs-brep volume error {rel:+.3%} {'ok' if ok_vol else 'TOO COARSE'}")
    print(f"\nwrote {paths.step}")
    print(f"      {paths.stl}")

    if args.plot_pdf is not None:
        code = write_plot(args, params, material, violations)
        if code:
            return code
    return 0 if (watertight and ok_vol) else 1


if __name__ == "__main__":
    raise SystemExit(main())
