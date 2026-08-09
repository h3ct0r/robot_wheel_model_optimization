#!/usr/bin/env python3
"""First-week step 4: fit the segmented ring to the FEA k_r(delta) and check it.

    python scripts/run_rom.py --tiny                 # fit + report, no simulator needed
    python scripts/run_rom.py --tiny --mujoco        # also build the ring and press it

Three numbers come out, and they answer different questions:

    fit error      does a ring of N radial springs reproduce the FEA load curve at all?
    MuJoCo gap     does a real MuJoCo ring reproduce the analytic ring it was fitted as?
    contact count  is the ring fine enough to resolve the contact patch, or is it three
                   point loads in a trench coat?

Exit 0 if the fit is good enough to build on, 1 if not, 2 if a dependency is missing.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))


from wheelopt.cad.cli import (
    add_geometry_args,
    add_material_args,
    material_from_args,
    params_from_args,
)
from wheelopt.fea.loadcase import LoadCase, LoadCaseKind, MeshSpec, SolverSpec
from wheelopt.rom.fit import contact_segments, fit_spring_law, fit_tabulated_law
from wheelopt.rom.ring import TabulatedLaw, ring_for_design, ring_force_n

TINY = {"radius": 60.0, "width": 30.0, "spokes": 6, "thickness": 5.0,
        "rim_thickness": 3.0, "hub_radius": 20.0}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_geometry_args(p)
    add_material_args(p)
    p.add_argument("--tiny", action="store_true", help="the debug design, matching run_fea")
    p.add_argument("--segments", type=int, action="append",
                   help="repeatable; defaults to a 12/24/36/48 sweep")
    p.add_argument("--case", choices=("flat", "step_edge"), default="flat")
    p.add_argument("--delta-max", type=float, default=0.006)
    p.add_argument("--n-points", type=int, default=6)
    p.add_argument("--friction", type=float, default=LoadCase().friction_mu)
    p.add_argument("--plane-strain", action="store_true",
                   help="fit against the 2-D screening tier instead of the 3-D solid")
    p.add_argument("--law", choices=("cubic", "table", "both"), default="both",
                   help="spring law family. 'both' fits each and prints them side by side, "
                        "which is the comparison that matters on a buckling curve")
    p.add_argument("--intervals", type=int, default=None,
                   help="table resolution; default is one interval per two data points")
    p.add_argument("--monotone", action="store_true",
                   help="forbid a softening branch in the table. Off by default: a buckling "
                        "spoke softens, and requiring monotonicity is what kept the nominal "
                        "design unfittable")
    p.add_argument("--smoothing", type=float, default=0.1,
                   help="penalty on tangent changes between table intervals; 0 disables")
    p.add_argument("--mujoco", action="store_true",
                   help="also build the MuJoCo ring and press it into the floor")
    p.add_argument("--no-coupling", action="store_true",
                   help="force the band's bending stiffness to zero — the pre-coupling "
                        "model, kept as an A/B against it rather than as an option")
    p.add_argument("--cache", type=Path, default=REPO_ROOT / "data" / "cache" / "fea")
    p.add_argument("--threads", type=int, default=4)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.tiny:
        for key, value in TINY.items():
            if getattr(args, key) == build_parser().get_default(key):
                setattr(args, key, value)

    params = params_from_args(args)
    material = material_from_args(args)
    segments = args.segments or [12, 24, 36, 48]

    # One contact penalty for both tiers since #12 (2026-08-09): the plane-strain tier used
    # to need a hand-softened factor of 5 where the 3-D tier ran at the default 20, and 5 is
    # now the default because it costs 0.7-0.8% of the answer on the 3-D tier and buys the
    # conditioning outright. Only the mesh differs.
    mesh = (MeshSpec(dimension=2, size_spoke_m=0.0025, size_rim_m=0.003, size_hub_m=0.002)
            if args.plane_strain
            else MeshSpec(size_spoke_m=0.008, size_rim_m=0.010, size_hub_m=0.010))

    from wheelopt.fea.runner import run_load_case

    kind = LoadCaseKind.RADIAL_FLAT if args.case == "flat" else LoadCaseKind.RADIAL_STEP_EDGE
    case = LoadCase(kind=kind, delta_max_m=args.delta_max,
                    n_points_per_branch=args.n_points, friction_mu=args.friction)
    solver = SolverSpec(n_threads=args.threads)

    print(f"FEA: {'plane strain' if args.plane_strain else '3-D solid'}, {args.case}, "
          f"mu={args.friction}")
    result = run_load_case(params, material, case, mesh_spec=mesh, solver=solver,
                           cache_root=args.cache)
    if not result.ok:
        print(f"  {result.status.value}: {result.message}")
        return 2 if result.is_environment_failure else 1

    curve = result.curve
    load = curve.loading
    delta, force = curve.delta_m[load], curve.force_n[load]
    print(f"  peak {curve.peak_force_n:.3f} N at {delta.max() * 1e3:.1f} mm, "
          f"{len(delta)} loading points")
    if result.patch is not None:
        print(f"  contact patch at peak: {result.patch.length_m[-1] * 1e3:.1f} mm")

    families = ("cubic", "table") if args.law == "both" else (args.law,)
    print(f"\n{'N':>4} {'law':>6} {'k_bend':>8} {'k_hoop':>8} {'k(0) N/mm':>10} "
          f"{'RMS N':>8} {'RMS %':>7} {'max N':>7} {'in contact':>11} {'ok':>5}")
    best = None
    for n in segments:
        # Coupling is derived from the band's geometry and modulus, never chosen: it scales
        # with dtheta, so it is a different number at every segment count and the sweep below
        # is a sweep over discretisation, not over stiffness.
        spec = ring_for_design(params, material, n_segments=n)
        if args.no_coupling:
            spec = replace(spec, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0)
        for family in families:
            fit = (fit_spring_law(spec, delta, force) if family == "cubic"
                   else fit_tabulated_law(spec, delta, force, n_intervals=args.intervals,
                                          monotone=args.monotone, smoothing=args.smoothing))
            n_contact = contact_segments(spec, float(delta.max()), fit.law)
            # A ring whose contact is one or two segments can still fit the curve beautifully
            # — measured: 12 segments gives 1.33% RMS against 24 segments' 2.91%, with a
            # *single* segment carrying load. It is fitting a point load to a 34 mm patch.
            # Fit error alone cannot see this, so the count is flagged in the same row and
            # rings that fail it are excluded from `best` rather than winning on error.
            usable = fit.ok and n_contact >= 3
            flag = "" if n_contact >= 3 else "  <- point load, not a patch"
            if not fit.converged:
                flag += "  <- fit did not converge"
            print(f"{n:4d} {family:>6} {spec.band_bending_n_per_m:8.3f} "
                  f"{spec.band_hoop_n_per_m:8.1f} "
                  f"{float(fit.law.stiffness_n_per_m(0.0)) * 1e-3:10.2f} "
                  f"{fit.rms_error_n:8.4f} {fit.rms_error_fraction:7.2%} "
                  f"{fit.max_error_n:7.4f} {n_contact:11d} {usable!s:>5}{flag}")
            if usable and (best is None or fit.rms_error_fraction < best.rms_error_fraction):
                best = fit

    if best is None:  # pragma: no cover - segments is never empty
        return 1

    # The discretisation check the plan asks for, against the measured patch rather than a
    # rule of thumb: a ring whose contact spans two segments is not resolving a patch.
    n_contact = contact_segments(best.spec, float(delta.max()), best.law)
    if result.patch is not None and result.patch.length_m[-1] > 0:
        arc = best.spec.segment_arc_m
        expected = result.patch.length_m[-1] / arc
        print(f"\npatch spans {expected:.1f} segment arcs at {best.spec.n_segments} segments; "
              f"{n_contact} carry load")
    if n_contact < 3:
        print("  WARNING: fewer than three segments in contact — this ring is modelling "
              "point loads, not a patch, whatever the fit error says")

    print(f"\nbest fit: {best.summary()}")
    if isinstance(best.law, TabulatedLaw):
        print(f"  {best.law.summary()}")

    if args.mujoco:
        try:
            from wheelopt.rom.mjcf import MissingMuJoCo, static_load_deflection
        except ImportError:
            print("\nMuJoCo is not installed; pip install -e '.[sim]'")
            return 2
        try:
            simulated = static_load_deflection(best.spec, best.law, delta)
        except MissingMuJoCo as exc:
            print(f"\n{exc}")
            return 2
        analytic = ring_force_n(best.spec, best.law, delta)
        print(f"\nMuJoCo ring vs the analytic ring it was fitted as "
              f"({best.spec.n_segments} segments):")
        print(f"{'delta mm':>9} {'FEA N':>8} {'ring N':>8} {'mujoco N':>9} {'mj-ring':>9}")
        for d, f_fea, f_ring, f_mj in zip(delta, force, analytic, simulated):
            rel = (f_mj - f_ring) / f_ring if f_ring else float("nan")
            print(f"{d * 1e3:9.1f} {f_fea:8.3f} {f_ring:8.3f} {f_mj:9.3f} {rel:9.2%}")

    return 0 if best.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
