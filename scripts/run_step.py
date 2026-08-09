#!/usr/bin/env python3
"""First-week steps 5-6: drive the fitted ring at a step, beside a rigid wheel.

    python scripts/run_step.py --tiny                    # the four signatures
    python scripts/run_step.py --tiny --sweep            # + tallest step each wheel climbs

Step 5 is the run. Step 6 is the four questions in `docs/plan/16-first-week.md`, and this
script answers each one with a number and says which way the answer has to come out:

    envelopment     contact patch length at the step edge            compliant longer
    contact patch   mean patch length on the flat                    compliant longer
    climbs better   tallest step cleared at the same torque          compliant higher
    rolls worse     cost of transport on the flat                    compliant higher
    loaded radius   axle height against load                         must decrease

Exit 0 if every signature came out the way physics requires, 1 if any did not, 2 if a
dependency is missing. A 1 is not a bug in this script — it is the spike's answer, and
`16-first-week.md` says what to do about it.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))


from wheelopt.cad.cli import (
    add_geometry_args,
    add_material_args,
    material_from_args,
    params_from_args,
)
from wheelopt.fea.loadcase import LoadCase, LoadCaseKind, MeshSpec, SolverSpec
from wheelopt.rom.fit import fit_spring_law, fit_tabulated_law
from wheelopt.rom.ring import ring_for_design, solve_equilibrium
from wheelopt.sim.step_climb import judge_signatures, loaded_radius_table

TINY = {"radius": 60.0, "width": 30.0, "spokes": 6, "thickness": 5.0,
        "rim_thickness": 3.0, "hub_radius": 20.0}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_geometry_args(p)
    add_material_args(p)
    p.add_argument("--tiny", action="store_true", help="the debug design, matching run_rom")
    p.add_argument("--segments", type=int, default=24)
    p.add_argument("--delta-max", type=float, default=0.006)
    p.add_argument("--n-points", type=int, default=6)
    p.add_argument("--plane-strain", action="store_true")
    p.add_argument("--law", choices=("cubic", "table"), default="cubic",
                   help="spring law family. The table can represent a softening segment, "
                        "which the cubic cannot; see wheelopt.rom.ring.TabulatedLaw")
    p.add_argument("--payload", type=float, default=None,
                   help="kg on the axle; default puts the wheel at half its fitted range")
    p.add_argument("--step-height", type=float, default=None,
                   help="m; default is 0.05 for a nominal wheel, else 0.6 x radius")
    p.add_argument("--sweep", action="store_true",
                   help="also find the tallest step each wheel clears (slow: ~10 runs each)")
    p.add_argument("--cache", type=Path, default=REPO_ROOT / "data" / "cache" / "fea")
    p.add_argument("--threads", type=int, default=4)
    return p


def _fit_the_ring(args):
    """FEA -> ring fit. Returns (spec, fit) or (None, message)."""
    params = params_from_args(args)
    material = material_from_args(args)
    if args.plane_strain:
        mesh = MeshSpec(dimension=2, size_spoke_m=0.0025, size_rim_m=0.003, size_hub_m=0.002)
        factor = 5.0
    else:
        mesh = MeshSpec(size_spoke_m=0.008, size_rim_m=0.010, size_hub_m=0.010)
        factor = SolverSpec().contact_stiffness_factor

    from wheelopt.fea.runner import run_load_case

    case = LoadCase(kind=LoadCaseKind.RADIAL_FLAT, delta_max_m=args.delta_max,
                    n_points_per_branch=args.n_points)
    result = run_load_case(params, material, case, mesh_spec=mesh,
                           solver=SolverSpec(n_threads=args.threads,
                                             contact_stiffness_factor=factor),
                           cache_root=args.cache)
    if not result.ok:
        return None, f"{result.status.value}: {result.message}"

    loading = result.curve.loading
    delta = result.curve.delta_m[loading]
    force = result.curve.force_n[loading]
    spec = ring_for_design(params, material, n_segments=args.segments)
    fit = (fit_spring_law(spec, delta, force) if args.law == "cubic"
           else fit_tabulated_law(spec, delta, force))
    return (spec, fit), None


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.tiny:
        for key, value in TINY.items():
            if getattr(args, key) == build_parser().get_default(key):
                setattr(args, key, value)

    built, message = _fit_the_ring(args)
    if built is None:
        print(message)
        return 2 if "solver_missing" in message else 1
    spec, fit = built
    print(f"ring: {spec.n_segments} segments, R = {spec.radius_m * 1e3:.0f} mm, "
          f"band {spec.band_bending_n_per_m:.3f} / {spec.band_hoop_n_per_m:.1f} N/m")
    caveat = "" if fit.ok else "   <- NOT OK; read every number below as provisional"
    print(f"fit:  {fit.summary()}{caveat}")

    fit_max = float(np.max(fit.delta_m))
    # Sit the wheel at half its fitted indentation. Loading it to the platform's 24.5 N when
    # the fit only reaches a few newtons would make every number below an extrapolation, and
    # the run would be answering a question about the cubic rather than about compliance.
    design_delta = 0.5 * fit_max
    static_load = float(solve_equilibrium(spec, fit.law, design_delta).force_n)
    payload = args.payload if args.payload is not None else static_load / 9.81
    height = (args.step_height if args.step_height is not None
              else round(0.6 * spec.radius_m, 3))

    from wheelopt.sim.step_climb import RigSpec, highest_step_climbed, run_flat, run_step

    rig = RigSpec(payload_kg=payload, step_height_m=height)
    print(f"rig:  {payload:.3f} kg ({static_load:.2f} N) on the axle, "
          f"{rig.stall_torque_n_m(spec.radius_m):.3f} N·m stall, "
          f"{rig.no_load_speed_m_s} m/s free, "
          f"{height * 1e3:.0f} mm step ({height / spec.radius_m:.2f} R), "
          f"loss factor {rig.loss_factor}")

    runs = {}
    for name, rigid in (("compliant", False), ("rigid", True)):
        runs[name] = {
            "flat": run_flat(spec, fit.law, rig, rigid=rigid, fit_max_m=fit_max),
            "step": run_step(spec, fit.law, rig, rigid=rigid, fit_max_m=fit_max),
        }
        for phase, result in runs[name].items():
            if not result.ok:
                print(f"\n{name} {phase}: {result.message}")
                return 2 if "MuJoCo is not installed" in result.message else 1

    compliant, rigid_ = runs["compliant"], runs["rigid"]
    # The five signatures live in `wheelopt.sim.step_climb`, not here: `scripts/explore.py`
    # reports the same set, and two copies of a judgement is how one report comes to pass
    # while another fails on the same run.
    signatures = judge_signatures(
        spec, fit.law,
        compliant_flat=compliant["flat"], compliant_step=compliant["step"],
        rigid_flat=rigid_["flat"], rigid_step=rigid_["step"],
        step_height_m=height, static_load_n=static_load,
    )
    print(f"\n{'signature':<34} {'compliant':>12} {'rigid':>12}  verdict")
    for sig in signatures:
        print(f"{sig.name:<34} {sig.compliant:>12} {sig.rigid:>12}  "
              f"{'PASS' if sig.passed else 'FAIL'}")
    verdicts = [sig.passed for sig in signatures]

    radii = loaded_radius_table(
        spec, fit.law,
        [0.25 * static_load, 0.5 * static_load, static_load, 2.0 * static_load],
    )
    table = ", ".join(f"{f:.2f}->{r * 1e3:.2f}" for f, r in radii)
    print(f"\nload N -> loaded radius mm: {table}")
    print(f"peak segment compression on the step: "
          f"{compliant['step'].peak_compression_m * 1e3:.2f} mm "
          f"(fitted to {fit_max * 1e3:.1f} mm); "
          f"{compliant['step'].fraction_beyond_fit:.0%} of loaded samples beyond it")

    if args.sweep:
        print("\ntallest step cleared (sweep, 10 mm resolution):")
        for name, rigid in (("compliant", False), ("rigid", True)):
            tallest = highest_step_climbed(spec, fit.law, rig, rigid=rigid, fit_max_m=fit_max)
            print(f"  {name:<10} {tallest * 1e3:5.0f} mm  ({tallest / spec.radius_m:.2f} R)")

    passed = sum(verdicts)
    print(f"\n{passed}/{len(verdicts)} signatures came out the way physics requires.")
    if passed < len(verdicts):
        print("Read docs/plan/16-first-week.md 'Decision' before concluding anything: a "
              "failed signature may be the model, the rig, or the design.")
    return 0 if passed == len(verdicts) else 1


if __name__ == "__main__":
    raise SystemExit(main())
