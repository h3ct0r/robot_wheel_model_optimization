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
from wheelopt.fea.loadcase import LoadCase
from wheelopt.rom.build import build_ring, measure_tangential_law
from wheelopt.rom.ring import second_contact_delta_m, solve_equilibrium
from wheelopt.sim.step_climb import judge_signatures, loaded_radius_table

TINY = {"radius": 60.0, "width": 30.0, "spokes": 6, "thickness": 5.0,
        "rim_thickness": 3.0, "hub_radius": 20.0}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_geometry_args(p)
    add_material_args(p)
    p.add_argument("--tiny", action="store_true", help="the debug design, matching run_rom")
    p.add_argument("--segments", type=int, default=24,
                   help="ring resolution. Ignored by --law claw, where the segments are "
                        "the claws and the count is --spokes")
    p.add_argument("--delta-max", type=float, default=0.006, metavar="METRES",
                   help="how deep the FEA presses the wheel, METRES (not mm, unlike every "
                        "geometry flag)")
    p.add_argument("--n-points", type=int, default=6,
                   help="samples per branch of the FEA load sweep. Too few cannot resolve "
                        "a curve that peaks early -- 3 intervals on a claw give 10.4% RMS "
                        "where 4 give 1.7%")
    p.add_argument("--plane-strain", action="store_true",
                   help="use the 2-D screening FEA tier -- seconds instead of hours. The "
                        "3-D tier is the reference and cannot see lateral spoke buckling")
    p.add_argument("--law", choices=("cubic", "table", "claw"), default="cubic",
                   help="where the segment law comes from. 'cubic' and 'table' both "
                        "deconvolve the whole-wheel plate curve into segment laws; 'claw' "
                        "does no fit at all -- it measures ONE claw on the same plate and "
                        "uses that curve as the segment law directly, which for a bandless "
                        "wheel is what a segment is (TODO #18). Bandless only, and the "
                        "whole-wheel curve is then spent on validating it rather than "
                        "training it (TODO #29)")
    p.add_argument("--payload", type=float, default=None,
                   help="kg on the axle; default puts the wheel at half its fitted range")
    p.add_argument("--step-height", type=float, default=None,
                   help="m; default is 0.05 for a nominal wheel, else 0.6 x radius")
    p.add_argument("--tangential", nargs="?", const="hinge", default=None,
                   choices=("hinge", "slide"),
                   help="give every claw a second in-plane freedom (TODO #20), with its law "
                        "measured by a TIP_TANGENTIAL sweep on this design's own claw sector. "
                        "'hinge' rotates the claw about its root and is the right element "
                        "(TODO #27); 'slide' translates its tip and is kept only for "
                        "comparison -- it lengthens the claw as it splays. Bandless rings "
                        "only: the band tendons couple radial joints only, so a banded ring "
                        "would shear for free.")
    p.add_argument("--tangential-max", type=float, default=None, metavar="M",
                   help="how far the tangential sweep goes, metres. Must reach the "
                        "deflections the wheel actually sees: the claw stiffens 3.6x between "
                        "4 and 40 mm, so a short sweep extrapolates a straight line through a "
                        "curve that is anything but. Default is 90%% of the claw length -- "
                        "derived, because the hinge law cannot represent a tip that has "
                        "travelled further sideways than the claw is long, and a fixed "
                        "default in metres is one claw length on exactly one design")
    p.add_argument("--sweep", action="store_true",
                   help="also find the tallest step each wheel clears (slow: ~10 runs each)")
    p.add_argument("--cache", type=Path, default=REPO_ROOT / "data" / "cache" / "fea",
                   help="content-addressed FEA cache. A hit is seconds, a miss is minutes")
    p.add_argument("--threads", type=int, default=4,
                   help="CalculiX threads. Changes how long the answer takes, not what it "
                        "is, so it is excluded from the cache key")
    return p


def _fit_the_ring(args):
    """FEA -> ring. Returns (spec, fit) or (None, message).

    The chain itself lives in `wheelopt.rom.build` so that this script and `run_rover.py`
    cannot drift into building different rings from the same flags — the same argument that
    put the geometry flags in `cad/cli.py` and the digest in `hashing.py`.
    """
    built = build_ring(
        params_from_args(args), material_from_args(args),
        law=args.law, n_segments=args.segments, plane_strain=args.plane_strain,
        delta_max_m=args.delta_max, n_points=args.n_points,
        cache_root=args.cache, n_threads=args.threads,
    )
    return ((built.spec, built.fit), None) if built.ok else (None, built.message)



def _tangential_max_m(args, spec) -> float:
    """How far to sweep the claw tip. 90% of the claw length unless the caller said."""
    return (args.tangential_max if args.tangential_max is not None
            else 0.9 * spec.claw_length_m)


def _measure_tangential_law(args, spec):
    """The claw's own tangential curve, as a law for ``args.tangential``. See `rom.build`."""
    return measure_tangential_law(
        params_from_args(args), material_from_args(args), spec,
        element=args.tangential, sweep_max_m=_tangential_max_m(args, spec),
        cache_root=args.cache, n_threads=args.threads,
    )


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
    # iterations == 0 is `validate_ring`'s marker: nothing was fitted, so the error below is
    # against a curve the law never saw. Say which, because a 5% held-out error and a 5% fit
    # error are not the same claim and the summary line looks identical.
    kind = "held out" if fit.iterations == 0 else "fitted"
    print(f"fit:  [{kind}] {fit.summary()}{caveat}")
    if not spec.is_coupled:
        engage = second_contact_delta_m(spec)
        probe = np.linspace(1e-5, 0.6 * spec.radius_m, 2000)
        carried = np.array([solve_equilibrium(spec, fit.law, float(d)).force_n for d in probe])
        nominal = LoadCase().nominal_load_n
        # First crossing, walked by hand. `np.interp` wants an increasing table and a claw
        # curve is not one -- it peaks at ~2.4 mm and softens for the next ten, which is the
        # whole reason `is_monotone_nonneg` was dropped as a gate. Handed a falling table it
        # returns a number rather than an error, and that number is not a crossing.
        hit = np.flatnonzero(carried >= nominal)
        if hit.size == 0:
            where = f"beyond {probe[-1] * 1e3:.0f} mm"
        elif hit[0] == 0:
            where = f"under {probe[0] * 1e3:.2f} mm"
        else:
            k = int(hit[0])
            span = carried[k] - carried[k - 1]
            frac = (nominal - carried[k - 1]) / span if span > 0 else 0.0
            where = f"{(probe[k - 1] + frac * (probe[k] - probe[k - 1])) * 1e3:.2f} mm"
        print(f"      a second claw engages at R(1-cos 2pi/n) = {engage * 1e3:.2f} mm; "
              f"below that the wheel is one claw and above it the ROM is unvalidated.\n"
              f"      the platform's {nominal:.1f} N per wheel sits at {where}")

    fit_max = float(np.max(fit.delta_m))
    # Sit the wheel at half its fitted indentation. Loading it to the platform's 24.5 N when
    # the fit only reaches a few newtons would make every number below an extrapolation, and
    # the run would be answering a question about the cubic rather than about compliance.
    design_delta = 0.5 * fit_max
    static_load = float(solve_equilibrium(spec, fit.law, design_delta).force_n)
    payload = args.payload if args.payload is not None else static_load / 9.81
    height = (args.step_height if args.step_height is not None
              else round(0.6 * spec.radius_m, 3))

    from wheelopt.sim.step_climb import (
        RigSpec,
        run_flat,
        run_step,
        step_climb_profile,
    )

    tangential_law = None
    if args.tangential:
        if spec.is_coupled:
            print("--tangential needs a bandless ring; this one has a band "
                  f"({spec.band_bending_n_per_m:.3f} / {spec.band_hoop_n_per_m:.1f} N/m). "
                  "Use --rim-thickness 0 geometry, or drop the flag.")
            return 1
        tangential_law, kinematics, message = _measure_tangential_law(args, spec)
        if tangential_law is None:
            print(f"tangential sweep failed: {message}")
            return 1
        from wheelopt.rom.ring import TipEquivalentLaw

        # Report both laws in tip coordinates whichever element is in use, so the two runs are
        # comparable and the stiffening is the same number in both.
        at_tip = (TipEquivalentLaw(tangential_law, spec.claw_length_m)
                  if args.tangential == "hinge" else tangential_law)
        sweep_max = _tangential_max_m(args, spec)
        near = float(at_tip.stiffness_n_per_m(0.001))
        far = float(at_tip.stiffness_n_per_m(0.9 * sweep_max))
        print(f"tangential: {args.tangential} element, claw {spec.claw_length_m * 1e3:.1f} mm "
              f"long, law measured over 0-{sweep_max * 1e3:.1f} mm: "
              f"{near / 1e3:.4f} N/mm at the tip near the origin, {far / 1e3:.4f} N/mm at "
              f"{0.9 * sweep_max * 1e3:.1f} mm ({far / near:.1f}x stiffer); "
              f"radial is {fit.law.stiffness_n_per_m(0.0) / near:.0f}x the near value")
        if kinematics is not None:
            measured, predicted = kinematics
            print("  tip travel inward, FEA vs the hinge idealisation "
                  "(a slide would predict the negative of these):")
            for k in sorted({0, len(measured) // 2, len(measured) - 1}):
                print(f"    measured {measured[k] * 1e3:+7.3f} mm, "
                      f"hinge predicts {predicted[k] * 1e3:+7.3f} mm")

    rig = RigSpec(payload_kg=payload, step_height_m=height)
    print(f"rig:  {payload:.3f} kg ({static_load:.2f} N) on the axle, "
          f"{rig.stall_torque_n_m(spec.radius_m):.3f} N·m stall, "
          f"{rig.no_load_speed_m_s} m/s free, "
          f"{height * 1e3:.0f} mm step ({height / spec.radius_m:.2f} R), "
          f"loss factor {rig.loss_factor}")

    runs = {}
    for name, rigid in (("compliant", False), ("rigid", True)):
        runs[name] = {
            "flat": run_flat(spec, fit.law, rig, rigid=rigid, fit_max_m=fit_max,
                             tangential_law=tangential_law,
                             tangential_element=args.tangential),
            "step": run_step(spec, fit.law, rig, rigid=rigid, fit_max_m=fit_max,
                             tangential_law=tangential_law,
                             tangential_element=args.tangential),
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
        print("\ntallest step cleared (sweep, 10 mm resolution; # cleared, . did not, "
              "E run failed):")
        for name, rigid in (("compliant", False), ("rigid", True)):
            profile = step_climb_profile(spec, fit.law, rig, rigid=rigid,
                                         fit_max_m=fit_max,
                                         tangential_law=tangential_law,
                                         tangential_element=args.tangential)
            print(f"  {name:<10} {profile.summary()} "
                  f"({profile.tallest_m / spec.radius_m:.2f} R)")
        print("  10 mm buckets, and a 1% change in the fitted law moves the answer one "
              "bucket; read it as a bucket, not a millimetre.")

    passed = sum(verdicts)
    print(f"\n{passed}/{len(verdicts)} signatures came out the way physics requires.")
    if passed < len(verdicts):
        print("Read docs/plan/16-first-week.md 'Decision' before concluding anything: a "
              "failed signature may be the model, the rig, or the design.")
    return 0 if passed == len(verdicts) else 1


if __name__ == "__main__":
    raise SystemExit(main())
