#!/usr/bin/env python3
"""Scenario S1 — drive the step ladder, fit the success curve, store every run.

    python scripts/run_s1.py                          # the default 10 x 8 ladder
    python scripts/run_s1.py --heights 20:120:20      # a narrower ladder
    python scripts/run_s1.py --repeat 2 --gate        # the Phase 0 determinism gate

This is the first scenario end to end: `configs/robot.yaml`'s robot, a constant-throttle
controller, a ladder of step heights across terrain seeds, and the height at P(success) = 0.9
as a continuous metric with an error bar rather than a bisected threshold.

Every run becomes a row under `data/experiments`. That is not bookkeeping — it is what makes
the determinism gate a query. `--gate` runs the whole ladder twice and asks the store whether
any `run_id` came back with two different answers.

Exit 0 if the fit is usable (and, with `--gate`, if nothing disagreed); 1 if not; 2 if a
dependency is missing.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from wheelopt.platform import PlatformSpecError, load_platform
from wheelopt.sim.s1_step import S1Config, run_s1, terrain_for_seed
from wheelopt.store import ExperimentStore, StoreError


def heights_from(text: str) -> tuple[float, ...]:
    """``start:stop:step`` in millimetres, inclusive of ``stop``."""
    try:
        start, stop, step = (float(x) for x in text.split(":"))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"--heights wants start:stop:step in mm, e.g. 20:200:20; got {text!r}"
        ) from exc
    if step <= 0 or stop < start:
        raise argparse.ArgumentTypeError("--heights needs a positive step and stop >= start")
    return tuple(np.round(np.arange(start, stop + 1e-9, step) * 1e-3, 4))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--config", type=Path, default=None, help="platform YAML")
    p.add_argument("--heights", type=heights_from, default=None,
                   help="ladder as start:stop:step in mm (default 20:200:20)")
    p.add_argument("--seeds", type=int, default=8, help="terrain seeds; 08-metrics wants >= 8")
    p.add_argument("--radius", type=float, default=85.0, help="wheel radius, mm")
    p.add_argument("--width", type=float, default=30.0, help="wheel width, mm")
    p.add_argument("--wheel-mass", type=float, default=300.0, help="per wheel, grams")
    p.add_argument("--throttle", type=float, default=1.0,
                   help="fraction of the platform's stall torque at all four axles")
    p.add_argument("--duration", type=float, default=6.0, help="seconds per run")
    p.add_argument("--friction", type=float, nargs=2, default=(0.3, 1.0),
                   metavar=("LO", "HI"),
                   help="edge-friction range, sampled log-uniformly per terrain seed. Log rather "
                        "than uniform because 0.3 vs 0.4 matters far more to a climb than 0.9 vs 1.0")
    p.add_argument("--approach", type=float, default=15.0, help="max yaw, degrees")
    p.add_argument("--design-hash", default=None,
                   help="what the rows are about; defaults to a label built from the wheel")
    p.add_argument("--realisation", type=int, default=0, help="material realisation index")
    p.add_argument("--repeat", type=int, default=1,
                   help="run the whole ladder this many times; identical by construction")
    p.add_argument("--gate", action="store_true",
                   help="check the store for repeated run_ids that disagree (needs --repeat 2)")
    p.add_argument("--store", type=Path, default=REPO_ROOT / "data" / "experiments",
                   help="experiment store root; every rung of every seed becomes a row")
    p.add_argument("--no-store", action="store_true", help="run without writing any rows")
    p.add_argument("--manifest-out", type=Path, default=None, metavar="JSON",
                   help="write run_id -> metrics for this ladder to a JSON file. The "
                        "cross-machine half of the determinism gate: commit the file, and a "
                        "second machine runs the same ladder with --manifest against it")
    p.add_argument("--manifest", type=Path, default=None, metavar="JSON",
                   help="compare this ladder against a reference manifest written elsewhere "
                        "with --manifest-out. Exit 1 on any disagreement — a metric, a "
                        "status, a missing run or a version skew")
    p.add_argument("--tolerance", choices=("exact", "cross-machine"), default="exact",
                   help="how --manifest compares numbers. 'exact' is bit-identical — the "
                        "same-machine gate, where anything less is a bug. 'cross-machine' "
                        "allows the measured floating-point drift between platforms "
                        "(store.CROSS_MACHINE_RTOL: 0.5%% default, 20%% on energy_j — the "
                        "gate's first x86-64-vs-arm64 verdict, 2026-08-12, measured energy "
                        "drift of 3.6%% through contact-rich stalls). Verdicts, statuses "
                        "and run identity stay exact in both modes")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        platform = load_platform(args.config)
    except PlatformSpecError as exc:
        print(f"platform spec: {exc}")
        return 2

    config = S1Config(
        heights_m=args.heights if args.heights is not None else S1Config().heights_m,
        n_seeds=args.seeds, friction_range=tuple(args.friction),
        approach_deg=args.approach, duration_s=args.duration, throttle=args.throttle,
    )
    wheel = {"wheel_radius_m": args.radius * 1e-3, "wheel_width_m": args.width * 1e-3,
             "wheel_mass_kg": args.wheel_mass * 1e-3}
    # Width included since 2026-08-11: it changes the wheel's transverse inertia and so the
    # numbers, and a label that omits it lets two different wheels share every run_id.
    design_hash = (args.design_hash
                   or f"rigid-R{args.radius:.0f}-w{args.width:.0f}-m{args.wheel_mass:.0f}")

    if args.gate and args.repeat < 2:
        print("--gate needs --repeat 2 or more: an empty disagreement list proves nothing "
              "if nothing was ever run twice.")
        return 1

    print(f"robot: {platform.name}, wheels R {args.radius:.0f} mm rigid")
    print(f"ladder: {len(config.heights_m)} rungs "
          f"{config.heights_m[0] * 1e3:.0f}-{config.heights_m[-1] * 1e3:.0f} mm "
          f"x {config.n_seeds} seeds = {len(config.heights_m) * config.n_seeds} runs"
          f"{f' x {args.repeat}' if args.repeat > 1 else ''}")
    print(f"terrain: friction {config.friction_range[0]:.2f}-{config.friction_range[1]:.2f}, "
          f"approach +/-{config.approach_deg:.0f} deg")
    for seed in range(min(config.n_seeds, 4)):
        friction, approach = terrain_for_seed(seed, config)
        print(f"  seed {seed}: mu {friction:.3f}, {approach:+.1f} deg")
    if config.n_seeds > 4:
        print(f"  ... and {config.n_seeds - 4} more")

    store = None if args.no_store else ExperimentStore(args.store)
    outcome = None
    for attempt in range(args.repeat):
        outcome = run_s1(platform, config, design_hash=design_hash,
                         material_realisation=args.realisation, **wheel)
        if store is not None:
            store.append(outcome.records)
        if args.repeat > 1:
            print(f"\npass {attempt + 1}/{args.repeat}: {outcome.summary()}")

    assert outcome is not None
    print("\nrung    cleared")
    for height in config.heights_m:
        mask = outcome.heights_m == height
        cleared = int(outcome.successes[mask].sum()) if mask.any() else 0
        total = int(mask.sum())
        bar = "#" * cleared + "." * (total - cleared)
        print(f"  {height * 1e3:5.0f} mm  {cleared}/{total}  [{bar}]")
    if outcome.n_failed:
        print(f"  {outcome.n_failed} run(s) never produced a verdict; excluded from the fit")

    print(f"\nS1 step height at P=90%: {outcome.summary()}")
    if store is not None:
        print(f"  {len(store.files)} batch file(s) under {store.runs_dir}")

    if args.gate:
        try:
            disagreements = store.disagreements()
            repeats = store.repeat_counts()
        except StoreError as exc:
            print(f"\ngate: {exc}")
            return 1
        print(f"\ndeterminism gate: {len(repeats)} run_id(s) evaluated more than once")
        if not repeats:
            print("  NOT A PASS — nothing was repeated, so there was nothing to compare.")
            return 1
        if disagreements:
            print(f"  FAIL — {len(disagreements)} run_id(s) gave different metrics:")
            for run_id, variants, rows in disagreements[:10]:
                print(f"    {run_id}  {variants} distinct results over {rows} rows")
            return 1
        print("  PASS — every repeated evaluation returned identical metrics.")

    if args.manifest_out is not None or args.manifest is not None:
        import json

        from wheelopt.store import (
            CROSS_MACHINE_RTOL,
            compare_manifests,
            manifest_from_records,
        )

        manifest = manifest_from_records(outcome.records)
        if args.manifest_out is not None:
            args.manifest_out.parent.mkdir(parents=True, exist_ok=True)
            args.manifest_out.write_text(json.dumps(manifest, indent=1, sort_keys=True))
            print(f"\nmanifest: {len(manifest['rows'])} run(s) -> {args.manifest_out}")
        if args.manifest is not None:
            reference = json.loads(args.manifest.read_text())
            rtol = CROSS_MACHINE_RTOL if args.tolerance == "cross-machine" else None
            problems = compare_manifests(reference, manifest, rtol=rtol)
            print(f"\ncross-machine gate against {args.manifest.name}: "
                  f"{len(manifest['rows'])} run(s) compared, tolerance {args.tolerance}")
            if problems:
                print(f"  FAIL — {len(problems)} disagreement(s):")
                for line in problems[:20]:
                    print(f"    {line}")
                if len(problems) > 20:
                    print(f"    ... and {len(problems) - 20} more")
                return 1
            print("  PASS — " + ("bit-identical with the reference, on this machine."
                                 if rtol is None else
                                 "agrees with the reference within the measured "
                                 "cross-platform drift (verdicts and statuses exact)."))

    # A gate run is judged by the gate. The threshold fit on a deliberately small CI ladder
    # is honestly unusable — 3 rungs cannot locate P=0.9 — and failing the job for that would
    # make the determinism gate unrunnable at exactly the size a CI budget allows. Reaching
    # this line means every requested gate passed; a plain run keeps the fit as its verdict.
    # `--manifest-out` counts as a gate run too: writing the reference is the other half of
    # the same instrument, and exit 1 there broke `write && verify` chains on the CI ladder.
    if args.gate or args.manifest is not None or args.manifest_out is not None:
        if not outcome.ok:
            print(f"  (fit not usable on this ladder — {outcome.fit.reason or 'small ladder'}"
                  " — which is not what a gate run is judged by)")
        return 0

    return 0 if outcome.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
