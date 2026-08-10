#!/usr/bin/env python3
"""Render the step-climb runs, so the rig can be looked at rather than only measured.

    python scripts/render_step.py --tiny              # GIF + contact sheet, both wheels
    python scripts/render_step.py --tiny --height 60  # a step neither wheel clears

Every check on `wheelopt.sim.step_climb` so far has been numeric — contact forces, patch
spans, mass matrices — and a rig can pass all of those while looking obviously wrong on
screen. Two of the three bugs found in that rig by other means (the axle turning backwards,
the wheel accelerating off to 41 m) would have been apparent in one second of video. This is
the differently-shaped check.

Writes to `data/renders/` by default: an animated GIF per wheel, and one contact sheet holding
both wheels at the same instants, compliant above rigid, so the pair can be compared frame for
frame. Pillow only — no imageio, no ffmpeg.
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

TINY = {"radius": 60.0, "width": 30.0, "spokes": 6, "thickness": 5.0,
        "rim_thickness": 3.0, "hub_radius": 20.0}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_geometry_args(p)
    add_material_args(p)
    p.add_argument("--tiny", action="store_true")
    p.add_argument("--segments", type=int, default=24)
    p.add_argument("--delta-max", type=float, default=0.006)
    p.add_argument("--n-points", type=int, default=6)
    p.add_argument("--plane-strain", action="store_true")
    p.add_argument("--payload", type=float, default=None)
    p.add_argument("--height", type=float, default=None, help="step height in mm")
    p.add_argument("--law", choices=("cubic", "table"), default="cubic",
                   help="spring law family, as in run_step.py. The claw designs need 'table'")
    p.add_argument("--tangential", nargs="?", const="hinge", default=None,
                   choices=("hinge", "slide"),
                   help="give every claw its second in-plane freedom, as in run_step.py. "
                        "Bandless rings only")
    p.add_argument("--tangential-max", type=float, default=None, metavar="M")
    p.add_argument("--duration", type=float, default=3.0, help="seconds to render")
    p.add_argument("--fps", type=int, default=25)
    p.add_argument("--pixels", type=int, default=640,
                   help="frame width; --width is the wheel's, not the image's")
    p.add_argument("--sheet-frames", type=int, default=6)
    p.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "renders")
    p.add_argument("--cache", type=Path, default=REPO_ROOT / "data" / "cache" / "fea")
    p.add_argument("--threads", type=int, default=4)
    return p


def _fit_the_ring(args):
    params = params_from_args(args)
    material = material_from_args(args)
    # One contact penalty for both tiers since #12 (2026-08-09): the plane-strain tier used
    # to need a hand-softened factor of 5 where the 3-D tier ran at the default 20, and 5 is
    # now the default because it costs 0.7-0.8% of the answer on the 3-D tier and buys the
    # conditioning outright. Only the mesh differs.
    mesh = (MeshSpec(dimension=2, size_spoke_m=0.0025, size_rim_m=0.003, size_hub_m=0.002)
            if args.plane_strain
            else MeshSpec(size_spoke_m=0.008, size_rim_m=0.010, size_hub_m=0.010))

    from wheelopt.fea.runner import run_load_case

    case = LoadCase(kind=LoadCaseKind.RADIAL_FLAT, delta_max_m=args.delta_max,
                    n_points_per_branch=args.n_points)
    result = run_load_case(params, material, case, mesh_spec=mesh,
                           solver=SolverSpec(n_threads=args.threads),
                           cache_root=args.cache)
    if not result.ok:
        return None, f"{result.status.value}: {result.message}"
    loading = result.curve.loading
    spec = ring_for_design(params, material, n_segments=args.segments)
    fitter = fit_spring_law if args.law == "cubic" else fit_tabulated_law
    return (spec, fitter(spec, result.curve.delta_m[loading],
                                 result.curve.force_n[loading])), None


def _render_run(spec, law, rig, *, rigid, fps, pixels, duration_s, fit_max_m,
                tangential_law=None, tangential_element=None):
    """Film one run. Returns ``(frames, times, axle_x, result)``.

    The physics is **not** here. `observe_step` runs the same loop `run_step` measures and
    hands back the live state after every integrator step; this only decides which of those
    steps become pictures. It had its own loop until 2026-08-09 and had drifted off the rig it
    was supposed to be showing — no stable timestep, and the loss-factor damping still pushed
    through `qfrc_applied` where #27 moved it to native joint damping. The frames were of a
    simulation nobody measured, which is the one thing a renderer must not be.
    """
    import mujoco

    from wheelopt.sim.step_climb import observe_step

    height = int(pixels * 9 / 16)
    renderer = None
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth, camera.elevation = 90.0, -12.0
    camera.distance = 9.0 * spec.radius_m
    # Framed on the step, not on the wheel. A camera locked to the axle keeps the wheel dead
    # centre in every frame, which is exactly the information being looked for: whether it got
    # anywhere. The first render of this was six identical pictures of a wheel. The view pans
    # only enough to keep an approaching wheel on screen, and stops at the obstacle.
    focus_x = rig.step_x_m - 1.2 * spec.radius_m

    # The step is chosen by the rig, not by this script: a stiff law tightens it, so the
    # frame interval has to be derived after the fact rather than from `rig.timestep_s`.
    frames, times, positions = [], [], []
    state = {"every": None, "carriage": None}

    def observe(k, model, data):
        nonlocal renderer
        if renderer is None:
            renderer = mujoco.Renderer(model, height=height, width=pixels)
            state["carriage"] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "carriage")
            state["every"] = max(1, round(1.0 / (fps * model.opt.timestep)))
        if k % state["every"] or data.time > duration_s:
            return
        axle_x = float(data.xpos[state["carriage"], 0])
        camera.lookat[:] = (min(axle_x + 1.2 * spec.radius_m, focus_x) if axle_x < focus_x
                            else max(focus_x, axle_x - 0.5 * spec.radius_m),
                            0.0,
                            rig.step_height_m * 0.5 + spec.radius_m * 0.5)
        renderer.update_scene(data, camera)
        frames.append(renderer.render().copy())
        times.append(float(data.time))
        positions.append(axle_x)

    # `fit_max_m` is not optional here even though `observe_step` defaults it to infinity:
    # against infinity `fraction_beyond_fit` is 0% for every run, which reads as "well inside
    # the fit" and means "not measured". It printed exactly that -- 0% where run_step says
    # 14% -- for as long as it took to notice.
    result = observe_step(spec, law, rig, observe, rigid=rigid, fit_max_m=fit_max_m,
                          tangential_law=tangential_law,
                          tangential_element=tangential_element)
    if renderer is not None:
        renderer.close()
    return frames, times, positions, result


def _label(image, text):
    """Burn a caption into the top-left. Rendered frames carry no context on their own."""
    from PIL import Image, ImageDraw

    picture = Image.fromarray(image)
    draw = ImageDraw.Draw(picture)
    draw.rectangle([0, 0, 8 + 7 * len(text), 20], fill=(0, 0, 0))
    draw.text((5, 5), text, fill=(255, 255, 255))
    return picture


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

    try:
        import mujoco  # noqa: F401
        from PIL import Image
    except ImportError as exc:
        print(f"rendering needs mujoco and Pillow: {exc}")
        return 2

    from wheelopt.sim.step_climb import RigSpec

    fit_max = float(np.max(fit.delta_m))
    static_load = float(solve_equilibrium(spec, fit.law, 0.5 * fit_max).force_n)
    payload = args.payload if args.payload is not None else static_load / 9.81
    height = (args.height * 1e-3 if args.height is not None
              else round(0.6 * spec.radius_m, 3))
    # The rig's own duration, so the film ends where the measured run ends rather than at a
    # separate number that could quietly disagree with it.
    rig = RigSpec(payload_kg=payload, step_height_m=height, duration_s=args.duration)

    # The tangential law comes from run_step's helper, not a copy of it: it is the same sweep,
    # the same claw sector and the same change of coordinates, and two of those would be two
    # chances to film a claw the measured run does not have.
    tangential_law = None
    if args.tangential:
        if spec.is_coupled:
            print("--tangential needs a bandless ring; use --rim-thickness 0")
            return 1
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from run_step import _measure_tangential_law

        tangential_law, _kinematics, message = _measure_tangential_law(args, spec)
        if tangential_law is None:
            print(f"tangential sweep failed: {message}")
            return 1

    print(f"ring {spec.n_segments} segments, R {spec.radius_m * 1e3:.0f} mm, "
          f"fit {fit.rms_error_fraction:.2%}")
    print(f"rig  {payload:.3f} kg, {height * 1e3:.0f} mm step ({height / spec.radius_m:.2f} R)"
          f", {args.duration} s at {args.fps} fps")

    args.out.mkdir(parents=True, exist_ok=True)
    rendered = {}
    for name, rigid in (("compliant", False), ("rigid", True)):
        frames, times, positions, result = _render_run(
            spec, fit.law, rig, rigid=rigid, fps=args.fps, pixels=args.pixels,
            duration_s=args.duration, fit_max_m=fit_max,
            tangential_law=tangential_law, tangential_element=args.tangential,
        )
        if not result.ok:
            print(f"  {name}: {result.message}")
            return 1
        rendered[name] = (frames, times, positions)
        verdict = "CLIMBED" if result.climbed else "did not climb"
        print(f"  {name:<10} {verdict}, {result.fraction_beyond_fit:.0%} of loaded samples "
              f"past the fitted range")
        pictures = [_label(frame, f"{name}  t={t:.2f}s  x={x * 1e3:.0f}mm")
                    for frame, t, x in zip(frames, times, positions)]
        path = args.out / f"step_{name}_{height * 1e3:.0f}mm.gif"
        pictures[0].save(path, save_all=True, append_images=pictures[1:],
                         duration=int(1000 / args.fps), loop=0)
        print(f"  {path}  ({len(pictures)} frames)")

    # One sheet, both wheels, same instants: the comparison is the point, and flipping
    # between two GIFs is a poor way to see a difference in where the wheel ended up.
    n = min(len(rendered["compliant"][0]), len(rendered["rigid"][0]))
    picks = np.linspace(0, n - 1, args.sheet_frames).round().astype(int)
    tile_h, tile_w = rendered["compliant"][0][0].shape[:2]
    sheet = Image.new("RGB", (tile_w * len(picks), tile_h * 2))
    for column, index in enumerate(picks):
        for row, name in enumerate(("compliant", "rigid")):
            frames, times, positions = rendered[name]
            tile = _label(frames[index],
                          f"{name} t={times[index]:.2f}s x={positions[index] * 1e3:.0f}mm")
            sheet.paste(tile, (column * tile_w, row * tile_h))
    sheet_path = args.out / f"step_compare_{height * 1e3:.0f}mm.png"
    sheet.save(sheet_path)
    print(f"  {sheet_path}")

    for name in ("compliant", "rigid"):
        travel = rendered[name][2]
        print(f"{name:<10} travelled {(travel[-1] - travel[0]) * 1e3:7.1f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
