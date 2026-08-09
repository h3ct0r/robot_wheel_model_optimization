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
from wheelopt.rom.fit import fit_spring_law
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
    return (spec, fit_spring_law(spec, result.curve.delta_m[loading],
                                 result.curve.force_n[loading])), None


def _render_run(spec, law, rig, *, rigid, fps, pixels, duration_s):
    """Integrate and grab a frame every 1/fps. Returns (frames, times, axle_x)."""
    import mujoco

    from wheelopt.sim.step_climb import build_scenario_mjcf, segment_damping_n_s_per_m

    model = mujoco.MjModel.from_xml_string(build_scenario_mjcf(spec, rig, rigid=rigid))
    data = mujoco.MjData(model)
    damping = 0.0 if rigid else segment_damping_n_s_per_m(
        law, spec, rig.payload_kg, rig.loss_factor
    )
    segment_qpos = np.array([], dtype=np.int64)
    segment_dofs = np.array([], dtype=np.int64)
    if not rigid:
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"j{i}")
               for i in range(spec.n_segments)]
        segment_qpos = model.jnt_qposadr[ids]
        segment_dofs = model.jnt_dofadr[ids]
    axle_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "axle")]
    carriage = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "carriage")

    height = int(pixels * 9 / 16)
    renderer = mujoco.Renderer(model, height=height, width=pixels)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth, camera.elevation = 90.0, -12.0
    camera.distance = 9.0 * spec.radius_m
    # Framed on the step, not on the wheel. A camera locked to the axle keeps the wheel dead
    # centre in every frame, which is exactly the information being looked for: whether it got
    # anywhere. The first render of this was six identical pictures of a wheel. The view pans
    # only enough to keep an approaching wheel on screen, and stops at the obstacle.
    focus_x = rig.step_x_m - 1.2 * spec.radius_m

    settle_steps = int(0.6 / rig.timestep_s)
    n_steps = int(duration_s / rig.timestep_s)
    every = max(1, round(1.0 / (fps * rig.timestep_s)))
    frames, times, positions = [], [], []

    for k in range(n_steps):
        if not rigid:
            compression = -data.qpos[segment_qpos]
            data.qfrc_applied[segment_dofs] = (
                law.force_n(compression) - damping * data.qvel[segment_dofs]
            )
        data.ctrl[0] = (0.0 if k < settle_steps else
                        rig.motor_torque_n_m(spec.radius_m, float(data.qvel[axle_dof])))
        mujoco.mj_step(model, data)
        if k % every:
            continue
        axle_x = float(data.xpos[carriage, 0])
        camera.lookat[:] = (min(axle_x + 1.2 * spec.radius_m, focus_x) if axle_x < focus_x
                            else max(focus_x, axle_x - 0.5 * spec.radius_m),
                            0.0,
                            rig.step_height_m * 0.5 + spec.radius_m * 0.5)
        renderer.update_scene(data, camera)
        frames.append(renderer.render().copy())
        times.append(float(data.time))
        positions.append(float(data.xpos[carriage, 0]))

    renderer.close()
    return frames, times, positions


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
    rig = RigSpec(payload_kg=payload, step_height_m=height)

    print(f"ring {spec.n_segments} segments, R {spec.radius_m * 1e3:.0f} mm, "
          f"fit {fit.rms_error_fraction:.2%}")
    print(f"rig  {payload:.3f} kg, {height * 1e3:.0f} mm step ({height / spec.radius_m:.2f} R)"
          f", {args.duration} s at {args.fps} fps")

    args.out.mkdir(parents=True, exist_ok=True)
    rendered = {}
    for name, rigid in (("compliant", False), ("rigid", True)):
        frames, times, positions = _render_run(
            spec, fit.law, rig, rigid=rigid, fps=args.fps, pixels=args.pixels,
            duration_s=args.duration,
        )
        rendered[name] = (frames, times, positions)
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
