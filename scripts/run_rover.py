#!/usr/bin/env python3
"""Drive the whole robot at a step, and optionally film it.

    python scripts/run_rover.py --step 60                 # one run, numbers only
    python scripts/run_rover.py --step 60 --render        # + a GIF in data/renders
    python scripts/run_rover.py --sweep                   # tallest step it clears

The robot is `configs/robot.yaml` — chassis box, wheelbase, track, inertia and the motor's
own torque-speed curve, all read rather than invented. Wheels are **rigid** for now; the
compliant ring goes in the same mounts once `TODO.md` #29 settles what law to put in them.

Read the climb number next to the single-wheel rig's, not instead of it. They are different
questions: `run_step.py` asks what one wheel does with a dead weight on it, this asks what
four driven wheels and a rigid chassis do together, and the second flatters a rigid wheel
enormously because three wheels push while one climbs.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from wheelopt.platform import PlatformSpecError, load_platform
from wheelopt.sim.rover import RoverSpec, observe_rover


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--config", type=Path, default=None, help="platform YAML")
    p.add_argument("--step", type=float, default=60.0, help="step height, mm")
    p.add_argument("--radius", type=float, default=85.0, help="wheel radius, mm")
    p.add_argument("--width", type=float, default=30.0, help="wheel width, mm")
    p.add_argument("--wheel-mass", type=float, default=300.0, help="per wheel, grams")
    p.add_argument("--throttle", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=6.0, help="seconds")
    p.add_argument("--friction", type=float, default=1.0)
    p.add_argument("--sweep", action="store_true",
                   help="find the tallest step it clears, in 10 mm buckets")
    p.add_argument("--render", action="store_true", help="write a GIF and a contact sheet")
    p.add_argument("--fps", type=int, default=25)
    p.add_argument("--pixels", type=int, default=900)
    p.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "renders")
    return p


def _wheel_args(args) -> dict:
    return {
        "wheel_radius_m": args.radius * 1e-3,
        "wheel_width_m": args.width * 1e-3,
        "wheel_mass_kg": args.wheel_mass * 1e-3,
    }


def _render(platform, scenario, args):
    """Film one run through `observe_rover`, so the frames are of the measured run."""
    import mujoco
    from PIL import Image, ImageDraw

    height = int(args.pixels * 9 / 16)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth, camera.elevation = 90.0, -14.0
    camera.distance = 2.6 * platform.chassis_length_m
    frames, times, positions = [], [], []
    state = {"renderer": None, "every": None, "chassis": None}

    def observe(k, model, data):
        if state["renderer"] is None:
            state["renderer"] = mujoco.Renderer(model, height=height, width=args.pixels)
            state["chassis"] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
            state["every"] = max(1, round(1.0 / (args.fps * model.opt.timestep)))
        if k % state["every"]:
            return
        x = float(data.xpos[state["chassis"], 0])
        # Track the robot, but keep the step in frame as it is approached: the point of the
        # film is the obstacle, and a camera welded to the body shows a robot that never moves.
        camera.lookat[:] = (x, 0.0, 0.5 * scenario.step_height_m + 0.15)
        state["renderer"].update_scene(data, camera)
        frames.append(state["renderer"].render().copy())
        times.append(float(data.time))
        positions.append(x)

    result = observe_rover(platform, scenario, observe, **_wheel_args(args))
    if state["renderer"] is not None:
        state["renderer"].close()
    if not result.ok or not frames:
        return result, None, None

    def label(image, text):
        picture = Image.fromarray(image)
        draw = ImageDraw.Draw(picture)
        draw.rectangle([0, 0, 8 + 7 * len(text), 20], fill=(0, 0, 0))
        draw.text((5, 5), text, fill=(255, 255, 255))
        return picture

    args.out.mkdir(parents=True, exist_ok=True)
    pictures = [label(f, f"t={t:.2f}s  x={x * 1e3:.0f}mm")
                for f, t, x in zip(frames, times, positions)]
    tag = f"{scenario.step_height_m * 1e3:.0f}mm"
    gif = args.out / f"rover_{tag}.gif"
    pictures[0].save(gif, save_all=True, append_images=pictures[1:],
                     duration=int(1000 / args.fps), loop=0)

    picks = np.linspace(0, len(pictures) - 1, 6).round().astype(int)
    tile_w, tile_h = pictures[0].size
    sheet = Image.new("RGB", (tile_w * 3, tile_h * 2))
    for n, index in enumerate(picks):
        sheet.paste(pictures[index], ((n % 3) * tile_w, (n // 3) * tile_h))
    sheet_path = args.out / f"rover_{tag}_sheet.png"
    sheet.save(sheet_path)
    return result, gif, sheet_path


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        platform = load_platform(args.config)
    except PlatformSpecError as exc:
        print(f"platform spec: {exc}")
        return 2

    for warning in platform.consistency_warnings():
        print(f"  [platform] {warning}")
    ride = platform.ground_clearance_m + 0.5 * platform.chassis_height_m
    print(f"robot: {platform.name}, {platform.chassis_mass_kg:.1f} kg chassis + "
          f"4 x {args.wheel_mass:.0f} g wheels, "
          f"{platform.chassis_length_m * 1e3:.0f}x{platform.chassis_width_m * 1e3:.0f}x"
          f"{platform.chassis_height_m * 1e3:.0f} mm")
    print(f"       wheelbase {platform.wheelbase_m * 1e3:.0f} mm, track "
          f"{platform.track_width_m * 1e3:.0f} mm, R {args.radius:.0f} mm, ride "
          f"{ride * 1e3:.0f} mm")
    print(f"drive: {platform.stall_torque_n_m:.1f} N·m stall x4 at "
          f"{args.throttle:.2f} throttle, {platform.no_load_speed_rad_s:.1f} rad/s free "
          f"({platform.no_load_speed_rad_s * args.radius * 1e-3:.2f} m/s)")
    if not platform.frozen:
        print("       NOTE meta.frozen is false — these are estimates, not a measured robot")

    def scenario_at(height_mm: float) -> RoverSpec:
        return RoverSpec(step_height_m=height_mm * 1e-3, friction=args.friction,
                         duration_s=args.duration, throttle=args.throttle)

    if args.sweep:
        print("\ntallest step cleared (10 mm buckets; # cleared, . did not, E run failed):")
        heights = np.arange(10.0, 2.1 * args.radius, 10.0)
        marks, tallest = [], 0.0
        for height in heights:
            result = observe_rover(platform, scenario_at(float(height)), **_wheel_args(args))
            marks.append("E" if not result.ok else ("#" if result.climbed else "."))
            if result.ok and result.climbed:
                tallest = float(height)
        print(f"  rigid wheels  {tallest:5.0f} mm  [{''.join(marks)}] "
              f"{heights[0]:.0f}-{heights[-1]:.0f} mm  ({tallest / args.radius:.2f} R)")
        if tallest >= heights[-1]:
            print("  <- AT THE SWEEP CEILING; the true value is >= this")
        return 0

    scenario = scenario_at(args.step)
    gif = sheet = None
    if args.render:
        try:
            import mujoco  # noqa: F401
            from PIL import Image  # noqa: F401
        except ImportError as exc:
            print(f"rendering needs mujoco and Pillow: {exc}")
            return 2
        result, gif, sheet = _render(platform, scenario, args)
    else:
        result = observe_rover(platform, scenario, **_wheel_args(args))

    if not result.ok:
        print(f"\n{result.message}")
        return 1

    print(f"\nstep {args.step:.0f} mm ({args.step / args.radius:.2f} R)")
    print(f"  climbed            {result.climbed}")
    print(f"  travelled          {result.distance_m * 1e3:.0f} mm")
    print(f"  final clearance    {result.final_clearance_m * 1e3:.1f} mm "
          f"(standing is {ride * 1e3:.0f} mm)")
    print(f"  peak pitch / roll  {np.degrees(result.peak_pitch_rad):.1f}° / "
          f"{np.degrees(result.peak_roll_rad):.2f}°")
    print(f"  chassis hit step   {result.chassis_hit_step}")
    print(f"  axle work          {result.energy_j:.1f} J")
    for path in (gif, sheet):
        if path is not None:
            print(f"  {path}")
    return 0 if result.climbed else 1


if __name__ == "__main__":
    raise SystemExit(main())
