#!/usr/bin/env python3
"""Drive the whole robot at a step, and optionally film it.

    python scripts/run_rover.py --obstacle-height 60          # numbers only
    python scripts/run_rover.py --obstacle-height 60 --render # + MP4/GIF in data/renders
    python scripts/run_rover.py --sweep                       # tallest it clears

    # Four segmented rings instead of four cylinders, built from this design's own FEA:
    python scripts/run_rover.py --compliant --radius 60 --rim-thickness 0 --spokes 12 \\
        --thickness 6 --claw-taper 0.6 --spoke-phase -90 --plane-strain --law claw \\
        --tangential hinge --obstacle-height 50 --render

The robot is `configs/robot.yaml` — chassis box, wheelbase, track, inertia and the motor's
own torque-speed curve, all read rather than invented.

**`--compliant` is a picture, not a measurement** (`TODO.md` #30, #31). The ring is planar, so
a wheel loaded out of plane — by roll, by an angled approach, by dropping off an edge — is
perfectly rigid; and above second-claw engagement neither tangential element is validated,
straddling the FEA by +62.7% (slide) and −49.5% (hinge). The banner says so on every run.

Read the climb number next to the single-wheel rig's, not instead of it. They are different
questions: `run_step.py` asks what one wheel does with a dead weight on it, this asks what
four driven wheels and a rigid chassis do together, and the second flatters a rigid wheel
enormously because three wheels push while one climbs.

Every stage announces itself and reports its own wall time, and the long ones carry a progress
bar on stderr — so piping stdout to a file gives clean output and the bar still finds the
terminal. Uncached, the FEA behind a claw ring is minutes of otherwise-total silence, and
silence is indistinguishable from a hang.
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
from wheelopt.platform import PlatformSpecError, load_platform
from wheelopt.progress import Bar, Stage
from wheelopt.rom.build import build_ring, measure_tangential_law
from wheelopt.sim.rover import CAD_OVERLAY_RGBA, RoverSpec, observe_rover
from wheelopt.video import VideoUnavailable, write_mp4


class _Formatter(argparse.ArgumentDefaultsHelpFormatter,
                 argparse.RawDescriptionHelpFormatter):
    """Show the defaults that mean something, and lay the epilog out as written.

    Defaults are shown because most flags here carry a unit and a physical meaning, and "what
    is it now" is the first question about each. Raw description because the epilog is a block
    of runnable command lines, which argparse would otherwise reflow into a paragraph.

    Two defaults are suppressed rather than printed. A switch is already fully described by
    the fact that it is a switch, so ``(default: False)`` on every one of them is a column of
    noise between the reader and the text that matters. And ``(default: None)`` is worse than
    silence — it reads as though nothing happens, when in every case here the real default is
    a value the help sentence states in words.
    """

    def _get_help_string(self, action: argparse.Action) -> str | None:
        if action.default is None or action.nargs == 0:
            return action.help
        return super()._get_help_string(action)


_EPILOG = """
examples:
  # One run against a 60 mm step, rigid wheels, numbers only.
  run_rover.py --obstacle-height 60

  # The tallest obstacle this wheel clears, as a profile.
  run_rover.py --radius 85 --sweep

  # Flat ground: ride harshness instead of a climb. This is the metric that
  # separates wheel designs -- the step-climb sweep on the rover does not.
  run_rover.py --obstacle-height 0 --compliant --radius 60 --rim-thickness 0 \\
      --spokes 3 --thickness 6 --claw-taper 0.6 --spoke-phase -90 \\
      --plane-strain --law claw

  # Four segmented rings from this design's own FEA, the real CAD shape ghosted
  # over them, filmed. Needs CalculiX for the ring and build123d for the overlay.
  run_rover.py --compliant --stl --radius 60 --rim-thickness 0 --spokes 12 \\
      --thickness 6 --claw-taper 0.6 --spoke-phase -90 --plane-strain \\
      --law claw --tangential hinge --obstacle-height 50 --render --no-gif

exit codes:
  0  the robot cleared the obstacle (or --sweep / a flat run finished)
  1  it did not clear, or the run failed
  2  a dependency or a config file is missing

note:
  Without --compliant or --stl the wheels are plain rigid cylinders, and only
  --radius, --width and --wheel-mass are read. The rest of the geometry and
  material groups describe a design that nothing builds in that mode.

docs:
  docs/run_rover.md -- every flag with figures: the scene, the four wheel
  models, what each stage of the pipeline reads, and how to read the result.
"""


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        epilog=_EPILOG,
        formatter_class=_Formatter,
    )
    p.add_argument("--config", type=Path, default=None,
                   help="platform YAML describing the robot: chassis mass, dimensions, "
                        "inertia, wheelbase, track, ground clearance and motor curve. "
                        "Defaults to configs/robot.yaml")
    # Named for the obstacle, not for "step", which in a MuJoCo script also means the
    # integration step and had to be disambiguated by context every time it was read.
    p.add_argument("--obstacle-height", type=float, default=60.0,
                   help="height of the step the robot drives at, mm. Reported back as a "
                        "fraction of wheel radius. EXACTLY 0 is a different scenario, not a "
                        "small step: no obstacle at all, and the run reports ride harshness "
                        "instead of a climb")
    add_geometry_args(p)
    add_material_args(p)

    run = p.add_argument_group("the run")
    run.add_argument("--wheel-mass", type=float, default=300.0,
                     help="mass of one whole wheel, grams. Held equal between rigid and "
                          "segmented wheels, so the comparison is at matched mass")
    run.add_argument("--throttle", type=float, default=1.0,
                     help="fraction of the platform's stall torque commanded at all four "
                          "axles. Torque is clipped at zero, so a motor never brakes")
    run.add_argument("--duration", type=float, default=6.0,
                     help="simulated seconds. Also sizes the step box, so a longer run "
                          "cannot climb the step and drive off the far end")
    run.add_argument("--friction", type=float, default=1.0,
                     help="Coulomb friction of floor and step. 1.0 is TPU on concrete, "
                          "generous on purpose so a failed climb is not just traction")
    run.add_argument("--approach", type=float, default=0.0,
                     help="yaw of the robot relative to the obstacle face, degrees. Rotates "
                          "the heading; it does not steer, so it stays clear of the "
                          "unvalidated skid-steer scrub. Refused at 90 or beyond")
    run.add_argument("--sweep", action="store_true",
                     help="instead of one run, sweep 10 mm to 2.1 R in 10 mm buckets and "
                          "report the tallest cleared plus the profile. Quote the answer as "
                          "a bucket, not a millimetre")

    compliant = p.add_argument_group(
        "compliant wheels (TODO #30/#31 -- a picture, not a measurement)")
    compliant.add_argument("--compliant", action="store_true",
                           help="replace the four rigid cylinders with four segmented rings "
                                "built from this design's own FEA. Needs CalculiX. Read the "
                                "result as a visualisation: the ring is planar, so a wheel "
                                "loaded out of plane is rigid, and above second-claw "
                                "engagement neither tangential element is validated")
    compliant.add_argument("--law", choices=("cubic", "table", "claw"), default="claw",
                           help="where the segment spring law comes from. 'claw' presses ONE "
                                "claw and uses that curve directly, with no fit at all "
                                "(bandless designs only); 'cubic' and 'table' deconvolve the "
                                "whole-wheel curve instead")
    compliant.add_argument("--segments", type=int, default=24,
                           help="ring resolution. IGNORED by --law claw, where the segments "
                                "are the claws and the count is --spokes")
    compliant.add_argument("--tangential", nargs="?", const="hinge", default=None,
                           choices=("hinge", "slide"), metavar="{hinge,slide}",
                           help="give every claw a second in-plane freedom. 'hinge' rotates "
                                "it about its root and is the right element; 'slide' moves "
                                "the tip and is kept only for comparison. Bare --tangential "
                                "means hinge")
    compliant.add_argument("--plane-strain", action="store_true",
                           help="use the 2-D screening FEA tier -- seconds instead of hours. "
                                "The 3-D tier is the reference and cannot see lateral spoke "
                                "buckling at all")
    compliant.add_argument("--delta-max", type=float, default=0.012, metavar="METRES",
                           help="how deep the FEA presses the wheel, METRES (note: not mm, "
                                "unlike every geometry flag)")
    compliant.add_argument("--n-points", type=int, default=10,
                           help="samples per branch of the FEA load sweep. Too few cannot "
                                "resolve a curve that peaks early")
    compliant.add_argument("--cache", type=Path,
                           default=REPO_ROOT / "data" / "cache" / "fea",
                           help="content-addressed FEA cache. A hit is seconds, a miss is "
                                "minutes; the run says which it got")
    compliant.add_argument("--threads", type=int, default=4,
                           help="CalculiX threads. Changes how long the answer takes, not "
                                "what it is, so it is excluded from the cache key")

    overlay = p.add_argument_group("CAD overlay (decoration -- changes no number)")
    overlay.add_argument("--stl", action="store_true",
                         help="draw this design's real CAD geometry over each wheel, "
                              "translucent grey, so the ring's capsules can be seen against "
                              "the shape they stand for. No collision, no mass. Needs "
                              "build123d; the STL is cached under data/wheels by design hash")
    overlay.add_argument("--mesh-alpha", type=float, default=CAD_OVERLAY_RGBA[3],
                         metavar="A",
                         help="opacity of the CAD overlay, 0 invisible to 1 solid")

    output = p.add_argument_group("output")
    output.add_argument("--render", action="store_true",
                        help="write an MP4, a GIF and a contact sheet to --out")
    output.add_argument("--no-mp4", action="store_true",
                        help="skip the H.264 video (needs ffmpeg on PATH)")
    output.add_argument("--no-gif", action="store_true",
                        help="skip the GIF; the MP4 is around 13x smaller and full-colour")
    output.add_argument("--fps", type=int, default=25,
                        help="frames per second, used BOTH to sample the simulation and to "
                             "play it back, so the video runs at real speed")
    output.add_argument("--pixels", type=int, default=900,
                        help="frame width in pixels; height is 9/16 of it")
    output.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "renders",
                        help="directory for the video, GIF and contact sheet")
    return p


def _wheel_args(args) -> dict:
    return {
        "wheel_radius_m": args.radius * 1e-3,
        "wheel_width_m": args.width * 1e-3,
        "wheel_mass_kg": args.wheel_mass * 1e-3,
        **args.ring,
        **({} if args.mesh is None else
           {"visual_mesh": args.mesh,
            "visual_rgba": (*CAD_OVERLAY_RGBA[:3], args.mesh_alpha)}),
    }


def _build_the_overlay(args) -> str:
    """The CAD STL for the translucent overlay, stashed on ``args.mesh``. "" or a message.

    Optional in the strong sense: a machine without OCCT still runs every simulation here, and
    a design the geometry stage refuses still runs too. Only the decoration is lost, so a
    failure prints and continues rather than stopping the run.
    """
    args.mesh = None
    if not args.stl:
        return ""
    if not 0.0 <= args.mesh_alpha <= 1.0:
        return "--mesh-alpha must be between 0 and 1"
    from wheelopt.cad.export import wheel_stl

    with Stage("CAD, exporting the wheel STL for the overlay") as stage:
        try:
            args.mesh = wheel_stl(params_from_args(args), material_from_args(args),
                                  REPO_ROOT / "data" / "wheels")
        except (ImportError, ValueError) as exc:
            stage.note("skipped")
            print(f"   no overlay: {type(exc).__name__}: {exc}")
            print("   the simulation is unaffected -- the mesh is decoration")
            return ""
        stage.note(f"{args.mesh.stat().st_size / 1e3:.0f} kB, {args.mesh.name}")
    return ""


def _build_the_rings(args) -> str:
    """FEA -> ring, stashed on ``args.ring``. Returns "" or a message.

    Announced stage by stage. Uncached, the two CalculiX solves behind a claw ring are minutes
    of otherwise-total silence, and silence is indistinguishable from a hang.
    """
    args.ring = {}
    if not args.compliant:
        return ""
    params, material = params_from_args(args), material_from_args(args)
    tier = "plane strain" if args.plane_strain else "3-D"
    sweeps = "whole wheel + one claw" if args.law == "claw" else "whole wheel"
    with Stage(f"FEA, {tier}, {sweeps}, {args.delta_max * 1e3:.0f} mm x "
               f"{args.n_points} points") as stage:
        built = build_ring(params, material, law=args.law, n_segments=args.segments,
                           plane_strain=args.plane_strain, delta_max_m=args.delta_max,
                           n_points=args.n_points, cache_root=args.cache,
                           n_threads=args.threads)
        stage.note("cached" if built.cached
                   else f"{built.solver_seconds:.0f}s in the solver")
        if not built.ok:
            stage.note("no ring")
    if not built.ok:
        return built.message

    args.ring = {"spec": built.spec, "law": built.fit.law}
    kind = "held out" if built.fit.iterations == 0 else "fitted"
    print(f"   ring: {built.spec.n_segments} segments/wheel, "
          f"R {built.spec.radius_m * 1e3:.0f} mm, [{kind}] {built.fit.summary()}")
    if not built.fit.ok:
        print("         <- the law did not pass its gate; this is a picture, not a number")

    if args.tangential:
        with Stage(f"FEA, tangential sweep for the {args.tangential} element"):
            law, kinematics, message = measure_tangential_law(
                params, material, built.spec, element=args.tangential,
                cache_root=args.cache, n_threads=args.threads)
        if law is None:
            return f"tangential sweep failed: {message}"
        args.ring["tangential_law"] = law
        args.ring["tangential_element"] = args.tangential
        print(f"   {args.tangential} element on a {built.spec.claw_length_m * 1e3:.1f} mm claw")
        if kinematics is not None:
            measured, predicted = kinematics
            print(f"   tip travel inward at full sweep: FEA {measured[-1] * 1e3:+.2f} mm, "
                  f"hinge predicts {predicted[-1] * 1e3:+.2f} mm")
    return ""


def _run_one(platform, scenario, args, *, label: str, also=None):
    """One rover run with a progress bar over its own integration steps.

    The bar is driven from ``observe_rover``'s observer hook — the same hook the renderer
    uses, and ``also`` chains onto it, so a filmed run gets both and neither runs a second
    simulation. Redraw is rate limited inside :class:`~wheelopt.progress.Bar`, so this costs
    a clock read per step.

    **The step count comes from the compiled model, not from the scenario.** A segmented run
    has its timestep tightened by ``stable_timestep_s`` *inside* ``observe_rover``, so a total
    computed from ``scenario.timestep_s`` here is the number of steps that were asked for and
    not the number that will run — the bar would reach 100% less than half way and sit there.
    """
    bar = Bar(int(scenario.duration_s / scenario.timestep_s), label)

    def observe(k, model, data):
        if k == 0:
            bar.total = max(1, round(scenario.duration_s / model.opt.timestep))
        bar.update(k + 1)
        if also is not None:
            also(k, model, data)

    try:
        result = observe_rover(platform, scenario, observe, **_wheel_args(args))
    finally:
        bar.close()
    return result


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

    with Stage(f"simulating and filming {scenario.duration_s:.1f} s at "
               f"{args.fps} fps, {args.pixels}px", inline=False) as stage:
        result = _run_one(platform, scenario, args, label="simulating", also=observe)
        if state["renderer"] is not None:
            state["renderer"].close()
        stage.note(f"{len(frames)} frames")
    if not result.ok or not frames:
        return result, None, None

    def label(image, text):
        picture = Image.fromarray(image)
        draw = ImageDraw.Draw(picture)
        draw.rectangle([0, 0, 8 + 7 * len(text), 20], fill=(0, 0, 0))
        draw.text((5, 5), text, fill=(255, 255, 255))
        return picture

    args.out.mkdir(parents=True, exist_ok=True)
    tag = f"{scenario.step_height_m * 1e3:.0f}mm"
    gif = args.out / f"rover_{tag}.gif"
    mp4 = args.out / f"rover_{tag}.mp4"
    sheet_path = args.out / f"rover_{tag}_sheet.png"
    written = []
    with (Stage(f"labelling {len(frames)} frames", inline=False) as stage,
          Bar(len(frames), "labelling") as bar):
        pictures = []
        for n, (frame, t, x) in enumerate(zip(frames, times, positions)):
            pictures.append(label(frame, f"t={t:.2f}s  x={x * 1e3:.0f}mm"))
            bar.update(n + 1)
        bar.close()
        picks = np.linspace(0, len(pictures) - 1, 6).round().astype(int)
        tile_w, tile_h = pictures[0].size
        sheet = Image.new("RGB", (tile_w * 3, tile_h * 2))
        for n, index in enumerate(picks):
            sheet.paste(pictures[index], ((n % 3) * tile_w, (n // 3) * tile_h))
        sheet.save(sheet_path)
        stage.note(f"contact sheet {tile_w * 3}x{tile_h * 2}")

    if not args.no_mp4:
        with Stage("encoding H.264") as stage:
            try:
                write_mp4([np.asarray(p) for p in pictures], mp4, fps=args.fps)
                stage.note(f"{mp4.stat().st_size / 1e6:.1f} MB")
                written.append(mp4)
            except VideoUnavailable as exc:
                stage.note("skipped")
                print(f"   no mp4: {exc}")
    if not args.no_gif:
        with Stage("encoding GIF") as stage:
            pictures[0].save(gif, save_all=True, append_images=pictures[1:],
                             duration=int(1000 / args.fps), loop=0)
            stage.note(f"{gif.stat().st_size / 1e6:.1f} MB")
            written.append(gif)
    return result, written, sheet_path


def _report_harshness(platform, args, result, *, flat: bool) -> None:
    """The measured harshness, and the two analytic numbers that check it from outside.

    Objective 3 in ``docs/plan/08-metrics.md``. The measurement is MuJoCo's; the polygon drop
    is closed-form trigonometry on the tip count, and the loaded ripple comes from the ring's
    own spring law with no dynamics in it at all. The project's standing rule is that a model
    needs at least one check against a number it did not produce, and a wheel whose measured
    harshness moves while its polygon drop does not — or the reverse — is worth stopping over.
    """
    print(f"  mean speed         {result.mean_speed_m_s:.2f} m/s")
    print(f"  ride harshness     {result.harshness_rms_m_s2:.2f} m/s^2 RMS vertical"
          + (f", tips at {result.tip_frequency_hz:.0f} Hz"
             if result.tip_frequency_hz else ""))
    if not flat:
        print("                     <- an obstacle run: this is the step, not the wheel. "
              "Use --obstacle-height 0")

    params = params_from_args(args)
    if params.rim_thickness_mm == 0.0 and params.n_spokes >= 3:
        drop = params.polygon_drop_mm
        print(f"  polygon drop       {drop:.1f} mm ({drop / args.radius:.1%} of R) rigid, "
              f"{params.n_spokes} tips  [closed form]")
    spec, law = args.ring.get("spec"), args.ring.get("law")
    if spec is None or law is None:
        return
    from wheelopt.rom.ring import ride_height_ripple_m

    # What one wheel actually carries: the whole robot, four ways. From the platform, not a
    # constant — the same wheel under a heavier chassis has a different ripple.
    load_n = 9.81 * (platform.chassis_mass_kg + 4.0 * args.wheel_mass * 1e-3) / 4.0
    try:
        ripple, lo, hi = ride_height_ripple_m(spec, law, load_n)
    except ValueError as exc:      # a banded spec has no phase, so no ripple
        print(f"  loaded ripple      not defined: {exc}")
        return
    if not np.isfinite(ripple):
        print(f"  loaded ripple      BOTTOMS OUT at {load_n:.1f} N — the ring cannot carry "
              "the robot at some phase")
        return
    print(f"  loaded ripple      {ripple * 1e3:.2f} mm at {load_n:.1f} N/wheel, "
          f"delta {lo * 1e3:.2f}-{hi * 1e3:.2f} mm  [ring, no dynamics]")
    # The characteristic failure of this project is a plausible number quoted outside the
    # range that produced it, and a few-clawed wheel walks straight into it: the polygon drop
    # of a 3-tip R 60 wheel is 30 mm, and the FEA sweep behind the law goes to --delta-max,
    # 12 mm by default. A tabulated law extrapolates on its last slope without complaint.
    knots = getattr(law, "knots_m", None)
    if knots is not None and hi > float(knots[-1]):
        print(f"                     <- EXTRAPOLATED: the law was measured to "
              f"{float(knots[-1]) * 1e3:.1f} mm and this needs {hi * 1e3:.1f} mm "
              f"({hi / float(knots[-1]):.1f}x). Raise --delta-max")


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

    message = _build_the_rings(args) or _build_the_overlay(args)
    if message:
        print(message)
        return 2 if "solver_missing" in message else 1

    def scenario_at(height_mm: float) -> RoverSpec:
        return RoverSpec(step_height_m=height_mm * 1e-3, friction=args.friction,
                         approach_deg=args.approach, duration_s=args.duration,
                         throttle=args.throttle)

    if args.sweep:
        heights = np.arange(10.0, 2.1 * args.radius, 10.0)
        print(f"\ntallest obstacle cleared ({len(heights)} runs, 10 mm buckets; "
              "# cleared, . did not, E run failed):")
        marks, tallest = [], 0.0
        for n, height in enumerate(heights):
            with Stage(f"[{n + 1}/{len(heights)}] {height:.0f} mm", inline=False) as stage:
                result = _run_one(platform, scenario_at(float(height)), args,
                                  label=f"{height:.0f} mm")
                verdict = "E" if not result.ok else ("#" if result.climbed else ".")
                stage.note({"E": result.message or "failed", "#": "CLEARED",
                            ".": "did not clear"}[verdict])
            marks.append(verdict)
            if result.ok and result.climbed:
                tallest = float(height)
        label = "compliant" if args.compliant else "rigid"
        print(f"\n  {label:<12}  {tallest:5.0f} mm  [{''.join(marks)}] "
              f"{heights[0]:.0f}-{heights[-1]:.0f} mm  ({tallest / args.radius:.2f} R)")
        if tallest >= heights[-1]:
            print("  <- AT THE SWEEP CEILING; the true value is >= this")
        return 0

    flat = args.obstacle_height <= 0.0
    scenario = scenario_at(args.obstacle_height)
    written, sheet = [], None
    if args.render:
        try:
            import mujoco  # noqa: F401
            from PIL import Image  # noqa: F401
        except ImportError as exc:
            print(f"rendering needs mujoco and Pillow: {exc}")
            return 2
        result, written, sheet = _render(platform, scenario, args)
    else:
        what = ("flat ground, measuring ride harshness" if flat else
                f"a {args.obstacle_height:.0f} mm obstacle")
        with Stage(f"simulating {scenario.duration_s:.1f} s at {what}",
                   inline=False) as stage:
            result = _run_one(platform, scenario, args, label="simulating")
            stage.note("done" if flat else
                       ("climbed" if result.ok and result.climbed else
                        ("did not clear" if result.ok else "failed")))

    if not result.ok:
        print(f"\n{result.message}")
        return 1

    if flat:
        print("\nflat ground — no obstacle (08-metrics.md S5, objective 3)")
    else:
        print(f"\nobstacle {args.obstacle_height:.0f} mm "
              f"({args.obstacle_height / args.radius:.2f} R)")
        print(f"  climbed            {result.climbed}")
    print(f"  travelled          {result.distance_m * 1e3:.0f} mm")
    if not flat:
        print(f"  final clearance    {result.final_clearance_m * 1e3:.1f} mm "
              f"(standing is {ride * 1e3:.0f} mm)")
    print(f"  peak pitch / roll  {np.degrees(result.peak_pitch_rad):.1f}° / "
          f"{np.degrees(result.peak_roll_rad):.2f}°")
    if not flat:
        print(f"  chassis hit step   {result.chassis_hit_step}")
    print(f"  axle work          {result.energy_j:.1f} J")
    _report_harshness(platform, args, result, flat=flat)
    for path in [*written, sheet]:
        if path is not None:
            print(f"  {path}")
    return 0 if (flat or result.climbed) else 1


if __name__ == "__main__":
    raise SystemExit(main())
