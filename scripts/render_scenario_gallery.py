#!/usr/bin/env python3
"""One picture per scenario, for the README's gallery — rendered from the real models.

Each frame is a genuine `observe_rover` run (rigid R 60 wheels, the platform's own
drivetrain, the simplified shell over the contact box) captured at a moment chosen to show
what the scenario *is* — a wheel at the riser, a wheel in the trench, the robot mid-field.
Not mock-ups: if `build_rover_mjcf` changes, re-running this regenerates the truth.

S2's tilt is the one piece of presentation: the slope scenario tilts *gravity* (see
`RoverSpec.slope_deg`), which a still frame cannot show, so its image is rotated by the
gradient angle — the physics' own equivalence between tilted gravity over flat ground and
level gravity over a ramp, applied to the pixels.

    python scripts/render_scenario_gallery.py          # writes docs/img/scenarios/*.png
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from wheelopt.platform import load_platform
from wheelopt.sim.rover import RoverSpec, observe_rover

OUT = REPO_ROOT / "docs" / "img" / "scenarios"
SHELL = REPO_ROOT / "configs" / "pipebot_simplified.stl"
WHEEL = {"wheel_radius_m": 0.060, "wheel_width_m": 0.030, "wheel_mass_kg": 0.30}
WIDTH, HEIGHT = 640, 360

#: name -> (RoverSpec kwargs, capture time s, camera (azimuth, elevation, distance,
#: lookat_dx, lookat_z), post-rotation deg)
SHOTS = {
    "s1_step": ({"step_height_m": 0.060, "duration_s": 4.0}, 3.6,
                (108, -14, 0.85, 0.10, 0.12), 0.0),
    "s2_slope": ({"step_height_m": 0.0, "slope_deg": 15.0, "duration_s": 3.5}, 3.0,
                 (90, -8, 0.85, 0.0, 0.13), 15.0),
    "s3_gap": ({"step_height_m": 0.0, "gap_width_m": 0.140, "duration_s": 4.5}, 4.2,
               (100, -18, 0.75, 0.08, 0.08), 0.0),
    "s4_rubble": ({"step_height_m": 0.0, "rubble_height_m": 0.030, "duration_s": 5.5},
                  5.0, (105, -16, 0.85, 0.10, 0.10), 0.0),
    "s5_flat": ({"step_height_m": 0.0, "duration_s": 3.0}, 2.6,
                (95, -12, 0.85, 0.0, 0.12), 0.0),
    "s6_spin": ({"step_height_m": 0.0, "spin": True, "duration_s": 6.0}, 5.6,
                (90, -50, 0.95, 0.0, 0.0), 0.0),
    "s7_washboard": ({"step_height_m": 0.0, "washboard_amplitude_m": 0.020,
                      "washboard_wavelength_m": 0.100, "duration_s": 4.5}, 4.0,
                     (100, -13, 0.85, 0.10, 0.11), 0.0),
}


def main() -> int:
    import mujoco
    from PIL import Image

    platform = load_platform()
    OUT.mkdir(parents=True, exist_ok=True)
    for name, (spec_kwargs, at_s, cam_spec, rotate_deg) in SHOTS.items():
        scenario = RoverSpec(**spec_kwargs)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.azimuth, cam.elevation, cam.distance = cam_spec[0], cam_spec[1], cam_spec[2]
        state: dict = {"renderer": None, "frame": None}

        def observe(k, model, data, _state=state, _cam=cam, _at=at_s, _dx=cam_spec[3],
                    _z=cam_spec[4]):
            if _state["renderer"] is None:
                _state["renderer"] = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
            if _state["frame"] is None and data.time >= _at:
                _cam.lookat[:] = (float(data.xpos[1, 0]) + _dx, 0.0, _z)
                _state["renderer"].update_scene(data, _cam)
                _state["frame"] = _state["renderer"].render().copy()

        result = observe_rover(platform, scenario, observe,
                               chassis_mesh=SHELL if SHELL.is_file() else None, **WHEEL)
        if state["renderer"] is not None:
            state["renderer"].close()
        if not result.ok or state["frame"] is None:
            print(f"{name}: FAILED ({result.message or 'no frame captured'})")
            return 1
        picture = Image.fromarray(state["frame"])
        if rotate_deg:
            # The gravity-tilt made visible: rotate and crop back to size, zoomed just
            # enough that no black corner survives the rotation.
            zoom = 1.35
            big = picture.resize((int(WIDTH * zoom), int(HEIGHT * zoom)))
            big = big.rotate(rotate_deg, resample=Image.BICUBIC)
            left = (big.width - WIDTH) // 2
            top = (big.height - HEIGHT) // 2
            picture = big.crop((left, top, left + WIDTH, top + HEIGHT))
        path = OUT / f"{name}.png"
        picture.save(path, optimize=True)
        print(f"{name}: {path.relative_to(REPO_ROOT)} ({path.stat().st_size // 1024} kB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
