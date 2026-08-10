"""The four-wheel rover: geometry from the platform spec, and a climb that means what it says.

Most of this is text and arithmetic, so it runs without MuJoCo. The two tests that actually
drive the robot are skipped without it.

The tests that matter most are the ones checking that the *scenario* cannot flatter the
result. Two versions of this model were wrong in ways every plausible number survived: a step
box shorter than the robot's own travel, so it climbed, crossed and drove off the far end and
the final frame showed it back on the floor; and a climb predicate satisfied by a robot merely
leaning nose-up against the face. Both are regression-tested below.
"""

from __future__ import annotations

import importlib.util
import unittest
from dataclasses import replace

import numpy as np

from wheelopt.platform import load_platform
from wheelopt.sim.rover import (
    RoverSpec,
    build_rover_mjcf,
    observe_rover,
    run_rover,
    wheel_mounts,
)

HAVE_MUJOCO = importlib.util.find_spec("mujoco") is not None

PLATFORM = load_platform()
WHEEL = {"wheel_radius_m": 0.085, "wheel_width_m": 0.030, "wheel_mass_kg": 0.30}


class TestMounts(unittest.TestCase):
    def test_four_wheels_at_the_corners_of_wheelbase_by_track(self):
        mounts = wheel_mounts(PLATFORM)
        self.assertEqual([m.name for m in mounts], ["fl", "fr", "rl", "rr"])
        self.assertEqual({abs(m.x_m) for m in mounts}, {0.5 * PLATFORM.wheelbase_m})
        self.assertEqual({abs(m.y_m) for m in mounts}, {0.5 * PLATFORM.track_width_m})

    def test_the_mounts_move_when_the_platform_does(self):
        """Invariant 2 for a vehicle: nothing here is a constant."""
        wider = replace(PLATFORM, track_width_m=0.5, wheelbase_m=0.3)
        mounts = wheel_mounts(wider)
        self.assertAlmostEqual(mounts[0].y_m, 0.25)
        self.assertAlmostEqual(mounts[0].x_m, 0.15)

    def test_left_and_right_are_opposite_sides(self):
        mounts = {m.name: m for m in wheel_mounts(PLATFORM)}
        self.assertGreater(mounts["fl"].y_m, 0.0)
        self.assertLess(mounts["fr"].y_m, 0.0)
        self.assertEqual(mounts["fl"].side, -mounts["fr"].side)


class TestMjcf(unittest.TestCase):
    def xml(self, scenario: RoverSpec | None = None, **kwargs) -> str:
        return build_rover_mjcf(PLATFORM, scenario or RoverSpec(), **{**WHEEL, **kwargs})

    def test_it_has_a_free_chassis_four_axles_and_four_motors(self):
        text = self.xml()
        self.assertIn("<freejoint", text)
        for name in ("fl", "fr", "rl", "rr"):
            self.assertIn(f'name="{name}_axle"', text)
            self.assertIn(f'joint="{name}_axle"', text)
        self.assertEqual(text.count("<motor "), 4)

    def test_the_chassis_carries_the_platform_mass_and_inertia(self):
        """Not a box density: `robot.yaml` states the inertia, and a geom-derived one would
        silently ignore it the moment a measured inertia replaced the formula."""
        text = self.xml()
        self.assertIn(f'mass="{PLATFORM.chassis_mass_kg:.9f}"', text)
        ixx, iyy, izz = PLATFORM.chassis_inertia_kg_m2
        self.assertIn(f'diaginertia="{ixx:.9g} {iyy:.9g} {izz:.9g}"', text)

    def test_the_chassis_is_a_contact_geom(self):
        """It must be able to belly out on a tall step. A decorative chassis would let the
        robot straddle obstacles taller than its own ground clearance."""
        text = self.xml()
        self.assertIn('name="body" type="box"', text)
        self.assertNotIn('name="body" type="box" contype="0"', text)

    def test_the_step_outruns_the_robot(self):
        """The regression. A 4 m step and a 6.9 m run meant the robot climbed it, crossed it
        and drove off the far end; the final frame then showed it on the floor at exactly its
        ride height, which reads as "never climbed" and is the opposite of what happened."""
        scenario = RoverSpec(step_height_m=0.05, duration_s=6.0)
        text = self.xml(scenario)
        line = next(x for x in text.splitlines() if 'name="step"' in x)
        centre = float(line.split('pos="')[1].split()[0])
        half = float(line.split('size="')[1].split()[0])
        reach = scenario.duration_s * PLATFORM.no_load_speed_rad_s * WHEEL["wheel_radius_m"]
        self.assertGreater(centre + half, scenario.step_x_m + reach)

    def test_a_longer_run_gets_a_longer_step(self):
        short = self.xml(RoverSpec(duration_s=2.0))
        long = self.xml(RoverSpec(duration_s=20.0))
        def size_of(text: str) -> float:
            line = next(x for x in text.splitlines() if 'name="step"' in x)
            return float(line.split('size="')[1].split()[0])
        self.assertGreater(size_of(long), 3.0 * size_of(short))

    def test_the_wheels_get_the_rings_inertia_when_a_spec_is_given(self):
        """A solid cylinder has half a ring's inertia about its axle. Four wheels' worth is a
        real share of a 10 kg robot's resistance to acceleration, so the fairness argument the
        single-wheel rig makes applies here too."""
        from wheelopt.rom.ring import RingSpec

        spec = RingSpec(radius_m=0.085, n_segments=24, root_radius_m=0.020)
        solid = self.xml()
        ringed = self.xml(spec=spec)
        self.assertNotEqual(solid, ringed)


class TestDrive(unittest.TestCase):
    def test_the_torque_curve_is_the_platforms_own(self):
        scenario = RoverSpec()
        self.assertAlmostEqual(scenario.motor_torque_n_m(PLATFORM, 0.0),
                               PLATFORM.stall_torque_n_m)
        self.assertAlmostEqual(
            scenario.motor_torque_n_m(PLATFORM, PLATFORM.no_load_speed_rad_s), 0.0)

    def test_it_never_brakes(self):
        """Clipped, not signed: a motor commanded forward and overspeeding would brake, and a
        braking wheel rolling off a step reads as a climb failure caused by the wheel."""
        scenario = RoverSpec()
        self.assertEqual(
            scenario.motor_torque_n_m(PLATFORM, 3.0 * PLATFORM.no_load_speed_rad_s), 0.0)

    def test_throttle_scales_it(self):
        half = RoverSpec(throttle=0.5)
        self.assertAlmostEqual(half.motor_torque_n_m(PLATFORM, 0.0),
                               0.5 * PLATFORM.stall_torque_n_m)


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
class TestRuns(unittest.TestCase):
    def test_a_low_step_is_climbed_and_the_robot_ends_standing_on_it(self):
        scenario = RoverSpec(step_height_m=0.04, duration_s=5.0)
        result = observe_rover(PLATFORM, scenario, **WHEEL)
        self.assertTrue(result.ok, result.message)
        self.assertTrue(result.climbed)
        ride = PLATFORM.ground_clearance_m + 0.5 * PLATFORM.chassis_height_m
        # Standing on the upper ground, not leaning on it: the chassis centre is one ride
        # height above the step's top face.
        self.assertAlmostEqual(result.final_clearance_m, ride, delta=0.02)
        self.assertFalse(result.chassis_hit_step)

    def test_a_step_taller_than_the_wheel_is_not_climbed(self):
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.16, duration_s=5.0),
                               **WHEEL)
        self.assertTrue(result.ok, result.message)
        self.assertFalse(result.climbed)

    def test_the_observer_sees_every_step(self):
        seen = []
        scenario = RoverSpec(step_height_m=0.02, duration_s=1.2)
        result = observe_rover(PLATFORM, scenario, lambda k, m, d: seen.append(k), **WHEEL)
        self.assertTrue(result.ok, result.message)
        self.assertEqual(len(seen), int(scenario.duration_s / scenario.timestep_s))

    def test_a_run_shorter_than_its_settle_time_is_refused(self):
        """Caught by a test rather than by inspection: `settle_steps` then indexes past the
        end of the history, and the run dies with an IndexError inside the summary rather
        than saying what was wrong with the scenario."""
        with self.assertRaises(ValueError) as ctx:
            RoverSpec(duration_s=0.5, settle_s=0.8)
        self.assertIn("settle", str(ctx.exception))

    def test_a_bad_model_is_a_result_and_not_a_crash(self):
        """Invariant 4. A negative wheel radius is a nonsense model; MuJoCo must refuse it
        and this must hand back a typed failure rather than propagate."""
        result = run_rover(PLATFORM, RoverSpec(),
                           wheel_radius_m=-0.05, wheel_width_m=0.03, wheel_mass_kg=0.3)
        self.assertFalse(result.ok)
        self.assertTrue(result.message)

    def test_the_robot_pitches_at_a_step_and_not_on_the_flat(self):
        """The signature a single-wheel rig cannot produce, and the reason this model exists."""
        flat = observe_rover(PLATFORM, RoverSpec(step_height_m=0.001, duration_s=4.0), **WHEEL)
        stepped = observe_rover(PLATFORM, RoverSpec(step_height_m=0.08, duration_s=4.0),
                                **WHEEL)
        self.assertTrue(flat.ok and stepped.ok)
        self.assertLess(np.degrees(flat.peak_pitch_rad), 3.0)
        self.assertGreater(np.degrees(stepped.peak_pitch_rad), 8.0)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
