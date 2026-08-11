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
import struct
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

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


def _tetrahedron_stl() -> bytes:
    """A minimal valid binary STL, so the overlay tests need MuJoCo but not OCCT."""
    corners = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0],
                        [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]], dtype=np.float32)
    faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
    out = bytearray(b"\0" * 80 + struct.pack("<I", len(faces)))
    for a, b, c in faces:
        out += struct.pack("<3f", 0.0, 0.0, 0.0)
        for index in (a, b, c):
            out += corners[index].tobytes()
        out += struct.pack("<H", 0)
    return bytes(out)


def _marked_stl() -> bytes:
    """A tetrahedron with one vertex 100 mm along CAD **-y**, the direction a claw tip points
    on a `spoke_phase_deg = -90` design. Where that vertex lands says which way the CAD frame
    was rotated into MuJoCo's, and nothing else in a symmetric wheel does."""
    corners = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 0.0, 10.0],
                        [0.0, -100.0, 0.0]], dtype=np.float32)
    faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
    out = bytearray(b"\0" * 80 + struct.pack("<I", len(faces)))
    for a, b, c in faces:
        out += struct.pack("<3f", 0.0, 0.0, 0.0)
        for index in (a, b, c):
            out += corners[index].tobytes()
        out += struct.pack("<H", 0)
    return bytes(out)


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


class TestSegmentedWheels(unittest.TestCase):
    """Four rings in four mounts. TODO #30's picture, ahead of #31's number."""

    from wheelopt.rom.ring import RingSpec

    SPEC = RingSpec(radius_m=0.060, n_segments=12, root_radius_m=0.022)

    @property
    def SMALL(self) -> dict:  # a property, so ruff sees no mutable class attribute
        return {"wheel_radius_m": 0.060, "wheel_width_m": 0.045, "wheel_mass_kg": 0.30}

    def xml(self, **kwargs) -> str:
        return build_rover_mjcf(PLATFORM, RoverSpec(), spec=self.SPEC, segmented=True,
                                **{**self.SMALL, **kwargs})

    def test_every_wheel_gets_its_own_namespace(self):
        """MJCF names are global. Four subtrees all calling their first segment `seg0` is a
        model that does not compile; four all wiring to the *first* wheel's joints is worse,
        because it does."""
        text = self.xml()
        for name in ("fl", "fr", "rl", "rr"):
            self.assertIn(f'name="{name}_seg0"', text)
            self.assertIn(f'name="{name}_j0"', text)
        self.assertEqual(text.count('name="fl_j0"'), 1)

    def test_the_whole_wheel_still_weighs_what_was_asked_for(self):
        """Otherwise the rigid comparator is not a comparator. The per-segment mass is derived
        from `wheel_mass_kg` so hub + N segments comes back to it."""
        from wheelopt.rom.mjcf import HUB_MASS_KG

        text = self.xml()
        per_segment = (self.SMALL["wheel_mass_kg"] - HUB_MASS_KG) / self.SPEC.n_segments
        emitted = {float(chunk.split('"')[0])
                   for chunk in text.split('mass="')[1:] if "capsule" not in chunk}
        segment_masses = [m for m in emitted if abs(m - per_segment) < 1e-12]
        self.assertEqual(len(segment_masses), 1, f"expected {per_segment} among {emitted}")
        hub = HUB_MASS_KG
        total = hub + self.SPEC.n_segments * per_segment
        self.assertAlmostEqual(total, self.SMALL["wheel_mass_kg"], places=12)

    def test_the_capsules_carry_a_chosen_colour(self):
        """They previously carried none — `ring_bodies` emitted no `rgba`, so the capsules took
        MuJoCo's built-in geom colour, which reads as a washed olive-green under this scene's
        lighting. The one colour in the render that had not been chosen was the one carrying
        the result, which is the same shape of mistake as an unstated default anywhere else."""
        from wheelopt.sim.rover import SEGMENT_RGBA

        line = next(x for x in self.xml().splitlines() if 'name="fl_g0"' in x)
        self.assertIn("rgba=", line)
        red, green, blue, _ = SEGMENT_RGBA
        self.assertGreater(red, green, "must not read as green")
        self.assertGreater(green, blue)

    def test_the_single_wheel_rig_is_untouched_by_that(self):
        """`ring_bodies` states that it emits exactly the XML it emitted before each optional
        feature existed. `rgba=None` has to keep that promise, or every step_climb golden
        comparison silently changes."""
        from wheelopt.rom.mjcf import ring_bodies

        plain = ring_bodies(self.SPEC, segment_half_width_m=0.015)
        self.assertTrue(all("rgba" not in line for line in plain))

    def test_a_wheel_lighter_than_its_hub_is_refused(self):
        with self.assertRaises(ValueError) as ctx:
            self.xml(wheel_mass_kg=0.01)
        self.assertIn("hub", str(ctx.exception))

    def test_segments_need_a_spec(self):
        with self.assertRaises(ValueError):
            build_rover_mjcf(PLATFORM, RoverSpec(), segmented=True, **self.SMALL)

    def test_the_segments_cannot_touch_the_chassis(self):
        """The regression, and it is the nastiest bug in this module so far because the model
        it produced looked *right*: the robot settled to exactly its 170 mm ride height with
        exactly the 1.06 mm of segment compression 24.5 N implies. It simply could not move.

        MuJoCo excludes a body from its parent, so a rigid tyre — a child of the chassis —
        never touches it. A segment's parent is the wheel, whose parent is the chassis, and a
        *grandparent* is not excluded. With R 60 mm wheels under 70 mm of ground clearance the
        top half of every wheel is inside the chassis box, so all 48 segments jammed against
        it: 28 contacts, 0.01 rad/s, 0.3 J of axle work, 1 mm travelled.
        """
        from wheelopt.sim.rover import GROUND_CONAFFINITY, SEGMENT_CONTYPE

        text = self.xml()
        chassis = next(x for x in text.splitlines() if 'name="body" type="box"' in x)
        floor = next(x for x in text.splitlines() if 'name="floor"' in x)
        segment = next(x for x in text.splitlines() if 'name="fl_g0"' in x)
        # Chassis is on the default (1, 1); segments carry their own bit; ground carries both.
        self.assertIn(f'contype="{SEGMENT_CONTYPE}"', segment)
        self.assertIn('conaffinity="0"', segment)
        self.assertIn(f'conaffinity="{GROUND_CONAFFINITY}"', floor)
        self.assertNotIn("contype", chassis)
        # The masks, stated as the arithmetic MuJoCo actually does: two geoms collide when
        # (contype_a & conaffinity_b) or (contype_b & conaffinity_a) is non-zero.
        def collide(a: tuple[int, int], b: tuple[int, int]) -> bool:
            return bool((a[0] & b[1]) or (b[0] & a[1]))

        segment_mask = (SEGMENT_CONTYPE, 0)
        chassis_mask = (1, 1)
        ground_mask = (1, GROUND_CONAFFINITY)
        self.assertFalse(collide(segment_mask, chassis_mask), "segment must not see chassis")
        self.assertFalse(collide(segment_mask, segment_mask), "nor another segment")
        self.assertTrue(collide(segment_mask, ground_mask), "but must see the ground")
        self.assertTrue(collide(chassis_mask, ground_mask), "and the chassis must belly out")
        self.assertTrue(collide((1, 1), ground_mask), "and a rigid tyre still touches down")

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_a_segmented_rover_actually_drives(self):
        """The behavioural half of the test above. Every static number was right while the
        robot was welded to the floor, so the only check that catches it is motion."""
        from wheelopt.rom.ring import SpringLaw

        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.02, duration_s=4.0),
                               spec=self.SPEC, law=SpringLaw(a=2.0e4), **self.SMALL)
        self.assertTrue(result.ok, result.message)
        self.assertGreater(result.distance_m, 0.5)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_a_law_without_a_spec_is_a_result_and_not_a_crash(self):
        from wheelopt.rom.ring import SpringLaw

        result = run_rover(PLATFORM, RoverSpec(), law=SpringLaw(a=1.0e4), **self.SMALL)
        self.assertFalse(result.ok)
        self.assertIn("RingSpec", result.message)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_a_rigid_wheel_cannot_be_given_claw_compliance_by_accident(self):
        """Same guard the single-wheel rig makes: honouring a tangential law with no segments
        would build four cylinders and claim they had claw compliance."""
        from wheelopt.rom.ring import SpringLaw

        result = observe_rover(PLATFORM, RoverSpec(duration_s=1.5),
                               tangential_law=SpringLaw(a=1.0), **self.SMALL)
        self.assertTrue(result.ok, result.message)


class TestCadOverlay(unittest.TestCase):
    """The translucent real geometry drawn over each wheel. Decoration, and it must stay so."""

    @property
    def SMALL(self) -> dict:  # a property, so ruff sees no mutable class attribute
        return {"wheel_radius_m": 0.060, "wheel_width_m": 0.045, "wheel_mass_kg": 0.30}

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        # A stand-in STL: MuJoCo needs a real, loadable mesh, and building one through OCCT
        # would make this test need the CAD kernel to check something that is not about CAD.
        self.stl = Path(self.tmp.name) / "probe.stl"
        self.stl.write_bytes(_tetrahedron_stl())

    def tearDown(self):
        self.tmp.cleanup()

    def xml(self, **kwargs) -> str:
        return build_rover_mjcf(PLATFORM, RoverSpec(), **{**self.SMALL, **kwargs})

    def test_it_is_absent_unless_asked_for(self):
        self.assertNotIn("cadwheel", self.xml())

    def test_one_asset_and_one_geom_per_wheel(self):
        text = self.xml(visual_mesh=self.stl)
        self.assertEqual(text.count('<mesh name="cadwheel"'), 1)
        for name in ("fl", "fr", "rl", "rr"):
            self.assertIn(f'name="{name}_cad" type="mesh" mesh="cadwheel"', text)

    def test_it_collides_with_nothing_and_weighs_nothing(self):
        """The whole safety property. A mesh geom given the material default would acquire a
        mass and a collision surface, and the ROM would be running on the CAD geometry — which
        is exactly what ADR-0002 exists to prevent, arrived at by decoration."""
        line = next(x for x in self.xml(visual_mesh=self.stl).splitlines()
                    if 'name="fl_cad"' in x)
        for attribute in ('contype="0"', 'conaffinity="0"', 'mass="0"', 'density="0"'):
            self.assertIn(attribute, line)

    def test_the_path_is_absolute(self):
        """So the model does not depend on the process's working directory — the same trap
        that made a scratchpad script solve into an invisible second cache."""
        text = self.xml(visual_mesh=Path("data/wheels/x.stl"))
        path = text.split('file="')[1].split('"')[0]
        self.assertTrue(Path(path).is_absolute())

    def test_millimetres_are_converted_at_the_asset(self):
        text = self.xml(visual_mesh=self.stl)
        self.assertIn('scale="0.001 0.001 0.001"', text)

    def test_the_alpha_is_what_was_asked_for(self):
        text = self.xml(visual_mesh=self.stl, visual_rgba=(0.1, 0.2, 0.3, 0.75))
        self.assertIn('name="cadmat" rgba="0.100 0.200 0.300 0.750"', text)

    def test_it_uses_the_same_euler_as_the_rigid_cylinder(self):
        """CAD lays the wheel in x-y with the axle along z; MuJoCo wants x-z with the axle
        along y and segment 0 at -z. Rx(+pi/2) sends CAD -y to MuJoCo -z, so a design with
        `spoke_phase_deg = -90` puts its claw tip under the contact point. The opposite sign
        puts it on top — a half-pitch error that reads as fine in a still frame."""
        from wheelopt.sim.rover import CAD_TO_WHEEL_EULER_X

        self.assertAlmostEqual(CAD_TO_WHEEL_EULER_X, np.pi / 2)
        text = self.xml(visual_mesh=self.stl)
        tyre = next(x for x in text.splitlines() if 'name="fl_tyre"' in x)
        cad = next(x for x in text.splitlines() if 'name="fl_cad"' in x)
        self.assertIn("1.5707963", tyre)
        self.assertIn("1.5707963", cad)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_the_cad_frame_lands_where_the_ring_does(self):
        """The sign of the euler, checked against the ring rather than by eye.

        A marked vertex at CAD (0, -1, 0) — the direction a `spoke_phase_deg = -90` claw tip
        points — must arrive at MuJoCo (0, 0, -1) in the wheel frame, which is where segment 0
        sits. The opposite sign puts it at +z, on top of the wheel: on a twelve-fold design
        that is a half-pitch error, and in a still frame it looks completely fine.

        Measured on the real geometry the same way (2026-08-10): mesh radius 60.00 mm against
        a design R of 60, width 45.0 against 45, and tip bearings at multiples of 30 degrees
        *including zero* — straight down, where the ring's segment 0 is.
        """
        import mujoco

        from wheelopt.rom.ring import RingSpec

        marker = Path(self.tmp.name) / "marker.stl"
        marker.write_bytes(_marked_stl())
        # A *segmented* model, because the point is to compare the mesh against the ring. On
        # the rigid model `fl_seg0` does not exist, `mj_name2id` returns -1, and `xpos[-1]` is
        # the last body — which is another wheel at the same height, so the comparison reads
        # 0.0 and passes nothing. Every id below is asserted to have resolved.
        model = mujoco.MjModel.from_xml_string(self.xml(
            visual_mesh=marker, segmented=True,
            spec=RingSpec(radius_m=0.060, n_segments=12, root_radius_m=0.022)))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "fl_cad")
        self.assertGreaterEqual(geom, 0)
        mesh = model.geom_dataid[geom]
        start = model.mesh_vertadr[mesh]
        verts = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
        world = verts @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fl")
        local = (world - data.xpos[wheel]) @ data.xmat[wheel].reshape(3, 3)

        # The marker is the vertex furthest from the origin; it was placed at CAD -y.
        far = local[np.argmax(np.linalg.norm(local, axis=1))]
        self.assertAlmostEqual(far[0], 0.0, places=6)
        self.assertLess(far[2], 0.0, "the CAD -y direction must land at MuJoCo -z, not +z")
        self.assertAlmostEqual(abs(far[2]), 0.100, places=6)   # 100 mm, scaled to metres

        segment = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fl_seg0")
        self.assertGreaterEqual(segment, 0, "fl_seg0 must exist for this comparison to mean "
                                            "anything; xpos[-1] is a different wheel")
        seg_local = ((data.xpos[segment] - data.xpos[wheel])
                     @ data.xmat[wheel].reshape(3, 3))
        self.assertLess(seg_local[2], 0.0)
        self.assertAlmostEqual(seg_local[0], 0.0, places=6)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_the_overlay_changes_no_number_at_all(self):
        """The claim the flag makes, checked rather than asserted in a docstring. Same seedless
        deterministic scenario with and without the mesh: every reported quantity identical."""
        scenario = RoverSpec(step_height_m=0.04, duration_s=3.0)
        plain = observe_rover(PLATFORM, scenario, **self.SMALL)
        drawn = observe_rover(PLATFORM, scenario, visual_mesh=self.stl, **self.SMALL)
        self.assertTrue(plain.ok and drawn.ok, drawn.message)
        self.assertEqual(plain.climbed, drawn.climbed)
        for field_name in ("distance_m", "final_clearance_m", "peak_pitch_rad",
                           "peak_roll_rad", "energy_j"):
            with self.subTest(field=field_name):
                self.assertEqual(getattr(plain, field_name), getattr(drawn, field_name))

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_a_model_with_the_overlay_compiles(self):
        import mujoco

        model = mujoco.MjModel.from_xml_string(self.xml(visual_mesh=self.stl))
        self.assertEqual(model.nmesh, 1)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_a_missing_mesh_file_is_a_result_and_not_a_crash(self):
        result = run_rover(PLATFORM, RoverSpec(duration_s=1.5),
                           visual_mesh=Path("does/not/exist.stl"), **self.SMALL)
        self.assertFalse(result.ok)
        self.assertTrue(result.message)


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


class TestFlatGroundIsAScenario(unittest.TestCase):
    """`step_height_m = 0` is the harshness case, not a degenerate step."""

    def test_a_negative_step_is_refused_by_name(self):
        """A trench is S3, and nothing here models one. Refused rather than passed to MuJoCo,
        which would emit a box of negative size and fail with a message about XML."""
        with self.assertRaises(ValueError) as ctx:
            RoverSpec(step_height_m=-0.01)
        self.assertIn("flat ground", str(ctx.exception))

    def test_flat_ground_emits_no_step_geom_at_all(self):
        """Not a zero-height box, which MuJoCo refuses, and not an epsilon one, which would be
        a lip the robot bumps over — contributing exactly the acceleration the scenario exists
        to measure, from the scenario instead of from the wheel."""
        text = build_rover_mjcf(PLATFORM, RoverSpec(step_height_m=0.0), **WHEEL)
        self.assertNotIn('name="step"', text)
        self.assertIn('name="floor"', text)
        self.assertIn('name="step"', build_rover_mjcf(PLATFORM, RoverSpec(step_height_m=0.05),
                                                      **WHEEL))


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
class TestHarshness(unittest.TestCase):
    """Objective 3 in `08-metrics.md`: RMS vertical chassis acceleration."""

    def test_a_rigid_cylinder_on_flat_ground_is_perfectly_smooth(self):
        """The floor of the metric, and the thing that makes the claw numbers mean something.
        A cylinder rolling on a plane has no polygon forcing at all, so anything above ~0 here
        would be the solver rather than the wheel."""
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=4.0), **WHEEL)
        self.assertTrue(result.ok, result.message)
        self.assertLess(result.harshness_rms_m_s2, 0.1)
        self.assertEqual(result.tip_frequency_hz, 0.0, "a cylinder has no tips")
        self.assertGreater(result.mean_speed_m_s, 0.3, "and it has to actually be moving")

    def test_flat_ground_has_nothing_to_have_climbed(self):
        """Both halves of the `climbed` test pass on flat ground the moment the robot has
        driven a metre — past `step_x + L/2` and at exactly its ride height above a step of
        zero height. Reporting True for that is a true sentence about nothing."""
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=4.0), **WHEEL)
        self.assertTrue(result.ok, result.message)
        self.assertGreater(result.distance_m, 1.0, "it did drive past the phantom face")
        self.assertFalse(result.climbed)

    def test_the_launch_transient_is_excluded(self):
        """The largest vertical acceleration in the whole run is the squat as the robot leaves
        rest at stall torque. That is a fact about the motor; a harshness number that included
        it would rank drivetrains rather than wheels."""
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=4.0), **WHEEL)
        self.assertTrue(result.ok, result.message)
        settle = int(0.8 / 5.0e-4)
        whole = float(np.sqrt(np.mean(result.history[settle:, 5] ** 2)))
        self.assertLess(result.harshness_rms_m_s2, whole)

    def test_the_acceleration_is_read_from_the_solver_not_differenced(self):
        """`qacc` on the free joint's z DOF, not a second difference of the height history: at
        a 5e-4 s step, differencing multiplies contact noise by 4e6 and measures the
        integrator. The check is that a standing robot reads ~0 rather than -g — the contact
        force balances gravity, and an accelerometer bolted to the chassis would agree."""
        scenario = RoverSpec(step_height_m=0.0, duration_s=1.5, settle_s=0.8)
        result = observe_rover(PLATFORM, scenario, **WHEEL)
        self.assertTrue(result.ok, result.message)
        settled = result.history[int(0.6 / scenario.timestep_s):
                                 int(0.8 / scenario.timestep_s), 5]
        self.assertLess(float(np.max(np.abs(settled))), 1.0,
                        "a robot at rest on the floor is not accelerating")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
