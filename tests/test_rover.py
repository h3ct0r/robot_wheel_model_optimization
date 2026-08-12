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

    def test_a_wider_wheel_stands_further_out(self):
        """External mounting (2026-08-11): the inner face seats against the side plate, so
        the track is 2·(mount_face + width/2) and the wheel's own width widens the support
        polygon. The no-width call is the ORIGINAL tucked-under wheels' reference track."""
        narrow = wheel_mounts(PLATFORM, wheel_width_m=0.030)
        wide = wheel_mounts(PLATFORM, wheel_width_m=0.060)
        self.assertEqual({abs(m.y_m) for m in narrow},
                         {PLATFORM.wheel_mount_face_m + 0.015})
        self.assertEqual({abs(m.y_m) for m in wide},
                         {PLATFORM.wheel_mount_face_m + 0.030})
        # And the reference configuration is narrower than any external mount: the original
        # wheels tuck under the shell, inboard of the plates.
        self.assertLess(0.5 * PLATFORM.track_width_m, PLATFORM.wheel_mount_face_m)


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


class TestChassisMesh(unittest.TestCase):
    """The robot's real shell drawn over the chassis box. Decoration, like the wheel overlay —
    and the box must remain the contact geom, because MuJoCo would collide the shell by its
    convex hull, replacing the measured flat belly the nose-in regime depends on with the
    hull of a pipe."""

    @property
    def SMALL(self) -> dict:
        return {"wheel_radius_m": 0.060, "wheel_width_m": 0.045, "wheel_mass_kg": 0.30}

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.stl = Path(self.tmp.name) / "shell.stl"
        self.stl.write_bytes(_tetrahedron_stl())

    def tearDown(self):
        self.tmp.cleanup()

    def xml(self, **kwargs) -> str:
        return build_rover_mjcf(PLATFORM, RoverSpec(), **{**self.SMALL, **kwargs})

    def test_it_is_absent_unless_asked_for(self):
        text = self.xml()
        self.assertNotIn("cadchassis", text)
        self.assertIn('name="body" type="box"', text)
        self.assertIn('material="bodymat"', text)

    def test_the_box_stays_the_contact_geom_and_only_fades(self):
        """The safety property. The shell is a picture; the box is the surface a belly strike
        happens on, and it must keep colliding — faded, never removed."""
        text = self.xml(chassis_mesh=self.stl)
        box = next(x for x in text.splitlines() if 'name="body" type="box"' in x)
        self.assertNotIn('contype="0"', box)          # still collides
        self.assertIn('rgba=', box)                    # but only as a ghost
        self.assertNotIn('material="bodymat"', box)

    def test_the_shell_collides_with_nothing_and_weighs_nothing(self):
        line = next(x for x in self.xml(chassis_mesh=self.stl).splitlines()
                    if 'name="chassis_cad"' in x)
        for attribute in ('contype="0"', 'conaffinity="0"', 'mass="0"', 'density="0"'):
            self.assertIn(attribute, line)

    def test_the_path_is_absolute_and_millimetres_convert_at_the_asset(self):
        text = self.xml(chassis_mesh=Path("configs/x.stl"))
        line = next(x for x in text.splitlines() if 'name="cadchassis"' in x)
        self.assertTrue(Path(line.split('file="')[1].split('"')[0]).is_absolute())
        self.assertIn('scale="0.001 0.001 0.001"', line)

    def test_it_is_placed_by_the_axle_line_not_the_bounding_box(self):
        """The formula, pinned: the mesh's measured axle-stub midpoint must land on the
        body's axle line, which sits (axle_to_belly + half height) below the chassis centre
        regardless of wheel radius. Placing by bounding box instead would centre the shell's
        asymmetric overhang (~105 mm nose / ~71 mm tail) and put all four stubs off their
        axles."""
        from wheelopt.sim.rover import CHASSIS_MESH_AXLE_MM, CHASSIS_MESH_QUAT

        line = next(x for x in self.xml(chassis_mesh=self.stl).splitlines()
                    if 'name="chassis_cad"' in x)
        pos = [float(v) for v in line.split('pos="')[1].split('"')[0].split()]
        lat, tall, long_mid = CHASSIS_MESH_AXLE_MM
        self.assertAlmostEqual(pos[0], long_mid * 1e-3, places=9)
        # Negated: the +90 deg z rotation maps the mesh's lateral axis to +body_y, so a
        # midline left of the mesh's own origin shifts the mesh right, not left.
        self.assertAlmostEqual(pos[1], -lat * 1e-3, places=9)
        dz = -(PLATFORM.axle_to_belly_m + 0.5 * PLATFORM.chassis_height_m)
        self.assertAlmostEqual(pos[2], dz - tall * 1e-3, places=9)
        self.assertIn(f'quat="{CHASSIS_MESH_QUAT[0]} {CHASSIS_MESH_QUAT[1]} '
                      f'{CHASSIS_MESH_QUAT[2]} {CHASSIS_MESH_QUAT[3]}"', line)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_the_shell_changes_no_number_at_all(self):
        """The claim the default-on flag rests on, checked: same deterministic scenario with
        and without the shell, every reported quantity identical."""
        scenario = RoverSpec(step_height_m=0.04, duration_s=3.0)
        plain = observe_rover(PLATFORM, scenario, **self.SMALL)
        drawn = observe_rover(PLATFORM, scenario, chassis_mesh=self.stl, **self.SMALL)
        self.assertTrue(plain.ok and drawn.ok, drawn.message)
        self.assertEqual(plain.climbed, drawn.climbed)
        for field_name in ("distance_m", "final_clearance_m", "peak_pitch_rad",
                           "peak_roll_rad", "energy_j", "harshness_rms_m_s2"):
            with self.subTest(field=field_name):
                self.assertEqual(getattr(plain, field_name), getattr(drawn, field_name))

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_a_model_with_the_shell_compiles(self):
        import mujoco

        model = mujoco.MjModel.from_xml_string(self.xml(chassis_mesh=self.stl))
        self.assertEqual(model.nmesh, 1)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_the_real_shell_puts_its_axle_stubs_on_the_sims_axles(self):
        """The transform checked against the artefact it was measured from, not against its
        own constants: load the real simplified STL, map every vertex through the compiled
        geom pose, and the four axle stubs (the r 7.5 cylinders the external wheels mount
        on) must land on the sim's axle line — x at ±wheelbase/2, z at the axle depth, and
        y starting at the wheel-mount face the platform states."""
        import mujoco

        real = Path(__file__).resolve().parents[1] / "configs" / "pipebot_simplified.stl"
        if not real.is_file():
            self.skipTest("configs/pipebot_simplified.stl not present")
        model = mujoco.MjModel.from_xml_string(self.xml(chassis_mesh=real))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "chassis_cad")
        chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        mesh = model.geom_dataid[geom]
        start = model.mesh_vertadr[mesh]
        verts = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
        world = verts @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        local = (world - data.xpos[chassis]) @ data.xmat[chassis].reshape(3, 3)

        # The stubs are the only structure outboard of the wheel-mount face.
        dz_axle = -(PLATFORM.axle_to_belly_m + 0.5 * PLATFORM.chassis_height_m)
        stubs = local[np.abs(local[:, 1]) > PLATFORM.wheel_mount_face_m - 5e-4]
        self.assertGreater(len(stubs), 0)
        for x_sign in (+1.0, -1.0):
            for y_sign in (+1.0, -1.0):
                stub = stubs[(np.sign(stubs[:, 0]) == x_sign)
                             & (np.sign(stubs[:, 1]) == y_sign)]
                self.assertGreater(len(stub), 0)
                # Axis of the cylinder: mean over the surface points.
                self.assertLess(
                    abs(abs(stub[:, 0].mean()) - 0.5 * PLATFORM.wheelbase_m), 0.003)
                self.assertLess(abs(stub[:, 2].mean() - dz_axle), 0.003)


class TestChassisCollision(unittest.TestCase):
    """`chassis_collision="primitives"`: the pipe, the dome nose and the bracket plates as
    real contact geoms, against the calibrated box. A physics switch, not a rendering one —
    each test here pins one half of that sentence."""

    @property
    def SMALL(self) -> dict:
        return {"wheel_radius_m": 0.060, "wheel_width_m": 0.030, "wheel_mass_kg": 0.30}

    def xml(self, **kwargs) -> str:
        return build_rover_mjcf(PLATFORM, RoverSpec(), **{**self.SMALL, **kwargs})

    def test_the_default_is_the_calibrated_box(self):
        text = self.xml()
        self.assertIn('name="body" type="box"', text)
        self.assertNotIn("chassis_col_", text)

    def test_primitives_replace_the_box_entirely(self):
        """Both at once would collide the step against two overlapping chassis and count
        every belly strike twice."""
        text = self.xml(chassis_collision="primitives")
        self.assertNotIn('name="body"', text)
        from wheelopt.sim.rover import CHASSIS_PRIMITIVES_MM

        for name, _, _, _ in CHASSIS_PRIMITIVES_MM:
            line = next(x for x in text.splitlines() if f'name="chassis_col_{name}"' in x)
            self.assertIn('mass="0" density="0"', line)     # inertia stays the platform's
            self.assertNotIn('contype="0"', line)           # but they DO collide

    def test_nonsense_mode_is_refused_by_name(self):
        with self.assertRaises(ValueError):
            self.xml(chassis_collision="hull")

    def test_the_primitives_are_the_simplified_model_exactly(self):
        """Coverage measured, not asserted from hope: every non-stub vertex of
        `pipebot_simplified.stl` lies ON or INSIDE the primitive union (p100 = 0.00 mm when
        this was pinned) — the simplified model was CAD-authored from these same shapes, so
        the collision set is lossless, not an approximation. If the model is re-exported
        with new geometry, this is the test that says the primitives no longer describe it."""
        real = Path(__file__).resolve().parents[1] / "configs" / "pipebot_simplified.stl"
        if not real.is_file():
            self.skipTest("configs/pipebot_simplified.stl not present")
        with open(real, "rb") as f:
            f.seek(80)
            (n,) = struct.unpack("<I", f.read(4))
            raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        v = raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
        pts = v.reshape(-1, 3)
        pts = pts[np.abs(pts[:, 0]) <= 98.2]        # the stubs live inside the wheels
        from wheelopt.sim.rover import CHASSIS_PRIMITIVES_MM

        dists = np.full(len(pts), np.inf)
        for _, kind, centre, size in CHASSIS_PRIMITIVES_MM:
            c = np.asarray(centre)
            if kind == "cylinder":
                radius, half = size
                dr = np.maximum(np.hypot(pts[:, 0] - c[0], pts[:, 2] - c[2]) - radius, 0.0)
                dy = np.maximum(np.abs(pts[:, 1] - c[1]) - half, 0.0)
                d = np.hypot(dr, dy)
            elif kind == "sphere":
                d = np.maximum(np.linalg.norm(pts - c, axis=1) - size[0], 0.0)
            else:
                excess = np.maximum(np.abs(pts - c) - np.asarray(size), 0.0)
                d = np.linalg.norm(excess, axis=1)
            dists = np.minimum(dists, d)
        self.assertLess(float(dists.max()), 0.5, "a mesh vertex escaped the primitives")

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_the_primitive_belly_hangs_below_the_box_belly(self):
        """The number that makes this a physics change: the bracket plates' bottoms sit
        23 mm BELOW the axle line, where the box belly sits 7.5 mm ABOVE it — 30.5 mm of
        clearance the box claims and the model denies."""
        import mujoco

        model = mujoco.MjModel.from_xml_string(self.xml(chassis_collision="primitives"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        lows = []
        for g in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            if name and name.startswith("chassis_col_") and model.geom_type[g] == 6:  # box
                lows.append(float(data.geom_xpos[g, 2] - model.geom_size[g, 2]))
        # In the chassis frame, so the start-height whisker cancels: the plates reach
        # (axle depth + 23 mm) below the centre, against the box belly's half-height.
        low = min(lows) - float(data.xpos[chassis, 2])
        dz_axle = -(PLATFORM.axle_to_belly_m + 0.5 * PLATFORM.chassis_height_m)
        self.assertAlmostEqual(low, dz_axle - 0.023, places=6)
        box_belly = -0.5 * PLATFORM.chassis_height_m
        self.assertAlmostEqual(box_belly - low, 0.0305, places=6)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_an_untouched_chassis_changes_nothing(self):
        """On flat ground the chassis never contacts anything, so the two collision models
        must produce identical numbers — the difference between them is entirely in what
        happens when chassis meets terrain, never in free dynamics."""
        scenario = RoverSpec(step_height_m=0.0, duration_s=2.0)
        box = observe_rover(PLATFORM, scenario, **self.SMALL)
        prim = observe_rover(PLATFORM, scenario, chassis_collision="primitives",
                             **self.SMALL)
        self.assertTrue(box.ok and prim.ok, prim.message)
        for field_name in ("distance_m", "peak_pitch_rad", "energy_j",
                           "harshness_rms_m_s2", "mean_speed_m_s"):
            with self.subTest(field=field_name):
                self.assertEqual(getattr(box, field_name), getattr(prim, field_name))

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_an_unrealisable_hinge_is_a_result_and_not_a_crash(self):
        """Invariant 4, caught in the field 2026-08-11: a 3-segment R 60 ring's capsule
        radius exceeds its own claw root, so the hinge pivot would sit beyond the axle —
        a legitimate refusal that escaped `observe_rover` as a raw ValueError and killed
        the run instead of returning a typed failure."""
        from wheelopt.rom.ring import RingSpec, TabulatedLaw

        spec = RingSpec(radius_m=0.060, n_segments=3, root_radius_m=0.022)
        law = TabulatedLaw(knots_m=np.array([0.0, 0.01]), slopes_n_per_m=np.array([1e4]))
        result = run_rover(PLATFORM, RoverSpec(duration_s=1.5), spec=spec, law=law,
                           tangential_law=law, tangential_element="hinge", **self.SMALL)
        self.assertFalse(result.ok)
        self.assertIn("3 segments is too few", result.message)

    @unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
    def test_the_dome_meets_a_tall_step_and_the_strike_is_detected(self):
        """At a 100 mm step on R 60 wheels the dome's leading surface protrudes past the
        wheels (226 mm from the chassis centre at riser-top height against the wheel's
        185), so the nose strikes the riser — and `chassis_hit_step` must see it now that
        there is no geom called "body" to watch."""
        result = run_rover(PLATFORM, RoverSpec(step_height_m=0.100, duration_s=4.0),
                           chassis_collision="primitives", **self.SMALL)
        self.assertTrue(result.ok, result.message)
        self.assertTrue(result.chassis_hit_step)
        self.assertFalse(result.climbed)


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
        # 25 mm, not the old 40: the MEASURED robot has 30 mm of ground clearance
        # (2026-08-11, was a 70 mm estimate), so anything taller than the belly is a
        # different test — the one below.
        scenario = RoverSpec(step_height_m=0.025, duration_s=6.0)
        result = observe_rover(PLATFORM, scenario, **WHEEL)
        self.assertTrue(result.ok, result.message)
        self.assertTrue(result.climbed)
        # Wheel-dependent since the axle_to_belly adoption: an R 85 wheel rides at
        # 92.5 mm of belly clearance, not the 30 mm measured at the original r 22.5 wheels.
        ride = (PLATFORM.ground_clearance_for(WHEEL["wheel_radius_m"])
                + 0.5 * PLATFORM.chassis_height_m)
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

    def test_the_robot_pitches_below_its_clearance_and_noses_in_above_it(self):
        """The signature a single-wheel rig cannot produce — re-pinned twice on 2026-08-11
        and the history is the point. v1 (estimated robot, 70 mm constant clearance): rears
        toward 90 at an 80 mm step. v2 (measured robot, 30 mm CONSTANT clearance): bellies
        at 7 deg — but the constant was wrong, because the belly rides a fixed 7.5 mm above
        the AXLE and the measured 30 mm belonged to the original r 22.5 wheels. v3, this
        one: with R 85 fitted the belly sits at 92.5 mm, so an 80 mm step is a genuine climb
        attempt (pitch ~20 deg, no belly) and a 100 mm step is a NOSE-IN — the chassis
        overhangs the front axle by 88 mm and strikes the riser at under 1 deg of pitch."""
        flat = observe_rover(PLATFORM, RoverSpec(step_height_m=0.001, duration_s=4.0), **WHEEL)
        climbing = observe_rover(PLATFORM, RoverSpec(step_height_m=0.08, duration_s=6.0),
                                 **WHEEL)
        nosing = observe_rover(PLATFORM, RoverSpec(step_height_m=0.10, duration_s=6.0),
                               **WHEEL)
        self.assertTrue(flat.ok and climbing.ok and nosing.ok)
        self.assertLess(np.degrees(flat.peak_pitch_rad), 3.0)
        self.assertGreater(np.degrees(climbing.peak_pitch_rad), 10.0)
        self.assertFalse(climbing.chassis_hit_step, "80 mm is below the R85 belly line")
        self.assertTrue(nosing.chassis_hit_step, "100 mm is above it: nose-in")
        self.assertLess(np.degrees(nosing.peak_pitch_rad), 3.0,
                        "a nose-in stops the robot before it can pitch")


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


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
class TestValidityEnvelope(unittest.TestCase):
    """`multi_contact_fraction` — how much of a run left the range the ROM was checked over.

    TODO #31. A bandless claw ring reproduces the FEA to 0.036% while one claw carries the
    load, and the two available elements straddle it by +75%/-46% once two do. The point of
    measuring it is that a compliant number is worth what this fraction says it is worth.
    """

    from wheelopt.rom.ring import RingSpec, SpringLaw

    SPEC = RingSpec(radius_m=0.060, n_segments=12, root_radius_m=0.022,
                    tip_half_thickness_m=0.0018)

    @property
    def SEGMENTED(self) -> dict:
        return {"wheel_radius_m": 0.060, "wheel_width_m": 0.045, "wheel_mass_kg": 0.30,
                "spec": self.SPEC, "law": self.SpringLaw(23000.0)}

    def test_rigid_wheels_report_nothing_rather_than_zero_by_accident(self):
        """A cylinder has no segments, so the fraction is not a small number — it is not a
        number at all, and 0.0 has to mean 'no ring here' rather than 'a ring that stayed
        inside its envelope'. Checked together with the peak compression, which would be the
        giveaway if segments were being read from a rigid model."""
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=3.0), **WHEEL)
        self.assertTrue(result.ok, result.message)
        self.assertEqual(result.multi_contact_fraction, 0.0)
        self.assertEqual(result.peak_compression_m, 0.0)

    def test_a_rolling_claw_wheel_is_on_one_claw_for_most_of_a_flat_run(self):
        """The number this test exists for was **70%** in its first version, and that was an
        artefact: an absolute 1 um threshold counts a claw still ringing down after it leaves
        the ground. The static ring says a wheel at this load carries on one claw everywhere
        but the half-pitch crossing, so a flat run must come out small. It reads 9%."""
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=3.0),
                               **self.SEGMENTED)
        self.assertTrue(result.ok, result.message)
        self.assertLess(result.multi_contact_fraction, 0.35)
        self.assertGreater(result.peak_compression_m, 0.0, "the claws must actually compress")

    def test_a_softer_wheel_shares_more_and_the_fraction_stays_a_fraction(self):
        """Two iterations of this test asserted things the physics refused, and the record
        is the point. v1 said a step deepens compression — the measured robot's 30 mm belly
        grounds on the step first and protects the wheels. v2 said stiffness leaves the
        share unchanged — measured, a 3.8x softer law shares 47 pp MORE, because its static
        sag genuinely engages the neighbours; the share criterion is relative (10% of the
        deepest claw, `MULTI_CONTACT_SHARE`) but the physics it measures is allowed to move.
        What is actually invariant: softer never shares less, and the fraction stays in
        [0, 1] with working claws on both sides."""
        soft = dict(self.SEGMENTED)
        soft["law"] = self.SpringLaw(6000.0)
        stiff = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=3.0),
                              **self.SEGMENTED)
        softer = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=3.0), **soft)
        self.assertTrue(stiff.ok and softer.ok)
        self.assertGreater(stiff.peak_compression_m, 0.001, "the claws must actually work")
        self.assertGreaterEqual(softer.multi_contact_fraction,
                                stiff.multi_contact_fraction)
        for r in (stiff, softer):
            self.assertGreaterEqual(r.multi_contact_fraction, 0.0)
            self.assertLessEqual(r.multi_contact_fraction, 1.0)

    def test_the_fraction_is_a_fraction(self):
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.03, duration_s=3.0),
                               **self.SEGMENTED)
        self.assertTrue(result.ok, result.message)
        self.assertGreaterEqual(result.multi_contact_fraction, 0.0)
        self.assertLessEqual(result.multi_contact_fraction, 1.0)


class TestWashboard(unittest.TestCase):
    """S7's corrugation (`TODO.md` #33): the terrain where compliance can win."""

    def test_smooth_ground_emits_no_washboard(self):
        text = build_rover_mjcf(PLATFORM, RoverSpec(step_height_m=0.0), **WHEEL)
        self.assertNotIn("wash", text)

    def test_the_corrugation_is_a_strip_of_boxes_sampling_the_sinusoid(self):
        spec = RoverSpec(step_height_m=0.0, washboard_amplitude_m=0.010,
                         washboard_wavelength_m=0.100)
        text = build_rover_mjcf(PLATFORM, spec, **WHEEL)
        boxes = [x for x in text.splitlines() if 'name="wash' in x]
        self.assertGreater(len(boxes), 100, "the strip must outrun the robot")
        tops = []
        for line in boxes:
            z = float(line.split('pos="')[1].split('"')[0].split()[2])
            half_h = float(line.split('size="')[1].split('"')[0].split()[2])
            tops.append(z + half_h)
        self.assertAlmostEqual(max(tops), 0.010, delta=0.0006,
                               msg="the crests must reach the stated amplitude")
        self.assertGreater(min(tops), 0.0, "every box must stand proud of the floor")

    def test_the_entry_is_a_ramp_and_not_a_step_edge(self):
        """The strip starts at a TROUGH. Starting at a crest puts a full-amplitude face at
        the entry, and the transient of hitting it would be charged to the corrugation —
        the scenario contributing the acceleration the wheel is being scored on, again."""
        spec = RoverSpec(step_height_m=0.0, washboard_amplitude_m=0.012,
                         washboard_wavelength_m=0.200)
        text = build_rover_mjcf(PLATFORM, spec, **WHEEL)
        first = next(x for x in text.splitlines() if 'name="wash' in x)
        z = float(first.split('pos="')[1].split('"')[0].split()[2])
        half_h = float(first.split('size="')[1].split('"')[0].split()[2])
        # Half, not a whisker: at eight boxes per wave the first riser after the skipped
        # sliver is ~0.3 A, and the failure this guards against — starting at a crest — would
        # put the full A at the entry face.
        self.assertLess(z + half_h, 0.5 * spec.washboard_amplitude_m)

    def test_a_step_and_a_washboard_together_are_refused(self):
        """S1 is the step and S7 is the corrugation; a harshness number taken while climbing
        would be about the step. Nothing defines the combination, so nothing may run it."""
        with self.assertRaises(ValueError):
            RoverSpec(step_height_m=0.05, washboard_amplitude_m=0.010)

    def test_nonsense_corrugations_are_refused_by_name(self):
        with self.assertRaises(ValueError):
            RoverSpec(washboard_amplitude_m=-0.001)
        with self.assertRaises(ValueError):
            RoverSpec(step_height_m=0.0, washboard_amplitude_m=0.01,
                      washboard_wavelength_m=0.0)


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
class TestWashboardDynamics(unittest.TestCase):
    def test_a_rigid_wheel_reads_the_corrugation_it_could_not_read_on_the_flat(self):
        """The reason S7 exists. On the flat a rigid cylinder is the unbeatable 0.00 m/s2;
        on the washboard it must follow the ground, and the measured jump is 40x. Without
        this, the harshness objective scores every compliant wheel against a wheel nobody
        can print, on the one surface where that wheel cannot lose."""
        flat = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=3.0), **WHEEL)
        rough = observe_rover(
            PLATFORM,
            RoverSpec(step_height_m=0.0, duration_s=3.0, washboard_amplitude_m=0.010,
                      washboard_wavelength_m=0.100),
            **WHEEL)
        self.assertTrue(flat.ok and rough.ok, f"{flat.message} {rough.message}")
        self.assertLess(flat.harshness_rms_m_s2, 0.1)
        self.assertGreater(rough.harshness_rms_m_s2, 10.0 * max(flat.harshness_rms_m_s2, 0.5))
        self.assertGreater(rough.mean_speed_m_s, 0.2, "it must still make progress")


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
class TestStabilityMargin(unittest.TestCase):
    """Objective 5: worst-moment distance to static tip-over (`08-metrics.md`)."""

    def test_flat_ground_is_nearly_level_and_a_step_eats_margin(self):
        flat = observe_rover(PLATFORM, RoverSpec(step_height_m=0.0, duration_s=3.0), **WHEEL)
        step = observe_rover(PLATFORM, RoverSpec(step_height_m=0.08, duration_s=4.0), **WHEEL)
        self.assertTrue(flat.ok and step.ok)
        self.assertGreater(flat.stability_margin, 0.9, "driving on the flat barely pitches")
        self.assertLess(step.stability_margin, flat.stability_margin - 0.1)

    def test_the_margin_is_the_peaks_against_the_platforms_own_angles(self):
        """Derived, not invented in the sim: the same peaks the result already carries,
        scored against `tipover_angles_rad`. If the platform's CG or track changes, the
        margin moves with no simulator change — invariant 2 for an objective."""
        result = observe_rover(PLATFORM, RoverSpec(step_height_m=0.06, duration_s=4.0),
                               **WHEEL)
        self.assertTrue(result.ok, result.message)
        pitch_crit, roll_crit = PLATFORM.tipover_angles_rad()
        expected = 1.0 - max(result.peak_pitch_rad / pitch_crit,
                             result.peak_roll_rad / roll_crit)
        self.assertAlmostEqual(result.stability_margin, expected, places=12)


class TestTipoverAngles(unittest.TestCase):
    def test_the_angles_come_from_the_geometry(self):
        """atan(half_span / z_cg). On the MEASURED robot the track (157) is far shorter
        than the wheelbase (250) — the wheels tuck under the shell — so ROLL is the tight
        axis (35.9 against 49.0 deg), the reverse of the fictional wide-track box this test
        first pinned. The stability objective inherits that: this robot's risk is tipping
        sideways on a slope, not backwards on a step."""
        pitch_crit, roll_crit = PLATFORM.tipover_angles_rad()
        z_cg = (PLATFORM.ground_clearance_m + 0.5 * PLATFORM.chassis_height_m
                + PLATFORM.com_offset_m[2])
        self.assertAlmostEqual(pitch_crit, np.arctan2(0.5 * PLATFORM.wheelbase_m, z_cg))
        self.assertAlmostEqual(roll_crit, np.arctan2(0.5 * PLATFORM.track_width_m, z_cg))
        self.assertLess(roll_crit, pitch_crit)

    def test_a_cg_below_ground_is_refused(self):
        from wheelopt.platform import PlatformSpecError

        impossible = replace(PLATFORM, com_offset_m=(0.0, 0.0, -1.0))
        with self.assertRaises(PlatformSpecError):
            impossible.tipover_angles_rad()


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
