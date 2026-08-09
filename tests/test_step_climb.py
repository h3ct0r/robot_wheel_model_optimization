"""The step-climb rig: the drive model, the fairness of the comparison, and the metrics.

The tests that matter most here are the ones asserting the compliant and rigid wheels differ
in **compliance and nothing else**. A signature test whose two wheels also differ in mass or
in rotational inertia will produce the answer the project wants for a reason the project did
not intend, and no amount of care in the physics elsewhere recovers from that.

MuJoCo tests are skipped without the simulator. The MJCF is text, so most of the structure can
be checked without it.
"""

from __future__ import annotations

import unittest
from dataclasses import replace

import numpy as np

from wheelopt.cad.materials import MaterialSpec
from wheelopt.cad.params import WheelParams
from wheelopt.rom.ring import RingSpec, SpringLaw, ring_for_design
from wheelopt.sim.step_climb import (
    EXPLICIT_STABILITY_LIMIT,
    RigSpec,
    build_scenario_mjcf,
    ring_axle_inertia_kg_m2,
    run_step,
    segment_damping_n_s_per_m,
    stable_timestep_s,
)

try:
    import mujoco

    HAVE_MUJOCO = True
except ImportError:  # pragma: no cover - environment dependent
    HAVE_MUJOCO = False

TINY_PARAMS = WheelParams(outer_radius_mm=60.0, width_mm=30.0, n_spokes=6,
                          spoke_thickness_mm=5.0, rim_thickness_mm=3.0,
                          hub_radius_mm=20.0)
SPEC = ring_for_design(TINY_PARAMS, MaterialSpec(name="TPU_95A", infill_density=0.4), 24)
#: The law `run_rom.py --tiny` fits at 24 segments.
LAW = SpringLaw(a=178.3, b=-5.039e4, c=1.248e7)
RIG = RigSpec(payload_kg=0.120, step_height_m=0.036)
HALF_WIDTH, SEGMENT_MASS = 0.015, 0.002


class TestDriveModel(unittest.TestCase):
    def test_stall_torque_scales_with_load_and_radius(self):
        # A tractive coefficient, not a fixed torque: a 0.12 kg debug wheel must not be
        # driven with the robot's 2.8 N.m, which would fling it rather than roll it.
        heavy = replace(RIG, payload_kg=2.5)
        self.assertAlmostEqual(
            heavy.stall_torque_n_m(0.085) / RIG.stall_torque_n_m(0.085),
            2.5 / 0.120, places=9,
        )

    def test_the_platform_sizing_reproduces_its_own_number(self):
        # configs/robot.yaml sizes the motor at ~1.3x vehicle weight in tractive force and
        # quotes 2.8 N.m derated on the nominal wheel. If this drifts, one of the two is wrong.
        nominal = RigSpec(payload_kg=2.5)
        self.assertAlmostEqual(nominal.stall_torque_n_m(0.085), 2.71, delta=0.15)

    def test_torque_falls_linearly_to_zero_at_the_no_load_speed(self):
        radius = SPEC.radius_m
        stall = RIG.stall_torque_n_m(radius)
        no_load_rate = RIG.no_load_speed_m_s / radius
        self.assertAlmostEqual(RIG.motor_torque_n_m(radius, 0.0), stall, places=12)
        self.assertAlmostEqual(RIG.motor_torque_n_m(radius, 0.5 * no_load_rate),
                               0.5 * stall, places=12)
        self.assertAlmostEqual(RIG.motor_torque_n_m(radius, no_load_rate), 0.0, places=12)

    def test_it_never_brakes_and_never_exceeds_stall(self):
        # Overspeeding must not turn the drive into a brake: a wheel braking as it drops off
        # a step would read as a climb failure caused by compliance.
        radius = SPEC.radius_m
        stall = RIG.stall_torque_n_m(radius)
        self.assertEqual(RIG.motor_torque_n_m(radius, 1e6), 0.0)
        self.assertAlmostEqual(RIG.motor_torque_n_m(radius, -1e6), stall, places=12)


class TestDamping(unittest.TestCase):
    def test_it_is_proportional_to_the_loss_factor(self):
        base = segment_damping_n_s_per_m(LAW, SPEC, 0.12, 0.15)
        double = segment_damping_n_s_per_m(LAW, SPEC, 0.12, 0.30)
        self.assertAlmostEqual(double / base, 2.0, places=9)

    def test_a_lossless_material_gets_no_damping(self):
        self.assertEqual(segment_damping_n_s_per_m(LAW, SPEC, 0.12, 0.0), 0.0)

    def test_it_moves_with_the_law_and_the_payload(self):
        # Invariant 2: derived, not tuned. A stiffer law or a heavier payload must move it.
        stiffer = segment_damping_n_s_per_m(replace(LAW, a=4 * LAW.a), SPEC, 0.12, 0.15)
        heavier = segment_damping_n_s_per_m(LAW, SPEC, 0.48, 0.15)
        base = segment_damping_n_s_per_m(LAW, SPEC, 0.12, 0.15)
        self.assertNotAlmostEqual(stiffer, base)
        self.assertNotAlmostEqual(heavier, base)

    def test_it_refuses_a_massless_rig(self):
        with self.assertRaises(ValueError):
            segment_damping_n_s_per_m(LAW, SPEC, 0.0, 0.15)


class TestExplicitStability(unittest.TestCase):
    """The timestep bound on a segment law driven through ``qfrc_applied``.

    Measured 2026-08-09 on the bandless R 60 mm claw ring, 2 g segments, k = 19.76 kN/m
    (ω = 3143 rad/s): ω·h = 0.251 and below run clean, 0.314 and above diverge inside 5 ms.
    The radial-only rig had been running at ω·h = 0.63 and surviving, because an out-of-contact
    radial segment sits at exactly u = 0 with exactly zero force and nothing excites it. The
    tangential joint's axis sweeps through gravity as the wheel turns, so it is excited every
    revolution and the marginal mode grows.
    """

    def test_a_soft_law_does_not_move_the_timestep(self):
        soft = SpringLaw(a=180.0)  # omega = 300 rad/s, needs 6.7e-4
        self.assertEqual(stable_timestep_s([soft], 0.002, 2.0e-4), 2.0e-4)

    def test_a_stiff_law_tightens_it_to_the_measured_bound(self):
        stiff = SpringLaw(a=19760.0)
        got = stable_timestep_s([stiff], 0.002, 2.0e-4)
        self.assertAlmostEqual(got, 6.3628e-05, places=8)
        omega = np.sqrt(19760.0 / 0.002)
        self.assertAlmostEqual(omega * got, EXPLICIT_STABILITY_LIMIT, places=12)
        # Comfortably under the divergence observed at omega*h = 0.314.
        self.assertLess(omega * got, 0.251)

    def test_the_stiffest_law_wins_and_none_is_ignored(self):
        stiff, soft = SpringLaw(a=19760.0), SpringLaw(a=180.0)
        self.assertEqual(stable_timestep_s([stiff, soft], 0.002, 2.0e-4),
                         stable_timestep_s([soft, stiff], 0.002, 2.0e-4))
        self.assertEqual(stable_timestep_s([soft, None], 0.002, 2.0e-4),
                         stable_timestep_s([soft], 0.002, 2.0e-4))

    def test_no_laws_leaves_the_request_alone(self):
        """The rigid wheel has no segment springs, so it must not be slowed down to match
        the compliant one — that would change the comparator to fix the subject."""
        self.assertEqual(stable_timestep_s([], 0.002, 2.0e-4), 2.0e-4)
        self.assertEqual(stable_timestep_s([None], 0.002, 2.0e-4), 2.0e-4)


class TestScenarioMjcf(unittest.TestCase):
    def test_the_rigid_wheel_has_no_ring_and_no_band(self):
        xml = build_scenario_mjcf(SPEC, RIG, rigid=True)
        self.assertNotIn("<tendon>", xml)
        self.assertNotIn('name="seg0"', xml)
        self.assertIn('name="rigidwheel"', xml)

    def test_the_compliant_wheel_carries_the_fitted_ring_and_its_band(self):
        xml = build_scenario_mjcf(SPEC, RIG, rigid=False)
        self.assertIn("<tendon>", xml)
        self.assertIn(f'name="seg{SPEC.n_segments - 1}"', xml)
        self.assertNotIn('name="rigidwheel"', xml)

    def test_the_axle_turns_the_way_that_drives_forward(self):
        # The rig's first version used axis "0 -1 0" and drove the whole thing 41 m backwards,
        # away from the step it was supposed to climb. Rolling toward +x needs omega > 0
        # about +y, because the contact point moves at -omega*R in x.
        self.assertIn('name="axle" type="hinge" axis="0 1 0"', build_scenario_mjcf(
            SPEC, RIG, rigid=False))

    def test_the_tangential_freedom_is_refused_on_a_banded_ring(self):
        """`SPEC` is the tiny `T3`, which has a band. The band tendons couple radial joints
        only, so tangential slides would let the segments shear with nothing resisting — the
        one deformation a shear band exists to carry. Refused rather than silently softened,
        matching `solve_equilibrium_2dof`.
        """
        self.assertTrue(SPEC.is_coupled)
        with self.assertRaises(ValueError):
            build_scenario_mjcf(SPEC, RIG, rigid=False, tangential=True)

    def test_a_bandless_ring_gets_one_extra_joint_per_segment(self):
        bandless = replace(SPEC, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0)
        plain = build_scenario_mjcf(bandless, RIG, rigid=False)
        splayed = build_scenario_mjcf(bandless, RIG, rigid=False, tangential=True)
        self.assertNotIn('name="t0"', plain)
        for i in range(bandless.n_segments):
            self.assertIn(f'name="t{i}"', splayed)

    def test_the_step_top_sits_at_the_requested_height(self):
        rig = replace(RIG, step_height_m=0.05)
        xml = build_scenario_mjcf(SPEC, rig, rigid=True)
        self.assertIn('pos="1.350000000 0 0.025000000"', xml)  # box centre = half height


class TestFairComparison(unittest.TestCase):
    """Mass, radius and inertia must match. Only compliance may differ."""

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
    def test_the_rigid_wheel_loads_across_the_design_space(self):
        """The rigid wheel's inertia must satisfy MuJoCo's triangle inequality everywhere.

        A thin ring's diametral moment is exactly half its axial one, which puts
        ``transverse + transverse == inertia`` — on the boundary, not inside it. Whether the
        model then loaded depended on which way the last printed digit rounded: it worked on
        `--tiny` and failed on the first design `scripts/explore.py` was pointed at, with
        "inertia must satisfy A + B >= C". Swept rather than spot-checked, because a
        boundary case that passes for one geometry says nothing about the next.
        """
        import mujoco

        for n_segments in (8, 10, 12, 24, 36, 48):
            for radius_m in (0.060, 0.085, 0.100):
                for half_width in (0.006, 0.015, 0.035):
                    spec = RingSpec(radius_m=radius_m, n_segments=n_segments)
                    xml = build_scenario_mjcf(spec, RIG, rigid=True,
                                              segment_half_width_m=half_width)
                    with self.subTest(n=n_segments, r=radius_m, w=half_width):
                        mujoco.MjModel.from_xml_string(xml)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
    def test_the_analytic_axle_inertia_is_the_ring_the_model_actually_builds(self):
        """Checked against the assembled model, not against the formula it came from.

        The rigid wheel's inertia is set from :func:`ring_axle_inertia_kg_m2`, so if that
        function and the MJCF ever disagree about where the segments sit or how heavy they
        are, the "fair comparison" tests below would happily compare two wheels that are not
        the same wheel.

        Read off the joint-space mass matrix, whose axle diagonal *is* the inertia the solver
        will use. Summing ``body_inertia`` by hand instead looks equivalent and is not:
        MuJoCo stores each body's moments in its own principal frame, sorted, so for a capsule
        lying along the axle the axial moment is not element 1. That route reports a 3.2%
        error against a formula that is exact.
        """
        model = mujoco.MjModel.from_xml_string(build_scenario_mjcf(SPEC, RIG, rigid=False))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        dense = np.zeros((model.nv, model.nv), dtype=np.float64)
        mujoco.mj_fullM(model, data, dense)
        axle_dof = model.jnt_dofadr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "axle")]

        analytic = ring_axle_inertia_kg_m2(SPEC, HALF_WIDTH, SEGMENT_MASS)
        self.assertAlmostEqual(float(dense[axle_dof, axle_dof]) / analytic, 1.0, places=6)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
    def test_both_wheels_have_the_same_total_mass(self):
        masses = []
        for rigid in (False, True):
            model = mujoco.MjModel.from_xml_string(
                build_scenario_mjcf(SPEC, RIG, rigid=rigid))
            carriage = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "carriage")
            masses.append(float(model.body_subtreemass[carriage]))
        self.assertAlmostEqual(masses[0], masses[1], places=9)
        self.assertAlmostEqual(masses[0], RIG.payload_kg, places=9)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
    def test_both_wheels_have_the_same_axle_inertia(self):
        """The confound this rig would otherwise have.

        A solid cylinder of the ring's mass has half the ring's inertia about the axle,
        because a ring carries its mass at the rim. Less rotational inertia means harder
        acceleration *and* less angular momentum arriving at the step — both of which push
        the climb comparison toward the compliant wheel for reasons that are not compliance.
        """
        inertia = {}
        for rigid in (False, True):
            model = mujoco.MjModel.from_xml_string(
                build_scenario_mjcf(SPEC, RIG, rigid=rigid))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            dense = np.zeros((model.nv, model.nv), dtype=np.float64)
            mujoco.mj_fullM(model, data, dense)
            axle_dof = model.jnt_dofadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "axle")]
            inertia[rigid] = float(dense[axle_dof, axle_dof])
        self.assertAlmostEqual(inertia[True] / inertia[False], 1.0, places=6)

    def test_a_ring_carries_more_inertia_than_a_solid_disc_of_the_same_mass(self):
        # The reason the override above exists, stated as a number rather than a belief.
        mass = SPEC.n_segments * SEGMENT_MASS
        solid = 0.5 * mass * SPEC.radius_m**2
        ring = ring_axle_inertia_kg_m2(SPEC, HALF_WIDTH, SEGMENT_MASS)
        self.assertGreater(ring / solid, 1.5)


@unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
class TestRuns(unittest.TestCase):
    SHORT = replace(RIG, duration_s=1.2)

    def test_the_wheel_rolls_forward(self):
        result = run_step(SPEC, LAW, self.SHORT, rigid=True)
        self.assertTrue(result.ok, result.message)
        self.assertGreater(result.history[-1, 1], result.history[0, 1])

    def test_a_rigid_wheel_reports_no_compliance(self):
        result = run_step(SPEC, LAW, self.SHORT, rigid=True)
        self.assertEqual(result.peak_compression_m, 0.0)
        self.assertEqual(result.fraction_beyond_fit, 0.0)

    def test_the_fit_range_is_reported_and_never_clamps(self):
        # fit_max_m is a reporting threshold. Passing an absurdly small one must change the
        # reported fraction and nothing else — a silently clamped force law would be a worse
        # lie than an extrapolated one.
        loose = run_step(SPEC, LAW, self.SHORT, fit_max_m=float("inf"))
        tight = run_step(SPEC, LAW, self.SHORT, fit_max_m=1e-9)
        self.assertEqual(loose.fraction_beyond_fit, 0.0)
        self.assertGreater(tight.fraction_beyond_fit, 0.5)
        self.assertAlmostEqual(loose.peak_compression_m, tight.peak_compression_m, places=12)

    def test_a_broken_scenario_returns_a_result_rather_than_raising(self):
        # Invariant 4. A negative step height is not a physical obstacle; MuJoCo will reject
        # the geom, and the caller must get a typed failure back.
        result = run_step(SPEC, LAW, replace(self.SHORT, step_height_m=-1.0))
        self.assertFalse(result.ok)
        self.assertTrue(result.message)
