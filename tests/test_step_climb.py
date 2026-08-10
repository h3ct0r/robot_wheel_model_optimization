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
    ClimbProfile,
    RigSpec,
    build_scenario_mjcf,
    default_step_heights_m,
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
#: The same wheel with the band taken out, for the bandless-only paths.
SPEC_BANDLESS = replace(SPEC, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0,
                        root_radius_m=0.020)
HALF_WIDTH, SEGMENT_MASS = 0.015, 0.002


class TestClimbProfile(unittest.TestCase):
    """The sweep keeps its pattern, because the maximum alone cannot be read.

    Pure: `ClimbProfile` is a record, so none of this needs MuJoCo. The sweep that fills it
    is exercised by the runs further down.
    """

    @staticmethod
    def profile(marks: str, first_mm: float = 10.0) -> ClimbProfile:
        """`#` cleared, `.` did not, `E` the run failed. Heights 10 mm apart."""
        heights = (first_mm + 10.0 * np.arange(len(marks))) * 1e-3
        return ClimbProfile(
            heights_m=heights,
            climbed=np.array([m == "#" for m in marks]),
            failed=np.array([m == "E" for m in marks]),
        )

    def test_a_clean_climb_is_monotone_and_uncensored(self):
        p = self.profile("######...")
        self.assertAlmostEqual(p.tallest_m, 0.060)
        self.assertTrue(p.monotone)
        self.assertFalse(p.censored)

    def test_a_bounce_over_a_step_it_failed_below_is_not_monotone(self):
        """The reason the profile exists. Both of these report 60 mm; one is a wheel that
        climbs and one is a wheel that got lucky at a single height, and the maximum cannot
        tell them apart."""
        climb, bounce = self.profile("######..."), self.profile("###..#...")
        self.assertEqual(climb.tallest_m, bounce.tallest_m)
        self.assertTrue(climb.monotone)
        self.assertFalse(bounce.monotone)

    def test_a_result_at_the_top_of_the_range_is_censored(self):
        self.assertTrue(self.profile("######").censored)
        self.assertFalse(self.profile("#####.").censored)

    def test_a_wheel_that_cleared_nothing_reports_zero_and_is_not_censored(self):
        p = self.profile("......")
        self.assertEqual(p.tallest_m, 0.0)
        self.assertFalse(p.censored)

    def test_a_failed_run_is_not_a_cleared_one_nor_a_refusal(self):
        """`E` must not read as "did not climb": a diverged sweep would then look like a poor
        wheel, which is the difference between a result and the absence of one."""
        p = self.profile("##EE..")
        self.assertAlmostEqual(p.tallest_m, 0.020)
        self.assertEqual(int(np.count_nonzero(p.failed)), 2)
        self.assertFalse(p.monotone is None)

    def test_the_default_range_clears_the_radius(self):
        """It ran to 1.01 R until 2026-08-09, and the R 60 mm claw clears exactly 60 mm — so
        the ceiling was active on the very design that found it."""
        heights = default_step_heights_m(SPEC)
        self.assertGreater(heights[-1], SPEC.radius_m * 1.4)
        self.assertLessEqual(float(heights[0]), 0.01)


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
            build_scenario_mjcf(SPEC, RIG, rigid=False, tangential="slide")

    def test_a_bandless_ring_gets_one_extra_joint_per_segment(self):
        bandless = replace(SPEC, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0)
        plain = build_scenario_mjcf(bandless, RIG, rigid=False)
        splayed = build_scenario_mjcf(bandless, RIG, rigid=False, tangential="slide")
        self.assertNotIn('name="t0"', plain)
        for i in range(bandless.n_segments):
            self.assertIn(f'name="t{i}"', splayed)

    def test_damping_is_a_joint_attribute_and_not_an_applied_force(self):
        """Where the loss factor's damping is *integrated*, which is not a style question.

        ``implicitfast`` folds a joint's native ``damping`` into the implicit velocity step
        and integrates ``qfrc_applied`` explicitly. Explicit is stable only while
        ``c·h < 2·I_eff``, and ``I_eff`` for a segment joint is far below the segment mass —
        see the next test. Applied explicitly, the physically derived damping blew the driven
        claw wheel up on round-off, in free flight, before it touched anything (2026-08-09).

        Zero must still emit nothing, so a ring nobody damped is the XML it always was.
        """
        bandless = replace(SPEC, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0,
                           root_radius_m=0.020)
        plain = build_scenario_mjcf(bandless, RIG, rigid=False)
        self.assertNotIn("damping", plain)
        damped = build_scenario_mjcf(bandless, RIG, rigid=False, tangential="hinge",
                                     radial_damping=14.34, tangential_damping_c=3.12e-3)
        self.assertIn('name="j0" type="slide"', damped)
        for line in damped.splitlines():
            if 'name="j0"' in line:
                self.assertIn('damping="14.34"', line)
            if 'name="t0"' in line:
                self.assertIn('damping="0.00312"', line)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
    def test_a_segment_joint_weighs_far_less_than_a_segment(self):
        """The measurement that forbids integrating the damping explicitly.

        Every hinge axis is parallel to the axle and the axle is free, so a torque on one claw
        is reacted by the other eleven and by the carriage: the inertia the joint actually
        presents is the *reduced* one, not the segment's own. Measured by putting a unit
        generalised force on the joint and reading ``qacc`` with everything else free.

        On the 12-claw R 60 mm ring, 2 g segments, on this rig: the hinge's composite inertia
        is 3.26e-6 kg·m² and its effective inertia is **3.03e-7**, 10.8x smaller; the
        tangential slide's is 3.61e-4 kg against a 2 g segment, 5.5x smaller. The collective
        mode across twelve claws is smaller again, which is how ``c·h/I`` reached 9 at a
        timestep the spring bound called safe.

        The effective value moves a little with the carriage mass, because the carriage is one
        of the things that reacts — hence 1% tolerances on numbers otherwise pinned to three
        figures, and a factor-of-five margin asserted separately so the *conclusion* does not
        depend on the rig.
        """
        import mujoco

        bandless = replace(SPEC, band_bending_n_per_m=0.0, band_hoop_n_per_m=0.0,
                           root_radius_m=0.020, n_segments=12)
        for element, composite, expected in (("hinge", 3.2585e-6, 3.0269e-7),
                                             ("slide", 2.0e-3, 3.6064e-4)):
            model = mujoco.MjModel.from_xml_string(
                build_scenario_mjcf(bandless, RIG, rigid=False, tangential=element))
            data = mujoco.MjData(model)
            dof = int(model.jnt_dofadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "t0")])
            self.assertAlmostEqual(float(model.dof_M0[dof]), composite, delta=1e-3 * composite)
            data.qfrc_applied[:] = 0.0
            mujoco.mj_forward(model, data)
            baseline = float(data.qacc[dof])
            data.qfrc_applied[dof] = 1.0
            mujoco.mj_forward(model, data)
            effective = 1.0 / (float(data.qacc[dof]) - baseline)
            with self.subTest(element=element):
                self.assertAlmostEqual(effective, expected, delta=1e-2 * expected)
                self.assertLess(effective, 0.2 * composite)

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


@unittest.skipUnless(HAVE_MUJOCO, "mujoco is not installed")
class TestSofteningLaw(unittest.TestCase):
    """TODO #23: a segment whose force *falls* with compression, driven on purpose.

    `TabulatedLaw` has been able to represent one since #16 and nothing had ever run one —
    `run_step.py --tiny --law table` fits a law that happens not to soften, so it passes 5/5
    without testing this at all. Measured 2026-08-09: it runs, it does not grow energy, and it
    needs no smaller timestep. The wheel drops onto the next stable branch instead of
    diverging, because the payload is a dead weight on a free carriage rather than a
    prescribed displacement.
    """

    #: Knot forces on the tiny design's own 0/2/4/6 mm grid. Every one is a legal spring:
    #: `TabulatedLaw` refuses a *negative* accumulated force, not a falling one.
    KNOTS = np.array([0.0, 0.002, 0.004, 0.006])
    MONOTONE = np.array([0.0, 0.2635, 0.6624, 1.9735])
    SOFT_MIDDLE = np.array([0.0, 0.2635, 0.0241, 1.3352])
    COLLAPSE = np.array([0.0, 1.9735, 0.6000, 0.0500])

    def law(self, forces):
        from wheelopt.rom.ring import TabulatedLaw

        return TabulatedLaw.from_forces(self.KNOTS, forces)

    def test_a_softening_law_is_a_valid_spring_and_says_so(self):
        soft = self.law(self.SOFT_MIDDLE)
        self.assertTrue(soft.is_valid_spring)
        self.assertFalse(soft.is_monotone_nonneg)
        self.assertIn("softens", soft.summary())
        self.assertTrue(self.law(self.MONOTONE).is_monotone_nonneg)

    def test_a_softening_segment_need_not_give_a_softening_wheel(self):
        """The part that is not obvious, and the reason the mild case is uneventful.

        As δ grows more segments engage, so the wheel's curve is the sum of a growing number
        of falling terms. On the bandless tiny ring that wins for the mild case — the segment
        tangent reaches -0.12 N/mm and the **wheel's** stays positive at +0.111, so nothing
        is unstable at all. It does not win for the sharp one: -0.687 per segment gives the
        wheel -1.747. So "a softening law" is not one behaviour, and which it is depends on
        the ring as much as on the law.
        """
        from wheelopt.rom.ring import ring_force_n

        delta = np.linspace(0.0005, 0.008, 30)
        for forces, expected in ((self.MONOTONE, 0.1317), (self.SOFT_MIDDLE, 0.1106),
                                 (self.COLLAPSE, -1.7471)):
            law = self.law(forces)
            force = np.array([float(ring_force_n(SPEC_BANDLESS, law, d)) for d in delta])
            worst = float(np.min(np.gradient(force, delta))) / 1e3
            with self.subTest(forces=forces[-1]):
                self.assertAlmostEqual(worst, expected, places=4)
                if forces is not self.MONOTONE:
                    self.assertLess(float(np.min(law.slopes_n_per_m)), 0.0)

    def test_it_runs_without_growing_energy_or_needing_a_smaller_step(self):
        """The concern #23 was filed on, and it does not materialise.

        A softening branch is statically unstable, so the wheel snaps through — and then
        *lands*, because the payload is a dead weight on a free carriage rather than a
        prescribed displacement, and the flat extrapolation past the last knot always offers a
        branch to land on. The sharpest case crushes the axle from 60 mm to 22.5 mm and
        compresses a segment 38.7 mm against a 6 mm fit, which is a collapse and a result; the
        history stays finite throughout. The timestep bound never binds, because it is set by
        ``k(0)`` and the softening branch does not change that.
        """
        from wheelopt.rom.mjcf import SEGMENT_MASS_KG, stable_timestep_s
        from wheelopt.rom.ring import solve_equilibrium

        for forces in (self.MONOTONE, self.SOFT_MIDDLE, self.COLLAPSE):
            law = self.law(forces)
            static = float(solve_equilibrium(SPEC_BANDLESS, law, 0.003).force_n)
            rig = replace(RIG, payload_kg=max(static / 9.81, 1e-3), step_height_m=0.036)
            result = run_step(SPEC_BANDLESS, law, rig, fit_max_m=0.006)
            with self.subTest(peak=forces[1]):
                self.assertEqual(stable_timestep_s([law], SEGMENT_MASS_KG, rig.timestep_s),
                                 rig.timestep_s)
                self.assertTrue(result.ok, result.message)
                self.assertTrue(bool(np.all(np.isfinite(result.history))))
                # Above the floor and below the undeformed radius plus the step: bounded, not
                # necessarily gentle.
                z = result.history[:, 2]
                self.assertGreater(float(np.min(z)), 0.0)
                self.assertLess(float(np.max(z)), 2.0 * SPEC_BANDLESS.radius_m)

        # And the collapse is reported rather than hidden: a run 6x past its fitted range
        # must say so, whatever the five signatures make of it.
        law = self.law(self.COLLAPSE)
        static = float(solve_equilibrium(SPEC_BANDLESS, law, 0.003).force_n)
        rig = replace(RIG, payload_kg=max(static / 9.81, 1e-3), step_height_m=0.036)
        crushed = run_step(SPEC_BANDLESS, law, rig, fit_max_m=0.006)
        self.assertGreater(crushed.peak_compression_m, 6.0 * 0.006)
        self.assertGreater(crushed.fraction_beyond_fit, 0.5)

    def test_the_minimum_tangent_is_not_a_usable_damping_stiffness(self):
        """#23 proposed deriving the damping from the minimum tangent instead of ``k(0)``.
        On a softening law that is negative, so ``c = η k / ω`` injects energy. Recorded as a
        refuted suggestion rather than silently ignored."""
        law = self.law(self.COLLAPSE)
        self.assertLess(float(np.min(law.slopes_n_per_m)), 0.0)
        self.assertGreater(float(law.stiffness_n_per_m(0.0)), 0.0)
        self.assertGreater(segment_damping_n_s_per_m(law, SPEC_BANDLESS, 0.12, 0.15), 0.0)
