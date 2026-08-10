"""First-week step 5: drive the ring at a step, beside a rigid wheel of the same radius.

``docs/plan/16-first-week.md`` step 5, and the four questions step 6 asks of the result:

    Does the compliant wheel **envelop** the step edge?
    Is its **contact patch larger** than the rigid wheel's under the same load?
    Does it **climb better** and **roll worse**?
    Does loaded rolling radius **decrease** with load?

The rig, and what it is not
---------------------------
One wheel on a vertical slider, driven by a torque at the axle: a single-wheel test rig, not
a robot. The hub carries the quarter-vehicle mass on a horizontal and a vertical slide, and
the axle torque's reaction goes to the world rather than pitching a chassis, because there is
no chassis to pitch. That is deliberate. The spike is asking whether *compliance* changes the
four signatures, and a chassis would add weight transfer, a second wheel's traction and a
suspension geometry — three more ways for the answer to come out right for the wrong reason.

Both wheels are built to the **same radius and the same total mass**, so the only difference
between them is where the compliance is. The rigid one is a cylinder; the compliant one is
:mod:`wheelopt.rom.mjcf`'s ring, the same object that was fitted to the FEA, imported rather
than re-derived.

The two things this rig cannot honestly claim
---------------------------------------------
**Damping is an input, not a measurement.** Cost of transport on flat is supposed to come out
*higher* for a softer wheel, and it will only do that if the ring dissipates. A hyperelastic
FEA cannot supply that number — the loading and unloading branches coincide by construction
(``fea/extract.py``) — so the loss factor here is a material constant with a literature
provenance and a wide error bar. A cost-of-transport ranking is therefore a statement about
:data:`TPU_LOSS_FACTOR` as much as about the geometry. See ``docs/plan/07-materials.md``.

**The spring law is only valid where it was fitted.** A wheel meeting a 50 mm step deflects
far past the indentation range of a flat-plate sweep, and a cubic extrapolated beyond its data
is a guess. Rather than hide that, every run reports
:attr:`StepResult.fraction_beyond_fit` — the share of contact samples whose compression left
the fitted range. Read the climb result with that number next to it.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from itertools import pairwise

import numpy as np

from ..rom.mjcf import (
    EXPLICIT_STABILITY_LIMIT,
    HUB_MASS_KG,
    SEGMENT_MASS_KG,
    MissingMuJoCo,
    TangentialElement,
    coupling_tendons,
    hinge_arm_m,
    resolve_tangential_element,
    ring_bodies,
    stable_timestep_s,
    tangential_damping,
)
from ..rom.ring import (
    RadialLaw,
    RingSpec,
    TipEquivalentLaw,
    solve_equilibrium,
    symmetric_force_n,
)

__all__ = [
    # Re-exported from `wheelopt.rom.mjcf`, where they now live: the bound is a property of
    # driving a joint through `qfrc_applied`, not of this scenario. Kept importable here
    # because this is where it was found and where the rig reads it.
    "EXPLICIT_STABILITY_LIMIT",
    "TPU_LOSS_FACTOR",
    "ClimbProfile",
    "RigSpec",
    "Signature",
    "StepResult",
    "build_scenario_mjcf",
    "default_step_heights_m",
    "highest_step_climbed",
    "judge_signatures",
    "loaded_radius_table",
    "observe_step",
    "ring_axle_inertia_kg_m2",
    "run_flat",
    "run_step",
    "segment_damping_n_s_per_m",
    "stable_timestep_s",
    "step_climb_profile",
]

#: Loss factor (tan δ) of printed TPU at room temperature and rolling frequencies.
#: **Provisional.** ``docs/plan/07-materials.md`` says the Prony series this should come from
#: needs DMA equipment the project does not have, so this is a literature midpoint for
#: thermoplastic polyurethane, not a measurement of the filament in use. Reported values span
#: roughly 0.05-0.30 depending on hard-segment content, temperature and frequency; anything
#: this constant decides should be re-run across that span before it is believed.
TPU_LOSS_FACTOR = 0.15


@dataclass(frozen=True, slots=True)
class RigSpec:
    """The test rig: what is driving the wheel, over what, and how hard."""

    #: Mass on the axle, kg. Set it from the load the wheel is *fitted* for, not from the
    #: platform, when the two disagree — a wheel driven outside its fitted range answers a
    #: question about extrapolation rather than about compliance.
    payload_kg: float = 2.5
    #: Step height, metres. The plan's headline case is 0.050.
    step_height_m: float = 0.050
    #: Where the step's face sits, metres. Far enough that the wheel is rolling steadily.
    step_x_m: float = 0.35
    #: Stall torque as a multiple of ``m·g·R`` — a tractive coefficient, so a lighter or
    #: smaller wheel is not accidentally driven with a robot's motor. 1.3 is the platform's
    #: own sizing rationale (``configs/robot.yaml``: "tractive force of ~1.3x vehicle
    #: weight"), which on the nominal wheel comes to 2.8 N·m against the 4.0 N·m stall.
    torque_ratio: float = 1.3
    #: Free-running speed, m/s. With it the drive is a **motor**, ``τ = τ_stall(1 − ω/ω₀)``,
    #: rather than a constant torque. Not a refinement: a constant torque on a wheel with no
    #: resistance to speak of accelerates without limit — the first version of this rig put
    #: the wheel 41 m away at 3 s and still accelerating, which makes cost of transport
    #: meaningless and every contact metric a measurement of a wheel in flight. A real motor
    #: also stalls at its stall torque against an obstacle, which is exactly the quantity the
    #: climbing question is about. ``configs/robot.yaml`` gives 14 rad/s at the output on the
    #: nominal wheel, i.e. 1.19 m/s; 0.4 is the platform's stated target speed.
    no_load_speed_m_s: float = 0.4
    #: Ground friction. 1.0 is TPU on concrete, which is generous and deliberately so: a
    #: climb that fails on traction says nothing about compliance.
    friction: float = 1.0
    duration_s: float = 4.0
    timestep_s: float = 2.0e-4
    #: Contact stiffness for the *ground*, as a solver time constant. A regulariser, never a
    #: compliance model (ADR-0001, invariant 8) — kept an order of magnitude stiffer than the
    #: ring so that what the wheel does is the ring's doing.
    contact_solref_s: float = 1.0e-3
    loss_factor: float = TPU_LOSS_FACTOR

    def stall_torque_n_m(self, radius_m: float) -> float:
        """Torque at zero speed, N·m."""
        return self.torque_ratio * self.payload_kg * 9.81 * radius_m

    def motor_torque_n_m(self, radius_m: float, axle_rate_rad_s: float) -> float:
        """The linear torque-speed curve, clipped to forward drive only.

        Clipped rather than allowed negative: a motor commanded forward and overspeeding
        would brake, and a braking wheel rolling down off a step would look like a climb
        failure caused by compliance rather than by the drive model.
        """
        no_load_rate = self.no_load_speed_m_s / radius_m
        fraction = 1.0 - axle_rate_rad_s / no_load_rate
        return self.stall_torque_n_m(radius_m) * min(max(fraction, 0.0), 1.0)


@dataclass(frozen=True, slots=True)
class StepResult:
    """One run. Never raises — invariant 4 applies to a scenario like any other evaluation."""

    ok: bool
    message: str = ""
    climbed: bool = False
    #: Furthest the axle travelled, metres.
    distance_m: float = 0.0
    #: Mechanical work put in at the axle, joules.
    energy_j: float = 0.0
    #: Dimensionless: work in, per unit weight, per unit distance.
    cost_of_transport: float = float("nan")
    #: Longest ground contact patch reached while within a radius of the step edge, metres.
    #: The envelopment measure: a wheel that wraps a corner touches it over a length, a rigid
    #: one touches it at a point.
    edge_patch_m: float = 0.0
    #: Deepest compression any segment reached, metres. Zero for a rigid wheel.
    peak_compression_m: float = 0.0
    #: Share of samples where a loaded segment was compressed past the fitted range. The
    #: honesty term: a climb achieved entirely out here is an extrapolation, not a result.
    fraction_beyond_fit: float = 0.0
    #: Axle height minus step height at the end, metres. Positive and near the radius means
    #: the wheel is standing on the step.
    final_clearance_m: float = 0.0
    #: Mean contact patch length while rolling on the flat before the step, metres.
    mean_flat_patch_m: float = 0.0
    history: np.ndarray = field(default_factory=lambda: np.empty(0))


def segment_damping_n_s_per_m(law: RadialLaw, spec: RingSpec, payload_kg: float,
                              loss_factor: float) -> float:
    """Viscous damping per segment equivalent to a hysteretic loss factor, N·s/m.

    A hysteretic material with loss factor ``η`` dissipates the same energy per cycle as a
    viscous damper ``c = η k / ω`` — but only at the frequency ``ω`` where the equivalence is
    struck, which is the whole weakness of the substitution and the reason it is written out
    here rather than buried in a constant. The frequency chosen is the wheel's vertical bounce
    mode on its own springs, ``ω = sqrt(k_total / m)``, because that is the mode a step
    excites and the one that governs whether the wheel lands or bounces.

    Derived from the fitted law and the actual payload, so it moves when either does
    (invariant 2). It is not a tuning knob and must not be used as one.

    **On a softening law this is ambiguous, and the ambiguity is worth about 8% of cost of
    transport** (measured 2026-08-09, ``TODO.md`` #23). The equivalence ``c = η k / ω`` wants a
    *storage* stiffness, and a segment on a negative-tangent branch has none: the tangent at
    the operating point is the wrong sign. Two defensible readings remain — the tangent at
    ``u = 0``, used here, and the secant at the static deflection — and on the tiny design's
    hand-softened tables they differ by up to 40%, moving cost of transport by −7.4% and −9.2%.

    The tangent at zero stays, for two reasons. It is the only reading that is defined without
    knowing the operating point, and the disagreement is an order of magnitude below the loss
    factor's own: ``TPU_LOSS_FACTOR`` is a literature midpoint on a 0.05–0.30 span, a factor of
    six, and every cost-of-transport number is already a statement about *that*.

    What is **not** an option, though ``TODO.md`` #23 proposed it, is reading the *minimum*
    tangent. On a softening law that is negative — −0.687 N/mm on the sharpest case tested — so
    ``η k / ω`` comes out negative and the damper injects energy.
    """
    if payload_kg <= 0:
        raise ValueError("payload_kg must be positive")
    # Tangent stiffness of the whole ring at its static deflection is what sets the bounce
    # frequency; approximate it by the small-strain value, which is the conservative end.
    per_segment = float(law.stiffness_n_per_m(0.0))
    in_contact = max(3.0, 0.25 * spec.n_segments)
    total = per_segment * in_contact
    omega = np.sqrt(max(total, 1e-9) / payload_kg)
    return float(loss_factor * per_segment / max(omega, 1e-9))


def ring_axle_inertia_kg_m2(spec: RingSpec, segment_half_width_m: float,
                            segment_mass_kg: float) -> float:
    """Rotational inertia of the segmented ring about its axle, kg·m².

    Each segment is a capsule lying along the axle at radius ``body_radius``, so it
    contributes its own axial inertia plus the parallel-axis term. The hub is a 5 mm sphere
    at the centre and contributes ~1e-6 of the total, but it is in the mass, so it is in here
    too rather than being dropped as "small" — the point of this function is that the rigid
    wheel matches the ring exactly, and "exactly" is checked in ``tests/test_step_climb.py``
    against the joint-space mass matrix MuJoCo actually integrates.

    The capsule's own axial term is the split of a cylinder and two hemispheres, not
    ``½mr²``. The cap correction is worth 0.007% of the total here, which is far too small to
    matter dynamically and precisely large enough to stop the test above being an equality.
    Approximating it would leave a permanent 7e-5 fudge in a comparison whose whole value is
    that the two wheels are identical.
    """
    capsule_radius = 0.25 * spec.segment_arc_m
    body_radius = spec.radius_m - capsule_radius
    cylinder_volume = 2.0 * np.pi * capsule_radius**2 * segment_half_width_m
    cap_volume = (4.0 / 3.0) * np.pi * capsule_radius**3
    cylinder_fraction = cylinder_volume / (cylinder_volume + cap_volume)
    own = segment_mass_kg * capsule_radius**2 * (
        0.5 * cylinder_fraction + 0.4 * (1.0 - cylinder_fraction)
    )
    segments = spec.n_segments * (own + segment_mass_kg * body_radius**2)
    hub = 0.4 * HUB_MASS_KG * 0.005**2
    return float(segments + hub)


def build_scenario_mjcf(spec: RingSpec, rig: RigSpec, *, rigid: bool,
                        segment_half_width_m: float = 0.015,
                        segment_mass_kg: float = 0.002,
                        tangential: TangentialElement | None = None,
                        radial_damping: float = 0.0,
                        tangential_damping_c: float = 0.0) -> str:
    """MJCF for one wheel on the rig. ``rigid=True`` swaps the ring for a cylinder.

    The rigid wheel is given the ring's **total** mass, not a nominal one, so the pair differ
    in compliance and in nothing else. Getting that wrong would make the compliant wheel climb
    better because it is heavier, which is a real effect and the wrong one.
    """
    radius = spec.radius_m
    ring_mass = spec.n_segments * segment_mass_kg + HUB_MASS_KG
    start_z = radius + 0.0005  # a whisker clear, so the first step settles rather than bangs

    parts = [
        '<mujoco model="step_climb">',
        '  <compiler angle="radian"/>',
        ('  <option gravity="0 0 -9.81" integrator="implicitfast" '
         f'timestep="{rig.timestep_s:.9g}"/>'),
        "  <default>",
        (f'    <geom friction="{rig.friction:.4g} 0.005 0.0001" '
         f'solref="{rig.contact_solref_s:.9g} 1" solimp="0.98 0.999 0.001" condim="3"/>'),
        "  </default>",
        # Offscreen framebuffer, for `scripts/render_step.py`. MuJoCo's default is 640x480 and
        # a larger `--pixels` raises rather than downscaling, so a claw wheel — twelve thin
        # segments that need the resolution to be legible at all — could not be filmed above
        # 640 wide. Rendering only; it changes nothing the solver sees.
        "  <visual>",
        '    <global offwidth="1920" offheight="1080"/>',
        "  </visual>",
        # Lights, textures and colours. Visual only — MuJoCo does not integrate any of it, so
        # the model that gets rendered is the model that gets measured, which is the whole
        # point of being able to look at it. The floor checker is not decoration: without
        # ground texture a tracking camera gives no cue that the wheel is moving at all, and
        # the first render of this rig was unreadable for exactly that reason.
        "  <asset>",
        ('    <texture name="grid" type="2d" builtin="checker" width="512" height="512" '
         'rgb1="0.24 0.26 0.30" rgb2="0.32 0.34 0.39"/>'),
        ('    <material name="floormat" texture="grid" texrepeat="40 8" '
         'reflectance="0.05"/>'),
        '    <material name="stepmat" rgba="0.55 0.42 0.32 1"/>',
        '    <material name="wheelmat" rgba="0.20 0.65 0.85 1"/>',
        '    <material name="hubmat" rgba="0.95 0.75 0.15 1"/>',
        "  </asset>",
        "  <worldbody>",
        '    <light pos="0 -1.5 2" dir="0 0.5 -1" directional="true" diffuse="0.7 0.7 0.7"/>',
        '    <light pos="1.5 -1 1.5" dir="-0.5 0.3 -1" diffuse="0.4 0.4 0.4"/>',
        '    <geom name="floor" type="plane" size="5 1 0.1" material="floormat"/>',
        # The step is a box whose top face is the upper ground. Its left face is the riser the
        # wheel has to get over; the corner between them is deliberately sharp, because a
        # filleted one is a different and easier obstacle.
        (f'    <geom name="step" type="box" pos="{rig.step_x_m + 1.0:.9f} 0 '
         f'{rig.step_height_m / 2:.9f}" size="1.0 0.5 {rig.step_height_m / 2:.9f}" '
         'material="stepmat"/>'),
        f'    <body name="carriage" pos="0 0 {start_z:.9f}">',
        '      <joint name="ride_x" type="slide" axis="1 0 0"/>',
        '      <joint name="ride_z" type="slide" axis="0 0 1"/>',
        (f'      <geom name="mass" type="sphere" size="0.005" '
         f'mass="{max(rig.payload_kg - ring_mass, 1e-4):.9f}" contype="0" conaffinity="0"/>'),
        '      <body name="hub" pos="0 0 0">',
        # Axis +y, so a positive torque rolls the wheel toward +x: the contact point moves
        # at -omega*R*xhat, hence forward motion needs omega > 0 about +y. The first version
        # used -y and drove the whole rig backwards past the step it was meant to climb.
        '        <joint name="axle" type="hinge" axis="0 1 0"/>',
    ]

    if rigid:
        # Mass and radius match the ring; so must the rotational inertia, or the comparison
        # is not about compliance. A solid cylinder of the same mass has half the ring's
        # inertia about the axle, because the ring carries its mass at the rim — and a wheel
        # with less rotational inertia accelerates harder *and* arrives at the step with less
        # angular momentum. Both effects push the climb result the way the spike wants it,
        # which is exactly the kind of help a result does not need. The explicit <inertial>
        # overrides the geom's computed one.
        inertia = ring_axle_inertia_kg_m2(spec, segment_half_width_m, segment_mass_kg)
        # Diametral moment of a cylindrical shell of finite width: I_axle/2 + m·L²/12 with
        # L = 2·half_width. The rig is planar and never uses it — but MuJoCo validates the
        # inertia triangle inequality at load, and the *thin-ring* value ``0.5 * inertia``
        # sits exactly **on** it: transverse + transverse == inertia. Whether the model then
        # loads depends on which way `%.12g` rounds the last digit, so it worked on `--tiny`
        # and failed on the first design `scripts/explore.py` was pointed at, with
        # "inertia must satisfy A + B >= C". The width term is not a fudge to clear the
        # check; it is the moment these capsules actually have, and it happens to be
        # strictly inside the boundary rather than on it.
        transverse = 0.5 * inertia + ring_mass * segment_half_width_m**2 / 3.0
        parts += [
            (f'        <inertial pos="0 0 0" mass="{ring_mass:.9f}" '
             f'diaginertia="{transverse:.12g} {inertia:.12g} {transverse:.12g}"/>'),
            (f'        <geom name="rigidwheel" type="cylinder" euler="1.5707963 0 0" '
             f'size="{radius:.9f} {segment_half_width_m:.9f}" mass="0" density="0" '
             'material="wheelmat"/>'),
        ]
    else:
        parts.append(
            f'        <geom name="hubgeom" type="sphere" size="0.005" mass="{HUB_MASS_KG}" '
            'contype="0" conaffinity="0" material="hubmat"/>'
        )
        # conaffinity 0 keeps segments from colliding with each other; contype 1 lets them
        # hit the floor and the step. Neighbour interaction must arrive through the band.
        parts += ring_bodies(spec, segment_half_width_m=segment_half_width_m,
                             segment_mass_kg=segment_mass_kg, tangential=tangential,
                             radial_damping=radial_damping,
                             tangential_damping_c=tangential_damping_c, indent=8)

    parts += ["      </body>", "    </body>", "  </worldbody>"]
    if not rigid:
        parts += coupling_tendons(spec)
    parts += [
        "  <actuator>",
        '    <motor name="drive" joint="axle" gear="1" ctrlrange="-100 100"/>',
        "  </actuator>",
        "</mujoco>",
    ]
    return "\n".join(parts)


def _simulate(spec, law, rig, *, rigid, fit_max_m, tangential_law=None,
              tangential_element=None, settle_s=0.6, observer=None):
    """Shared integration loop. Returns the per-step history and the model handles.

    ``observer``, if given, is called ``observer(k, model, data)`` after every ``mj_step``.
    It exists so that ``scripts/render_step.py`` can film **this** run rather than a second
    copy of it. That renderer had its own loop until 2026-08-09 and had drifted: it applied the
    loss-factor damping through ``qfrc_applied`` and never asked for the stable timestep, so it
    was still integrating the pre-#27 rig, and every frame it produced was of a simulation
    nobody was measuring. A renderer whose whole job is to be a differently-shaped check on the
    numbers has to be looking at the thing the numbers came from.
    """
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise MissingMuJoCo("MuJoCo is not installed; pip install -e '.[sim]'") from exc

    # A rigid wheel has no segments, so it cannot have a tangential freedom either. Silently
    # honouring the argument there would build a cylinder and claim it had claw compliance.
    tangential_law = None if rigid else tangential_law
    element = resolve_tangential_element(tangential_law, tangential_element)
    # A hinge's coordinate is an angle, so every rule below — the timestep bound and the
    # hysteretic damping equivalence — is applied to the law referred to the tip, and the
    # damping is referred back. Doing it any other way means writing both rules twice in two
    # unit systems, which is a second chance to lose a factor of the moment arm.
    arm = hinge_arm_m(spec) if element == "hinge" else 0.0
    equivalent_law = (TipEquivalentLaw(tangential_law, arm) if element == "hinge"
                      else tangential_law)
    # Tighten the timestep if the segment laws need it, before the model is built — `rig`
    # carries the timestep into the MJCF. A rigid wheel has no segment laws and keeps the
    # requested step, so the pair are compared at whatever each one needs to be correct
    # rather than at whatever the softer one can survive.
    if not rigid:
        rig = replace(rig, timestep_s=stable_timestep_s(
            [law, equivalent_law], SEGMENT_MASS_KG, rig.timestep_s))
    damping = 0.0 if rigid else segment_damping_n_s_per_m(
        law, spec, rig.payload_kg, rig.loss_factor
    )
    # The tangential mode is far softer than the radial one on a claw, so its bounce frequency
    # and therefore its equivalent viscous damping are different numbers. Derived from the
    # tangential law by the same rule rather than reusing the radial one, which would
    # over-damp it by the square root of the stiffness ratio.
    tan_damping = 0.0 if tangential_law is None else tangential_damping(
        spec, element,
        segment_damping_n_s_per_m(equivalent_law, spec, rig.payload_kg, rig.loss_factor),
    )
    # Both go into the MJCF as native joint damping, not into `qfrc_applied`. The loss factor
    # is physics and stays; how it is *integrated* is not, and explicitly is wrong here --
    # `implicitfast` handles native damping implicitly, and the effective inertia of a segment
    # joint is two orders below the segment mass, so the explicit form blew up on round-off in
    # free flight. See `wheelopt.rom.mjcf.ring_bodies`.
    model = mujoco.MjModel.from_xml_string(
        build_scenario_mjcf(spec, rig, rigid=rigid, tangential=element,
                            segment_mass_kg=SEGMENT_MASS_KG,
                            radial_damping=damping, tangential_damping_c=tan_damping)
    )
    data = mujoco.MjData(model)

    def joint_addr(prefix: str) -> tuple[np.ndarray, np.ndarray]:
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}{i}")
               for i in range(spec.n_segments)]
        return (np.asarray(model.jnt_dofadr[ids], dtype=np.int64),
                np.asarray(model.jnt_qposadr[ids], dtype=np.int64))

    empty = np.empty(0, dtype=np.int64)
    segment_dofs, segment_qpos = joint_addr("j") if not rigid else (empty, empty)
    tangential_dofs, tangential_qpos = (joint_addr("t") if tangential_law is not None
                                        else (empty, empty))
    axle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "axle")
    axle_dof = model.jnt_dofadr[axle]
    carriage = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "carriage")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    step_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")

    n_steps = int(rig.duration_s / rig.timestep_s)
    settle_steps = int(settle_s / rig.timestep_s)
    history = np.zeros((n_steps, 7), dtype=np.float64)

    for k in range(n_steps):
        if not rigid:
            compression = -data.qpos[segment_qpos]
            data.qfrc_applied[segment_dofs] = law.force_n(compression)
            if tangential_law is not None:
                # Symmetric: a claw bends the same either way, so the restoring force is
                # sign(v)*f(|v|). Passing `v` straight to `force_n` would make one direction
                # free and the other doubled, and the wheel would drift in the rolling
                # direction under load.
                splay = data.qpos[tangential_qpos]
                data.qfrc_applied[tangential_dofs] = -symmetric_force_n(
                    tangential_law, splay
                )
        # Torque only after the wheel has settled onto the ground, so the run measures rolling
        # rather than the transient of a wheel dropped onto a plane while being spun.
        torque = (0.0 if k < settle_steps
                  else rig.motor_torque_n_m(spec.radius_m, float(data.qvel[axle_dof])))
        data.ctrl[0] = torque
        mujoco.mj_step(model, data)
        if observer is not None:
            observer(k, model, data)

        deepest = float(np.max(-data.qpos[segment_qpos])) if not rigid else 0.0
        history[k] = (
            data.time,
            float(data.xpos[carriage, 0]),
            float(data.xpos[carriage, 2]),
            float(torque * data.qvel[axle_dof]),  # instantaneous axle power, W
            _contact_span_m(data, {floor}),
            deepest,
            _contact_span_m(data, {step_geom}),
        )
    return model, data, history, settle_steps


def _contact_span_m(data, surfaces: set[int]) -> float:
    """Length of the contact patch **in the plane of travel**, metres.

    Not a contact count. MuJoCo resolves a cylinder on a plane as exactly two points at the
    same ``x``, separated along the axle — so a rigid wheel reports two contacts at any load
    and a raw count says the rigid wheel has the bigger patch, which is the opposite of the
    truth. Projecting onto x-z discards that axle-direction separation and measures the thing
    the FEA measures: how far along the surface the wheel is touching. A rigid cylinder gives
    zero by construction, which is the correct answer for a line contact.

    ``surfaces`` is a **single** geom for the edge measurement, not the whole ground. Measured
    against floor and step together, a rigid wheel poised at the corner reports a 66 mm patch
    on the tiny wheel — it is touching the lower ground *and* the upper one, and the span
    between two separate point contacts is not a patch. Envelopment is about how much of one
    surface the wheel wraps.
    """
    points = [data.contact[c].pos for c in range(data.ncon)
              if data.contact[c].geom1 in surfaces or data.contact[c].geom2 in surfaces]
    if len(points) < 2:
        return 0.0
    planar = np.array([(p[0], p[2]) for p in points])
    spread = planar[:, None, :] - planar[None, :, :]
    return float(np.sqrt((spread**2).sum(-1)).max())


def _summarise(spec, rig, history, settle_steps, fit_max_m, rigid) -> StepResult:
    time = history[:, 0]
    x, z = history[:, 1], history[:, 2]
    power, patch, compression = history[:, 3], history[:, 4], history[:, 5]
    step_patch = history[:, 6]
    dt = float(np.median(np.diff(time))) if len(time) > 1 else rig.timestep_s

    # Climbed: standing on the upper surface, past the riser. The threshold is midway between
    # the two resting heights, so it cannot be reached by a bounce on the lower ground.
    on_top = (z > spec.radius_m + 0.5 * rig.step_height_m) & (x > rig.step_x_m)
    climbed = bool(np.any(on_top))

    driving = slice(settle_steps, None)
    energy = float(np.sum(np.maximum(power[driving], 0.0)) * dt)
    distance = float(np.max(x) - x[settle_steps])
    weight = rig.payload_kg * 9.81
    cot = energy / (weight * distance) if distance > 1e-4 else float("nan")

    # Only count the flat patch while the wheel is actually on the ground and rolling: an
    # airborne sample contributes a zero that averages into a smaller patch, and a wheel that
    # bounces more would then appear to have less contact for the wrong reason.
    before_step = ((x < rig.step_x_m - spec.radius_m)
                   & (np.arange(len(x)) >= settle_steps) & (patch > 0.0))
    flat_patch = float(np.mean(patch[before_step])) if np.any(before_step) else 0.0

    # Against the riser and its corner only: how much of the *obstacle* the wheel wraps.
    at_edge = np.abs(x - rig.step_x_m) < spec.radius_m
    edge_patch = float(np.max(step_patch[at_edge])) if np.any(at_edge) else 0.0

    loaded = compression > 1e-6
    beyond = (float(np.mean(compression[loaded] > fit_max_m)) if np.any(loaded) else 0.0)

    return StepResult(
        ok=True,
        climbed=climbed,
        distance_m=distance,
        energy_j=energy,
        cost_of_transport=cot,
        edge_patch_m=edge_patch,
        peak_compression_m=float(np.max(compression)) if not rigid else 0.0,
        fraction_beyond_fit=beyond,
        final_clearance_m=float(z[-1]) - (rig.step_height_m if x[-1] > rig.step_x_m else 0.0),
        mean_flat_patch_m=flat_patch,
        history=history,
    )


def run_step(spec: RingSpec, law: RadialLaw, rig: RigSpec, *, rigid: bool = False,
             fit_max_m: float = float("inf"),
             tangential_law: RadialLaw | None = None,
             tangential_element: TangentialElement | None = None) -> StepResult:
    """Drive one wheel at the step. Returns a typed result; does not raise (invariant 4).

    Args:
        fit_max_m: the deepest indentation the spring law was fitted at. Used only to report
            :attr:`StepResult.fraction_beyond_fit`; it does not clamp anything, because a
            silently clamped force law is a worse lie than an extrapolated one.
        tangential_law: if given, every segment also gets a second in-plane freedom
            (TODO #20). Ignored for ``rigid=True``, which has no segments. Default None keeps
            the radial-only wheel every earlier result was measured on. **The units follow
            the element**: N/m against a slide, N·m/rad against a hinge.
        tangential_element: ``"slide"`` or ``"hinge"``, defaulting to the hinge whenever a law
            is given — see :func:`~wheelopt.rom.mjcf.resolve_tangential_element`. A driven
            wheel wants the hinge; the slide is here to be compared against, not used.
    """
    try:
        _model, _data, history, settle = _simulate(
            spec, law, rig, rigid=rigid, fit_max_m=fit_max_m, tangential_law=tangential_law,
            tangential_element=tangential_element,
        )
    except MissingMuJoCo as exc:
        return StepResult(ok=False, message=str(exc))
    except Exception as exc:  # noqa: BLE001 - a diverged scenario is a result, not a crash
        return StepResult(ok=False, message=f"{type(exc).__name__}: {exc}")
    if not np.all(np.isfinite(history)):
        return StepResult(ok=False, message="the scenario diverged (non-finite state)")
    return _summarise(spec, rig, history, settle, fit_max_m, rigid)


def run_flat(spec: RingSpec, law: RadialLaw, rig: RigSpec, *, rigid: bool = False,
             fit_max_m: float = float("inf"),
             tangential_law: RadialLaw | None = None,
             tangential_element: TangentialElement | None = None) -> StepResult:
    """The same run with no step, for cost of transport and the flat contact patch."""
    return run_step(spec, law, replace(rig, step_height_m=1e-6, step_x_m=1e3),
                    rigid=rigid, fit_max_m=fit_max_m, tangential_law=tangential_law,
                    tangential_element=tangential_element)


def observe_step(spec: RingSpec, law: RadialLaw, rig: RigSpec, observer, *,
                 rigid: bool = False, fit_max_m: float = float("inf"),
                 tangential_law: RadialLaw | None = None,
                 tangential_element: TangentialElement | None = None) -> StepResult:
    """:func:`run_step`, with ``observer(k, model, data)`` called after every integrator step.

    The whole of the renderer's access to the rig. Public so that filming a run cannot drift
    from measuring one: both go through :func:`_simulate`, so the timestep, the damping and the
    second freedom are whatever the measured run uses, and a frame is a picture of the state
    the history row was taken from.

    The observer must not write to ``data`` — it is handed the live arrays for speed, and a
    write would make the run it is filming a different run again, in a way no test would catch.
    """
    try:
        _model, _data, history, settle = _simulate(
            spec, law, rig, rigid=rigid, fit_max_m=fit_max_m, tangential_law=tangential_law,
            tangential_element=tangential_element, observer=observer,
        )
    except MissingMuJoCo as exc:
        return StepResult(ok=False, message=str(exc))
    except Exception as exc:  # noqa: BLE001 - a diverged scenario is a result, not a crash
        return StepResult(ok=False, message=f"{type(exc).__name__}: {exc}")
    if not np.all(np.isfinite(history)):
        return StepResult(ok=False, message="the scenario diverged (non-finite state)")
    return _summarise(spec, rig, history, settle, fit_max_m, rigid)


def default_step_heights_m(spec: RingSpec) -> np.ndarray:
    """The heights :func:`highest_step_climbed` sweeps by default, metres.

    Public so a caller can tell a climb from a **censored** one: a sweep that ends at its own
    ceiling is reporting the ceiling, not the wheel. It ran to ``1.01 R`` until 2026-08-09,
    which is a ceiling a good claw wheel reaches — the R 60 mm claw clears exactly 60 mm — so
    the answer would have been pinned at ``R`` for anything better without a word. It now
    reaches ``1.5 R``, and the claw fails at 70 mm, so the bound is no longer active on the
    design that found it.

    Not open-ended, because a wheel that climbs its own diameter is a result to be doubted
    rather than to be measured more finely, and every extra height costs a run.
    """
    return np.arange(0.01, 1.5 * spec.radius_m + 1e-12, 0.01)


@dataclass(frozen=True, slots=True)
class ClimbProfile:
    """Which step heights a wheel cleared — the "climbs better" signature, in full.

    The maximum alone is not enough, for two reasons the sweep itself creates.

    **The predicate is not monotone.** A wheel can bounce over an obstacle it cannot roll
    over, so a bare maximum cannot tell "cleared everything up to 60 mm" from "failed 40 and
    flew over 60". Only the pattern distinguishes them, and the pattern costs nothing extra
    to keep.

    **The sweep can run out.** A maximum equal to the top of the range is the range's answer,
    not the wheel's; :attr:`censored` says so rather than leaving the caller to compare
    floats.
    """

    heights_m: np.ndarray
    #: True where the wheel got over. Aligned with :attr:`heights_m`.
    climbed: np.ndarray
    #: True where the run itself failed — diverged, or MuJoCo missing. Distinct from "did not
    #: climb", which is a result; this is the absence of one, and averaging the two together
    #: is how a broken sweep comes to look like a poor wheel.
    failed: np.ndarray

    @property
    def tallest_m(self) -> float:
        """Tallest height cleared, metres. Zero if none were."""
        got = self.heights_m[self.climbed]
        return float(np.max(got)) if len(got) else 0.0

    @property
    def censored(self) -> bool:
        """Whether the tallest cleared height is the top of the swept range."""
        return bool(len(self.heights_m)) and self.tallest_m >= float(self.heights_m[-1])

    @property
    def monotone(self) -> bool:
        """Whether every height below the tallest was also cleared.

        False means the wheel bounced over something it could not climb, and the maximum is
        then not a capability — read the pattern.
        """
        below = self.climbed[self.heights_m <= self.tallest_m + 1e-12]
        return bool(np.all(below)) if len(below) else True

    def summary(self) -> str:  # pragma: no cover - display only
        marks = "".join("E" if f else ("#" if c else ".")
                        for c, f in zip(self.climbed, self.failed))
        note = "  <- AT THE SWEEP CEILING; the true value is >= this" if self.censored else ""
        if not self.monotone:
            note += "  <- NOT MONOTONE; it cleared a step it failed below, so read the pattern"
        return (f"{self.tallest_m * 1e3:5.0f} mm  [{marks}] "
                f"{self.heights_m[0] * 1e3:.0f}-{self.heights_m[-1] * 1e3:.0f} mm{note}")


def step_climb_profile(spec: RingSpec, law: RadialLaw, rig: RigSpec, *,
                       rigid: bool = False, fit_max_m: float = float("inf"),
                       heights_m: np.ndarray | None = None,
                       tangential_law: RadialLaw | None = None,
                       tangential_element: TangentialElement | None = None) -> ClimbProfile:
    """Run the whole sweep and keep every outcome. See :class:`ClimbProfile`.

    A sweep rather than a bisection, because bisecting a non-monotone predicate silently
    returns whichever side it happened to land on.

    Resolution is 10 mm, and it is coarse enough to matter: on the R 60 mm claw a **1%** change
    in the fitted radial law — the difference the #12 contact floor makes, 0.3% in peak force
    and 1.2% in ``k(0)`` — moves the answer one whole bucket, 50 mm to 60 mm. Quote it to one
    bucket, and do not read a one-bucket difference between two designs as a ranking.
    """
    heights = default_step_heights_m(spec) if heights_m is None else np.asarray(heights_m)
    climbed = np.zeros(len(heights), dtype=bool)
    failed = np.zeros(len(heights), dtype=bool)
    for i, height in enumerate(heights):
        result = run_step(spec, law, replace(rig, step_height_m=float(height)),
                          rigid=rigid, fit_max_m=fit_max_m,
                          tangential_law=tangential_law,
                          tangential_element=tangential_element)
        climbed[i] = bool(result.ok and result.climbed)
        failed[i] = not result.ok
    return ClimbProfile(heights_m=heights, climbed=climbed, failed=failed)


def highest_step_climbed(spec: RingSpec, law: RadialLaw, rig: RigSpec, *,
                         rigid: bool = False, fit_max_m: float = float("inf"),
                         heights_m: np.ndarray | None = None,
                         tangential_law: RadialLaw | None = None,
                         tangential_element: TangentialElement | None = None) -> float:
    """Tallest step this wheel gets over, metres.

    Kept as the one-number form for callers that only rank. **Prefer
    :func:`step_climb_profile`**: this discards exactly the information needed to tell a climb
    from a bounce, and to tell a real answer from a sweep that ran out of range.
    """
    return step_climb_profile(
        spec, law, rig, rigid=rigid, fit_max_m=fit_max_m, heights_m=heights_m,
        tangential_law=tangential_law, tangential_element=tangential_element,
    ).tallest_m


@dataclass(frozen=True, slots=True)
class Signature:
    """One of the five qualitative checks first-week step 6 asks for.

    Not a metric — a *prediction with a direction*. Each one says "physics requires the
    compliant wheel to beat the rigid one here", and the value of the set is that they are
    hard to satisfy all at once by accident. A rig with the axle turning backwards, or a
    solid cylinder standing in for a ring at half its rotational inertia, fails some of them.

    Lives here rather than in the driver script because there are now two callers
    (``scripts/run_step.py`` and ``scripts/explore.py``) and "the five signatures" must mean
    the same thing to both. Two copies of a judgement is how a passing report and a failing
    one come to disagree about the same run.
    """

    name: str
    compliant: str
    rigid: str
    passed: bool


def loaded_radius_table(spec: RingSpec, law: RadialLaw,
                        loads_n: Sequence[float]) -> list[tuple[float, float]]:
    """Axle height against static load, from the analytic ring. Signature five.

    Bisection on ``F(δ)``, which the ring makes monotone in δ even when the *segment* law is
    not: a deeper indentation can only add contact, never remove it.
    """
    rows = []
    for target in loads_n:
        lo, hi = 0.0, 0.9 * spec.radius_m
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if solve_equilibrium(spec, law, mid).force_n < target:
                lo = mid
            else:
                hi = mid
        rows.append((float(target), spec.radius_m - 0.5 * (lo + hi)))
    return rows


def judge_signatures(
    spec: RingSpec,
    law: RadialLaw,
    *,
    compliant_flat: StepResult,
    compliant_step: StepResult,
    rigid_flat: StepResult,
    rigid_step: StepResult,
    step_height_m: float,
    static_load_n: float,
) -> list[Signature]:
    """The five signatures, in the order ``docs/plan/16-first-week.md`` §6 asks them.

    Note the direction of the fourth: the compliant wheel must cost **more** to transport on
    the flat. Compliance is not free, and a model that showed a soft wheel rolling more
    cheaply than a rigid one would be reporting a bug, not a discovery.
    """
    radii = loaded_radius_table(
        spec, law,
        [0.25 * static_load_n, 0.5 * static_load_n, static_load_n, 2.0 * static_load_n],
    )
    falling = all(later[1] < earlier[1] for earlier, later in pairwise(radii))
    return [
        Signature("patch at the step edge, mm",
                  f"{compliant_step.edge_patch_m * 1e3:.1f}",
                  f"{rigid_step.edge_patch_m * 1e3:.1f}",
                  compliant_step.edge_patch_m > rigid_step.edge_patch_m),
        Signature("mean patch on the flat, mm",
                  f"{compliant_flat.mean_flat_patch_m * 1e3:.1f}",
                  f"{rigid_flat.mean_flat_patch_m * 1e3:.1f}",
                  compliant_flat.mean_flat_patch_m > rigid_flat.mean_flat_patch_m),
        Signature(f"climbed the {step_height_m * 1e3:.0f} mm step",
                  str(compliant_step.climbed), str(rigid_step.climbed),
                  compliant_step.climbed >= rigid_step.climbed),
        Signature("cost of transport, flat",
                  f"{compliant_flat.cost_of_transport:.4f}",
                  f"{rigid_flat.cost_of_transport:.4f}",
                  compliant_flat.cost_of_transport > rigid_flat.cost_of_transport),
        Signature("loaded radius falls with load",
                  f"{radii[0][1] * 1e3:.2f}->{radii[-1][1] * 1e3:.2f} mm", "-", falling),
    ]
