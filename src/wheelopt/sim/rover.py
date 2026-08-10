"""The whole robot: a chassis on four wheels, driven over an obstacle.

Where this sits relative to :mod:`wheelopt.sim.step_climb`. That module is a *single-wheel
test rig* — one wheel on a two-degree-of-freedom carriage, deliberately so, because the
first-week spike was asking whether compliance changes five signatures and a chassis would
have added weight transfer, a second wheel's traction and a suspension geometry, three more
ways for the answer to come out right for the wrong reason. This module adds exactly those
things back, on purpose, once the single-wheel answer is in hand.

Everything dimensional comes from ``configs/robot.yaml`` through
:func:`~wheelopt.platform.load_platform`. Nothing here invents a mass, a wheelbase or a motor
curve — invariant 2 in the form it takes for a vehicle. In particular the drive is the
platform's own ``motor.stall_torque`` and ``motor.no_load_speed``, not
:attr:`~wheelopt.sim.step_climb.RigSpec.torque_ratio`, which is a per-wheel heuristic that
happens to reproduce this platform's sizing rationale and is not the same statement.

What this model is honest about
-------------------------------
**The chassis is a box.** No suspension, no articulation, no differential: the axles are rigid
in the body and the only compliance in the vehicle is the wheels. That is the point — it makes
the wheel the whole story — but it means a four-wheel rover on rigid wheels is a rigid body on
four contact points, and it will climb by pitching and by luck rather than by conforming.

**Skid steer scrubs, and this drives straight.** Four non-steered wheels cannot turn without
sliding sideways, and lateral scrub of a segmented capsule ring is not validated against
anything in this project. :func:`build_rover_mjcf` will happily let you command different
torques left and right; the results would not be trustworthy yet, which is why nothing here
does it and :func:`drive_straight` is the only drive supplied.

**The ROM is planar and a robot is not.** Each ring lies in its own x-z plane and its segments
move radially and in-plane-tangentially. A rover that rolls, or drops one wheel off an edge,
loads a wheel **out of plane**, and there the ring is perfectly rigid. That is a real gap of
the same family as "the shear band does not shear": ``F(delta)`` can be right while the
behaviour is not. It does not bite for a rigid-wheel run, which is why that comes first.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..platform import PlatformSpec, load_platform
from ..rom.mjcf import HUB_MASS_KG
from ..rom.ring import RingSpec
from .step_climb import TPU_LOSS_FACTOR, ring_axle_inertia_kg_m2

__all__ = [
    "RoverResult",
    "RoverSpec",
    "WheelMount",
    "build_rover_mjcf",
    "observe_rover",
    "run_rover",
    "wheel_mounts",
]


@dataclass(frozen=True, slots=True)
class WheelMount:
    """Where one wheel hangs off the chassis, in body coordinates, metres."""

    name: str
    x_m: float
    y_m: float
    #: +1 for the left side, -1 for the right. Only used to name things and, later, to give
    #: the two sides different torques; the geometry is carried by ``y_m``.
    side: int


def wheel_mounts(platform: PlatformSpec) -> list[WheelMount]:
    """The four axle positions implied by the wheelbase and the track.

    Centred on the chassis: front and rear at ``±wheelbase/2`` along x, left and right at
    ``±track/2`` along y. The track is measured wheel-centre to wheel-centre, so half of it
    is where the wheel's mid-plane goes, and the wheels sit **outboard** of the 300 mm body
    on this platform — which is what ``robot.yaml`` says and what
    :meth:`~wheelopt.platform.PlatformSpec.consistency_warnings` checks.

    Ordered front-left, front-right, rear-left, rear-right, and that order is the order of
    every per-wheel array this module returns.
    """
    half_base, half_track = 0.5 * platform.wheelbase_m, 0.5 * platform.track_width_m
    return [
        WheelMount("fl", +half_base, +half_track, +1),
        WheelMount("fr", +half_base, -half_track, -1),
        WheelMount("rl", -half_base, +half_track, +1),
        WheelMount("rr", -half_base, -half_track, -1),
    ]


@dataclass(frozen=True, slots=True)
class RoverSpec:
    """The scenario: what the robot drives over, and how hard it is driven.

    Deliberately thin. Everything about the *robot* lives in
    :class:`~wheelopt.platform.PlatformSpec` and is read from ``configs/robot.yaml``; this
    holds only what the platform cannot know — the obstacle and the run.
    """

    #: Step height, metres. The obstacle is a box whose top face is the upper ground, as in
    #: the single-wheel rig, and its edge is sharp on purpose.
    step_height_m: float = 0.050
    #: Where the step's face sits, metres. Far enough ahead that the robot is rolling
    #: steadily and has settled onto its wheels before it arrives.
    step_x_m: float = 0.80
    #: Ground friction. 1.0 is TPU on concrete — generous, deliberately: a climb that fails
    #: on traction says nothing about the wheel.
    friction: float = 1.0
    duration_s: float = 6.0
    timestep_s: float = 5.0e-4
    #: Seconds of settling before any torque is commanded. The robot is dropped a whisker
    #: onto the floor and must come to rest first, or the run measures a bounce.
    settle_s: float = 0.8
    #: Fraction of the platform's stall torque to command. 1.0 is the motor flat out.
    throttle: float = 1.0
    #: Ground contact stiffness as a solver time constant. A regulariser, never a compliance
    #: model (ADR-0001, invariant 8).
    contact_solref_s: float = 1.0e-3
    loss_factor: float = TPU_LOSS_FACTOR

    def __post_init__(self) -> None:
        if self.timestep_s <= 0:
            raise ValueError("timestep_s must be positive")
        if self.duration_s <= self.settle_s:
            raise ValueError(
                f"duration_s {self.duration_s} must exceed settle_s {self.settle_s}: the "
                "robot is given no torque until it has settled, so a shorter run measures a "
                "stationary robot and every metric below is a metric of nothing"
            )

    def motor_torque_n_m(self, platform: PlatformSpec, axle_rate_rad_s: float) -> float:
        """The platform's own linear torque-speed curve, clipped to forward drive.

        ``tau = throttle * tau_stall * (1 - omega/omega_0)``, floored at zero. Clipped rather
        than allowed negative for the reason the single-wheel rig gives: a motor commanded
        forward and overspeeding would brake, and a braking wheel rolling off a step would
        look like a climb failure caused by the wheel rather than by the drive model.
        """
        fraction = 1.0 - axle_rate_rad_s / platform.no_load_speed_rad_s
        return self.throttle * platform.stall_torque_n_m * min(max(fraction, 0.0), 1.0)


@dataclass(frozen=True, slots=True)
class RoverResult:
    """One run. Never raises — invariant 4 applies to a scenario like any other evaluation."""

    ok: bool
    message: str = ""
    #: Whether the chassis ended up on top of the step.
    climbed: bool = False
    #: How far the chassis centre travelled along x, metres.
    distance_m: float = 0.0
    #: Chassis height above the *upper* ground at the end, metres. Near the ride height means
    #: it is standing on the step; near minus the step height means it never left the floor.
    final_clearance_m: float = 0.0
    #: Largest absolute pitch reached, radians. The signature a single-wheel rig cannot have.
    peak_pitch_rad: float = 0.0
    #: Largest absolute roll reached, radians. Non-zero only if the wheels met the step at
    #: different times, which on a square-on approach means the solver, not the terrain.
    peak_roll_rad: float = 0.0
    #: Whether the chassis box itself touched the step — bellying out rather than climbing.
    chassis_hit_step: bool = False
    #: Mechanical work at the four axles, joules.
    energy_j: float = 0.0
    #: (time, x, z, pitch, roll) per step.
    history: np.ndarray = field(default_factory=lambda: np.empty((0, 5)))


def build_rover_mjcf(
    platform: PlatformSpec,
    scenario: RoverSpec,
    *,
    wheel_radius_m: float,
    wheel_width_m: float,
    wheel_mass_kg: float,
    spec: RingSpec | None = None,
) -> str:
    """MJCF for the whole robot. Rigid wheels only for now.

    ``spec`` is accepted and, when given, only used to give the rigid wheels the **ring's**
    rotational inertia rather than a solid cylinder's. That is the same fairness argument the
    single-wheel rig makes and it matters more here, not less: four wheels' worth of rotational
    inertia is a real share of a 10 kg robot's resistance to acceleration, and a solid cylinder
    has half a ring's. Passing ``None`` gives an honest solid cylinder, which is a different
    vehicle and should be labelled as one.

    The chassis is a box of the platform's own dimensions with the platform's own inertia,
    on a free joint. It is a **contact geom**, not a decoration: a box with 70 mm of ground
    clearance will belly out on a step taller than that, and watching it do so is most of why
    this model exists.
    """
    mounts = wheel_mounts(platform)
    half = (0.5 * platform.chassis_length_m, 0.5 * platform.chassis_width_m,
            0.5 * platform.chassis_height_m)
    ixx, iyy, izz = platform.chassis_inertia_kg_m2
    # Ride height: the axles sit at the wheel's radius, and the chassis centre sits one
    # ground-clearance above the floor plus half its own height. Taken from the platform
    # rather than from the wheel so that a wheel outside the envelope shows up as a chassis
    # sitting wrong, instead of being silently accommodated.
    axle_z = wheel_radius_m
    body_z = platform.ground_clearance_m + half[2]
    # How far the robot could possibly travel, plus a margin, so the step cannot run out.
    reach = scenario.duration_s * platform.no_load_speed_rad_s * wheel_radius_m
    step_half_len = 0.5 * (reach + 2.0 * platform.chassis_length_m)
    step_centre_x = scenario.step_x_m + step_half_len
    start_z = body_z + 0.0005  # a whisker clear, so it settles rather than bangs

    parts = [
        '<mujoco model="rover">',
        '  <compiler angle="radian"/>',
        ('  <option gravity="0 0 -9.81" integrator="implicitfast" '
         f'timestep="{scenario.timestep_s:.9g}"/>'),
        "  <default>",
        (f'    <geom friction="{scenario.friction:.4g} 0.005 0.0001" '
         f'solref="{scenario.contact_solref_s:.9g} 1" solimp="0.98 0.999 0.001" '
         'condim="3"/>'),
        "  </default>",
        "  <visual>",
        '    <global offwidth="1920" offheight="1080"/>',
        "  </visual>",
        "  <asset>",
        ('    <texture name="grid" type="2d" builtin="checker" width="512" height="512" '
         'rgb1="0.24 0.26 0.30" rgb2="0.32 0.34 0.39"/>'),
        ('    <material name="floormat" texture="grid" texrepeat="60 20" '
         'reflectance="0.05"/>'),
        '    <material name="stepmat" rgba="0.55 0.42 0.32 1"/>',
        '    <material name="wheelmat" rgba="0.20 0.65 0.85 1"/>',
        '    <material name="bodymat" rgba="0.85 0.85 0.88 1"/>',
        "  </asset>",
        "  <worldbody>",
        '    <light pos="0 -2 3" dir="0 0.4 -1" directional="true" diffuse="0.7 0.7 0.7"/>',
        '    <light pos="2 -1.5 2" dir="-0.5 0.3 -1" diffuse="0.35 0.35 0.35"/>',
        '    <geom name="floor" type="plane" size="8 3 0.1" material="floormat"/>',
        # The step runs to beyond anything the robot can reach in the time allowed, so it is
        # an upper *ground* rather than a platform. Sized rather than fixed at 2 m: a robot
        # doing 1.15 m/s for 6 s covers 6.9 m, and a 4 m step let it climb, cross, and drive
        # off the far end — after which the final frame shows it back on the floor at exactly
        # the ride height, which reads as "never climbed" and is the opposite of the truth.
        (f'    <geom name="step" type="box" pos="{step_centre_x:.9f} 0 '
         f'{scenario.step_height_m / 2:.9f}" size="{step_half_len:.9f} 1.5 '
         f'{scenario.step_height_m / 2:.9f}" material="stepmat"/>'),
        f'    <body name="chassis" pos="0 0 {start_z:.9f}">',
        '      <freejoint name="root"/>',
        (f'      <inertial pos="{platform.com_offset_m[0]:.9f} '
         f'{platform.com_offset_m[1]:.9f} {platform.com_offset_m[2]:.9f}" '
         f'mass="{platform.chassis_mass_kg:.9f}" '
         f'diaginertia="{ixx:.9g} {iyy:.9g} {izz:.9g}"/>'),
        (f'      <geom name="body" type="box" size="{half[0]:.9f} {half[1]:.9f} '
         f'{half[2]:.9f}" mass="0" density="0" material="bodymat"/>'),
    ]

    # The rigid wheel's inertia about its own axle. Matched to the ring when a spec is given.
    if spec is not None:
        inertia = ring_axle_inertia_kg_m2(spec, 0.5 * wheel_width_m,
                                          (wheel_mass_kg - HUB_MASS_KG)
                                          / max(spec.n_segments, 1))
    else:
        inertia = 0.5 * wheel_mass_kg * wheel_radius_m**2
    # Transverse moment of a wheel of finite width. The rover is *not* planar — it yaws and
    # rolls — so unlike the single-wheel rig this term is integrated rather than merely
    # needed to clear MuJoCo's triangle inequality.
    transverse = 0.5 * inertia + wheel_mass_kg * (0.5 * wheel_width_m) ** 2 / 3.0

    for mount in mounts:
        # The axle sits at the wheel radius above the ground, i.e. below the chassis centre
        # by however much the ride height exceeds it.
        dz = axle_z - body_z
        parts += [
            (f'      <body name="{mount.name}" pos="{mount.x_m:.9f} {mount.y_m:.9f} '
             f'{dz:.9f}">'),
            # +y, so a positive torque rolls the robot toward +x — the same convention the
            # single-wheel rig fixed after driving itself backwards past its own step.
            f'        <joint name="{mount.name}_axle" type="hinge" axis="0 1 0"/>',
            (f'        <inertial pos="0 0 0" mass="{wheel_mass_kg:.9f}" '
             f'diaginertia="{transverse:.12g} {inertia:.12g} {transverse:.12g}"/>'),
            (f'        <geom name="{mount.name}_tyre" type="cylinder" '
             f'euler="1.5707963 0 0" size="{wheel_radius_m:.9f} '
             f'{0.5 * wheel_width_m:.9f}" mass="0" density="0" material="wheelmat"/>'),
            "      </body>",
        ]

    parts += ["    </body>", "  </worldbody>", "  <actuator>"]
    parts += [f'    <motor name="{m.name}_drive" joint="{m.name}_axle" gear="1" '
              'ctrlrange="-100 100"/>' for m in mounts]
    parts += ["  </actuator>", "</mujoco>"]
    return "\n".join(parts)


def observe_rover(
    platform: PlatformSpec,
    scenario: RoverSpec,
    observer=None,
    *,
    wheel_radius_m: float,
    wheel_width_m: float,
    wheel_mass_kg: float,
    spec: RingSpec | None = None,
) -> RoverResult:
    """Drive the robot straight at the step. ``observer(k, model, data)`` after every step.

    The observer hook is the same contract :func:`~wheelopt.sim.step_climb.observe_step` uses,
    and for the same reason: ``scripts/render_rover.py`` must film *this* run rather than a
    second copy of it, because a renderer showing a different simulation checks nothing.
    """
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - environment dependent
        return RoverResult(ok=False,
                           message=f"MuJoCo is not installed; pip install -e '.[sim]': {exc}")

    try:
        xml = build_rover_mjcf(platform, scenario, wheel_radius_m=wheel_radius_m,
                               wheel_width_m=wheel_width_m, wheel_mass_kg=wheel_mass_kg,
                               spec=spec)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
    except Exception as exc:  # noqa: BLE001 - a bad model is a result, not a crash
        return RoverResult(ok=False, message=f"{type(exc).__name__}: {exc}")

    mounts = wheel_mounts(platform)
    axle_dofs = np.array(
        [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                            f"{m.name}_axle")] for m in mounts],
        dtype=np.int64,
    )
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    body_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "body")
    step_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")

    n_steps = int(scenario.duration_s / scenario.timestep_s)
    settle_steps = int(scenario.settle_s / scenario.timestep_s)
    history = np.zeros((n_steps, 5), dtype=np.float64)
    energy = 0.0
    hit_step = False

    try:
        for k in range(n_steps):
            rates = data.qvel[axle_dofs]
            if k < settle_steps:
                torques = np.zeros(len(mounts))
            else:
                torques = np.array([scenario.motor_torque_n_m(platform, float(w))
                                    for w in rates])
            data.ctrl[:] = torques
            mujoco.mj_step(model, data)
            if observer is not None:
                observer(k, model, data)

            energy += float(np.sum(torques * data.qvel[axle_dofs])) * scenario.timestep_s
            # Orientation as a rotation matrix, so pitch and roll are read without a
            # quaternion convention to get wrong. Row-major 3x3; column 0 is the body x axis.
            rot = data.xmat[chassis].reshape(3, 3)
            pitch = float(np.arctan2(-rot[2, 0], np.hypot(rot[2, 1], rot[2, 2])))
            roll = float(np.arctan2(rot[2, 1], rot[2, 2]))
            history[k] = (data.time, float(data.xpos[chassis, 0]),
                          float(data.xpos[chassis, 2]), pitch, roll)
            if not hit_step:
                for c in range(data.ncon):
                    pair = (data.contact[c].geom1, data.contact[c].geom2)
                    if body_geom in pair and step_geom in pair:
                        hit_step = True
                        break
    except Exception as exc:  # noqa: BLE001 - a diverged scenario is a result
        return RoverResult(ok=False, message=f"{type(exc).__name__}: {exc}")

    if not np.all(np.isfinite(history)):
        return RoverResult(ok=False, message="the scenario diverged (non-finite state)")
    return _summarise(platform, scenario, history, settle_steps, energy, hit_step)


def _summarise(platform, scenario, history, settle_steps, energy_j, hit_step) -> RoverResult:
    """Turn the history into the handful of numbers worth quoting."""
    x, z = history[:, 1], history[:, 2]
    driving = slice(settle_steps, None)
    ride = platform.ground_clearance_m + 0.5 * platform.chassis_height_m
    clearance = float(z[-1]) - scenario.step_height_m

    # Climbed means the **whole body** got onto the upper ground and settled there, not that
    # some part of it got past the face. Both halves are load-bearing, and each was wrong on
    # its own in the first version: a robot nose-up against a 100 mm step is past `step_x`
    # and 113 mm above the upper ground, so the x test alone passes it; and a robot that has
    # climbed sits at its ride height, so a loose height threshold passes the leaner too.
    # Require the chassis centre a full half-length beyond the face, and within a fifth of
    # its ride height of where it would stand.
    on_top = ((x > scenario.step_x_m + 0.5 * platform.chassis_length_m)
              & (np.abs(z - scenario.step_height_m - ride) < 0.2 * ride))
    climbed = bool(np.any(on_top[driving]))
    return RoverResult(
        ok=True,
        climbed=climbed,
        distance_m=float(np.max(x) - x[settle_steps]),
        final_clearance_m=clearance,
        peak_pitch_rad=float(np.max(np.abs(history[driving, 3]))),
        peak_roll_rad=float(np.max(np.abs(history[driving, 4]))),
        chassis_hit_step=bool(hit_step),
        energy_j=float(energy_j),
        history=history,
    )


def run_rover(platform: PlatformSpec | None = None, scenario: RoverSpec | None = None, *,
              wheel_radius_m: float = 0.085, wheel_width_m: float = 0.030,
              wheel_mass_kg: float = 0.30, spec: RingSpec | None = None) -> RoverResult:
    """Convenience entry point: load the platform, run one scenario, return the result."""
    return observe_rover(platform or load_platform(), scenario or RoverSpec(),
                         wheel_radius_m=wheel_radius_m, wheel_width_m=wheel_width_m,
                         wheel_mass_kg=wheel_mass_kg, spec=spec)
