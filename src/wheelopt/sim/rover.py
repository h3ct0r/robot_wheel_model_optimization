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

**Skid steer scrubs, and this drives straight — but the gap is narrower than it reads**
(re-scoped 2026-08-11, ``TODO.md`` #38). The ring is planar, and for a *wide* claw that is a
defensible structural approximation rather than a hole: a claw's out-of-plane stiffness is
``(w/t)²`` times its tangential one — ~72x on the R 60 family, a closed form on
``WheelParams.lateral_stiffness_ratio`` — so laterally the wheel is quasi-rigid and a
skid-steer turn is a **friction** problem, which MuJoCo's Coulomb cone does model. What is
genuinely unvalidated is narrower: the discrete tips scrubbing sideways (patch-level
behaviour, not structure), and any design where ``w/t`` collapses. Turning scenarios stay
out of this module until #38's checks land — a 3-D FEA lateral case for the structure, and a
printed-wheel spin-in-place test for the friction (ADR-0008 makes hardware the ground
truth). :func:`drive_straight` is the only drive supplied, deliberately, until then.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from ..platform import PlatformSpec, load_platform
from ..rom.mjcf import (
    HUB_MASS_KG,
    TangentialElement,
    coupling_tendons,
    hinge_arm_m,
    resolve_tangential_element,
    ring_bodies,
    stable_timestep_s,
    tangential_damping,
)
from ..rom.ring import RadialLaw, RingSpec, TipEquivalentLaw, symmetric_force_n
from .step_climb import (
    TPU_LOSS_FACTOR,
    ring_axle_inertia_kg_m2,
    segment_damping_n_s_per_m,
)

__all__ = [
    "RoverResult",
    "RoverSpec",
    "WheelMount",
    "build_rover_mjcf",
    "observe_rover",
    "run_rover",
    "wheel_mounts",
]


#: Collision bitmasks. The single-wheel rig needed none of this because its carriage is not a
#: contact geom; the rover's chassis **is** one, and that turns an ordinary MuJoCo rule into a
#: trap. MuJoCo excludes contacts between a body and its *parent*, so a rigid tyre — a child of
#: the chassis — never touches it. A segment's parent is the wheel and the wheel's parent is the
#: chassis, which is a *grandparent* relationship and is **not** excluded. With R 60 mm wheels
#: under 70 mm of ground clearance the top half of every wheel sits inside the chassis box, so
#: every segment jammed against it and the axles could not turn: 28 contacts, 0.01 rad/s, and a
#: robot that had settled to exactly the right ride height with exactly the right compression.
#:
#: So the segments get their own contype bit and the ground carries both. Segment (2, 0) meets
#: floor (1, 3) because 2 & 3, meets no other segment because 2 & 0, and meets the chassis
#: (1, 1) not at all because 2 & 1 and 1 & 0 are both zero. Rigid wheels stay on the default
#: (1, 1) and are unaffected.
SEGMENT_CONTYPE = 2
GROUND_CONAFFINITY = 3

#: A segment counts as *sharing* the load when it is compressed at least this fraction of the
#: deepest segment on its own wheel. Used only by ``RoverResult.multi_contact_fraction``.
#:
#: A share rather than an absolute depth, because the quantity is "how many claws carry this
#: wheel" and that is scale-free — the same wheel under a step and on the flat should be judged
#: the same way. It also avoids the failure that produced the first version of this number: at
#: an absolute 1 um a claw still ringing down after leaving the ground counts as in contact,
#: which read 70% of a flat run against a static ring that is on one claw for all but the
#: half-pitch crossing.
MULTI_CONTACT_SHARE = 0.10

#: Rotation taking the CAD wheel frame to the MuJoCo wheel frame, radians about x.
#:
#: Measured rather than reasoned (the sign is a coin flip and both look plausible on a
#: symmetric wheel): build123d lays the wheel in its **x-y** plane with the axle along z and
#: the part spanning +/-width/2 in z, in millimetres, centroid exactly at the origin. MuJoCo's
#: wheels lie in **x-z** with the axle along y and segment 0 at the bottom, -z. A rotation of
#: +pi/2 about x sends CAD -y to MuJoCo -z, so a design with ``spoke_phase_deg = -90`` — which
#: puts a claw tip at CAD -90 deg, verified off the exported triangles — lands that tip under
#: the contact point where the ring's segment 0 is. The opposite sign puts it on top, which on
#: a twelve-fold wheel is a half-pitch error that reads as "fine" in a still frame.
CAD_TO_WHEEL_EULER_X = 1.5707963267948966

#: Millimetres to metres, for the mesh asset's ``scale``.
CAD_MESH_SCALE = 0.001

#: Colour of the ring's segment capsules — **the physics**, the thing to look at. Deep amber,
#: chosen to read against the grey floor, the brown step and the translucent overlay, and to
#: be nobody's default. It was previously nothing at all: `ring_bodies` emitted no ``rgba``, so
#: the capsules took MuJoCo's built-in geom colour, which under this scene's lighting comes out
#: a washed olive-green. The one colour in the render that had not been chosen was the one
#: carrying the result.
SEGMENT_RGBA = (0.93, 0.55, 0.13, 1.0)

#: Colour of the translucent CAD overlay — **decoration**, the shape the physics stands for.
#: Neutral grey on purpose: it should recede behind the capsules rather than compete with them.
CAD_OVERLAY_RGBA = (0.62, 0.63, 0.67, 0.40)

#: Colour of the chassis STL, when one is drawn. Near-solid, because unlike the wheel overlay
#: it does not stand in front of any physics — the box it replaces visually is faded instead
#: (`CHASSIS_BOX_GHOST_RGBA`), so the contact proxy stays visible through the shell.
CHASSIS_MESH_RGBA = (0.80, 0.80, 0.84, 0.95)

#: The chassis box when a mesh is drawn over it: a faint ghost. Still the **contact geom** —
#: fading it is a render choice, hiding it entirely would hide the surface a belly strike
#: actually happens on, which is most of why the box exists.
CHASSIS_BOX_GHOST_RGBA = (0.85, 0.85, 0.88, 0.12)

#: Where the axle line sits in ``configs/pipebot_simplified.stl``'s own frame, millimetres,
#: as ``(lateral, vertical, longitudinal)`` — the model's axes are (x lateral, z vertical,
#: y longitudinal), spanning 233 × 153 × 425 mm. Measured, not read from anywhere: the model
#: carries explicit **axle stubs** (r 7.5 cylinders, confirmed in the STEP), whose axes sit
#: at y = 103.48/353.51 — wheelbase 250.03 against the platform's 250, the cross-check —
#: midpoint 228.49, all at z = 24.50. Lateral midline −0.50 (stub tips at +116/−117, plate
#: faces at +97/−98, both symmetric about it).
CHASSIS_MESH_AXLE_MM = (-0.50, 24.50, 228.49)

#: Rotation taking the mesh frame to the chassis body frame, MuJoCo w-x-y-z. The mesh axes
#: are (lateral, longitudinal, vertical); the body's are (forward, left, up). Front is the
#: mesh's **low**-y end — the end with the ~105 mm overhang, the nose — so the map is
#: body_x = −y, body_y = +x, body_z = +z: a +90° rotation about z (det +1, verified
#: numerically against the stub positions).
CHASSIS_MESH_QUAT = (0.7071067811865476, 0.0, 0.0, 0.7071067811865476)

#: The chassis as collision **primitives** read off ``pipebot_simplified.stl`` — the
#: alternative to the calibrated box (``chassis_collision="primitives"``). Each entry is
#: ``(name, kind, centre, size)`` in the MESH frame, millimetres, so the one registration
#: (`CHASSIS_MESH_AXLE_MM`) places physics and picture alike. Measured 2026-08-11:
#:
#: - ``shell``: the pipe is an exact r 72.5 cylinder about (x 0, z 82), y 54–424.
#: - ``nose``: the front cap is an exact r 55 hemisphere centred at (0, 54, 82) — 10 673
#:   mesh vertices within 1 mm. A dome, not a wall: it can ride a step edge up.
#: - ``plate_*``: the outer bracket plates at |x| ≈ 97.25, one per axle station, y spans
#:   79–155 and 302–378 (asymmetric about their stations, mirror-symmetric about the body),
#:   z 1.5–47.5 — their pointed bottom corners are the lowest material on the machine,
#:   23 mm BELOW the axle line. The box these replace has no belly there at all.
#: - ``web_*``: the matching inner walls at |x| ≈ 52.5, same spans below the pipe.
#:
#: Boxes over-fill the plates' chamfered corners (a box bottom edge runs the full length
#: where the real plate rises to a point) — conservative, and stated rather than hidden.
#: The axle stubs are deliberately absent: they live inside the wheels' hubs, and the wheel
#: bodies are children of the chassis, which MuJoCo never collides with their parent.
CHASSIS_PRIMITIVES_MM = (
    ("shell", "cylinder", (0.0, 239.0, 82.0), (72.5, 185.0)),
    ("nose", "sphere", (0.0, 54.0, 82.0), (55.0,)),
    # (name, "box", centre (lat, long, vert), HALF sizes (lat, long, vert)); mesh +x is
    # body +y = the robot's LEFT, so the l/r in the names is the body's side, not the sign.
    ("plate_fl", "box", (+97.25, 117.0, 24.5), (0.75, 38.0, 23.0)),
    ("plate_fr", "box", (-97.25, 117.0, 24.5), (0.75, 38.0, 23.0)),
    ("plate_rl", "box", (+97.25, 340.0, 24.5), (0.75, 38.0, 23.0)),
    ("plate_rr", "box", (-97.25, 340.0, 24.5), (0.75, 38.0, 23.0)),
    ("web_fl", "box", (+52.5, 117.0, 24.5), (1.0, 38.0, 23.0)),
    ("web_fr", "box", (-52.5, 117.0, 24.5), (1.0, 38.0, 23.0)),
    ("web_rl", "box", (+52.5, 340.0, 24.5), (1.0, 38.0, 23.0)),
    ("web_rr", "box", (-52.5, 340.0, 24.5), (1.0, 38.0, 23.0)),
)


def _chassis_collision_primitives(dz_axle_m: float, *, ghosted: bool) -> list[str]:
    """Geom lines for `CHASSIS_PRIMITIVES_MM`, transformed mesh frame → body frame.

    Same translation the mesh geom uses — the axle line is the registration for both — so
    the primitives sit exactly under the drawn shell. ``ghosted`` fades them to the box's
    ghost colour when the shell is drawn over them; without a shell they are the visible
    robot and keep the body colour.
    """
    lat_mid, z_axle, long_mid = CHASSIS_MESH_AXLE_MM
    rgba = CHASSIS_BOX_GHOST_RGBA if ghosted else (0.85, 0.85, 0.88, 1.0)
    common = ('mass="0" density="0" '
              f'rgba="{rgba[0]:.3f} {rgba[1]:.3f} {rgba[2]:.3f} {rgba[3]:.3f}"')
    lines = []
    for name, kind, centre, size in CHASSIS_PRIMITIVES_MM:
        pos = (f'pos="{(long_mid - centre[1]) * 1e-3:.9f} '
               f'{(centre[0] - lat_mid) * 1e-3:.9f} '
               f'{dz_axle_m + (centre[2] - z_axle) * 1e-3:.9f}"')
        if kind == "cylinder":
            radius, half_len = size
            # MuJoCo's cylinder axis is local z; Ry(+90°) lays it along body x.
            lines.append(f'      <geom name="chassis_col_{name}" type="cylinder" {pos} '
                         f'euler="0 1.5707963 0" size="{radius * 1e-3:.9f} '
                         f'{half_len * 1e-3:.9f}" {common}/>')
        elif kind == "sphere":
            lines.append(f'      <geom name="chassis_col_{name}" type="sphere" {pos} '
                         f'size="{size[0] * 1e-3:.9f}" {common}/>')
        else:
            half_lat, half_long, half_vert = size
            lines.append(f'      <geom name="chassis_col_{name}" type="box" {pos} '
                         f'size="{half_long * 1e-3:.9f} {half_lat * 1e-3:.9f} '
                         f'{half_vert * 1e-3:.9f}" {common}/>')
    return lines


@dataclass(frozen=True, slots=True)
class WheelMount:
    """Where one wheel hangs off the chassis, in body coordinates, metres."""

    name: str
    x_m: float
    y_m: float
    #: +1 for the left side, -1 for the right. Only used to name things and, later, to give
    #: the two sides different torques; the geometry is carried by ``y_m``.
    side: int


def wheel_mounts(platform: PlatformSpec,
                 wheel_width_m: float | None = None) -> list[WheelMount]:
    """The four axle positions: ``±wheelbase/2`` along x, ``±track/2`` along y.

    **The track depends on the wheel** (2026-08-11): candidate wheels mount externally,
    inner face against the side plates, so ``wheel_width_m`` sets the track through
    :meth:`~wheelopt.platform.PlatformSpec.track_for` — a wider wheel stands further out
    and widens its own support polygon. ``None`` falls back to the stored
    ``track_width_m``, which is the ORIGINAL r 22.5 wheels' tucked-under track, kept as
    the reference configuration.

    Ordered front-left, front-right, rear-left, rear-right, and that order is the order of
    every per-wheel array this module returns.
    """
    track = (platform.track_width_m if wheel_width_m is None
             else platform.track_for(wheel_width_m))
    half_base, half_track = 0.5 * platform.wheelbase_m, 0.5 * track
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
    #:
    #: **Exactly 0 means flat ground**, and it is a scenario rather than a degenerate step:
    #: no box is emitted at all, and the run measures ride harshness instead of a climb
    #: (`08-metrics.md` S5, objective 3). A bandless wheel runs on discrete tips, so it is a
    #: polygon and the axle rises and falls once per tip — the cost compliance is supposed to
    #: buy back, and the axis on which a wheel with too few claws is supposed to lose. A
    #: zero-height *box* would be a MuJoCo error, so this is checked rather than passed on.
    step_height_m: float = 0.050
    #: Where the step's face sits, metres. Far enough ahead that the robot is rolling
    #: steadily and has settled onto its wheels before it arrives.
    step_x_m: float = 0.80
    #: Scenario **S7**: peak-to-trough height of a sinusoidal corrugation on the ground,
    #: metres. Zero is smooth ground and emits no terrain at all.
    #:
    #: This exists because flat ground cannot show what compliance is *for*. A smooth rigid
    #: cylinder on a smooth plane reads 0.00 m/s² of chassis acceleration and cannot be beaten,
    #: so every compliant wheel is scored on how close it gets back to a wheel nobody can
    #: print (`TODO.md` #33). On a corrugation a rigid wheel must follow the ground and a
    #: compliant one need not, which is where the sign is supposed to reverse.
    washboard_amplitude_m: float = 0.0
    #: Wavelength of that corrugation, metres. The other half of S7's sweep: harshness is a
    #: resonance question, so amplitude alone says nothing — what matters is the wavelength
    #: against the wheel's own diameter and the tip-passing pitch.
    washboard_wavelength_m: float = 0.10
    #: Ground friction. 1.0 is TPU on concrete — generous, deliberately: a climb that fails
    #: on traction says nothing about the wheel.
    friction: float = 1.0
    #: Yaw of the robot relative to the step's face, degrees. `08-metrics.md` randomises S1
    #: over ±15°, because a step met square is the easiest case and a real approach is not.
    #: The robot still drives straight — this rotates its heading, it does not steer, so it
    #: stays clear of the unvalidated skid-steer scrub.
    approach_deg: float = 0.0
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
        if self.step_height_m < 0.0:
            raise ValueError(
                f"step_height_m {self.step_height_m} is negative: a trench is a different "
                "scenario (S3 gap) and nothing here models one. Use exactly 0 for flat "
                "ground, which is the harshness case"
            )
        if abs(self.approach_deg) >= 90.0:
            raise ValueError(
                f"approach_deg {self.approach_deg} is at or past 90 deg: the robot would "
                "drive along the step's face rather than at it, and every metric below "
                "would be about a robot that never met the obstacle"
            )
        if self.duration_s <= self.settle_s:
            raise ValueError(
                f"duration_s {self.duration_s} must exceed settle_s {self.settle_s}: the "
                "robot is given no torque until it has settled, so a shorter run measures a "
                "stationary robot and every metric below is a metric of nothing"
            )
        if self.washboard_amplitude_m < 0.0:
            raise ValueError("washboard_amplitude_m must be non-negative")
        if self.washboard_amplitude_m > 0.0:
            if self.washboard_wavelength_m <= 0.0:
                raise ValueError("a washboard needs a positive wavelength")
            if self.step_height_m > 0.0:
                raise ValueError(
                    "a step and a washboard together is a scenario nothing defines: S1 is "
                    "the step and S7 is the corrugation, and a harshness number taken while "
                    "climbing would be about the step (see the flat-ground note on "
                    "harshness_rms_m_s2). Run them separately"
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
    #: **Objective 5** (`08-metrics.md`): worst-case margin to static tip-over, unitless.
    #: ``1 − max(|pitch|/pitch_crit, |roll|/roll_crit)`` over the driving phase, with the
    #: critical angles from :meth:`~wheelopt.platform.PlatformSpec.tipover_angles_rad`.
    #: 1.0 is a run that never pitched or rolled at all; 0.0 is the CG passing vertically
    #: over a wheel contact line; negative is past it. **Maximise the minimum** (CVaR'd over
    #: seeds), because stability is a worst-moment property — a run that averages upright and
    #: tips once has tipped.
    #:
    #: Static reference, deliberately: the angles are where the CG crosses the support
    #: polygon at rest, and a fast robot can tip earlier (momentum) or hang on later (a wheel
    #: pressed against a riser). The margin is a *score against a fixed yardstick*, which is
    #: what an objective needs — comparable across designs — not a tip-over predictor.
    stability_margin: float = 1.0
    #: Mechanical work at the four axles, joules.
    energy_j: float = 0.0
    #: RMS vertical chassis acceleration over the steady window, m/s². **Objective 3**
    #: (`08-metrics.md`), and the axis on which a wheel with too few tips is meant to lose.
    #:
    #: Read as an accelerometer would: a chassis standing still reads 0, not −g, because this
    #: is the *net* acceleration the solver produced and gravity is balanced by the contact
    #: force. It is quoted over a **steady window** (the second half of the driving phase), so
    #: the launch transient — which is about the motor, not the wheel — is excluded. On a
    #: climb run it is dominated by the obstacle and says nothing about the wheel; the
    #: scenario for it is flat ground, ``step_height_m = 0``.
    harshness_rms_m_s2: float = 0.0
    #: Mean forward speed over the same steady window, m/s. Harshness scales with speed, so
    #: the two are only comparable together — a wheel that is smooth because it is slow has
    #: not won anything.
    mean_speed_m_s: float = 0.0
    #: Tip-passing frequency implied by that speed, Hz, or 0 for a wheel with no tip count.
    #: ``n_segments * v / (2πR)``. The forcing frequency behind the harshness number.
    tip_frequency_hz: float = 0.0
    #: Fraction of the driving phase in which **some wheel had more than one segment
    #: compressed**, 0 to 1. Zero for rigid wheels, which have no segments.
    #:
    #: This is the ROM's validity envelope, measured instead of assumed (``TODO.md`` #31).
    #: A bandless claw ring reproduces the FEA to 0.036% while one claw carries the load;
    #: past second-claw engagement the radial slide reads **+74.7%** and the root hinge
    #: **−45.6%** against the same FEA, and no choice of spring law repairs that. So a
    #: compliant run's number is worth what this fraction says it is worth, and quoting one
    #: without it is quoting a model outside the range anybody checked.
    multi_contact_fraction: float = 0.0
    #: Deepest compression any single segment reached, metres. The other half of the same
    #: question: how far past its measured range the segment law was asked to extrapolate.
    peak_compression_m: float = 0.0
    #: (time, x, z, pitch, roll, vertical acceleration) per step.
    history: np.ndarray = field(default_factory=lambda: np.empty((0, 6)))


#: Boxes per washboard wavelength. Eight puts the sampling error at
#: ``A/2 · (1 − cos(π/8))`` ≈ 3.8% of the half-amplitude — under half a millimetre on the
#: tallest corrugation in S7's sweep — while keeping the geom count in the hundreds.
WASHBOARD_BOXES_PER_WAVE = 8


def _washboard_geoms(scenario: RoverSpec, reach_m: float, half_width_m: float) -> list[str]:
    """S7's corrugation as a strip of boxes whose tops sample the sinusoid.

    Boxes rather than a heightfield, deliberately. An ``hfield`` asset wants its elevation
    data supplied *after* compilation through ``model.hfield_data``, and this module's whole
    contract is that the returned XML string **is** the model — a model that is wrong until a
    second call patches it is exactly the split-brain the string exists to prevent. Boxes are
    self-contained, and at eight per wavelength the top-face stair-step is 3.8% of the
    half-amplitude, far below the tread features a real washboard road carries.

    The first trough sits at ``step_x_m`` and the strip runs to the robot's reach plus a
    body length, for the same reason the step does: terrain that runs out mid-run turns the
    end of the world into the tallest obstacle in the scenario. Starting at a trough rather
    than a crest means the leading face of the first box is at floor level — the entry to the
    washboard is a ramp of boxes, not a step edge, so the transient belongs to the
    corrugation rather than to its own doorstep.
    """
    amplitude = scenario.washboard_amplitude_m
    if amplitude <= 0.0:
        return []
    wavelength = scenario.washboard_wavelength_m
    dx = wavelength / WASHBOARD_BOXES_PER_WAVE
    start = scenario.step_x_m
    length = reach_m + 0.4  # past anything the robot can touch
    n_boxes = int(np.ceil(length / dx))
    geoms = []
    for i in range(n_boxes):
        centre_x = start + (i + 0.5) * dx
        # Height of the sinusoid at the box centre, trough at the strip's start. The box top
        # is that height; its bottom is sunk into the floor so there is no underside edge.
        top = 0.5 * amplitude * (1.0 - np.cos(2.0 * np.pi * (centre_x - start) / wavelength))
        if top < 5.0e-4:
            continue  # a sub-half-millimetre sliver flickers in the contact solver
        half_h = 0.5 * (top + 0.01)   # sunk 10 mm below grade
        geoms.append(
            f'    <geom name="wash{i}" type="box" pos="{centre_x:.9f} 0 '
            f'{top - half_h:.9f}" size="{0.5 * dx:.9f} {half_width_m:.9f} '
            f'{half_h:.9f}" contype="1" conaffinity="{GROUND_CONAFFINITY}" '
            'material="stepmat"/>'
        )
    return geoms


def build_rover_mjcf(
    platform: PlatformSpec,
    scenario: RoverSpec,
    *,
    wheel_radius_m: float,
    wheel_width_m: float,
    wheel_mass_kg: float,
    spec: RingSpec | None = None,
    segmented: bool = False,
    tangential: TangentialElement | None = None,
    radial_damping: float = 0.0,
    tangential_damping_c: float = 0.0,
    visual_mesh: Path | str | None = None,
    visual_rgba: tuple[float, float, float, float] = CAD_OVERLAY_RGBA,
    chassis_mesh: Path | str | None = None,
    chassis_collision: str = "box",
) -> str:
    """MJCF for the whole robot, on rigid wheels or on four segmented rings.

    ``spec`` alone gives **rigid** wheels carrying the *ring's* rotational inertia rather than
    a solid cylinder's — a solid cylinder has half a ring's, and four wheels' worth is a real
    share of a 10 kg robot's resistance to acceleration, so the fairness argument the
    single-wheel rig makes matters more here rather than less. ``spec=None`` gives an honest
    solid cylinder, which is a different vehicle and should be labelled as one.

    ``segmented=True`` additionally replaces each cylinder with the ring itself: a hub and
    ``spec.n_segments`` bodies from :func:`~wheelopt.rom.mjcf.ring_bodies`, per wheel,
    namespaced by ``prefix``. The whole wheel still weighs ``wheel_mass_kg`` — the per-segment
    mass is derived from it — so the two wheel models remain comparable.

    **Read a segmented rover as a picture, not as a measurement** (``docs/plan/TODO.md`` #30
    and #31). Two limits bite here that never bit the single-wheel rig. The ring is **planar**:
    each one lies in its own x-z plane and has no out-of-plane freedom, so a wheel loaded by
    chassis roll, by an angled approach, or by dropping off an edge is perfectly rigid — and a
    rover does all three constantly. And above second-claw engagement the ring's element is
    unvalidated in either form, straddling the FEA by +62.7% with a radial slide and −49.5%
    with a root hinge. Watching twelve claws fold over a step is still worth having: it is the
    class of wrongness that five seconds of video catches and no metric flags.

    ``visual_mesh`` draws this design's **real CAD geometry** over each wheel, translucent, as
    a decoration only. It is the opposite of a modelling change and must stay that way: the
    geom carries ``contype=0 conaffinity=0 mass=0 density=0``, so it collides with nothing and
    weighs nothing, and every number the run reports is identical with and without it. The
    point is to see the ring's capsules — which *are* the physics — against the shape they
    stand for, and to watch which claw is actually in contact. Handing this mesh to a collision
    system instead is precisely what ADR-0002 exists to prevent.

    The chassis is a box of the platform's own dimensions with the platform's own inertia,
    on a free joint. It is a **contact geom**, not a decoration: a box with 70 mm of ground
    clearance will belly out on a step taller than that, and watching it do so is most of why
    this model exists.

    ``chassis_collision`` picks which chassis collides. ``"box"`` (the default) is the
    calibrated flat-bellied box above. ``"primitives"`` replaces it with the shapes read off
    the simplified model (`CHASSIS_PRIMITIVES_MM`): the pipe cylinder, the hemispherical
    nose, and the bracket plates whose points reach 23 mm below the axle line. **This is a
    physics change, not a rendering one** — the primitives belly out ~30 mm earlier and
    lower than the box, and the dome nose can ride a step edge the box's flat face
    hard-stops against. It exists so the two chassis models can be compared on the same
    runs; neither is validated against the machine yet (the plates' low points are in the
    CAD, unconfirmed on hardware).

    ``chassis_mesh`` draws the robot's **real shell** (``configs/pipebot_simplified.stl``)
    over that box, under the same contract as ``visual_mesh``: zero mass, zero collision,
    every number byte-identical with and without it. The box stays the contact geom and is
    faded to a ghost rather than removed, because it is the surface a belly strike actually
    happens on. The mesh must never become the physics: MuJoCo collides a mesh by its convex
    hull, which would replace the measured flat belly with the hull of a pipe shell. The mesh
    is placed by its **axle stubs** (`CHASSIS_MESH_AXLE_MM`), not its bounding box — so where
    the shell disagrees with the box, the shell is telling the truth: its overhang is
    asymmetric, ~105 mm at the nose against the box's centred 88, and its axle stubs run
    from the side plates into the wheels, which mount externally on them.
    """
    if segmented and spec is None:
        raise ValueError("segmented wheels need a RingSpec; there is nothing to build")
    if chassis_collision not in ("box", "primitives"):
        raise ValueError(
            f"chassis_collision {chassis_collision!r} is neither 'box' nor 'primitives'; "
            "honouring it silently would simulate a chassis nobody asked for"
        )
    segment_mass_kg = (
        (wheel_mass_kg - HUB_MASS_KG) / max(spec.n_segments, 1) if spec is not None else 0.0
    )
    if segmented and segment_mass_kg <= 0.0:
        raise ValueError(
            f"wheel_mass_kg {wheel_mass_kg:.4f} does not exceed the {HUB_MASS_KG:.4f} kg hub, "
            "so the segments would have zero or negative mass"
        )
    mounts = wheel_mounts(platform, wheel_width_m)
    half = (0.5 * platform.chassis_length_m, 0.5 * platform.chassis_width_m,
            0.5 * platform.chassis_height_m)
    ixx, iyy, izz = platform.chassis_inertia_kg_m2
    # Ride height: the axles sit at the wheel's radius, and the chassis centre sits one
    # ground-clearance above the floor plus half its own height. Taken from the platform
    # rather than from the wheel so that a wheel outside the envelope shows up as a chassis
    # sitting wrong, instead of being silently accommodated.
    axle_z = wheel_radius_m
    # Wheel-dependent, not the stated constant: the belly rides a fixed 7.5 mm above the
    # axle line (bracket geometry), so a bigger wheel lifts the chassis one-for-one. The
    # constant-clearance version put an R 85 axle 55 mm above its own belly.
    body_z = platform.ground_clearance_for(wheel_radius_m) + half[2]
    # How far the robot could possibly travel, plus a margin, so the step cannot run out.
    reach = scenario.duration_s * platform.no_load_speed_rad_s * wheel_radius_m
    step_half_len = 0.5 * (reach + 2.0 * platform.chassis_length_m)
    step_centre_x = scenario.step_x_m + step_half_len
    # ...and how far *sideways*, which is not zero the moment the approach is angled. At 15
    # degrees a 6.9 m run drifts 1.8 m off centre, so a step fixed at the old 1.5 m half-width
    # would have let the robot climb it and then drive off the side — the same failure as the
    # step being shorter than the run, in the axis nobody was looking at.
    yaw = np.deg2rad(scenario.approach_deg)
    step_half_width = max(1.5, abs(reach * np.sin(yaw)) + platform.chassis_length_m)
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
        *([] if visual_mesh is None else [
            (f'    <material name="cadmat" rgba="{visual_rgba[0]:.3f} {visual_rgba[1]:.3f} '
             f'{visual_rgba[2]:.3f} {visual_rgba[3]:.3f}"/>'),
            # Absolute path, so the model does not depend on the process's working directory.
            # `scale` converts the CAD's millimetres to SI at the asset rather than in the
            # geom, which is the boundary the units policy names.
            (f'    <mesh name="cadwheel" file="{Path(visual_mesh).resolve()}" '
             f'scale="{CAD_MESH_SCALE} {CAD_MESH_SCALE} {CAD_MESH_SCALE}"/>'),
        ]),
        *([] if chassis_mesh is None else [
            (f'    <mesh name="cadchassis" file="{Path(chassis_mesh).resolve()}" '
             f'scale="{CAD_MESH_SCALE} {CAD_MESH_SCALE} {CAD_MESH_SCALE}"/>'),
        ]),
        "  </asset>",
        "  <worldbody>",
        '    <light pos="0 -2 3" dir="0 0.4 -1" directional="true" diffuse="0.7 0.7 0.7"/>',
        '    <light pos="2 -1.5 2" dir="-0.5 0.3 -1" diffuse="0.35 0.35 0.35"/>',
        (f'    <geom name="floor" type="plane" size="8 3 0.1" contype="1" '
         f'conaffinity="{GROUND_CONAFFINITY}" material="floormat"/>'),
        # The step runs to beyond anything the robot can reach in the time allowed, so it is
        # an upper *ground* rather than a platform. Sized rather than fixed at 2 m: a robot
        # doing 1.15 m/s for 6 s covers 6.9 m, and a 4 m step let it climb, cross, and drive
        # off the far end — after which the final frame shows it back on the floor at exactly
        # the ride height, which reads as "never climbed" and is the opposite of the truth.
        # Flat ground emits no step at all rather than a zero-height one. A MuJoCo box of
        # size 0 is a compile error, and a box of size epsilon would be a lip the robot
        # bumps over — which is exactly the acceleration the harshness scenario is trying
        # to measure, contributed by the scenario instead of by the wheel.
        *([] if scenario.step_height_m <= 0.0 else [
            (f'    <geom name="step" type="box" pos="{step_centre_x:.9f} 0 '
             f'{scenario.step_height_m / 2:.9f}" size="{step_half_len:.9f} '
             f'{step_half_width:.9f} {scenario.step_height_m / 2:.9f}" contype="1" '
             f'conaffinity="{GROUND_CONAFFINITY}" material="stepmat"/>'),
        ]),
        *_washboard_geoms(scenario, reach, step_half_width),
        # Yaw about z, MuJoCo's w-x-y-z order. The step's face stays normal to world x, so
        # the angle is entirely in the robot's heading and "past the face" stays an x test.
        (f'    <body name="chassis" pos="0 0 {start_z:.9f}" '
         f'quat="{np.cos(0.5 * yaw):.9f} 0 0 {np.sin(0.5 * yaw):.9f}">'),
        '      <freejoint name="root"/>',
        (f'      <inertial pos="{platform.com_offset_m[0]:.9f} '
         f'{platform.com_offset_m[1]:.9f} {platform.com_offset_m[2]:.9f}" '
         f'mass="{platform.chassis_mass_kg:.9f}" '
         f'diaginertia="{ixx:.9g} {iyy:.9g} {izz:.9g}"/>'),
        # The contact chassis. In "box" mode one calibrated box, and only its paint changes
        # when the shell is drawn over it — the box recedes to a ghost so the render shows
        # the robot rather than its proxy, but a belly strike still happens on (and is
        # visible on) it. In "primitives" mode the box is gone entirely and the shapes read
        # off the simplified model collide instead.
        *([
            (f'      <geom name="body" type="box" size="{half[0]:.9f} {half[1]:.9f} '
             f'{half[2]:.9f}" mass="0" density="0" '
             + ('material="bodymat"/>' if chassis_mesh is None else
                f'rgba="{CHASSIS_BOX_GHOST_RGBA[0]:.3f} {CHASSIS_BOX_GHOST_RGBA[1]:.3f} '
                f'{CHASSIS_BOX_GHOST_RGBA[2]:.3f} {CHASSIS_BOX_GHOST_RGBA[3]:.3f}"/>')),
        ] if chassis_collision == "box" else
            _chassis_collision_primitives(axle_z - body_z,
                                          ghosted=chassis_mesh is not None)),
        *([] if chassis_mesh is None else [
            # The real shell, placed by its axle line: the mesh's axle-stub midpoint lands
            # on the body's (the translation below), and its axle height lands at the sim's
            # axle depth — which is (axle_z − body_z) below the chassis centre and
            # independent of wheel radius, because clearance is R + axle_to_belly. With the
            # +90° z rotation the mesh's lateral axis maps to +body_y, so the lateral
            # midline enters the y term NEGATED. Same decoration contract as the wheel
            # overlay: no mass, no contact, no number moves.
            (f'      <geom name="chassis_cad" type="mesh" mesh="cadchassis" '
             f'pos="{CHASSIS_MESH_AXLE_MM[2] * 1e-3:.9f} '
             f'{-CHASSIS_MESH_AXLE_MM[0] * 1e-3:.9f} '
             f'{(axle_z - body_z) - CHASSIS_MESH_AXLE_MM[1] * 1e-3:.9f}" '
             f'quat="{CHASSIS_MESH_QUAT[0]} {CHASSIS_MESH_QUAT[1]} '
             f'{CHASSIS_MESH_QUAT[2]} {CHASSIS_MESH_QUAT[3]}" '
             f'contype="0" conaffinity="0" mass="0" density="0" '
             f'rgba="{CHASSIS_MESH_RGBA[0]:.3f} {CHASSIS_MESH_RGBA[1]:.3f} '
             f'{CHASSIS_MESH_RGBA[2]:.3f} {CHASSIS_MESH_RGBA[3]:.3f}"/>'),
        ]),
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
        ]
        if segmented:
            # A hub carrying N segments, exactly as the single-wheel rig builds it — same
            # `ring_bodies`, so a wheel driven here is the ring that was fitted rather than a
            # second one that has drifted. `prefix` is what makes four of them coexist: MJCF
            # names are global, and four subtrees all calling their first segment `seg0`
            # collide. The per-segment mass is derived so the whole wheel still weighs
            # `wheel_mass_kg`, which is what keeps the rigid comparator a fair one.
            parts += [
                (f'        <geom name="{mount.name}_hub" type="sphere" size="0.005" '
                 f'mass="{HUB_MASS_KG:.9f}" contype="0" conaffinity="0" '
                 'material="wheelmat"/>'),
            ]
            parts += ring_bodies(
                spec, segment_half_width_m=0.5 * wheel_width_m,
                segment_mass_kg=segment_mass_kg, tangential=tangential,
                radial_damping=radial_damping,
                tangential_damping_c=tangential_damping_c,
                contype=SEGMENT_CONTYPE, conaffinity=0, rgba=SEGMENT_RGBA,
                prefix=f"{mount.name}_", indent=8,
            )
        else:
            parts += [
                (f'        <inertial pos="0 0 0" mass="{wheel_mass_kg:.9f}" '
                 f'diaginertia="{transverse:.12g} {inertia:.12g} {transverse:.12g}"/>'),
                (f'        <geom name="{mount.name}_tyre" type="cylinder" '
                 f'euler="1.5707963 0 0" size="{wheel_radius_m:.9f} '
                 f'{0.5 * wheel_width_m:.9f}" mass="0" density="0" material="wheelmat"/>'),
            ]
        if visual_mesh is not None:
            # Decoration, and every attribute here says so. `mass="0" density="0"` keeps it
            # out of the inertia (a mesh geom would otherwise be given the material default
            # and quietly change the wheel's mass), and the zeroed collision masks keep it out
            # of the contact set. Same euler as the rigid cylinder: see CAD_TO_WHEEL_EULER_X.
            parts.append(
                f'        <geom name="{mount.name}_cad" type="mesh" mesh="cadwheel" '
                f'euler="{CAD_TO_WHEEL_EULER_X:.7f} 0 0" contype="0" conaffinity="0" '
                'mass="0" density="0" material="cadmat"/>'
            )
        parts.append("      </body>")

    parts += ["    </body>", "  </worldbody>"]
    if segmented and spec.is_coupled:
        for mount in mounts:
            parts += coupling_tendons(spec, prefix=f"{mount.name}_")
    parts += ["  <actuator>"]
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
    law: RadialLaw | None = None,
    tangential_law: RadialLaw | None = None,
    tangential_element: TangentialElement | None = None,
    visual_mesh: Path | str | None = None,
    visual_rgba: tuple[float, float, float, float] = CAD_OVERLAY_RGBA,
    chassis_mesh: Path | str | None = None,
    chassis_collision: str = "box",
) -> RoverResult:
    """Drive the robot straight at the step. ``observer(k, model, data)`` after every step.

    The observer hook is the same contract :func:`~wheelopt.sim.step_climb.observe_step` uses,
    and for the same reason: ``scripts/render_rover.py`` must film *this* run rather than a
    second copy of it, because a renderer showing a different simulation checks nothing.

    Passing ``law`` switches all four wheels from rigid cylinders to **segmented rings** driven
    by that spring law. The mechanics are the single-wheel rig's, deliberately: the same
    ``ring_bodies``, the same ``qfrc_applied`` per step (MuJoCo gets no force law from the XML,
    invariant 8), the same ``stable_timestep_s`` bound, and the same loss-factor damping
    emitted as native joint ``damping`` rather than as an applied force. See
    :func:`build_rover_mjcf` for what a segmented rover is and is not evidence of.
    """
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - environment dependent
        return RoverResult(ok=False,
                           message=f"MuJoCo is not installed; pip install -e '.[sim]': {exc}")

    if law is not None and spec is None:
        return RoverResult(ok=False, message="a spring law needs a RingSpec to live on")
    segmented = law is not None
    # A rigid wheel has no segments, so it cannot have a tangential freedom either — the same
    # guard the single-wheel rig makes, for the same reason: honouring the argument silently
    # would build four cylinders and claim they had claw compliance.
    tangential_law = tangential_law if segmented else None
    element = resolve_tangential_element(tangential_law, tangential_element)

    radial_damping = tan_damping = 0.0
    if segmented:
        arm = hinge_arm_m(spec) if element == "hinge" else 0.0
        # A hinge's coordinate is an angle, so the timestep bound and the damping equivalence
        # are both applied to the law referred to the *tip* and the damping referred back.
        equivalent_law = (TipEquivalentLaw(tangential_law, arm) if element == "hinge"
                          else tangential_law)
        segment_mass = (wheel_mass_kg - HUB_MASS_KG) / max(spec.n_segments, 1)
        # `qfrc_applied` is an external force, so `implicitfast` integrates it explicitly and a
        # stiff segment law diverges at the rover's default step. Tightened before the model is
        # built, because the scenario carries the timestep into the MJCF.
        scenario = replace(scenario, timestep_s=stable_timestep_s(
            [law, equivalent_law], segment_mass, scenario.timestep_s))
        # The payload one wheel carries: the whole robot on four wheels.
        per_wheel_kg = (platform.chassis_mass_kg + 4.0 * wheel_mass_kg) / 4.0
        radial_damping = segment_damping_n_s_per_m(
            law, spec, per_wheel_kg, scenario.loss_factor)
        tan_damping = 0.0 if tangential_law is None else tangential_damping(
            spec, element,
            segment_damping_n_s_per_m(equivalent_law, spec, per_wheel_kg,
                                      scenario.loss_factor),
        )

    try:
        xml = build_rover_mjcf(platform, scenario, wheel_radius_m=wheel_radius_m,
                               wheel_width_m=wheel_width_m, wheel_mass_kg=wheel_mass_kg,
                               spec=spec, segmented=segmented,
                               tangential=element if segmented else None,
                               radial_damping=radial_damping,
                               tangential_damping_c=tan_damping,
                               visual_mesh=visual_mesh, visual_rgba=visual_rgba,
                               chassis_mesh=chassis_mesh,
                               chassis_collision=chassis_collision)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
    except Exception as exc:  # noqa: BLE001 - a bad model is a result, not a crash
        return RoverResult(ok=False, message=f"{type(exc).__name__}: {exc}")

    mounts = wheel_mounts(platform, wheel_width_m)
    axle_dofs = np.array(
        [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                            f"{m.name}_axle")] for m in mounts],
        dtype=np.int64,
    )

    def joint_addr(letter: str) -> tuple[np.ndarray, np.ndarray]:
        """Every wheel's segment joints, flattened. One array, one assignment per step."""
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                 f"{m.name}_{letter}{i}")
               for m in mounts for i in range(spec.n_segments)]
        return (np.asarray(model.jnt_dofadr[ids], dtype=np.int64),
                np.asarray(model.jnt_qposadr[ids], dtype=np.int64))

    empty = np.empty(0, dtype=np.int64)
    segment_dofs, segment_qpos = joint_addr("j") if segmented else (empty, empty)
    tangential_dofs, tangential_qpos = (joint_addr("t") if tangential_law is not None
                                        else (empty, empty))
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    # Every geom the chassis collides with the world through: the one calibrated box, or
    # the primitive set. Collected by name rather than assumed singular, so the belly-strike
    # verdict means the same thing under either chassis model.
    chassis_geoms = frozenset(
        g for g in range(model.ngeom)
        if (name := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g))
        and (name == "body" or name.startswith("chassis_col_"))
    )
    # -1 on a flat run, where there is no step to hit. Geom ids are non-negative, so the
    # membership test below is simply never true and `chassis_hit_step` stays False.
    step_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")
    # Vertical acceleration straight off the free joint. Its three translational DOFs are the
    # chassis frame's position in *world* coordinates, so `qacc[+2]` is the world-z
    # acceleration the solver produced — no differencing of a position history, which at
    # 5e-4 s would amplify contact noise by 4e6 and measure the integrator.
    root_z_dof = int(model.jnt_dofadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")]) + 2

    n_steps = int(scenario.duration_s / scenario.timestep_s)
    settle_steps = int(scenario.settle_s / scenario.timestep_s)
    history = np.zeros((n_steps, 6), dtype=np.float64)
    energy = 0.0
    hit_step = False
    # The ROM's validity envelope, counted rather than assumed (TODO #31). A segment only
    # compresses when the ground pushes it, so "compressed" is "in contact" — and more than
    # one per wheel is exactly the second-claw engagement past which the elements straddle
    # the FEA. Counted per wheel, because four wheels sharing one count would call a rover
    # with one claw down on each of four wheels a four-claw contact.
    multi_contact_steps = 0
    peak_compression = 0.0
    per_wheel = (len(mounts), spec.n_segments) if segmented else (0, 0)

    try:
        for k in range(n_steps):
            if segmented:
                # All four rings in one assignment. The joints were laid out wheel-major, so a
                # single flat array covers 4 x n_segments and no wheel can be forgotten.
                data.qfrc_applied[segment_dofs] = law.force_n(-data.qpos[segment_qpos])
                if tangential_law is not None:
                    # Symmetric: a claw bends the same either way, so the restoring force is
                    # sign(v)*f(|v|). Passing the signed coordinate straight to `force_n` makes
                    # one direction free and the other doubled, and the wheel drifts under load.
                    data.qfrc_applied[tangential_dofs] = -symmetric_force_n(
                        tangential_law, data.qpos[tangential_qpos]
                    )
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
                          float(data.xpos[chassis, 2]), pitch, roll,
                          float(data.qacc[root_z_dof]))
            if segmented and k >= settle_steps:
                # `qpos` is signed and a compressed segment sits at negative coordinate, the
                # same convention `qfrc_applied` is written with above.
                squash = -data.qpos[segment_qpos].reshape(per_wheel)
                deepest = squash.max(axis=1, keepdims=True)
                peak_compression = max(peak_compression, float(deepest.max()))
                # **Sharing, not touching.** A claw that has just left the ground is still
                # ringing down through a few microns of compression, so an absolute threshold
                # counts it as in contact: at 1 um this read 70% of a flat run, against a
                # static ring that carries on one claw for all but the half-pitch crossing.
                # That is the project's standing failure — a plausible number measuring the
                # wrong thing — so the criterion is a share of the deepest claw on the same
                # wheel, with a floor so an airborne wheel does not divide by its own noise.
                loaded = (squash > MULTI_CONTACT_SHARE * deepest) & (deepest > 1.0e-4)
                if int(np.max(np.count_nonzero(loaded, axis=1))) > 1:
                    multi_contact_steps += 1
            if not hit_step:
                for c in range(data.ncon):
                    pair = (data.contact[c].geom1, data.contact[c].geom2)
                    if step_geom in pair and (pair[0] in chassis_geoms
                                              or pair[1] in chassis_geoms):
                        hit_step = True
                        break
    except Exception as exc:  # noqa: BLE001 - a diverged scenario is a result
        return RoverResult(ok=False, message=f"{type(exc).__name__}: {exc}")

    if not np.all(np.isfinite(history)):
        return RoverResult(ok=False, message="the scenario diverged (non-finite state)")
    driving_steps = max(n_steps - settle_steps, 1)
    return _summarise(platform, scenario, history, settle_steps, energy, hit_step,
                      n_tips=0 if spec is None else spec.n_segments,
                      wheel_radius_m=wheel_radius_m, wheel_width_m=wheel_width_m,
                      multi_contact_fraction=multi_contact_steps / driving_steps,
                      peak_compression_m=peak_compression)


def _summarise(platform, scenario, history, settle_steps, energy_j, hit_step, *,
               n_tips: int = 0, wheel_radius_m: float = 0.0, wheel_width_m: float = 0.0,
               multi_contact_fraction: float = 0.0,
               peak_compression_m: float = 0.0) -> RoverResult:
    """Turn the history into the handful of numbers worth quoting."""
    x, z = history[:, 1], history[:, 2]
    driving = slice(settle_steps, None)
    ride = platform.ground_clearance_for(wheel_radius_m) + 0.5 * platform.chassis_height_m
    clearance = float(z[-1]) - scenario.step_height_m

    # The steady window: the second half of the driving phase. The first half is the launch
    # transient — the robot leaves rest at stall torque and the largest vertical acceleration
    # in the whole run is the squat as it does so, which is a fact about the motor and would
    # otherwise dominate a harshness number meant to be about the wheel.
    steady_from = settle_steps + (len(history) - settle_steps) // 2
    steady = slice(steady_from, None)
    span_s = float(history[-1, 0] - history[steady_from, 0]) if len(history) > steady_from else 0.0
    speed = float(x[-1] - x[steady_from]) / span_s if span_s > 0.0 else 0.0
    harshness = float(np.sqrt(np.mean(history[steady, 5] ** 2))) if span_s > 0.0 else 0.0
    # Tip-passing frequency: n tips per revolution, v/(2πR) revolutions per second. Zero for
    # a solid cylinder, which has no tips and therefore no polygon forcing at all.
    tip_hz = (n_tips * abs(speed) / (2.0 * np.pi * wheel_radius_m)
              if n_tips and wheel_radius_m > 0.0 else 0.0)

    # Climbed means the **whole body** got onto the upper ground and settled there, not that
    # some part of it got past the face. Both halves are load-bearing, and each was wrong on
    # its own in the first version: a robot nose-up against a 100 mm step is past `step_x`
    # and 113 mm above the upper ground, so the x test alone passes it; and a robot that has
    # climbed sits at its ride height, so a loose height threshold passes the leaner too.
    # Require the chassis centre a full half-length beyond the face, and within a fifth of
    # its ride height of where it would stand.
    # Objective 5: worst-moment margin to static tip-over. Peak excursions are already
    # computed below; the margin is the same numbers against the platform's own critical
    # angles, so a taller CG or a narrower track shows up here without any code changing.
    # The wheel this run was fitted with sets the track (external mounting), so the roll
    # yardstick moves with it — that is the support polygon actually under the robot.
    pitch_crit, roll_crit = platform.tipover_angles_rad(
        track_m=platform.track_for(wheel_width_m) if wheel_width_m > 0.0 else None)
    peak_pitch = float(np.max(np.abs(history[driving, 3])))
    peak_roll = float(np.max(np.abs(history[driving, 4])))
    stability = 1.0 - max(peak_pitch / pitch_crit, peak_roll / roll_crit)

    on_top = ((x > scenario.step_x_m + 0.5 * platform.chassis_length_m)
              & (np.abs(z - scenario.step_height_m - ride) < 0.2 * ride))
    # On flat ground both halves of that test pass the moment the robot has driven a metre,
    # and reporting `climbed=True` for it would be true of a sentence nobody asked. There is
    # no obstacle, so there is nothing to have climbed.
    climbed = bool(np.any(on_top[driving])) and scenario.step_height_m > 0.0
    return RoverResult(
        ok=True,
        climbed=climbed,
        distance_m=float(np.max(x) - x[settle_steps]),
        final_clearance_m=clearance,
        peak_pitch_rad=peak_pitch,
        peak_roll_rad=peak_roll,
        chassis_hit_step=bool(hit_step),
        stability_margin=float(stability),
        energy_j=float(energy_j),
        harshness_rms_m_s2=harshness,
        mean_speed_m_s=speed,
        tip_frequency_hz=float(tip_hz),
        multi_contact_fraction=float(multi_contact_fraction),
        peak_compression_m=float(peak_compression_m),
        history=history,
    )


def run_rover(platform: PlatformSpec | None = None, scenario: RoverSpec | None = None, *,
              wheel_radius_m: float = 0.085, wheel_width_m: float = 0.030,
              wheel_mass_kg: float = 0.30, spec: RingSpec | None = None,
              law: RadialLaw | None = None, tangential_law: RadialLaw | None = None,
              tangential_element: TangentialElement | None = None,
              visual_mesh: Path | str | None = None,
              visual_rgba: tuple[float, float, float, float] = CAD_OVERLAY_RGBA,
              chassis_mesh: Path | str | None = None,
              chassis_collision: str = "box",
              ) -> RoverResult:
    """Convenience entry point: load the platform, run one scenario, return the result."""
    return observe_rover(platform or load_platform(), scenario or RoverSpec(),
                         wheel_radius_m=wheel_radius_m, wheel_width_m=wheel_width_m,
                         wheel_mass_kg=wheel_mass_kg, spec=spec, law=law,
                         tangential_law=tangential_law,
                         tangential_element=tangential_element,
                         visual_mesh=visual_mesh, visual_rgba=visual_rgba,
                         chassis_mesh=chassis_mesh, chassis_collision=chassis_collision)
