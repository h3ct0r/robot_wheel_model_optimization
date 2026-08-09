"""Build the segmented ring as a MuJoCo model, and press it into the floor.

This is the only module in :mod:`wheelopt.rom` that needs MuJoCo. Everything the fit depends
on lives in :mod:`wheelopt.rom.ring` and :mod:`wheelopt.rom.fit` as pure numpy, so the model
below can be wrong without the fit being wrong — and ``scripts/run_rom.py`` compares the two
precisely so that a disagreement points here.

What the comparison is worth
----------------------------
The analytic ring works out where each segment ends up: uncoupled, from the geometric
interference of the undeformed circle with a flat plate, ``u = R - (R - δ)/cos θ``; coupled,
from a constrained equilibrium whose contact set it has to find. MuJoCo assumes neither: it
resolves real contacts between real geoms with a real constraint solver, and the segments are
free to end up wherever the forces put them. Agreement between the two is therefore a check on
that reasoning, and it gets sharper with coupling — an uncoupled ring only tests the geometry,
a coupled one tests the active-set solve as well, because MuJoCo finds the patch by touching
the floor rather than by iterating a multiplier sign condition. Disagreement is informative
either way, which is why the script reports the gap rather than asserting it away.

Invariant 8 (CLAUDE.md) applies here with force: the compliance in this model is the **joint**
force law, never ``solref``/``solimp``. Contact is configured to be as close to rigid as
MuJoCo will comfortably run, because any softness there is a numerical regulariser
contaminating a stiffness this project is trying to measure.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .ring import (
    RadialLaw,
    RingSpec,
    curvature_operator,
    segment_angles,
    symmetric_force_n,
)

__all__ = [
    "MissingMuJoCo",
    "RingModel",
    "build_mjcf",
    "coupling_tendons",
    "ring_bodies",
    "static_load_deflection",
]

#: Segment body mass, kg. The comparison below is quasi-static — driven slowly, gravity off —
#: so this affects only the solver's conditioning, not the answer. Kept small and uniform
#: rather than derived from the wheel's mass, because a *dynamic* run must derive it from
#: geometry and material (invariant 2) and a placeholder that looks derived is worse than one
#: that obviously is not.
SEGMENT_MASS_KG = 0.002
HUB_MASS_KG = 0.05


class MissingMuJoCo(ImportError):
    """MuJoCo is not installed. ``pip install -e '.[sim]'``."""


@dataclass(frozen=True, slots=True)
class RingModel:
    """A built MuJoCo ring, plus the indices needed to drive and read it."""

    xml: str
    #: Joint ids of the radial segment slides, in segment order.
    segment_joints: np.ndarray
    spec: RingSpec
    law: RadialLaw
    #: Joint ids of the tangential slides, in segment order. Empty when the model was built
    #: without a tangential law — empty rather than absent, so a caller that indexes it gets
    #: no forces rather than an attribute error, and a radial-only ring stays radial-only.
    tangential_joints: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.int64)
    )
    tangential_law: RadialLaw | None = None


def build_mjcf(
    spec: RingSpec,
    law: RadialLaw,
    *,
    segment_half_width_m: float = 0.015,
    indentation_m: float = 0.0,
    tangential: bool = False,
) -> str:
    """Emit the MJCF for one ring.

    The wheel lies in the x-z plane and rolls along x; segment 0 faces straight down at the
    contact point. Each segment hangs off the hub on a slide joint along its own radius, so
    a *negative* joint position is compression — the sign convention the force law expects.

    Gravity is off. This model is pressed by a prescribed hub displacement, and a weight term
    would add a load the FEA sweep did not have.

    The hub is **welded to the world** at ``R - indentation_m`` rather than given a slide
    joint that gets rewritten each step. Prescribing a coordinate by assignment fights the
    solver: it teleports the body, the constraint forces spike, and the run dies with "Nan,
    Inf or huge value in QACC". A hub with no degrees of freedom cannot be teleported, and
    rebuilding the model per δ costs milliseconds.
    """
    r = spec.radius_m
    parts = [
        '<mujoco model="wheel_ring">',
        '  <compiler angle="radian"/>',
        # No gravity: the sweep is displacement-controlled, exactly as the FEA was.
        '  <option gravity="0 0 0" integrator="implicitfast" timestep="0.0005"/>',
        "  <default>",
        # solref/solimp here are a numerical regulariser for the *floor* contact only, and
        # are deliberately stiff. The wheel's compliance is the joint force law. See ADR-0001.
        '    <geom solref="0.0002 1" solimp="0.99 0.999 0.0001" condim="1"/>',
        "  </default>",
        "  <worldbody>",
        ('    <geom name="floor" type="plane" size="2 2 0.1" pos="0 0 0" '
         'contype="0" conaffinity="1"/>'),
        f'    <body name="hub" pos="0 0 {r - indentation_m:.9f}">',
        (f'      <geom name="hubgeom" type="sphere" size="0.002" mass="{HUB_MASS_KG}" '
         'contype="0" conaffinity="0"/>'),
    ]

    parts += ring_bodies(spec, segment_half_width_m=segment_half_width_m,
                         tangential=tangential, indent=6)
    parts += ["    </body>", "  </worldbody>"]
    parts += coupling_tendons(spec)
    parts += ["</mujoco>"]
    return "\n".join(parts)


def ring_bodies(
    spec: RingSpec,
    *,
    segment_half_width_m: float = 0.015,
    segment_mass_kg: float = SEGMENT_MASS_KG,
    contype: int = 1,
    conaffinity: int = 0,
    tangential: bool = False,
    indent: int = 6,
) -> list[str]:
    """The ``N`` segment bodies, to be nested inside whatever carries the hub.

    Shared between the static press in this module and the rolling scenarios in
    :mod:`wheelopt.sim`, so that a wheel driven at a step is the *same* ring that was fitted
    to the FEA and not a second one that has drifted. Everything positional is relative to
    the parent body's origin, which is the hub centre.

    ``tangential`` adds a **second slide per segment**, along ``(cos θ, 0, sin θ)`` — the
    in-plane perpendicular to the segment's own radius, which is the freedom
    :func:`~wheelopt.rom.ring.solve_equilibrium_2dof` calls ``v``.

    **A slide is the wrong element past small ``v``, and a driven wheel is not small ``v``.**
    See :func:`~wheelopt.rom.ring.solve_equilibrium_2dof` for the measurement: a sliding tip
    moves *outward* as it splays where a hinged one moves inward, by 30% of the wheel radius
    at one claw length of deflection, and that is a positive feedback against the ground. Use
    this for static presses and small-splay work; a rolling wheel under drive torque needs the
    root hinge of ``TODO.md`` #27 instead. The axis is chosen so the
    two models are the same model: moving a segment by ``v`` along it raises its tip by
    ``v sin θ``, exactly the term in the analytic height equation. A claw's tangential
    stiffness is 134× below its radial one (2026-08-08), so this is not a refinement for the
    claw topology — it is the compliance a step edge actually loads.

    Off by default, and off means *absent* rather than locked: a ring built without it is the
    same XML it was before the joint existed, so nothing that predates this has silently
    become a two-freedom result. Whoever adds the joint must also drive it — MuJoCo gets no
    force law from the XML, only from ``qfrc_applied`` (invariant 8).

    **Refused on a banded spec**, matching
    :func:`~wheelopt.rom.ring.solve_equilibrium_2dof`. The refusal is not a limitation of the
    solver here — MuJoCo would happily integrate it — it is that the model would be wrong.
    :func:`coupling_tendons` couples the *radial* joints only, because the analytic band is an
    energy in the compressions; give the segments a tangential freedom as well and they can
    shear past each other with **nothing resisting**, which is the one deformation a shear
    band exists to carry. A silently softer ring is exactly the failure this project keeps
    finding, so it raises instead.
    """
    if tangential and spec.is_coupled:
        raise ValueError(
            "a tangential freedom needs a band that shears; coupling_tendons couples only "
            "the radial joints, so a banded ring with tangential slides would shear for free"
        )
    theta = segment_angles(spec)
    r = spec.radius_m
    seg_r = 0.5 * spec.segment_arc_m
    pad = " " * indent
    # The capsule's own radius stands off the body origin, so a body placed at R puts the
    # running surface at R + r_capsule and contact begins ~4 mm early on this wheel — the
    # model then reports five to six times the analytic force and looks like a stiffness
    # error rather than a geometry one. Seat the body so the capsule *surface* is at R.
    # No guard on body_radius: it is r(1 - pi/2n), positive for every n >= 2, and
    # RingSpec already refuses fewer than three segments. A check here would be a branch
    # that can never run, which reads as a safety net and is not one.
    body_radius = r - seg_r * 0.5

    lines: list[str] = []
    for i, angle in enumerate(theta):
        # Outward radial unit vector for this segment; segment 0 points at the floor.
        ux, uz = float(np.sin(angle)), float(-np.cos(angle))
        px, pz = body_radius * ux, body_radius * uz
        lines += [
            f'{pad}<body name="seg{i}" pos="{px:.9f} 0 {pz:.9f}">',
            # Positive q is outward. The old upper bound of 0.01 m was written when nothing
            # could push a segment outward at all; with the band there, segments beside the
            # patch bulge, and a joint limit that quietly caps that bulge would look like a
            # stiffness disagreement rather than the constraint it is. Both bounds are now
            # far outside anything physical, so hitting one means a bug, not a design.
            (f'{pad}  <joint name="j{i}" type="slide" axis="{ux:.9f} 0 {uz:.9f}" '
             f'range="{-0.9 * r:.9f} {0.5 * r:.9f}" limited="true"/>'),
        ]
        if tangential:
            # Perpendicular to the radius, in the wheel's plane. Segment 0 points down, so
            # its tangential axis is +x, along the rolling direction — which is the direction
            # a claw bends when it catches a step, and the reason this joint exists.
            tx, tz = float(np.cos(angle)), float(np.sin(angle))
            lines.append(
                f'{pad}  <joint name="t{i}" type="slide" axis="{tx:.9f} 0 {tz:.9f}" '
                f'range="{-0.5 * r:.9f} {0.5 * r:.9f}" limited="true"/>'
            )
        lines += [
            # The capsule spans the wheel's WIDTH, i.e. along y, the axle direction.
            # Laying it along x instead makes each segment a 30 mm bar in the rolling
            # direction: neighbours 15.7 mm apart then overlap permanently, and the model
            # reports tens of newtons of contact force before the floor is even touched.
            (f'{pad}  <geom name="g{i}" type="capsule" fromto="'
             f'0 {-segment_half_width_m:.9f} 0 0 {segment_half_width_m:.9f} 0" '
             f'size="{seg_r * 0.5:.9f}" mass="{segment_mass_kg}" '
             f'contype="{contype}" conaffinity="{conaffinity}"/>'),
            f"{pad}</body>",
        ]
    return lines


def coupling_tendons(spec: RingSpec, *, indent: int = 2) -> list[str]:
    """The shear band, as fixed tendons: ``N`` bending ones and one hoop.

    This is an exact rendering of the analytic band, not an approximation of it, and the
    reason is worth stating because it is not obvious that MuJoCo can do this at all. A
    **fixed tendon** has length ``L = Σ_j coef_j · q_j`` and, given ``stiffness`` and
    ``springlength="0"``, stores ``(k/2) L²`` and applies ``-k·L·coef_j`` to each joint. Both
    of the band's energies are already of that form:

    - bending, ``(k_b/2) Σ_i D_i²`` with ``D_i`` a three-term window on the compressions
      (:func:`~wheelopt.rom.ring.curvature_operator`) — one tendon per segment, coefficients
      taken from row ``i``;
    - hoop, ``(k_h/2)(Σ_i u_i)²`` — one tendon spanning every joint with unit coefficients.

    No linearisation, no lumped rotational springs between bodies, no equality constraint to
    close the ring: the wrap is already in the circulant.

    ``springlength="0"`` is load-bearing and must stay explicit: MuJoCo's default is the
    tendon length at the model's initial configuration, which here is zero anyway, but only
    because the ring is built undeformed. Leaving it implicit would make the band's rest state
    a property of how the model happened to be posed at compile time.

    The sign of ``q`` versus ``u`` does not matter — the energies are even in ``L`` — but for
    the record the joint reads ``q = -u``, so each tendon length is the negated form.
    """
    if not spec.is_coupled:
        return []
    lines = ["  <tendon>"]
    if spec.band_bending_n_per_m > 0.0:
        operator = curvature_operator(spec)
        for i in range(spec.n_segments):
            lines.append(f'    <fixed name="bend{i}" '
                         f'stiffness="{spec.band_bending_n_per_m:.9g}" springlength="0">')
            # Only the three non-zero entries of the row; a coef of 0 is legal but makes the
            # XML N times bigger and hides which segments are actually neighbours.
            for j in sorted({(i - 1) % spec.n_segments, i, (i + 1) % spec.n_segments}):
                lines.append(f'      <joint joint="j{j}" coef="{operator[i, j]:.9g}"/>')
            lines.append("    </fixed>")
    if spec.band_hoop_n_per_m > 0.0:
        lines.append(f'    <fixed name="hoop" stiffness="{spec.band_hoop_n_per_m:.9g}" '
                     'springlength="0">')
        lines += [f'      <joint joint="j{i}" coef="1"/>' for i in range(spec.n_segments)]
        lines.append("    </fixed>")
    lines.append("  </tendon>")
    return lines


def _load(
    spec: RingSpec,
    law: RadialLaw,
    *,
    tangential_law: RadialLaw | None = None,
    **kwargs,
) -> tuple[RingModel, object, object]:
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise MissingMuJoCo("MuJoCo is not installed; pip install -e '.[sim]'") from exc

    xml = build_mjcf(spec, law, tangential=tangential_law is not None, **kwargs)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    def ids(prefix: str) -> np.ndarray:
        return np.array(
            [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}{i}")
             for i in range(spec.n_segments)],
            dtype=np.int64,
        )

    ring = RingModel(
        xml=xml,
        segment_joints=ids("j"),
        spec=spec,
        law=law,
        tangential_joints=(ids("t") if tangential_law is not None
                           else np.empty(0, dtype=np.int64)),
        tangential_law=tangential_law,
    )
    return ring, model, data


def static_load_deflection(
    spec: RingSpec,
    law: RadialLaw,
    delta_m: np.ndarray,
    *,
    tangential_law: RadialLaw | None = None,
    settle_steps: int = 3000,
    damping: float = 4.0,
) -> np.ndarray:
    """Press the ring into the floor at each δ and read the floor reaction, newtons.

    The reaction is the **vertical sum of the floor contact forces**, not a re-evaluation of
    the spring law. That distinction is the whole point: reading the law back would compare
    :mod:`wheelopt.rom.ring`'s formula against itself with MuJoCo only supplying the
    penetrations, and would agree even if the contact geometry were wrong. Taking it from the
    contact solver makes this an independent measurement of the same quantity the FEA reads
    off its rigid-body reference node.

    Args:
        tangential_law: if given, each segment also gets the in-plane perpendicular freedom
            and this law resists it, **symmetrically** — a claw bends the same either way, so
            the force is ``sign(v)·f(|v|)`` and not ``f(v)``, which would make one direction
            free and the other doubled.
        settle_steps: steps per δ, with the segments damped. Quasi-static by construction —
            the hub cannot move, so this is a relaxation, not a dynamic run.
        damping: joint damping, N·s/m. Sets how fast it settles, not where it settles.
    """
    import mujoco

    deltas = np.atleast_1d(np.asarray(delta_m, dtype=np.float64))
    out = np.empty(len(deltas), dtype=np.float64)
    force6 = np.zeros(6, dtype=np.float64)

    for k, delta in enumerate(deltas):
        ring, model, data = _load(spec, law, tangential_law=tangential_law,
                                  indentation_m=float(delta))
        # Indexed by joint rather than by the whole dof block: with two joints per segment the
        # block is interleaved, and `model.jnt_dofadr[:]` on its own no longer says which
        # entry is radial. Getting that wrong would drive a tangential dof with a radial law
        # — 134x too stiff on a claw, and it would read as a contact bug.
        radial_dof = model.jnt_dofadr[ring.segment_joints]
        radial_qpos = model.jnt_qposadr[ring.segment_joints]
        tan_dof = model.jnt_dofadr[ring.tangential_joints]
        tan_qpos = model.jnt_qposadr[ring.tangential_joints]

        for _ in range(settle_steps):
            u = -data.qpos[radial_qpos]
            data.qfrc_applied[radial_dof] = (law.force_n(u)
                                             - damping * data.qvel[radial_dof])
            if tangential_law is not None:
                v = data.qpos[tan_qpos]
                data.qfrc_applied[tan_dof] = (-symmetric_force_n(tangential_law, v)
                                              - damping * data.qvel[tan_dof])
            mujoco.mj_step(model, data)

        total = 0.0
        for c in range(data.ncon):
            mujoco.mj_contactForce(model, data, c, force6)
            # Contact frame x is the normal; rotate it back into world coordinates.
            frame = data.contact[c].frame.reshape(3, 3)
            total += float((frame.T @ force6[:3])[2])
        out[k] = abs(total)

    return out
