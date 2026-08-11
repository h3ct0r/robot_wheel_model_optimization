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
from typing import Literal

import numpy as np

from .ring import (
    RadialLaw,
    RingSpec,
    TipEquivalentLaw,
    curvature_operator,
    segment_angles,
    symmetric_force_n,
)

__all__ = [
    "EXPLICIT_STABILITY_LIMIT",
    "STATIC_TIMESTEP_S",
    "TANGENTIAL_ELEMENTS",
    "MissingMuJoCo",
    "RingModel",
    "TangentialElement",
    "build_mjcf",
    "coupling_tendons",
    "hinge_arm_m",
    "hinge_pivot_radius_m",
    "resolve_tangential_element",
    "ring_bodies",
    "segment_body_radius_m",
    "stable_timestep_s",
    "static_load_deflection",
    "tangential_damping",
]

#: Which second freedom a segment gets, if any.
#:
#: ``"slide"`` is the tip translating along the in-plane perpendicular — the element of
#: :func:`~wheelopt.rom.ring.solve_equilibrium_2dof`. It is **wrong past small deflection**: a
#: sliding tip moves *outward* as it splays, by 30% of the wheel radius at one claw length,
#: and that is a positive feedback against the ground. Kept because the static press and the
#: flat-plate sweeps stay well inside where it is valid, and because retiring it silently
#: would erase the comparison that settled ``TODO.md`` #27.
#:
#: ``"hinge"`` is the claw rotating about its root — :func:`~wheelopt.rom.ring.
#: solve_equilibrium_hinge`. Keeps the claw's length fixed by construction, so use it for
#: anything driven.
TangentialElement = Literal["slide", "hinge"]
TANGENTIAL_ELEMENTS: tuple[str, ...] = ("slide", "hinge")

#: Segment body mass, kg. The comparison below is quasi-static — driven slowly, gravity off —
#: so this affects only the solver's conditioning, not the answer. Kept small and uniform
#: rather than derived from the wheel's mass, because a *dynamic* run must derive it from
#: geometry and material (invariant 2) and a placeholder that looks derived is worse than one
#: that obviously is not.
def _rgba(colour: tuple[float, float, float, float]) -> str:
    """An MJCF ``rgba`` attribute value, quoted. Four components, no exceptions."""
    if len(colour) != 4:
        raise ValueError(f"rgba needs four components, got {len(colour)}")
    if any(not 0.0 <= c <= 1.0 for c in colour):
        raise ValueError(f"rgba components must be in [0, 1]; got {colour}")
    return '"' + " ".join(f"{c:.3f}" for c in colour) + '"'


SEGMENT_MASS_KG = 0.002
HUB_MASS_KG = 0.05

#: Largest ``ω·h`` at which a segment force law applied through ``qfrc_applied`` integrates
#: stably. **Measured**, not derived: on the bandless R 60 mm claw ring at k = 19.76 kN/m and
#: 2 g segments (ω = 3143 rad/s), ω·h = 0.251 and below run clean for 0.6 s while 0.314 and
#: above diverge inside 5 ms. 0.2 sits about 25% under the observed boundary.
#:
#: The bound exists because ``qfrc_applied`` is an **external** force: ``implicitfast``
#: integrates a joint's own ``damping`` attribute implicitly but not this, so a stiff segment
#: spring is integrated explicitly and has a stability limit the rest of the model does not.
#: It is therefore a fact about *this file's* modelling choice — the compliance is a joint
#: force law (invariant 8) — and not about any one scenario, which is why it lives here rather
#: than beside the rolling rig that first hit it.
EXPLICIT_STABILITY_LIMIT = 0.2

#: Timestep for the static press, seconds, before the stability bound above is applied.
STATIC_TIMESTEP_S = 0.0005


def stable_timestep_s(laws, segment_mass_kg: float, requested_s: float) -> float:
    """Largest timestep at or below ``requested_s`` that integrates these laws stably.

    Why this is not optional, and why it is not a fudge. The radial-only rolling rig ran at
    ω·h = 0.63 on the bandless claw design — well past the boundary above — and was stable,
    but **by luck rather than by design**: an out-of-contact radial segment sits at exactly
    ``u = 0`` where the law returns exactly zero, so nothing excites the mode. A second
    freedom has no such luck. Its axis sweeps through gravity as the wheel turns, so every
    segment is driven at its own natural frequency for the whole run, and the marginal mode
    grows until the joint limits fire and the solver gives up.

    The same luck held for the **static press**, and for the same reason, until the hinge
    arrived: a segment pressed straight onto a plate has nothing to excite its neighbours'
    radial modes either. With a second freedom the press diverged at δ = 18 mm on 24 and 48
    segments, at the fixed 0.5 ms it had always used — measured 2026-08-09, and the reason
    :func:`static_load_deflection` now asks this function too.

    The remedy is the timestep, deliberately, in preference to the two alternatives that also
    work. **Extra joint damping** (``c ≥ 5 N·s/m`` measured, over and above the loss factor's)
    is dissipation that no material supplied — the loss factor is already the damping model,
    and cost of transport is one of the five signatures, so damping *added* there would be
    answering a physics question with a solver setting. **Heavier segments** (``≥ 20 g``
    measured, against 2 g) would change the ring's mass, which the rigid comparator matches, so
    it would move both wheels to fix one. A smaller timestep costs time and changes no answer.

    **This bound covers the springs only.** The loss factor's damping is a different problem
    with a different answer: it goes into the joints' own ``damping`` attribute, which
    ``implicitfast`` integrates implicitly and unconditionally, because bounding it explicitly
    would need the joints' *effective* inertia rather than the segment mass — a quantity two
    orders of magnitude smaller here. See :func:`ring_bodies`.

    Args:
        laws: the laws driven through ``qfrc_applied``; ``None`` entries are skipped. A hinge
            law must be passed as a :class:`~wheelopt.rom.ring.TipEquivalentLaw`, because the
            bound is on ``sqrt(k/m)`` and a moment over a rotation is neither.
        segment_mass_kg: the mass those laws accelerate.
        requested_s: the timestep the caller wanted. Never increased.
    """
    omega = max(
        (float(np.sqrt(max(law.stiffness_n_per_m(0.0), 0.0) / segment_mass_kg))
         for law in laws if law is not None),
        default=0.0,
    )
    if omega <= 0.0:
        return requested_s
    return min(requested_s, EXPLICIT_STABILITY_LIMIT / omega)


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
    #: Joint ids of the second freedom, in segment order — slides or hinges according to
    #: :attr:`tangential_element`. Empty when the model was built without one: empty rather
    #: than absent, so a caller that indexes it gets no forces rather than an attribute error,
    #: and a radial-only ring stays radial-only.
    tangential_joints: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.int64)
    )
    #: The law resisting that freedom. **Newtons per metre for a slide, N·m per radian for a
    #: hinge** — the unit follows :attr:`tangential_element`, and driving a hinge with a slide
    #: law would apply a torque numerically equal to a force, which is not a small error.
    tangential_law: RadialLaw | None = None
    tangential_element: TangentialElement | None = None


def build_mjcf(
    spec: RingSpec,
    law: RadialLaw,
    *,
    segment_half_width_m: float = 0.015,
    indentation_m: float = 0.0,
    tangential: TangentialElement | None = None,
    timestep_s: float = STATIC_TIMESTEP_S,
    radial_damping: float = 0.0,
    tangential_damping_c: float = 0.0,
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
        ('  <option gravity="0 0 0" integrator="implicitfast" '
         f'timestep="{timestep_s:.9g}"/>'),
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
                         tangential=tangential, radial_damping=radial_damping,
                         tangential_damping_c=tangential_damping_c, indent=6)
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
    tangential: TangentialElement | None = None,
    radial_damping: float = 0.0,
    tangential_damping_c: float = 0.0,
    rgba: tuple[float, float, float, float] | None = None,
    prefix: str = "",
    indent: int = 6,
) -> list[str]:
    """The ``N`` segment bodies, to be nested inside whatever carries the hub.

    Shared between the static press in this module and the rolling scenarios in
    :mod:`wheelopt.sim`, so that a wheel driven at a step is the *same* ring that was fitted
    to the FEA and not a second one that has drifted. Everything positional is relative to
    the parent body's origin, which is the hub centre.

    ``tangential`` selects the second freedom; see :data:`TangentialElement`. ``None`` — the
    default — emits exactly the XML this function emitted before either freedom existed, so
    nothing that predates them has silently become a two-freedom result. Whoever adds the
    joint must also drive it: MuJoCo gets no force law from the XML, only from
    ``qfrc_applied`` (invariant 8).

    ``"slide"`` adds a second slide along ``(cos θ, 0, sin θ)``, the in-plane perpendicular to
    the segment's own radius — the freedom :func:`~wheelopt.rom.ring.solve_equilibrium_2dof`
    calls ``v``. Moving a segment by ``v`` along it raises the tip by ``v sin θ``, exactly the
    term in that function's height equation, so the two are the same model. **Small ``v``
    only**; see :data:`TangentialElement`.

    ``"hinge"`` re-seats the whole body. Instead of sitting at the tread, the body's origin
    moves to the claw's pivot near the hub (:func:`hinge_pivot_radius_m` — one capsule radius
    inboard of ``root_radius_m``, for the reason argued there), gets a hinge about the
    out-of-plane axis, and carries the capsule out to the tread on an offset. So the running
    surface is where it always was and only the pivot is new. The radial slide stays, and stays
    listed *after* the hinge, because MuJoCo composes a body's joints in order: the slide's
    axis is carried round by the rotation, so it remains the claw's own axis rather than a
    fixed direction in the hub frame. (Verified against ``mj_kinematics``: at ``φ = 90°`` a
    positive slide moves the tip along ``+x``, the rotated radial, and the root-to-tip
    distance is unchanged by the rotation.) That ordering is the entire realisation of
    ``TODO.md`` #27 — get it backwards and the model is the slide again, wearing a hinge.

    The hinge axis is ``(0, -1, 0)`` for every segment, not a per-segment vector: the wheel
    lies in the x-z plane, so ``-ĵ × e_r = e_t`` identically, and a positive rotation carries
    every tip toward its own ``+e_t``. The rotational inertia about that hinge is whatever the
    offset capsule gives — derived from the mass and the geometry, never chosen (invariant 2).

    **Refused on a banded spec**, matching the analytic solvers. The refusal is not a
    limitation of MuJoCo — it would happily integrate it — it is that the model would be
    wrong. :func:`coupling_tendons` couples the *radial* joints only, because the analytic
    band is an energy in the compressions; give the segments a second freedom as well and they
    can shear past each other with **nothing resisting**, which is the one deformation a shear
    band exists to carry. A silently softer ring is exactly the failure this project keeps
    finding, so it raises instead.

    ``radial_damping`` and ``tangential_damping_c`` become the joints' own ``damping``
    attributes, in that joint's units. **They must arrive here and not through
    ``qfrc_applied``**, and the difference is not stylistic: ``implicitfast`` folds a joint's
    native damping into the implicit velocity step and integrates an applied force explicitly.
    A dashpot integrated explicitly is stable only while ``c·h < 2·I_eff``, and ``I_eff`` for
    these joints is **not** the segment mass. Measured 2026-08-09 on the 12-claw R 60 mm ring:
    the hinge's composite inertia is 3.26e-6 kg·m² but its effective inertia — one unit torque
    in, ``qacc`` out, everything else free to react — is 3.03e-7, and the collective mode
    across twelve claws whose axes are all parallel to the axle is a further order down. The
    physically derived loss-factor damping then blew up at ``c·h/I ≈ 9``, from round-off, in
    free flight, before the wheel had touched anything. Passing the *same* number as native
    damping is not the "dissipation no material supplied" that
    :func:`stable_timestep_s` rejects — it is the same dissipation, integrated stably.

    ``prefix`` namespaces every emitted name — bodies, joints and geoms alike. Empty by
    default, so a single-wheel model is the XML it always was. It exists because MJCF names
    are global: a four-wheel rover nesting this subtree four times collides on ``seg0`` and
    the model will not compile. Pass the *same* prefix to :func:`coupling_tendons`, which
    references these joints by name and would otherwise wire every wheel's band to the first
    wheel's joints — a model that compiles and is wrong, which is worse than one that does not.

    Raises:
        ValueError: on an unknown element name, on a banded spec, or — for ``"hinge"`` — on a
            spec with no ``root_radius_m``.
    """
    if tangential is not None and tangential not in TANGENTIAL_ELEMENTS:
        raise ValueError(
            f"unknown tangential element {tangential!r}; expected one of "
            f"{TANGENTIAL_ELEMENTS} or None"
        )
    if tangential is not None and spec.is_coupled:
        raise ValueError(
            "a tangential freedom needs a band that shears; coupling_tendons couples only "
            "the radial joints, so a banded ring with a second freedom would shear for free"
        )
    if tangential == "hinge" and spec.root_radius_m <= 0.0:
        raise ValueError(
            "a root hinge needs a root: RingSpec.root_radius_m is 0, which would pivot every "
            "claw about the axle. Build the spec with ring_for_design"
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
    body_radius = segment_body_radius_m(spec)
    hinged = tangential == "hinge"
    # Omitted entirely when zero, so a ring nobody has asked to damp emits the XML it always
    # did and no earlier result silently acquires dissipation.
    rad_damp = f' damping="{radial_damping:.9g}"' if radial_damping else ""
    tan_damp = f' damping="{tangential_damping_c:.9g}"' if tangential_damping_c else ""
    # Where the body sits, and how far the capsule is carried out from it. For the hinge the
    # body is the pivot and the capsule is the tip; otherwise the body *is* the tip and the
    # offset is exactly zero, so the geometry is unchanged from before the hinge existed.
    origin_radius = hinge_pivot_radius_m(spec) if hinged else body_radius
    arm = body_radius - origin_radius

    lines: list[str] = []
    for i, angle in enumerate(theta):
        # Outward radial unit vector for this segment; segment 0 points at the floor.
        ux, uz = float(np.sin(angle)), float(-np.cos(angle))
        px, pz = origin_radius * ux, origin_radius * uz
        lines.append(f'{pad}<body name="{prefix}seg{i}" pos="{px:.9f} 0 {pz:.9f}">')
        if hinged:
            # Before the slide, so the slide's axis rotates with it. Limits at +-pi/2: past
            # that the claw points back into the hub, which this model does not describe, and
            # a run that reaches the stop has already collapsed.
            lines.append(
                f'{pad}  <joint name="{prefix}t{i}" type="hinge" axis="0 -1 0" '
                f'range="{-0.5 * np.pi:.9f} {0.5 * np.pi:.9f}" limited="true"{tan_damp}/>'
            )
        lines.append(
            # Positive q is outward. The old upper bound of 0.01 m was written when nothing
            # could push a segment outward at all; with the band there, segments beside the
            # patch bulge, and a joint limit that quietly caps that bulge would look like a
            # stiffness disagreement rather than the constraint it is. Both bounds are now
            # far outside anything physical, so hitting one means a bug, not a design.
            f'{pad}  <joint name="{prefix}j{i}" type="slide" axis="{ux:.9f} 0 {uz:.9f}" '
            f'range="{-0.9 * r:.9f} {0.5 * r:.9f}" limited="true"{rad_damp}/>'
        )
        if tangential == "slide":
            # Perpendicular to the radius, in the wheel's plane. Segment 0 points down, so
            # its tangential axis is +x, along the rolling direction — which is the direction
            # a claw bends when it catches a step, and the reason this joint exists.
            tx, tz = float(np.cos(angle)), float(np.sin(angle))
            lines.append(
                f'{pad}  <joint name="{prefix}t{i}" type="slide" axis="{tx:.9f} 0 {tz:.9f}" '
                f'range="{-0.5 * r:.9f} {0.5 * r:.9f}" limited="true"{tan_damp}/>'
            )
        # The `+ 0.0` normalises a negative zero: `arm` is exactly 0 without the hinge, and
        # `0.0 * -1.0` formats as "-0.000000000", which is legal MJCF and a diff for nothing.
        gx, gz = arm * ux + 0.0, arm * uz + 0.0
        lines += [
            # The capsule spans the wheel's WIDTH, i.e. along y, the axle direction.
            # Laying it along x instead makes each segment a 30 mm bar in the rolling
            # direction: neighbours 15.7 mm apart then overlap permanently, and the model
            # reports tens of newtons of contact force before the floor is even touched.
            (f'{pad}  <geom name="{prefix}g{i}" type="capsule" fromto="'
             f'{gx:.9f} {-segment_half_width_m:.9f} {gz:.9f} '
             f'{gx:.9f} {segment_half_width_m:.9f} {gz:.9f}" '
             f'size="{seg_r * 0.5:.9f}" mass="{segment_mass_kg}" '
             f'contype="{contype}" conaffinity="{conaffinity}"'
             f'{"" if rgba is None else f" rgba={_rgba(rgba)}"}/>'),
            f"{pad}</body>",
        ]
    return lines


def segment_body_radius_m(spec: RingSpec) -> float:
    """Radius at which a segment's capsule *centre* sits, metres.

    ``R`` minus the capsule's own radius, so that the running *surface* is at ``R``. Shared by
    the body layout and by anything that needs the hinge's moment arm, because computing it
    twice is how the geometry and the dynamics come to disagree.
    """
    return spec.radius_m - 0.25 * spec.segment_arc_m


def hinge_pivot_radius_m(spec: RingSpec) -> float:
    """Radius at which the hinge joint is placed, metres — **not** ``root_radius_m``.

    It sits one capsule radius inboard of the true root, and that offset is deliberate.

    The moment the floor applies about the pivot is ``λ`` times the *horizontal* distance from
    the pivot to the contact point. On a plane the contact point is directly below the
    capsule's centre, so that distance is the pivot-to-**centre** arm, not the pivot-to-tip
    one — the capsule's own radius contributes nothing horizontal. Pivot at the true root and
    the arm comes out ``L - ρ``: 9.8% short at 24 segments on the R 60 mm claw, 4.9% at 48.
    Offsetting the pivot by the same ``ρ`` makes it exactly ``L``.

    That 10% is not a rounding. A short arm means less moment for the same contact force,
    which means less rotation, which means a claw *stiffer* than the one the FEA sweep fitted
    — and the question this element exists to answer is whether the wheel folds over. Erring
    stiff is erring toward "it does not", which is the answer we would like to hear and
    therefore the one to be careful about.

    What it costs: the pivot is ``ρ`` nearer the axle than the real claw root, so the tip's
    distance from the hub centre shrinks along a slightly different curve. That is second
    order in ``ρ/R_root`` and does not touch the sign, where the arm error is first order in
    the quantity the model turns on.

    Raises:
        ValueError: if the capsule is larger than the hub radius, which would put the pivot at
            or through the axle. That means far too few segments for the wheel, not a bug.
    """
    capsule = 0.25 * spec.segment_arc_m
    pivot = spec.root_radius_m - capsule
    if pivot <= 0.0:
        raise ValueError(
            f"the capsule radius {capsule * 1e3:.2f} mm is not smaller than the claw root "
            f"radius {spec.root_radius_m * 1e3:.2f} mm, so the hinge would sit at or beyond "
            f"the axle. {spec.n_segments} segments is too few for this wheel"
        )
    return pivot


def hinge_arm_m(spec: RingSpec) -> float:
    """Pivot-to-capsule-centre distance for the hinge element, metres.

    Exactly :attr:`RingSpec.claw_length_m` by construction — see
    :func:`hinge_pivot_radius_m`, which is where that construction is argued. Kept as its own
    function because it is the moment arm, and a reader following the dynamics should not have
    to rediscover that the two are the same number.
    """
    return segment_body_radius_m(spec) - hinge_pivot_radius_m(spec)


def coupling_tendons(spec: RingSpec, *, prefix: str = "",
                     indent: int = 2) -> list[str]:
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
            lines.append(f'    <fixed name="{prefix}bend{i}" '
                         f'stiffness="{spec.band_bending_n_per_m:.9g}" springlength="0">')
            # Only the three non-zero entries of the row; a coef of 0 is legal but makes the
            # XML N times bigger and hides which segments are actually neighbours.
            for j in sorted({(i - 1) % spec.n_segments, i, (i + 1) % spec.n_segments}):
                lines.append(
                    f'      <joint joint="{prefix}j{j}" coef="{operator[i, j]:.9g}"/>')
            lines.append("    </fixed>")
    if spec.band_hoop_n_per_m > 0.0:
        lines.append(f'    <fixed name="{prefix}hoop" '
                     f'stiffness="{spec.band_hoop_n_per_m:.9g}" '
                     'springlength="0">')
        lines += [f'      <joint joint="{prefix}j{i}" coef="1"/>'
                  for i in range(spec.n_segments)]
        lines.append("    </fixed>")
    lines.append("  </tendon>")
    return lines


def _load(
    spec: RingSpec,
    law: RadialLaw,
    *,
    tangential_law: RadialLaw | None = None,
    tangential_element: TangentialElement | None = None,
    **kwargs,
) -> tuple[RingModel, object, object]:
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise MissingMuJoCo("MuJoCo is not installed; pip install -e '.[sim]'") from exc

    element = resolve_tangential_element(tangential_law, tangential_element)
    xml = build_mjcf(spec, law, tangential=element, **kwargs)
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
        tangential_joints=(ids("t") if element is not None
                           else np.empty(0, dtype=np.int64)),
        tangential_law=tangential_law,
        tangential_element=element,
    )
    return ring, model, data


def tangential_damping(
    spec: RingSpec, element: TangentialElement | None, linear_n_s_per_m: float
) -> float:
    """The second freedom's damping, in whatever units its own coordinate needs.

    A slide's coordinate is a length and takes ``linear_n_s_per_m`` unchanged. A hinge's is an
    angle, and a dashpot of rate ``c`` acting at the tip is ``c·arm²`` about the root — the
    same conversion as ``m·arm²`` for inertia. Passing the linear number straight to a hinge
    is not a mild mistuning: on the R 60 mm claw ``arm² = 1.6e-3``, so 4 N·s/m becomes
    4 N·m·s/rad, roughly **1800× critical** for that joint, and every static press would
    return the undeformed ring while looking like a stiffness result.
    """
    if element != "hinge":
        return linear_n_s_per_m
    arm = hinge_arm_m(spec)
    return linear_n_s_per_m * arm * arm


def resolve_tangential_element(
    tangential_law: RadialLaw | None, tangential_element: TangentialElement | None
) -> TangentialElement | None:
    """Which element a caller meant, given a law and an optional element name.

    One rule in one place, because there are three callers and the failure mode is silent: a
    hinge driven by a slide law applies a torque in N·m numerically equal to a force in N,
    which on the R 60 mm claw is a factor of ``L`` = 0.04 out and reads as a soft wheel.

    - no law: no freedom, whatever was named. A joint nothing drives is a free joint.
    - a law and no name: **hinge**, the element ``TODO.md`` #27 concluded is the right one.
      Defaulting to the slide would keep old call sites silently on the wrong element.
    - a law and a name: the name.
    """
    if tangential_law is None:
        return None
    element = tangential_element or "hinge"
    if element not in TANGENTIAL_ELEMENTS:
        raise ValueError(
            f"unknown tangential element {element!r}; expected one of {TANGENTIAL_ELEMENTS}"
        )
    return element


def static_load_deflection(
    spec: RingSpec,
    law: RadialLaw,
    delta_m: np.ndarray,
    *,
    tangential_law: RadialLaw | None = None,
    tangential_element: TangentialElement | None = None,
    settle_s: float = 1.5,
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
        tangential_law: if given, each segment also gets a second freedom and this law
            resists it, **symmetrically** — a claw bends the same either way, so the
            generalised force is ``sign(q)·f(|q|)`` and not ``f(q)``, which would make one
            direction free and the other doubled.
        tangential_element: ``"slide"`` or ``"hinge"``; defaults to the hinge whenever a law
            is given. See :func:`resolve_tangential_element`.
        settle_s: **simulated seconds** per δ, with the segments damped, not a step count —
            the timestep is chosen by :func:`stable_timestep_s` and varies with the laws, so a
            fixed count would silently shorten the relaxation on a stiff design and return a
            ring that had not finished moving.
        damping: joint damping, N·s/m. Sets how fast it settles, not where it settles; the
            hinge gets the equivalent torsional value from :func:`tangential_damping`. Emitted
            as the joints' native ``damping`` rather than applied as a force, so it is
            integrated implicitly — see :func:`ring_bodies`.
    """
    import mujoco

    deltas = np.atleast_1d(np.asarray(delta_m, dtype=np.float64))
    out = np.empty(len(deltas), dtype=np.float64)
    force6 = np.zeros(6, dtype=np.float64)
    element = resolve_tangential_element(tangential_law, tangential_element)
    equivalent = (TipEquivalentLaw(tangential_law, hinge_arm_m(spec))
                  if element == "hinge" else tangential_law)
    timestep = stable_timestep_s([law, equivalent], SEGMENT_MASS_KG, STATIC_TIMESTEP_S)
    settle_steps = max(1, round(settle_s / timestep))
    tan_damping = tangential_damping(spec, element, damping)

    for k, delta in enumerate(deltas):
        ring, model, data = _load(spec, law, tangential_law=tangential_law,
                                  tangential_element=tangential_element,
                                  indentation_m=float(delta), timestep_s=timestep,
                                  radial_damping=damping,
                                  tangential_damping_c=tan_damping)
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
            data.qfrc_applied[radial_dof] = law.force_n(u)
            if tangential_law is not None:
                v = data.qpos[tan_qpos]
                data.qfrc_applied[tan_dof] = -symmetric_force_n(tangential_law, v)
            mujoco.mj_step(model, data)

        total = 0.0
        for c in range(data.ncon):
            mujoco.mj_contactForce(model, data, c, force6)
            # Contact frame x is the normal; rotate it back into world coordinates.
            frame = data.contact[c].frame.reshape(3, 3)
            total += float((frame.T @ force6[:3])[2])
        out[k] = abs(total)

    return out
