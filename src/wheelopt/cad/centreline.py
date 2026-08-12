"""Spoke centreline and outline generation — pure numpy, no OCCT.

Separated from :mod:`wheelopt.cad.compliant_spoke` deliberately: the geometry *maths* is
testable without an OCCT install, leaving the build123d layer as a thin adapter that only
turns point arrays into faces. When the CAD kernel changes, this module does not.

All coordinates are millimetres in the wheel's XY plane, hub centred at the origin.
"""

from __future__ import annotations

import numpy as np

from .params import SpokeProfile, WheelParams

__all__ = [
    "OVERLAP_FRACTION",
    "attachment_overlap_mm",
    "min_gap_between_spokes",
    "spoke_centreline",
    "spoke_outline",
]

#: Spokes are buried this fraction of their own thickness into the hub and shear band, so
#: the unions are between overlapping solids rather than face-coincident ones.
OVERLAP_FRACTION = 0.75


def attachment_overlap_mm(params: WheelParams) -> float:
    """How far to bury the spoke ends in the hub and the shear band, millimetres.

    The overlap only has to beat the kernel's tolerance — coincident faces are what break a
    boolean union, and a few tenths of a millimetre is enough. What it must **not** do is
    exceed the members it is burying into.

    That second condition used to hold by luck. At a 2 mm spoke the naive
    ``0.75 * thickness`` is 1.5 mm against a 3 mm shear band, which is exactly the cap
    below. At the 7 mm spoke the then-24.5 N platform needed, it is 5.25 mm against the same
    3 mm band — so the spokes ran straight through the running surface and the wheel came
    out 175 mm across while reporting an 85 mm radius. Mass, inertia and the FEA tread node
    set were all wrong together, and every other check still passed.
    """
    overlap = OVERLAP_FRACTION * params.spoke_thickness_mm
    if params.has_shear_band:
        overlap = min(overlap, 0.5 * params.rim_thickness_mm)
    inward = 0.5 * (params.hub_radius_mm - params.hub_bore_radius_mm)
    return max(min(overlap, inward), 0.0)


def spoke_centreline(params: WheelParams, index: int, overlap_mm: float = 0.0) -> np.ndarray:
    """Centreline of spoke ``index`` as an ``(n, 2)`` array of XY points, millimetres.

    The centreline runs from the hub outer surface to the shear band inner surface along a
    ray at the spoke's pitch angle, offset tangentially according to the profile.

    Args:
        params: wheel geometry.
        index: spoke number, ``0 <= index < params.n_spokes``.
        overlap_mm: extend the centreline this far *into* the hub and the shear band at
            each end. A small overlap makes the boolean union robust — coincident faces
            are the classic source of OCCT union failures. Applied at the outer end only
            when there is a shear band to overlap into.
    """
    n = int(params.spoke_samples)
    theta = np.deg2rad(params.spoke_phase_deg) + index * params.spoke_pitch_angle_rad

    radial = np.array([np.cos(theta), np.sin(theta)])
    tangential = np.array([-np.sin(theta), np.cos(theta)])

    r_start = params.hub_radius_mm - overlap_mm
    # Without a shear band the tip *is* the running surface, so it must land exactly on the
    # outer radius: there is nothing outboard to overlap into, and overlapping anyway would
    # make the wheel silently larger than its own `outer_radius_mm`.
    r_end = params.rim_inner_radius_mm + (overlap_mm if params.has_shear_band else 0.0)

    # Normalised station along the span, 0 at the hub, 1 at the shear band.
    t = np.linspace(0.0, 1.0, n)
    radii = r_start + t * (r_end - r_start)

    sag = params.spoke_sagitta_mm
    profile = params.spoke_profile

    if profile is SpokeProfile.STRAIGHT:
        offset = np.zeros_like(t)
    elif profile is SpokeProfile.CURVED:
        # Single half-sine bulge: zero at both ends, maximum at mid-span. Matches the
        # sagitta definition in WheelParams and keeps the attachment angles radial, which
        # is what makes the FEA boundary conditions clean.
        offset = sag * np.sin(np.pi * t)
    elif profile is SpokeProfile.S_CURVE:
        # Full sine: bulges one way over the inner half, the other way over the outer half.
        # Distributes bending along the span instead of concentrating it at mid-span.
        offset = sag * np.sin(2.0 * np.pi * t)
    else:  # pragma: no cover - enum is exhaustive
        raise ValueError(f"unhandled spoke profile {profile!r}")

    points = radii[:, None] * radial[None, :] + offset[:, None] * tangential[None, :]
    if params.is_l_claw:
        points = _append_hook(params, points, theta)
    return points


#: Minimum samples spent on the right-angle bend of an L claw, whatever its arc length.
#: The bend is a few millimetres against a span of tens, so proportional allocation gives it
#: two or three points — and :func:`_unit_normals` takes central differences, so a corner
#: described by three points has its normal swing 90 degrees in one step. The outline then
#: has a notch on the inside of the bend instead of a fillet: geometrically a self-intersection,
#: and a face OCCT refuses.
HOOK_BEND_SAMPLES = 9


def _append_hook(params: WheelParams, leg: np.ndarray, theta: float) -> np.ndarray:
    """Turn a radial claw centreline into an L: bend, then a foot along the running surface.

    Three pieces, and the second and third are built in **polar** coordinates rather than in
    the spoke's local Cartesian frame that the leg uses. That is not fussiness. A foot built as
    a straight line at constant local ``u`` is a *chord*, and a chord of a 60 mm circle stands
    3.2 mm proud of it at 20 mm out — so the foot would poke through the running surface, and
    :func:`_clip_to_radius` would then eat it from the outside until, at a tapered tip, the
    outline crossed itself. The foot follows the circle because the ground does.

    The pieces:

    1. **The leg**, already computed, but shortened: it now ends where the bend begins, at
       ``sqrt(ρ² − b²)`` rather than at the running surface.
    2. **The bend**, a circular arc of radius ``b`` tangent to the radial ray *and* internally
       tangent to the circle ``r = R_c``. Both tangencies at once fix it: the centre sits at
       radius ``ρ = R_c − b``, at angular offset ``δ = arcsin(b/ρ)``, and the arc sweeps
       ``π/2 − δ``. Exact, so the join is smooth to machine precision rather than to a
       tolerance.
    3. **The foot**, an arc at constant radius ``R_c``.

    ``R_c`` is ``outer_radius_mm`` less half the *tip* thickness, so the foot's outer face lands
    on the running surface rather than the centreline doing so — which would put half the foot
    outside the wheel it belongs to.
    """
    sign = 1.0 if params.tip_hook_mm > 0 else -1.0
    r_c = params.outer_radius_mm - 0.5 * params.tip_thickness_mm
    bend = params.hook_bend_radius_mm
    rho = r_c - bend
    if bend <= 0.0 or rho <= bend:
        # No room to turn the corner. Screening rejects this design; returning the plain claw
        # keeps the geometry layer total, so a caller that skipped screening gets a wheel
        # rather than an exception (invariant 3 in spirit — this module never raises either).
        return leg

    delta = float(np.arcsin(bend / rho))
    r_leg_end = float(np.sqrt(rho * rho - bend * bend))
    # The leg is rescaled rather than rebuilt: it keeps its profile offset, which is zero at
    # both ends, so the bend still starts on a radial heading.
    scale = r_leg_end / np.linalg.norm(leg[-1])
    leg = leg * np.linspace(1.0, scale, len(leg))[:, None] if scale < 1.0 else leg

    centre = rho * np.array([np.cos(theta + sign * delta), np.sin(theta + sign * delta)])
    # Start of the arc is the leg's tangent point; sweep towards the circle's tangent point.
    start = np.arctan2(*(leg[-1] - centre)[::-1])
    sweep = sign * (0.5 * np.pi - delta)
    phi = np.linspace(start, start + sweep, HOOK_BEND_SAMPLES)[1:]
    corner = centre + bend * np.stack([np.cos(phi), np.sin(phi)], axis=1)

    # The foot, from the circle's tangent point onward along the running surface.
    beta = abs(params.tip_hook_mm) / r_c
    n_foot = max(2, round(params.spoke_samples * beta * r_c
                          / max(np.linalg.norm(leg[-1] - leg[0]), 1e-9)))
    angles = theta + sign * (delta + np.linspace(0.0, beta, n_foot + 1)[1:])
    foot = r_c * np.stack([np.cos(angles), np.sin(angles)], axis=1)

    return np.vstack([leg, corner, foot])


def _unit_normals(points: np.ndarray) -> np.ndarray:
    """In-plane unit normals along a polyline, via central differences.

    End points use one-sided differences so the outline stays square at the attachments.
    """
    tangents = np.empty_like(points)
    tangents[1:-1] = points[2:] - points[:-2]
    tangents[0] = points[1] - points[0]
    tangents[-1] = points[-1] - points[-2]

    lengths = np.linalg.norm(tangents, axis=1, keepdims=True)
    lengths = np.where(lengths < 1e-12, 1.0, lengths)
    tangents = tangents / lengths

    # Rotate +90 degrees in-plane.
    return np.stack([-tangents[:, 1], tangents[:, 0]], axis=1)


def _thickness_profile_mm(params: WheelParams, centre: np.ndarray) -> np.ndarray:
    """Section thickness at every point of the centreline, millimetres.

    Linear in **arc length**, not in point index. The centreline is sampled at uniform
    parameter rather than uniform distance, so a curved or S-curve profile bunches its
    points; interpolating on the index would put the taper in the wrong place along the
    span and would move it when the profile changed. Arc length is what a beam's bending
    moment varies along, so it is also the right variable physically.

    Root is at index 0, which for a spoke drawn with an attachment overlap is *inside* the
    hub. Full thickness there is what is wanted — that is the most heavily loaded section
    and it is buried in the joint.
    """
    if params.claw_taper_ratio >= 1.0:
        return np.full(len(centre), params.spoke_thickness_mm, dtype=np.float64)

    steps = np.linalg.norm(np.diff(centre, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(steps)])
    total = arc[-1]
    fraction = arc / total if total > 1e-12 else np.zeros_like(arc)
    scale = 1.0 + (params.claw_taper_ratio - 1.0) * fraction
    return params.spoke_thickness_mm * scale


def spoke_outline(params: WheelParams, index: int, overlap_mm: float = 0.0) -> np.ndarray:
    """Closed outline of one spoke as an ``(2n, 2)`` array of XY points, millimetres.

    The outline walks up one side of the centreline and back down the other. The first
    point is **not** repeated at the end — callers close the loop.
    """
    centre = spoke_centreline(params, index, overlap_mm=overlap_mm)
    normals = _unit_normals(centre)
    half = 0.5 * _thickness_profile_mm(params, centre)[:, None]

    left = centre + half * normals
    right = centre - half * normals

    if not params.has_shear_band:
        # Truncate the tip flush with the running surface.
        #
        # It is not flush on its own. The outline is offset perpendicular to the centreline,
        # and near the outer end a curved centreline is tilted away from radial, so the
        # offset has a radial component and the outboard side of the tip rides *outside*
        # `outer_radius_mm`. At the nominal design that is 389 um; at the corner of the
        # design space (S-curve, kappa 0.03, t 8 mm on a 60 mm wheel) it is 3.3 mm, and it
        # is the shoulder rather than the corner that sticks out furthest.
        #
        # Small enough to read as rounding, large enough to matter: the FEA tread node set
        # is `|r - R| < 0.1 mm`, so the true first-contact material would sit outside it and
        # never be offered to the contact search. With a shear band the question never
        # arose — the band was the running surface and the tips were buried inside it.
        _clip_to_radius(left, params.outer_radius_mm)
        _clip_to_radius(right, params.outer_radius_mm)

    return np.vstack([left, right[::-1]])


def _clip_to_radius(points: np.ndarray, radius_mm: float) -> None:
    """Pull any point outside ``radius_mm`` back onto that circle, in place.

    Radially, so each point keeps its angle: what was a tip bulging past the running
    surface becomes a short flat truncated flush with it.
    """
    norms = np.linalg.norm(points, axis=1)
    outside = norms > radius_mm
    if np.any(outside):
        points[outside] *= (radius_mm / norms[outside])[:, None]


def min_gap_between_spokes(params: WheelParams) -> float:
    """True minimum clearance between adjacent spoke outlines, millimetres.

    The analytic estimate in :attr:`WheelParams.min_interspoke_gap_mm` is taken at the hub
    and is deliberately permissive. For curved and S-curve profiles the true minimum can
    occur anywhere along the span, so this samples both outlines and measures directly.

    Negative means the spokes intersect.
    """
    a = spoke_outline(params, 0)
    b = spoke_outline(params, 1)
    # Pairwise distances between the two outlines. Outlines are O(100) points, so the
    # O(n^2) form is a few microseconds and avoids a scipy dependency.
    deltas = a[:, None, :] - b[None, :, :]
    return float(np.min(np.linalg.norm(deltas, axis=2)))
