"""Wheel design parameters.

Units policy (see CLAUDE.md): millimetres are permitted **only** in CAD parameter
definitions, and are converted at the boundary. Every field here is in mm and is suffixed
`_mm`; :meth:`WheelParams.to_si` performs the conversion for downstream consumers.

Nothing in this module imports build123d — it must stay importable without OCCT so that
constraint screening (`wheelopt.cad.constraints`) costs milliseconds.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass
from enum import Enum

__all__ = ["SpokeProfile", "WheelParams", "PARAM_BOUNDS"]


class SpokeProfile(str, Enum):
    """Spoke centreline shape.

    The three implemented profiles span the useful range of buckling behaviour:
    ``STRAIGHT`` buckles unpredictably (Euler column), ``CURVED`` buckles in a direction
    set by the sign of the curvature, and ``S_CURVE`` distributes bending along the span
    and so lowers peak root stress at a given deflection.
    """

    STRAIGHT = "straight"
    CURVED = "curved"
    S_CURVE = "s_curve"


@dataclass(frozen=True, slots=True)
class WheelParams:
    """Geometry of one compliant-spoke wheel (family ``T3``).

    All lengths in millimetres. See ``docs/plan/04-design-space.md`` for the ranges these
    are searched over and the reasoning behind each bound.
    """

    # --- primary geometry -------------------------------------------------------------
    #: Defaults are the *nominal* design for the platform in ``configs/robot.yaml``: a
    #: 400 x 300 x 200 mm, 10 kg four-wheel skid-steer rover carrying 24.5 N per wheel.
    #: They are sized to survive that load, not to be optimal — see the note on
    #: ``spoke_thickness_mm``.
    outer_radius_mm: float = 85.0
    width_mm: float = 45.0
    #: Radial thickness of the shear band. **Zero means there is no shear band at all** —
    #: the spoke tips become the running surface. That is a topology switch, not a point in
    #: the continuous range, so it is exempted from ``PARAM_BOUNDS``; see
    #: :attr:`has_shear_band` for what else changes.
    rim_thickness_mm: float = 3.0
    hub_radius_mm: float = 22.0
    #: Half the drivetrain shaft: an 8 mm D-shaft on this platform. Screening rejects any
    #: design more than 0.5 mm off it — the interface is fixed, not searched.
    hub_bore_radius_mm: float = 4.0

    # --- spoke structure --------------------------------------------------------------
    n_spokes: int = 12
    #: 7 mm is thick for a "compliant" spoke, and it is what the load demands. Euler
    #: buckling of one spoke goes as t^3/L^2, so at a 60 mm span the 24.5 N wheel load —
    #: 61 N at the 2.5x margin the buckling constraint asks for — needs 6-8 mm. Thickening
    #: also *lowers* the effective modulus (a thicker feature has a smaller shell fraction,
    #: so more of it is infill void), which eats some of the gain back. FEA is the arbiter;
    #: this default is only a defensible starting point.
    spoke_thickness_mm: float = 7.0
    #: Signed curvature, 1/mm. Positive bulges in the +tangential direction. The sign
    #: matters: it selects the buckling direction under drive torque.
    spoke_curvature_1_per_mm: float = 0.004
    #: Tip thickness as a fraction of :attr:`spoke_thickness_mm`. **This is what makes a
    #: claw a claw rather than a strut.**
    #:
    #: A bandless spoke is a cantilever with a free tip, so the bending moment is largest at
    #: the root and zero at the tip; a uniform section is therefore over-thick everywhere
    #: except the root, which wastes mass and stiffens the tip exactly where conformity is
    #: wanted. Every claw in the PaTS-Wheel taxonomy (``docs/papers``) tapers.
    #:
    #: 1.0 is the uniform strut this project built first, and is the default so that nothing
    #: predating the claw work changes. Below 1.0 the outline narrows linearly in arc length
    #: from root to tip.
    #:
    #: **The minimum-wall check must read the tip, not the root** — see
    #: :attr:`tip_thickness_mm`. A 7 mm spoke at 0.15 taper has a 1.05 mm tip, which is
    #: unprintable while ``spoke_thickness_mm`` still looks comfortable.
    claw_taper_ratio: float = 1.0
    #: Arc length of a tangential **foot** at the claw tip, millimetres, measured along the
    #: running surface. Zero — the default — is the plain radial claw and changes nothing.
    #: Non-zero turns the claw into a literal **L**: a radial leg, a filleted right-angle
    #: bend, and a foot lying along the circle at ``outer_radius_mm``. Family ``T7L``.
    #:
    #: **Signed, like** :attr:`spoke_curvature_1_per_mm`. Positive puts the foot in the
    #: +tangential direction, negative in −tangential, and the two are genuinely different
    #: wheels once the thing is driven: a foot that trails the leg is dragged onto the ground
    #: and folds *closed* under drive torque, one that leads it is levered *open*. Nothing in
    #: this project measures that difference yet, which is why both signs are expressible and
    #: neither is the default.
    #:
    #: **What it is for.** A radial claw touches the ground at a point, so a bandless wheel is
    #: a polygon and its axle drops ``R(1 − cos π/n)`` per pitch — the harshness measured on
    #: 2026-08-10. A foot spreads that contact over an arc, so the drop is taken over the
    #: *gap between feet* instead: see :attr:`polygon_drop_mm`, which reads this field.
    #:
    #: **Requires a bandless design.** With a shear band the tip is buried in it and there is
    #: nowhere for a foot to go; ``constraints.check_design`` rejects the combination rather
    #: than ignoring the field, which would be the silent-default failure this project keeps
    #: finding.
    tip_hook_mm: float = 0.0
    spoke_profile: SpokeProfile = SpokeProfile.CURVED
    #: Rotational phase of the spoke pattern, degrees; spoke 0 sits at this angle from +x.
    #: Irrelevant with a shear band — the running surface is a cylinder whatever the spokes
    #: do underneath — and **decisive** without one, because then only the tips touch and
    #: the answer depends on whether a tip or a gap faces the ground. Not a design variable
    #: and not in ``PARAM_BOUNDS``; see ``wheelopt.fea.loadcase.phase_for_tip_contact``.
    spoke_phase_deg: float = 0.0

    # --- tread ------------------------------------------------------------------------
    tread_depth_mm: float = 0.0

    # --- discretisation (not a design variable) ---------------------------------------
    #: Samples along a spoke centreline when building its face. Higher is smoother and
    #: slower. Not searched over; changing it changes geometry, so it belongs in the
    #: pipeline version, not the design vector.
    spoke_samples: int = 41

    # ---------------------------------------------------------------------------------
    # Derived quantities
    # ---------------------------------------------------------------------------------

    @property
    def has_shear_band(self) -> bool:
        """False when the spoke tips are the running surface.

        A bandless wheel is a different animal downstream, not just a thinner one:
        contact becomes discrete (``n_spokes`` tips instead of a cylinder, so the response
        depends on :attr:`spoke_phase_deg`), and the ring reduced-order model loses the
        member whose bending stiffness couples adjacent segments — see
        ``docs/plan/06-compliance-rom.md`` §3.
        """
        return self.rim_thickness_mm > 0.0

    @property
    def tip_thickness_mm(self) -> float:
        """Thickness at the free end of the claw — the **thinnest** material in the spoke.

        Every printability check about the spoke belongs on this, not on
        :attr:`spoke_thickness_mm`, which is the root. The two are the same number only
        while :attr:`claw_taper_ratio` is 1.0, which is exactly why a check written against
        the root keeps passing after someone adds a taper.
        """
        return self.spoke_thickness_mm * self.claw_taper_ratio

    @property
    def effective_thickness_mm(self) -> float:
        """Thickness of the **uniform** spoke that bends like this tapered one, millimetres.

        The section a slenderness proxy should read, and the answer to ``docs/plan/TODO.md``
        #21. Reading the root understates slenderness — the root is the stiffest section — and
        errs toward accepting a claw that buckles; reading the tip overstates it by as much.

        Derived, not fitted. A cantilever tapering linearly from ``t0`` at the root to
        ``r·t0`` at the tip has tip compliance ``∫₀ᴸ (L-x)²/(E I(x)) dx`` with ``I ∝ t(x)³``.
        The integral is elementary and gives ``L³/(3 E I₀) · Φ(r)`` with

            Φ(r) = 3[-ln r + 2r - 3/2 - r²/2] / (1 - r)³

        so the uniform spoke of equal tip deflection has ``t_eff = t₀ / Φ(r)^(1/3)``. ``Φ → 1``
        as ``r → 1`` — the expression is 0/0 there and the limit is taken explicitly below,
        because the naive form returns a NaN for the uniform strut that is most of the design
        space.

        **Measured against the alternative that sounds more correct.** The Rayleigh quotient
        for a fixed-free Euler mode is the buckling-theoretic answer and gives a *different*
        effective thickness (7.11 mm against 7.08 at ``r = 0.6``, 6.71 against 6.47 at 0.4).
        On the frictionless claw-sector plate sweep of 2026-08-09 the compliance form collapses
        the taper dependence of the measured plateau load better — a 10% spread across
        ``r = 1.0 → 0.4`` against Rayleigh's 20% and the root's **110%** — so the closed form
        is both cheaper and, on this data, more faithful. The likely reason is that a claw on a
        plate is not an axially loaded column: its tip slides and rotates, so the tip-load
        weighting is nearer the truth than the buckling mode's.
        """
        r = self.claw_taper_ratio
        if r <= 0.0:
            return 0.0
        return self.spoke_thickness_mm / _taper_compliance_factor(r) ** (1.0 / 3.0)

    @property
    def is_claw(self) -> bool:
        """Whether this is a tapered free-tip claw rather than a uniform strut.

        A claw needs no shear band by definition — the tip is the running surface — so this
        is only true when the taper is real *and* there is nothing for the tip to attach to.
        A tapered spoke buried in a band is a legal shape, just not a claw.
        """
        return self.claw_taper_ratio < 1.0 and not self.has_shear_band

    @property
    def is_l_claw(self) -> bool:
        """Whether this claw has a tangential foot at its tip — family ``T7L``.

        Bandless is part of the definition, not a separate check: a foot on a spoke buried in
        a shear band is not an L claw, it is an unbuildable shape, and
        ``constraints.check_design`` says so.
        """
        return self.tip_hook_mm != 0.0 and not self.has_shear_band

    @property
    def hook_bend_radius_mm(self) -> float:
        """Centreline radius of the right-angle bend between leg and foot, millimetres.

        **Not cosmetic, and not free to choose.** The outline is the centreline offset by half
        the local thickness, and offsetting a corner of centreline radius ``ρ`` by ``h`` makes
        the inside face a circle of radius ``ρ − h``: at ``ρ = h`` it degenerates to a point
        and below it the polygon **turns inside out**. A self-intersecting outline is not a
        drawing artefact — it is a face OCCT will refuse or, worse, fuse into a solid with a
        reversed patch. So the bend is ``0.75 t_tip`` against a half-thickness of ``0.5 t_tip``,
        a 1.5x margin, and is capped at half the foot so a short hook stays a hook rather than
        becoming all fillet.
        """
        return min(0.75 * self.tip_thickness_mm, 0.5 * abs(self.tip_hook_mm))

    @property
    def contact_patch_mm(self) -> float:
        """Length of one tip's ground footprint, millimetres. Zero with a shear band.

        **The third correction to the same line, so it lives here now rather than in a
        message.** A uniform strut's footprint is its thickness; the check quoted
        :attr:`spoke_thickness_mm` and was right until a taper appeared, whereupon it
        overstated the patch by ``1/taper``. It was moved to :attr:`tip_thickness_mm`, which is
        right until a *foot* appears — an L claw lies on the ground along its whole foot and
        touches over ``|tip_hook_mm|``, which on the R 60 twelve-claw design is 12 mm against
        the 3.6 mm the tip would report. Reading the wrong one understates the patch by 3.3x
        and does so while looking entirely reasonable.
        """
        if self.has_shear_band:
            return 0.0
        return abs(self.tip_hook_mm) if self.is_l_claw else self.tip_thickness_mm

    @property
    def contact_arc_rad(self) -> float:
        """Angular span of one claw's running surface, radians. Zero without a foot.

        ``|tip_hook_mm| / outer_radius_mm`` — the foot's arc, and deliberately **not** the
        bend's, which also comes within a hair of the running surface near its tangent point.
        Understating the contact arc understates the benefit of a foot, which is the safe
        direction for a quantity used to argue that this topology is worth having.
        """
        if not self.is_l_claw or self.outer_radius_mm <= 0.0:
            return 0.0
        return abs(self.tip_hook_mm) / self.outer_radius_mm

    @property
    def rim_inner_radius_mm(self) -> float:
        """Inner radius of the shear band, where the spokes attach.

        With no shear band this equals :attr:`outer_radius_mm`, which is what makes the
        spoke tips land exactly on the running surface.
        """
        return self.outer_radius_mm - self.rim_thickness_mm

    @property
    def spoke_span_mm(self) -> float:
        """Radial distance a spoke bridges. Negative means the design is degenerate."""
        return self.rim_inner_radius_mm - self.hub_radius_mm

    @property
    def spoke_pitch_angle_rad(self) -> float:
        return 2.0 * math.pi / self.n_spokes

    @property
    def polygon_drop_mm(self) -> float:
        """Ride-height drop of this wheel **treated as rigid**, over one spoke pitch.

        ``R(1 - cos(π/n))``. Only meaningful without a shear band, where the running surface
        is ``n_spokes`` discrete tips and the wheel is therefore a regular polygon: the axle
        rides at ``R`` with a tip straight down and falls to ``R cos(π/n)`` midway between
        two, ``n`` times a revolution.

        **Half the pitch, not the whole one** — the neighbouring ``R(1 - cos(2π/n))`` is the
        second-claw *engagement* threshold, a different question with a similar formula.

        **It is not a bound on the real ripple**, in either direction. A stiff claw rides
        smoother than this because it deflects into the gap; a very soft one rides *rougher*,
        because at deflections of a fifth of the radius the phase changes how many claws carry
        the load and that swamps the geometry — measured at 4x this value on an R 85 mm,
        8-claw, 3.7 N/mm design. The honest number needs the fitted law:
        :func:`wheelopt.rom.ring.ride_height_ripple_m`. This one is here because it costs
        nothing, needs no FEA, and is the right thing for a millisecond pre-filter to report.

        **An L claw's foot changes the formula, and that is the whole point of the foot.** A
        radial claw touches at one point, so the free half-pitch is ``π/n``. A foot spreads the
        contact over :attr:`contact_arc_rad`, so the axle only falls over what is left between
        two feet: ``R(1 − cos(π/n − β/2))``. At ``β ≥ 2π/n`` the feet meet and the running
        surface is continuous — the drop is exactly zero, and the wheel has become a very
        strangely constructed solid tyre.
        """
        half_gap = math.pi / self.n_spokes - 0.5 * self.contact_arc_rad
        if half_gap <= 0.0:
            return 0.0
        return self.outer_radius_mm * (1.0 - math.cos(half_gap))

    @property
    def spoke_sagitta_mm(self) -> float:
        """Mid-span tangential offset of the centreline from the chord.

        Small-curvature approximation ``s = kappa * L^2 / 8``. Exact for the shallow arcs
        this design space uses; the error at the upper curvature bound is under 2%.
        """
        if self.spoke_profile is SpokeProfile.STRAIGHT:
            return 0.0
        return self.spoke_curvature_1_per_mm * self.spoke_span_mm**2 / 8.0

    # NOTE: there is deliberately no analytic inter-spoke gap approximation here.
    # An earlier version screened on the arc pitch at the hub, which over-reports the gap
    # at low spoke counts and *under*-reports it at high ones — so it was permissive in
    # one part of the design space and silently rejected feasible designs in another.
    # The exact geometric check (`centreline.min_gap_between_spokes`) costs microseconds,
    # so the approximation bought nothing and risked a second source of truth.

    @property
    def bounding_box_mm(self) -> tuple[float, float, float]:
        d = 2.0 * self.outer_radius_mm
        return (d, d, self.width_mm)

    # ---------------------------------------------------------------------------------

    def to_si(self) -> dict[str, float]:
        """Convert every ``*_mm`` field to metres. Curvature converts to 1/m.

        Order matters: ``_1_per_mm`` must be tested before ``_mm``, since a reciprocal
        length also ends in ``_mm`` but scales by 1e3, not 1e-3. Getting this backwards
        makes curvature wrong by a factor of a million — large enough to be caught, but
        only if something downstream checks.
        """
        out: dict[str, float] = {}
        for key, value in asdict(self).items():
            if key.endswith("_1_per_mm"):
                out[key[: -len("_1_per_mm")] + "_1_per_m"] = float(value) * 1e3
            elif key.endswith("_mm"):
                out[key[:-3] + "_m"] = float(value) * 1e-3
            else:
                out[key] = value
        return out

    def design_hash(self) -> str:
        """Stable content hash of the design vector, for cache keys.

        Does **not** include the pipeline or ROM version — the caller composes those in.
        See invariant 5 in CLAUDE.md.
        """
        payload = asdict(self)
        payload["spoke_profile"] = self.spoke_profile.value
        blob = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(blob.encode()).hexdigest()[:16]


def _taper_compliance_factor(r: float) -> float:
    """``Φ(r) = 3[-ln r + 2r - 3/2 - r²/2] / (1 - r)³`` — by how much a taper softens a
    cantilever's tip. ``Φ(1) = 1``; ``Φ(0.25) = 2.525``.

    Two branches, and the second is not a nicety. The bracket is
    ``k³/3 + k⁴/4 + k⁵/5 + …`` with ``k = 1 - r``, assembled by cancelling four terms of order
    one — so near ``r = 1`` it is catastrophic cancellation. At ``r = 0.999999`` the true value
    is 3e-19 and the direct form returns a small **negative** number, whose cube root in Python
    is *complex*. An untapered spoke is most of this design space, and a complex effective
    thickness propagates into a comparison that raises somewhere else entirely.

    So the series is used for ``k ≤ 0.1``, where 30 terms reach full double precision, and the
    closed form beyond it, where it is good to 1e-13. The two agree to 1e-14 across the join.
    """
    k = 1.0 - r
    if abs(k) <= 0.1:
        # Φ = 1 + Σ_{n≥1} 3 kⁿ/(n+3), the series of the bracket divided by k³/3.
        total, power = 1.0, 1.0
        for n in range(1, 31):
            power *= k
            total += 3.0 * power / (n + 3)
        return total
    return 3.0 * (-math.log(r) + 2.0 * r - 1.5 - 0.5 * r * r) / (k * k * k)


#: Search bounds, mirroring ``docs/plan/04-design-space.md``. Kept here so the constraint
#: pre-filter and the optimiser read the same numbers.
#:
#: ``rim_thickness_mm`` has one legal value outside its range: exactly ``0.0``, meaning no
#: shear band. The pre-filter exempts it, because it is a topology choice rather than a
#: continuous variable — widening the range to ``[0, 6]`` instead would quietly admit the
#: 0.3 mm rims in between, which no nozzle can print.
PARAM_BOUNDS: dict[str, tuple[float, float]] = {
    # Upper limit is the print bed, not the robot: 100 mm radius is a 200 mm disc on a
    # 220 mm bed. The chassis would happily take more.
    "outer_radius_mm": (60.0, 100.0),
    "width_mm": (30.0, 70.0),
    "rim_thickness_mm": (1.2, 8.0),
    # Lowered 6 -> 3 on 2026-08-10 to admit **deliberately bad** baselines: a design that a
    # good one has to beat is worth more than a bound that only admits plausible wheels, and
    # a 3-spoke wheel is genuinely awful — its axle drops `R(1 - cos 60 deg)` = half a radius
    # between tips, 30 mm on a 60 mm wheel.
    #
    # This does **not** reverse TODO #19, which asked whether claws want *fewer* tips and
    # measured that they want more: a passive claw wheel unloads a claw completely once per
    # pitch below about 12 tips, whatever the claw's stiffness, and 4 (the PaTS-Wheel letter's
    # row) is only reachable because those claws are gear-driven rather than passive springs.
    # That finding stands and is still reported, as the WARNING
    # `constraints.claw_ride_harshness` raises whenever there is no band. What changed is only
    # that the *search space* no longer refuses to express a bad wheel.
    #
    # **Three, not one, and the floor is the model rather than taste.** Below three the
    # formulas stop meaning anything instead of merely reporting badly: `polygon_drop_m` at
    # n=1 is `R(1 - cos 180 deg)` = 2R, an axle dropping twice the wheel radius, and
    # `second_contact_delta_m` at n=1 is `R(1 - cos 360 deg)` = **0**, which reads as "a second
    # claw engages immediately" on a wheel that has no second claw. `RingSpec` refuses fewer
    # than three segments for the same reason, and `check_design` calls one or two spokes
    # DEGENERATE — geometry that cannot be built rather than a design that scores poorly.
    "n_spokes": (3, 36),
    # The lower bound sits *below* the minimum printable TPU wall (1.6 mm) on purpose, so
    # that `spoke_min_wall` stays a live check rather than being made unreachable by the
    # range. The upper bound is set by buckling at 24.5 N — see WheelParams.
    "spoke_thickness_mm": (1.2, 8.0),
    "spoke_curvature_1_per_mm": (-0.03, 0.03),
    # 1.0 is a uniform strut. The lower bound is not a printability limit — `spoke_min_wall`
    # reads `tip_thickness_mm` and enforces that separately, and must stay the binding check
    # so that a thin root and an aggressive taper cannot slip through by each looking fine.
    # 0.25 is where a tapered cantilever stops behaving like a beam and starts behaving like
    # a hinge, which is a different model, not a thinner one.
    "claw_taper_ratio": (0.25, 1.0),
    "tread_depth_mm": (0.0, 4.0),
    # Symmetric like the curvature, and for the same reason: the sign is a design choice
    # (a trailing foot folds closed under drive torque, a leading one is levered open), not
    # a magnitude with a direction bolted on. Zero is *inside* this range and means no foot,
    # unlike `rim_thickness_mm` whose zero is a topology switch outside its range — here the
    # plain radial claw is the continuous limit of a shortening foot, not a different animal.
    #
    # The magnitude bound is loose on purpose. What actually limits a foot is the arc to the
    # next claw, which is `2 pi R / n` and runs from 31 mm (R 60, twelve claws) to 209 mm
    # (R 100, three) — no scalar can express that, so `hook_reach` and `interspoke_gap` do it
    # exactly and this only keeps the search inside printable, meshable sizes.
    "tip_hook_mm": (-40.0, 40.0),
}
