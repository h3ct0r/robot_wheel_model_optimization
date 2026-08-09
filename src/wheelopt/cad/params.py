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
    def is_claw(self) -> bool:
        """Whether this is a tapered free-tip claw rather than a uniform strut.

        A claw needs no shear band by definition — the tip is the running surface — so this
        is only true when the taper is real *and* there is nothing for the tip to attach to.
        A tapered spoke buried in a band is a legal shape, just not a claw.
        """
        return self.claw_taper_ratio < 1.0 and not self.has_shear_band

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
    "n_spokes": (6, 36),
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
}
