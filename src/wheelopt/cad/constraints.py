"""Constraint pre-filter.

Invariant 3 (CLAUDE.md): constraints are a **fast pre-filter returning a typed violation
vector**. An infeasible design must cost milliseconds, not a 6-minute FEA run, and must
never raise. Nothing in this module imports build123d or touches the filesystem.

The checks here are *screening* checks — analytic, conservative, and deliberately
permissive where geometry is needed for certainty. The authoritative geometric checks
(true minimum inter-spoke gap, self-contact at deflection, watertightness) run later,
against the actual solid.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .centreline import min_gap_between_spokes
from .materials import MaterialSpec
from .params import PARAM_BOUNDS, WheelParams

__all__ = ["Severity", "Violation", "PlatformLimits", "check_design", "is_feasible"]


class Severity(str, Enum):
    #: Geometry cannot be built at all — degenerate or self-inconsistent.
    DEGENERATE = "degenerate"
    #: Geometry builds, but the design is outside the searchable envelope or unprintable.
    INFEASIBLE = "infeasible"
    #: Buildable and printable, but likely to behave badly. Logged, not rejected.
    WARNING = "warning"


@dataclass(frozen=True, slots=True)
class Violation:
    """One failed check. ``margin`` is negative when violated, by construction."""

    name: str
    severity: Severity
    value: float
    limit: float
    margin: float
    message: str

    def __str__(self) -> str:  # pragma: no cover - display only
        return f"[{self.severity.value}] {self.name}: {self.message}"


@dataclass(frozen=True, slots=True)
class PlatformLimits:
    """The subset of ``configs/robot.yaml`` that geometry screening needs.

    Defaults are placeholders and deliberately generous. Real values come from the frozen
    platform spec; a campaign run against these defaults is not a valid campaign.
    """

    #: Geometric room for the wheel. On this platform the wheels hang outboard of a
    #: 400 x 300 x 200 mm chassis, so nothing encloses them — the binding limit is the
    #: print bed below, and this sits just inside it.
    wheel_well_radius_mm: float = 105.0
    max_width_mm: float = 70.0
    #: Half of an 8 mm D-shaft. Four 24.5 N wheels with climbing torque need more than the
    #: 6 mm shaft a 4 kg robot would use.
    shaft_radius_mm: float = 4.0
    #: Bambu Lab X1C, matching `manufacturing.bed_size` in configs/robot.yaml. These two
    #: used to disagree; the YAML is the one that describes the actual printer, and
    #: `tests/test_platform.py` fails the moment they drift again — which is how the move
    #: from the 220x220 Ender-3 to this was caught.
    #:
    #: The binding number is the **smallest** dimension, 225 mm, because a wheel's bounding
    #: box is 2R x 2R x width and it has to fit however it is oriented. So this caps the
    #: radius at 112.5 mm, just above the 105 mm wheel well that binds first anyway.
    bed_size_mm: tuple[float, float, float] = (250.0, 225.0, 250.0)
    #: Nozzle traversal clearance between adjacent spokes.
    min_interspoke_gap_mm: float = 2.0
    min_wall_thickness_tpu_mm: float = 1.6
    min_wall_thickness_rigid_mm: float = 1.2


def _violation(
    name: str,
    severity: Severity,
    value: float,
    limit: float,
    message: str,
    *,
    lower_bound: bool,
) -> Violation:
    margin = (value - limit) if lower_bound else (limit - value)
    return Violation(name, severity, value, limit, margin, message)


def check_design(
    params: WheelParams,
    material: MaterialSpec,
    limits: PlatformLimits | None = None,
) -> list[Violation]:
    """Screen a design. Returns an empty list if it passes. **Never raises.**"""
    lim = limits or PlatformLimits()
    v: list[Violation] = []

    min_wall = (
        lim.min_wall_thickness_tpu_mm if material.is_elastomer else lim.min_wall_thickness_rigid_mm
    )

    # --- degenerate geometry ----------------------------------------------------------
    if params.spoke_span_mm <= 0.0:
        v.append(
            _violation(
                "spoke_span",
                Severity.DEGENERATE,
                params.spoke_span_mm,
                0.0,
                "hub reaches the shear band: no room for spokes "
                f"(rim inner r={params.rim_inner_radius_mm:.1f} mm, "
                f"hub r={params.hub_radius_mm:.1f} mm)",
                lower_bound=True,
            )
        )

    if params.hub_bore_radius_mm >= params.hub_radius_mm:
        v.append(
            _violation(
                "hub_bore",
                Severity.DEGENERATE,
                params.hub_bore_radius_mm,
                params.hub_radius_mm,
                "shaft bore is larger than the hub",
                lower_bound=False,
            )
        )

    if params.rim_thickness_mm >= params.outer_radius_mm:
        v.append(
            _violation(
                "rim_thickness",
                Severity.DEGENERATE,
                params.rim_thickness_mm,
                params.outer_radius_mm,
                "shear band thicker than the wheel radius",
                lower_bound=False,
            )
        )

    if params.n_spokes < 3:
        v.append(
            _violation(
                "n_spokes",
                Severity.DEGENERATE,
                float(params.n_spokes),
                3.0,
                "fewer than three spokes cannot support the rim",
                lower_bound=True,
            )
        )

    # A spoke bulging further than the span it bridges folds back on itself.
    if abs(params.spoke_sagitta_mm) > 0.5 * max(params.spoke_span_mm, 1e-9):
        v.append(
            _violation(
                "spoke_sagitta",
                Severity.DEGENERATE,
                abs(params.spoke_sagitta_mm),
                0.5 * params.spoke_span_mm,
                "curvature so high the spoke folds back on itself",
                lower_bound=False,
            )
        )

    # --- envelope ---------------------------------------------------------------------
    if params.outer_radius_mm > lim.wheel_well_radius_mm:
        v.append(
            _violation(
                "envelope_radius",
                Severity.INFEASIBLE,
                params.outer_radius_mm,
                lim.wheel_well_radius_mm,
                "wheel does not fit the wheel well",
                lower_bound=False,
            )
        )

    if params.width_mm > lim.max_width_mm:
        v.append(
            _violation(
                "envelope_width",
                Severity.INFEASIBLE,
                params.width_mm,
                lim.max_width_mm,
                "wheel exceeds the track-width allowance",
                lower_bound=False,
            )
        )

    if abs(params.hub_bore_radius_mm - lim.shaft_radius_mm) > 0.5:
        v.append(
            _violation(
                "shaft_fit",
                Severity.INFEASIBLE,
                params.hub_bore_radius_mm,
                lim.shaft_radius_mm,
                "bore does not match the drivetrain shaft (interface is fixed, not searched)",
                lower_bound=False,
            )
        )

    bx, by, bz = params.bounding_box_mm
    bed_x, bed_y, bed_z = lim.bed_size_mm
    if bx > bed_x or by > bed_y or bz > bed_z:
        v.append(
            _violation(
                "print_bed",
                Severity.INFEASIBLE,
                max(bx, by, bz),
                min(bed_x, bed_y, bed_z),
                f"bounding box {bx:.0f}x{by:.0f}x{bz:.0f} mm does not fit the bed",
                lower_bound=False,
            )
        )

    # --- manufacturability ------------------------------------------------------------
    # Read the TIP, not the root. They are the same number only while the taper is 1.0, so
    # a check written against `spoke_thickness_mm` goes on passing after a claw taper is
    # added and silently admits an unprintable tip: 7 mm at 0.15 taper is 1.05 mm of TPU.
    # This is the same shape of failure as every other one in the log — a value that looks
    # innocuous and means something else.
    if params.tip_thickness_mm < min_wall:
        v.append(
            _violation(
                "spoke_min_wall",
                Severity.INFEASIBLE,
                params.tip_thickness_mm,
                min_wall,
                f"spoke {'tip ' if params.claw_taper_ratio < 1.0 else ''}thinner than the "
                f"minimum printable wall for {material.name}",
                lower_bound=True,
            )
        )

    # Exempt when there is no band at all: zero is a topology switch, and there is no
    # unprintably-thin feature to reject. Anything strictly between 0 and min_wall still is.
    if params.has_shear_band and params.rim_thickness_mm < min_wall:
        v.append(
            _violation(
                "rim_min_wall",
                Severity.INFEASIBLE,
                params.rim_thickness_mm,
                min_wall,
                f"shear band thinner than the minimum printable wall for {material.name}",
                lower_bound=True,
            )
        )

    # Exact geometric clearance between adjacent spoke outlines. Not an approximation:
    # this samples the real outlines, so it is valid for curved and S-curve profiles where
    # the tightest point is not necessarily at the hub.
    #
    # Only meaningful once the geometry is non-degenerate. Running it on a design with
    # zero spokes or an inverted span would raise, which invariant 3 forbids — a
    # constraint check that throws kills the campaign rather than rejecting one design.
    geometry_is_sane = not any(x.severity is Severity.DEGENERATE for x in v)
    if geometry_is_sane:
        true_gap = min_gap_between_spokes(params)
        if true_gap < lim.min_interspoke_gap_mm:
            v.append(
                _violation(
                    "interspoke_gap",
                    Severity.INFEASIBLE,
                    true_gap,
                    lim.min_interspoke_gap_mm,
                    f"{params.n_spokes} spokes of {params.spoke_thickness_mm:.1f} mm leave "
                    f"only {true_gap:.2f} mm clearance; the nozzle cannot traverse without "
                    "dragging",
                    lower_bound=True,
                )
            )

    # --- the L claw's foot (family T7L) ------------------------------------------------
    if params.tip_hook_mm != 0.0 and params.has_shear_band:
        # Rejected rather than ignored. Silently dropping a field the caller set is this
        # project's recurring failure — a value that does nothing and reads as though it did.
        v.append(
            _violation(
                "hook_needs_bandless",
                Severity.INFEASIBLE,
                params.rim_thickness_mm,
                0.0,
                f"a {params.tip_hook_mm:+.1f} mm tip foot needs the tip to be free, and this "
                f"design buries it in a {params.rim_thickness_mm:.1f} mm shear band. Set "
                "--rim-thickness 0 for the bandless claw topology, or drop the foot",
                lower_bound=False,
            )
        )
    elif params.is_l_claw:
        # The foot must reach the ground without reaching the *next* claw. `interspoke_gap`
        # above measures the true clearance and is the binding check; this one exists because
        # its message is about a nozzle, and a foot long enough to lap its neighbour is a
        # different mistake that deserves to be named as one.
        span = params.contact_arc_rad + math.asin(
            min(params.hook_bend_radius_mm
                / max(params.outer_radius_mm - params.hook_bend_radius_mm, 1e-9), 1.0)
        )
        if span >= params.spoke_pitch_angle_rad:
            v.append(
                _violation(
                    "hook_reach",
                    Severity.INFEASIBLE,
                    math.degrees(span),
                    math.degrees(params.spoke_pitch_angle_rad),
                    f"the foot spans {math.degrees(span):.0f} deg of a "
                    f"{math.degrees(params.spoke_pitch_angle_rad):.0f} deg pitch, so it laps "
                    f"the next claw. Shorten it below "
                    f"{params.spoke_pitch_angle_rad * params.outer_radius_mm:.0f} mm or use "
                    "fewer claws",
                    lower_bound=False,
                )
            )
        if abs(params.tip_hook_mm) < params.tip_thickness_mm:
            v.append(
                _violation(
                    "hook_too_short",
                    Severity.WARNING,
                    abs(params.tip_hook_mm),
                    params.tip_thickness_mm,
                    f"a {abs(params.tip_hook_mm):.1f} mm foot on a "
                    f"{params.tip_thickness_mm:.1f} mm tip is a rounded corner rather than an "
                    "L: the bend radius is capped at half the foot, so most of what was asked "
                    "for is fillet. polygon_drop_mm still credits it for the arc",
                    lower_bound=True,
                )
            )

    # Written as two branches rather than one comparison because with no shear band the
    # single comparison reads 0 >= 0 and rejects an untreaded design for cutting through a
    # band it does not have.
    if not params.has_shear_band:
        if params.tread_depth_mm > 0.0:
            v.append(
                _violation(
                    "tread_depth",
                    Severity.INFEASIBLE,
                    params.tread_depth_mm,
                    0.0,
                    "there is no shear band to cut a tread into",
                    lower_bound=False,
                )
            )
    elif params.tread_depth_mm >= params.rim_thickness_mm:
        v.append(
            _violation(
                "tread_depth",
                Severity.INFEASIBLE,
                params.tread_depth_mm,
                params.rim_thickness_mm,
                "tread cuts through the shear band",
                lower_bound=False,
            )
        )

    # --- search bounds ----------------------------------------------------------------
    for field_name, (lo, hi) in PARAM_BOUNDS.items():
        value = float(getattr(params, field_name))
        # The one documented exemption: rim_thickness_mm == 0 selects the bandless
        # topology and is deliberately outside the continuous searched range.
        if field_name == "rim_thickness_mm" and not params.has_shear_band:
            continue
        if not lo <= value <= hi:
            v.append(
                _violation(
                    f"bounds_{field_name}",
                    Severity.INFEASIBLE,
                    value,
                    hi if value > hi else lo,
                    f"{field_name}={value:g} outside searched range [{lo:g}, {hi:g}]",
                    lower_bound=value < lo,
                )
            )

    # --- warnings ---------------------------------------------------------------------
    # Straight spokes are Euler columns: buckling direction is set by manufacturing noise,
    # which is exactly the kind of unmodelled sensitivity that fails to transfer.
    if params.spoke_profile.value == "straight":
        v.append(
            Violation(
                "straight_spoke_buckling",
                Severity.WARNING,
                0.0,
                0.0,
                0.0,
                "straight spokes buckle in an unpredictable direction; expect poor "
                "sim-to-real agreement (see docs/plan/10-reality-gap.md)",
            )
        )

    # A bandless wheel is buildable and printable — it is the simplest way to isolate spoke
    # compliance, since there is no second spring in series with it. But it changes the
    # *kind* of contact, not just its stiffness, and both consequences are silent:
    #   - contact is discrete, so the response depends on `spoke_phase_deg`, and rolling
    #     produces `n_spokes` stiffness cycles per revolution rather than a ripple;
    #   - the ring ROM's segment-to-segment coupling is the shear band's bending stiffness,
    #     which here is zero (docs/plan/06-compliance-rom.md section 3).
    if not params.has_shear_band:
        v.append(
            Violation(
                "no_shear_band",
                Severity.WARNING,
                0.0,
                0.0,
                0.0,
                # The patch is as wide as the material that touches the ground, and what that
                # is has now changed twice: the root, then the tip once claws tapered, then
                # the foot once claws became L-shaped. `contact_patch_mm` owns the question so
                # there is no fourth time.
                f"no shear band: the {params.n_spokes} spoke "
                f"{'feet' if params.is_l_claw else 'tips'} are the running surface, so "
                f"contact is discrete over {params.contact_patch_mm:.1f} mm patches and "
                "depends on spoke_phase_deg; the ring ROM has no shear-band stiffness to fit",
            )
        )
        # How rough that discrete contact is, as a number. TODO #19 asked for the claw-count
        # lower bound to be re-derived from the load case rather than inherited from a banded
        # wheel's range, and the answer came out the *opposite* way to the one anticipated:
        # a passive claw wheel wants **more** claws, not fewer.
        #
        # Measured 2026-08-09 on R 85 mm at the platform's 24.5 N, two claws an order of
        # magnitude apart in stiffness (3.7 and 13.5 N/mm): the axle's peak-to-peak movement
        # over one pitch reaches the wheel's own static deflection — i.e. the trailing claw
        # leaves the ground entirely once per pitch — at **n = 10 to 12** on both, and needs
        # n >= 12 (stiff claw) or n >= 24 (soft claw) to fall to half of it. The PaTS-Wheel
        # letter's four-claw row is not a counter-example: that row is gear-driven and the
        # wheel transforms, so its claws are not carrying the load as passive springs.
        #
        # n >= 12 on any radius is polygon_drop <= 3.4% of R, which is the threshold below.
        # A WARNING and not infeasible, on purpose: the real criterion needs the fitted law
        # (`wheelopt.rom.ring.ride_height_ripple_m`) and a pre-filter does not have one, so
        # this flags the geometry that cannot be rescued by any stiffness rather than
        # pretending to judge the ride.
        drop_fraction = params.polygon_drop_mm / max(params.outer_radius_mm, 1e-9)
        if drop_fraction > 0.035:
            v.append(
                _violation(
                    "claw_ride_harshness",
                    Severity.WARNING,
                    drop_fraction,
                    0.035,
                    f"{params.n_spokes} tips on R {params.outer_radius_mm:.0f} mm drop the "
                    f"axle {params.polygon_drop_mm:.1f} mm ({drop_fraction:.1%} of R) between "
                    "tips; measured, a wheel this coarse unloads a claw completely once a "
                    "pitch. Confirm with ride_height_ripple_m before believing any ride "
                    "metric from it",
                    lower_bound=False,
                )
            )

    # Slenderness. Very slender spokes are dominated by buckling rather than bending, and
    # sit outside the range the ring ROM is fitted over.
    #
    # Reads `effective_thickness_mm`, not the root, since TODO #21 closed on 2026-08-09. The
    # root is the *stiffest* section of a tapered claw, so it understated slenderness and
    # erred toward accepting a claw that buckles -- non-conservatively. The effective
    # thickness is the uniform spoke of equal tip compliance, derived in closed form on
    # `WheelParams`, and it is 13% below the root at taper 0.6 and 36% below at 0.25.
    #
    # **What this check can and cannot see** (2026-08-10 opened #28, 2026-08-11 closed it).
    # It fires in the slender-strut corner it was written for -- a 1.6 mm spoke on an R 100
    # banded wheel reads 48 -- and it is BLIND to the claw family, measured rather than
    # argued: on the R 60, 12-claw design at mu = 0.6 the stick branch's limit point falls
    # 105 -> 39 -> 23 N as the taper goes 1.0 -> 0.6 -> 0.4, a 4.6x collapse, while this
    # proxy creeps 6.3 -> 7.2 -> 7.8. No threshold on an axis that flat can rank that
    # family, so DO NOT tune this constant to catch claws; the check that catches them is
    # `fea_buckling`, which measures each design's own limit point against 2.5x nominal and
    # fails all three tapered claws above while this warning stays silent. This one is kept
    # for the corner where geometry alone is enough to know the ROM fit will be poor.
    slenderness = params.spoke_span_mm / max(params.effective_thickness_mm, 1e-9)
    if slenderness > 40.0:
        v.append(
            _violation(
                "spoke_slenderness",
                Severity.WARNING,
                slenderness,
                40.0,
                f"spoke slenderness {slenderness:.0f} (on an effective thickness of "
                f"{params.effective_thickness_mm:.2f} mm against a {params.spoke_thickness_mm:.2f} "
                "mm root) is buckling-dominated; ROM fit may be poor",
                lower_bound=False,
            )
        )

    # Thin features print solid regardless of the infill setting, so an optimiser that
    # believes it is reducing mass via infill is fooling itself.
    if material.shell_fraction(params.spoke_thickness_mm) >= 1.0 and material.infill_density < 1.0:
        v.append(
            Violation(
                "infill_ineffective",
                Severity.WARNING,
                material.infill_density,
                1.0,
                0.0,
                f"spoke at {params.spoke_thickness_mm:.1f} mm prints solid with "
                f"{material.wall_count} walls; infill_density has no effect",
            )
        )

    return v


def is_feasible(violations: list[Violation]) -> bool:
    """True if nothing worse than a warning was found."""
    return not any(x.severity is not Severity.WARNING for x in violations)
