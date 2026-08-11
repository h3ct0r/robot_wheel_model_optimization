"""Shared argparse wiring for the command-line entry points.

``gen_wheel.py`` and ``run_fea.py`` both need to turn flags into a
:class:`~wheelopt.cad.params.WheelParams` and a
:class:`~wheelopt.cad.materials.MaterialSpec`. Duplicating that would let the two drift
into producing *different* ``design_hash()`` values for the same flags, which would silently
split the cache — two directories of results for what the user believes is one design.
"""

from __future__ import annotations

import argparse

from .materials import BASE_DENSITIES_KG_M3, InfillPattern, MaterialSpec
from .params import PARAM_BOUNDS, SpokeProfile, WheelParams

__all__ = [
    "add_geometry_args",
    "add_material_args",
    "params_from_args",
    "material_from_args",
]


def _range(field: str, unit: str = "", fmt: str = "g") -> str:
    """``"(60-100 mm)"`` for a searched field, read from ``PARAM_BOUNDS`` rather than typed.

    The help text for a bound and the screening that enforces it are the same number in two
    places, and a help string that has drifted from the check is worse than none — it tells
    you a value is legal right up until the run is rejected for it. Reading the bound here
    means the two cannot disagree.
    """
    low, high = PARAM_BOUNDS[field]
    tail = f" {unit}" if unit else ""
    return f"({low:{fmt}} to {high:{fmt}}{tail})"


def add_geometry_args(parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
    """Add the wheel geometry flags. Defaults come from ``WheelParams``.

    Every flag carries help, and the searched ones quote their own screening bound through
    :func:`_range`. These describe a **design**, so a caller that never builds geometry —
    ``run_rover.py`` without ``--compliant`` or ``--stl``, which runs rigid cylinders — reads
    only radius and width and ignores the rest.
    """
    d = WheelParams()
    g = parser.add_argument_group("geometry (mm)")
    g.add_argument("--radius", type=float, default=d.outer_radius_mm,
                   help=f"outer radius {_range('outer_radius_mm', 'mm')}. Also capped by the "
                        "chassis wheel well and the print bed, both from configs/robot.yaml")
    g.add_argument("--width", type=float, default=d.width_mm,
                   help=f"tread width {_range('width_mm', 'mm')}. Plane-strain FEA force "
                        "scales linearly with this")
    g.add_argument("--rim-thickness", type=float, default=d.rim_thickness_mm,
                   help=f"shear band thickness {_range('rim_thickness_mm', 'mm')}, or exactly "
                        "0 for no band, which is a topology switch rather than the bottom of "
                        "the range: the spoke tips become the running surface (T7)")
    g.add_argument("--hub-radius", type=float, default=d.hub_radius_mm,
                   help="hub outer radius, mm. Sets the claw root, so claw length is "
                        "radius minus this")
    g.add_argument("--bore-radius", type=float, default=d.hub_bore_radius_mm,
                   help="shaft bore radius, mm. Fixed by the drivetrain, not searched")
    g.add_argument("--spokes", type=int, default=d.n_spokes,
                   help=f"number of spokes {_range('n_spokes')}. Bandless, this is also the "
                        "ring ROM's segment count")
    g.add_argument("--thickness", type=float, default=d.spoke_thickness_mm,
                   help=f"spoke thickness at the ROOT {_range('spoke_thickness_mm', 'mm')}")
    g.add_argument("--curvature", type=float, default=d.spoke_curvature_1_per_mm,
                   help=f"signed spoke curvature, 1/mm "
                        f"{_range('spoke_curvature_1_per_mm')}. Sign sets which way the "
                        "spoke bows, and so which direction of drive torque stiffens it")
    g.add_argument("--profile", choices=[p.value for p in SpokeProfile],
                   default=d.spoke_profile.value,
                   help="spoke centreline family")
    g.add_argument("--claw-taper", type=float, default=d.claw_taper_ratio,
                   help=f"tip thickness as a fraction of the root "
                        f"{_range('claw_taper_ratio')}. 1.0 is a uniform strut; below 1.0 "
                        "makes it a T7 claw. The minimum-wall check reads the TIP, so a "
                        "thick root with an aggressive taper is still rejected")
    g.add_argument("--tip-hook", type=float, default=d.tip_hook_mm,
                   help=f"length of a tangential FOOT at the claw tip, mm "
                        f"{_range('tip_hook_mm', 'mm')}, turning the claw into a literal L. "
                        "0 is the plain radial claw. SIGNED like --curvature: the sign is "
                        "which way the foot points. Needs --rim-thickness 0, and is rejected "
                        "rather than ignored with a band")
    g.add_argument("--tread-depth", type=float, default=d.tread_depth_mm,
                   help=f"depth of the tread grooves {_range('tread_depth_mm', 'mm')}; "
                        "0 is a smooth tread")
    g.add_argument("--spoke-phase", type=float, default=d.spoke_phase_deg,
                   help="rotational phase of the spoke pattern, degrees. Only matters "
                        "without a shear band; -90 puts a tip at the contact point")
    return g


def add_material_args(parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
    """Add the material flags. Defaults match the ``TPU95A`` preset.

    These set density *and* stiffness, by different laws — mass scales linearly with infill
    and stiffness by a Gibson-Ashby power law — so they are not a cosmetic choice.
    """
    g = parser.add_argument_group("material")
    g.add_argument("--material", choices=sorted(BASE_DENSITIES_KG_M3), default="TPU_95A",
                   help="filament preset: base density and hyperelastic coefficients")
    g.add_argument("--infill", type=float, default=0.4,
                   help="infill fraction, 0 to 1. Knocks down mass linearly and stiffness by "
                        "a Gibson-Ashby power law -- deliberately not the same curve")
    g.add_argument("--pattern", choices=[p.value for p in InfillPattern],
                   default=InfillPattern.GYROID.value,
                   help="infill pattern; sets the packing efficiency behind the knock-down")
    g.add_argument("--walls", type=int, default=3,
                   help="perimeter count. A feature thinner than 2x this prints solid and "
                        "--infill then does nothing, which is reported as a warning")
    return g


def params_from_args(args: argparse.Namespace) -> WheelParams:
    return WheelParams(
        outer_radius_mm=args.radius,
        width_mm=args.width,
        rim_thickness_mm=args.rim_thickness,
        hub_radius_mm=args.hub_radius,
        hub_bore_radius_mm=args.bore_radius,
        n_spokes=args.spokes,
        spoke_thickness_mm=args.thickness,
        spoke_curvature_1_per_mm=args.curvature,
        claw_taper_ratio=args.claw_taper,
        tip_hook_mm=args.tip_hook,
        spoke_profile=SpokeProfile(args.profile),
        spoke_phase_deg=args.spoke_phase,
        tread_depth_mm=args.tread_depth,
    )


def material_from_args(args: argparse.Namespace) -> MaterialSpec:
    return MaterialSpec(
        name=args.material,
        infill_density=args.infill,
        infill_pattern=InfillPattern(args.pattern),
        wall_count=args.walls,
    )
