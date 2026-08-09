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
from .params import SpokeProfile, WheelParams

__all__ = [
    "add_geometry_args",
    "add_material_args",
    "params_from_args",
    "material_from_args",
]


def add_geometry_args(parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
    """Add the wheel geometry flags. Defaults come from ``WheelParams``."""
    d = WheelParams()
    g = parser.add_argument_group("geometry (mm)")
    g.add_argument("--radius", type=float, default=d.outer_radius_mm)
    g.add_argument("--width", type=float, default=d.width_mm)
    g.add_argument("--rim-thickness", type=float, default=d.rim_thickness_mm,
                   help="shear band thickness; 0 = no shear band, spoke tips run on the "
                        "ground")
    g.add_argument("--hub-radius", type=float, default=d.hub_radius_mm)
    g.add_argument("--bore-radius", type=float, default=d.hub_bore_radius_mm)
    g.add_argument("--spokes", type=int, default=d.n_spokes)
    g.add_argument("--thickness", type=float, default=d.spoke_thickness_mm)
    g.add_argument("--curvature", type=float, default=d.spoke_curvature_1_per_mm,
                   help="signed spoke curvature, 1/mm")
    g.add_argument("--profile", choices=[p.value for p in SpokeProfile],
                   default=d.spoke_profile.value)
    g.add_argument("--claw-taper", type=float, default=d.claw_taper_ratio,
                   help="tip thickness as a fraction of the root. 1.0 is a uniform strut; "
                        "below 1.0 makes it a T7 claw. The minimum-wall check reads the "
                        "TIP, so a thick root with an aggressive taper is still rejected")
    g.add_argument("--tread-depth", type=float, default=d.tread_depth_mm)
    g.add_argument("--spoke-phase", type=float, default=d.spoke_phase_deg,
                   help="rotational phase of the spoke pattern, degrees. Only matters "
                        "without a shear band; -90 puts a tip at the contact point")
    return g


def add_material_args(parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
    """Add the material flags. Defaults match the ``TPU95A`` preset."""
    g = parser.add_argument_group("material")
    g.add_argument("--material", choices=sorted(BASE_DENSITIES_KG_M3), default="TPU_95A")
    g.add_argument("--infill", type=float, default=0.4, help="0-1")
    g.add_argument("--pattern", choices=[p.value for p in InfillPattern],
                   default=InfillPattern.GYROID.value)
    g.add_argument("--walls", type=int, default=3)
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
