"""Material specifications and effective-density homogenisation.

FDM parts are not solid: a printed wall shell encloses a sparse infill lattice. Meshing
that lattice is intractable at campaign scale, so it is homogenised — the part is treated
as a solid of *effective* density and effective stiffness. See
``docs/plan/04-design-space.md``.

This module owns density only. Effective **stiffness** homogenisation is calibrated from
coupon tests and lives with the FEA stage; it is deliberately not guessed here.

Invariant 2 (CLAUDE.md): mass is always derived from geometry and material. Nothing in the
pipeline may hard-code a wheel mass.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

__all__ = ["InfillPattern", "MaterialSpec", "BASE_DENSITIES_KG_M3", "TPU95A", "PLA", "PETG"]


class InfillPattern(str, Enum):
    GYROID = "gyroid"
    GRID = "grid"
    HONEYCOMB = "honeycomb"
    CONCENTRIC = "concentric"


#: Bulk polymer densities, kg/m^3. Filament datasheet values; the *printed* density is
#: lower because of inter-bead voids, which the ``packing_efficiency`` term absorbs.
BASE_DENSITIES_KG_M3: dict[str, float] = {
    "TPU_85A": 1200.0,
    "TPU_95A": 1210.0,
    "TPU_98A": 1220.0,
    "TPU_60D": 1230.0,
    "PLA": 1240.0,
    "PETG": 1270.0,
}


@dataclass(frozen=True, slots=True)
class MaterialSpec:
    """A material plus the print parameters that determine its effective properties."""

    name: str
    infill_density: float = 1.0  # 0-1 fraction
    infill_pattern: InfillPattern = InfillPattern.GYROID
    wall_count: int = 3
    nozzle_diameter_mm: float = 0.4
    layer_height_mm: float = 0.2
    #: Fraction of the nominal extrusion volume actually filled with polymer. Accounts for
    #: inter-bead voids. ~0.95 is typical for well-tuned FDM; measure it by weighing a
    #: printed coupon of known volume (see docs/plan/07-materials.md).
    packing_efficiency: float = 0.95

    @property
    def base_density_kg_m3(self) -> float:
        try:
            return BASE_DENSITIES_KG_M3[self.name]
        except KeyError as exc:  # pragma: no cover - configuration error
            raise KeyError(
                f"unknown material {self.name!r}; known: {sorted(BASE_DENSITIES_KG_M3)}"
            ) from exc

    @property
    def wall_thickness_mm(self) -> float:
        return self.wall_count * self.nozzle_diameter_mm

    @property
    def is_elastomer(self) -> bool:
        return self.name.startswith("TPU")

    def shell_fraction(self, feature_thickness_mm: float) -> float:
        """Volume fraction of a feature occupied by solid perimeter walls.

        A feature thinner than twice the wall thickness is printed solid — this is why
        thin spokes are far stiffer and heavier than their infill setting suggests, and
        why ``infill_density`` is nearly meaningless for a 1.6 mm spoke.

        Modelled as a slab of thickness ``t`` with walls of thickness ``w`` on both faces.
        """
        if feature_thickness_mm <= 0.0:
            return 1.0
        solid_span = 2.0 * self.wall_thickness_mm
        if feature_thickness_mm <= solid_span:
            return 1.0
        return solid_span / feature_thickness_mm

    def effective_density_kg_m3(self, feature_thickness_mm: float) -> float:
        """Homogenised density of a feature of the given thickness.

        ``rho_eff = rho_base * packing * (shell + (1 - shell) * infill)``
        """
        shell = self.shell_fraction(feature_thickness_mm)
        fill = shell + (1.0 - shell) * self.infill_density
        return self.base_density_kg_m3 * self.packing_efficiency * fill

    def __post_init__(self) -> None:
        # Validation belongs here, not at use sites. A bad material spec is a
        # configuration error and should fail loudly and immediately — unlike an
        # infeasible *design*, which must return a typed violation (invariant 3).
        if not 0.0 <= self.infill_density <= 1.0:
            raise ValueError(f"infill_density must be in [0, 1], got {self.infill_density}")
        if self.wall_count < 1:
            raise ValueError(f"wall_count must be >= 1, got {self.wall_count}")
        if not 0.0 < self.packing_efficiency <= 1.0:
            raise ValueError(
                f"packing_efficiency must be in (0, 1], got {self.packing_efficiency}"
            )
        if self.name not in BASE_DENSITIES_KG_M3:
            raise KeyError(
                f"unknown material {self.name!r}; known: {sorted(BASE_DENSITIES_KG_M3)}"
            )


TPU95A = MaterialSpec(name="TPU_95A", infill_density=0.40, wall_count=3)
PLA = MaterialSpec(name="PLA", infill_density=0.25, wall_count=3)
PETG = MaterialSpec(name="PETG", infill_density=0.30, wall_count=3)
