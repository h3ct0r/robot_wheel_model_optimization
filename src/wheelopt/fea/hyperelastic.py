"""Hyperelastic material models and the printed-TPU coefficient table.

``wheelopt.cad.materials`` deliberately owns density only, and says so: effective
*stiffness* homogenisation "is calibrated from coupon tests and lives with the FEA stage".
This is that stage.

**On "third-order Mooney-Rivlin".** docs/plan/07-materials.md cites a third-order
Mooney-Rivlin fit as the starting point. CalculiX's ``*HYPERELASTIC, MOONEY-RIVLIN`` card
is **not** that: it takes three constants (C10, C01, D1) and is identical to
``POLYNOMIAL, N=1``. The third-order model is ``POLYNOMIAL, N=3`` — nine C-coefficients
plus three D-coefficients. Both readings of the phrase appear in the literature, so
:class:`HyperelasticModel` carries all nine C-terms with zeros permitted and records which
convention its source used.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

from ..cad.materials import InfillPattern, MaterialSpec

__all__ = [
    "DEFAULT_POISSON_EFFECTIVE",
    "PATTERN_EXPONENT",
    "SOLID_TPU_FITS",
    "HyperelasticModel",
    "UnknownMaterial",
    "for_material",
    "stiffness_knockdown",
]


class UnknownMaterial(KeyError):
    """No hyperelastic fit exists for this material.

    Raised loudly and on purpose, mirroring the policy in ``cad.materials``: silently
    substituting a default modulus would produce a plausible wrong stiffness for every
    design in a campaign, which is the failure mode this project is most exposed to.
    Callers in the pipeline catch this and return ``FeaStatus.DECK_INVALID``.
    """


#: Effective Poisson ratio used to set the compressibility terms D_i.
#:
#: **This is load-bearing, not a detail.** Bulk TPU is nearly incompressible (nu -> 0.5),
#: but CalculiX has no hybrid elements: a fully-integrated C3D10 with nu = 0.4999 locks
#: volumetrically and reports a radial stiffness that is too high and entirely plausible.
#: A printed part with infill voids genuinely *is* more compressible than bulk TPU, so a
#: reduced value is defensible on physical grounds as well as numerical ones. Quantified
#: directly by the locking check in scripts/verify_fea.py — do not change this without
#: re-running it.
DEFAULT_POISSON_EFFECTIVE = 0.46

#: Ratio by which each successive volumetric coefficient D_k exceeds D_1, making the
#: higher-order volumetric terms contribute nothing. See :meth:`HyperelasticModel.with_poisson`
#: — the natural-looking choice of zero means the opposite of what it reads as.
HIGHER_ORDER_D_RATIO = 1.0e6


@dataclass(frozen=True, slots=True)
class HyperelasticModel:
    """A polynomial hyperelastic model in CalculiX's parameter ordering.

    Strain energy, with I1b/I2b the deviatoric invariants and J the volume ratio:

        W = sum_{i+j=1..N} C_ij (I1b-3)^i (I2b-3)^j + sum_{k=1..N} (1/D_k) (J-1)^(2k)

    All C coefficients have units of stress (Pa); all D coefficients are 1/Pa.
    """

    #: (C10, C01, C20, C11, C02, C30, C21, C12, C03) in Pa. Zeros are permitted.
    c: tuple[float, ...]
    #: (D1, D2, D3) in 1/Pa.
    d: tuple[float, ...]
    #: N in ``*HYPERELASTIC, POLYNOMIAL, N=<order>``.
    order: int
    #: Citation, and which reading of "third-order" the source used.
    source: str

    def __post_init__(self) -> None:
        if self.order not in (1, 2, 3):
            raise ValueError("CalculiX POLYNOMIAL supports N = 1, 2 or 3")
        if len(self.c) != 9:
            raise ValueError("c must hold all 9 coefficients, padded with zeros")
        if len(self.d) != 3:
            raise ValueError("d must hold 3 coefficients, padded with zeros")
        if self.initial_shear_modulus_pa <= 0:
            raise ValueError("C10 + C01 must be positive for a physical material")

    @property
    def initial_shear_modulus_pa(self) -> float:
        """mu_0 = 2 (C10 + C01). The small-strain shear modulus."""
        return 2.0 * (self.c[0] + self.c[1])

    @property
    def initial_bulk_modulus_pa(self) -> float:
        """K_0 = 2 / D1. Infinite (returned as ``inf``) for a fully incompressible fit."""
        return float("inf") if self.d[0] == 0.0 else 2.0 / self.d[0]

    @property
    def initial_youngs_pa(self) -> float:
        """E = 9 K mu / (3 K + mu), reducing to 3 mu when incompressible."""
        mu = self.initial_shear_modulus_pa
        k = self.initial_bulk_modulus_pa
        if k == float("inf"):
            return 3.0 * mu
        return 9.0 * k * mu / (3.0 * k + mu)

    @property
    def poisson_effective(self) -> float:
        k, mu = self.initial_bulk_modulus_pa, self.initial_shear_modulus_pa
        if k == float("inf"):
            return 0.5
        return (3.0 * k - 2.0 * mu) / (2.0 * (3.0 * k + mu))

    def scaled(self, factor: float) -> HyperelasticModel:
        """Scale stiffness by ``factor``, preserving the shape of the stress-strain curve.

        Every C coefficient has units of stress, so multiplying them all by one number
        rescales the whole curve without distorting it, and the D coefficients scale
        inversely to hold the Poisson ratio fixed. This is a first-order homogenisation:
        honest, but not a substitute for the RVE study in docs/plan/04-design-space.md.
        """
        if factor <= 0:
            raise ValueError("stiffness scale factor must be positive")
        return HyperelasticModel(
            c=tuple(x * factor for x in self.c),
            d=tuple(x / factor for x in self.d),
            order=self.order,
            source=f"{self.source} x{factor:.4f}",
        )

    def with_poisson(self, nu: float) -> HyperelasticModel:
        """Set the compressibility terms from an effective Poisson ratio.

        D1 = 2/K with K = 2 mu (1 + nu) / (3 (1 - 2 nu)).

        **Higher-order D terms are set large, not zero.** The volumetric energy is
        ``sum_k (1/D_k) (J-1)^(2k)``, so the coefficient is the *reciprocal*: D2 = 0 does
        not mean "no second-order term", it means an infinitely stiff one. CalculiX does
        not reject it either — it warns that a default was substituted and carries on with
        D2 ~ 1e-15, i.e. 1/D2 ~ 1e15 Pa, which at only 1% volume change contributes stress
        comparable to the shear modulus. The wheel then comes out volumetrically locked and
        far too stiff, for reasons invisible anywhere in the output.

        Setting D_k = D1 * :data:`HIGHER_ORDER_D_RATIO` makes those terms negligible, which
        is what "the literature fit does not constrain them" actually implies.
        """
        if not -1.0 < nu < 0.5:
            raise ValueError("effective Poisson ratio must lie in (-1, 0.5)")
        mu = self.initial_shear_modulus_pa
        k = 2.0 * mu * (1.0 + nu) / (3.0 * (1.0 - 2.0 * nu))
        d1 = 2.0 / k
        return HyperelasticModel(
            c=self.c,
            d=(d1, d1 * HIGHER_ORDER_D_RATIO, d1 * HIGHER_ORDER_D_RATIO**2),
            order=self.order,
            source=self.source,
        )

    def n_active_c(self) -> int:
        """How many C coefficients CalculiX expects for this order: 2, 5 or 9."""
        return {1: 2, 2: 5, 3: 9}[self.order]

    def calculix_card(self, name: str) -> str:
        """Emit ``*MATERIAL`` + ``*HYPERELASTIC``, 8 constants per line with continuation.

        CalculiX reads the polynomial constants in the order
        ``C10 C01 C20 C11 C02 C30 C21 C12 C03 D1 D2 D3``, truncated to the count implied
        by N — 2+1 for N=1, 5+2 for N=2, 9+3 for N=3.
        """
        n_c = self.n_active_c()
        values = list(self.c[:n_c]) + list(self.d[: self.order])
        lines = [f"*MATERIAL, NAME={name}", f"*HYPERELASTIC, POLYNOMIAL, N={self.order}"]
        for start in range(0, len(values), 8):
            chunk = values[start : start + 8]
            lines.append(" " + ", ".join(f"{v:.8e}" for v in chunk))
        return "\n".join(lines)

    def coefficient_digest(self) -> str:
        """Hash of the actual coefficients, for the cache key.

        Hashing the *numbers* rather than the table name is what makes invariant 5 bite:
        re-seeding the literature table with better fits invalidates cached results even if
        nobody remembers to bump the pipeline version.
        """
        payload = json.dumps(
            {"c": list(self.c), "d": list(self.d), "order": self.order},
            sort_keys=True,
            separators=(",", ":"),
        )
        return hashlib.sha256(payload.encode()).hexdigest()[:16]


def _model(c10: float, c01: float, c20: float, c11: float, c02: float, source: str):
    """Second-order fit helper — the shape most published TPU fits actually take."""
    return HyperelasticModel(
        c=(c10, c01, c20, c11, c02, 0.0, 0.0, 0.0, 0.0),
        d=(0.0, 0.0, 0.0),
        order=2,
        source=source,
    )


#: Hyperelastic fits for **solid** printed TPU, Pa, before any infill knock-down.
#:
#: PROVISIONAL. These are order-of-magnitude seeds consistent with the shore-hardness trend
#: and the model-choice guidance in docs/plan/07-materials.md (neo-Hookean captures only the
#: initial linear zone; Mooney-Rivlin and Ogden track the full curve). They are **not**
#: transcribed from a specific table, and they must be replaced by the coupon fits described
#: in docs/plan/07-materials.md before any quantitative claim is made. Because the cache key
#: hashes the coefficients, replacing them invalidates prior results automatically.
#:
#: Ordering sanity: shore 85A softer than 95A softer than 98A softer than 60D.
SOLID_TPU_FITS: dict[str, HyperelasticModel] = {
    "TPU_85A": _model(
        0.55e6, 0.14e6, 0.09e6, -0.03e6, 0.01e6, "provisional seed, shore 85A"
    ),
    "TPU_95A": _model(
        1.30e6, 0.32e6, 0.21e6, -0.07e6, 0.02e6, "provisional seed, shore 95A"
    ),
    "TPU_98A": _model(
        2.10e6, 0.52e6, 0.34e6, -0.11e6, 0.03e6, "provisional seed, shore 98A"
    ),
    "TPU_60D": _model(
        3.60e6, 0.90e6, 0.58e6, -0.19e6, 0.05e6, "provisional seed, shore 60D"
    ),
}

#: Gibson-Ashby scaling exponent per infill pattern: E_eff/E_solid ~ (rho_eff/rho_solid)^n.
#:
#: n = 2 is the classic bending-dominated open-cell value. Patterns whose walls carry load
#: axially rather than in bending sit lower — concentric infill is nearly aligned with hoop
#: load, hence its much weaker penalty.
PATTERN_EXPONENT: dict[InfillPattern, float] = {
    InfillPattern.GRID: 2.0,
    InfillPattern.HONEYCOMB: 1.8,
    InfillPattern.GYROID: 1.7,
    InfillPattern.CONCENTRIC: 1.2,
}


def stiffness_knockdown(material: MaterialSpec, feature_thickness_mm: float) -> float:
    """Fraction of solid-material stiffness retained by the printed feature.

    Voigt mixture of a solid perimeter shell and a Gibson-Ashby infill core, reusing
    :meth:`~wheelopt.cad.materials.MaterialSpec.shell_fraction` so that stiffness and mass
    are driven by the *same* geometry-derived quantity. That matters for invariant 2 and it
    keeps the "thin spokes print solid, so infill density is meaningless" warning in
    ``cad.constraints`` consistent between the two.

    **The asymmetry with density is deliberate.** ``effective_density_kg_m3`` mixes linearly
    in volume fraction, ``shell + (1-shell) * phi``; stiffness mixes as
    ``shell + (1-shell) * phi**n``. Halving the infill roughly halves the mass but cuts the
    stiffness by about four. Anyone reading both functions will assume one is a bug unless
    it says so here.
    """
    shell = material.shell_fraction(feature_thickness_mm)
    n = PATTERN_EXPONENT[material.infill_pattern]
    core = material.infill_density**n
    return float(material.packing_efficiency * (shell + (1.0 - shell) * core))


def for_material(
    material: MaterialSpec,
    feature_thickness_mm: float,
    *,
    poisson_effective: float = DEFAULT_POISSON_EFFECTIVE,
) -> HyperelasticModel:
    """The hyperelastic model for a printed feature of this material and thickness.

    Raises:
        UnknownMaterial: if no fit exists. Never silently defaults.
    """
    try:
        solid = SOLID_TPU_FITS[material.name]
    except KeyError as exc:
        raise UnknownMaterial(
            f"no hyperelastic fit for {material.name!r}; known: "
            f"{sorted(SOLID_TPU_FITS)}. Rigid materials are not modelled as hyperelastic — "
            "add an elastic card instead."
        ) from exc

    knockdown = stiffness_knockdown(material, feature_thickness_mm)
    return solid.scaled(knockdown).with_poisson(poisson_effective)
