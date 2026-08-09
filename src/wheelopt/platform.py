"""Loader for ``configs/robot.yaml`` — the platform spec everything depends on.

Why this module exists
----------------------
``configs/robot.yaml`` was written to be the single source of truth for the robot, and its
own comments say so. Until now nothing read it, so the *real* source of truth was
:class:`~wheelopt.cad.constraints.PlatformLimits` — a dataclass whose defaults happened to
agree with the YAML because both were edited by hand in the same sitting. That is precisely
the failure mode ``docs/experiments/log.md`` keeps recording: a plausible value that means
something else, with nothing to catch the divergence.

What this module does *not* do
------------------------------
It does not make screening read the filesystem. ``cad/constraints.py`` and ``cad/params.py``
stay pure and importable without I/O — invariant 3 says an infeasible design costs
milliseconds, and a YAML parse per candidate is not that. The dataclass defaults remain the
fast path. This module is how a caller *chooses* a platform explicitly, and
``tests/test_platform.py`` is what stops the two from drifting apart silently.

Units
-----
The YAML is SI (metres) throughout — see its header. :class:`PlatformLimits` is in
millimetres, because it screens CAD parameters. The conversion happens here, at the
boundary, per the CLAUDE.md units policy, and the test asserts on the converted values: a
``wheel_well_radius`` of ``0.105`` passed through unconverted would read as a 0.1 mm wheel
well and reject every design, which is loud — but the same mistake on a tolerance would not
be.
"""

from __future__ import annotations

import datetime as _dt
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .cad.constraints import PlatformLimits

__all__ = [
    "PlatformSpec",
    "PlatformSpecError",
    "default_config_path",
    "load_platform",
]

#: Standard gravity. Used only for cross-checking the declared wheel load against the
#: declared mass — never to *derive* a load, so that the YAML stays the stated authority.
_G = 9.81


class PlatformSpecError(ValueError):
    """The platform spec is missing, malformed, or internally inconsistent.

    Deliberately raises rather than returning a typed failure. Invariant 4 governs
    *evaluations* — a diverged sim must not kill a campaign. A platform spec that cannot be
    read is a startup error: every result downstream would be scaled against a value nobody
    supplied. Fail before the campaign, not during it.
    """


def default_config_path() -> Path:
    """``configs/robot.yaml`` relative to the repository root.

    Resolved from this file rather than the working directory so that a worker started
    anywhere finds the same spec.
    """
    return Path(__file__).resolve().parents[2] / "configs" / "robot.yaml"


def _require(mapping: dict[str, Any], path: str) -> Any:
    """Fetch ``a.b.c`` from nested dicts, or raise naming the full path."""
    node: Any = mapping
    walked: list[str] = []
    for key in path.split("."):
        if not isinstance(node, dict) or key not in node:
            where = ".".join(walked) or "<root>"
            raise PlatformSpecError(f"missing key {path!r}: {where} has no {key!r}")
        node = node[key]
        walked.append(key)
    if node is None:
        raise PlatformSpecError(f"key {path!r} is null; the spec is incomplete")
    return node


def _number(mapping: dict[str, Any], path: str, *, positive: bool = True) -> float:
    value = _require(mapping, path)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlatformSpecError(f"key {path!r} must be a number, got {value!r}")
    if positive and value <= 0.0:
        raise PlatformSpecError(f"key {path!r} must be positive, got {value!r}")
    return float(value)


def _triple(mapping: dict[str, Any], path: str) -> tuple[float, float, float]:
    value = _require(mapping, path)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise PlatformSpecError(f"key {path!r} must be a list of three numbers, got {value!r}")
    return (float(value[0]), float(value[1]), float(value[2]))


@dataclass(frozen=True, slots=True)
class PlatformSpec:
    """The parsed platform spec. All lengths in **metres**, all masses in kg, loads in N.

    Only the fields something downstream consumes, or that a cross-check needs, are
    modelled. Motor and battery values are read but not exposed yet: nothing computes an
    actuation constraint today, and a field that exists but is never read is how the
    duplication this module exists to remove got started.
    """

    #: --- provenance -------------------------------------------------------------------
    name: str
    frozen: bool
    frozen_date: str | None
    source_path: Path

    #: --- chassis ----------------------------------------------------------------------
    chassis_mass_kg: float
    chassis_length_m: float
    chassis_width_m: float
    chassis_height_m: float

    #: --- drivetrain -------------------------------------------------------------------
    configuration: str
    n_driven_wheels: int
    track_width_m: float

    #: --- interface --------------------------------------------------------------------
    shaft_diameter_m: float

    #: --- wheel envelope ---------------------------------------------------------------
    min_radius_m: float
    max_radius_m: float
    min_width_m: float
    max_width_m: float
    wheel_well_radius_m: float
    max_mass_fraction: float

    #: --- operating point --------------------------------------------------------------
    nominal_wheel_load_n: float

    #: --- manufacturing ----------------------------------------------------------------
    bed_size_m: tuple[float, float, float]
    min_interspoke_gap_m: float
    min_wall_thickness_tpu_m: float
    min_wall_thickness_rigid_m: float
    max_material_grams: float

    # ----------------------------------------------------------------------------------
    # Derived
    # ----------------------------------------------------------------------------------

    @property
    def max_wheel_mass_kg(self) -> float:
        """Mass budget for one wheel, from ``wheel_envelope.max_mass_fraction``."""
        return self.max_mass_fraction * self.chassis_mass_kg

    def platform_limits(self) -> PlatformLimits:
        """The screening envelope, in millimetres.

        This is the conversion boundary. Everything above is metres; everything returned
        here is millimetres.
        """
        return PlatformLimits(
            wheel_well_radius_mm=1e3 * self.wheel_well_radius_m,
            max_width_mm=1e3 * self.max_width_m,
            shaft_radius_mm=1e3 * 0.5 * self.shaft_diameter_m,
            bed_size_mm=(
                1e3 * self.bed_size_m[0],
                1e3 * self.bed_size_m[1],
                1e3 * self.bed_size_m[2],
            ),
            min_interspoke_gap_mm=1e3 * self.min_interspoke_gap_m,
            min_wall_thickness_tpu_mm=1e3 * self.min_wall_thickness_tpu_m,
            min_wall_thickness_rigid_mm=1e3 * self.min_wall_thickness_rigid_m,
        )

    def param_bounds(self) -> dict[str, tuple[float, float]]:
        """Search bounds this spec pins down, in millimetres.

        Only radius and width: every other entry in
        :data:`wheelopt.cad.params.PARAM_BOUNDS` is a property of the wheel design space,
        not of the robot, and the robot spec has no business claiming them.
        """
        return {
            "outer_radius_mm": (1e3 * self.min_radius_m, 1e3 * self.max_radius_m),
            "width_mm": (1e3 * self.min_width_m, 1e3 * self.max_width_m),
        }

    # ----------------------------------------------------------------------------------
    # Checks
    # ----------------------------------------------------------------------------------

    def require_frozen(self) -> None:
        """Raise unless ``meta.frozen`` is true.

        A campaign driver calls this before spending compute. The YAML's own header says a
        run against unfrozen values is a pipeline test, not a result; this is what makes
        that statement enforceable rather than advisory.
        """
        if not self.frozen:
            raise PlatformSpecError(
                f"platform spec {self.source_path} is not frozen (meta.frozen is false). "
                "Results produced against it are pipeline tests, not results. Set "
                "meta.frozen and meta.frozen_date once the values are settled."
            )

    def consistency_warnings(self) -> list[str]:
        """Soft cross-checks between values that are stated independently.

        Each of these *could* be derived instead of stated, and deliberately is not: the
        YAML is meant to be readable and hand-editable, and a derived field cannot be
        overridden when reality disagrees with the formula. The cost of stating them is
        that they can disagree, so this reports where they do. Returns an empty list when
        the spec is coherent; never raises.
        """
        out: list[str] = []

        # The load per wheel against the mass it carries. `chassis_mass_kg` excludes the
        # wheels, so the total is chassis + n x wheel, and the wheel mass is not known
        # without geometry (invariant 2) — so bracket it: bare chassis at the low end,
        # chassis plus a full mass budget of wheels at the high end.
        lo = self.chassis_mass_kg * _G / self.n_driven_wheels
        hi = (self.chassis_mass_kg + self.n_driven_wheels * self.max_wheel_mass_kg) * _G / (
            self.n_driven_wheels
        )
        if not lo <= self.nominal_wheel_load_n <= hi:
            out.append(
                f"operating_point.nominal_wheel_load = {self.nominal_wheel_load_n:.1f} N is "
                f"outside [{lo:.1f}, {hi:.1f}] N, the range implied by chassis.mass = "
                f"{self.chassis_mass_kg:g} kg over {self.n_driven_wheels} wheels"
            )

        # A wheel larger than the bed cannot be printed whatever the chassis allows. The
        # YAML says the bed is the binding limit; this checks that it still is.
        bed_min = min(self.bed_size_m[0], self.bed_size_m[1])
        if 2.0 * self.max_radius_m > bed_min:
            out.append(
                f"wheel_envelope.max_radius = {self.max_radius_m:g} m is a "
                f"{2e3 * self.max_radius_m:.0f} mm disc, larger than the "
                f"{1e3 * bed_min:.0f} mm print bed"
            )

        if self.min_radius_m > self.max_radius_m:
            out.append("wheel_envelope.min_radius exceeds max_radius")
        if self.min_width_m > self.max_width_m:
            out.append("wheel_envelope.min_width exceeds max_width")

        # Wheels hang outboard, so the track is the chassis width plus two wheel widths at
        # minimum. A track narrower than the chassis would mean they are inside it.
        if self.track_width_m < self.chassis_width_m:
            out.append(
                f"drivetrain.track_width = {self.track_width_m:g} m is narrower than the "
                f"{self.chassis_width_m:g} m chassis, but the wheels are outboard"
            )

        if self.wheel_well_radius_m < self.max_radius_m:
            out.append(
                f"wheel_envelope.wheel_well_radius = {self.wheel_well_radius_m:g} m is "
                f"smaller than max_radius = {self.max_radius_m:g} m, so no wheel at the top "
                "of the range fits"
            )

        if self.frozen and not self.frozen_date:
            out.append("meta.frozen is true but meta.frozen_date is unset")

        return out


def load_platform(path: str | Path | None = None) -> PlatformSpec:
    """Read and validate a platform spec.

    Raises :class:`PlatformSpecError` if the file is missing, unparseable, or missing a
    required key. Does **not** raise on :meth:`PlatformSpec.consistency_warnings` — those
    are reported, not enforced, so that an intentionally odd platform can still be studied.
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment-dependent
        raise PlatformSpecError(
            "reading the platform spec needs PyYAML (`pip install pyyaml`)"
        ) from exc

    config_path = Path(path) if path is not None else default_config_path()
    try:
        text = config_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise PlatformSpecError(f"cannot read platform spec {config_path}: {exc}") from exc

    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise PlatformSpecError(f"{config_path} is not valid YAML: {exc}") from exc

    if not isinstance(raw, dict):
        raise PlatformSpecError(f"{config_path} must contain a mapping at the top level")

    frozen = _require(raw, "meta.frozen")
    if not isinstance(frozen, bool):
        raise PlatformSpecError(f"meta.frozen must be a boolean, got {frozen!r}")

    frozen_date_raw = raw.get("meta", {}).get("frozen_date")
    if isinstance(frozen_date_raw, (_dt.date, _dt.datetime)):
        frozen_date = frozen_date_raw.isoformat()
    elif frozen_date_raw is None:
        frozen_date = None
    else:
        frozen_date = str(frozen_date_raw)

    n_driven = _require(raw, "drivetrain.n_driven_wheels")
    if not isinstance(n_driven, int) or isinstance(n_driven, bool) or n_driven < 1:
        raise PlatformSpecError(
            f"drivetrain.n_driven_wheels must be a positive integer, got {n_driven!r}"
        )

    spec = PlatformSpec(
        name=str(_require(raw, "meta.name")),
        frozen=frozen,
        frozen_date=frozen_date,
        source_path=config_path,
        chassis_mass_kg=_number(raw, "chassis.mass"),
        chassis_length_m=_number(raw, "chassis.length"),
        chassis_width_m=_number(raw, "chassis.width"),
        chassis_height_m=_number(raw, "chassis.height"),
        configuration=str(_require(raw, "drivetrain.configuration")),
        n_driven_wheels=n_driven,
        track_width_m=_number(raw, "drivetrain.track_width"),
        shaft_diameter_m=_number(raw, "wheel_interface.shaft_diameter"),
        min_radius_m=_number(raw, "wheel_envelope.min_radius"),
        max_radius_m=_number(raw, "wheel_envelope.max_radius"),
        min_width_m=_number(raw, "wheel_envelope.min_width"),
        max_width_m=_number(raw, "wheel_envelope.max_width"),
        wheel_well_radius_m=_number(raw, "wheel_envelope.wheel_well_radius"),
        max_mass_fraction=_number(raw, "wheel_envelope.max_mass_fraction"),
        nominal_wheel_load_n=_number(raw, "operating_point.nominal_wheel_load"),
        bed_size_m=_triple(raw, "manufacturing.bed_size"),
        min_interspoke_gap_m=_number(raw, "manufacturing.min_interspoke_gap"),
        min_wall_thickness_tpu_m=_number(raw, "manufacturing.min_wall_thickness_tpu"),
        min_wall_thickness_rigid_m=_number(raw, "manufacturing.min_wall_thickness_rigid"),
        max_material_grams=_number(raw, "manufacturing.max_material_grams"),
    )

    # The units guard. The spec header promises SI; a file written in millimetres would
    # parse cleanly and then screen every design against a 0.22 mm print bed. Chassis
    # dimensions are the safest tell — a terrestrial robot is not 400 metres long.
    if spec.chassis_length_m > 10.0 or spec.max_radius_m > 1.0:
        raise PlatformSpecError(
            f"{config_path} looks like it is in millimetres, not metres "
            f"(chassis.length = {spec.chassis_length_m:g}, "
            f"wheel_envelope.max_radius = {spec.max_radius_m:g}). "
            "The platform spec is SI throughout; see its header."
        )

    return spec
