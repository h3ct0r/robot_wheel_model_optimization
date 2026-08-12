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
import math
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
    modelled. A field that exists but is never read is how the duplication this module
    exists to remove got started, so each block below names its consumer.

    The **vehicle** block arrived with the four-wheel rover scenario (2026-08-09). Before it,
    the single-wheel rig invented its drive from ``RigSpec.torque_ratio = 1.3`` — a heuristic
    that happens to reproduce this platform's own sizing rationale, but is not the platform's
    motor. A rover has to use the real one, and it has to know where the axles are.

    Battery values are still read-and-discarded: nothing computes an energy budget yet.
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
    #: Track measured at the ORIGINAL r 22.5 mm wheels, which tuck under the shell, metres.
    #: A reference measurement, not the operating value: candidate wheels (R 40–90) cannot
    #: tuck under and mount EXTERNALLY on the axle stubs, so use :meth:`track_for` — track is
    #: wheel-dependent on this robot, exactly as clearance is.
    track_width_m: float
    #: Lateral distance from the chassis centreline to the plate face an external wheel's
    #: inner side seats against, metres. Measured off `configs/pipebot_simplified.stl`
    #: (2026-08-11): axle stubs emerge at x = +97/−98 about a midline of −0.5 — ±97.5 exactly.
    wheel_mount_face_m: float
    #: Front axle to rear axle, metres. Needed to place wheels; unused by screening.
    wheelbase_m: float

    #: --- vehicle dynamics, for `wheelopt.sim` -----------------------------------------
    #: Principal moments of the chassis about its own centre of mass, kg·m². The uniform-box
    #: formula on the chassis dimensions, per `robot.yaml` — an estimate, like the mass.
    chassis_inertia_kg_m2: tuple[float, float, float]
    #: Centre of mass relative to the chassis geometric centre, metres.
    com_offset_m: tuple[float, float, float]
    #: Clearance measured at the ORIGINAL r 22.5 mm wheels, metres. A reference measurement,
    #: not the operating value: use :meth:`ground_clearance_for`, because clearance is
    #: wheel-dependent on this robot.
    ground_clearance_m: float
    #: Belly height above the axle line, metres — bracket geometry, invariant under wheel
    #: swaps. Real clearance = wheel radius + this.
    axle_to_belly_m: float
    #: Per driven wheel, **at the output** — after the gearbox. `motor.stall_torque` and
    #: `motor.no_load_speed` in the YAML.
    stall_torque_n_m: float
    no_load_speed_rad_s: float
    #: `operating_point.target_speed`, m/s.
    target_speed_m_s: float

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

    def digest(self) -> str:
        """Content hash of every numeric field, for run identity (invariant 5).

        The platform shapes every simulated number — mass, motor curve, clearance, all of it —
        and until 2026-08-11 none of that was in a run's identity: re-measuring the robot
        produced the **same run_ids with different metrics inside**, which the cross-machine
        manifest gate reads as non-determinism. It is the third instance of the same bug
        (`S1Config.rung_name` and the FEA `SolverSpec` were the others), so the fix follows
        the same rule: anything that changes the numbers is in the key. Provenance fields
        (`name`, `source_path`, `frozen*`) are the named exclusions — renaming the file does
        not change what it measured.
        """
        from .hashing import content_digest

        payload = {
            field: getattr(self, field)
            for field in self.__dataclass_fields__
            if field not in ("name", "frozen", "frozen_date", "source_path")
        }
        return content_digest(payload)[:12]

    def ground_clearance_for(self, wheel_radius_m: float) -> float:
        """Belly height above the ground with a given wheel fitted, metres.

        ``wheel_radius + axle_to_belly``. The stated ``ground_clearance_m`` is the same
        formula evaluated at the original r 22.5 mm wheels; treating it as a constant sank
        an R 85 mm candidate's axle 55 mm above its own belly in the simulator, which is how
        this method came to exist (2026-08-11 log). On this robot a bigger wheel buys belly
        clearance one-for-one, which is most of why wheel radius matters at all.
        """
        if wheel_radius_m <= 0.0:
            raise PlatformSpecError("wheel_radius_m must be positive")
        return wheel_radius_m + self.axle_to_belly_m

    def track_for(self, wheel_width_m: float) -> float:
        """Track with a given external wheel fitted, metres: ``2·(mount_face + width/2)``.

        Candidate wheels mount **externally** — inner face against the side plate at
        ``wheel_mount_face_m``, hub on the axle stub — so a wider wheel stands further out
        and widens its own support polygon. That is physics, not flattery: unlike the
        radius yardstick in :meth:`tipover_angles_rad`, width genuinely moves the wheel
        contact line. The stated ``track_width_m`` is the ORIGINAL r 22.5 wheels' track
        (they tuck under the shell, a mounting no candidate wheel can reach).
        """
        if wheel_width_m <= 0.0:
            raise PlatformSpecError("wheel_width_m must be positive")
        return 2.0 * (self.wheel_mount_face_m + 0.5 * wheel_width_m)

    def tipover_angles_rad(self, track_m: float | None = None) -> tuple[float, float]:
        """Static tip-over angles ``(pitch_crit, roll_crit)``, radians.

        The angle at which the chassis CG passes vertically over the wheel contact line:
        ``atan(half_wheelbase / z_cg)`` about pitch, ``atan(half_track / z_cg)`` about roll,
        with ``z_cg`` the CG height above the ground — ride height at the chassis centre plus
        the stated CG offset. The stability objective (`08-metrics.md` objective 5) scores
        peak excursions against these.

        **Chassis-only, and that is the conservative side.** The wheels (~1.2 kg of a ~9 kg
        robot) sit at axle height, below the chassis CG, so the true combined CG is lower and
        the true critical angles larger than these. A margin computed against this pair
        under-reports safety rather than over-reporting it.

        Static, not dynamic: a robot climbing at speed can tip below these angles (momentum)
        or hang past them (a wheel against a riser). They are the reference the *margin* is
        expressed in, not a prediction of the tipping instant — the same role ride height
        plays for the climb predicate.
        """
        # At the reference wheels; the margin is a comparative score, and quoting it against
        # one fixed yardstick keeps designs comparable (a per-wheel yardstick would let a
        # taller wheel flatter its own margin by raising the angles it is scored against).
        # The TRACK may be overridden, and that is not the same concession: an external
        # wheel's width genuinely moves the contact line outward (`track_for`), where a
        # wheel's radius merely raises the CG it is scored under.
        z_cg = self.ground_clearance_m + 0.5 * self.chassis_height_m + self.com_offset_m[2]
        if z_cg <= 0.0:
            raise PlatformSpecError(
                f"CG height {z_cg:.4f} m is not above the ground; tip-over angles are "
                "undefined. Check chassis.com_offset against the ride height."
            )
        track = self.track_width_m if track_m is None else track_m
        return (math.atan2(0.5 * self.wheelbase_m, z_cg),
                math.atan2(0.5 * track, z_cg))

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

        # The measured robot tucks its wheels UNDER the shell (track 157 inside a 231 mm
        # body), so a track narrower than the chassis is the normal state, not a
        # contradiction — the pre-2026-08-11 version of this check warned on exactly that,
        # having been written for the fictional outboard-wheel box robot. What would still
        # be inconsistent is a track so narrow the wheels overlap the centreline.
        if self.track_width_m < self.max_width_m:
            out.append(
                f"drivetrain.track_width = {self.track_width_m:g} m leaves less than one "
                f"max-width wheel between the two sides"
            )

        if self.wheel_well_radius_m < self.max_radius_m:
            out.append(
                f"wheel_envelope.wheel_well_radius = {self.wheel_well_radius_m:g} m is "
                f"smaller than max_radius = {self.max_radius_m:g} m, so no wheel at the top "
                "of the range fits"
            )

        # The wheelbase against the chassis it sits under. Axles inside the footprint is the
        # normal case; a wheelbase longer than the chassis means the wheels stick out fore and
        # aft, which is legal but is almost always a units slip.
        if self.wheelbase_m > self.chassis_length_m:
            out.append(
                f"drivetrain.wheelbase = {self.wheelbase_m:g} m exceeds the "
                f"{self.chassis_length_m:g} m chassis length, so the axles are outside the body"
            )

        # The stated inertia against the uniform-box formula the YAML says it came from. Not
        # derived, so that a measured inertia can replace it, but a stated one that no longer
        # matches its own stated provenance is the quiet-wrong-number failure in a new place.
        box = (
            self.chassis_mass_kg * (self.chassis_width_m**2 + self.chassis_height_m**2) / 12.0,
            self.chassis_mass_kg * (self.chassis_length_m**2 + self.chassis_height_m**2) / 12.0,
            self.chassis_mass_kg * (self.chassis_length_m**2 + self.chassis_width_m**2) / 12.0,
        )
        for axis, stated, expected in zip("xyz", self.chassis_inertia_kg_m2, box):
            if expected > 0 and abs(stated - expected) > 0.05 * expected:
                out.append(
                    f"chassis.inertia I{axis}{axis} = {stated:g} kg·m² is more than 5% from "
                    f"the uniform-box value {expected:.4f} its own comment claims"
                )

        # A motor that cannot turn the wheel it is on. Torque at the biggest permitted wheel
        # against the load it carries: below about mu = 0.3 of tractive coefficient the robot
        # cannot climb anything, whatever the wheel is made of.
        traction = (self.n_driven_wheels * self.stall_torque_n_m / self.max_radius_m)
        weight = self.chassis_mass_kg * _G
        if weight > 0 and traction < 0.3 * weight:
            out.append(
                f"motor.stall_torque = {self.stall_torque_n_m:g} N·m over "
                f"{self.n_driven_wheels} wheels gives {traction:.0f} N of tractive force at "
                f"the largest permitted wheel, under 0.3x the {weight:.0f} N vehicle weight"
            )

        if self.no_load_speed_rad_s * self.min_radius_m < self.target_speed_m_s:
            out.append(
                f"operating_point.target_speed = {self.target_speed_m_s:g} m/s is above the "
                f"{self.no_load_speed_rad_s * self.min_radius_m:.2f} m/s a "
                f"{self.no_load_speed_rad_s:g} rad/s output reaches on the smallest wheel"
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
        wheel_mount_face_m=_number(raw, "drivetrain.wheel_mount_face"),
        wheelbase_m=_number(raw, "drivetrain.wheelbase"),
        chassis_inertia_kg_m2=_triple(raw, "chassis.inertia"),
        com_offset_m=_triple(raw, "chassis.com_offset"),
        ground_clearance_m=_number(raw, "chassis.ground_clearance_min"),
        axle_to_belly_m=_number(raw, "chassis.axle_to_belly"),
        stall_torque_n_m=_number(raw, "motor.stall_torque"),
        no_load_speed_rad_s=_number(raw, "motor.no_load_speed"),
        target_speed_m_s=_number(raw, "operating_point.target_speed"),
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
