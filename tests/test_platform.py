"""The platform spec loader, and the drift guard between it and the code defaults.

Most of this file is one idea: ``configs/robot.yaml`` claims to be the source of truth, and
several dataclass defaults duplicate parts of it for speed. Duplication is fine; *silent*
duplication is not. :class:`TestSpecMatchesCodeDefaults` is what turns "they agree because
someone edited both" into "they agree because CI says so".

The rest covers the loader's failure surface. A malformed platform spec must fail loudly at
startup — see the docstring on ``PlatformSpecError`` for why this is not a violation of
invariant 4.
"""

from __future__ import annotations

import textwrap
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from wheelopt.cad.constraints import PlatformLimits, check_design, is_feasible
from wheelopt.cad.materials import TPU95A
from wheelopt.cad.params import PARAM_BOUNDS, WheelParams
from wheelopt.fea.loadcase import LoadCase
from wheelopt.platform import (
    PlatformSpec,
    PlatformSpecError,
    default_config_path,
    load_platform,
)

SPEC = load_platform()


class TestLoadsCheckedInSpec(unittest.TestCase):
    def test_default_path_is_the_repo_config(self):
        self.assertEqual(default_config_path().name, "robot.yaml")
        self.assertTrue(default_config_path().is_file())

    def test_parses(self):
        self.assertIsInstance(SPEC, PlatformSpec)
        self.assertEqual(SPEC.n_driven_wheels, 4)
        self.assertEqual(SPEC.configuration, "skid_steer")

    def test_the_chassis_is_the_stated_requirement(self):
        # 400 x 300 x 200 mm was given as a hard platform requirement, not derived.
        # Everything else in the spec hangs off it, so it gets its own check.
        self.assertAlmostEqual(SPEC.chassis_length_m, 0.400)
        self.assertAlmostEqual(SPEC.chassis_width_m, 0.300)
        self.assertAlmostEqual(SPEC.chassis_height_m, 0.200)

    def test_is_internally_consistent(self):
        self.assertEqual(SPEC.consistency_warnings(), [])

    def test_is_not_frozen_yet(self):
        # Deliberate: the chassis envelope is a requirement, but the mass, motors, battery
        # and inertia are class-typical estimates. Freezing would assert a confidence
        # nothing has earned. When that changes, this test changes with it.
        self.assertFalse(SPEC.frozen)
        with self.assertRaises(PlatformSpecError):
            SPEC.require_frozen()


class TestSpecMatchesCodeDefaults(unittest.TestCase):
    """The drift guard. Each of these is a value stated in two places."""

    def test_platform_limits_round_trip(self):
        # Field by field rather than `==` on the dataclass: the metre->millimetre
        # multiply is exact for today's values but need not stay that way, and a drift
        # guard that fails on the last bit of a float teaches people to delete it.
        # `_fields` is compared first so a field added to PlatformLimits and not to the
        # loader is still caught.
        from_spec = SPEC.platform_limits()
        expected = PlatformLimits()
        self.assertEqual(PlatformLimits.__slots__, type(from_spec).__slots__)
        for field in PlatformLimits.__slots__:
            with self.subTest(field=field):
                actual, want = getattr(from_spec, field), getattr(expected, field)
                if isinstance(want, tuple):
                    self.assertEqual(len(actual), len(want))
                    for a, w in zip(actual, want, strict=True):
                        self.assertAlmostEqual(a, w, places=9)
                else:
                    self.assertAlmostEqual(actual, want, places=9)

    def test_units_were_converted(self):
        # The specific failure this guards: metres passed through as millimetres. Asserting
        # equality above would catch it, but only if someone reads the number; assert the
        # magnitude explicitly so the intent survives a future edit.
        limits = SPEC.platform_limits()
        self.assertAlmostEqual(limits.wheel_well_radius_mm, 105.0)
        self.assertAlmostEqual(limits.shaft_radius_mm, 4.0)
        self.assertEqual(limits.bed_size_mm, (220.0, 220.0, 250.0))

    def test_search_bounds_agree(self):
        for field, bounds in SPEC.param_bounds().items():
            with self.subTest(field=field):
                self.assertIn(field, PARAM_BOUNDS)
                lo, hi = PARAM_BOUNDS[field]
                self.assertAlmostEqual(bounds[0], lo)
                self.assertAlmostEqual(bounds[1], hi)

    def test_nominal_load_agrees_with_the_fea_default(self):
        # If these diverge, every k_r(delta) is scaled against a load nobody specified and
        # the `fea_load_range` warning fires against the wrong target.
        self.assertAlmostEqual(SPEC.nominal_wheel_load_n, LoadCase().nominal_load_n)

    def test_bore_matches_the_shaft(self):
        self.assertAlmostEqual(
            WheelParams().hub_bore_radius_mm, 1e3 * 0.5 * SPEC.shaft_diameter_m
        )

    def test_the_nominal_design_screens_clean_against_the_spec(self):
        # End to end: the default wheel, screened against limits built from the YAML rather
        # than from the dataclass defaults.
        violations = check_design(WheelParams(), TPU95A, SPEC.platform_limits())
        self.assertTrue(is_feasible(violations), [str(x) for x in violations])

    def test_the_nominal_design_is_inside_the_envelope(self):
        params = WheelParams()
        self.assertLessEqual(1e3 * SPEC.min_radius_m, params.outer_radius_mm)
        self.assertLessEqual(params.outer_radius_mm, 1e3 * SPEC.max_radius_m)
        self.assertLessEqual(1e3 * SPEC.min_width_m, params.width_mm)
        self.assertLessEqual(params.width_mm, 1e3 * SPEC.max_width_m)


class TestDerived(unittest.TestCase):
    def test_wheel_mass_budget(self):
        # 5% of 8.8 kg. A budget, not a measurement — the actual mass comes from geometry.
        self.assertAlmostEqual(SPEC.max_wheel_mass_kg, 0.44)

    def test_param_bounds_claims_only_robot_properties(self):
        # The spec constrains how big a wheel may be. It has no opinion on spoke curvature,
        # and must not acquire one by accident.
        self.assertEqual(set(SPEC.param_bounds()), {"outer_radius_mm", "width_mm"})


MINIMAL = """
meta:
  name: test-rig
  frozen: false
  frozen_date: null
chassis:
  mass: 8.8
  length: 0.40
  width: 0.30
  height: 0.20
  com_offset: [0.0, 0.0, 0.0]
  inertia: [0.0953, 0.1467, 0.1833]
  ground_clearance_min: 0.070
drivetrain:
  configuration: skid_steer
  n_driven_wheels: 4
  track_width: 0.35
  wheelbase: 0.26
motor:
  stall_torque: 4.0
  no_load_speed: 14.0
wheel_interface:
  shaft_diameter: 0.008
wheel_envelope:
  min_radius: 0.060
  max_radius: 0.100
  min_width: 0.030
  max_width: 0.070
  wheel_well_radius: 0.105
  max_mass_fraction: 0.05
operating_point:
  nominal_wheel_load: 24.5
  target_speed: 0.6
manufacturing:
  bed_size: [0.220, 0.220, 0.250]
  min_interspoke_gap: 0.002
  min_wall_thickness_tpu: 0.0016
  min_wall_thickness_rigid: 0.0012
  max_material_grams: 450
"""


class SpecFileCase(unittest.TestCase):
    """Helper: write a spec to a temp file and load it."""

    def load(self, text: str) -> PlatformSpec:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "robot.yaml"
            path.write_text(textwrap.dedent(text), encoding="utf-8")
            return load_platform(path)

    def assert_rejects(self, text: str, *, contains: str) -> None:
        with self.assertRaises(PlatformSpecError) as ctx:
            self.load(text)
        self.assertIn(contains, str(ctx.exception))


class TestLoaderRejectsBadSpecs(SpecFileCase):
    def test_minimal_spec_loads(self):
        spec = self.load(MINIMAL)
        self.assertEqual(spec.name, "test-rig")
        self.assertEqual(spec.consistency_warnings(), [])

    def test_the_vehicle_fields_are_read(self):
        """The block `wheelopt.sim`'s rover needs. Every one of these sat in `robot.yaml`
        unread until 2026-08-09, so the single-wheel rig invented its drive from a heuristic
        instead of the platform's own motor."""
        spec = self.load(MINIMAL)
        self.assertAlmostEqual(spec.wheelbase_m, 0.26)
        self.assertEqual(spec.chassis_inertia_kg_m2, (0.0953, 0.1467, 0.1833))
        self.assertEqual(spec.com_offset_m, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(spec.ground_clearance_m, 0.070)
        self.assertAlmostEqual(spec.stall_torque_n_m, 4.0)
        self.assertAlmostEqual(spec.no_load_speed_rad_s, 14.0)
        self.assertAlmostEqual(spec.target_speed_m_s, 0.6)

    def test_a_spec_without_a_wheelbase_is_refused(self):
        """Required, not defaulted. A default wheelbase would place four wheels somewhere
        plausible and wrong, which is worse than refusing."""
        self.assert_rejects(MINIMAL.replace("  wheelbase: 0.26\n", ""),
                            contains="drivetrain.wheelbase")

    def test_missing_file(self):
        with self.assertRaises(PlatformSpecError):
            load_platform(Path("/nonexistent") / "robot.yaml")

    def test_missing_key_names_the_path(self):
        self.assert_rejects(
            MINIMAL.replace("  nominal_wheel_load: 24.5\n", ""),
            contains="operating_point.nominal_wheel_load",
        )

    def test_null_value_is_not_a_value(self):
        # The spec shipped with ~40 nulls before it was filled in. A null that parses as
        # "present" would screen every design against None.
        self.assert_rejects(
            MINIMAL.replace("nominal_wheel_load: 24.5", "nominal_wheel_load: null"),
            contains="incomplete",
        )

    def test_non_numeric_value(self):
        self.assert_rejects(
            MINIMAL.replace("nominal_wheel_load: 24.5", "nominal_wheel_load: heavy"),
            contains="must be a number",
        )

    def test_zero_load_is_rejected(self):
        self.assert_rejects(
            MINIMAL.replace("nominal_wheel_load: 24.5", "nominal_wheel_load: 0.0"),
            contains="must be positive",
        )

    def test_millimetres_are_caught(self):
        # The units trap: a spec written in mm parses perfectly and is wrong by 1000x
        # everywhere. Catch it at the door.
        mm = (
            MINIMAL.replace("length: 0.40", "length: 400")
            .replace("width: 0.30", "width: 300")
            .replace("height: 0.20", "height: 200")
            .replace("max_radius: 0.100", "max_radius: 100")
        )
        self.assert_rejects(mm, contains="millimetres")

    def test_bad_yaml(self):
        self.assert_rejects("meta: {name: x\n  frozen: false", contains="not valid YAML")

    def test_top_level_must_be_a_mapping(self):
        self.assert_rejects("- just\n- a\n- list\n", contains="mapping")

    def test_wheel_count_must_be_a_positive_integer(self):
        self.assert_rejects(
            MINIMAL.replace("n_driven_wheels: 4", "n_driven_wheels: 0"),
            contains="positive integer",
        )

    def test_bed_size_must_be_a_triple(self):
        self.assert_rejects(
            MINIMAL.replace("bed_size: [0.220, 0.220, 0.250]", "bed_size: 0.220"),
            contains="three numbers",
        )

    def test_frozen_must_be_boolean(self):
        self.assert_rejects(
            MINIMAL.replace("frozen: false", "frozen: yes-please"),
            contains="must be a boolean",
        )


class TestConsistencyWarnings(SpecFileCase):
    """These report rather than raise: an unusual platform is still a studiable platform."""

    def test_load_inconsistent_with_mass(self):
        spec = self.load(MINIMAL.replace("nominal_wheel_load: 24.5", "nominal_wheel_load: 5.0"))
        self.assertTrue(any("nominal_wheel_load" in w for w in spec.consistency_warnings()))

    def test_wheel_larger_than_the_bed(self):
        spec = self.load(
            MINIMAL.replace("max_radius: 0.100", "max_radius: 0.200").replace(
                "wheel_well_radius: 0.105", "wheel_well_radius: 0.210"
            )
        )
        self.assertTrue(any("print bed" in w for w in spec.consistency_warnings()))

    def test_wheel_well_smaller_than_the_largest_wheel(self):
        spec = self.load(MINIMAL.replace("wheel_well_radius: 0.105", "wheel_well_radius: 0.080"))
        self.assertTrue(any("wheel_well_radius" in w for w in spec.consistency_warnings()))

    def test_wheelbase_longer_than_the_chassis(self):
        spec = self.load(MINIMAL.replace("wheelbase: 0.26", "wheelbase: 0.50"))
        self.assertTrue(any("wheelbase" in w for w in spec.consistency_warnings()))

    def test_inertia_that_no_longer_matches_its_own_provenance(self):
        """`chassis.inertia` is stated, not derived, so that a measurement can replace the
        formula. The cost is that it can drift from the formula its comment still cites."""
        spec = self.load(MINIMAL.replace("inertia: [0.0953, 0.1467, 0.1833]",
                                         "inertia: [0.0953, 0.9000, 0.1833]"))
        self.assertTrue(any("uniform-box" in w for w in spec.consistency_warnings()))
        # And the untouched axes must not also complain, or the check says nothing.
        self.assertEqual(sum("uniform-box" in w for w in spec.consistency_warnings()), 1)

    def test_a_motor_that_cannot_move_the_robot(self):
        spec = self.load(MINIMAL.replace("stall_torque: 4.0", "stall_torque: 0.4"))
        self.assertTrue(any("tractive force" in w for w in spec.consistency_warnings()))

    def test_a_target_speed_the_drivetrain_cannot_reach(self):
        spec = self.load(MINIMAL.replace("target_speed: 0.6", "target_speed: 3.0"))
        self.assertTrue(any("target_speed" in w for w in spec.consistency_warnings()))

    def test_track_narrower_than_the_chassis(self):
        spec = self.load(MINIMAL.replace("track_width: 0.35", "track_width: 0.25"))
        self.assertTrue(any("track_width" in w for w in spec.consistency_warnings()))

    def test_inverted_envelope(self):
        spec = self.load(MINIMAL.replace("min_width: 0.030", "min_width: 0.090"))
        self.assertTrue(any("min_width" in w for w in spec.consistency_warnings()))

    def test_frozen_without_a_date(self):
        spec = self.load(MINIMAL.replace("frozen: false", "frozen: true"))
        self.assertTrue(any("frozen_date" in w for w in spec.consistency_warnings()))
        spec.require_frozen()  # frozen is frozen, date or not

    def test_a_date_object_becomes_a_string(self):
        # PyYAML parses an unquoted ISO date into datetime.date, not str.
        spec = self.load(
            MINIMAL.replace("frozen: false", "frozen: true").replace(
                "frozen_date: null", "frozen_date: 2026-08-07"
            )
        )
        self.assertEqual(spec.frozen_date, "2026-08-07")
        self.assertEqual(spec.consistency_warnings(), [])


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
