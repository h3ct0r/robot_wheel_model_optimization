"""Every flag has help, and the bounds it quotes are the bounds that are enforced.

Help text is the part of a CLI nobody notices rotting. Two failures matter here and both are
silent: a flag added without help, which then reads as though it does nothing; and a help
string that states a range the screening no longer uses, which is worse than none — it tells
you a value is legal right up until the run is rejected for it.

`scripts/` is not a package, so the parsers are loaded by path.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
import unittest
from pathlib import Path

from wheelopt.cad.cli import add_geometry_args, add_material_args
from wheelopt.cad.params import PARAM_BOUNDS

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load(name: str):
    """Import a script by path and hand back its module.

    Registered in ``sys.modules`` before it is executed, which is not optional: a
    ``@dataclass(slots=True)`` in the script rebuilds its class and looks itself up by
    ``cls.__module__``, so an unregistered module fails with an `AttributeError` about
    `NoneType` that says nothing about the real cause.
    """
    spec = importlib.util.spec_from_file_location(f"_script_{name}", SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(spec.name, None)
        raise
    return module


def flags(parser: argparse.ArgumentParser) -> list[argparse.Action]:
    return [a for a in parser._actions if a.dest not in ("help", argparse.SUPPRESS)]


class TestSharedArgs(unittest.TestCase):
    """`cad/cli.py` feeds every entry point, so a gap here is a gap in all of them."""

    def parser(self) -> argparse.ArgumentParser:
        p = argparse.ArgumentParser()
        add_geometry_args(p)
        add_material_args(p)
        return p

    def test_every_geometry_and_material_flag_has_help(self):
        missing = [a.option_strings[0] for a in flags(self.parser()) if not a.help]
        self.assertEqual(missing, [], f"no help for {missing}")

    def test_the_quoted_bounds_are_the_enforced_bounds(self):
        """The help reads `PARAM_BOUNDS` rather than restating it, so this checks the wiring
        rather than the numbers: change a bound and the help follows, with no second edit."""
        by_dest = {a.dest: a for a in flags(self.parser())}
        for dest, field in (("radius", "outer_radius_mm"), ("width", "width_mm"),
                            ("spokes", "n_spokes"), ("thickness", "spoke_thickness_mm"),
                            ("claw_taper", "claw_taper_ratio")):
            low, high = PARAM_BOUNDS[field]
            with self.subTest(flag=dest):
                self.assertIn(f"{low:g} to {high:g}", by_dest[dest].help)

    def test_units_are_stated_where_a_number_carries_one(self):
        by_dest = {a.dest: a for a in flags(self.parser())}
        for dest, unit in (("radius", "mm"), ("width", "mm"), ("hub_radius", "mm"),
                           ("bore_radius", "mm"), ("thickness", "mm"),
                           ("curvature", "1/mm"), ("spoke_phase", "degrees")):
            with self.subTest(flag=dest):
                self.assertIn(unit, by_dest[dest].help)


class TestRunRover(unittest.TestCase):
    def setUp(self):
        self.parser = load("run_rover").build_parser()

    def test_every_flag_has_help(self):
        missing = [a.option_strings[0] for a in flags(self.parser) if not a.help]
        self.assertEqual(missing, [], f"no help for {missing}")

    def test_the_unit_that_breaks_the_pattern_says_so(self):
        """`--delta-max` is metres while every geometry flag is millimetres. That is a real
        wart; until it is fixed the help has to shout about it, because the failure mode is a
        caller passing 12 and pressing the wheel twelve metres."""
        action = next(a for a in flags(self.parser) if a.dest == "delta_max")
        self.assertIn("METRES", action.help)
        self.assertEqual(action.metavar, "METRES")

    def test_switches_do_not_advertise_a_false_default(self):
        """`(default: False)` on every switch is a column of noise between the reader and the
        text that matters, and `(default: None)` reads as though nothing happens."""
        text = self.parser.format_help()
        self.assertNotIn("(default: False)", text)
        self.assertNotIn("(default: None)", text)

    def test_values_still_show_their_defaults(self):
        text = self.parser.format_help()
        self.assertIn("(default: 60.0)", text)      # --obstacle-height
        self.assertIn("(default: 25)", text)        # --fps

    def test_the_examples_survive_formatting(self):
        """Runnable command lines in the epilog, which a non-raw formatter reflows into an
        unusable paragraph."""
        text = self.parser.format_help()
        self.assertIn("examples:", text)
        self.assertIn("--compliant --stl", text)
        self.assertIn("exit codes:", text)

    def test_the_help_names_the_flags_it_talks_about(self):
        """A cross-reference that has gone stale points the reader at nothing."""
        text = self.parser.format_help()
        for referenced in ("--law claw", "--spokes", "--compliant", "--stl", "--infill"):
            with self.subTest(flag=referenced):
                self.assertIn(referenced, text)


class TestOtherEntryPoints(unittest.TestCase):
    """The shared groups reach these too; this catches a script-local flag added without help."""

    def test_run_step_and_run_s1_have_help_on_every_flag(self):
        for name in ("run_step", "run_s1", "plot_geometry"):
            with self.subTest(script=name):
                parser = load(name).build_parser()
                missing = [a.option_strings[0] for a in flags(parser) if not a.help]
                self.assertEqual(missing, [], f"{name}: no help for {missing}")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
