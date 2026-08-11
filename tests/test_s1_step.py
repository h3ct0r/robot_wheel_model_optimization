"""Scenario S1 — the ladder, its seeding, and the rows it puts in the store.

The config and the terrain sampling are pure and always run. Driving the robot needs MuJoCo
and is skipped without it; the full ladder is 80 runs, so the tests here use a short one and
`scripts/run_s1.py` is what exercises the real thing.
"""

from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np

from wheelopt.platform import load_platform
from wheelopt.sim.rover import RoverSpec
from wheelopt.sim.s1_step import S1_NAME, S1Config, run_s1, terrain_for_seed
from wheelopt.store import ExperimentStore, RunStatus

HAVE_MUJOCO = importlib.util.find_spec("mujoco") is not None
HAVE_STORE = (importlib.util.find_spec("pyarrow") is not None
              and importlib.util.find_spec("duckdb") is not None)

PLATFORM = load_platform()
WHEEL = {"wheel_radius_m": 0.085, "wheel_width_m": 0.030, "wheel_mass_kg": 0.30}
SHORT = S1Config(heights_m=(0.04, 0.08, 0.12), n_seeds=2, duration_s=5.0)


class TestConfig(unittest.TestCase):
    def test_the_default_ladder_spans_the_planned_range(self):
        config = S1Config()
        self.assertGreaterEqual(min(config.heights_m), 0.01)
        self.assertLessEqual(max(config.heights_m), 0.20)
        self.assertGreaterEqual(config.n_seeds, 8)

    def test_a_duplicate_rung_is_refused(self):
        """Two rungs at one height are two rows with one `run_id`, which the determinism
        gate would read as an evaluation repeated and disagreeing with itself."""
        with self.assertRaises(ValueError) as ctx:
            S1Config(heights_m=(0.04, 0.08, 0.04))
        self.assertIn("run_id", str(ctx.exception))

    def test_each_rung_gets_its_own_scenario_name(self):
        config = S1Config(heights_m=(0.04, 0.08))
        names = {config.rung_name(h) for h in config.heights_m}
        self.assertEqual(len(names), 2)
        self.assertTrue(all(n.startswith(S1_NAME + "/") for n in names))

    def test_a_malformed_ladder_is_refused(self):
        for kwargs in ({"heights_m": ()}, {"heights_m": (0.0,)}, {"n_seeds": 0},
                       {"friction_range": (1.0, 0.5)}, {"friction_range": (0.0, 1.0)}):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                S1Config(**kwargs)


class TestTerrain(unittest.TestCase):
    def test_a_seed_is_a_terrain_and_not_a_coin_flip(self):
        """The same seed gives the same world at every rung, so the ladder makes one terrain
        progressively harder. Re-sampling per rung would turn 8 seeds into 80 conditions and
        the success curve would measure the sampler as much as the wheel."""
        self.assertEqual(terrain_for_seed(3, SHORT), terrain_for_seed(3, SHORT))

    def test_different_seeds_give_different_terrain(self):
        seen = {terrain_for_seed(s, SHORT) for s in range(8)}
        self.assertEqual(len(seen), 8)

    def test_the_samples_land_inside_their_ranges(self):
        config = S1Config(friction_range=(0.3, 1.0), approach_deg=15.0)
        for seed in range(32):
            friction, approach = terrain_for_seed(seed, config)
            self.assertGreaterEqual(friction, 0.3)
            self.assertLessEqual(friction, 1.0)
            self.assertLessEqual(abs(approach), 15.0)

    def test_it_reproduces_across_processes(self):
        """`default_rng` is PCG64 by specification, not by accident, so this is the same on
        another machine two days later — which is what the Phase 0 gate asks of it."""
        self.assertAlmostEqual(terrain_for_seed(0, SHORT)[0], 0.6459144234635222)


class TestApproachAngle(unittest.TestCase):
    """The rover gained a heading for S1. Both halves had to change, not just the quaternion."""

    def xml(self, **kwargs) -> str:
        from wheelopt.sim.rover import build_rover_mjcf

        return build_rover_mjcf(PLATFORM, RoverSpec(**kwargs), **WHEEL)

    def test_the_chassis_is_yawed(self):
        square = self.xml(approach_deg=0.0)
        angled = self.xml(approach_deg=15.0)
        self.assertIn('quat="1.000000000 0 0 0.000000000"', square)
        self.assertIn(f'quat="{np.cos(np.deg2rad(7.5)):.9f} 0 0 '
                      f'{np.sin(np.deg2rad(7.5)):.9f}"', angled)

    def test_an_angled_run_gets_a_wider_step(self):
        """The lateral twin of the step-shorter-than-the-run bug. At 15 degrees a 6.9 m run
        drifts 1.8 m off centre, and the old fixed 1.5 m half-width would have let the robot
        climb the step and then drive off the side of it."""
        def half_width(text: str) -> float:
            line = next(x for x in text.splitlines() if 'name="step"' in x)
            return float(line.split('size="')[1].split()[1])

        self.assertGreater(half_width(self.xml(approach_deg=15.0)),
                           half_width(self.xml(approach_deg=0.0)))

    def test_driving_along_the_face_is_refused(self):
        with self.assertRaises(ValueError) as ctx:
            RoverSpec(approach_deg=90.0)
        self.assertIn("face", str(ctx.exception))


@unittest.skipUnless(HAVE_MUJOCO, "MuJoCo not installed")
class TestRunS1(unittest.TestCase):
    def test_it_produces_one_row_per_rung_per_seed(self):
        out = run_s1(PLATFORM, SHORT, design_hash="probe", **WHEEL)
        self.assertEqual(len(out.records), len(SHORT.heights_m) * SHORT.n_seeds)
        self.assertEqual(len(out.heights_m), len(out.records) - out.n_failed)

    def test_every_row_has_a_distinct_run_id(self):
        """The property that makes the store's determinism gate meaningful for a ladder."""
        out = run_s1(PLATFORM, SHORT, design_hash="probe", **WHEEL)
        self.assertEqual(len({r.run_id for r in out.records}), len(out.records))

    def test_repeating_the_ladder_reproduces_it_exactly(self):
        """The Phase 0 gate, run for real rather than against synthetic rows: same design,
        same seeds, same versions, so every `run_id` repeats and every verdict must agree."""
        first = run_s1(PLATFORM, SHORT, design_hash="probe", **WHEEL)
        second = run_s1(PLATFORM, SHORT, design_hash="probe", **WHEEL)
        self.assertEqual([r.run_id for r in first.records],
                         [r.run_id for r in second.records])
        self.assertEqual([r.metrics for r in first.records],
                         [r.metrics for r in second.records])

    def test_a_taller_step_is_never_easier(self):
        """Within one terrain seed the ladder should be monotone. Not asserted as a hard
        rule -- a bounce can clear a step the next one down did not -- but a seed that
        succeeds at 120 mm and fails at 40 mm means the predicate is wrong."""
        out = run_s1(PLATFORM, SHORT, design_hash="probe", **WHEEL)
        per_seed: dict[int, list[tuple[float, bool]]] = {}
        for record in out.records:
            if record.status is RunStatus.OK:
                per_seed.setdefault(record.seed, []).append(
                    (record.params["step_height_m"], bool(record.metrics["climbed"])))
        for seed, rungs in per_seed.items():
            rungs.sort()
            cleared = [h for h, ok in rungs if ok]
            failed = [h for h, ok in rungs if not ok]
            if cleared and failed:
                with self.subTest(seed=seed):
                    self.assertLess(min(cleared), max(failed))

    def test_the_rows_carry_their_terrain(self):
        out = run_s1(PLATFORM, SHORT, design_hash="probe", **WHEEL)
        row = out.records[0]
        self.assertIn("friction", row.params)
        self.assertIn("approach_deg", row.params)
        self.assertEqual(row.params["step_height_m"], SHORT.heights_m[0])

    def test_extra_provenance_is_carried_onto_every_row(self):
        out = run_s1(PLATFORM, SHORT, design_hash="probe",
                     params={"claw_taper_ratio": 0.6}, **WHEEL)
        self.assertTrue(all(r.params["claw_taper_ratio"] == 0.6 for r in out.records))

    @unittest.skipUnless(HAVE_STORE, "needs pyarrow and duckdb")
    def test_the_rows_go_straight_into_the_store(self):
        with tempfile.TemporaryDirectory() as tmp:
            store = ExperimentStore(Path(tmp))
            out = run_s1(PLATFORM, SHORT, design_hash="probe", **WHEEL)
            store.append(out.records)
            rows = store.query(
                "SELECT count(*) FROM runs WHERE scenario LIKE 'S1_step/%'")
            self.assertEqual(rows[0][0], len(out.records))
            # And the ladder comes back out of SQL, which is the point of storing it.
            heights = store.query(
                "SELECT DISTINCT CAST(params ->> 'step_height_m' AS DOUBLE) FROM runs "
                "ORDER BY 1")
            self.assertEqual([h for (h,) in heights], list(SHORT.heights_m))


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
