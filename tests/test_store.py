"""The experiment store, and the determinism gate it exists to make expressible.

Two tiers, as everywhere else in this repo: building and hashing a record is pure Python and
always runs; writing Parquet and querying it needs pyarrow and duckdb and is skipped without
them. The pure tier is where the properties that matter live — what `run_id` covers, and more
importantly what it must *not* cover.
"""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from wheelopt.store import (
    STORE_SCHEMA_VERSION,
    ExperimentStore,
    RunRecord,
    RunStatus,
    StoreError,
    compare_manifests,
    manifest_from_records,
    pipeline_versions,
)

HAVE_ARROW = importlib.util.find_spec("pyarrow") is not None
HAVE_DUCKDB = importlib.util.find_spec("duckdb") is not None
HAVE_STORE = HAVE_ARROW and HAVE_DUCKDB


def a_record(**kwargs) -> RunRecord:
    base = {
        "design_hash": "abc123",
        "scenario": "S1_step",
        "seed": 0,
        "material_realisation": 0,
        "metrics": {"climb_height_m": 0.03, "cost_of_transport": 0.0748},
    }
    return RunRecord(**{**base, **kwargs})


class TestRunId(unittest.TestCase):
    """What the identity covers, and what it must not."""

    def test_the_same_inputs_give_the_same_id(self):
        self.assertEqual(a_record().run_id, a_record().run_id)

    def test_the_outputs_are_not_in_it(self):
        """The property the whole determinism gate rests on. If metrics were in the key,
        two runs that disagreed would land on two different ids and the gate would be
        checking that a set of unique values has no duplicates — vacuously true, forever."""
        quiet = a_record(metrics={"climb_height_m": 0.03})
        loud = a_record(metrics={"climb_height_m": 0.90})
        self.assertEqual(quiet.run_id, loud.run_id)

    def test_neither_is_the_timestamp_or_the_message(self):
        self.assertEqual(a_record(created_at="1999-01-01T00:00:00.000+00:00").run_id,
                         a_record(created_at="2026-08-10T12:00:00.000+00:00").run_id)
        self.assertEqual(a_record().run_id,
                         a_record(diagnostics={"energy_drift_j": 1e9}).run_id)

    def test_every_identity_field_changes_it(self):
        base = a_record()
        for field_name, value in (("design_hash", "def456"), ("scenario", "S2_ramp"),
                                  ("seed", 1), ("material_realisation", 1)):
            with self.subTest(field=field_name):
                self.assertNotEqual(base.run_id, replace(base, **{field_name: value}).run_id)

    def test_a_pipeline_bump_changes_it(self):
        """Invariant 5 in the store. A ROM version bump must not let a stale row pass for a
        fresh one -- and `rom-0.6.0` is exactly such a bump: the same design now yields a
        different segment law, and with it a 30 mm climb where 0.5.0 gave 60."""
        base = a_record()
        moved = replace(base, versions={**base.versions, "rom": "rom-99.0.0"})
        self.assertNotEqual(base.run_id, moved.run_id)

    def test_the_versions_come_from_the_modules_that_own_them(self):
        from wheelopt.rom import ROM_VERSION

        self.assertEqual(pipeline_versions()["rom"], ROM_VERSION)
        self.assertEqual(pipeline_versions()["store"], STORE_SCHEMA_VERSION)


class TestRecordValidation(unittest.TestCase):
    def test_a_failed_row_must_say_why(self):
        """Invariant 4 makes failures rows rather than exceptions, which only helps if the
        row carries the cause. A silent `sim_diverged` is a design nobody can act on."""
        with self.assertRaises(ValueError) as ctx:
            a_record(status=RunStatus.SIM_DIVERGED)
        self.assertIn("message", str(ctx.exception))
        a_record(status=RunStatus.SIM_DIVERGED, message="qacc went non-finite at t=0.31")

    def test_a_screened_out_design_is_a_row_and_not_an_error(self):
        record = a_record(status=RunStatus.SCREENED_OUT, message="spoke_min_wall",
                          metrics={})
        self.assertEqual(record.status, RunStatus.SCREENED_OUT)

    def test_a_metric_that_is_not_a_number_is_refused(self):
        """A metric arriving as a string reads fine in a print and silently becomes a text
        column that no aggregation can touch."""
        with self.assertRaises(ValueError):
            a_record(metrics={"climb_height_m": "0.03"})

    def test_identity_fields_are_required(self):
        for kwargs in ({"design_hash": ""}, {"scenario": ""}, {"seed": -1},
                       {"material_realisation": -1}):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                a_record(**kwargs)


class TestRow(unittest.TestCase):
    def test_the_open_ended_maps_become_sorted_json(self):
        """Sorted so that two rows recording the same thing are byte-identical -- which is
        what `disagreements` compares. Insertion order is not a difference in the data."""
        one = a_record(metrics={"a": 1.0, "b": 2.0}).as_row()["metrics"]
        other = a_record(metrics={"b": 2.0, "a": 1.0}).as_row()["metrics"]
        self.assertEqual(one, other)
        self.assertEqual(json.loads(one), {"a": 1.0, "b": 2.0})

    def test_the_row_carries_its_schema_version(self):
        self.assertEqual(a_record().as_row()["schema_version"], STORE_SCHEMA_VERSION)


@unittest.skipUnless(HAVE_STORE, "needs pyarrow and duckdb")
class TestRoundTrip(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = ExperimentStore(Path(self.tmp.name))

    def tearDown(self):
        self.tmp.cleanup()

    def test_what_goes_in_comes_out(self):
        self.store.append([a_record(), a_record(seed=1)])
        rows = self.store.records()
        self.assertEqual(len(rows), 2)
        self.assertEqual({r["seed"] for r in rows}, {0, 1})
        self.assertEqual(rows[0]["metrics"]["cost_of_transport"], 0.0748)

    def test_an_empty_batch_writes_nothing(self):
        self.assertIsNone(self.store.append([]))
        self.assertEqual(self.store.files, [])

    def test_batches_accumulate_rather_than_overwrite(self):
        for seed in range(3):
            self.store.append([a_record(seed=seed)])
        self.assertEqual(len(self.store.files), 3)
        self.assertEqual(len(self.store.records()), 3)

    def test_no_partial_file_is_ever_visible(self):
        """Written to `.parquet.tmp` and renamed, so a reader globbing `*.parquet` cannot
        pick up a half-written batch from a worker that was killed mid-write."""
        self.store.append([a_record()])
        self.assertEqual(list(self.store.runs_dir.glob("*.tmp")), [])

    def test_sql_reaches_into_the_json(self):
        self.store.append([a_record(), a_record(seed=1, metrics={"climb_height_m": 0.06,
                                                                 "cost_of_transport": 0.1})])
        rows = self.store.query(
            "SELECT seed FROM runs WHERE CAST(metrics ->> 'climb_height_m' AS DOUBLE) > 0.05"
        )
        self.assertEqual(rows, [(1,)])

    def test_querying_an_empty_store_is_an_error_and_not_an_empty_list(self):
        """"Nothing matched" and "you are pointed at the wrong directory" are different
        answers, and a campaign analysed against the wrong path returns the first."""
        with self.assertRaises(StoreError):
            self.store.query("SELECT * FROM runs")

    def test_a_bad_query_is_a_StoreError_and_not_a_duckdb_exception(self):
        self.store.append([a_record()])
        with self.assertRaises(StoreError):
            self.store.query("SELECT nonexistent_column FROM runs")


@unittest.skipUnless(HAVE_STORE, "needs pyarrow and duckdb")
class TestDeterminismGate(unittest.TestCase):
    """The Phase 0 gate: identical theta -> identical score, two machines, two days apart."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = ExperimentStore(Path(self.tmp.name))

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_repeated_run_that_agrees_reports_nothing(self):
        self.store.append([a_record(), a_record(created_at="2026-08-12T09:00:00.000+00:00")])
        self.assertEqual(self.store.disagreements(), [])
        self.assertEqual(self.store.repeat_counts(), [(a_record().run_id, 2)])

    def test_a_repeated_run_that_disagrees_is_caught(self):
        self.store.append([
            a_record(),
            a_record(metrics={"climb_height_m": 0.03, "cost_of_transport": 0.0749}),
        ])
        caught = self.store.disagreements()
        self.assertEqual(len(caught), 1)
        self.assertEqual(caught[0][0], a_record().run_id)
        self.assertEqual(caught[0][1], 2)

    def test_it_can_be_narrowed_to_the_metrics_that_matter(self):
        """A campaign may knowingly tolerate drift in a timing-like metric while requiring
        bit-equality of the physical ones. The default is still the whole map."""
        self.store.append([
            a_record(metrics={"climb_height_m": 0.03, "wall_clock_s": 12.0}),
            a_record(metrics={"climb_height_m": 0.03, "wall_clock_s": 13.5}),
        ])
        self.assertEqual(len(self.store.disagreements()), 1)
        self.assertEqual(self.store.disagreements(metrics=["climb_height_m"]), [])

    def test_different_seeds_are_not_a_disagreement(self):
        """Two seeds are two experiments. Only a repeated `run_id` is a repeat."""
        self.store.append([
            a_record(seed=0, metrics={"climb_height_m": 0.03}),
            a_record(seed=1, metrics={"climb_height_m": 0.06}),
        ])
        self.assertEqual(self.store.disagreements(), [])
        self.assertEqual(self.store.repeat_counts(), [])

    def test_a_gate_with_nothing_repeated_is_not_a_gate_that_passed(self):
        """The failure the two methods exist to separate. `disagreements` is empty here, and
        that proves nothing at all -- no evaluation was ever run twice."""
        self.store.append([a_record(seed=0), a_record(seed=1)])
        self.assertEqual(self.store.disagreements(), [])
        self.assertEqual(self.store.repeat_counts(), [])

    def test_failed_runs_do_not_enter_the_gate(self):
        """A diverged run has no trustworthy metrics, so comparing two of them says nothing
        about determinism. They are still stored -- which region diverges is a result."""
        self.store.append([
            a_record(status=RunStatus.SIM_DIVERGED, message="a", metrics={"x": 1.0}),
            a_record(status=RunStatus.SIM_DIVERGED, message="b", metrics={"x": 2.0}),
        ])
        self.assertEqual(self.store.disagreements(), [])
        self.assertEqual(len(self.store.records()), 2)


class TestManifest(unittest.TestCase):
    """The cross-machine half of the determinism gate: records -> JSON -> comparison."""

    def rows(self, energy: float = 12.5) -> list[RunRecord]:
        return [RunRecord(design_hash="d1", scenario="S1_step/h=0.040@abcd1234", seed=s,
                          material_realisation=0,
                          metrics={"climbed": 1.0, "energy_j": energy})
                for s in (0, 1)]

    def test_a_manifest_round_trips_floats_bit_for_bit(self):
        """`json` writes float64 with repr, which is exact — so equality on the far side is
        equality of numbers, not of formatting. The value is chosen to be awkward."""
        import json

        awkward = 0.1 + 0.2   # 0.30000000000000004
        manifest = manifest_from_records(self.rows(energy=awkward))
        back = json.loads(json.dumps(manifest))
        self.assertEqual(compare_manifests(manifest, back), [])
        run_id = next(iter(back["rows"]))
        self.assertEqual(back["rows"][run_id]["metrics"]["energy_j"], awkward)

    def test_identical_records_agree_and_a_changed_metric_is_named(self):
        reference = manifest_from_records(self.rows())
        self.assertEqual(compare_manifests(reference, manifest_from_records(self.rows())), [])
        problems = compare_manifests(reference, manifest_from_records(self.rows(energy=13.0)))
        self.assertTrue(problems)
        self.assertTrue(all("energy_j" in line for line in problems), problems)

    def test_a_version_skew_is_reported_first(self):
        """A version skew explains every numeric difference after it; reporting the numbers
        first sends someone debugging arithmetic that never ran on the same code."""
        reference = manifest_from_records(self.rows())
        candidate = manifest_from_records(self.rows(energy=99.0))
        candidate = {**candidate, "versions": {**candidate["versions"], "rom": "rom-9.9.9"}}
        problems = compare_manifests(reference, candidate)
        self.assertIn("version skew", problems[0])

    def test_missing_and_extra_runs_are_two_experiments_not_nondeterminism(self):
        """What the gate caught on its first day: a ladder at a different duration produced
        the same run_ids with different numbers, because duration was not in the key. With
        the key fixed, a changed input reads as missing+extra runs — a different experiment —
        which is the honest description."""
        reference = manifest_from_records(self.rows())
        other = [RunRecord(design_hash="d1", scenario="S1_step/h=0.040@ffff0000", seed=0,
                           material_realisation=0, metrics={"climbed": 1.0})]
        problems = compare_manifests(reference, manifest_from_records(other))
        self.assertTrue(any("missing run" in x for x in problems))
        self.assertTrue(any("extra run" in x for x in problems))

    def test_a_run_that_fails_on_one_machine_only_is_the_loudest_disagreement(self):
        reference = manifest_from_records(self.rows())
        failed = [RunRecord(design_hash="d1", scenario="S1_step/h=0.040@abcd1234", seed=s,
                            material_realisation=0, status=RunStatus.SIM_FAILED,
                            message="diverged")
                  for s in (0, 1)]
        problems = compare_manifests(reference, manifest_from_records(failed))
        self.assertTrue(any("status" in x for x in problems), problems)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
