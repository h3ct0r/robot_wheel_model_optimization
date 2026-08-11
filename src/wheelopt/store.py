"""The experiment store: one row per (design, scenario, seed, material realisation).

`docs/plan/13-engineering.md`: *"Questions will arise that haven't been thought of yet; don't
lose the data in log files."* That is the whole brief. This module is deliberately dumb — it
records what happened and answers SQL about it. It computes no metrics, aggregates nothing,
and decides nothing.

**Parquet files, read by DuckDB.** Not a single ``.duckdb`` file, and the reason is the
campaign rather than the query language. A campaign is many worker processes running for days
and expecting to be interrupted (`13-engineering.md` again), and DuckDB takes a single writer
lock on its own database file. Append-only Parquet gives every worker its own file, makes an
interrupted run cost at most the batch it was writing, and still gets full SQL over the lot
through ``read_parquet``. The trade is that a query opens N files instead of one; at the
~10^4 rows this project plans that is not a cost worth engineering around.

**Three JSON columns, and that is on purpose.** ``params``, ``metrics`` and ``diagnostics``
are stored as JSON text rather than exploded into columns. The metric set *will* change —
`08-metrics.md` is not final and `04-design-space.md` gains topologies — and a rigid schema
turns each of those into a migration, which in practice means old rows quietly get NULLs in
the new columns and nobody notices which runs predate the change. DuckDB reads into JSON
natively (``metrics ->> 'cost_of_transport'``), so the query cost is small and the schema
never lies about what a row actually recorded. Identity, provenance and status *are* columns,
because those are the things every row has had and will have.

**What makes a row unique is `run_id`, and it does not depend on the answer.** It hashes the
design, the scenario, the seed, the material realisation and every pipeline version — the
inputs, and nothing else. So the Phase 0 determinism gate is a query rather than a procedure:
group by ``run_id`` and ask whether the metrics agree. Two rows with one ``run_id`` and
different numbers is precisely the failure that gate exists to catch, and it cannot be
expressed at all if the outputs are in the key.

Nothing here imports duckdb or pyarrow at module scope. Building and validating records is
pure Python, so the record layer is testable on a machine with neither installed — the same
split `rom.ring` and `rom.mjcf` already use.
"""

from __future__ import annotations

import json
import os
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import Enum
from pathlib import Path
from typing import Any

from .hashing import content_digest, plain

__all__ = [
    "STORE_SCHEMA_VERSION",
    "ExperimentStore",
    "RunRecord",
    "RunStatus",
    "StoreError",
    "compare_manifests",
    "manifest_from_records",
    "pipeline_versions",
]

#: Layout of the Parquet rows. Bump when a *column* is added, removed or re-meaned — not when
#: a key inside `params`/`metrics`/`diagnostics` changes, which is the whole reason those are
#: JSON. It is written into every row so that a reader can tell which rows it understands
#: rather than inferring it from which columns happen to be present.
STORE_SCHEMA_VERSION = "store-0.1.0"


class StoreError(RuntimeError):
    """The store could not be read or written. Never raised for a *failed run*."""


class RunStatus(str, Enum):
    """What happened to one evaluation. Only ``OK`` carries trustworthy metrics.

    Invariant 4: a failed evaluation is a **row**, not an exception. A campaign that drops
    its failures cannot answer "which region of the space diverges", which is a result rather
    than an inconvenience — and a region that silently produces no rows looks like a region
    nobody sampled.

    Deliberately coarser than :class:`~wheelopt.fea.results.FeaStatus`. That enum distinguishes
    nine ways CalculiX can fail because the FEA driver has to act differently on each; a
    campaign only needs to know which *stage* lost the design, and the detail survives in
    ``message`` and in the FEA cache entry.
    """

    #: Ran to completion and the metrics mean what they say.
    OK = "ok"
    #: Rejected by the analytic pre-filter. Not a failure — the design was never built.
    SCREENED_OUT = "screened_out"
    #: The geometry stage could not produce a solid.
    CAD_FAILED = "cad_failed"
    #: Meshing, solving or extraction failed. See ``message`` for which.
    FEA_FAILED = "fea_failed"
    #: FEA succeeded but no ring could be built from the curve, or the fit missed its gate.
    ROM_FAILED = "rom_failed"
    #: The simulation ran and produced a non-finite state, or ended in a non-physical pose.
    SIM_DIVERGED = "sim_diverged"
    #: The simulation could not start: no MuJoCo, an invalid model, a bad scenario.
    SIM_FAILED = "sim_failed"


def pipeline_versions() -> dict[str, str]:
    """Every version string that can move a number, as one dict for the ``run_id``.

    Read from the modules that own them rather than restated here, so a bump lands in the
    store without anyone remembering to update the store. Invariant 5 in a second place: the
    FEA cache key already carries these, and a row whose ``run_id`` ignored them would claim
    two evaluations were the same run when the pipeline underneath had changed.
    """
    from .cad.export import PIPELINE_VERSION as CAD_VERSION
    from .fea import FEA_PIPELINE_VERSION
    from .rom import ROM_VERSION

    return {
        "cad": CAD_VERSION,
        "fea": FEA_PIPELINE_VERSION,
        "rom": ROM_VERSION,
        "store": STORE_SCHEMA_VERSION,
    }


def _utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


@dataclass(frozen=True, slots=True)
class RunRecord:
    """One evaluation of one design under one scenario, seed and material realisation.

    ``material_realisation`` is an index into the sampled material population, not a material
    name: invariant 7 scores every design over terrain seeds **times** material realisations
    and aggregates with CVaR, so the realisation is part of what identifies a row rather than
    part of what it measured.
    """

    design_hash: str
    scenario: str
    seed: int
    material_realisation: int
    status: RunStatus = RunStatus.OK
    message: str = ""
    #: The design and material, for reading rows back without re-deriving them. Not part of
    #: `run_id` — `design_hash` already stands for all of it, and duplicating it into the key
    #: would let a formatting change to a parameter invalidate a run that is identical.
    params: Mapping[str, Any] = field(default_factory=dict)
    metrics: Mapping[str, float] = field(default_factory=dict)
    #: Solver diagnostics: warning counts, max penetration, energy drift, FEA increments, ROM
    #: fit residual. `13-engineering.md` calls these the artifact detectors, and they are the
    #: reason a suspicious result can be interrogated months later instead of re-run.
    diagnostics: Mapping[str, float] = field(default_factory=dict)
    versions: Mapping[str, str] = field(default_factory=pipeline_versions)
    created_at: str = field(default_factory=_utc_now)

    def __post_init__(self) -> None:
        if not self.design_hash:
            raise ValueError("design_hash is required; it is what a row is about")
        if not self.scenario:
            raise ValueError("scenario is required")
        if self.seed < 0 or self.material_realisation < 0:
            raise ValueError("seed and material_realisation must be non-negative")
        if self.status is not RunStatus.OK and not self.message:
            raise ValueError(
                f"status {self.status.value} needs a message: a failed row whose cause was "
                "not written down is a row nobody can act on"
            )
        for name, value in self.metrics.items():
            if not isinstance(value, (int, float)):
                # ValueError rather than the TypeError ruff prefers, and deliberately: every
                # other check in this dataclass — and in `RingSpec`, `RoverSpec`, `LoadCase` —
                # raises ValueError, so a caller wrapping record construction in one `except
                # ValueError` would otherwise miss exactly this one.
                raise ValueError(  # noqa: TRY004
                    f"metric {name!r} is {type(value).__name__}, not a number"
                )

    @property
    def run_id(self) -> str:
        """Content hash of the **inputs**. Same inputs, same id — whatever came out.

        This is the determinism gate's handle. It covers design, scenario, seed, material
        realisation and every pipeline version, and covers no metric, no diagnostic and no
        timestamp. A run repeated on another machine two days later must land on this same
        id, and the gate is then a group-by rather than a bespoke comparison.
        """
        return content_digest({
            "design": self.design_hash,
            "scenario": self.scenario,
            "seed": self.seed,
            "material_realisation": self.material_realisation,
            "versions": dict(self.versions),
        })

    def as_row(self) -> dict[str, Any]:
        """Flatten to the Parquet column set. JSON for the three open-ended maps."""
        return {
            "run_id": self.run_id,
            "design_hash": self.design_hash,
            "scenario": self.scenario,
            "seed": int(self.seed),
            "material_realisation": int(self.material_realisation),
            "status": self.status.value,
            "message": self.message,
            "params": json.dumps(plain(dict(self.params)), sort_keys=True),
            "metrics": json.dumps(plain(dict(self.metrics)), sort_keys=True),
            "diagnostics": json.dumps(plain(dict(self.diagnostics)), sort_keys=True),
            "versions": json.dumps(dict(self.versions), sort_keys=True),
            "schema_version": STORE_SCHEMA_VERSION,
            "created_at": self.created_at,
        }


#: Column order and Parquet types. One place, so a writer and a reader cannot disagree.
_COLUMNS: tuple[tuple[str, str], ...] = (
    ("run_id", "string"),
    ("design_hash", "string"),
    ("scenario", "string"),
    ("seed", "int64"),
    ("material_realisation", "int64"),
    ("status", "string"),
    ("message", "string"),
    ("params", "string"),
    ("metrics", "string"),
    ("diagnostics", "string"),
    ("versions", "string"),
    ("schema_version", "string"),
    ("created_at", "string"),
)


class ExperimentStore:
    """Append-only Parquet under ``root``, queried with DuckDB.

    Writes go to ``root/runs/<timestamp>-<pid>-<n>.parquet``, one file per :meth:`append`.
    Reads glob the lot. Nothing is ever updated in place, so a reader is never looking at a
    half-written row and a crashed worker costs one batch.
    """

    def __init__(self, root: Path | str) -> None:
        self.root = Path(root)
        self.runs_dir = self.root / "runs"
        self._written = 0

    def __repr__(self) -> str:  # pragma: no cover - display only
        return f"ExperimentStore({str(self.root)!r})"

    # -- writing ---------------------------------------------------------------------

    def append(self, records: Iterable[RunRecord]) -> Path | None:
        """Write a batch. Returns the file written, or ``None`` for an empty batch.

        Written to a temporary name in the same directory and renamed into place, so a
        reader globbing ``*.parquet`` can never see a partial file. The FEA cache uses the
        same trick for the same reason.
        """
        rows = [r.as_row() for r in records]
        if not rows:
            return None
        pa, pq = _arrow()
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        self._written += 1
        stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%f")
        final = self.runs_dir / f"{stamp}-{os.getpid()}-{self._written}.parquet"
        staging = final.with_suffix(".parquet.tmp")
        table = pa.table(
            {name: [row[name] for row in rows] for name, _ in _COLUMNS},
            schema=pa.schema([(name, getattr(pa, kind)()) for name, kind in _COLUMNS]),
        )
        try:
            pq.write_table(table, staging)
            os.replace(staging, final)
        except OSError as exc:
            staging.unlink(missing_ok=True)
            raise StoreError(f"could not write {final}: {exc}") from exc
        return final

    # -- reading ---------------------------------------------------------------------

    @property
    def files(self) -> list[Path]:
        """Every batch file, oldest first. The names sort chronologically by construction."""
        return sorted(self.runs_dir.glob("*.parquet")) if self.runs_dir.is_dir() else []

    def query(self, sql: str, *, table: str = "runs") -> list[tuple]:
        """Run SQL against the store. ``table`` is the name the rows are bound to.

        Raises :class:`StoreError` when there is nothing to read, rather than returning an
        empty result: "no rows matched" and "the store you named does not exist" are
        different answers, and silently conflating them is how a campaign comes to be
        analysed against the wrong directory.
        """
        files = self.files
        if not files:
            raise StoreError(f"no runs under {self.runs_dir}; nothing has been appended")
        duckdb = _duckdb()
        connection = duckdb.connect()
        try:
            # The relation API rather than `CREATE VIEW ... read_parquet(?)`: DuckDB refuses
            # a prepared parameter inside a CREATE VIEW, and the obvious workaround —
            # interpolating the paths into the SQL — puts a filesystem path a user chose
            # inside a quoted string literal. Passing the list through the API keeps it data.
            connection.read_parquet([str(p) for p in files]).create_view(table)
            return connection.execute(sql).fetchall()
        except Exception as exc:  # duckdb raises its own hierarchy, not a subclass tree
            raise StoreError(f"query failed: {exc}") from exc
        finally:
            connection.close()

    def records(self) -> list[dict[str, Any]]:
        """Every row as a dict, with the four JSON columns decoded. Convenience, not SQL."""
        rows = self.query("SELECT * FROM runs ORDER BY created_at, run_id")
        names = [name for name, _ in _COLUMNS]
        out = []
        for row in rows:
            item = dict(zip(names, row))
            for key in ("params", "metrics", "diagnostics", "versions"):
                item[key] = json.loads(item[key])
            out.append(item)
        return out

    def disagreements(self, *, metrics: Sequence[str] | None = None) -> list[tuple]:
        """Rows sharing a ``run_id`` whose metrics differ. **The Phase 0 determinism gate.**

        Identical inputs must give an identical score. Because ``run_id`` hashes the inputs
        and nothing else, "the same evaluation done twice" is exactly "two rows with one
        ``run_id``", and the gate is whether their ``metrics`` JSON is byte-identical.

        Empty means every repeated evaluation agreed — including the case where nothing was
        repeated, which is *not* the same as passing and is why
        :func:`~wheelopt.store.ExperimentStore.repeat_counts` exists next to it.

        Args:
            metrics: compare only these keys. ``None`` compares the whole metrics map, which
                is stricter and is what the gate should use — a diagnostic that drifts is
                still a machine that is not reproducing.
        """
        if metrics is None:
            expression = "metrics"
        else:
            # Doubled quotes, because a metric name is a caller's string and this one has to
            # land inside a SQL literal. `->>` takes no bind parameter in an aggregate here.
            parts = ", ".join(f"""metrics ->> '{name.replace("'", "''")}'"""
                              for name in metrics)
            expression = f"concat_ws('|', {parts})"
        return self.query(
            "SELECT run_id, count(DISTINCT " + expression + ") AS variants, "
            "count(*) AS rows FROM runs WHERE status = 'ok' GROUP BY run_id "
            "HAVING count(DISTINCT " + expression + ") > 1 ORDER BY run_id"
        )

    def repeat_counts(self) -> list[tuple]:
        """``(run_id, n)`` for every evaluation recorded more than once.

        The companion to :meth:`disagreements`, and the reason the gate needs both: an empty
        disagreement list proves nothing if nothing was ever repeated. A determinism gate
        that passes because the campaign never ran a design twice is the quiet-wrong-answer
        failure this project keeps finding.
        """
        return self.query(
            "SELECT run_id, count(*) AS n FROM runs WHERE status = 'ok' "
            "GROUP BY run_id HAVING count(*) > 1 ORDER BY n DESC, run_id"
        )


def manifest_from_records(records: Iterable[RunRecord]) -> dict[str, Any]:
    """``run_id -> metrics`` for a batch of runs, as one JSON-serialisable dict.

    The cross-machine half of the Phase 0 determinism gate (`11-phases.md`: *identical θ →
    identical score on two machines, two days apart*). The store's own gate proves one
    machine agrees with itself; a manifest is that claim made portable — machine A commits
    what it measured, machine B re-runs the same ladder and compares. Pure records-in,
    dict-out, so the comparing side needs neither DuckDB nor pyarrow.

    Metrics only, not diagnostics: diagnostics carry wall-times and iteration counts, which
    are honest machine-to-machine differences and not what the gate is about. Failed runs are
    included with their status — a run that diverges on one machine and succeeds on the other
    is the loudest possible disagreement, and dropping failures would hide exactly that.

    Floats survive the JSON round trip bit-for-bit: ``json`` serialises them with ``repr``,
    which is exact for float64, so equality below is equality of the numbers and not of a
    formatting.
    """
    rows: dict[str, Any] = {}
    versions: dict[str, str] = {}
    for record in records:
        rows[record.run_id] = {
            "status": record.status.value,
            "metrics": dict(sorted(record.metrics.items())),
        }
        versions.update(record.versions)
    return {"schema": STORE_SCHEMA_VERSION, "versions": versions, "rows": rows}


def compare_manifests(reference: Mapping[str, Any],
                      candidate: Mapping[str, Any]) -> list[str]:
    """Every way ``candidate`` disagrees with ``reference``, as human-readable lines.

    Empty means the gate passed. Ordered so the most structural problem is named first: a
    version skew explains every numeric difference after it, so reporting the numbers first
    would send someone debugging arithmetic that was never run on the same code.
    """
    problems: list[str] = []
    for key, ref in sorted(reference.get("versions", {}).items()):
        got = candidate.get("versions", {}).get(key)
        if got != ref:
            problems.append(f"version skew: {key} is {got!r} against the reference {ref!r}")
    ref_rows = reference.get("rows", {})
    cand_rows = candidate.get("rows", {})
    for run_id in sorted(set(ref_rows) - set(cand_rows)):
        problems.append(f"missing run: {run_id} was never evaluated")
    for run_id in sorted(set(cand_rows) - set(ref_rows)):
        problems.append(f"extra run: {run_id} is not in the reference — the inputs differ, "
                        "so this is two different experiments, not one repeated")
    for run_id in sorted(set(ref_rows) & set(cand_rows)):
        ref_row, cand_row = ref_rows[run_id], cand_rows[run_id]
        if ref_row["status"] != cand_row["status"]:
            problems.append(f"{run_id}: status {cand_row['status']!r} against "
                            f"{ref_row['status']!r}")
            continue
        for name in sorted(set(ref_row["metrics"]) | set(cand_row["metrics"])):
            a = ref_row["metrics"].get(name)
            b = cand_row["metrics"].get(name)
            if a != b:
                problems.append(f"{run_id}: {name} = {b!r} against {a!r}")
    return problems


def _arrow():
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise StoreError(
            "writing the store needs pyarrow: pip install -e '.[store]'"
        ) from exc
    return pa, pq


def _duckdb():
    try:
        import duckdb
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise StoreError(
            "querying the store needs duckdb: pip install -e '.[store]'"
        ) from exc
    return duckdb
