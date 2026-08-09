"""Read CalculiX text output. Pure — takes strings, returns arrays. Never raises on
malformed input; returns what it could find and lets the caller decide.

``.dat`` is the channel this pipeline is built on: plain text, one block per requested
output per time point, and therefore capturable as a test fixture. ``.frd`` is a binary-ish
field format for CalculiX GraphiX; it is written for debugging and visualisation only, and
nothing here depends on it.

A ``.dat`` block looks like::

    forces (fx,fy,fz) for set NREF and time  0.5000000E+00

         4231  1.234568E-14  2.345678E+01 -3.456789E-15

The header line names the quantity and the time; the rows are node id followed by
components. Header wording varies between CalculiX versions, so matching is on distinctive
substrings rather than exact text.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

import numpy as np

__all__ = ["DatBlock", "parse_dat", "parse_sta", "StaSummary", "collect"]

_TIME_RE = re.compile(r"time\s*[= ]\s*([-+0-9.eEdD]+)")
_NUMBER_RE = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eEdD][-+]?\d+)?")
_COMPONENTS_RE = re.compile(r"\(([^)]*)\)")

#: Header component names that identify a row rather than carry a value.
#:
#: CalculiX uses three different leading-column conventions in one file, and the header is
#: the only thing that distinguishes them::
#:
#:     stresses (elem, integ.pnt.,sxx,...)      2 ids: element and integration point
#:     displacements (vx,vy,vz) for set NREF    1 id:  the node number, unnamed
#:     contact stress (slave node,press,...)    1 id:  named in the component list
#:     total force (fx,fy,fz) for set NREF      0 ids: TOTALS=ONLY prints a bare sum
#:
#: So value columns are counted from the header and taken from the *end* of each row.
#: Without that, a stress block silently yields the integration-point index where sxx
#: should be — which reads as a small, entirely plausible stress.
_ID_COMPONENTS = ("elem", "integ", "node", "set", "slave")


def _to_float(token: str) -> float:
    """CalculiX writes Fortran D-exponents in some builds."""
    try:
        return float(token.replace("D", "E").replace("d", "e"))
    except ValueError:
        return float("nan")


@dataclass(frozen=True, slots=True)
class DatBlock:
    """One printed result block: a quantity, at a time, over a set of entities."""

    quantity: str
    time: float
    header: str
    #: (n,) entity ids — node or element numbers.
    ids: np.ndarray
    #: (n, k) component values, with any leading id columns already stripped.
    values: np.ndarray
    #: (n,) integration-point indices for element output; empty for nodal output.
    sub_ids: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int64))
    #: Component names taken from the header, e.g. ``("sxx", "syy", ...)``.
    components: tuple[str, ...] = ()


def _value_component_count(header: str) -> int | None:
    """How many value columns the header advertises, or ``None`` if it does not say."""
    match = _COMPONENTS_RE.search(header)
    if not match:
        return None
    names = [n.strip().lower() for n in match.group(1).split(",") if n.strip()]
    values = [n for n in names if not any(n.startswith(p) for p in _ID_COMPONENTS)]
    return len(values) or None


def _component_names(header: str) -> tuple[str, ...]:
    match = _COMPONENTS_RE.search(header)
    if not match:
        return ()
    names = [n.strip().lower() for n in match.group(1).split(",") if n.strip()]
    return tuple(n for n in names if not any(n.startswith(p) for p in _ID_COMPONENTS))


def _classify(header: str) -> str:
    """Map a CalculiX block header onto a stable internal name.

    **Contact blocks are matched first, and that ordering is load-bearing.** CalculiX names
    one of them ``relative contact displacement (slave node,normal,tang1,tang2)``; matching
    on ``displacement`` before ``contact`` classifies it as nodal displacement, where it
    then overwrites the reference-node history at the same time value and the load curve
    silently comes out empty.
    """
    h = header.lower()
    if "contact" in h:
        if "stress" in h:
            return "contact_stress"
        if "displacement" in h or "relative" in h:
            return "contact_displacement"
        if "element" in h or "number" in h:
            return "contact_count"
        return "contact_other"
    if "force" in h:
        return "total_force" if "total" in h else "force"
    if "displacement" in h:
        return "displacement"
    if "stress" in h:
        return "stress"
    if "strain" in h:
        return "strain"
    return h.strip()[:40] or "unknown"


def parse_dat(text: str) -> list[DatBlock]:
    """Parse a whole ``.dat`` file into blocks. Unrecognised content is skipped."""
    blocks: list[DatBlock] = []
    header: str | None = None
    time = float("nan")
    rows: list[list[float]] = []

    def flush() -> None:
        nonlocal header, rows
        if header is not None and rows:
            width = max(len(r) for r in rows)
            padded = np.array(
                [r + [float("nan")] * (width - len(r)) for r in rows], dtype=np.float64
            )
            # Value columns are counted from the header and taken from the end of the row.
            # n_ids is therefore 0, 1 or 2 depending on the block — a TOTALS=ONLY force
            # block carries no id at all, so clamping to 1 would consume fx as the id and
            # silently drop fz.
            n_values = _value_component_count(header)
            n_ids = max(0, width - n_values) if n_values else 1
            blocks.append(
                DatBlock(
                    quantity=_classify(header),
                    time=time,
                    header=header.strip(),
                    ids=(
                        padded[:, 0].astype(np.int64)
                        if n_ids >= 1
                        else np.arange(len(padded), dtype=np.int64)
                    ),
                    sub_ids=(
                        padded[:, 1].astype(np.int64)
                        if n_ids > 1
                        else np.zeros(0, dtype=np.int64)
                    ),
                    values=padded[:, n_ids:],
                    components=_component_names(header),
                )
            )
        header, rows = None, []

    for raw in text.splitlines():
        line = raw.rstrip()
        if not line.strip():
            continue
        stripped = line.strip()
        starts_with_number = stripped[0].isdigit() or stripped[0] in "+-."

        if not starts_with_number:
            flush()
            header = line
            m = _TIME_RE.search(line)
            time = _to_float(m.group(1)) if m else float("nan")
            continue

        if header is None:
            continue
        tokens = _NUMBER_RE.findall(stripped)
        if len(tokens) < 2:
            continue
        rows.append([_to_float(t) for t in tokens])

    flush()
    return blocks


@dataclass(frozen=True, slots=True)
class StaSummary:
    """Increment history from ``.sta``. Pipeline health, per ADR-0005."""

    n_increments: int = 0
    n_cutbacks: int = 0
    final_time: float = 0.0
    attempts: list[tuple[int, float]] = field(default_factory=list)


def parse_sta(text: str) -> StaSummary:
    """Parse the ``.sta`` increment table.

    Columns are step, increment, attempt, iterations, and the accumulated and incremental
    step time. An attempt number above 1 means the increment was cut back and retried,
    which is the honest measure of how hard a design was to solve.
    """
    increments = 0
    cutbacks = 0
    final_time = 0.0
    attempts: list[tuple[int, float]] = []

    for raw in text.splitlines():
        stripped = raw.strip()
        if not stripped or not (stripped[0].isdigit() or stripped[0] in "+-."):
            continue
        tokens = _NUMBER_RE.findall(stripped)
        if len(tokens) < 5:
            continue
        try:
            attempt = int(_to_float(tokens[2]))
            step_time = _to_float(tokens[4])
        except (ValueError, IndexError):
            continue
        increments += 1
        if attempt > 1:
            cutbacks += 1
        if np.isfinite(step_time):
            final_time = max(final_time, step_time)
            attempts.append((attempt, step_time))

    return StaSummary(
        n_increments=increments,
        n_cutbacks=cutbacks,
        final_time=final_time,
        attempts=attempts,
    )


def collect(blocks: list[DatBlock], quantity: str) -> dict[float, DatBlock]:
    """Index blocks of one quantity by time. Later blocks win on duplicate times."""
    return {b.time: b for b in blocks if b.quantity == quantity and np.isfinite(b.time)}
