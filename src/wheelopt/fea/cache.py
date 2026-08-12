"""Cache-key composition and on-disk layout for FEA results.

Invariant 5 (CLAUDE.md): *every cache key includes the pipeline version and the ROM
version*. Changing ring discretisation, fitting procedure or material homogenisation must
invalidate prior results. This module is where that is enforced for the FEA stage.

The key covers everything that can change the numbers:

    CAD pipeline version    geometry generation changed
    FEA pipeline version    deck, BCs, output requests or extraction changed
    design hash             the wheel itself
    material payload        density, infill, walls
    hyperelastic digest     the actual coefficients, not the table name
    load case               what is being pressed into what, and how far
    mesh spec               discretisation
    solver identity         ccx and gmsh versions

Deliberately *not* in the key: output directory, thread count, timeout. Those change how
long the answer takes, not what it is. Every *other* solver setting is in the key — see
:data:`SOLVER_TIMING_ONLY`. They were once all excluded under the same reasoning, which was
wrong for contact stiffness and the increment controls.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ..cad.export import PIPELINE_VERSION as CAD_PIPELINE_VERSION
from ..cad.materials import MaterialSpec
from ..cad.params import WheelParams
from ..hashing import content_digest
from ..hashing import plain as _plain
from . import FEA_PIPELINE_VERSION
from .hyperelastic import HyperelasticModel
from .loadcase import LoadCase, MeshSpec, SolverSpec

__all__ = ["SOLVER_TIMING_ONLY", "SOLVER_UNKNOWN", "cache_dir_for", "fea_cache_key"]

#: Placeholder solver identity used when no binary is present. Deck generation and key
#: computation must work without CalculiX installed — that is what makes the unit tests
#: meaningful on a machine that cannot solve anything. Results computed under this identity
#: are never *stored*; see :mod:`wheelopt.fea.runner`.
SOLVER_UNKNOWN = "solver-unknown"


#: ``SolverSpec`` fields that genuinely only change how long the answer takes. Everything
#: else in that dataclass goes into the key. Named as an exclusion list rather than an
#: inclusion list on purpose: a field added later is hashed by default, so the failure mode
#: of forgetting to update this is a redundant cache miss, not a silently shared entry.
SOLVER_TIMING_ONLY = frozenset({"timeout_s", "n_threads"})


def _solver_physics(solver: SolverSpec | None) -> dict[str, Any]:
    """The part of a ``SolverSpec`` that changes the numbers."""
    if solver is None:
        return {}
    return {
        k: v for k, v in _plain(solver).items() if k not in SOLVER_TIMING_ONLY
    }


def fea_cache_key(
    params: WheelParams,
    material: MaterialSpec,
    hyper: HyperelasticModel,
    load_case: LoadCase,
    mesh_spec: MeshSpec,
    solver_identity: str = SOLVER_UNKNOWN,
    solver: SolverSpec | None = None,
) -> str:
    """Content hash for one FEA evaluation. 16 hex characters.

    Stable across processes and dict insertion order: every nested structure is sorted
    before serialisation.

    ``solver`` carries the *settings*; ``solver_identity`` carries the binary's version.
    Both matter. The settings were originally left out on the reasoning that they "change
    how long the answer takes, not what it is" — true of the timeout and the thread count,
    and false of everything else in there. ``contact_stiffness_factor`` is the contact
    compliance: measured 2026-08-08, changing it from 20 to 5 turned a diverged plane-strain
    run into a converged one, and because all three factors hashed identically the second
    run was served the first one's result from cache. The increment controls decide whether
    a limit point is found at all, and ``min_increment`` decides whether the run completes
    or fails. None of that is timing.
    """
    payload = {
        "cad_pipeline": CAD_PIPELINE_VERSION,
        "fea_pipeline": FEA_PIPELINE_VERSION,
        "design": params.design_hash(),
        "material": _plain(material),
        "hyperelastic": hyper.coefficient_digest(),
        "load_case": _plain(load_case),
        "mesh": _plain(mesh_spec),
        "solver": solver_identity,
        "solver_settings": _solver_physics(solver),
    }
    return content_digest(payload)


def cache_dir_for(cache_root: Path, key: str) -> Path:
    """Directory holding one evaluation's artefacts.

    Layout, all inside ``data/cache/fea/<key>/``::

        job.inp           the deck, exactly as solved
        job.dat           printed results
        job.sta           increment history
        job.frd           field output, for CalculiX GraphiX only
        ccx.stdout.log    solver output
        mesh.json.gz      the mesh, so a re-parse needs no gmsh
        meta.json         versions, solver identity, timing
        result.json       the parsed FeaResult

    Written to ``<key>.tmp-<pid>/`` and moved into place atomically on success, so an
    interrupted campaign — and campaigns run for days, so they will be interrupted — cannot
    leave a half-written entry that later looks like a cache hit.
    """
    return Path(cache_root) / key
