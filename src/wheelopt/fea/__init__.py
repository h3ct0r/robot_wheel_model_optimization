"""Offline FEA tier: STEP geometry -> CalculiX -> reduced-order model parameters.

This stage runs **outside** the optimisation loop (invariant 1, ADR-0002). It is slow,
cached, and per-design; its output is the stiffness data the segmented-ring ROM is fitted
to, which is what the inner loop actually simulates.

Layering mirrors ``wheelopt.cad``: everything that can be pure numpy or pure text is, and
the two modules that need a heavyweight external dependency are as thin as possible.

    results       typed results and failure statuses          pure
    hyperelastic  material model + literature coefficients    pure
    loadcase      what to simulate, and how finely            pure
    indenter      flat-plate and step-edge contact bodies     numpy
    deck          mesh + material + load case -> .inp text    pure
    parse         CalculiX .dat/.sta text -> arrays           pure
    extract       arrays -> k_r(delta), contact patch         numpy
    cache         cache-key composition                       pure
    mesh          STEP -> second-order tet mesh               needs gmsh
    runner        subprocess orchestration                    needs ccx

Only ``mesh`` requires gmsh and only ``runner`` requires the CalculiX binary; both are
imported lazily so that deck generation and result parsing are testable with neither.
"""

from __future__ import annotations

#: Bump whenever the FEA pipeline changes in a way that alters results for the same design:
#: deck structure, boundary conditions, output requests, extraction procedure. Composed into
#: every cache key alongside the CAD ``PIPELINE_VERSION`` — see invariant 5.
FEA_PIPELINE_VERSION = "fea-0.1.0"

# The step-4 (ROM) entry point and its result types. Importing this package pulls in only
# pure/numpy code; gmsh and the CalculiX binary are imported lazily, inside the functions
# that need them, so `import wheelopt.fea` works on a machine that can solve nothing.
from .loadcase import IndenterSpec, LoadCase, LoadCaseKind, MeshSpec, SolverSpec
from .results import ContactPatch, FeaResult, FeaStatus, LoadCurve
from .runner import find_ccx, run_load_case

__all__ = [
    "FEA_PIPELINE_VERSION",
    "ContactPatch",
    "FeaResult",
    "FeaStatus",
    "IndenterSpec",
    "LoadCase",
    "LoadCaseKind",
    "LoadCurve",
    "MeshSpec",
    "SolverSpec",
    "find_ccx",
    "run_load_case",
]
