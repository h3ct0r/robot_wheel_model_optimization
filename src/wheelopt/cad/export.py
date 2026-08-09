"""Export geometry to STEP and STL.

STEP is the source of truth: FEA needs BREP, and so does anything downstream that cares
about exact surfaces or material region tagging (ADR-0003). STL is a derived,
lossy artefact for the simulator's collision and visual meshes and is never authoritative.

Filenames are content-addressed by design hash so that the cache and the artefacts on disk
cannot disagree about which geometry is which.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from .params import WheelParams

if TYPE_CHECKING:  # pragma: no cover
    from build123d import Part

__all__ = [
    "ExportPaths",
    "export",
    "is_watertight",
    "weld_vertices",
    "remesh",
    "ANGULAR_TOLERANCE_RAD",
    "PIPELINE_VERSION",
]

#: Vertices closer than this (metres) are the same vertex. Five orders of magnitude below
#: the finest tessellation tolerance in use, and nine above double-precision noise on
#: 0.1 m coordinates, so it can only ever merge points OCCT already considers coincident.
WELD_TOLERANCE_M = 1e-9

#: Default maximum angular deviation between adjacent facet normals, radians. Matches
#: OCCT's own default. On cylindrical geometry this, not the chordal tolerance, is what
#: sets facet count — see :func:`remesh`.
ANGULAR_TOLERANCE_RAD = 0.1


def remesh(
    part: "Part",
    tolerance_mm: float,
    angular_tolerance_rad: float = ANGULAR_TOLERANCE_RAD,
) -> None:
    """Discard any existing triangulation and mesh the shape to an **absolute** tolerance.

    Every OCCT entry point that produces triangles — ``Shape.tessellate``, ``export_stl`` —
    goes through ``BRepMesh_IncrementalMesh``, and build123d calls it with ``isRelative=True``
    and no prior clean. Two consequences, both silent:

    1. **The linear tolerance is scaled by feature size**, which on this geometry made it
       inert: 0.4 mm and 0.025 mm produced the identical mesh.
    2. **An existing triangulation is reused.** OCCT's boolean operations leave one behind,
       so the mesh reflected the union's internal deflection, not the caller's request — and
       the exported STL changed depending on whether anything had tessellated the part
       earlier in the process. Same design, different collision mesh, by call order.

    Meshing is a mutation of the shape, so this is a command, not a query: call it, then read
    the triangulation off the faces or hand the shape to a writer.
    """
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools

    BRepTools.Clean_s(part.wrapped)
    BRepMesh_IncrementalMesh(part.wrapped, tolerance_mm, False, angular_tolerance_rad, True)

#: Bump whenever geometry generation changes in a way that alters output for the same
#: design vector. Composed into cache keys — see invariant 5.
PIPELINE_VERSION = "cad-0.1.0"


@dataclass(frozen=True, slots=True)
class ExportPaths:
    step: Path
    stl: Path
    stem: str


def weld_vertices(
    vertices: np.ndarray,
    faces: np.ndarray,
    tolerance_m: float = WELD_TOLERANCE_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Merge geometrically coincident vertices and reindex the faces.

    A BREP tessellator meshes each face independently, so vertices along a shared edge are
    emitted once per adjacent face. The result is geometrically closed but topologically
    shredded: roughly 60% of the vertices of a typical wheel are duplicates, no edge is
    shared, and :func:`is_watertight` — correctly — rejects it. Anything that reasons about
    connectivity rather than pure geometry (convex decomposition, the manifold check, STL
    consumers) needs the welded form.

    Mass properties are unaffected: the divergence-theorem integral sums per-triangle
    contributions and never consults connectivity.

    Args:
        vertices: ``(n, 3)`` positions in metres.
        faces: ``(m, 3)`` indices into ``vertices``.
        tolerance_m: coordinates are quantised onto a grid of this size before matching.

    Returns:
        ``(welded_vertices, reindexed_faces)``. Deterministic: the output vertex order is
        the lexicographic order of the quantised positions.
    """
    v = np.asarray(vertices, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int64)
    if len(v) == 0 or len(f) == 0:
        return v, f

    quantised = np.round(v / tolerance_m).astype(np.int64)
    _, first_index, inverse = np.unique(
        quantised, axis=0, return_index=True, return_inverse=True
    )
    # Keep the original coordinates of a representative vertex rather than the quantised
    # ones, so welding never perturbs geometry.
    welded = v[first_index]
    return welded, inverse.reshape(-1)[f]


def is_watertight(faces: np.ndarray) -> tuple[bool, int]:
    """Check that every edge in a triangle mesh is shared by exactly two faces.

    A leaky mesh produces silently wrong contact in the simulator — the failure is not an
    error, it is a plausible-looking wrong answer, which is far worse. Screening for it
    here is cheap.

    Returns:
        ``(watertight, n_bad_edges)``.
    """
    f = np.asarray(faces, dtype=np.int64)
    if len(f) == 0:
        return False, 0

    edges = np.vstack([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
    edges = np.sort(edges, axis=1)  # undirected
    _, counts = np.unique(edges, axis=0, return_counts=True)
    n_bad = int(np.sum(counts != 2))
    return n_bad == 0, n_bad


def export(
    part: "Part",
    params: WheelParams,
    out_dir: Path,
    *,
    stl_tolerance_mm: float = 0.05,
    stl_angular_tolerance_rad: float = ANGULAR_TOLERANCE_RAD,
    prefix: str = "wheel",
) -> ExportPaths:
    """Write STEP and STL for one wheel, named by design hash.

    The STL is meshed explicitly via :func:`remesh` first. Left to ``export_stl`` alone the
    mesh depends on the shape's meshing history rather than on the arguments here, which
    makes the collision mesh a function of call order — see :func:`remesh`.

    Args:
        part: the solid.
        params: used for the content hash in the filename.
        out_dir: created if absent.
        stl_tolerance_mm: chordal deviation for the STL.
        stl_angular_tolerance_rad: angular deviation for the STL. Dominant on curved
            surfaces; refine it alongside the chordal tolerance, not instead of it.
        prefix: filename prefix.
    """
    from build123d import export_step, export_stl

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = f"{prefix}_{params.spoke_profile.value}_{params.design_hash()}"
    step_path = out_dir / f"{stem}.step"
    stl_path = out_dir / f"{stem}.stl"

    export_step(part, str(step_path))

    remesh(part, stl_tolerance_mm, stl_angular_tolerance_rad)
    export_stl(
        part,
        str(stl_path),
        tolerance=stl_tolerance_mm,
        angular_tolerance=stl_angular_tolerance_rad,
    )

    return ExportPaths(step=step_path, stl=stl_path, stem=stem)
