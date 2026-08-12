"""STEP -> second-order tetrahedral mesh, via gmsh. The only module that needs gmsh.

docs/plan/14-cad-toolchain.md: "FEA mesh from STEP, not STL." The STL is a lossy derived
artefact (ADR-0003); meshing it would bake the tessellation error into every stiffness
number and defeat the point of carrying BREP through the pipeline.

Surfaces are identified **by node radius, not by STEP face id**. Face ids are assigned by
OCCT during the boolean operations and are stable only until something upstream changes
the order of a union; the bore is the only cylinder at ``hub_bore_radius`` and the tread is
the only cylinder at ``outer_radius``, and that stays true across any re-export.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..cad.params import WheelParams
from .loadcase import MeshSpec

__all__ = ["GMSH_TO_ABAQUS_TET10", "FeaMesh", "MeshFailure", "MeshStats", "mesh_step"]

#: gmsh orders TET10 mid-side nodes as (0,1) (1,2) (0,2) (0,3) (2,3) (1,3); Abaqus C3D10
#: expects (1,2) (2,3) (3,1) (1,4) (2,4) (3,4). The first four agree, and so do the first
#: four mid-side nodes — but the **last two are swapped**. Getting this wrong produces a
#: mesh that solves happily and reports the wrong stiffness, so it is asserted directly by
#: the single-element patch test in scripts/verify_fea.py rather than trusted.
GMSH_TO_ABAQUS_TET10 = (0, 1, 2, 3, 4, 5, 6, 7, 9, 8)


class MeshFailure(RuntimeError):
    """gmsh could not produce a usable mesh. Caught by the runner and typed as
    ``FeaStatus.MESH_FAILED`` — it must never escape an evaluation (invariant 4)."""


@dataclass(frozen=True, slots=True)
class MeshStats:
    n_nodes: int
    n_elements: int
    min_volume_m3: float
    max_aspect_ratio: float
    gmsh_version: str

    def summary(self) -> str:  # pragma: no cover - display only
        return (
            f"{self.n_elements} elements, {self.n_nodes} nodes, "
            f"min vol {self.min_volume_m3:.2e} m^3, gmsh {self.gmsh_version}"
        )


@dataclass(frozen=True, slots=True)
class FeaMesh:
    """A meshed wheel, in **metres**, with geometrically identified sets.

    Node and element indices are 1-based throughout, matching the deck they are written to.
    """

    nodes_m: np.ndarray
    #: (m, 10) for C3D10 or (m, 4) for C3D4, 1-based, in Abaqus node order.
    elements: np.ndarray
    element_type: str
    #: name -> 1-based node ids. Always contains "bore", "tread", "hub", "rim", "spokes".
    node_sets: dict[str, np.ndarray] = field(default_factory=dict)
    #: name -> 1-based element ids. Always contains "hub", "rim", "spokes".
    element_sets: dict[str, np.ndarray] = field(default_factory=dict)
    stats: MeshStats | None = None

    @property
    def n_nodes(self) -> int:
        return len(self.nodes_m)

    @property
    def n_elements(self) -> int:
        return len(self.elements)


def classify_nodes(
    nodes_m: np.ndarray, params: WheelParams, spec: MeshSpec
) -> dict[str, np.ndarray]:
    """Split nodes into named sets by radius. 1-based ids.

    ``bore`` becomes the fixed boundary (the axle) and ``tread`` the contact slave surface,
    so these two decide the whole boundary-value problem.

    The tread set is the outer cylinder **only**. With ``tread_depth_mm > 0`` the outer
    surface is three bands at ``R`` plus groove floors at ``R - depth``; the floors never
    touch a flat plate and must not be offered to the contact search.
    """
    tol = spec.surface_tolerance_m
    r = np.hypot(nodes_m[:, 0], nodes_m[:, 1])
    ids = np.arange(1, len(nodes_m) + 1, dtype=np.int64)

    bore_r = params.hub_bore_radius_mm * 1e-3
    hub_r = params.hub_radius_mm * 1e-3
    rim_inner_r = params.rim_inner_radius_mm * 1e-3
    outer_r = params.outer_radius_mm * 1e-3

    sets = {
        "bore": ids[np.abs(r - bore_r) < tol] if bore_r > 0 else ids[r < tol],
        "tread": ids[np.abs(r - outer_r) < tol],
        "hub": ids[r <= hub_r + tol],
    }
    if params.has_shear_band:
        sets["rim"] = ids[r >= rim_inner_r - tol]
        sets["spokes"] = ids[(r > hub_r + tol) & (r < rim_inner_r - tol)]
    else:
        # `rim_inner_r == outer_r` here, so the banded expressions would label the tips as
        # rim and drop them from "spokes" — and the tips are where a bandless wheel carries
        # contact and peaks in stress. Everything outboard of the hub is spoke.
        sets["rim"] = ids[:0]
        sets["spokes"] = ids[r > hub_r + tol]
    if len(sets["bore"]) == 0:
        raise MeshFailure("no nodes found on the hub bore; the axle cannot be restrained")
    if len(sets["tread"]) == 0:
        raise MeshFailure("no nodes found on the tread; there is nothing to contact")
    return sets


def classify_elements(
    nodes_m: np.ndarray, elements: np.ndarray, params: WheelParams, n_corners: int = 4
) -> dict[str, np.ndarray]:
    """Split elements by centroid radius. 1-based ids.

    ``spokes`` is what stress output is requested over — the whole mesh would be tens of
    megabytes per time point for a quantity only the spokes need.

    Args:
        n_corners: how many leading nodes are corners — 4 for a tetrahedron, 3 for the
            plane-strain triangles in :mod:`wheelopt.fea.section2d`. Averaging a CPE6's
            first four nodes would mix three corners with one mid-side node and pull every
            centroid toward one edge, which misclassifies elements near a set boundary
            without producing anything that looks wrong.
    """
    corners = elements[:, :n_corners] - 1
    centroids = nodes_m[corners].mean(axis=1)
    r = np.hypot(centroids[:, 0], centroids[:, 1])
    ids = np.arange(1, len(elements) + 1, dtype=np.int64)

    hub_r = params.hub_radius_mm * 1e-3
    rim_inner_r = params.rim_inner_radius_mm * 1e-3
    if not params.has_shear_band:
        # See classify_nodes: with no band the tip elements are the ones under contact, so
        # they must stay in the set that stress output is requested over.
        return {"hub": ids[r <= hub_r], "rim": ids[:0], "spokes": ids[r > hub_r]}
    return {
        "hub": ids[r <= hub_r],
        "rim": ids[r >= rim_inner_r],
        "spokes": ids[(r > hub_r) & (r < rim_inner_r)],
    }


def _tet_volumes(nodes_m: np.ndarray, elements: np.ndarray) -> np.ndarray:
    c = nodes_m[elements[:, :4] - 1]
    return np.abs(np.einsum("ij,ij->i", c[:, 1] - c[:, 0],
                            np.cross(c[:, 2] - c[:, 0], c[:, 3] - c[:, 0]))) / 6.0


#: Parametric sample points for the quadratic Jacobian check: the four corners, six edge
#: midpoints, and the centroid of the reference tetrahedron. A C3D10 whose mid-side nodes
#: have been curved onto a surface can be inverted at these points while the straight-edge
#: corner volume stays positive, which is exactly the case CalculiX rejects at t=0.
_JAC_SAMPLES = np.array(
    [
        (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
        (0.5, 0.0, 0.0), (0.0, 0.5, 0.0), (0.0, 0.0, 0.5),
        (0.5, 0.5, 0.0), (0.5, 0.0, 0.5), (0.0, 0.5, 0.5),
        (0.25, 0.25, 0.25),
    ],
    dtype=np.float64,
)


def _tet10_shape_gradients(pt: np.ndarray) -> np.ndarray:
    """Gradients of the 10 C3D10 shape functions at one parametric point (10, 3)."""
    r, s, t = pt
    lam = np.array([1.0 - r - s - t, r, s, t])
    grad = np.array([[-1.0, -1.0, -1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    out = np.zeros((10, 3))
    for i in range(4):
        out[i] = (4.0 * lam[i] - 1.0) * grad[i]
    # Abaqus C3D10 mid-side ordering: (0,1) (1,2) (2,0) (0,3) (1,3) (2,3)
    for k, (a, b) in enumerate([(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]):
        out[4 + k] = 4.0 * (lam[a] * grad[b] + lam[b] * grad[a])
    return out


def _min_quadratic_jacobian(nodes_m: np.ndarray, elements: np.ndarray) -> np.ndarray:
    """Smallest Jacobian determinant over the sample points, per C3D10 element.

    Non-positive anywhere means the element is folded and CalculiX will reject it. This is
    the check the naive corner-volume test misses.
    """
    coords = nodes_m[elements - 1]  # (m, 10, 3)
    worst = np.full(len(elements), np.inf)
    for pt in _JAC_SAMPLES:
        grads = _tet10_shape_gradients(pt)          # (10, 3)
        jac = np.einsum("mni,nj->mij", coords, grads)  # (m, 3, 3)
        worst = np.minimum(worst, np.linalg.det(jac))
    return worst


def mesh_step(step_path: Path, params: WheelParams, spec: MeshSpec) -> FeaMesh:
    """Mesh a STEP file into second-order tetrahedra.

    The STEP is authored in millimetres (the CAD layer's only non-SI zone), so gmsh works
    in millimetres and the conversion to metres happens once, here, at the boundary.

    Raises:
        MeshFailure: on any meshing problem. The runner turns this into a typed result.
    """
    # Without this, a dimension-2 spec meshed here produces tetrahedra carrying
    # `spec.element_type`, which is "CPE6" — 3-D solid elements declared to CalculiX as
    # plane-strain triangles, with three of their ten nodes read as the whole element. The
    # deck is well-formed and the solve runs. Measured before the guard: 49 313 "CPE6"
    # written from a call that was meant to take the 2-D path.
    if spec.dimension != 3:
        raise MeshFailure(
            f"mesh_step is the solid path but MeshSpec.dimension is {spec.dimension}; "
            "use wheelopt.fea.section2d.mesh_section for the plane-strain tier"
        )
    try:
        import gmsh
    except ImportError as exc:  # pragma: no cover - environment
        raise MeshFailure("gmsh is not installed; pip install -e '.[fea]'") from exc

    step_path = Path(step_path)
    if not step_path.exists():
        raise MeshFailure(f"STEP file not found: {step_path}")

    hub_r_mm = params.hub_radius_mm
    # With no shear band this coincides with the outer radius, so the rim branch below
    # would apply the (coarser) rim size to the tips — the one place a bandless wheel most
    # needs resolution. Push it out of reach instead and let the spoke size govern.
    rim_inner_mm = params.rim_inner_radius_mm if params.has_shear_band else float("inf")
    size_hub_mm = spec.size_hub_m * 1e3
    size_rim_mm = spec.size_rim_m * 1e3
    size_spoke_mm = spec.size_spoke_m * 1e3

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        # Determinism (Phase 0 gate): Delaunay is single-threaded and reproducible. HXT is
        # multithreaded and returns a different mesh run to run, which would make every
        # cache key a lie.
        gmsh.option.setNumber("Mesh.Algorithm3D", spec.algorithm_3d)
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.option.setNumber("Mesh.ElementOrder", spec.order)
        if spec.order == 2:
            gmsh.option.setNumber("Mesh.HighOrderOptimize", spec.high_order_optimize)
            gmsh.option.setNumber("Mesh.SecondOrderIncomplete", 0)
            # Straight-sided quadratic tets. Curving mid-side nodes onto the bore and spoke
            # fillets folds the element and CalculiX rejects it outright; see MeshSpec.
            gmsh.option.setNumber(
                "Mesh.SecondOrderLinear", 1 if spec.second_order_linear else 0
            )
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.option.setNumber("Mesh.OptimizeNetgen", 1 if spec.optimize_netgen else 0)
        # Curvature-driven refinement over-resolves the small bore for no benefit; the
        # radius callback below is the intended size control.
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)

        gmsh.model.add("wheel")
        gmsh.model.occ.importShapes(str(step_path))
        gmsh.model.occ.synchronize()

        volumes = gmsh.model.getEntities(3)
        if not volumes:
            raise MeshFailure(f"STEP contains no solid volume: {step_path}")
        if len(volumes) > 1:
            raise MeshFailure(
                f"STEP contains {len(volumes)} volumes; the CAD stage must emit one solid"
            )

        def size_at(dim, tag, x, y, z, lc):
            r = (x * x + y * y) ** 0.5
            if r <= hub_r_mm:
                return size_hub_mm
            if r >= rim_inner_mm:
                return size_rim_mm
            return size_spoke_mm

        gmsh.model.mesh.setSizeCallback(size_at)
        try:
            gmsh.model.mesh.generate(3)
        except Exception as exc:  # gmsh raises a bare Exception, e.g. "PLC Error"
            # This module's contract is that every meshing problem arrives as a
            # MeshFailure, which the runner turns into a typed result (invariant 4). A raw
            # gmsh exception escaping here crashes the whole evaluation instead. It is
            # easy to provoke: a size field coarser than a feature — say a 18 mm element
            # on a 4 mm bore — makes the surface facets self-intersect.
            raise MeshFailure(
                f"gmsh failed to generate the volume mesh: {exc}. "
                f"Element sizes were hub {size_hub_mm:g} / spoke {size_spoke_mm:g} / "
                f"rim {size_rim_mm:g} mm against a {params.hub_bore_radius_mm:g} mm bore; "
                "a size larger than the smallest feature is the usual cause."
            ) from exc

        node_tags, coords, _ = gmsh.model.mesh.getNodes()
        if len(node_tags) == 0:
            raise MeshFailure("gmsh produced no nodes")

        tet_type = 11 if spec.order == 2 else 4
        n_per_elem = 10 if spec.order == 2 else 4
        elem_tags, elem_nodes = gmsh.model.mesh.getElementsByType(tet_type)
        if len(elem_tags) == 0:
            raise MeshFailure(
                f"gmsh produced no order-{spec.order} tetrahedra "
                f"(element type {tet_type})"
            )

        # gmsh node tags are not guaranteed contiguous; compact them to 1..n.
        order = np.argsort(node_tags)
        sorted_tags = node_tags[order]
        nodes_m = coords.reshape(-1, 3)[order] * 1e-3

        conn = np.searchsorted(sorted_tags, elem_nodes.reshape(-1, n_per_elem)) + 1
        if spec.order == 2:
            conn = conn[:, list(GMSH_TO_ABAQUS_TET10)]

        version = gmsh.option.getString("General.Version")
    finally:
        gmsh.finalize()

    volumes_m3 = _tet_volumes(nodes_m, conn)
    if not np.all(volumes_m3 > 0):
        raise MeshFailure(f"{int(np.sum(volumes_m3 <= 0))} degenerate tetrahedra")

    if spec.order == 2:
        # Catch folded quadratic elements here, as a typed MeshFailure, rather than letting
        # CalculiX hit "nonpositive jacobian determinant" and abort the solve at t=0.
        min_jac = _min_quadratic_jacobian(nodes_m, conn)
        n_bad = int(np.sum(min_jac <= 0))
        if n_bad:
            raise MeshFailure(
                f"{n_bad} C3D10 elements have a non-positive Jacobian; the solve would be "
                "rejected. Enable MeshSpec.second_order_linear or coarsen near the bore."
            )

    node_sets = classify_nodes(nodes_m, params, spec)
    element_sets = classify_elements(nodes_m, conn, params)

    edges = nodes_m[conn[:, [1, 2, 3, 2, 3, 3]] - 1] - nodes_m[conn[:, [0, 0, 0, 1, 1, 2]] - 1]
    edge_len = np.linalg.norm(edges, axis=2)
    aspect = edge_len.max(axis=1) / np.maximum(edge_len.min(axis=1), 1e-12)

    stats = MeshStats(
        n_nodes=len(nodes_m),
        n_elements=len(conn),
        min_volume_m3=float(volumes_m3.min()),
        max_aspect_ratio=float(aspect.max()),
        gmsh_version=version,
    )
    return FeaMesh(
        nodes_m=nodes_m,
        elements=conn,
        element_type=spec.element_type,
        node_sets=node_sets,
        element_sets=element_sets,
        stats=stats,
    )
