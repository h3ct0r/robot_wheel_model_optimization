"""The wheel cross-section, and its plane-strain mesh.

Why this exists
---------------
The 3-D tier is correct and, at the re-specified platform, unaffordable: the nominal design
meshes to ~50 k C3D10 and solves in roughly 20 hours per sweep, because a 3 mm shear band, a
7 mm spoke and a 4 mm bore set the element size no matter what size field is asked for. The
ROM fit needs one evaluation *per design*. Dropping to a 2-D plane-strain slice of the
cross-section was the contingency named in the implementation plan for first-week step 3 —
"converges in seconds, gives a ``k_r(δ)`` of the right shape (scale by width)" — and it is a
deliberate fidelity reduction, not a refinement. (That plan is a session artefact, not a
committed document; ``docs/plan/16-first-week.md`` does not mention it.)

What plane strain actually assumes
----------------------------------
That nothing moves out of plane: ``ε_zz = 0`` everywhere. A real wheel of finite width has
free faces at ``z = ±W/2`` that bulge under load, so the *constitutive* effect of plane strain
is to over-constrain and stiffen. Plane stress makes the opposite error, and the truth is
between them.

**Measured, though, this tier comes out softer than the 3-D one, not stiffer** — 0.90 on peak
force and 0.86-0.90 on ``k_r`` (2026-08-08, ``--tiny`` banded, frictionless, matched load
case). The two runs differ in more than dimensionality: the section is meshed at 2.5 mm
against the solid's 8 mm, and a coarse mesh over-stiffens, so some of the gap is convergence
rather than the plane-strain assumption. Which effect dominates is **not established**, and
the sign of the difference should not be reasoned about a priori — it was, and the reasoning
was wrong. ``scripts/verify_fea.py`` measures the ratio; treat a 2-D ``k_r`` as a shape to fit
and a magnitude to calibrate.

The other thing it cannot see is out-of-plane behaviour entirely: lateral spoke buckling,
sidewall taper, and any spanwise pattern. Those are exactly the phenomena the 3-D tier exists
for, so this does not replace it — it screens.

Geometry provenance
-------------------
The section is built from :mod:`wheelopt.cad.centreline`, the same module the 3-D solid is
built from, rather than by sectioning the exported STEP. That makes the two identical by
construction instead of by agreement, and it keeps this module free of OCCT — it needs gmsh
and numpy only. The 3-D wheel is this section extruded to ``width_mm`` (plus the tread cut,
which this tier ignores; see :func:`section_area_mm2`).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..cad.centreline import attachment_overlap_mm, spoke_outline
from ..cad.params import WheelParams
from .loadcase import MeshSpec
from .mesh import FeaMesh, MeshFailure, MeshStats, classify_elements, classify_nodes

__all__ = [
    "GMSH_TO_ABAQUS_TRI6",
    "mesh_claw_sector",
    "mesh_section",
    "section_area_mm2",
    "section_polygons",
]

#: gmsh orders TRI6 as three corners then the mid-side nodes of (0,1) (1,2) (2,0). Abaqus
#: CPE6 wants the same. Kept explicit so the assumption is visible next to
#: :data:`wheelopt.fea.mesh.GMSH_TO_ABAQUS_TET10`, where the equivalent mapping is *not*
#: the identity and silently reports a wrong stiffness when assumed to be.
GMSH_TO_ABAQUS_TRI6 = (0, 1, 2, 3, 4, 5)

#: Fewest bore nodes a hub wedge may keep before :func:`mesh_claw_sector` refuses it. Four is
#: two second-order edges — the least that reads as a clamped arc rather than a pair of pins.
#: Not tuned; it is a floor below which the model is qualitatively wrong, and a wedge near it
#: has not been validated against the full hub in any case.
_MIN_SECTOR_BORE_NODES = 4


def section_polygons(params: WheelParams) -> list[np.ndarray]:
    """The spoke outlines of the cross-section, millimetres, one ``(n, 2)`` array each.

    Hub and shear band are annuli and are described by radii rather than polygons; see
    :func:`mesh_section`, which builds them with gmsh primitives so their curvature is exact
    rather than faceted.

    The overlap is the same :func:`~wheelopt.cad.centreline.attachment_overlap_mm` the solid
    uses, so the spokes bury into the hub and band by the same amount here as there.
    """
    overlap = attachment_overlap_mm(params)
    return [spoke_outline(params, i, overlap_mm=overlap) for i in range(params.n_spokes)]


def _polygon_area_mm2(points: np.ndarray) -> float:
    """Shoelace area of a closed polygon given without its repeated first point."""
    x, y = points[:, 0], points[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y)))


def section_area_mm2(params: WheelParams) -> float:
    """Analytic area of the section, millimetres squared.

    Upper bound, not an identity: the spoke polygons overlap the hub and band annuli by
    ``attachment_overlap_mm`` at each end, and this sums the parts without subtracting those
    overlaps. Used to sanity-check the meshed area from above — a mesh larger than this has
    a self-intersecting outline, which is the failure that matters.

    Ignores ``tread_depth_mm``: the tread is a set of circumferential grooves, so a section
    at one axial station either cuts a groove or misses it, and the 2-D tier has no axial
    station. A treaded design is meshed here as if untreaded and screened as such.
    """
    r_out = params.outer_radius_mm
    r_hub = params.hub_radius_mm
    r_bore = params.hub_bore_radius_mm
    area = np.pi * (r_hub**2 - r_bore**2)
    if params.has_shear_band:
        area += np.pi * (r_out**2 - params.rim_inner_radius_mm**2)
    area += sum(_polygon_area_mm2(p) for p in section_polygons(params))
    return float(area)


def _n_connected_components(elements: np.ndarray, n_nodes: int) -> int:
    """Connected components of the element graph, joined through shared corner nodes.

    This is the check that the section is one wheel and not a hub with six loose spokes
    floating near it. It replaces a face-count check, which could not tell the difference:
    `fragment` produces many faces for a perfectly sound section, and `fuse` produces the
    right *number* of faces for an unsound one.

    Union-find over corner nodes only — a mid-side node is shared by exactly the elements
    that already share the corners at either end of its edge, so it adds nothing.
    """
    parent = np.arange(n_nodes + 1, dtype=np.int64)

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for tri in elements[:, :3]:
        union(int(tri[0]), int(tri[1]))
        union(int(tri[1]), int(tri[2]))

    used = np.unique(elements[:, :3])
    return len({find(int(n)) for n in used})


#: Swapping the first two corners of a TRI6 reverses its winding. Corners 1 and 2 exchange;
#: corner 3 is fixed; the mid-side node of edge (1,2) is fixed, and the mid-side nodes of
#: (2,3) and (3,1) exchange with each other.
_TRI6_REVERSE = (1, 0, 2, 3, 5, 4)


def _tri_signed_areas(nodes_m: np.ndarray, elements: np.ndarray) -> np.ndarray:
    """Signed area of each triangle. Negative means clockwise in the x-y plane."""
    c = nodes_m[elements[:, :3] - 1]
    return 0.5 * (
        (c[:, 1, 0] - c[:, 0, 0]) * (c[:, 2, 1] - c[:, 0, 1])
        - (c[:, 2, 0] - c[:, 0, 0]) * (c[:, 1, 1] - c[:, 0, 1])
    )


def _tri_areas(nodes_m: np.ndarray, elements: np.ndarray) -> np.ndarray:
    """Unsigned area of each triangle, for measuring the meshed section."""
    return np.abs(_tri_signed_areas(nodes_m, elements))


def _orient_ccw(nodes_m: np.ndarray, elements: np.ndarray) -> np.ndarray:
    """Rewind every clockwise triangle counter-clockwise.

    CalculiX expands a plane-strain element into a 3-D layer along +z, so a clockwise
    triangle expands inside-out and the solve dies at t=0 with "nonpositive jacobian
    determinant". gmsh does not orient consistently across the faces `fragment` produces —
    measured on the verification wheel: the mesh contains both windings — so this cannot be
    assumed, only enforced. An unsigned area check cannot see the problem at all, which is
    how it got as far as the solver.
    """
    flip = _tri_signed_areas(nodes_m, elements) < 0.0
    if np.any(flip):
        elements = elements.copy()
        elements[flip] = elements[flip][:, list(_TRI6_REVERSE)]
    return elements


@dataclass(frozen=True, slots=True)
class ClawSector:
    """Mesh **one claw** and the hub it hangs off, instead of the whole wheel.

    The point of the `T7` claw family for the ROM is that the ring's segments can *be* the
    claws, one for one — at which point the segment spring law stops being a deconvolution of
    a whole-wheel ``F(δ)`` and becomes a direct measurement. This is the geometry that makes
    that measurement possible, and it is only meaningful for a **bandless** design: with a
    shear band the claws are not independent and one of them is not a model of anything.

    The saving is real and it is the reason the 20 h/sweep 3-D problem may be recoverable.
    It also compounds with :attr:`hub_span_deg`: at 360° the hub is meshed in full, so cost
    falls only by the spokes dropped; at one pitch angle the mesh is independent of
    ``n_spokes`` altogether.

    Attributes:
        hub_span_deg: angular width of the hub wedge kept, degrees, centred on the claw.
            ``None`` keeps the **whole hub annulus**, which is exactly right and costs more.
            A finite span cuts two radial faces that do not exist in the real wheel and
            leaves them **free**, so the hub end of the claw is held less firmly than it
            really is and the measured claw is **softer** than the truth. That error is
            one-signed, which is what makes it usable: it is a bound, not a wobble. The
            magnitude is measured against ``None`` rather than assumed — see the log.
    """

    hub_span_deg: float | None = None

    def __post_init__(self) -> None:
        if self.hub_span_deg is not None and not 0.0 < self.hub_span_deg <= 360.0:
            raise ValueError("hub_span_deg must be in (0, 360] degrees, or None")


def mesh_claw_sector(
    params: WheelParams, spec: MeshSpec, *, hub_span_deg: float | None = None
) -> FeaMesh:
    """One claw plus its hub, meshed as plane-strain CPE6. See :class:`ClawSector`.

    Raises:
        MeshFailure: if the design has a shear band — one claw of a banded wheel is not an
            independent structure, and measuring it as though it were would report a
            stiffness the wheel does not have.
    """
    if params.has_shear_band:
        raise MeshFailure(
            "a claw sector is only meaningful without a shear band: the band couples the "
            f"claws, so one of them is not a model of anything. rim_thickness_mm is "
            f"{params.rim_thickness_mm:g}; set it to 0 for the T7 topology"
        )
    return mesh_section(params, spec, claw_sector=ClawSector(hub_span_deg=hub_span_deg))


def mesh_section(
    params: WheelParams, spec: MeshSpec, *, claw_sector: ClawSector | None = None
) -> FeaMesh:
    """Mesh the cross-section into second-order plane-strain triangles (CPE6).

    Returns a :class:`~wheelopt.fea.mesh.FeaMesh` whose nodes carry ``z = 0``, so
    :func:`~wheelopt.fea.mesh.classify_nodes` and
    :func:`~wheelopt.fea.mesh.classify_elements` — which classify on ``hypot(x, y)`` —
    apply unchanged, and the bore/tread sets mean the same thing they do in 3-D.

    Triangles rather than the CPE8 quads originally sketched: recombination can
    silently leave a mixed quad/triangle mesh, which would need two element blocks and two
    section cards, and the 3-D tier already established that near-incompressible locking is
    not biting at the ν≈0.46 the infill knock-down gives (``verify_fea.py`` check 2). The
    robustness is worth more here than the extra accuracy per element.

    Raises:
        MeshFailure: on any meshing problem, so the runner can type it (invariant 4).
    """
    if spec.dimension != 2:
        raise MeshFailure(
            f"mesh_section is the plane-strain path but MeshSpec.dimension is "
            f"{spec.dimension}; use mesh_step for the 3-D solid"
        )
    try:
        import gmsh
    except ImportError as exc:  # pragma: no cover - environment
        raise MeshFailure("gmsh is not installed; pip install -e '.[fea]'") from exc

    hub_r_mm = params.hub_radius_mm
    rim_inner_mm = params.rim_inner_radius_mm if params.has_shear_band else float("inf")
    size_hub_mm = spec.size_hub_m * 1e3
    size_rim_mm = spec.size_rim_m * 1e3
    size_spoke_mm = spec.size_spoke_m * 1e3

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        # Same determinism contract as the 3-D path: single-threaded, reproducible, so the
        # cache key describes exactly one mesh. Algorithm 5 (Delaunay) for surfaces.
        gmsh.option.setNumber("Mesh.Algorithm", 5)
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.option.setNumber("Mesh.SecondOrderIncomplete", 0)
        gmsh.option.setNumber("Mesh.SecondOrderLinear", 1 if spec.second_order_linear else 0)
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)

        gmsh.model.add("wheel_section")
        occ = gmsh.model.occ

        # Hub annulus. Built as primitives so the bore and the outer surfaces are true
        # circles — the contact and boundary sets are picked by radius, and a faceted circle
        # would put nodes off the nominal radius and out of the tolerance band.
        hub = occ.addDisk(0.0, 0.0, 0.0, hub_r_mm, hub_r_mm)
        if params.hub_bore_radius_mm > 0:
            bore = occ.addDisk(
                0.0, 0.0, 0.0, params.hub_bore_radius_mm, params.hub_bore_radius_mm
            )
            cut, _ = occ.cut([(2, hub)], [(2, bore)])
            parts = list(cut)
        else:
            parts = [(2, hub)]

        if params.has_shear_band:
            outer = occ.addDisk(0.0, 0.0, 0.0, params.outer_radius_mm, params.outer_radius_mm)
            inner = occ.addDisk(
                0.0, 0.0, 0.0, params.rim_inner_radius_mm, params.rim_inner_radius_mm
            )
            band, _ = occ.cut([(2, outer)], [(2, inner)])
            parts.extend(band)

        if claw_sector is not None and claw_sector.hub_span_deg is not None:
            # Keep only a wedge of the hub. Built by intersecting the annulus with a pie
            # slice rather than by drawing the wedge outline, so the bore and outer arcs stay
            # true circles: `classify_nodes` picks the bore set by |r - r_bore| < 0.1 mm, and
            # a faceted arc puts nodes outside that band and silently loses the constraint.
            centre = np.deg2rad(params.spoke_phase_deg)
            half = np.deg2rad(0.5 * claw_sector.hub_span_deg)
            reach = 4.0 * params.hub_radius_mm
            wedge_pts = [(0.0, 0.0)] + [
                (reach * np.cos(centre + a), reach * np.sin(centre + a))
                for a in np.linspace(-half, half, max(3, int(claw_sector.hub_span_deg / 5)))
            ]
            tags = [occ.addPoint(float(x), float(y), 0.0) for x, y in wedge_pts]
            wedge_lines = [
                occ.addLine(tags[i], tags[(i + 1) % len(tags)]) for i in range(len(tags))
            ]
            wedge = occ.addPlaneSurface([occ.addCurveLoop(wedge_lines)])
            kept, _ = occ.intersect(parts, [(2, wedge)])
            parts = list(kept)
            if not parts:
                raise MeshFailure(
                    f"a {claw_sector.hub_span_deg:g} deg hub wedge intersected the hub "
                    "annulus to nothing"
                )

        spokes = section_polygons(params)
        if claw_sector is not None:
            # Spoke 0 sits at `spoke_phase_deg`, which is also where the wedge is centred and
            # where `phase_for_tip_contact` aims the indenter. Keeping index 0 rather than an
            # arbitrary one is what makes those three agree.
            spokes = spokes[:1]
        for polygon in spokes:
            pts = [occ.addPoint(float(x), float(y), 0.0) for x, y in polygon]
            lines = [
                occ.addLine(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))
            ]
            loop = occ.addCurveLoop(lines)
            parts.append((2, occ.addPlaneSurface([loop])))

        # `fragment`, not `fuse`. Fusing these coplanar overlapping faces leaves them as
        # separate faces (measured: 8 in, 8 out), which meshes as 8 unconnected patches —
        # a model that solves and reports a wheel with detached spokes. Fragment imprints
        # them into a conformal set of faces that share edges, and therefore share nodes.
        # The face count afterwards is meaningless (20 for a 6-spoke wheel); whether the
        # mesh is actually one connected body is checked below, on the mesh itself.
        if len(parts) > 1:
            occ.fragment(parts[:1], parts[1:])
        occ.synchronize()

        if not gmsh.model.getEntities(2):
            raise MeshFailure("the cross-section produced no surface")

        def size_at(dim, tag, x, y, z, lc):
            r = (x * x + y * y) ** 0.5
            if r <= hub_r_mm:
                return size_hub_mm
            if r >= rim_inner_mm:
                return size_rim_mm
            return size_spoke_mm

        gmsh.model.mesh.setSizeCallback(size_at)
        try:
            gmsh.model.mesh.generate(2)
        except Exception as exc:  # gmsh raises a bare Exception
            raise MeshFailure(
                f"gmsh failed to mesh the cross-section: {exc}. Element sizes were hub "
                f"{size_hub_mm:g} / spoke {size_spoke_mm:g} / rim {size_rim_mm:g} mm "
                f"against a {params.hub_bore_radius_mm:g} mm bore."
            ) from exc

        node_tags, coords, _ = gmsh.model.mesh.getNodes()
        if len(node_tags) == 0:
            raise MeshFailure("gmsh produced no nodes")

        elem_tags, elem_nodes = gmsh.model.mesh.getElementsByType(9)  # TRI6
        if len(elem_tags) == 0:
            raise MeshFailure("gmsh produced no 6-node triangles")

        order = np.argsort(node_tags)
        sorted_tags = node_tags[order]
        nodes_m = coords.reshape(-1, 3)[order] * 1e-3
        # Flatten exactly. gmsh returns z as a float that is zero to rounding; CalculiX
        # expands plane-strain elements about z and a stray 1e-19 would be a real, tiny
        # out-of-plane coordinate.
        nodes_m[:, 2] = 0.0

        conn = np.searchsorted(sorted_tags, elem_nodes.reshape(-1, 6)) + 1
        conn = conn[:, list(GMSH_TO_ABAQUS_TRI6)]
        version = gmsh.option.getString("General.Version")
    finally:
        gmsh.finalize()

    conn = _orient_ccw(nodes_m, conn)
    areas_m2 = _tri_signed_areas(nodes_m, conn)
    if not np.all(areas_m2 > 0):
        raise MeshFailure(
            f"{int(np.sum(areas_m2 <= 0))} triangles are degenerate or could not be "
            "oriented counter-clockwise"
        )

    n_parts = _n_connected_components(conn, len(nodes_m))
    if n_parts != 1:
        raise MeshFailure(
            f"the cross-section meshed as {n_parts} disconnected bodies. Either the spokes "
            "do not reach both the hub and the running surface, or the parts were not "
            "imprinted conformally — a mesh like this solves happily and describes a wheel "
            "whose spokes are not attached to anything."
        )

    node_sets = classify_nodes(nodes_m, params, spec)
    element_sets = classify_elements(nodes_m, conn, params, n_corners=3)

    if claw_sector is not None and claw_sector.hub_span_deg is not None:
        # The bore is where the deck clamps the wheel to the shaft. Cutting a wedge out of
        # the hub cuts the bore arc with it: measured on the nominal claw design, a 30 deg
        # wedge on a 4 mm bore keeps ~2 mm of arc and **two** nodes. Two nodes is four
        # constraints in 2-D, which is enough to stop rigid-body motion and therefore enough
        # to *solve* — it just describes a claw pinned at two points rather than clamped to a
        # shaft, and it reports a compliance that is mostly the wedge pivoting. That is the
        # failure this project keeps meeting: it converges, and the number is wrong.
        n_bore = len(node_sets.get("bore", ()))
        if n_bore < _MIN_SECTOR_BORE_NODES:
            raise MeshFailure(
                f"a {claw_sector.hub_span_deg:g} deg hub wedge keeps only {n_bore} bore "
                f"node(s); {_MIN_SECTOR_BORE_NODES} are needed before the shaft constraint "
                "is a clamp rather than a pin. Widen hub_span_deg, or use the full hub "
                "annulus (hub_span_deg=None), which is exact and still drops every spoke "
                "but one"
            )

    edges = nodes_m[conn[:, [1, 2, 0]] - 1] - nodes_m[conn[:, [0, 1, 2]] - 1]
    edge_len = np.linalg.norm(edges, axis=2)
    aspect = edge_len.max(axis=1) / np.maximum(edge_len.min(axis=1), 1e-12)

    stats = MeshStats(
        n_nodes=len(nodes_m),
        n_elements=len(conn),
        # Area, not volume — the field is shared with the 3-D path and the unit differs.
        # Reported as m^2 in the summary line so the two cannot be confused.
        min_volume_m3=float(areas_m2.min()),
        max_aspect_ratio=float(aspect.max()),
        gmsh_version=version,
    )
    return FeaMesh(
        nodes_m=nodes_m,
        elements=conn,
        element_type="CPE6",
        node_sets=node_sets,
        element_sets=element_sets,
        stats=stats,
    )
