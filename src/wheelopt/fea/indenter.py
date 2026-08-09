"""Rigid contact bodies: the flat plate and the filleted step edge. Pure numpy.

CalculiX has no analytical rigid surfaces, so whatever the wheel is pressed against has to
be real elements. They are meshed here rather than in gmsh because their shape is trivial
and their node numbering needs to be predictable: the deck declares a contact master
surface by element face number, and guessing gmsh's face numbering is a needless risk.

Construction is the same for both cases. Take the 2-D profile the wheel actually touches,
offset it inward by the block thickness, and sweep along the wheel's axis. That yields a
single layer of C3D8 hexahedra following the profile — including around the corner fillet —
with the contact face always on the same local face number. One element through thickness
is enough because the body is rigid; nothing is being resolved inside it.

Geometry convention, shared with the deck: the wheel's axis is **z**, its mid-plane is
z = 0, and the indenter sits below at negative **y** and is driven in **+y** to compress.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .loadcase import IndenterSpec, LoadCaseKind

__all__ = ["IndenterMesh", "build_indenter", "CONTACT_FACE", "CONTACT_FACE_2D"]

#: Local CPE4 face carrying the contact surface. CalculiX numbers plane-element faces by
#: their edges: S1 = 1-2, S2 = 2-3, S3 = 3-4, S4 = 4-1. Nodes 1 and 2 are the two profile
#: points, so the wheel-facing edge is **S1** — not the S3 the hexahedron uses. Getting this
#: wrong declares the indenter's back edge as the master surface, which touches nothing and
#: reports a wheel that fell through the floor.
CONTACT_FACE_2D = 1

#: Local C3D8 face carrying the contact surface, given the node ordering built below.
#: CalculiX numbers hexahedron faces S1 = 1-2-3-4, S2 = 5-8-7-6, S3 = 1-5-6-2,
#: S4 = 2-6-7-3, S5 = 3-7-8-4, S6 = 4-8-5-1. Nodes 1,2 are the profile points and 5,6 are
#: the same points on the next sweep station, so the wheel-facing face is S3. The winding
#: fix below swaps the two halves of the connectivity, which maps S3 onto the same four
#: nodes, so this holds either way.
CONTACT_FACE = 3


@dataclass(frozen=True, slots=True)
class IndenterMesh:
    """A rigid contact body, in metres."""

    #: (n, 3) node coordinates. z is identically 0 for the plane-strain body.
    nodes_m: np.ndarray
    #: (m, 8) 1-based node indices in C3D8 order, or (m, 4) in CPE4 order for the
    #: plane-strain body. Positive Jacobian either way.
    elements: np.ndarray
    #: 1-based element indices whose :attr:`contact_face` is part of the master surface.
    contact_elements: np.ndarray
    #: Reference node position for ``*RIGID BODY``. Placed on the contact surface at the
    #: mid-plane so its displacement *is* the indentation depth, with no lever arm.
    ref_point_m: np.ndarray
    kind: LoadCaseKind
    element_type: str = "C3D8"
    #: Local face carrying the master surface. Differs between the two element types, so it
    #: travels with the mesh rather than being a module constant the deck has to guess.
    contact_face: int = CONTACT_FACE

    @property
    def n_nodes(self) -> int:
        return len(self.nodes_m)

    @property
    def n_elements(self) -> int:
        return len(self.elements)


def _flat_profile(spec: IndenterSpec, y0: float) -> np.ndarray:
    """Straight contact line under the wheel, from -x to +x."""
    n = max(2, int(np.ceil(2 * spec.half_length_m / spec.element_size_m)) + 1)
    x = np.linspace(-spec.half_length_m, spec.half_length_m, n)
    return np.stack([x, np.full_like(x, y0)], axis=1)


def _step_edge_profile(spec: IndenterSpec, y0: float) -> np.ndarray:
    """Step tread, filleted convex corner, then the riser dropping away.

    Traversed in +x along the tread and then downward, so that rotating the tangent by -90
    degrees always points into the material.
    """
    r = spec.edge_fillet_m
    tread_end = -r
    n_tread = max(2, int(np.ceil((spec.half_length_m + tread_end) / spec.element_size_m)) + 1)
    tread_x = np.linspace(-spec.half_length_m, tread_end, n_tread)
    tread = np.stack([tread_x, np.full_like(tread_x, y0)], axis=1)

    if r > 0.0:
        # Quarter arc from (-r, y0) to (0, y0 - r), centred at (-r, y0 - r).
        theta = np.linspace(np.pi / 2, 0.0, spec.fillet_segments + 1)[1:]
        arc = np.stack([-r + r * np.cos(theta), (y0 - r) + r * np.sin(theta)], axis=1)
    else:
        arc = np.zeros((0, 2))

    riser_top = y0 - r
    riser_bottom = y0 - spec.step_height_m
    n_riser = max(2, int(np.ceil((riser_top - riser_bottom) / spec.element_size_m)) + 1)
    riser_y = np.linspace(riser_top, riser_bottom, n_riser)[1:]
    riser = np.stack([np.zeros_like(riser_y), riser_y], axis=1)

    return np.vstack([tread, arc, riser])


def _inward_normals(profile: np.ndarray) -> np.ndarray:
    """Unit normals pointing away from the wheel, i.e. into the indenter's material.

    The profile is traversed so that rotating the tangent by -90 degrees, ``(tx, ty) ->
    (ty, -tx)``, points inward. Vertex normals average the two adjacent segments so the
    swept block does not self-intersect at the corner.
    """
    seg = np.diff(profile, axis=0)
    lengths = np.linalg.norm(seg, axis=1, keepdims=True)
    seg_unit = seg / np.where(lengths > 0, lengths, 1.0)
    seg_normal = np.stack([seg_unit[:, 1], -seg_unit[:, 0]], axis=1)

    normals = np.zeros_like(profile)
    normals[0] = seg_normal[0]
    normals[-1] = seg_normal[-1]
    normals[1:-1] = seg_normal[:-1] + seg_normal[1:]
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.where(norms > 0, norms, 1.0)


def _safe_offsets(profile: np.ndarray, thickness: float) -> np.ndarray:
    """Per-point offset distance that cannot turn the block inside out.

    Offsetting a *convex* corner inward by more than its radius of curvature folds the
    inner boundary through itself and produces negative-Jacobian elements. The step edge
    hits this immediately: a 6 mm block thickness around a 1 mm fillet inverts every
    element on the arc. Where the profile turns convexly, the offset is capped at half the
    local radius of curvature; everywhere else the full thickness is used.

    The resulting inner surface is uneven, which does not matter: the body is rigid, so
    nothing is resolved through its thickness and only the contact face has any influence.
    """
    t = np.full(len(profile), float(thickness))
    seg = np.diff(profile, axis=0)
    lengths = np.linalg.norm(seg, axis=1)
    unit = seg / np.where(lengths > 0, lengths, 1.0)[:, None]

    for i in range(1, len(profile) - 1):
        a, b = unit[i - 1], unit[i]
        cross = a[0] * b[1] - a[1] * b[0]
        turn = float(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0)))
        # cross < 0 is a convex turn under this traversal: the inward normals converge.
        if cross < 0 and turn > 1e-9:
            radius = min(lengths[i - 1], lengths[i]) / (2.0 * np.sin(turn / 2.0))
            t[i] = min(t[i], 0.5 * radius)
    return t


def _hex_volume(nodes: np.ndarray) -> float:
    """Signed volume of one hexahedron, via decomposition into five tetrahedra."""
    tets = ((0, 1, 3, 4), (1, 2, 3, 6), (1, 3, 4, 6), (1, 4, 5, 6), (3, 4, 6, 7))
    total = 0.0
    for a, b, c, d in tets:
        m = np.stack([nodes[b] - nodes[a], nodes[c] - nodes[a], nodes[d] - nodes[a]])
        total += float(np.linalg.det(m)) / 6.0
    return total


def _quad_area(nodes: np.ndarray) -> float:
    """Signed area of one quadrilateral, shoelace over its four corners."""
    x, y = nodes[:, 0], nodes[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def _plane_strain_indenter(
    kind: LoadCaseKind,
    spec: IndenterSpec,
    profile: np.ndarray,
    inner: np.ndarray,
    y0: float,
) -> IndenterMesh:
    """One strip of CPE4 quads along the profile — the 2-D counterpart of the swept block.

    Node ordering per quad is ``outer[i], outer[i+1], inner[i+1], inner[i]``, so the
    wheel-facing edge is nodes 1-2, i.e. :data:`CONTACT_FACE_2D`.
    """
    n_p = len(profile)
    nodes = np.zeros((2 * n_p, 3), dtype=np.float64)
    nodes[:n_p, :2] = profile
    nodes[n_p:, :2] = inner

    idx = np.arange(n_p - 1)
    quads = np.stack(
        [idx + 1, idx + 2, n_p + idx + 2, n_p + idx + 1], axis=1
    ).astype(np.int64)

    # Same orientation guard as the hexahedron: CalculiX rejects a negative Jacobian.
    # Reversing the node order flips the sign and maps edge 1-2 onto 4-1, so the contact
    # face has to be re-derived rather than assumed — hence the assertion below.
    if _quad_area(nodes[quads[0] - 1]) < 0:
        quads = quads[:, ::-1]
        # After reversal the profile pair sits at local nodes 3-4, which is face S3.
        contact_face = 3
    else:
        contact_face = CONTACT_FACE_2D

    ref = (
        np.array([0.0, y0, 0.0])
        if kind is LoadCaseKind.RADIAL_FLAT
        else np.array([0.0, y0 - spec.edge_fillet_m, 0.0])
    )
    return IndenterMesh(
        nodes_m=nodes,
        elements=quads,
        contact_elements=np.arange(1, len(quads) + 1, dtype=np.int64),
        ref_point_m=ref,
        kind=kind,
        element_type="CPE4",
        contact_face=contact_face,
    )


def build_indenter(
    kind: LoadCaseKind,
    spec: IndenterSpec,
    wheel_radius_m: float,
    wheel_width_m: float,
    initial_gap_m: float = 1e-4,
    dimension: int = 3,
) -> IndenterMesh:
    """Mesh the rigid body for one load case.

    Args:
        kind: which contact geometry.
        spec: extents, fillet and element size.
        wheel_radius_m: sets the standoff so the body starts just clear of the tread.
        wheel_width_m: the sweep must overhang the wheel so the patch never runs off it.
            Ignored when ``dimension == 2`` — there is no width to overhang.
        initial_gap_m: clearance at t=0. Small but non-zero: starting in exact contact
            makes the first increment a zero-stiffness problem.
        dimension: 3 sweeps the profile into C3D8 hexahedra; 2 leaves it as a single strip
            of CPE4 plane-strain quads for the tier in :mod:`wheelopt.fea.section2d`.

    Returns:
        An :class:`IndenterMesh` positioned below the wheel.
    """
    y0 = -(wheel_radius_m + initial_gap_m)

    if kind is LoadCaseKind.RADIAL_FLAT:
        profile = _flat_profile(spec, y0)
    elif kind is LoadCaseKind.RADIAL_STEP_EDGE:
        profile = _step_edge_profile(spec, y0)
    else:  # pragma: no cover - enum is exhaustive
        raise ValueError(f"no indenter for {kind}")

    normals = _inward_normals(profile)
    inner = profile + normals * _safe_offsets(profile, spec.thickness_m)[:, None]

    if dimension == 2:
        return _plane_strain_indenter(kind, spec, profile, inner, y0)

    half_width = max(spec.half_width_m, 0.5 * wheel_width_m + 2 * spec.element_size_m)
    n_z = max(2, int(np.ceil(2 * half_width / spec.element_size_m)) + 1)
    z = np.linspace(-half_width, half_width, n_z)

    n_p = len(profile)
    # Node ordering: for each sweep station, the outer profile then the inner offset.
    nodes = np.zeros((n_z * 2 * n_p, 3), dtype=np.float64)
    for k, zk in enumerate(z):
        base = k * 2 * n_p
        nodes[base : base + n_p, :2] = profile
        nodes[base + n_p : base + 2 * n_p, :2] = inner
        nodes[base : base + 2 * n_p, 2] = zk

    def outer(k: int, i: int) -> int:
        return k * 2 * n_p + i + 1  # 1-based

    def inner_id(k: int, i: int) -> int:
        return k * 2 * n_p + n_p + i + 1

    elements = []
    for k in range(n_z - 1):
        for i in range(n_p - 1):
            elements.append(
                [
                    outer(k, i), outer(k, i + 1), inner_id(k, i + 1), inner_id(k, i),
                    outer(k + 1, i), outer(k + 1, i + 1),
                    inner_id(k + 1, i + 1), inner_id(k + 1, i),
                ]
            )
    elems_arr = np.array(elements, dtype=np.int64).reshape(-1, 8)

    # A negative Jacobian is rejected by CalculiX. Swapping the two halves of the
    # connectivity flips the orientation and leaves face S3 on the same four nodes, so the
    # contact surface declaration is unaffected.
    if _hex_volume(nodes[elems_arr[0] - 1]) < 0:
        elems_arr = elems_arr[:, [4, 5, 6, 7, 0, 1, 2, 3]]

    contact_elements = np.arange(1, len(elems_arr) + 1, dtype=np.int64)

    # Reference node on the contact surface at the wheel's centreline. For the flat plate
    # that is directly under the hub; for the step edge it is the corner itself, which is
    # where the load is actually reacted.
    if kind is LoadCaseKind.RADIAL_FLAT:
        ref = np.array([0.0, y0, 0.0])
    else:
        ref = np.array([0.0, y0 - spec.edge_fillet_m, 0.0])

    return IndenterMesh(
        nodes_m=nodes,
        elements=elems_arr,
        contact_elements=contact_elements,
        ref_point_m=ref,
        kind=kind,
    )
