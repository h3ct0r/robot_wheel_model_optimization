"""Compliant-spoke wheel geometry (family ``T3``) — build123d adapter. See ADR-0003.

This module is a **thin adapter**. All geometry maths lives in
:mod:`wheelopt.cad.centreline` (pure numpy, tested without OCCT); everything here does is
turn point arrays into OCCT faces and solids. Keep it that way — it is the one layer that
cannot be tested without a heavyweight dependency, so it should contain as little logic as
possible.

Construction strategy: build the wheel's 2D cross-section in the XY plane as three
independent sketches (shear band, hub, spokes), extrude each, then union. Each boolean is
between simple solids, which is far more robust in OCCT than one compound sketch. Spokes
overlap their attachments slightly, because coincident faces are the classic cause of union
failures.

build123d is imported lazily so that importing ``wheelopt.cad`` for constraint screening
does not require OCCT.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from .centreline import attachment_overlap_mm, spoke_outline
from .constraints import Severity, Violation, check_design, is_feasible
from .export import ANGULAR_TOLERANCE_RAD, remesh, weld_vertices
from .materials import MaterialSpec
from .params import WheelParams

if TYPE_CHECKING:  # pragma: no cover
    from build123d import Part

__all__ = ["BuildResult", "MissingCadKernel", "build_wheel", "tessellate"]


class MissingCadKernel(ImportError):
    """Raised when build123d is unavailable. Installation is an environment problem."""


@dataclass(frozen=True, slots=True)
class BuildResult:
    """Outcome of a geometry build.

    ``part`` is ``None`` when the design was rejected by screening. That is a normal,
    expected outcome — infeasible designs return a typed result, they do not raise
    (invariant 3).
    """

    part: Part | None
    violations: list[Violation]
    brep_volume_m3: float | None

    @property
    def ok(self) -> bool:
        return self.part is not None


def _require_build123d() -> Any:
    try:
        import build123d as bd
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise MissingCadKernel(
            "build123d is required for geometry generation but is not installed.\n"
            "  pip install build123d\n"
            "Constraint screening and mass properties work without it; only this module "
            "and scripts/verify_cad.py need the CAD kernel."
        ) from exc
    return bd


def build_wheel(
    params: WheelParams,
    material: MaterialSpec,
    *,
    skip_screening: bool = False,
) -> BuildResult:
    """Build one compliant-spoke wheel.

    Args:
        params: geometry, millimetres.
        material: used only for screening here; mass properties are applied downstream.
        skip_screening: build even if screening rejects the design. Useful for inspecting
            *why* a design was rejected, and for the discretisation audit. Never set this
            in a campaign.

    Returns:
        :class:`BuildResult`. Check ``.ok`` before using ``.part``.

    Raises:
        MissingCadKernel: if build123d is not installed.
    """
    violations = check_design(params, material)
    if not skip_screening and not is_feasible(violations):
        return BuildResult(part=None, violations=violations, brep_volume_m3=None)

    if any(x.severity is Severity.DEGENERATE for x in violations):
        # Degenerate geometry must never reach OCCT — it produces confusing kernel errors
        # rather than an actionable message.
        return BuildResult(part=None, violations=violations, brep_volume_m3=None)

    bd = _require_build123d()

    half_w = params.width_mm / 2.0
    overlap = attachment_overlap_mm(params)

    # --- shear band -------------------------------------------------------------------
    # Omitted entirely when `rim_thickness_mm` is zero: the annulus would have zero area,
    # and the spoke tips are the running surface instead. Skipping the solid rather than
    # building a degenerate one keeps the failure out of OCCT.
    rim_part = None
    if params.has_shear_band:
        with bd.BuildPart() as rim_builder:
            with bd.BuildSketch(bd.Plane.XY):
                bd.Circle(radius=params.outer_radius_mm)
                bd.Circle(radius=params.rim_inner_radius_mm, mode=bd.Mode.SUBTRACT)
            bd.extrude(amount=half_w, both=True)
        rim_part = rim_builder.part

    # --- hub --------------------------------------------------------------------------
    with bd.BuildPart() as hub_builder:
        with bd.BuildSketch(bd.Plane.XY):
            bd.Circle(radius=params.hub_radius_mm)
            bd.Circle(radius=params.hub_bore_radius_mm, mode=bd.Mode.SUBTRACT)
        bd.extrude(amount=half_w, both=True)

    # --- spokes -----------------------------------------------------------------------
    # One sketch containing all spoke outlines, extruded once. The outlines are disjoint
    # (guaranteed by the interspoke-gap constraint), so this is a single clean extrude.
    with bd.BuildPart() as spokes_builder:
        with bd.BuildSketch(bd.Plane.XY):
            for index in range(params.n_spokes):
                outline = spoke_outline(params, index, overlap_mm=overlap)
                pts = [(float(x), float(y)) for x, y in outline]
                with bd.BuildLine():
                    bd.Polyline(*pts, close=True)
                bd.make_face()
        bd.extrude(amount=half_w, both=True)

    # Fused left-to-right in the same order as before so a banded wheel is bit-for-bit the
    # geometry it was; a bandless one simply starts from the hub.
    solids = [p for p in (rim_part, hub_builder.part, spokes_builder.part) if p is not None]
    wheel = solids[0]
    for solid in solids[1:]:
        wheel = wheel + solid

    # The bore is cut last so that spoke overlap can never intrude into the shaft.
    if params.hub_bore_radius_mm > 0.0:
        with bd.BuildPart() as bore_builder:
            with bd.BuildSketch(bd.Plane.XY):
                bd.Circle(radius=params.hub_bore_radius_mm)
            bd.extrude(amount=half_w * 1.1, both=True)
        wheel = wheel - bore_builder.part

    # --- tread ------------------------------------------------------------------------
    if params.tread_depth_mm > 0.0:
        wheel = _cut_tread(bd, wheel, params)

    brep_volume_m3 = float(wheel.volume) * 1e-9  # mm^3 -> m^3
    return BuildResult(part=wheel, violations=violations, brep_volume_m3=brep_volume_m3)


def _cut_tread(bd: Any, wheel: Part, params: WheelParams) -> Part:
    """Cut circumferential tread grooves into the outer surface.

    Straight circumferential grooves, deliberately: they are printable without support in
    the flat orientation, and they add lateral grip without introducing the sharp radial
    edges that provoke contact-solver artifacts (see docs/plan/10-reality-gap.md).
    """
    n_grooves = 3
    groove_width = params.width_mm / (2 * n_grooves + 1)

    with bd.BuildPart() as cutter:
        for i in range(n_grooves):
            z = -params.width_mm / 2 + groove_width * (2 * i + 1)
            with bd.BuildSketch(bd.Plane.XY.offset(z)):
                bd.Circle(radius=params.outer_radius_mm + 1.0)
                bd.Circle(
                    radius=params.outer_radius_mm - params.tread_depth_mm,
                    mode=bd.Mode.SUBTRACT,
                )
            bd.extrude(amount=groove_width)
    return wheel - cutter.part


def tessellate(
    part: Part,
    tolerance_mm: float = 0.05,
    angular_tolerance_rad: float = ANGULAR_TOLERANCE_RAD,
) -> tuple[np.ndarray, np.ndarray]:
    """Tessellate a solid to a **welded** triangle mesh in **metres**.

    Meshing goes through :func:`wheelopt.cad.export.remesh` rather than ``Shape.tessellate``
    so that the two tolerances below are what actually determine the mesh; see that
    function for why the build123d path silently ignores them.

    OCCT meshes each BREP face independently, so vertices on a shared edge are emitted
    once per adjacent face — about 60% of the raw output is duplicates, and the mesh, while
    geometrically closed, has no shared edges at all. Welding is what makes the result
    manifold, and therefore what makes :func:`wheelopt.cad.export.is_watertight` meaningful
    and convex decomposition possible. Mass properties are the same either way.

    Args:
        part: the solid.
        tolerance_mm: maximum chordal deviation, absolute.
        angular_tolerance_rad: maximum angular deviation between adjacent facet normals.
            On a wheel this is the *dominant* control — the surfaces that carry the volume
            error are cylindrical, so facet count tracks angle, not chord length.

    Returns:
        ``(vertices_m, faces)`` ready for :func:`wheelopt.cad.massprops.mass_properties`.
    """
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_Orientation
    from OCP.TopLoc import TopLoc_Location

    remesh(part, tolerance_mm, angular_tolerance_rad)

    chunks_v: list[np.ndarray] = []
    chunks_f: list[np.ndarray] = []
    offset = 0
    for face in part.faces():
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face.wrapped, loc)
        if poly is None:
            continue
        trsf = loc.Transformation()

        nodes = [poly.Node(i).Transformed(trsf) for i in range(1, poly.NbNodes() + 1)]
        chunks_v.append(np.array([[p.X(), p.Y(), p.Z()] for p in nodes], dtype=np.float64))

        tris = np.array(
            [[t.Value(1), t.Value(2), t.Value(3)] for t in poly.Triangles()],
            dtype=np.int64,
        ).reshape(-1, 3)
        # OCCT stores triangles in the face's parametric orientation; a REVERSED face must
        # have its winding flipped or the outward normal — and hence the sign of the volume
        # integral over that face — is inverted.
        if face.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
            tris = tris[:, [0, 2, 1]]
        chunks_f.append(tris - 1 + offset)
        offset += poly.NbNodes()

    if not chunks_v:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 3), dtype=np.int64)

    v = np.vstack(chunks_v) * 1e-3  # mm -> m
    f = np.vstack(chunks_f)
    return weld_vertices(v, f)
