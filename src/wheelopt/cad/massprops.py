"""Mass properties from a closed triangle mesh.

Invariant 2 (CLAUDE.md): mass, inertia and stiffness are always **derived** from geometry
and material, never hard-coded. A constant-inertia bug is silent and fatal — the optimiser
discovers that large wheels are free and drives the radius to its upper bound.

Why mesh-based rather than reading OCCT's ``GProp``: this module is pure numpy, so it can
be unit-tested against analytic solids *without* an OCCT install, and the same code path
serves both the CAD stage and any externally supplied STL. The BREP volume is still used
as an independent cross-check (see :func:`check_against_brep_volume`) — if the tessellation
is too coarse, the two disagree and the design is rejected rather than silently mis-massed.

Algorithm: Eberly's polyhedral mass properties (divergence theorem over the triangulated
boundary). Exact for polyhedra, so the only error is tessellation of curved surfaces.

All inputs in **metres**, all outputs in **SI**. Convert at the boundary.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["MassProperties", "check_against_brep_volume", "mass_properties"]


@dataclass(frozen=True, slots=True)
class MassProperties:
    """Rigid-body mass properties in SI units, about the centre of mass."""

    volume_m3: float
    mass_kg: float
    #: Centre of mass in the mesh frame, metres.
    com_m: np.ndarray
    #: 3x3 inertia tensor about the centre of mass, kg*m^2.
    inertia_kg_m2: np.ndarray

    @property
    def principal_moments_kg_m2(self) -> np.ndarray:
        return np.linalg.eigvalsh(self.inertia_kg_m2)

    def summary(self) -> str:  # pragma: no cover - display only
        ix, iy, iz = np.diag(self.inertia_kg_m2)
        cx, cy, cz = self.com_m
        return (
            f"volume  {self.volume_m3 * 1e6:10.3f} cm^3\n"
            f"mass    {self.mass_kg * 1e3:10.3f} g\n"
            f"com     ({cx * 1e3:.3f}, {cy * 1e3:.3f}, {cz * 1e3:.3f}) mm\n"
            f"inertia diag ({ix:.6e}, {iy:.6e}, {iz:.6e}) kg m^2"
        )


def _subexpressions(w0: np.ndarray, w1: np.ndarray, w2: np.ndarray):
    """Eberly's per-axis integral subexpressions, vectorised over triangles."""
    temp0 = w0 + w1
    f1 = temp0 + w2
    temp1 = w0 * w0
    temp2 = temp1 + w1 * temp0
    f2 = temp2 + w2 * f1
    f3 = w0 * temp1 + w1 * temp2 + w2 * f2
    g0 = f2 + w0 * (f1 + w0)
    g1 = f2 + w1 * (f1 + w1)
    g2 = f2 + w2 * (f1 + w2)
    return f1, f2, f3, g0, g1, g2


def mass_properties(
    vertices: np.ndarray,
    faces: np.ndarray,
    density_kg_m3: float,
) -> MassProperties:
    """Compute mass properties of a closed triangle mesh.

    Args:
        vertices: ``(n, 3)`` float array of vertex positions in **metres**.
        faces: ``(m, 3)`` integer array of triangle vertex indices.
        density_kg_m3: homogenised density — see :mod:`wheelopt.cad.materials`.

    Returns:
        :class:`MassProperties` about the centre of mass, in SI.

    Raises:
        ValueError: if the mesh is empty or encloses zero volume. A zero-volume mesh is a
            pipeline bug, not an infeasible design, so this raises rather than returning a
            typed violation.

    Note:
        Winding is normalised automatically: if the signed volume comes out negative the
        result is negated, so inward-facing meshes give correct magnitudes.
    """
    v = np.asarray(vertices, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int64)

    if v.ndim != 2 or v.shape[1] != 3:
        raise ValueError(f"vertices must be (n, 3), got {v.shape}")
    if f.ndim != 2 or f.shape[1] != 3:
        raise ValueError(f"faces must be (m, 3), got {f.shape}")
    if len(f) == 0:
        raise ValueError("mesh has no faces")

    p0, p1, p2 = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]

    # Edge vectors and the (unnormalised) face normal.
    e1 = p1 - p0
    e2 = p2 - p0
    d = np.cross(e1, e2)  # (m, 3)

    fx = _subexpressions(p0[:, 0], p1[:, 0], p2[:, 0])
    fy = _subexpressions(p0[:, 1], p1[:, 1], p2[:, 1])
    fz = _subexpressions(p0[:, 2], p1[:, 2], p2[:, 2])

    f1x, f2x, f3x, g0x, g1x, g2x = fx
    _f1y, f2y, f3y, g0y, g1y, g2y = fy
    _f1z, f2z, f3z, g0z, g1z, g2z = fz

    d0, d1, d2 = d[:, 0], d[:, 1], d[:, 2]

    i0 = np.sum(d0 * f1x) / 6.0

    i1 = np.sum(d0 * f2x) / 24.0
    i2 = np.sum(d1 * f2y) / 24.0
    i3 = np.sum(d2 * f2z) / 24.0

    i4 = np.sum(d0 * f3x) / 60.0
    i5 = np.sum(d1 * f3y) / 60.0
    i6 = np.sum(d2 * f3z) / 60.0

    i7 = np.sum(d0 * (p0[:, 1] * g0x + p1[:, 1] * g1x + p2[:, 1] * g2x)) / 120.0
    i8 = np.sum(d1 * (p0[:, 2] * g0y + p1[:, 2] * g1y + p2[:, 2] * g2y)) / 120.0
    i9 = np.sum(d2 * (p0[:, 0] * g0z + p1[:, 0] * g1z + p2[:, 0] * g2z)) / 120.0

    # Normalise winding: a consistently inward-facing mesh flips every integral's sign.
    if i0 < 0.0:
        i0, i1, i2, i3, i4, i5, i6, i7, i8, i9 = (
            -i0, -i1, -i2, -i3, -i4, -i5, -i6, -i7, -i8, -i9
        )

    volume = float(i0)
    if volume <= 0.0 or not np.isfinite(volume):
        raise ValueError(
            f"mesh encloses no volume (got {volume:g} m^3) — not watertight, or degenerate"
        )

    com = np.array([i1, i2, i3], dtype=np.float64) / volume
    mass = volume * density_kg_m3

    # Inertia about the origin, unit density.
    ixx = i5 + i6
    iyy = i4 + i6
    izz = i4 + i5
    ixy = -i7
    iyz = -i8
    ixz = -i9

    # Parallel-axis shift to the centre of mass (still unit density).
    cx, cy, cz = com
    ixx -= volume * (cy * cy + cz * cz)
    iyy -= volume * (cz * cz + cx * cx)
    izz -= volume * (cx * cx + cy * cy)
    ixy += volume * cx * cy
    iyz += volume * cy * cz
    ixz += volume * cz * cx

    inertia = density_kg_m3 * np.array(
        [
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ],
        dtype=np.float64,
    )

    return MassProperties(
        volume_m3=volume,
        mass_kg=mass,
        com_m=com,
        inertia_kg_m2=inertia,
    )


def check_against_brep_volume(
    mesh_volume_m3: float,
    brep_volume_m3: float,
    tolerance: float = 0.01,
) -> tuple[bool, float]:
    """Cross-check tessellated volume against the exact BREP volume.

    Curved surfaces are under-represented by tessellation, so the mesh volume of a wheel is
    systematically *lower* than the BREP volume. A large discrepancy means the tessellation
    is too coarse for mass properties to be trusted — and since inertia scales with radius
    squared, a coarse mesh biases the dynamics in a direction the optimiser can exploit.

    Returns:
        ``(within_tolerance, relative_error)``. Relative error is signed: negative means
        the mesh under-reports volume, which is the expected direction.
    """
    if brep_volume_m3 <= 0.0:
        return False, float("nan")
    rel = (mesh_volume_m3 - brep_volume_m3) / brep_volume_m3
    return bool(abs(rel) <= tolerance), float(rel)
