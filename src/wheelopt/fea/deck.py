"""Assemble a CalculiX input deck. Pure text in, pure text out — no solver, no gmsh.

Being a pure function of (mesh, indenter, material, load case) is what makes this testable
on a machine with neither dependency installed, and what makes a golden-file test
meaningful.

Design notes that are not obvious from the output:

**The indenter is a rigid body with a reference node.** CalculiX has no analytical rigid
surfaces, so the plate has to be meshed; making it a ``*RIGID BODY`` means one
``*NODE PRINT, NSET=NREF`` request yields the imposed displacement and the total reaction
force as two scalars per time point. That pair *is* the k_r(delta) table. The alternative —
summing reactions over a boundary node set — needs sign conventions and set bookkeeping
that are easy to get subtly wrong.

**Node-to-surface contact with the tread as slave.** A ``TYPE=NODE`` surface is just a node
set, which sidesteps mapping mesh boundary triangles back to their parent tetrahedron's
local face number entirely. The master is the indenter, whose face numbering this package
controls. It also yields per-slave-node ``CSTR``, which is exactly what the contact-patch
extraction wants.

**Displacement control and ``*TIME POINTS``.** See :class:`~wheelopt.fea.loadcase.LoadCase`
for why load control cannot work here, and why a solver-chosen output grid would make the
stiffness curve irreproducible.

Units are SI throughout: metres, pascals, newtons. The CAD layer's millimetres stop at the
mesh boundary.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..cad.materials import MaterialSpec
from ..cad.params import WheelParams
from . import FEA_PIPELINE_VERSION
from .hyperelastic import HyperelasticModel
from .indenter import IndenterMesh
from .loadcase import LoadCase, SolverSpec
from .mesh import FeaMesh

__all__ = ["DeckError", "DeckBundle", "build_deck"]

#: Young's modulus and Poisson ratio for the indenter. It is inside a ``*RIGID BODY``, so
#: these are never used to compute anything; CalculiX simply requires every element to
#: reference a material.
RIGID_E_PA = 2.1e11
RIGID_NU = 0.3

WHEEL_MATERIAL = "WHEEL"
RIGID_MATERIAL = "RIGID"


class DeckError(ValueError):
    """The deck cannot be assembled. Typed as ``FeaStatus.DECK_INVALID`` by the runner."""


@dataclass(frozen=True, slots=True)
class DeckBundle:
    """A deck plus the node ids needed to make sense of the results it produces."""

    text: str
    #: 1-based id of the rigid-body reference node, in deck numbering.
    ref_node: int
    #: 1-based ids of the tread slave nodes, in deck numbering.
    slave_nodes: np.ndarray
    #: Coordinates of those slave nodes, metres, for contact-patch geometry.
    slave_coords_m: np.ndarray
    n_nodes: int
    n_elements: int


def _fmt(x: float) -> str:
    return f"{x:.9e}"


def _node_block(nodes_m: np.ndarray, first_id: int) -> list[str]:
    return [
        f"{first_id + i}, {_fmt(p[0])}, {_fmt(p[1])}, {_fmt(p[2])}"
        for i, p in enumerate(nodes_m)
    ]


def _element_block(elements: np.ndarray, first_id: int, node_offset: int) -> list[str]:
    """Elements, continued onto a second line past 15 fields (CalculiX's line limit)."""
    lines = []
    for i, conn in enumerate(elements):
        ids = [str(int(n) + node_offset) for n in conn]
        head = f"{first_id + i}, " + ", ".join(ids[:15])
        if len(ids) > 15:
            lines.append(head + ",")
            lines.append(" " + ", ".join(ids[15:]))
        else:
            lines.append(head)
    return lines


def _set_block(keyword: str, name: str, ids: np.ndarray, per_line: int = 12) -> list[str]:
    lines = [f"*{keyword}, {keyword}={name}"]
    ids = np.asarray(ids, dtype=np.int64)
    for start in range(0, len(ids), per_line):
        lines.append(" " + ", ".join(str(int(i)) for i in ids[start : start + per_line]))
    return lines


def build_deck(
    mesh: FeaMesh,
    indenter: IndenterMesh | None,
    params: WheelParams,
    material: MaterialSpec,
    hyper: HyperelasticModel,
    load_case: LoadCase,
    solver: SolverSpec,
    *,
    design_hash: str = "",
    cache_key: str = "",
) -> DeckBundle:
    """Assemble the complete ``.inp`` for one load case.

    Two shapes of deck come out of here, chosen by
    :attr:`~wheelopt.fea.loadcase.LoadCaseKind.needs_indenter`:

    * **Contact.** The wheel is pressed against a meshed rigid indenter. ``indenter`` is
      required; ``NREF`` is the indenter's reference node.
    * **Prescribed tip.** No indenter, no contact pair, no friction. The tread node set is
      itself tied to ``NREF`` as a rigid body and that node is driven. ``indenter`` must be
      ``None``.

    The second exists because a ring segment *is* a rigid body on a slide, so driving the
    tread rigidly is the ring's kinematics written out in FEA — and because it removes the
    contact model from the measurement entirely, which is the only way to tell a structural
    answer from a contact one. ``NREF`` means the same thing in both, so nothing downstream
    of the solver changes.

    Raises:
        DeckError: if a required set is empty, or the indenter is present when it should not
            be (or absent when it should).
    """
    if mesh.n_elements == 0:
        raise DeckError("mesh has no elements")
    for name in ("bore", "tread"):
        if name not in mesh.node_sets or len(mesh.node_sets[name]) == 0:
            raise DeckError(f"mesh node set {name!r} is empty")
    if "spokes" not in mesh.element_sets or len(mesh.element_sets["spokes"]) == 0:
        raise DeckError("mesh element set 'spokes' is empty; stress output impossible")
    contact = load_case.kind.needs_indenter
    if contact and (indenter is None or indenter.n_elements == 0):
        raise DeckError(f"{load_case.kind.value} presses against an indenter and has none")
    if not contact and indenter is not None:
        raise DeckError(
            f"{load_case.kind.value} prescribes the tread directly and must not be given an "
            "indenter; an unused rigid body in the deck is a trap for the next reader"
        )

    plane_strain = mesh.element_type.startswith("CPE")

    # Deck numbering: wheel nodes 1..N, then indenter nodes, then the two rigid-body
    # control nodes. Elements likewise, wheel first.
    n_wheel_nodes = mesh.n_nodes
    ind_node_offset = n_wheel_nodes
    ind_first_node = ind_node_offset + 1
    n_ind_nodes = indenter.n_nodes if contact else 0
    ref_node = ind_node_offset + n_ind_nodes + 1
    rot_node = ref_node + 1
    n_nodes_total = rot_node

    n_wheel_elems = mesh.n_elements
    ind_first_elem = n_wheel_elems + 1

    slave = np.asarray(mesh.node_sets["tread"], dtype=np.int64)
    slave_coords = mesh.nodes_m[slave - 1]

    delta = load_case.delta_max_m
    # Contact stiffness scaled from the material and the mesh, never hard-coded: a fixed
    # value would make soft and stiff designs contact differently, which is invariant 2
    # violated in a way no plot would reveal.
    contact_k = (
        solver.contact_stiffness_factor * hyper.initial_youngs_pa / max(mesh_size(mesh), 1e-6)
    )

    lines: list[str] = [
        "** CalculiX deck generated by wheelopt",
        f"** fea pipeline : {FEA_PIPELINE_VERSION}",
        f"** design       : {design_hash}",
        f"** cache key    : {cache_key}",
        f"** load case    : {load_case.kind.value}",
        (f"** material     : {material.name} @ {material.infill_density:.0%} "
         f"{material.infill_pattern.value}"),
        f"** hyperelastic : {hyper.source}",
        "** units        : SI (m, Pa, N, s)",
        "**",
        "*NODE, NSET=NALL",
    ]
    lines += _node_block(mesh.nodes_m, 1)
    if contact:
        lines += _node_block(indenter.nodes_m, ind_first_node)
        anchor = indenter.ref_point_m
    else:
        # The rigid body's reference node. Put it at the centroid of the tread set rather
        # than anywhere convenient: a rigid body's reference node carries the rotation, so
        # an off-centre one turns a prescribed translation into a translation plus a moment.
        anchor = slave_coords.mean(axis=0)
    for node in (ref_node, rot_node):
        lines.append(f"{node}, {_fmt(anchor[0])}, {_fmt(anchor[1])}, {_fmt(anchor[2])}")

    lines.append(f"*ELEMENT, TYPE={mesh.element_type}, ELSET=EWHEEL")
    lines += _element_block(mesh.elements, 1, 0)
    if contact:
        lines.append(f"*ELEMENT, TYPE={indenter.element_type}, ELSET=EINDENT")
        lines += _element_block(indenter.elements, ind_first_elem, ind_node_offset)

    lines += _set_block("NSET", "NBORE", mesh.node_sets["bore"])
    lines += _set_block("NSET", "NTREAD", slave)
    lines.append(f"*NSET, NSET=NREF\n {ref_node}")
    lines.append(f"*NSET, NSET=NROT\n {rot_node}")
    lines += _set_block("ELSET", "ESPOKES", mesh.element_sets["spokes"])

    if contact:
        # Slave surface is a node set; master is the indenter faces we generated.
        lines.append("*SURFACE, NAME=SWHEEL, TYPE=NODE")
        lines.append(" NTREAD")
        lines.append("*SURFACE, NAME=SINDENT, TYPE=ELEMENT")
        for e in indenter.contact_elements:
            lines.append(f" {int(e) + n_wheel_elems}, S{indenter.contact_face}")

    # Plane-strain sections carry a thickness on the data line; CalculiX defaults it to 1 m
    # when omitted, which would report forces ~22x too large on a 45 mm wheel and look
    # entirely plausible. Both sections get the *same* thickness — the indenter is rigid so
    # its stiffness does not matter, but the contact area does, and area is length x
    # thickness. A mismatch would make the reaction force disagree with the contact force.
    section_data = [f" {_fmt(params.width_mm * 1e-3)}"] if plane_strain else []

    lines.append(hyper.calculix_card(WHEEL_MATERIAL))
    lines.append("*DENSITY")
    lines.append(f" {_fmt(material.effective_density_kg_m3(params.spoke_thickness_mm))}")
    lines.append(f"*SOLID SECTION, ELSET=EWHEEL, MATERIAL={WHEEL_MATERIAL}")
    lines += section_data

    if contact:
        lines.append(f"*MATERIAL, NAME={RIGID_MATERIAL}")
        lines.append("*ELASTIC")
        lines.append(f" {_fmt(RIGID_E_PA)}, {RIGID_NU}")
        lines.append(f"*SOLID SECTION, ELSET=EINDENT, MATERIAL={RIGID_MATERIAL}")
        lines += section_data
        lines.append(f"*RIGID BODY, ELSET=EINDENT, REF NODE={ref_node}, ROT NODE={rot_node}")

        lines.append("*SURFACE INTERACTION, NAME=SI1")
        lines.append("*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR")
        lines.append(f" {_fmt(contact_k)}")
        if load_case.friction_mu > 0:
            lines.append("*FRICTION")
            # Stick slope: contact stiffness scaled down keeps the tangential problem
            # conditioned without making stick artificially compliant.
            lines.append(f" {load_case.friction_mu}, {_fmt(contact_k * 0.05)}")
        lines.append("*CONTACT PAIR, INTERACTION=SI1, TYPE=NODE TO SURFACE")
        lines.append(" SWHEEL, SINDENT")
    else:
        # The tread *is* the rigid body. That is the modelling claim, not a convenience: a
        # ring segment is rigid and rides on a slide, so tying the tip to one driven node
        # reproduces the ring's kinematics exactly, and the reaction at that node is the
        # segment force the ROM wants. It also means the tip cannot rotate or deform
        # locally, which is the difference from a plate and the reason both are measured.
        lines.append(f"*RIGID BODY, NSET=NTREAD, REF NODE={ref_node}, ROT NODE={rot_node}")

    # The axle. Fixing the whole bore surface is slightly stiffer near the hub than a
    # kinematically coupled shaft would be; negligible for radial stiffness, and it avoids
    # introducing a coupling constraint whose own compliance would need justifying.
    lines.append("*BOUNDARY")
    # Plane-strain nodes have no third degree of freedom of their own — CalculiX expands
    # them into a 3-D layer and supplies the z constraint itself. Asking for DOF 3 here is
    # rejected outright.
    lines.append(" NBORE, 1, 2, 0.0" if plane_strain else " NBORE, 1, 3, 0.0")
    lines.append(f" {rot_node}, 1, 3, 0.0")
    # Which reference-node DOF is driven, and — the part that is easy to get wrong — which of
    # the others are held.
    #
    # Radial: drive 2, hold 1. A ring segment on a radial slide has no tangential freedom, so
    # holding it is the model, not a convenience.
    #
    # Tangential: drive 1 and leave **2 free**. This was first written holding 2 as well, on
    # the reasoning that a free radial coordinate would let the claw "relieve" the push. That
    # reasoning is backwards and the number said so: a claw bending tangentially sweeps its
    # tip along an *arc*, so it must come radially inward, and forbidding that forces the claw
    # to stretch along its own axis instead. It then reports the axial mode — measured
    # 7.35 N/mm against a beam-theory 0.0585, off by 125x, and constant with displacement
    # because nothing was bending. With 2 free the measurement is the tip compliance
    # ``1/C_tt``, which is what a step edge actually loads.
    driven = 1 if load_case.kind.is_tangential else 2
    held = (3,) if load_case.kind.is_tangential else (1, 3)
    for dof in held:
        if dof == 3 and plane_strain:
            continue  # CalculiX supplies the out-of-plane constraint itself; DOF 3 is rejected
        lines.append(f" {ref_node}, {dof}, {dof}, 0.0")

    lines.append("*AMPLITUDE, NAME=SWEEP")
    lines.append(" 0.0, 0.0, 1.0, 1.0, 2.0, 0.0")

    # Exactly ONE *TIME POINTS set, deliberately. CalculiX does not honour the
    # `TIME POINTS=<name>` on individual output requests when more than one set is defined:
    # the **last set defined wins for every request**, whatever name it was given. A
    # separate sparse set for stress output therefore does not reduce stress output — it
    # silently reduces *everything* to the sparse grid. Measured directly: two sets (8-point
    # and 2-point) produced 2 output times for both displacement and stress; one 8-point set
    # produced 8 for both. The cost of the single set is a larger .dat, since element stress
    # is now written at every sample; that is bounded by restricting *EL PRINT to ESPOKES.
    tp = load_case.time_points()
    lines.append("*TIME POINTS, NAME=TP")
    for start in range(0, len(tp), 8):
        lines.append(" " + ", ".join(f"{t:g}" for t in tp[start : start + 8]))

    lines.append(f"*STEP, NLGEOM, INC={solver.max_increments}")
    lines.append("*STATIC")
    lines.append(
        f" {solver.initial_increment}, {load_case.step_period}, "
        f"{solver.min_increment}, {solver.max_increment}"
    )
    lines.append("*BOUNDARY, AMPLITUDE=SWEEP")
    lines.append(f" {ref_node}, {driven}, {driven}, {_fmt(delta)}")

    lines.append("*NODE PRINT, NSET=NREF, TIME POINTS=TP, TOTALS=ONLY")
    lines.append(" RF")
    lines.append("*NODE PRINT, NSET=NREF, TIME POINTS=TP")
    lines.append(" U")
    if contact:
        lines.append("*CONTACT PRINT, TIME POINTS=TP")
        lines.append(" CSTR, CDIS, CNUM")
    lines.append("*EL PRINT, ELSET=ESPOKES, TIME POINTS=TP")
    lines.append(" S")
    lines.append("*NODE FILE, TIME POINTS=TP")
    lines.append(" U")
    lines.append("*EL FILE, TIME POINTS=TP")
    lines.append(" S")
    lines.append("*END STEP")

    return DeckBundle(
        text="\n".join(lines) + "\n",
        ref_node=ref_node,
        slave_nodes=slave,
        slave_coords_m=slave_coords,
        n_nodes=n_nodes_total,
        n_elements=n_wheel_elems + (indenter.n_elements if contact else 0),
    )


def mesh_size(mesh: FeaMesh) -> float:
    """Representative element size, metres. Used to scale the contact stiffness."""
    corners = mesh.nodes_m[mesh.elements[:, :4] - 1]
    edge = np.linalg.norm(corners[:, 1] - corners[:, 0], axis=1)
    return float(np.median(edge)) if len(edge) else 1e-3
