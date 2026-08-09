"""What to simulate, against what, and how finely. Pure configuration, no solver.

The load-case suite in docs/plan/06-compliance-rom.md is five cases; ``fea-0.1.0``
implements the two that first-week step 3 calls for — radial compression against a flat
plate and against a step edge. The step edge is the one the NPT literature omits and the
one that actually decides obstacle climbing.

Every field here enters the cache key, so adding a field with a default silently
invalidates nothing — bump ``FEA_PIPELINE_VERSION`` when the *meaning* of a field changes.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

__all__ = [
    "LoadCaseKind",
    "IndenterSpec",
    "LoadCase",
    "MeshSpec",
    "SolverSpec",
    "CONTACT_ANGLE_DEG",
    "phase_for_tip_contact",
]

#: Where on the wheel the indenter bears, measured from +x. Every load case approaches
#: along -y, so the contact point is at the bottom of the wheel.
CONTACT_ANGLE_DEG = -90.0


def phase_for_tip_contact(n_spokes: int, *, on_tip: bool = True) -> float:
    """``WheelParams.spoke_phase_deg`` that aims a spoke tip — or a gap — at the indenter.

    Only matters for a bandless wheel (``rim_thickness_mm == 0``), where the tips are the
    running surface: pressing on a tip and pressing on the gap between two tips are
    different experiments, and the default phase of 0 deg gives whichever one the spoke
    count happens to produce. For six spokes that is the gap, which would report a wheel
    far softer than it is for reasons having nothing to do with its design.

    Args:
        n_spokes: spoke count, used only for the half-pitch offset.
        on_tip: True aims a tip at the contact point (the stiff phase); False aims the
            midpoint between two tips at it (the soft phase).
    """
    if n_spokes < 1:
        raise ValueError("n_spokes must be at least 1")
    half_pitch = 180.0 / n_spokes
    return CONTACT_ANGLE_DEG + (0.0 if on_tip else half_pitch)


class LoadCaseKind(str, Enum):
    #: Compression against a flat rigid plate. The reference case; gives k_r(delta) and the
    #: contact patch that every ROM parameter is fitted to.
    RADIAL_FLAT = "radial_flat"
    #: Compression against the corner of a rigid step. The obstacle-climbing case.
    RADIAL_STEP_EDGE = "radial_step_edge"
    #: Push the tread **radially inward** by a prescribed displacement, with no plate and no
    #: contact at all. Only meaningful with ``MeshSpec.claw_sector``, where the tread is one
    #: claw's tip.
    #:
    #: This is the ring's own kinematics, expressed in FEA: a ring segment is a *rigid* body
    #: on a radial slide, so the tread node set is tied to a reference node as a rigid body
    #: and that node is driven. No contact means no penalty stiffness, no friction
    #: coefficient, and no stick/slip branch to pick — which is the point. Against the
    #: plate cases it is the clean check that a measured curve is structure and not contact.
    TIP_RADIAL = "tip_radial"
    #: The same, pushed **tangentially**. This is the degree of freedom the ring does not
    #: have (``docs/plan/TODO.md`` #20), and the one a claw is soft in: a claw points
    #: radially, so a radial load compresses it as a column and a tangential one bends it as
    #: a cantilever. Scoped at **576x** apart on the nominal claw, which is why it matters.
    TIP_TANGENTIAL = "tip_tangential"

    @property
    def needs_indenter(self) -> bool:
        """Whether this case presses the wheel against a meshed rigid body.

        The tip cases prescribe a displacement directly and have no contact, so the deck has
        no indenter, no surface interaction and no friction. Everything that reads a load
        curve is unchanged, because the driven reference node is still ``NREF``.
        """
        return self in (LoadCaseKind.RADIAL_FLAT, LoadCaseKind.RADIAL_STEP_EDGE)

    @property
    def is_tangential(self) -> bool:
        """Whether the prescribed displacement runs along the tread rather than into it."""
        return self is LoadCaseKind.TIP_TANGENTIAL


@dataclass(frozen=True, slots=True)
class IndenterSpec:
    """The rigid body the wheel is pressed against.

    CalculiX has no analytical rigid surfaces, so this is meshed as real C3D8 elements and
    tied to a reference node with ``*RIGID BODY``. That reference node is what makes
    extraction trivial: its reaction force *is* the total contact force, with no summation
    over node sets and no sign bookkeeping.
    """

    #: Half-extent of the plate along the rolling direction, metres. Must comfortably
    #: exceed the expected contact patch half-length.
    half_length_m: float = 0.045
    #: Half-extent across the wheel width, metres. Must exceed half the wheel width so the
    #: patch never runs off the master surface.
    half_width_m: float = 0.035
    #: Thickness of the meshed block, metres. Structurally irrelevant (it is rigid); kept
    #: small to keep the element count down.
    thickness_m: float = 0.006
    #: Height of the step for the step-edge case, metres. 50 mm per the first-week spike.
    step_height_m: float = 0.050
    #: Radius of the fillet on the step corner, metres. **Not cosmetic.** Node-to-face
    #: contact against a mathematically sharp 90-degree corner is a reliable source of
    #: non-convergence and of slave nodes slipping past the edge. Real steps are not sharp
    #: either. Recorded in the cache key because it changes the answer.
    edge_fillet_m: float = 0.001
    #: Facets around the fillet arc. Drives how smoothly the corner is represented.
    fillet_segments: int = 6
    #: Element size on the contact face, metres. Should be comparable to the wheel's tread
    #: element size or the node-to-face contact search behaves erratically.
    element_size_m: float = 0.002

    def __post_init__(self) -> None:
        if self.half_length_m <= 0 or self.half_width_m <= 0 or self.thickness_m <= 0:
            raise ValueError("indenter extents must be positive")
        if self.edge_fillet_m < 0:
            raise ValueError("edge_fillet_m must be non-negative")
        if self.fillet_segments < 1:
            raise ValueError("fillet_segments must be at least 1")


@dataclass(frozen=True, slots=True)
class LoadCase:
    """One quasi-static compression sweep.

    **Displacement-controlled, not force-controlled.** This is not a preference. A soft
    spoked structure passes through limit points where ``dF/ddelta`` goes negative; under
    force control the solve diverges exactly there, which is precisely the behaviour the
    case exists to observe. Under displacement control the limit point is traversed
    normally as long as F is single-valued in delta.

    The honest limitation: full snap-*back*, where delta itself is non-monotonic along the
    equilibrium path, needs an arc-length (Riks) solver. CalculiX does not have one, so a
    design that snaps back will still fail to converge and will be reported as
    ``SOLVER_DIVERGED``.

    Because loading is displacement-controlled, ``max_load_multiple`` cannot be an input.
    It is checked afterwards: if the sweep did not reach it, the result carries a warning
    ``Violation`` so the ROM fit knows the extrapolation limit of its own data.
    """

    kind: LoadCaseKind = LoadCaseKind.RADIAL_FLAT
    #: Static load per wheel, newtons. 10 kg all-up on four wheels = 24.5 N each.
    #: Provisional until configs/robot.yaml is frozen. Every sweep is scaled against this,
    #: so changing it changes what "3x nominal" means and invalidates cached results.
    nominal_load_n: float = 24.5
    #: The sweep should reach this multiple of nominal. Verified after the fact.
    max_load_multiple: float = 3.0
    #: Peak imposed displacement, metres. Chosen to overshoot 3x nominal on a soft design;
    #: on a stiff one the post-hoc check will flag the shortfall.
    delta_max_m: float = 0.012
    #: Samples on each branch. Output lands exactly on these via ``*TIME POINTS`` — with
    #: automatic incrementation, ``FREQUENCY=n`` would give a solver-dependent output grid
    #: and therefore a non-reproducible k_r(delta).
    n_points_per_branch: int = 20
    #: Coulomb friction between tread and indenter. TPU on a hard surface is high.
    friction_mu: float = 0.8
    indenter: IndenterSpec = IndenterSpec()

    def __post_init__(self) -> None:
        if self.delta_max_m <= 0:
            raise ValueError("delta_max_m must be positive")
        if self.n_points_per_branch < 2:
            raise ValueError("need at least 2 points per branch")
        if self.nominal_load_n <= 0:
            raise ValueError("nominal_load_n must be positive")
        if self.friction_mu < 0:
            raise ValueError("friction_mu must be non-negative")

    @property
    def target_load_n(self) -> float:
        return self.nominal_load_n * self.max_load_multiple

    @property
    def step_period(self) -> float:
        """Total pseudo-time. 0 -> 1 loads, 1 -> 2 unloads, in a single ``*STEP``."""
        return 2.0

    def time_points(self) -> list[float]:
        """Output sample times, ascending, excluding t=0 where nothing has happened."""
        n = self.n_points_per_branch
        return [round(i * (self.step_period / (2 * n)), 10) for i in range(1, 2 * n + 1)]

    def peak_load_time(self) -> float:
        """Pseudo-time at which the sweep reaches maximum indentation."""
        return 1.0


@dataclass(frozen=True, slots=True)
class MeshSpec:
    """Mesh density and element choice.

    Second-order tetrahedra (C3D10) throughout. Two reasons, and the second is the one that
    matters: CalculiX has **no hybrid elements**, so a near-incompressible hyperelastic
    material in fully-integrated elements locks and reports a stiffness that is too high but
    entirely plausible. Quadratic tets lock far less than linear ones. Shells for the thin
    spokes would need mid-surface extraction plus solid-shell ties — a day of work on its
    own, and not recoverable inside the step-3 budget.
    """

    #: Target element size on the spokes, metres. The spokes are the compliant structure;
    #: everything else is along for the ride. At 2 mm spoke thickness this wants to be
    #: small enough for ~2 quadratic elements across.
    size_spoke_m: float = 0.0012
    size_rim_m: float = 0.0018
    #: The hub is a solid annulus carrying ~40% of the volume and almost no strain. Coarse.
    size_hub_m: float = 0.0045
    #: 2 = C3D10. 1 = C3D4, which locks badly on incompressible material; available only
    #: for debugging mesh generation itself.
    order: int = 2
    #: gmsh 3D algorithm. 1 = Delaunay, single-threaded and **deterministic**. HXT (10) is
    #: multithreaded and gives different meshes run to run, which would break the Phase 0
    #: determinism gate and make every cache key a lie.
    algorithm_3d: int = 1
    #: Netgen volume optimisation. Measured deterministic, and improves element quality.
    optimize_netgen: bool = True
    #: gmsh high-order (mid-side node) optimisation. **Leave at 0** — it is the wrong tool
    #: for the actual problem and it fails two ways at once. Measured on this geometry,
    #: HighOrderOptimize=2:
    #:   * makes meshing **non-deterministic** — identical connectivity but node positions
    #:     differing by up to 0.13 mm run to run, defeating the Phase 0 determinism gate and
    #:     making the cache key describe a mesh that was not the one solved;
    #:   * intermittently **aborts the process** with an uncaught C++ exception
    #:     ("Failed to reach critical value ... ScaledJac"), which Python cannot catch, so
    #:     invariant 4 is powerless against it.
    high_order_optimize: int = 0
    #: Keep second-order mid-side nodes at the straight edge midpoints instead of curving
    #: them onto the surface. **This is what makes the mesh solvable.** Curved mid-side
    #: nodes on the tight bore and spoke fillets fold the quadratic tet — CalculiX then
    #: rejects it with "nonpositive jacobian determinant" and the whole solve dies at t=0.
    #: With straight edges, measured: 0 inverted elements (vs 9 without) *and* bit-identical
    #: meshing. The cost is slightly coarser representation of curved surfaces; on a wheel
    #: that is a small geometric error and a fully valid, reproducible mesh, which is the
    #: right trade.
    second_order_linear: bool = True
    #: Mesh only half the width and impose symmetry on the mid-plane. Would halve the DOF
    #: count, at the cost of suppressing antisymmetric modes — that is, lateral spoke
    #: buckling, which is exactly what this stage exists to detect.
    #:
    #: **NOT IMPLEMENTED.** The field is declared, plumbed through the CLI and hashed into
    #: the cache key, but neither :mod:`wheelopt.fea.mesh` nor :mod:`wheelopt.fea.deck` ever
    #: reads it — setting it produced a full-width model, at full cost, under a *different*
    #: cache key, and reported itself as a symmetric run. That is the exact failure this
    #: project keeps meeting (see the watch list in CLAUDE.md): a value that is a default,
    #: reads as innocuous, and means something else. :meth:`__post_init__` now rejects
    #: ``True`` rather than accepting it silently. The field is kept, not deleted, so that
    #: existing cache keys stay valid; implementing it means meshing z >= 0 only and adding
    #: ``*BOUNDARY`` z-symmetry on the mid-plane nodes.
    half_width_symmetry: bool = False
    #: Tolerance for classifying nodes onto the bore and tread surfaces, metres.
    surface_tolerance_m: float = 1e-4
    #: Which fidelity tier. ``3`` meshes the solid into C3D10 (:mod:`wheelopt.fea.mesh`);
    #: ``2`` meshes the cross-section into plane-strain CPE6
    #: (:mod:`wheelopt.fea.section2d`) and costs seconds rather than hours.
    #:
    #: The 2-D tier is a **deliberate fidelity reduction**, not an approximation that gets
    #: better with refinement: plane strain holds ``ε_zz = 0``, but a real wheel has free
    #: faces at ``z = ±W/2`` that bulge, so it reports a stiffness that is too high, and it
    #: cannot represent lateral spoke buckling at all. Use it to get the *shape* of
    #: ``k_r(δ)`` and to screen; calibrate the magnitude against the 3-D tier.
    dimension: int = 3
    #: Mesh **one claw and the hub**, not the whole wheel. Only for a bandless (`T7`) design,
    #: where the claws are independent and one of them is a model of a ring segment; with a
    #: band the claws share load through it and a single one is a model of nothing.
    #:
    #: This is what turns the ROM's segment law from a *deconvolution* of a whole-wheel
    #: ``F(δ)`` into a *measurement*: press one claw, and the curve you get is the segment
    #: spring law directly, with ``n_segments = n_spokes``. Measured on the nominal claw
    #: design at 2.5 mm elements: 3155 CPE6 for the wheel against **492** for one claw plus
    #: the full hub.
    #:
    #: The load case is unchanged — still a rigid flat plate — so this belongs to the *mesh*
    #: rather than to :class:`LoadCaseKind`. It reaches the cache key through this dataclass
    #: like every other field, which is the point of putting it here.
    claw_sector: bool = False
    #: Angular width of the hub wedge kept with the claw, degrees. ``None`` keeps the whole
    #: hub annulus and is **the default and the validated one**: the hub is then exactly the
    #: hub, and only the eleven unloaded claws are missing. A finite span makes cost
    #: independent of ``n_spokes``, but it cuts the bore arc the shaft constraint acts on —
    #: at 30° on this design that is two nodes, i.e. a pin, not a clamp — so
    #: :func:`~wheelopt.fea.section2d.mesh_claw_sector` refuses the narrow ones rather than
    #: solving them. Its effect on the answer has **not been measured**; do not use it for a
    #: result without doing that first.
    claw_hub_span_deg: float | None = None

    def __post_init__(self) -> None:
        if self.dimension not in (2, 3):
            raise ValueError("dimension must be 2 (plane strain) or 3 (solid)")
        if self.claw_sector and self.dimension != 2:
            raise NotImplementedError(
                "claw_sector is implemented on the plane-strain tier only; the 3-D sector "
                "would need the STEP sectioned rather than the centreline re-meshed"
            )
        if self.claw_hub_span_deg is not None and not self.claw_sector:
            raise ValueError("claw_hub_span_deg means nothing without claw_sector")
        if self.dimension == 2 and self.order != 2:
            raise ValueError("the plane-strain tier is second-order only (CPE6)")
        if self.order not in (1, 2):
            raise ValueError("order must be 1 (C3D4) or 2 (C3D10)")
        for name in ("size_spoke_m", "size_rim_m", "size_hub_m"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.surface_tolerance_m <= 0:
            raise ValueError("surface_tolerance_m must be positive")
        # Refuse rather than silently ignore. A spec is validated once, at construction,
        # before any compute is committed — this is not a per-design evaluation, so
        # invariant 4 does not apply (same reasoning as PlatformSpecError).
        if self.half_width_symmetry:
            raise NotImplementedError(
                "MeshSpec.half_width_symmetry is not implemented: the mesher and the deck "
                "generator both ignore it, so setting it gives a full-width model at full "
                "cost under a different cache key. Leave it False until mid-plane symmetry "
                "is actually built."
            )

    @property
    def element_type(self) -> str:
        if self.dimension == 2:
            return "CPE6"
        return "C3D10" if self.order == 2 else "C3D4"


@dataclass(frozen=True, slots=True)
class SolverSpec:
    """How to invoke CalculiX. Does not affect the physics, but does affect the cache key
    via the solver identity string — a different ccx version can give different answers."""

    #: Wall-clock budget per solve. A full-size wheel sweep is 10-40 minutes; the timeout
    #: exists to stop a pathological design from stalling a campaign, not to bound normal
    #: work.
    timeout_s: float = 3600.0
    n_threads: int = 4
    #: Initial, total, minimum and maximum increment for ``*STATIC``, in pseudo-time.
    initial_increment: float = 0.02
    min_increment: float = 1e-5
    max_increment: float = 0.05
    #: Maximum increments before CalculiX gives up.
    max_increments: int = 2000
    #: Contact stiffness multiplier. The absolute stiffness is derived from the material's
    #: initial modulus and the element size so that it scales with the design — a
    #: hard-coded contact stiffness would violate invariant 2 in a particularly sneaky way,
    #: since it would make soft and stiff designs contact differently.
    #:
    #: It scales as ``factor * E / max(element_size, contact_length_floor_m)``.
    #:
    #: **The default was 20 until 2026-08-09** (``TODO.md`` #12). Measured on both tiers
    #: before it moved: on plane strain 20/5/2 give 3.90/3.88/3.86 N, and on 3-D C3D10 the
    #: same ladder gives 4.29/4.26/4.18 N frictionless and 4.35/4.31/4.22 N at ``mu = 0.6``.
    #: So 20 to 5 costs **0.7-0.8%** of the answer on the reference tier and buys real
    #: conditioning — the frictional 3-D run goes from 60 increments with 3 cutbacks to 50
    #: with none, and a diverged frictional 2-D run converges. Going on to 2 is a different
    #: matter and was rejected: the answer moves 2.6-2.9% and the contact patch grows from
    #: 34.2 to 39.0 mm, which is penetration being reported as conformity.
    #:
    #: In the cache key, because it changes the answer. It was not, until a run at one
    #: factor was served another factor's cached result.
    contact_stiffness_factor: float = 5.0
    #: Smallest element size the penalty is allowed to see, metres. **This is the cap the
    #: scaling above needs**, and without it the factor alone cannot buy convergence on a fine
    #: mesh.
    #:
    #: Measured 2026-08-09 on the tiny design at ``mu = 0.6``, holding the *factor* at 5 and
    #: refining: 4.0 mm converges, 2.5 mm and 1.5 mm both diverge — and they diverge at
    #: ``factor = 20`` too, so lowering the factor is not the remedy. Holding the *penalty*
    #: instead, at the value a 4 mm element gives (``5 E / 0.004``, i.e. 1250 E per metre),
    #: every one of those meshes converges, and the two finest agree on the peak force to
    #: **0.02%** — 3.8898 against 3.8905 N. The divergence therefore tracks the absolute
    #: penalty, not the factor, and 4 mm is where this design's threshold sits.
    #:
    #: Calibrated, not derived: one design, one material, one load case. Re-check it before
    #: trusting it on a much larger wheel, where 4 mm is a relatively finer element. Set it to
    #: zero to recover the uncapped scaling.
    contact_length_floor_m: float = 0.004

    def __post_init__(self) -> None:
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if self.n_threads < 1:
            raise ValueError("n_threads must be at least 1")
        if not 0 < self.min_increment <= self.initial_increment <= self.max_increment:
            raise ValueError("require 0 < min <= initial <= max increment")
        if self.contact_stiffness_factor <= 0:
            raise ValueError("contact_stiffness_factor must be positive")
        if self.contact_length_floor_m < 0:
            raise ValueError("contact_length_floor_m must be non-negative")
