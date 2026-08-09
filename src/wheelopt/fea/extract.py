"""Turn parsed CalculiX output into ROM parameters. Pure numpy.

Produces the subset of docs/plan/06-compliance-rom.md section 2 that first-week step 3
calls for: the radial stiffness curve, the contact patch against load, loaded rolling
radius, peak spoke stress, and buckling detection.

What is deliberately **not** produced here is the hysteresis loss factor. A hyperelastic
constitutive model is path-independent: the unloading branch retraces the loading branch by
construction, so any "measured" loop area is either numerical noise or evidence that the
structure changed equilibrium path. Fitting a damping coefficient to that would be
inventing a number. See :class:`~wheelopt.fea.results.FeaResult`.
"""

from __future__ import annotations

import numpy as np

from ..cad.constraints import Severity, Violation
from ..cad.params import WheelParams
from .loadcase import LoadCase
from .parse import DatBlock, collect
from .results import ContactPatch, LoadCurve

__all__ = [
    "build_load_curve",
    "build_contact_patch",
    "loaded_radius",
    "detect_buckling",
    "loop_area_fraction",
    "spoke_stress",
    "fea_violations",
    "TPU_FATIGUE_LIMIT_PA",
    "CONTACT_PRESSURE_FRACTION",
]

#: TPU fatigue limit, Pa. docs/plan/07-materials.md, Polymers 2023. Reported rather than
#: optimised; cracks initiate at surface micropore aggregations well below yield.
TPU_FATIGUE_LIMIT_PA = 10.25e6

#: A slave node counts as "in contact" above this fraction of the peak pressure at that
#: load. A fraction rather than an absolute threshold, because peak pressure spans orders
#: of magnitude across the design space and a fixed pascal cutoff would silently redefine
#: the patch as designs get softer.
CONTACT_PRESSURE_FRACTION = 0.02

#: The loading branch counts as buckled where its tangent stiffness drops below this fraction
#: of the stiffest tangent it had reached earlier on the branch.
#:
#: 10% is a judgement, but it sits in a wide gap between measured curves rather than being
#: picked out of the air. Three references, all from ``docs/experiments/log.md``:
#:
#: * The **plateau** case the old sign test missed: tangent 12.1 N/mm falling to **+0.086**,
#:   a ratio of 0.7%. Comfortably caught.
#: * The nominal design on the plane-strain tier at zero friction: tangent 42.8 N/mm falling
#:   to **-6.98**, ratio **-0.175**. Caught by this test and by the sign test it replaces —
#:   every curve the old rule caught, this one catches at the same point or earlier.
#: * The **tiny** design's monotonically stiffening sweep, which nobody would call buckled:
#:   its tangent never dips at all, minimum ratio **1.157**. It is not near the threshold in
#:   any sense; there is more than a decade of margin on both sides.
#:
#: So the exact value is not load-bearing. What matters is that the test is a *ratio* and not
#: the sign test it replaced.
BUCKLING_STIFFNESS_FRACTION = 0.10


def build_load_curve(
    blocks: list[DatBlock], load_case: LoadCase, ref_node: int,
    cross_inward_sign: float = 0.0,
) -> LoadCurve | None:
    """Reaction force against imposed displacement at the rigid-body reference node.

    Args:
        cross_inward_sign: ``+1`` or ``-1``, whichever turns the *undriven* in-plane component
            into an inward-positive displacement; ``0`` means don't report it. Only the
            tangential tip case leaves an axis free, and only that case has anything to say
            here. The sign has to be passed in because this function sees displacements and
            not coordinates, and guessing it from the driven axis would be wrong for half the
            possible spoke phases — see :attr:`~wheelopt.fea.results.LoadCurve.cross_delta_m`.
    """
    forces = collect(blocks, "total_force") or collect(blocks, "force")
    displacements = collect(blocks, "displacement")
    if not forces or not displacements:
        return None

    times = sorted(t for t in forces if t in displacements)
    if not times:
        return None

    # Which component is the loading direction. Every case drives y except the tangential
    # tip case, which drives x — and reading y there gives a curve whose displacement is
    # *identically zero* while the force column still fills with plausible rising numbers,
    # so the failure looks like a stiffness result rather than a units mistake. Measured
    # before it was fixed: 7.35 N/mm against a beam-theory 0.06.
    axis = 0 if load_case.kind.is_tangential else 1

    delta, force, cross = [], [], []
    for t in times:
        fb, db = forces[t], displacements[t]
        # A TOTALS=ONLY block has no meaningful node id; a per-node block does.
        f_row = _row_for(fb, ref_node)
        d_row = _row_for(db, ref_node)
        if f_row is None or d_row is None:
            continue
        # Magnitudes, so the curve is positive in compression.
        force.append(abs(float(f_row[axis])))
        delta.append(abs(float(d_row[axis])))
        # The other in-plane axis, kept **signed**: which way it went is the whole content.
        cross.append(cross_inward_sign * float(d_row[1 - axis]))

    if len(delta) < 2:
        return None

    d = np.array(delta, dtype=np.float64)
    f = np.array(force, dtype=np.float64)
    t = np.array(times, dtype=np.float64)
    # The amplitude peaks at t = 1: everything up to it is loading, the rest unloading.
    loading = t <= 1.0 + 1e-9
    return LoadCurve(
        delta_m=d, force_n=f, loading=loading,
        cross_delta_m=np.array(cross, dtype=np.float64) if cross_inward_sign else None,
    )


def _row_for(block: DatBlock, node: int) -> np.ndarray | None:
    if len(block.values) == 0:
        return None
    match = np.flatnonzero(block.ids == node)
    if len(match):
        return block.values[match[0]]
    # TOTALS=ONLY blocks carry a single summed row whose id is not a real node.
    return block.values[0] if len(block.values) == 1 else None


def build_contact_patch(
    blocks: list[DatBlock],
    curve: LoadCurve,
    slave_nodes: np.ndarray,
    slave_coords_m: np.ndarray,
    section_thickness_m: float | None = None,
) -> ContactPatch | None:
    """Patch length, width, area and peak pressure against load, on the loading branch.

    Length is measured along the rolling direction (x), width across the wheel (z). Area is
    estimated from the in-contact node count and the mean slave-node spacing rather than
    from facets, because the slave surface is a node set and carries no connectivity.

    Args:
        section_thickness_m: set for the plane-strain tier, where every slave node lies at
            ``z = 0``. Without it the 3-D estimate returns a **width of exactly zero** and an
            area of ``n_nodes x spacing^2`` built from an in-plane spacing that describes
            nothing — measured: a 32.5 mm patch reported as 32.5 x 0.0 mm at 21.4 kPa. In
            plane strain the out-of-plane extent is the section thickness by definition, so
            area is length x thickness and the pressure that follows is comparable with the
            3-D one.
    """
    stresses = collect(blocks, "contact_stress")
    if not stresses:
        return None

    index = {n: i for i, n in enumerate(slave_nodes)}
    spacing = _mean_spacing(slave_coords_m)
    plane_strain = section_thickness_m is not None and section_thickness_m > 0.0

    times = sorted(t for t in stresses if t <= 1.0 + 1e-9)
    force_by_time = _force_by_time(curve)

    f_out, length, width, area, peak, counts = [], [], [], [], [], []
    for t in times:
        block = stresses[t]
        if len(block.values) == 0:
            continue
        pressure = np.abs(block.values[:, 0])
        p_max = float(pressure.max()) if len(pressure) else 0.0
        if p_max <= 0:
            continue

        hot = pressure >= CONTACT_PRESSURE_FRACTION * p_max
        rows = [index[n] for n, keep in zip(block.ids, hot) if keep and n in index]
        if not rows:
            continue
        pts = slave_coords_m[rows]

        patch_length = float(pts[:, 0].max() - pts[:, 0].min())
        f_out.append(force_by_time(t))
        length.append(patch_length)
        if plane_strain:
            # A single row of nodes at z = 0 standing in for the whole width. Length spans
            # the sampled nodes, so a patch one node wide has zero length and zero area —
            # correct, and the same degenerate case the 3-D path has.
            width.append(float(section_thickness_m))
            area.append(patch_length * float(section_thickness_m))
        else:
            width.append(float(pts[:, 2].max() - pts[:, 2].min()))
            area.append(float(len(rows) * spacing * spacing))
        peak.append(p_max)
        counts.append(len(rows))

    if not f_out:
        return None
    return ContactPatch(
        force_n=np.array(f_out),
        length_m=np.array(length),
        width_m=np.array(width),
        area_m2=np.array(area),
        peak_pressure_pa=np.array(peak),
        n_nodes=np.array(counts, dtype=np.int64),
    )


def _mean_spacing(coords: np.ndarray) -> float:
    """Median nearest-neighbour distance among slave nodes, metres."""
    if len(coords) < 2:
        return 0.0
    sample = coords[:: max(1, len(coords) // 200)]
    d = np.linalg.norm(sample[:, None, :] - sample[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    return float(np.median(d.min(axis=1)))


def _force_by_time(curve: LoadCurve):
    n = int(np.sum(curve.loading))
    times = np.linspace(0.0, 1.0, n + 1)[1:]

    def lookup(t: float) -> float:
        i = int(np.argmin(np.abs(times - t)))
        return float(curve.force_n[curve.loading][i])

    return lookup


def loaded_radius(curve: LoadCurve, params: WheelParams) -> np.ndarray:
    """Hub centre to contact plane, metres. Must decrease with load."""
    return params.outer_radius_mm * 1e-3 - curve.delta_m


def detect_buckling(curve: LoadCurve) -> tuple[bool, float | None]:
    """Where the loading branch loses most of its stiffness. ``(detected, force_there)``.

    A **collapse of the tangent**, not just a sign change. The earlier version of this
    required ``dF/dδ < 0`` strictly, and on that test the nominal design does not buckle: its
    tangent runs 12.1 N/mm, bottoms out at **+0.086 N/mm**, and climbs back to 10.0. It
    reported ``None`` for a curve that is nearly flat over several millimetres — a structure
    carrying a rising displacement at a constant load is buckling by any useful definition,
    and it was invisible because the criterion tested a *sign* rather than a *magnitude*.
    That is the failure class the CLAUDE.md watch list keeps naming, arriving here as a
    threshold rather than as a value.

    So the test is scale-free: buckling is the first point where the tangent falls below
    :data:`BUCKLING_STIFFNESS_FRACTION` of the stiffest tangent seen *earlier* on the same
    branch. A negative tangent still trips it — anything negative is below a fraction of a
    positive maximum — so every curve the old rule caught, this one catches too, at the same
    or an earlier point.

    Two deliberate limitations. The running maximum is taken over preceding samples only, so
    a curve that stiffens *after* the plateau (this one does, 0.086 → 10.0) is not
    retroactively judged against a stiffness it had not reached yet. And a single noisy
    sample can trip it: the tangent is a finite difference of solver output, and a *false*
    buckling report costs a design its feasibility rather than costing the campaign a wrong
    answer, which is the direction to err in. Detection is reported, never silently acted on.
    """
    k = curve.tangent_stiffness_n_per_m()
    if len(k) < 3:
        return False, None
    # Drop the first sample from both sides: before contact closes dF/ddelta is meaningless,
    # and using it as the reference stiffness would be worse than testing it. `reference[m]`
    # is then max(k[1..m+1]), the stiffest tangent strictly before the sample `k[m + 2]`.
    reference = np.maximum.accumulate(k[1:-1])
    collapsed = np.flatnonzero(
        (reference > 0.0) & (k[2:] < BUCKLING_STIFFNESS_FRACTION * reference)
    )
    if len(collapsed) == 0:
        return False, None
    idx = int(collapsed[0]) + 2
    return True, float(curve.force_n[curve.loading][idx])


def loop_area_fraction(curve: LoadCurve) -> float:
    """Enclosed loop area as a fraction of the loading-branch work.

    A hyperelastic model has no dissipation, so this should be ~0. It is reported as a
    **quality metric, not a material property**: a large value means contact chatter,
    friction locking, or that the unloading branch found a different equilibrium path —
    i.e. the structure buckled and did not come back the way it went.

    Both branches are integrated over their *common* displacement range. The two branches
    are generally not sampled at the same displacements — the loading branch may start well
    above zero — and integrating each over its own range compares the areas under different
    intervals, which reports a large loop for a curve that retraces perfectly.
    """
    load_d, load_f = curve.delta_m[curve.loading], curve.force_n[curve.loading]
    un_d, un_f = curve.delta_m[~curve.loading], curve.force_n[~curve.loading]
    if len(load_d) < 2 or len(un_d) < 2:
        return 0.0

    lo = max(float(load_d.min()), float(un_d.min()))
    hi = min(float(load_d.max()), float(un_d.max()))
    if not hi > lo:
        return 0.0

    grid = np.linspace(lo, hi, max(32, 2 * len(load_d)))
    load_order, un_order = np.argsort(load_d), np.argsort(un_d)
    f_load = np.interp(grid, load_d[load_order], load_f[load_order])
    f_unload = np.interp(grid, un_d[un_order], un_f[un_order])

    work = float(np.trapezoid(f_load, grid))
    if work <= 0:
        return 0.0
    return abs(float(np.trapezoid(f_load - f_unload, grid))) / work


def _von_mises(values: np.ndarray) -> np.ndarray:
    sxx, syy, szz, sxy, sxz, syz = (values[:, i] for i in range(6))
    vm = np.sqrt(
        0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
        + 3.0 * (sxy**2 + sxz**2 + syz**2)
    )
    return vm[np.isfinite(vm)]


def spoke_stress(blocks: list[DatBlock]) -> tuple[float | None, float | None]:
    """``(peak, p95)`` von Mises stress over the spoke elements, Pa, at peak load.

    Taken as the maximum over every sampled time rather than at the last one. The sweep
    ends *unloaded* — the amplitude returns to zero at the end of the step — so reading the
    final block reports the stress in a wheel carrying no load, which is approximately zero
    and would make every fatigue constraint pass.

    **Report both, trust only the second.** The spoke-to-rim and spoke-to-hub junctions are
    sharp re-entrant corners, where the stress is singular: the peak grows without bound as
    the mesh is refined and is not a physical quantity. The 95th percentile is
    mesh-convergent and is what a fatigue constraint should use. A spoke-root fillet in the
    CAD geometry is the real fix.
    """
    stresses = collect(blocks, "stress")
    if not stresses:
        return None, None

    best_peak, best_p95 = None, None
    for block in stresses.values():
        if block.values.shape[1] < 6:
            continue
        vm = _von_mises(block.values)
        if len(vm) == 0:
            continue
        peak = float(vm.max())
        if best_peak is None or peak > best_peak:
            best_peak, best_p95 = peak, float(np.percentile(vm, 95))
    return best_peak, best_p95


def fea_violations(
    curve: LoadCurve,
    load_case: LoadCase,
    params: WheelParams,
    p95_stress_pa: float | None,
    buckling_load_n: float | None,
    *,
    stress_safety_factor: float = 2.0,
    min_loaded_radius_fraction: float = 0.85,
    min_buckling_multiple: float = 2.5,
    min_contact_force_n: float = 1e-6,
) -> list[Violation]:
    """Design-level consequences, in the CAD stage's ``Violation`` vocabulary.

    These compose with ``wheelopt.cad.constraints.is_feasible`` unchanged, which is the
    whole reason for reusing the type rather than inventing an FEA-specific one.
    """
    out: list[Violation] = []
    nominal = load_case.nominal_load_n

    peak = curve.peak_force_n
    target = load_case.target_load_n

    # A sweep that never established contact converges beautifully, reports `ok`, and
    # produces a k_r(delta) of all zeros. Measured, not hypothetical: the `--tiny` bandless
    # wheel at the gap phase gives 0.00 N over the full 4 mm, because the deepest material
    # between two tips sits 5.4 mm below the running surface. Without this the result reads
    # as an infinitely compliant wheel rather than a load case that missed.
    if peak <= min_contact_force_n:
        out.append(
            Violation(
                name="fea_no_contact",
                severity=Severity.DEGENERATE,
                value=peak,
                limit=min_contact_force_n,
                margin=peak - min_contact_force_n,
                message=(
                    f"the indenter never touched the wheel: {peak:.3g} N over the whole "
                    f"{load_case.delta_max_m * 1e3:.1f} mm sweep. Nothing downstream can be "
                    "fitted to this. Check the indentation depth, and — on a bandless wheel "
                    "— check spoke_phase_deg, since the gap between two tips can be deeper "
                    "than the sweep"
                ),
            )
        )
        return out

    if peak < target:
        out.append(
            Violation(
                name="fea_load_range",
                severity=Severity.WARNING,
                value=peak,
                limit=target,
                margin=peak - target,
                message=(
                    f"sweep reached only {peak:.1f} N, short of "
                    f"{load_case.max_load_multiple:g}x nominal ({target:.1f} N); the ROM "
                    "fit is extrapolating above this load"
                ),
            )
        )

    r0 = params.outer_radius_mm * 1e-3
    idx = int(np.argmin(np.abs(curve.force_n[curve.loading] - nominal)))
    sag = float(curve.delta_m[curve.loading][idx])
    loaded = r0 - sag
    floor = min_loaded_radius_fraction * r0
    if loaded < floor:
        out.append(
            Violation(
                name="fea_static_sag",
                severity=Severity.INFEASIBLE,
                value=loaded,
                limit=floor,
                margin=loaded - floor,
                message=(
                    f"loaded radius {loaded * 1e3:.1f} mm at nominal load is below "
                    f"{min_loaded_radius_fraction:.0%} of {r0 * 1e3:.1f} mm"
                ),
            )
        )

    if p95_stress_pa is not None:
        limit = TPU_FATIGUE_LIMIT_PA / stress_safety_factor
        if p95_stress_pa > limit:
            out.append(
                Violation(
                    name="fea_peak_stress",
                    severity=Severity.INFEASIBLE,
                    value=p95_stress_pa,
                    limit=limit,
                    margin=limit - p95_stress_pa,
                    message=(
                        f"p95 spoke stress {p95_stress_pa / 1e6:.2f} MPa exceeds the "
                        f"fatigue limit / {stress_safety_factor:g} = {limit / 1e6:.2f} MPa"
                    ),
                )
            )

    if buckling_load_n is not None:
        limit = min_buckling_multiple * nominal
        if buckling_load_n < limit:
            out.append(
                Violation(
                    name="fea_buckling",
                    severity=Severity.INFEASIBLE,
                    value=buckling_load_n,
                    limit=limit,
                    margin=buckling_load_n - limit,
                    message=(
                        f"limit point at {buckling_load_n:.1f} N, below "
                        f"{min_buckling_multiple:g}x nominal ({limit:.1f} N)"
                    ),
                )
            )

    return out
