#!/usr/bin/env python3
"""Verification battery for the FEA stage. **Needs CalculiX; some checks also need gmsh.**

    conda install -c conda-forge calculix
    python scripts/verify_fea.py            # patch tests only, seconds
    python scripts/verify_fea.py --full     # adds full wheel sweeps, tens of minutes

The pure layers (hyperelastic, deck, parse, extract, cache, indenter) are covered by
`python -m unittest discover -s tests -t .` and need neither dependency. This script covers
what unit tests cannot: that CalculiX accepts our material card, that the constants are in
the order we think, that the units are consistent, and that the resulting wheel behaves
like a wheel.

Exit code 0 means the FEA stage is trustworthy enough to proceed to step 4 (the MuJoCo ring
model). Exit code 2 means the solver is missing — an environment problem, not a failure.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import numpy as np  # noqa: E402

from wheelopt.cad.materials import TPU95A  # noqa: E402
from wheelopt.cad.params import WheelParams  # noqa: E402
from wheelopt.fea.hyperelastic import HyperelasticModel, for_material  # noqa: E402
from wheelopt.fea.loadcase import LoadCase, LoadCaseKind, MeshSpec, SolverSpec  # noqa: E402
from wheelopt.fea.parse import parse_dat  # noqa: E402
from wheelopt.fea.results import common_force_n
from wheelopt.fea.runner import find_ccx, run_load_case  # noqa: E402

PASS, FAIL = "PASS", "FAIL"


@dataclass
class Check:
    name: str
    status: str
    detail: str = ""


results: list[Check] = []


def record(name: str, ok: bool, detail: str = "") -> bool:
    results.append(Check(name, PASS if ok else FAIL, detail))
    print(f"[{'  ok  ' if ok else ' FAIL '}] {name}" + (f"  — {detail}" if detail else ""))
    return ok


# --- 1. single-element patch test ------------------------------------------------------
# The highest-value check in this file, and the cheapest. One C3D10 under uniaxial stretch
# validates the material card syntax, the coefficient ordering, the unit system and the
# gmsh->Abaqus node ordering simultaneously, in under a second. Run it first: if it fails,
# nothing downstream can be interpreted.

def uniaxial_patch(ccx: Path, hyper: HyperelasticModel, eps: float = 1e-4):
    """Impose u_x = eps*x on every node; leave lateral free, remove only rigid-body modes.

    Prescribing *all three* components on every node would leave the system with no
    unknowns at all, and CalculiX then reports a stress field that is neither an error nor
    the answer. Free lateral contraction keeps it a real solve, and the closed form is then
    simply sigma_xx = E * eps.
    """
    a = 0.01
    corners = np.array([[0, 0, 0], [a, 0, 0], [0, a, 0], [0, 0, a]], dtype=float)
    pairs = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]
    nodes = np.vstack([corners] + [(corners[i] + corners[j]) / 2 for i, j in pairs])

    lines = ["*NODE, NSET=NALL"]
    for i, p in enumerate(nodes, 1):
        lines.append(f"{i}, {p[0]:.9e}, {p[1]:.9e}, {p[2]:.9e}")
    lines += [
        "*ELEMENT, TYPE=C3D10, ELSET=EALL",
        "1, " + ", ".join(str(i) for i in range(1, 11)),
        hyper.calculix_card("M"),
        "*SOLID SECTION, ELSET=EALL, MATERIAL=M",
        "*STEP, NLGEOM",
        "*STATIC",
        "*BOUNDARY",
    ]
    for i, p in enumerate(nodes, 1):
        lines.append(f"{i}, 1, 1, {eps * p[0]:.9e}")
    lines += ["1, 2, 2, 0.0", "1, 3, 3, 0.0", "3, 3, 3, 0.0"]
    lines += ["*EL PRINT, ELSET=EALL", " S", "*END STEP"]

    workdir = Path(tempfile.mkdtemp())
    (workdir / "patch.inp").write_text("\n".join(lines) + "\n")
    subprocess.run([str(ccx), "patch"], cwd=workdir, capture_output=True,
                   text=True, timeout=180)
    dat = workdir / "patch.dat"
    if not dat.exists():
        return None
    blocks = parse_dat(dat.read_text())
    return blocks[0] if blocks else None


def section_patch(ccx: Path) -> None:
    print("\n1. Single-element patch test")
    hyper = for_material(TPU95A, 2.0)
    eps = 1e-4
    block = uniaxial_patch(ccx, hyper, eps)
    if not record("CalculiX accepts the deck and writes results", block is not None):
        return

    expected = hyper.initial_youngs_pa * eps
    sxx = float(block.values[:, 0].mean())
    record(
        "axial stress matches E * eps",
        abs(sxx / expected - 1.0) < 0.01,
        f"{sxx:.3f} Pa vs {expected:.3f} Pa  (ratio {sxx / expected:.5f})",
    )
    lateral = float(np.abs(block.values[:, 1:3]).max())
    record(
        "lateral stress is free",
        lateral < 0.01 * abs(expected),
        f"max |syy,szz| = {lateral:.3e} Pa",
    )
    record(
        "stress is uniform across integration points",
        float(np.ptp(block.values[:, 0])) < 1e-6 * abs(expected) + 1e-6,
        f"spread {float(np.ptp(block.values[:, 0])):.3e} Pa",
    )
    record(
        "header advertises six stress components",
        len(block.components) == 6,
        ", ".join(block.components),
    )

    print("\n2. Compressibility sweep (locking sensitivity)")
    # Volumetric locking is a *constrained*-deformation phenomenon, so free uniaxial
    # stretch does not exhibit it. This sweep therefore verifies that D1 is honoured
    # across the range, not that the wheel is lock-free — that has to be measured on the
    # real geometry by comparing k_r at two Poisson ratios (--full).
    rows = []
    for nu in (0.40, 0.45, 0.46, 0.49, 0.4999):
        model = for_material(TPU95A, 2.0, poisson_effective=nu)
        b = uniaxial_patch(ccx, model, eps)
        if b is None:
            record(f"patch solves at nu={nu}", False)
            continue
        got = float(b.values[:, 0].mean())
        want = model.initial_youngs_pa * eps
        rows.append((nu, got / want))
    record(
        "D1 is honoured across the Poisson range",
        all(abs(r - 1.0) < 0.02 for _, r in rows),
        ", ".join(f"nu={nu}: {r:.4f}" for nu, r in rows),
    )


# --- 3. determinism --------------------------------------------------------------------

def section_determinism() -> None:
    print("\n3. Determinism")
    from wheelopt.cad.compliant_spoke import build_wheel
    from wheelopt.cad.export import export
    from wheelopt.fea.mesh import mesh_step

    params = WheelParams(outer_radius_mm=60.0, width_mm=30.0, n_spokes=6,
                         spoke_thickness_mm=5.0, hub_radius_mm=20.0)
    spec = MeshSpec(size_spoke_m=0.008, size_rim_m=0.010, size_hub_m=0.010)
    out = Path(tempfile.mkdtemp())
    result = build_wheel(params, TPU95A)
    if not record("verification geometry builds", result.ok):
        return
    step = export(result.part, params, out).step

    first = mesh_step(step, params, spec)
    second = mesh_step(step, params, spec)
    record(
        "meshing the same STEP twice is bit-identical",
        first.nodes_m.shape == second.nodes_m.shape
        and bool(np.array_equal(first.nodes_m, second.nodes_m))
        and bool(np.array_equal(first.elements, second.elements)),
        f"{first.n_elements} elements",
    )
    record(
        "mesh has no degenerate elements",
        first.stats.min_volume_m3 > 0,
        f"min volume {first.stats.min_volume_m3:.2e} m^3",
    )
    record(
        "surface sets are populated",
        len(first.node_sets["bore"]) > 0 and len(first.node_sets["tread"]) > 0,
        f"bore={len(first.node_sets['bore'])}, tread={len(first.node_sets['tread'])}",
    )

    # Invariant 4 at the meshing boundary. gmsh raises a bare Exception on a bad size
    # field — "PLC Error: Two facets intersect" for an element coarser than the bore — and
    # that used to escape mesh_step and take the whole evaluation with it, despite the
    # docstring promising MeshFailure. A campaign cannot afford that.
    from wheelopt.fea.mesh import MeshFailure

    absurd = MeshSpec(size_spoke_m=0.008, size_rim_m=0.010, size_hub_m=0.018)
    try:
        mesh_step(step, params, absurd)
        record("an unmeshable size field is reported, not raised", False,
               "the deliberately coarse mesh succeeded; pick a coarser one")
    except MeshFailure as exc:
        record("an unmeshable size field is reported, not raised", True,
               f"MeshFailure: {str(exc).split('.')[0][:60]}")
    except Exception as exc:  # noqa: BLE001 - that is the whole point of the check
        record("an unmeshable size field is reported, not raised", False,
               f"{type(exc).__name__} escaped instead of MeshFailure")


# --- 4/5. full wheel sweeps ------------------------------------------------------------

def section_sweeps(args) -> None:
    print("\n4. Flat-plate compression sweep")
    params = WheelParams(outer_radius_mm=60.0, width_mm=30.0, n_spokes=6,
                         spoke_thickness_mm=5.0, hub_radius_mm=20.0)
    # Geometry, mesh and sweep all match run_fea.py's --tiny preset on purpose: any
    # difference gives a different cache key, so the battery and the developer loop would
    # each solve the same job separately.
    spec = MeshSpec(size_spoke_m=0.008, size_rim_m=0.010, size_hub_m=0.010)
    solver = SolverSpec(timeout_s=args.timeout, n_threads=args.threads)

    flat = LoadCase(kind=LoadCaseKind.RADIAL_FLAT, delta_max_m=0.006,
                    n_points_per_branch=6)
    started = time.perf_counter()
    result = run_load_case(params, TPU95A, flat, mesh_spec=spec, solver=solver,
                           cache_root=args.cache)
    if not record("flat sweep converges", result.ok,
                  f"{result.status.value}, {time.perf_counter() - started:.0f} s"):
        print(f"        {result.message}")
        return

    curve = result.curve
    record("force is positive throughout", bool(np.all(curve.force_n >= -1e-9)),
           f"peak {curve.peak_force_n:.2f} N")
    record("force increases with indentation on the loading branch",
           bool(np.all(np.diff(curve.force_n[curve.loading]) > -1e-6)))
    k = curve.tangent_stiffness_n_per_m()
    record("radial stiffness is positive", bool(np.all(k[1:] > 0)),
           f"{k[1] / 1e3:.1f} -> {k[-1] / 1e3:.1f} kN/m")
    record("loaded radius decreases with load",
           bool(np.all(np.diff(result.loaded_radius_m[curve.loading]) < 0)))
    record("unloading retraces loading (hyperelastic, so it must)",
           result.loop_area_fraction < 0.05,
           f"loop area {result.loop_area_fraction:.3%} of loading work")

    if result.patch is not None and len(result.patch.force_n) > 1:
        record("contact patch grows with load",
               result.patch.length_m[-1] >= result.patch.length_m[0],
               f"{result.patch.length_m[0] * 1e3:.1f} -> "
               f"{result.patch.length_m[-1] * 1e3:.1f} mm")
    else:
        record("contact patch was extracted", False, "no contact pressure output")

    print("\n5. Step-edge compression sweep")
    step_case = LoadCase(kind=LoadCaseKind.RADIAL_STEP_EDGE, delta_max_m=0.006,
                         n_points_per_branch=6)
    started = time.perf_counter()
    edge = run_load_case(params, TPU95A, step_case, mesh_spec=spec, solver=solver,
                         cache_root=args.cache)
    if not record("step-edge sweep converges", edge.ok,
                  f"{edge.status.value}, {time.perf_counter() - started:.0f} s"):
        print(f"        {edge.message}")
        return

    if edge.patch is not None and result.patch is not None:
        # Compare at equal LOAD, not at equal indentation. Both sweeps run to the same
        # delta_max, but the step edge is the softer indenter — it reaches ~3.0 N where the
        # flat plate reaches ~4.4 N — so comparing the last sample of each pits a 3 N patch
        # against a 4.4 N one and the smaller-patch/higher-pressure claim stops meaning
        # anything. The common load is the smaller of the two peaks; neither sweep is
        # extrapolated past what it actually converged to.
        # `common_force_n` rather than min(peaks): contact output only starts once nodes
        # touch, so a sweep's lowest sampled load is not zero, and two sweeps can cover
        # disjoint load ranges. Asking for a load one of them never reached would clamp and
        # return a plausible number for a state that was never solved.
        common_n = common_force_n(edge.patch, result.patch)
        if common_n is None:
            record("the two sweeps overlap in load at all", False,
                   f"step edge {edge.patch.force_range_n[0]:.1f}-"
                   f"{edge.patch.force_range_n[1]:.1f} N vs flat "
                   f"{result.patch.force_range_n[0]:.1f}-"
                   f"{result.patch.force_range_n[1]:.1f} N: no comparable load")
            return

        edge_area, edge_pressure = edge.patch.at_force(common_n)
        flat_area, flat_pressure = result.patch.at_force(common_n)
        print(f"        comparing both cases at {common_n:.2f} N")

        record("step edge gives a smaller contact patch than a flat plate",
               edge_area <= flat_area,
               f"{edge_area * 1e6:.1f} vs {flat_area * 1e6:.1f} mm^2 at {common_n:.2f} N")
        record("step edge gives a higher contact pressure than a flat plate",
               edge_pressure >= flat_pressure,
               f"{edge_pressure / 1e3:.1f} vs {flat_pressure / 1e3:.1f} kPa (mean) "
               f"at {common_n:.2f} N")

        # Peak *nodal* pressure is reported, never asserted on: it tracks the number of
        # nodes in contact rather than the load, and on the flat sweep it decreases as the
        # load rises. See the note on ContactPatch.peak_pressure_pa.
        print(f"        peak nodal pressure (diagnostic, not mesh-convergent): "
              f"step {edge.patch.peak_pressure_pa[-1] / 1e3:.0f} vs "
              f"flat {result.patch.peak_pressure_pa[-1] / 1e3:.0f} kPa")

    section_calibration(params, spec, solver, args)


def section_calibration(params, spec3d, solver, args) -> None:
    """6. The plane-strain screening tier against the 3-D tier it stands in for.

    The point of the 2-D tier is to be cheap enough to run per design, which is only useful
    if its k_r has the right shape and a *known* offset. The offset is measured here rather
    than assumed: an earlier version of this module's docstring predicted the sign of it
    from theory and got it backwards.

    Frictionless on both sides. The 2-D tier does not converge with friction, so comparing
    it against a frictional 3-D run would fold that difference into the ratio.
    """
    print("\n6. Plane-strain tier against the 3-D tier")
    # Both sides frictionless. The section 4 sweep runs at the default mu = 0.8, and the
    # 2-D tier does not converge with friction, so reusing it would fold the friction
    # difference into the ratio and call it a dimensionality effect.
    case = LoadCase(kind=LoadCaseKind.RADIAL_FLAT, delta_max_m=0.006,
                    n_points_per_branch=6, friction_mu=0.0)
    three_d = run_load_case(params, TPU95A, case, mesh_spec=spec3d, solver=solver,
                            cache_root=args.cache)
    if not record("frictionless 3-D reference converges", three_d.ok, three_d.status.value):
        return

    spec2d = MeshSpec(dimension=2, size_spoke_m=0.0025, size_rim_m=0.003, size_hub_m=0.002)
    started = time.perf_counter()
    flat2d = run_load_case(params, TPU95A, case, mesh_spec=spec2d, solver=solver,
                           cache_root=args.cache)
    if not record("plane-strain sweep converges", flat2d.ok,
                  f"{flat2d.status.value}, {time.perf_counter() - started:.0f} s"):
        print(f"        {flat2d.message}")
        return

    ratio_f = flat2d.curve.peak_force_n / three_d.curve.peak_force_n
    record("plane-strain force is within 25% of the 3-D value",
           0.75 <= ratio_f <= 1.25,
           f"{flat2d.curve.peak_force_n:.2f} vs {three_d.curve.peak_force_n:.2f} N "
           f"(ratio {ratio_f:.2f})")

    k2, k3 = (c.tangent_stiffness_n_per_m() for c in (flat2d.curve, three_d.curve))
    ratio_k = float(k2[-1] / k3[-1])
    record("plane-strain stiffness is within 25% of the 3-D value",
           0.75 <= ratio_k <= 1.25,
           f"{k2[-1] / 1e3:.2f} vs {k3[-1] / 1e3:.2f} kN/m (ratio {ratio_k:.2f})")

    record("both tiers stiffen", bool(k2[-1] > k2[1] and k3[-1] > k3[1]))

    # Friction, which the tier could not do until the contact penalty was softened. Kept as
    # a check rather than a note because the failure was conditioning, not physics. The
    # softening is now the default (#12, 2026-08-09) and this asserts it stayed that way:
    # `stiff` is the old default of 20, and the pair says the tier converges where it used
    # to diverge. Asserting only the default would pass on any future default whatever.
    stiff = replace(solver, contact_stiffness_factor=20.0, contact_length_floor_m=0.0)
    frictional = run_load_case(params, TPU95A, replace(case, friction_mu=0.8),
                               mesh_spec=spec2d, solver=solver, cache_root=args.cache)
    harsh = run_load_case(params, TPU95A, replace(case, friction_mu=0.8),
                          mesh_spec=spec2d, solver=stiff, cache_root=args.cache)
    record("the old uncapped factor-20 penalty is what diverged", not harsh.ok,
           f"{harsh.status.value} at factor 20, uncapped")
    if record("plane-strain converges with friction at the default contact penalty",
              frictional.ok,
              f"{frictional.status.value}, mu=0.8, factor "
              f"{solver.contact_stiffness_factor:g}"):
        k_mu = frictional.curve.tangent_stiffness_n_per_m()[-1]
        record("friction changes the answer by less than 10%",
               abs(k_mu - k2[-1]) <= 0.10 * k2[-1],
               f"{k_mu / 1e3:.2f} vs {k2[-1] / 1e3:.2f} kN/m frictionless")

    if flat2d.patch is not None and three_d.patch is not None:
        # Width in 2-D is the section thickness by construction; without that it reports
        # zero, because every slave node sits at z = 0.
        record("plane-strain patch has a non-zero width",
               flat2d.patch.width_m[-1] > 0,
               f"{flat2d.patch.width_m[-1] * 1e3:.1f} mm")
        record("plane-strain patch length is within 20% of the 3-D value",
               abs(flat2d.patch.length_m[-1] - three_d.patch.length_m[-1])
               <= 0.2 * three_d.patch.length_m[-1],
               f"{flat2d.patch.length_m[-1] * 1e3:.1f} vs "
               f"{three_d.patch.length_m[-1] * 1e3:.1f} mm")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--full", action="store_true",
                        help="include the full wheel sweeps (tens of minutes)")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=5400.0)
    parser.add_argument("--cache", type=Path,
                        default=REPO_ROOT / "data" / "cache" / "fea")
    args = parser.parse_args(argv)

    ccx = find_ccx()
    print("=" * 72)
    print("FEA stage verification")
    print("=" * 72)
    if ccx is None:
        print("\nNo CalculiX binary found. Install it with:")
        print("  conda install -c conda-forge calculix")
        print("\nThe pure layers are testable without it:")
        print("  python -m unittest discover -s tests -t .")
        return 2
    print(f"solver: {ccx}")

    section_patch(ccx)
    try:
        section_determinism()
    except ImportError as exc:
        print(f"\n(skipping mesh checks: {exc})")
    if args.full:
        section_sweeps(args)
    else:
        print("\n(skipping wheel sweeps; pass --full to include them)")

    n_fail = sum(1 for c in results if c.status == FAIL)
    print("\n" + "=" * 72)
    print(f"{len(results) - n_fail}/{len(results)} checks passed")
    if n_fail:
        print("\nFailed:")
        for c in results:
            if c.status == FAIL:
                print(f"  - {c.name}" + (f" ({c.detail})" if c.detail else ""))
        print("\nDo not build the ROM on these results until they are resolved.")
    else:
        print("\nFEA stage looks sound. Next: docs/plan/16-first-week.md step 4 (MuJoCo ring).")
    print("=" * 72)
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
