#!/usr/bin/env python3
"""Verification battery for the CAD stage. **Run this on a machine with build123d.**

    pip install build123d
    python scripts/verify_cad.py

The pure-numpy layers (params, materials, constraints, centreline, massprops) are covered
by ``python -m unittest discover -s tests -t .`` and need no CAD kernel. This script covers
what unit tests cannot: that OCCT actually produces a valid solid, that the tessellation is
fine enough for mass properties, and that the numbers move in physically sensible
directions across the design space.

Exit code 0 means the CAD stage is trustworthy enough to proceed to the FEA stage
(docs/plan/16-first-week.md step 3).
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import numpy as np  # noqa: E402

from wheelopt.cad.constraints import check_design, is_feasible  # noqa: E402
from wheelopt.cad.materials import PLA, TPU95A  # noqa: E402
from wheelopt.cad.params import SpokeProfile, WheelParams  # noqa: E402

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"


@dataclass
class Check:
    name: str
    status: str
    detail: str = ""


results: list[Check] = []


def record(name: str, ok: bool, detail: str = "") -> bool:
    results.append(Check(name, PASS if ok else FAIL, detail))
    marker = "  ok  " if ok else " FAIL "
    print(f"[{marker}] {name}" + (f"  — {detail}" if detail else ""))
    return ok


def _self_intersects(outline: np.ndarray) -> bool:
    """Whether a closed polygon crosses itself. O(n^2), which at ~130 points is microseconds.

    Written out rather than delegated to the kernel on purpose. An outline is the centreline
    offset by half the local thickness, and offsetting a *corner* is where that construction
    breaks: inside a bend of centreline radius rho the offset face has radius rho - h, which
    at rho < h turns the polygon inside out. OCCT may refuse such a face, or may accept it and
    build a solid with a reversed patch whose volume is still plausible — the second is this
    project's standing failure mode, so the check has to be independent of the kernel.
    """
    n = len(outline)
    a, b = outline, np.roll(outline, -1, axis=0)

    def side(p, q, r):
        return np.sign((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]))

    for i in range(n):
        # Skip the neighbours: consecutive edges share an endpoint by construction.
        for j in range(i + 2, n - (1 if i == 0 else 0)):
            p1, p2, p3, p4 = a[i], b[i], a[j], b[j]
            if (side(p1, p2, p3) * side(p1, p2, p4) < 0
                    and side(p3, p4, p1) * side(p3, p4, p2) < 0):
                return True
    return False


#: Sections 4, 5 and 8 do the bulk of the builds and dominate the runtime.
QUICK_SECTIONS = (1, 2, 3, 6, 7, 9, 10, 11)
#: Section 3 reads the mass properties computed in section 2.
SECTION_DEPENDS = {3: (2,)}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--only",
        metavar="N[,N...]",
        help="run only these sections (1-11). Section 1 always runs; dependencies are pulled in.",
    )
    group.add_argument(
        "--quick",
        action="store_true",
        help=f"skip the multi-build sweeps. Equivalent to --only {','.join(map(str, QUICK_SECTIONS))}",
    )
    return parser.parse_args(argv)


def selected_sections(args: argparse.Namespace) -> set[int]:
    if args.quick:
        wanted = set(QUICK_SECTIONS)
    elif args.only:
        wanted = {int(x) for x in args.only.replace(" ", "").split(",") if x}
    else:
        return set(range(1, 12))
    for section, deps in SECTION_DEPENDS.items():
        if section in wanted:
            wanted.update(deps)
    wanted.add(1)  # the nominal build is every other section's precondition
    return wanted


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    want = selected_sections(args)
    try:
        import build123d  # noqa: F401
    except ImportError:
        print("build123d is not installed. This script cannot run without it.\n")
        print("  pip install build123d\n")
        print("The numpy layers are testable without it:")
        print("  python -m unittest discover -s tests -t .")
        return 2

    from wheelopt.cad.compliant_spoke import build_wheel, tessellate
    from wheelopt.cad.export import export, is_watertight
    from wheelopt.cad.massprops import check_against_brep_volume, mass_properties

    print("=" * 72)
    print("CAD stage verification")
    print("=" * 72)

    # --- 1. nominal build -------------------------------------------------------------
    print("\n1. Nominal design builds")
    nominal = WheelParams()
    t0 = time.perf_counter()
    result = build_wheel(nominal, TPU95A)
    build_s = time.perf_counter() - t0

    if not record("builds without error", result.ok, f"{build_s:.2f} s"):
        return 1
    record("build time under 10 s", build_s < 10.0, f"{build_s:.2f} s")
    record("positive BREP volume", (result.brep_volume_m3 or 0) > 0,
           f"{(result.brep_volume_m3 or 0) * 1e6:.2f} cm^3")

    # The bounding box is the only check that catches a whole-solid scale error. A wrong
    # `extrude(both=)` convention, for instance, would halve the width of every wheel while
    # leaving mass in range, Izz/(m R^2) unchanged (it is width-independent), the mesh-vs-BREP
    # cross-check self-consistent and every sweep below monotonic. Silent, and fatal to both
    # the FEA stiffness and the MuJoCo inertia.
    bbox = result.part.bounding_box()
    record("width matches width_mm", abs(bbox.size.Z - nominal.width_mm) < 1e-6,
           f"{bbox.size.Z:.4f} mm, want {nominal.width_mm:.4f}")
    record("diameter matches 2 x outer_radius_mm",
           abs(bbox.size.X - 2 * nominal.outer_radius_mm) < 1e-6,
           f"{bbox.size.X:.4f} mm, want {2 * nominal.outer_radius_mm:.4f}")
    record("solid is centred on the mid-plane", abs(bbox.center().Z) < 1e-9,
           f"z = {bbox.center().Z:+.2e} mm")

    # A Compound of disjoint solids still reports a volume and still exports, but the STEP
    # would carry several volumes and the FEA mesher would silently take one of them.
    n_solids = len(result.part.solids())
    record("result is a single solid", n_solids == 1, f"{n_solids} solids")

    # --- 2. mesh quality --------------------------------------------------------------
    if 2 in want:
        print("\n2. Tessellation and mesh quality")
        v, f = tessellate(result.part, tolerance_mm=0.05)
        record("mesh is non-empty", len(f) > 0, f"{len(f)} triangles")

        watertight, n_bad = is_watertight(f)
        record("mesh is watertight", watertight,
               "closed" if watertight else f"{n_bad} non-manifold edges")

        rho = TPU95A.effective_density_kg_m3(nominal.spoke_thickness_mm)
        mp = mass_properties(v, f, rho)
        ok_vol, rel = check_against_brep_volume(mp.volume_m3, result.brep_volume_m3)
        record("mesh volume within 1% of BREP", ok_vol, f"{rel:+.3%}")
        record("tessellation under-reports (inscribed)", rel <= 1e-9, f"{rel:+.3%}")

    # --- 3. mass properties are physical ----------------------------------------------
    if 3 in want:
        print("\n3. Mass properties are physically sensible")
        mass_g = mp.mass_kg * 1e3
        record("mass in a plausible range (100-600 g)", 100.0 < mass_g < 600.0, f"{mass_g:.1f} g")
        record("centre of mass on the axis", float(np.linalg.norm(mp.com_m[:2])) < 1e-4,
               f"offset {np.linalg.norm(mp.com_m[:2]) * 1e3:.4f} mm")

        izz = mp.inertia_kg_m2[2, 2]
        ixx = mp.inertia_kg_m2[0, 0]
        record("polar moment is the largest", izz > ixx, f"Izz/Ixx = {izz / ixx:.2f}")

        # A wheel's mass sits between a thin ring (m R^2) and a solid disc (m R^2 / 2), but
        # the T3 hub is a *solid* annulus out to hub_radius_mm and carries a large share of
        # the mass close to the axle, which pulls the ratio below the thin-disc value. The
        # lower bound is therefore set from the geometry family, not from the disc formula.
        # See docs/experiments/log.md, 2026-08-05.
        r = nominal.outer_radius_mm * 1e-3
        ratio = izz / (mp.mass_kg * r * r)
        record("Izz between solid-hub and ring bounds", 0.35 < ratio < 1.05,
               f"Izz/(m R^2) = {ratio:.3f}")

    # --- 4. monotonic responses -------------------------------------------------------
    def mass_of(p: WheelParams, material=TPU95A) -> float:
        res = build_wheel(p, material)
        if not res.ok:
            return float("nan")
        vv, ff = tessellate(res.part, tolerance_mm=0.08)
        d = material.effective_density_kg_m3(p.spoke_thickness_mm)
        return mass_properties(vv, ff, d).mass_kg

    if 4 in want:
        print("\n4. Parameter sweeps move in the right direction")

        m_thin = mass_of(WheelParams(spoke_thickness_mm=4.0))
        m_thick = mass_of(WheelParams(spoke_thickness_mm=8.0))
        record("thicker spokes weigh more", m_thick > m_thin,
               f"{m_thin * 1e3:.1f} g -> {m_thick * 1e3:.1f} g")

        # Spoke count is capped by the interspoke gap: at the nominal 7 mm thickness the
        # hub circumference runs out well before 24 spokes.
        m_few = mass_of(WheelParams(n_spokes=8))
        m_many = mass_of(WheelParams(n_spokes=14))
        record("more spokes weigh more", m_many > m_few,
               f"{m_few * 1e3:.1f} g -> {m_many * 1e3:.1f} g")

        m_narrow = mass_of(WheelParams(width_mm=32.0))
        m_wide = mass_of(WheelParams(width_mm=68.0))
        record("mass is near-linear in width", m_wide > m_narrow,
               f"{m_narrow * 1e3:.1f} g -> {m_wide * 1e3:.1f} g")

        # Compare the *materials*, which means holding the process fixed. The presets
        # differ in infill (TPU 40%, PLA 25%) as well as base density, and on a 7 mm spoke
        # the infill difference is the larger of the two — so comparing the presets
        # directly tests the slicer settings, not the polymer, and flips sign somewhere
        # around a 3 mm spoke. It passed for exactly that reason until the spokes grew.
        pla_same_process = replace(PLA, infill_density=TPU95A.infill_density,
                                   wall_count=TPU95A.wall_count)
        m_tpu = mass_of(WheelParams(), TPU95A)
        m_pla = mass_of(WheelParams(), pla_same_process)
        record("PLA is denser than TPU at equal geometry and equal process",
               m_pla > m_tpu, f"TPU {m_tpu * 1e3:.1f} g vs PLA {m_pla * 1e3:.1f} g")

    # --- 5. all profiles build --------------------------------------------------------
    if 5 in want:
        print("\n5. Every spoke profile builds")
        for profile in SpokeProfile:
            p = WheelParams(spoke_profile=profile)
            res = build_wheel(p, TPU95A, skip_screening=True)
            ok = res.ok and (res.brep_volume_m3 or 0) > 0
            record(f"profile {profile.value}", ok,
                   f"{(res.brep_volume_m3 or 0) * 1e6:.2f} cm^3" if ok else "build failed")

        # The tread cutter is the one geometry path with no unit-test coverage at all —
        # tread_depth_mm defaults to 0.0, so nothing above ever exercises it.
        treaded = WheelParams(tread_depth_mm=1.5)
        res = build_wheel(treaded, TPU95A)
        record("treaded wheel builds", res.ok,
               f"{(res.brep_volume_m3 or 0) * 1e6:.2f} cm^3" if res.ok else "build failed")
        if res.ok:
            n = len(res.part.solids())
            record("treaded wheel is a single solid", n == 1, f"{n} solids")
            record("tread removes material",
                   (res.brep_volume_m3 or 0) < (result.brep_volume_m3 or 0),
                   f"{(result.brep_volume_m3 or 0) * 1e6:.2f} -> "
                   f"{(res.brep_volume_m3 or 0) * 1e6:.2f} cm^3")
            tb = res.part.bounding_box()
            record("tread does not change the envelope",
                   abs(tb.size.Z - treaded.width_mm) < 1e-6
                   and abs(tb.size.X - 2 * treaded.outer_radius_mm) < 1e-6,
                   f"{tb.size.X:.3f} x {tb.size.Z:.3f} mm")

    # --- 6. screening agrees with the kernel ------------------------------------------
    if 6 in want:
        print("\n6. Screening and the CAD kernel agree")
        crowded = WheelParams(n_spokes=36, spoke_thickness_mm=4.0)
        screened_out = not is_feasible(check_design(crowded, TPU95A))
        record("crowded design is rejected by screening", screened_out)

        rejected = build_wheel(crowded, TPU95A)
        record("rejected design returns no part (does not raise)",
               (not rejected.ok) and rejected.part is None)

    # --- 7. determinism ---------------------------------------------------------------
    if 7 in want:
        print("\n7. Determinism")
        a = build_wheel(nominal, TPU95A)
        b = build_wheel(nominal, TPU95A)
        same = abs((a.brep_volume_m3 or 0) - (b.brep_volume_m3 or 0)) < 1e-15
        record("repeated builds give identical volume", same)

        va, fa = tessellate(a.part, tolerance_mm=0.05)
        vb, fb = tessellate(b.part, tolerance_mm=0.05)
        record("repeated tessellation is identical",
               va.shape == vb.shape and np.allclose(va, vb) and np.array_equal(fa, fb))

    # --- 8. tessellation convergence --------------------------------------------------
    if 8 in want:
        print("\n8. Tessellation convergence")
        # Both tolerances must be refined together. Sweeping the chordal tolerance alone
        # barely moves this mesh — the volume error lives on the cylindrical surfaces, where
        # the angular tolerance sets facet count — so a linear-only sweep produces a flat
        # line and a convergence check that passes without testing anything.
        volumes = []
        counts = []
        for tol in (0.4, 0.2, 0.1, 0.05, 0.025):
            vv, ff = tessellate(result.part, tolerance_mm=tol, angular_tolerance_rad=tol)
            volumes.append(mass_properties(vv, ff, 1000.0).volume_m3)
            counts.append(len(ff))

        record("refinement actually changes the mesh", len(set(counts)) == len(counts),
               " -> ".join(f"{c}" for c in counts) + " triangles")

        increasing = all(volumes[i] <= volumes[i + 1] + 1e-12 for i in range(len(volumes) - 1))
        record("volume converges upward as tolerance tightens", increasing,
               " -> ".join(f"{x * 1e6:.3f}" for x in volumes) + " cm^3")

        brep = result.brep_volume_m3 or float("nan")
        record("refined mesh approaches the BREP volume",
               abs(volumes[-1] - brep) < abs(volumes[0] - brep),
               f"{(volumes[0] - brep) / brep:+.4%} -> {(volumes[-1] - brep) / brep:+.4%}")

        drift = abs(volumes[-1] - volumes[-2]) / volumes[-1]
        record("0.05 mm / 0.05 rad is converged (<0.1% residual)", drift < 1e-3,
               f"{drift:.4%}")

    # --- 9. export round-trip ---------------------------------------------------------
    if 9 in want:
        print("\n9. Export")
        out_dir = REPO_ROOT / "data" / "verify"
        paths = export(result.part, nominal, out_dir)
        record("STEP written", paths.step.exists() and paths.step.stat().st_size > 0,
               f"{paths.step.stat().st_size / 1024:.0f} kB")
        record("STL written", paths.stl.exists() and paths.stl.stat().st_size > 0,
               f"{paths.stl.stat().st_size / 1024:.0f} kB")
        record("filename carries the design hash", nominal.design_hash() in paths.stem,
               paths.stem)

    # --- 10. bandless topology --------------------------------------------------------
    # `rim_thickness_mm = 0`: no shear band, spoke tips are the running surface. Every
    # check here is one the banded wheel gets for free from the rim cylinder, so this is
    # the section that would go silently wrong first.
    if 10 in want:
        print("\n10. Bandless wheel (no shear band)")
        from wheelopt.fea.loadcase import CONTACT_ANGLE_DEG, phase_for_tip_contact

        bandless = WheelParams(
            rim_thickness_mm=0.0,
            spoke_phase_deg=phase_for_tip_contact(WheelParams().n_spokes),
        )
        record("screening accepts it", is_feasible(check_design(bandless, TPU95A)))
        record("screening warns that contact is discrete",
               any(x.name == "no_shear_band" for x in check_design(bandless, TPU95A)))

        res = build_wheel(bandless, TPU95A)
        if record("builds without error", res.ok):
            n = len(res.part.solids())
            # The spokes are joined only through the hub. If the hub union ever failed, the
            # result would be n_spokes + 1 disjoint solids that still report a total volume.
            record("is a single solid (spokes joined through the hub)", n == 1, f"{n} solids")

            record("lighter than the banded wheel",
                   (res.brep_volume_m3 or 0) < (result.brep_volume_m3 or 0),
                   f"{(result.brep_volume_m3 or 0) * 1e6:.2f} -> "
                   f"{(res.brep_volume_m3 or 0) * 1e6:.2f} cm^3")

            # The tip must reach the running surface without passing through it. Outside is
            # the dangerous direction: the FEA tread node set is |r - R| < 0.1 mm, so
            # material beyond R would carry first contact and never enter the contact search.
            bb = res.part.bounding_box()
            reach = max(bb.size.X, bb.size.Y) / 2.0
            chord_sag = (0.5 * bandless.spoke_thickness_mm) ** 2 / (2 * bandless.outer_radius_mm)
            record("no material outside the running surface",
                   reach <= bandless.outer_radius_mm + 1e-6,
                   f"{reach:.5f} mm vs R {bandless.outer_radius_mm:.5f}")
            record("tips reach the running surface (within the chord sagitta)",
                   reach >= bandless.outer_radius_mm - 2 * chord_sag,
                   f"short by {(bandless.outer_radius_mm - reach) * 1e3:.1f} um, "
                   f"chord sagitta {chord_sag * 1e3:.1f} um")
            record("width unchanged", abs(bb.size.Z - bandless.width_mm) < 1e-6,
                   f"{bb.size.Z:.4f} mm")

            _, f_b = tessellate(res.part, tolerance_mm=0.05)
            watertight, n_bad = is_watertight(f_b)
            record("mesh is watertight", watertight,
                   "closed" if watertight else f"{n_bad} non-manifold edges")

            # Phase is inert with a band and decisive without one. If it did nothing, every
            # bandless FEA run would silently sample whatever phase the spoke count gave.
            lowest = min(vx.to_tuple()[1] for vx in res.part.vertices())
            record("a spoke tip sits at the contact point",
                   abs(lowest + bandless.outer_radius_mm) < 2 * chord_sag,
                   f"lowest material at y = {lowest:.4f} mm, "
                   f"contact at {CONTACT_ANGLE_DEG:.0f} deg -> y = {-bandless.outer_radius_mm:.4f}")

            turned = build_wheel(
                WheelParams(rim_thickness_mm=0.0,
                            spoke_phase_deg=phase_for_tip_contact(WheelParams().n_spokes,
                                                                  on_tip=False)),
                TPU95A,
            )
            gap_lowest = min(vx.to_tuple()[1] for vx in turned.part.vertices())
            record("the gap phase pulls material away from the contact point",
                   gap_lowest > lowest + 1e-3,
                   f"tip phase {lowest:.4f} mm vs gap phase {gap_lowest:.4f} mm")

    # --- 11. the L claw ---------------------------------------------------------------
    # `tip_hook_mm != 0`: a radial leg, a filleted right angle, and a foot lying along the
    # running surface (family `T7L`). The section exists because the offset outline is where
    # this topology fails, and it fails by *self-intersecting* — which OCCT may refuse, or
    # may accept into a solid with a reversed patch that still reports a plausible volume.
    if 11 in want:
        print("\n11. L claw (tangential foot at the tip)")
        from wheelopt.cad.centreline import spoke_outline
        from wheelopt.fea.loadcase import phase_for_tip_contact

        hook_mm = 12.0
        lclaw = WheelParams(
            outer_radius_mm=60.0, rim_thickness_mm=0.0, n_spokes=12,
            spoke_thickness_mm=6.0, claw_taper_ratio=0.6,
            tip_hook_mm=hook_mm,
            spoke_phase_deg=phase_for_tip_contact(12),
        )
        plain = replace(lclaw, tip_hook_mm=0.0)
        record("screening accepts it", is_feasible(check_design(lclaw, TPU95A)))
        record("a foot with a shear band is rejected, not ignored",
               any(x.name == "hook_needs_bandless"
                   for x in check_design(replace(lclaw, rim_thickness_mm=3.0), TPU95A)))

        # The outline must not cross itself. Checked here rather than trusted to the kernel:
        # a self-intersecting face is the specific way an offset right angle goes wrong, and
        # the bend radius (0.75 t_tip against a half-thickness of 0.5) is what prevents it.
        outline = spoke_outline(lclaw, 0)
        record("the offset outline does not cross itself",
               not _self_intersects(outline), f"{len(outline)} points")

        res = build_wheel(lclaw, TPU95A)
        if record("builds without error", res.ok):
            n = len(res.part.solids())
            record("is a single solid", n == 1, f"{n} solids")

            bb = res.part.bounding_box()
            reach = max(bb.size.X, bb.size.Y) / 2.0
            record("no material outside the running surface",
                   reach <= lclaw.outer_radius_mm + 1e-6,
                   f"{reach:.5f} mm vs R {lclaw.outer_radius_mm:.5f}")
            # A foot is material the plain claw does not have, at the radius where it is
            # heaviest. If the hook were silently doing nothing the volumes would match.
            plain_res = build_wheel(plain, TPU95A)
            record("the foot adds material",
                   (res.brep_volume_m3 or 0) > (plain_res.brep_volume_m3 or 0) * 1.02,
                   f"{(plain_res.brep_volume_m3 or 0) * 1e6:.2f} -> "
                   f"{(res.brep_volume_m3 or 0) * 1e6:.2f} cm^3")

            _, faces = tessellate(res.part, tolerance_mm=0.05)
            watertight, n_bad = is_watertight(faces)
            record("mesh is watertight", watertight,
                   "closed" if watertight else f"{n_bad} non-manifold edges")

            # The point of the foot: contact over an arc rather than at a point, so the axle
            # falls less between claws. Closed form, checked against the geometry that
            # produced it rather than against itself.
            record("the foot cuts the polygon drop",
                   lclaw.polygon_drop_mm < 0.5 * plain.polygon_drop_mm,
                   f"{plain.polygon_drop_mm:.2f} -> {lclaw.polygon_drop_mm:.2f} mm")
            record("the reported contact patch is the foot, not the tip",
                   abs(lclaw.contact_patch_mm - hook_mm) < 1e-9,
                   f"{lclaw.contact_patch_mm:.1f} mm (tip is "
                   f"{lclaw.tip_thickness_mm:.1f} mm)")

            # Mirroring the whole claw must be exact. **Both signs, not just the hook's** —
            # this check failed at 2.3e-5 when it flipped the foot alone, and the geometry was
            # right: with a bowed leg, (+bow, +foot) and (+bow, -foot) are a C and an S, two
            # genuinely different claws. The true mirror flips the curvature too, and then the
            # volumes agree to 1e-15. Recorded because a 2e-5 discrepancy is exactly the size
            # that gets waved through as tolerance when it is really a wrong test.
            mirrored = build_wheel(
                replace(lclaw, tip_hook_mm=-hook_mm,
                        spoke_curvature_1_per_mm=-lclaw.spoke_curvature_1_per_mm),
                TPU95A,
            )
            rel = abs((mirrored.brep_volume_m3 or 0) - (res.brep_volume_m3 or 1)) / (
                res.brep_volume_m3 or 1)
            record("mirroring the claw (foot AND bow) is exact", rel < 1e-12, f"{rel:.1e}")

            # ...and flipping only the foot is *not* a mirror, so it must NOT come out equal.
            # Without this, a hook that silently ignored its sign would pass the check above.
            foot_only = build_wheel(replace(lclaw, tip_hook_mm=-hook_mm), TPU95A)
            rel_foot = abs((foot_only.brep_volume_m3 or 0) - (res.brep_volume_m3 or 1)) / (
                res.brep_volume_m3 or 1)
            record("flipping only the foot gives a different claw, as it must",
                   rel_foot > 1e-9, f"{rel_foot:.1e} on a bowed leg")

    # --- summary ----------------------------------------------------------------------
    n_fail = sum(1 for c in results if c.status == FAIL)
    print("\n" + "=" * 72)
    print(f"{len(results) - n_fail}/{len(results)} checks passed")
    if n_fail:
        print("\nFailed:")
        for c in results:
            if c.status == FAIL:
                print(f"  - {c.name}" + (f" ({c.detail})" if c.detail else ""))
        print("\nDo not proceed to the FEA stage until these are resolved.")
    else:
        print("\nCAD stage looks sound. Next: docs/plan/16-first-week.md step 3 (CalculiX).")
    print("=" * 72)
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
