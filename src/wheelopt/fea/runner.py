"""Orchestrate one FEA evaluation: CAD -> mesh -> deck -> ccx -> parsed result.

**This module never raises.** Invariant 4: a diverged solve, a failed mesh, a missing
binary or a truncated output file are ordinary outcomes when running nonlinear contact FEA
on hundreds of soft structures, and every one of them must arrive at the caller as a typed
:class:`~wheelopt.fea.results.FeaResult`. ADR-0005 predicts a meaningful failure rate and
asks for it to be logged as a pipeline health metric, which only works if failures are
values.

CalculiX is a subprocess with file I/O, not a library call (ADR-0005). That is acceptable
precisely because this stage is offline and cached (invariant 1).
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import numpy as np

from ..cad.materials import MaterialSpec
from ..cad.params import WheelParams
from .cache import SOLVER_UNKNOWN, cache_dir_for, fea_cache_key
from .deck import DeckError, build_deck
from .extract import (
    build_contact_patch,
    build_load_curve,
    detect_buckling,
    fea_violations,
    loaded_radius,
    loop_area_fraction,
    spoke_stress,
)
from .hyperelastic import UnknownMaterial, for_material
from .indenter import build_indenter
from .loadcase import LoadCase, MeshSpec, SolverSpec
from .parse import parse_dat, parse_sta
from .results import FeaResult, FeaStatus, SolverDiagnostics, failure

__all__ = ["find_ccx", "solver_identity", "run_load_case", "REPO_CACHE_ROOT"]

REPO_CACHE_ROOT = Path(__file__).resolve().parents[3] / "data" / "cache" / "fea"

#: CalculiX signals fatal problems in stdout as often as through the exit code; these are
#: the phrases that mean "the answer in the .dat file is not trustworthy".
_ERROR_MARKERS = (
    "*ERROR",
    "increment size smaller than minimum",
    "job finished with errors",
)


def _usable(candidate: Path | None) -> bool:
    return bool(candidate and candidate.exists() and os.access(candidate, os.X_OK))


def find_ccx(explicit: Path | str | None = None) -> Path | None:
    """Locate the CalculiX executable. Returns ``None`` rather than raising.

    **An explicit path is final.** If the caller names a solver and it is not usable, the
    answer is ``None``, not "some other CalculiX found on ``PATH``". Falling back would
    mean a run pinned to one solver build silently produced results from another — and
    since the solver identity goes into the cache key, those results would be filed under
    the wrong key too.

    Otherwise: ``$WHEELOPT_CCX``, ``$CCX_PATH``, ``ccx`` on ``PATH``, the active conda
    prefix.
    """
    if explicit is not None:
        candidate = Path(explicit)
        return candidate if _usable(candidate) else None

    candidates: list[Path | None] = [
        Path(os.environ["WHEELOPT_CCX"]) if os.environ.get("WHEELOPT_CCX") else None,
        Path(os.environ["CCX_PATH"]) if os.environ.get("CCX_PATH") else None,
    ]
    found = shutil.which("ccx")
    if found:
        candidates.append(Path(found))
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        candidates.append(Path(prefix) / "bin" / "ccx")

    for candidate in candidates:
        if _usable(candidate):
            return candidate
    return None


def solver_identity(ccx: Path | None) -> str:
    """A string identifying the solver stack, for the cache key.

    Different CalculiX versions can give different answers to the same nonlinear contact
    problem, so the version belongs in the key. When no binary is present this returns the
    ``SOLVER_UNKNOWN`` sentinel so that deck generation and key computation still work —
    results computed under that identity are never written to the cache.
    """
    if ccx is None:
        return SOLVER_UNKNOWN
    try:
        out = subprocess.run(
            [str(ccx), "-v"], capture_output=True, text=True, timeout=30
        )
        text = (out.stdout + out.stderr).strip().splitlines()
        version = next((ln.strip() for ln in text if "ersion" in ln), "")
        version = version.replace("This is Version", "").strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return SOLVER_UNKNOWN
    try:
        import gmsh  # noqa: F401

        gmsh_version = gmsh.__version__  # type: ignore[attr-defined]
    except Exception:  # pragma: no cover - gmsh optional
        gmsh_version = "none"
    return f"ccx-{version}+gmsh-{gmsh_version}"


def _run_ccx(
    ccx: Path, workdir: Path, job: str, solver: SolverSpec
) -> tuple[int, str, bool]:
    """Run the solver. Returns ``(returncode, output, timed_out)``."""
    env = dict(os.environ)
    env["OMP_NUM_THREADS"] = str(solver.n_threads)
    env["CCX_NPROC_EQUATION_SOLVER"] = str(solver.n_threads)
    env["CCX_NPROC_STIFFNESS"] = str(solver.n_threads)

    proc = subprocess.Popen(
        [str(ccx), job],
        cwd=str(workdir),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,  # own process group, so a timeout can kill the OpenMP kids
    )
    try:
        out, _ = proc.communicate(timeout=solver.timeout_s)
        return proc.returncode, out or "", False
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):  # pragma: no cover
            proc.kill()
        out, _ = proc.communicate()
        return -1, out or "", True


def run_load_case(
    params: WheelParams,
    material: MaterialSpec,
    load_case: LoadCase | None = None,
    *,
    mesh_spec: MeshSpec | None = None,
    solver: SolverSpec | None = None,
    step_path: Path | None = None,
    cache_root: Path | None = None,
    ccx_path: Path | str | None = None,
    use_cache: bool = True,
    keep_workdir: bool = True,
) -> FeaResult:
    """Evaluate one load case on one design. **Never raises.**

    Args:
        params: the wheel.
        material: printed material, for both density and the hyperelastic fit.
        load_case: what to press it against. Defaults to flat-plate radial compression.
        mesh_spec: discretisation.
        solver: how to invoke CalculiX.
        step_path: an existing STEP to reuse. Built from ``params`` when omitted.
        cache_root: where results live. Defaults to ``data/cache/fea``.
        ccx_path: explicit solver path, overriding the search.
        use_cache: reuse an existing completed run for the same key.
        keep_workdir: keep the deck and raw output. Wanted for triage; the artefacts are
            gitignored.

    Returns:
        A typed result. Check ``.ok`` before reading any physical field.
    """
    load_case = load_case or LoadCase()
    mesh_spec = mesh_spec or MeshSpec()
    solver = solver or SolverSpec()
    cache_root = Path(cache_root) if cache_root else REPO_CACHE_ROOT

    ccx = find_ccx(ccx_path)
    identity = solver_identity(ccx)

    try:
        hyper = for_material(material, params.spoke_thickness_mm)
    except UnknownMaterial as exc:
        return failure(FeaStatus.DECK_INVALID, load_case, "", str(exc))

    key = fea_cache_key(
        params, material, hyper, load_case, mesh_spec, identity, solver
    )
    workdir = cache_dir_for(cache_root, key)

    if use_cache:
        cached = _load_cached(
            workdir, params, load_case, key,
            _section_thickness_m(params, mesh_spec),
        )
        if cached is not None:
            return cached

    if ccx is None:
        return failure(
            FeaStatus.SOLVER_MISSING,
            load_case,
            key,
            "no CalculiX binary found. Install it with "
            "`conda install -c conda-forge calculix`, or set $WHEELOPT_CCX.",
        )

    # --- geometry -------------------------------------------------------------------
    from .mesh import MeshFailure, mesh_step

    # Everything is written into the scratch directory and moved into place only on
    # success. Building the STEP straight into the final cache directory would both create
    # a directory that looks like a cache entry before there is anything in it, and lose
    # the geometry when `_promote` replaces that directory with the scratch one.
    tmp = workdir.parent / f"{workdir.name}.tmp-{os.getpid()}"
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True, exist_ok=True)

    try:
        if step_path is None:
            step_path = _build_step(params, material, tmp)
    except Exception as exc:  # noqa: BLE001 - invariant 4: nothing escapes
        shutil.rmtree(tmp, ignore_errors=True)
        return failure(FeaStatus.CAD_FAILED, load_case, key, f"{type(exc).__name__}: {exc}")

    try:
        if mesh_spec.dimension == 2:
            # The section is built from the same centreline module the solid is, so the
            # STEP above is not read here. It is still written: it is what the design hash
            # names, and a 2-D result should stay traceable to the geometry it came from.
            from .section2d import mesh_claw_sector, mesh_section

            mesh = (
                mesh_claw_sector(params, mesh_spec,
                                 hub_span_deg=mesh_spec.claw_hub_span_deg)
                if mesh_spec.claw_sector
                else mesh_section(params, mesh_spec)
            )
        else:
            mesh = mesh_step(Path(step_path), params, mesh_spec)
    except MeshFailure as exc:
        shutil.rmtree(tmp, ignore_errors=True)
        return failure(FeaStatus.MESH_FAILED, load_case, key, str(exc))
    except Exception as exc:  # noqa: BLE001
        shutil.rmtree(tmp, ignore_errors=True)
        return failure(
            FeaStatus.MESH_FAILED, load_case, key, f"{type(exc).__name__}: {exc}"
        )

    # --- deck -----------------------------------------------------------------------
    try:
        indenter = build_indenter(
            load_case.kind,
            load_case.indenter,
            params.outer_radius_mm * 1e-3,
            params.width_mm * 1e-3,
            dimension=mesh_spec.dimension,
        )
        bundle = build_deck(
            mesh, indenter, params, material, hyper, load_case, solver,
            design_hash=params.design_hash(), cache_key=key,
        )
    except (DeckError, ValueError) as exc:
        shutil.rmtree(tmp, ignore_errors=True)
        return failure(FeaStatus.DECK_INVALID, load_case, key, str(exc))
    except Exception as exc:  # noqa: BLE001
        shutil.rmtree(tmp, ignore_errors=True)
        return failure(
            FeaStatus.DECK_INVALID, load_case, key, f"{type(exc).__name__}: {exc}"
        )

    # --- solve ----------------------------------------------------------------------
    (tmp / "job.inp").write_text(bundle.text)

    started = time.perf_counter()
    returncode, output, timed_out = _run_ccx(ccx, tmp, "job", solver)
    wall = time.perf_counter() - started

    sta_path = tmp / "job.sta"
    sta = parse_sta(sta_path.read_text()) if sta_path.exists() else parse_sta("")
    (tmp / "ccx.stdout.log").write_text(output)

    diagnostics = SolverDiagnostics(
        n_increments=sta.n_increments,
        n_cutbacks=sta.n_cutbacks,
        wall_seconds=wall,
        n_nodes=bundle.n_nodes,
        n_elements=bundle.n_elements,
        completed_fraction=sta.final_time / load_case.step_period,
        log_tail="\n".join(output.splitlines()[-40:]),
    )

    if timed_out:
        return _finish_failure(
            tmp, workdir, keep_workdir,
            failure(FeaStatus.SOLVER_TIMEOUT, load_case, key,
                    f"exceeded {solver.timeout_s:.0f} s", diagnostics),
        )

    lowered = output.lower()
    hit_error = any(m.lower() in lowered for m in _ERROR_MARKERS)
    incomplete = sta.final_time < load_case.step_period - 1e-6

    if returncode != 0 and not hit_error and not incomplete:
        return _finish_failure(
            tmp, workdir, keep_workdir,
            failure(FeaStatus.SOLVER_CRASHED, load_case, key,
                    f"ccx exited {returncode}", diagnostics),
        )
    if hit_error or incomplete:
        return _finish_failure(
            tmp, workdir, keep_workdir,
            failure(FeaStatus.SOLVER_DIVERGED, load_case, key,
                    f"stopped at t={sta.final_time:.3f} of {load_case.step_period:.1f}"
                    f" after {sta.n_cutbacks} cutbacks", diagnostics),
        )

    # --- parse ----------------------------------------------------------------------
    result = _extract_result(
        tmp, params, load_case, key, bundle.ref_node,
        bundle.slave_nodes, bundle.slave_coords_m, diagnostics,
        _section_thickness_m(params, mesh_spec),
    )
    if not result.ok:
        return _finish_failure(tmp, workdir, keep_workdir, result)

    _write_bundle_meta(tmp, bundle, diagnostics)
    _promote(tmp, workdir, keep_workdir)
    return result


def _finish_failure(
    tmp: Path, workdir: Path, keep: bool, result: FeaResult
) -> FeaResult:
    """Keep failed runs on disk for triage, but never as a cache hit.

    A failure directory is left at ``<key>.failed`` rather than ``<key>``, so a rerun does
    not mistake it for a completed result while the evidence stays available.
    """
    if keep:
        dest = workdir.parent / f"{workdir.name}.failed"
        shutil.rmtree(dest, ignore_errors=True)
        try:
            tmp.replace(dest)
        except OSError:  # pragma: no cover
            shutil.rmtree(tmp, ignore_errors=True)
    else:
        shutil.rmtree(tmp, ignore_errors=True)
    return result


def _promote(tmp: Path, workdir: Path, keep: bool) -> None:
    """Move a completed run into place atomically.

    Campaigns run for days and will be interrupted; writing in place would leave a
    half-populated directory that the next run reads as a cache hit.
    """
    if not keep:
        shutil.rmtree(tmp, ignore_errors=True)
        return
    shutil.rmtree(workdir, ignore_errors=True)
    workdir.parent.mkdir(parents=True, exist_ok=True)
    try:
        tmp.replace(workdir)
    except OSError:  # pragma: no cover - cross-device
        shutil.copytree(tmp, workdir, dirs_exist_ok=True)
        shutil.rmtree(tmp, ignore_errors=True)


def _extract_result(
    workdir: Path,
    params: WheelParams,
    load_case: LoadCase,
    key: str,
    ref_node: int,
    slave_nodes: np.ndarray,
    slave_coords_m: np.ndarray,
    diagnostics: SolverDiagnostics,
    section_thickness_m: float | None = None,
) -> FeaResult:
    """Parse a completed run directory into a result. Shared by the solve and cache paths.

    Everything downstream of the solver lives here, so a cached run and a fresh one go
    through exactly the same code — and a change to extraction takes effect on cached runs
    without re-solving, which matters because extraction is the part most likely to change
    while the physics does not.
    """
    dat = workdir / "job.dat"
    if not dat.exists():
        return failure(
            FeaStatus.PARSE_FAILED, load_case, key, "no .dat in the run directory",
            diagnostics,
        )
    try:
        blocks = parse_dat(dat.read_text())
        curve = build_load_curve(blocks, load_case, ref_node)
    except Exception as exc:  # noqa: BLE001 - invariant 4
        return failure(
            FeaStatus.PARSE_FAILED, load_case, key, f"{type(exc).__name__}: {exc}",
            diagnostics,
        )
    if curve is None:
        return failure(
            FeaStatus.PARSE_FAILED, load_case, key,
            "no force/displacement history in the .dat", diagnostics,
        )

    patch = build_contact_patch(
        blocks, curve, slave_nodes, slave_coords_m, section_thickness_m
    )
    peak_stress, p95_stress = spoke_stress(blocks)
    buckled, buckling_load = detect_buckling(curve)

    return FeaResult(
        status=FeaStatus.OK,
        load_case=load_case,
        cache_key=key,
        curve=curve,
        patch=patch,
        peak_von_mises_pa=peak_stress,
        p95_von_mises_pa=p95_stress,
        loaded_radius_m=loaded_radius(curve, params),
        buckling_detected=buckled,
        buckling_load_n=buckling_load,
        hysteresis_loss_factor=None,  # see FeaResult; hyperelasticity has no dissipation
        loop_area_fraction=loop_area_fraction(curve),
        violations=fea_violations(curve, load_case, params, p95_stress, buckling_load),
        diagnostics=diagnostics,
    )


def _write_bundle_meta(workdir: Path, bundle, diagnostics: SolverDiagnostics) -> None:
    """Persist what extraction needs, so a cache hit costs no gmsh and no solver."""
    np.savez_compressed(
        workdir / "bundle.npz",
        ref_node=np.array([bundle.ref_node], dtype=np.int64),
        slave_nodes=bundle.slave_nodes,
        slave_coords_m=bundle.slave_coords_m,
        diagnostics=np.array(
            [
                diagnostics.n_increments,
                diagnostics.n_cutbacks,
                diagnostics.wall_seconds,
                diagnostics.n_nodes,
                diagnostics.n_elements,
                diagnostics.completed_fraction,
            ],
            dtype=np.float64,
        ),
    )


def _section_thickness_m(params: WheelParams, mesh_spec: MeshSpec) -> float | None:
    """Out-of-plane extent the contact patch should assume, or None in 3-D.

    In plane strain every slave node sits at z = 0, so the patch has no measurable width
    and the section thickness *is* the width by definition. Derived here rather than stored
    in the run metadata so that a cached 2-D result and a fresh one agree without a cache
    format change.
    """
    return params.width_mm * 1e-3 if mesh_spec.dimension == 2 else None


def _load_cached(
    workdir: Path,
    params: WheelParams,
    load_case: LoadCase,
    key: str,
    section_thickness_m: float | None = None,
) -> FeaResult | None:
    """Re-parse a completed run. ``None`` means "not a usable cache entry, go solve"."""
    meta = workdir / "bundle.npz"
    if not meta.exists() or not (workdir / "job.dat").exists():
        return None
    try:
        with np.load(meta) as data:
            ref_node = int(data["ref_node"][0])
            slave_nodes = data["slave_nodes"]
            slave_coords = data["slave_coords_m"]
            d = data["diagnostics"]
        diagnostics = SolverDiagnostics(
            n_increments=int(d[0]),
            n_cutbacks=int(d[1]),
            wall_seconds=float(d[2]),
            n_nodes=int(d[3]),
            n_elements=int(d[4]),
            completed_fraction=float(d[5]),
            log_tail="(cached)",
        )
    except Exception:  # noqa: BLE001 - a corrupt cache entry is a miss, not a crash
        return None

    result = _extract_result(
        workdir, params, load_case, key, ref_node, slave_nodes, slave_coords, diagnostics,
        section_thickness_m,
    )
    return result if result.ok else None


def _build_step(params: WheelParams, material: MaterialSpec, workdir: Path) -> Path:
    """Build and export geometry for this design, into the run directory."""
    from ..cad.compliant_spoke import build_wheel
    from ..cad.export import export

    result = build_wheel(params, material)
    if not result.ok or result.part is None:
        raise RuntimeError(
            "geometry rejected: "
            + "; ".join(f"{v.name}({v.severity.value})" for v in result.violations)
        )
    workdir.mkdir(parents=True, exist_ok=True)
    return export(result.part, params, workdir).step


def summarise(results: list[FeaResult]) -> dict[str, int]:
    """Failure-rate breakdown across a batch. The health metric ADR-0005 asks for."""
    counts: dict[str, int] = {}
    for r in results:
        counts[r.status.value] = counts.get(r.status.value, 0) + 1
    return dict(sorted(counts.items()))
