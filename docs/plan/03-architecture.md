# 03 — System architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                            ORCHESTRATOR                              │
│        Python · Hydra configs · DuckDB/Parquet experiment store      │
└───┬──────────────────────────────────────────────────────┬───────────┘
    │ design vector θ = (geometry, material, controller)    │ results
    ▼                                                       ▲
┌────────────────────────────┐                              │
│ 1. PARAMETRIC CAD          │  build123d                   │
│    θ → BREP solid          │  → STEP (FEA + archival)     │
│    + constraint pre-filter │  → STL (visual/collision)    │
│    + mass & inertia        │  → region material map       │
└──────────┬─────────────────┘                              │
           ▼                                                │
┌────────────────────────────┐                              │
│ 2a. COMPLIANCE ROM STAGE   │  ◀── the project's core      │
│  quasi-static FEA (offline)│  radial/lateral/torsional    │
│  → stiffness curves,       │  stiffness, contact patch    │
│    contact patch(load),    │  vs load, hysteresis loop,   │
│    hysteresis, buckling    │  buckling load, peak stress  │
│  → fit lumped ring params  │                              │
│  → GP surrogate: θ → ROM   │  (skips FEA after ~300 pts)  │
└──────────┬─────────────────┘                              │
           ▼                                                │
┌────────────────────────────┐                              │
│ 2b. ASSET PIPELINE         │  manifold repair → CoACD     │
│  ROM → segmented ring MJCF │  → hull budget check         │
│  rigid parts → collision   │  → joint stiffness/damping   │
└──────────┬─────────────────┘                              │
           ▼                                                │
┌────────────────────────────┐                              │
│ 3. SIMULATION TIERS        │                              │
│  T0  analytic screen       │  ms      geometric filter    │
│  T1  MuJoCo + ROM ring     │  1–20 s  MAIN WORKHORSE      │
│  T2a quasi-static FEA      │  min     ROM generation      │
│  T2b Chrono ANCF FEA tire  │  hr      verification, top-K │
│  T3  Gazebo + ROS 2        │  RT      integration         │
│  T4  Hardware rig          │  hr      final validation    │
└──────────┬─────────────────┘                              │
           ▼                                                │
┌────────────────────────────┐                              │
│ 4. METRIC AGGREGATION      │  per-scenario → CVaR         │
│                            │  → objectives + constraints ─┘
└──────────┬─────────────────┘
           ▼
┌────────────────────────────┐
│ 5. OPTIMISER               │  Sobol DoE → mixed-variable
│                            │  MOBO (qLogNEHVI) → CMA-ES
└────────────────────────────┘
```

## Caching

Everything is content-addressed: `hash(θ + pipeline version + ROM version)` → cache key. With
an FEA stage in the pipeline this stops being a nicety and becomes essential — a re-run of an
existing design must never recompute.

Cache layers, each independently keyed:

| Layer | Key includes | Typical cost avoided |
|---|---|---|
| CAD geometry | geometry params, CAD module version | seconds |
| FEA results | geometry + material params, FEA config version | minutes |
| ROM fit | FEA results hash, fitting procedure version | seconds |
| Simulation rollout | ROM hash + scenario + seed + sim config version | seconds to minutes |

## Failure handling

Every stage returns a typed result. Failure modes that must be representable without raising:

- geometry infeasible (constraint violation vector)
- mesh non-manifold or repair failed
- convex decomposition exceeded hull budget
- FEA failed to converge
- FEA detected buckling within the operating envelope
- ROM fit residual exceeded tolerance
- simulation diverged or exceeded step budget

A campaign must survive all of these. Log the rate of each as pipeline health metrics.
