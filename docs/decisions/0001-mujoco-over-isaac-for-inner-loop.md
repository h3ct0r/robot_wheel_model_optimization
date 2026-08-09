# ADR-0001 — MuJoCo as the inner-loop engine; Isaac soft bodies rejected

**Status:** accepted
**Date:** 2026-08-04

## Context

The inner loop must evaluate thousands of candidate wheels in closed-loop dynamic simulation
over an obstacle-traversal scenario suite. Candidate engines: MuJoCo (+MJX / MuJoCo-Warp),
Isaac Sim / Isaac Lab (PhysX), Gazebo, Project Chrono.

The wheels are compliant TPU structures, so the engine's handling of flexible materials is a
first-order concern, not a detail.

## Decision

**MuJoCo hosts the inner loop**, running a reduced-order compliant wheel built from ordinary
rigid bodies and joints (see ADR-0002). Isaac soft bodies are rejected outright.

## Alternatives considered

**Isaac Sim / PhysX FEM soft bodies — rejected.**
PhysX 5 has FEM soft bodies with tetrahedral simulation and collision meshes, but **PhysX soft
body simulation currently does not support static friction**. This is documented as the cause
of grasping failures in manipulation research. For a wheel, no static friction means no
correct traction, no stick–slip, and no correct climbing behaviour — it breaks the primary
metric of the entire project. Isaac Lab's deformable support is additionally described as
limited, and the deformable body schema is still under development and subject to change
between releases.

**MuJoCo `flex` — rejected for the inner loop.**
`flex` is genuine deformable-body support, and notably handles closed-loop topologies of any
genus (a wheel rim qualifies). But (a) the fast trilinear mode reduces the whole body to 24
DOF via bounding-box corners, far too coarse for individually bending spokes, and (b) MJX
scales poorly with collisions — contact cost scales with the number of *possible* contacts
rather than active ones, because JAX requires static shapes at compile time. That removes the
GPU-batching advantage which was the main reason to choose MuJoCo in the first place.

**Gazebo — rejected for the inner loop.**
Roughly real-time wall clock, weaker impulsive-contact solver, high asset reload cost per
design. Retained for Phase 4+ system integration only, where running the real ROS 2 stack
unmodified is the actual benefit.

**Chrono — rejected for the inner loop, adopted as ground truth.** See ADR-0004.

## Consequences

- Compliance must be modelled as a reduced-order structure, not a continuum. This is the
  project's central technical risk and its central contribution (ADR-0002).
- MuJoCo's `solref` / `solimp` are a numerical regulariser only. **They must never be used as
  a stand-in for material compliance.** Recorded as invariant 8 in `CLAUDE.md`.
- Batching remains available because the ROM uses ordinary rigid bodies and joints.
- A separate FEA tier is required, since MuJoCo cannot produce the stiffness data the ROM
  needs (ADR-0005).

## Revisit if

- PhysX soft bodies gain static friction support and it stabilises across releases.
- MJX contact handling changes to scale with active rather than possible contacts.
- A `flex` mode appears with per-spoke resolution at acceptable throughput.

## References

- [MuJoCo 3 flex discussion](https://github.com/google-deepmind/mujoco/discussions/1101)
- [MuJoCo XLA (MJX) docs](https://mujoco.readthedocs.io/en/stable/mjx.html)
- [Isaac Lab deformable object tutorial](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/01_assets/run_deformable_object.html)
- [TacEx (documents PhysX static-friction limitation)](https://arxiv.org/html/2411.04776v1)
