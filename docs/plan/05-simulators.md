# 05 — Simulator capability analysis

Engine by engine, specifically on the question: **can this handle flexible materials?**

## MuJoCo

**Soft contact (`solref` / `solimp`) is NOT material compliance.** Those parameters make the
*constraint solver* soft — a springy, penetrable contact point. They do not give a contact
patch that grows with load, a correct pressure distribution, hysteretic energy loss, or a
wheel whose effective radius shrinks under load. For a project where compliance is the object
of study, using soft contact as a stand-in is not a simplification, it is a category error.
Fine as a *numerical* regulariser; not a compliance model. (Invariant 8.)

**MuJoCo `flex` is real deformable-body support.** MuJoCo 3 introduced flexes: collections of
segments (1D), triangles (2D) and tetrahedra (3D). Critically, **flexes are not defined in a
hierarchical kinematic tree, so they can simulate closed-loop structures of any topological
genus — rubber bands, cloth**
([MuJoCo 3 discussion](https://github.com/google-deepmind/mujoco/discussions/1101)). A
compliant wheel rim is exactly such a closed loop. Deformation is controlled either by
equality constraints (stiff bodies) or passive forces (soft bodies), with elasticity plugins
implementing passive forces from discretised continuum mechanics models
([plugin README](https://github.com/google-deepmind/mujoco/blob/main/plugin/elasticity/README.md)).

**Two limitations disqualify flex from the inner loop:**

- **The fast flex mode is far too coarse.** In the trilinear option, only the 8 corners of the
  bounding box are free to move, with interior vertices computed by trilinear interpolation —
  24 DOF for the whole object. Fine for a squashy blob; cannot represent 24 individually
  bending spokes.
- **MJX scales badly with contacts.** MJX scales poorly with the number of collisions, causing
  significant slowdowns for deformable bodies, and contact cost scales with the number of
  *possible* contacts rather than active ones, because JAX requires static shapes at compile
  time. **This removes the GPU-batching advantage that was the entire reason to pick MuJoCo.**

**Verdict:** MuJoCo stays — as the host for a *reduced-order* compliant wheel built from
ordinary rigid bodies and joints (`06-compliance-rom.md`), not as an FEM solver. In that role
it is excellent and fast.

## Isaac Sim / PhysX

PhysX 5 has FEM soft bodies (inheriting from the former NVIDIA Flex library), with a
tetrahedral simulation mesh plus a tetrahedral collision mesh
([Isaac Lab deformable tutorial](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/01_assets/run_deformable_object.html)).

**However — decisively — PhysX soft body simulation currently does not support static
friction.** This is a documented failure mode that prevents effective grasping in
manipulation research. For a *wheel*, no static friction means no correct traction, no
stick-slip, no correct climbing behaviour — i.e. it breaks the primary metric. Isaac Lab's
deformable support is additionally described as limited, the deformable body schema is still
under development, and it may change between releases.

**Verdict: do not use Isaac soft bodies for this project.** Revisit only if static friction
lands and stabilises. See ADR-0001.

## Project Chrono — the fidelity anchor

Chrono::Vehicle offers **full finite element representations of tires using ANCF or Reissner
shell elements** — the most accurate and most computationally expensive tire models — and can
**account for simultaneous deformation in tire and soil** for high-fidelity off-road
simulation. There is a documented
[`ANCFTire` class](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_a_n_c_f_tire.html),
and shipped co-simulation test programs (`test_VEH_HMMWV_Cosimulation`,
`test_VEH_tireRig_Cosimulation`) for vehicles with deformable tires on granular terrain, with
vehicle and terrain coupled by explicit force-displacement co-simulation advanced on
non-blocking parallel threads.

The exact combination needed — a *flexible* wheel interacting with a *deformable or rigid*
ground, inside a *full vehicle* model — already exists, is validated, and is open source.

**Verdict: Chrono is ground truth.** Use it for (a) generating and validating reduced-order
models, (b) cross-engine auditing, (c) final verification of top designs. Never in the inner
loop. See ADR-0004.

## Gazebo

No meaningful soft-body support for this purpose. Keep for T3 system integration only, where
the wheel can be a calibrated rigid approximation and the value is running the real ROS 2
stack unmodified.

## Dedicated FEA (required tier)

For the offline quasi-static stage, a nonlinear hyperelastic FEA solver with contact:

| Tool | Verdict |
|---|---|
| **CalculiX** | **Recommended default.** Nonlinear structural, thermal and contact problems with material nonlinearity and large deformation; file-driven solver designed for batch automation. Abaqus-like input decks. Free, scriptable, well-documented for hyperelastic rubber |
| **FEniCSx** | Strong second choice. Cross-validated against Abaqus on hyperelastic large-deformation problems with sub-percent agreement, and a large-deformation viscoelasticity theory for elastomers has been implemented in it ([Int. J. Solids Struct., 2024](https://www.sciencedirect.com/science/article/abs/pii/S0020768324003822)). Choose if viscoelasticity must be done properly |
| **Chrono::FEA** | Use if one codebase for FEA and multibody is preferred; ANCF shells suit thin rims and spokes |
| **Abaqus / ANSYS** | If a licence exists, use it — the NPT literature is almost entirely Abaqus, so published results are easier to match |

See ADR-0005.

## Recommended tiering

| Tier | Engine | Role | Cost/design | Share of budget |
|---|---|---|---|---|
| **T0** | Analytic | Geometric + constraint screen | ~ms | all designs |
| **T1** | **MuJoCo + ROM ring model** | Closed-loop dynamic evaluation — **the workhorse** | 1–20 s | ~95% |
| **T2a-2D** | **CalculiX, plane-strain section (CPE6)** | Quasi-static FEA → ROM parameters, screening | ~20 s–1 min | first ~300, then surrogated |
| **T2a-3D** | **CalculiX, solid (C3D10)** | The same, at reference fidelity | **~20 h at nominal size** | a handful, and anything where buckling decides |
| **T2b** | **Chrono ANCF + co-sim** | Ground-truth verification, cross-engine audit | 0.5–4 h | top ~30–50 |
| **T3** | Gazebo + ROS 2 | System integration with the real stack | real-time | final phase |
| **T4** | Hardware rig | Validation | hours | ~6–8 designs |

### Why T2a is split

The 3-D tier was the only one until 2026-08-07, when the platform re-spec made the nominal
wheel big enough to measure honestly: 50 779 C3D10 / 279 336 DOF at ~23 min per increment,
about **20 hours per sweep**. Coarsening does not recover it — the 3 mm shear band, the 7 mm
spoke and the 4 mm bore set the element size regardless of the size field, and the coarsest
mesh that still meshes is only 25% smaller. That is roughly 30× the cost this table used to
assume, and it breaks the "first ~300 designs" column outright.

The plane-strain section (`wheelopt.fea.section2d`, `--plane-strain`) is the answer: same
deck generator, same contact, same extraction, 20 468 DOF instead of 279 336. Calibrated
against the 3-D tier at matched frictionless settings on the debug preset — peak force ratio
**0.90**, `k_r` ratio **0.86**, contact-patch length ratio **0.95** — and `verify_fea.py`
section 6 asserts those stay within ±25% rather than assuming they are 1.

What it cannot do, and why 3-D stays:

- **No out-of-plane behaviour at all.** Lateral spoke buckling, sidewall taper and any
  spanwise pattern are invisible. Buckling is a hard constraint in
  [`04-design-space.md`](04-design-space.md), so any design near it goes to 3-D.
- **Frictional contact needs a softened penalty**, which is the default since #12
  (2026-08-09) rather than a flag this tier has to remember: `contact_stiffness_factor = 5`
  and a 4 mm floor under the element size in `factor × E / element_size`. The floor is the
  part specific to this tier — its 2.5 mm section mesh sits below it, and without the floor
  the penalty rises as the mesh refines until a frictional run diverges *at any factor*. The
  answer moves ~1% across a tenfold change in penalty, so this is conditioning rather than
  physics.
- **The 0.90 ratio is one measurement on one geometry.** It is not yet known whether it is a
  constant to divide out or a function of topology.
