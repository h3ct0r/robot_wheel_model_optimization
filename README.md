# robot_wheel_model_optimization

Automatic optimisation of the 3D layout of **compliant (TPU) wheels** for a mobile
terrestrial robot — a 400 × 300 × 200 mm, ~10 kg four-wheel skid-steer platform carrying
24.5 N per wheel.

A parametric CAD model generates candidate wheels. Each is characterised by offline FEA,
reduced to a fast lumped-parameter ring model, and evaluated in closed-loop dynamic
simulation across an obstacle-traversal scenario suite. A multi-objective optimiser proposes
the next candidates. Designs are constrained to be FDM-printable, and the pipeline closes to
hardware.

**New here?** Read [`docs/OVERVIEW.md`](docs/OVERVIEW.md) — the project in plain language,
with a glossary of every acronym. No prior knowledge assumed.

**Status:** Phase 0. The one-week feasibility spike is **complete and its gate passed** — a
compliant wheel clears a 50 mm step against a rigid wheel's 20 mm at matched mass, radius and
rotational inertia, with 5/5 of the qualitative signatures the plan asked for. Current work is
the `T7` compliant-claw family. Open items are numbered in
[`docs/plan/TODO.md`](docs/plan/TODO.md).

## Quick start

```bash
source /opt/homebrew/Caskroom/miniforge/base/etc/profile.d/conda.sh
conda activate conda3.12
python -m unittest discover -s tests -t .          # 508 tests, ~17 s
python scripts/explore.py --spokes 8 --no-sim      # a design → one HTML page
```

Full setup, and why the first line is needed, is in [Environment](#environment) below.

## Where things are

| Path | Contents |
|---|---|
| [`docs/OVERVIEW.md`](docs/OVERVIEW.md) | **Start here** — plain-language summary and glossary |
| [`CLAUDE.md`](CLAUDE.md) | Working context — current phase, invariants, conventions |
| [`docs/plan/`](docs/plan/00-index.md) | The project plan, split into loadable sections |
| [`docs/decisions/`](docs/decisions/README.md) | Architecture decision records |
| [`docs/experiments/log.md`](docs/experiments/log.md) | Append-only experiment log |
| [`configs/robot.yaml`](configs/robot.yaml) | The platform spec — sized to the real chassis, not yet frozen |
| [`docs/plan/TODO.md`](docs/plan/TODO.md) | **Open work, numbered** — read before picking something up |
| `src/wheelopt/cad/` | Parametric geometry → STEP + STL + mass properties |
| `src/wheelopt/fea/` | CalculiX driver: STEP → mesh → load cases → stiffness curve |
| `src/wheelopt/rom/` | The segmented ring and its fit. Pure numpy except `mjcf.py` |
| `src/wheelopt/sim/` | MuJoCo scenario runners — the step-climb rig and its signatures |
| `src/wheelopt/report.py` | The self-contained HTML report behind `scripts/explore.py` |
| `scripts/` | Entry points. `explore.py` is the manual playground; `verify_*.py` are the gates |

## Environment

One conda environment, `conda3.12` (Python 3.12). Conda rather than a venv for two specific
reasons: **CalculiX has no PyPI distribution** (it is a Fortran binary, invoked as a
subprocess — [ADR-0005](docs/decisions/0005-calculix-for-batch-fea.md)), and **PyChrono has none either**, which the
ground-truth tier will need in Phase 1 ([ADR-0004](docs/decisions/0004-chrono-as-ground-truth.md)). Everything else
installs from PyPI into the same environment.

### Activating it

```bash
source /opt/homebrew/Caskroom/miniforge/base/etc/profile.d/conda.sh
conda activate conda3.12
```

**The `source` line is not optional on this machine.** Running `conda activate conda3.12` on
its own fails with `Run 'conda init' before 'conda activate'`, because the conda-init block in
`~/.zshrc` is commented out *and* points at `/Users/h3ct0r/miniconda3`, a prefix that no longer
exists. (The `mamba` block just below it points at the same dead prefix, so the `mamba` alias
in that shell is also stale.) The working install is the homebrew miniforge one under
`/opt/homebrew/Caskroom/miniforge/base`.

To make plain `conda activate conda3.12` work permanently, initialise the shell against the
*live* prefix once — this edits `~/.zshrc`, so it is left as your call rather than done for you:

```bash
/opt/homebrew/Caskroom/miniforge/base/bin/conda init zsh
```

Then delete the two dead blocks in `~/.zshrc` that reference `miniconda3`, and open a new
shell.

**Do not use the system `python3`.** Homebrew's 3.14.6 ships a broken `pyexpat` that breaks
`pip` itself.

### Creating it from scratch

```bash
brew install --cask miniforge
conda create -y -n conda3.12 -c conda-forge python=3.12 calculix=2.23
conda activate conda3.12
pip install -e '.[cad,fea,sim,viz,dev]'
```

### Checking it

```bash
python -c "import sys; print(sys.executable)"   # .../envs/conda3.12/bin/python
ccx -v                                          # CalculiX Version 2.23
python -m unittest discover -s tests -t .       # 508 tests
```

`ccx -v` prints the version and then **exits 201**. That is normal — it is not an error, and
nothing in the pipeline reads that exit code.

### What needs what

The extras are separate on purpose: a screening-only worker installs none of them, and the
numpy-only layers stay testable on a machine with no CAD kernel and no simulator.

| Extra | Brings | Needed by |
|---|---|---|
| *(none)* | numpy only | screening, centreline geometry, mass properties, the ring maths and its fit |
| `cad` | build123d / OCCT | building solids, STEP and STL export |
| `fea` | gmsh | meshing. **The solver `ccx` is a conda binary, not a pip package** |
| `sim` | mujoco | the ring in dynamics, the step-climb rig, rendering |
| `viz` | matplotlib | `--plot-pdf` and the HTML report |

## Running it

Everything below assumes the environment is active. Times are on this machine; the FEA cache
is content-addressed, so a repeated design is near-instant.

### The playground — start here

One design (or several) through the whole chain into a single self-contained HTML page:
section drawing, load curve, tangent stiffness, contact patch, fitted segment law, step-climb
signatures and an embedded animation.

```bash
python scripts/explore.py --spokes 8 --thickness 6 --no-sim      # ~1 min cold, ~2 s cached
python scripts/explore.py --compare spokes=6,10,14 --no-sim      # shared axes
python scripts/explore.py --rim-thickness 0 --claw-taper 0.6 --spoke-phase -90
```

Output lands in `data/explore/explore.html`. Every panel is labelled with the tier that
produced it and whether that tier **screens or decides**; panels whose numbers are currently
untrustworthy carry a caution banner. Read the banner before quoting the number.

### Individual stages

```bash
# Screen a design without building geometry. Milliseconds, no OCCT.
python scripts/gen_wheel.py --screen-only --spokes 14 --thickness 6.0

# Build it: STEP + STL + mass properties.
python scripts/gen_wheel.py --radius 85 --spokes 12 --profile curved --out data/wheels

# FEA, mesh and deck only, no solver. The developer loop.
python scripts/run_fea.py --dry-run --tiny --case flat --case step_edge

# FEA on the fast 2-D screening tier. Seconds, not hours.
python scripts/run_fea.py --plane-strain --size-spoke 0.0025 --size-rim 0.003 \
    --size-hub 0.002 --contact-stiffness 5 --case flat --plot-pdf

# Fit the segmented ring to the FEA curve, and optionally press it in MuJoCo.
python scripts/run_rom.py --tiny --mujoco

# Drive the fitted ring at a step beside a rigid wheel; judge the five signatures.
python scripts/run_step.py --tiny --sweep

# Render the step climb: GIF plus a compliant-vs-rigid contact sheet.
python scripts/render_step.py --tiny
```

### Verification batteries

Run these after touching the layer they cover. They need the real kernels — the unit tests
deliberately cannot catch what these catch.

```bash
python scripts/verify_cad.py            # 48 checks against OCCT
python scripts/verify_fea.py --full     # 30 checks against CalculiX (tens of minutes uncached)
python -m unittest discover -s tests -t .
```

### A caveat worth reading once

The 2-D plane-strain tier is what makes the loop interactive, and it is a **deliberate
fidelity reduction, not an approximation that improves with refinement**. It cannot see
lateral spoke buckling at all. Measured against the 3-D solid at matched settings: force ratio
0.90, stiffness ratio 0.86, patch length 0.95. Use it to screen; let the 3-D tier decide.

## Approach in one paragraph

Full FEM of a compliant wheel inside an optimisation loop is 3–5 orders of magnitude too
slow; rigid-body simulation cannot represent contact-patch growth, load-dependent rolling
radius, or hysteretic loss at all. The approach borrows the automotive tire field's solution:
generate a reduced-order flexible-ring model from FEA, run that in a fast rigid-body engine,
and — after a few hundred FEA runs — surrogate the FEA step away entirely so that design
*search* becomes tractable. Validity is checked by cross-engine rank correlation against
Chrono ANCF FEA tires, and finally against printed hardware.

## Key decisions at a glance

- **MuJoCo** hosts the inner loop. Isaac soft bodies are unusable here — PhysX soft bodies
  don't support static friction. ([ADR-0001](docs/decisions/0001-mujoco-over-isaac-for-inner-loop.md))
- **Reduced-order ring model**, calibrated offline from FEA. FEM never runs in the loop.
  ([ADR-0002](docs/decisions/0002-reduced-order-model-over-fem.md))
- **build123d** for CAD, because FEA needs STEP and multi-material needs region tagging.
  ([ADR-0003](docs/decisions/0003-build123d-over-openscad.md))
- **Chrono ANCF** as computational ground truth, never in the loop.
  ([ADR-0004](docs/decisions/0004-chrono-as-ground-truth.md))
- **Pareto search on CVaR**, no scalarisation, measured termination.
  ([ADR-0006](docs/decisions/0006-multi-objective-not-scalarised.md))

## Licence

TBD.
