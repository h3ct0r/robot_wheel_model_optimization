# Architecture decision records

One file per non-obvious decision. **Read the relevant ADR before proposing a change to
simulator choice, modelling fidelity, or the CAD toolchain** — several plausible-sounding
alternatives were evaluated and rejected for specific documented reasons.

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-mujoco-over-isaac-for-inner-loop.md) | MuJoCo hosts the inner loop; Isaac soft bodies rejected (no static friction) | accepted |
| [0002](0002-reduced-order-model-over-fem.md) | Reduced-order ring model calibrated from FEA; **FEM never in the loop** | accepted |
| [0003](0003-build123d-over-openscad.md) | build123d for CAD; STEP required for FEA and material region tagging | accepted |
| [0004](0004-chrono-as-ground-truth.md) | Chrono ANCF as computational ground truth; never in the loop | accepted |
| [0005](0005-calculix-for-batch-fea.md) | CalculiX for batch quasi-static FEA; FEniCSx as escape hatch | accepted |
| [0006](0006-multi-objective-not-scalarised.md) | Pareto search on CVaR; no scalarisation; measured termination | accepted |
| [0007](0007-coacd-for-convex-decomposition.md) | CoACD over V-HACD for collision geometry | accepted |

## Conventions

- Number sequentially. Never renumber.
- **Supersede, don't delete.** When a decision changes, write a new ADR that supersedes the
  old one and mark the old one `superseded by ADR-NNNN`. The reasoning that turned out wrong
  is as valuable as the reasoning that turned out right.
- Keep them short — roughly one page. Context, decision, alternatives *with specific reasons*,
  consequences, revisit-if.
- The **"Revisit if"** section is the most valuable part. It states the observable condition
  that would justify reopening the question, which prevents both premature churn and stubborn
  adherence.
- Use [`0000-template.md`](0000-template.md) as the starting point.
