# 02 — Research questions and novelty

## Framing

> **For a mobile robot with a fixed chassis and drivetrain, how should wheel geometry
> and material compliance be jointly chosen for rigid-obstacle mobility — and can that choice
> be made in simulation cheaply enough to search, yet accurately enough to transfer to
> FDM-printed TPU hardware?**

## Sub-questions

- **RQ1 (Design).** How do compliant wheel topologies (compliant-spoke, monolithic soft,
  soft-tread-on-rigid-hub) trade off against rigid baselines across obstacle capability,
  energy, speed and ride quality? Where does compliance stop paying?

- **RQ2 (Method).** Can a reduced-order compliance model, calibrated offline from FEA and
  surrogated across the design space, support closed-loop dynamic design optimisation at a
  throughput that full FEM cannot?

- **RQ3 (Validity).** How much of a simulation-derived ranking survives a change of physics
  engine and transfer to hardware — and is the loss *systematically larger* for compliant
  designs than rigid ones?

- **RQ4 (Robustness).** Given only literature material parameters, how much does
  material-model uncertainty degrade design selection, and does optimising under material
  uncertainty recover it?

RQ2 is the strongest methodological contribution. RQ4 turns an equipment limitation into a
research question, which is the right way to handle a constraint that cannot be removed.

## Where the novelty lives

| # | Contribution | Why defensible | Risk |
|---|---|---|---|
| **C1** | **FEA-calibrated reduced-order compliance models for design search** — geometry → ROM parameters surrogate, enabling thousands of closed-loop dynamic evaluations of compliant wheels | NPT literature uses quasi-static FEA + response surfaces; robotics uses rigid contact. Nobody bridges to dynamic closed-loop obstacle traversal | Medium — this is the technical core |
| **C2** | **Open benchmark suite** — terrains, metrics, baselines, reference designs, reference material models | No standard benchmark exists; every paper invents its own | Low |
| **C3** | **Compliant-vs-rigid Pareto study for small-robot obstacle traversal** | NPT work is road tires and rolling resistance; this is obstacle climbing at 10 cm scale | Low–medium |
| **C4** | **Design under material-model uncertainty** — optimise with randomised hyperelastic parameters, measure the selection penalty | Rarely done; directly addresses the honest limitation that printed elastomer properties are poorly known | Low |
| **C5** | **Sim-to-real audit quantifying whether compliance widens the reality gap** | The gap is universally acknowledged, almost never measured comparatively across material classes | Medium — needs hardware |

**Recommended framing:** C1 as the methodological headline, C3 as the empirical result, C2 as
the released artifact, C4 and C5 as the validity analysis. This is a coherent paper, and
arguably a coherent thesis.

## Positioning statements to use in writing

- Against the rover co-design paper: *they optimise continuous geometry of rigid wheels on
  deformable soil; we search across discrete compliant topologies on rigid obstacles, with a
  reduced-order compliance model that makes closed-loop dynamic search tractable.*
- Against the NPT literature: *they optimise quasi-static or steady-rolling road-tire
  performance at full scale; we optimise dynamic obstacle traversal at robot scale with
  FDM-manufacturable geometry and a closed sim-to-real loop.*
