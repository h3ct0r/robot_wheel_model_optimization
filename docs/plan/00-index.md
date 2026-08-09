# Project plan — index

Automatic optimisation of the 3D layout of compliant (TPU) wheels for a mobile
terrestrial robot.

Read `01` and `02` for orientation. `06` is the technical core. `16` is what to do first.
**`TODO.md` is what is left** — the numbered open list, read before picking up work.

| File | Section | Read when |
|---|---|---|
| [TODO.md](TODO.md) | **Open work, numbered** — plus standing gaps in what is modelled | Picking up work; before proposing something new |
| [01-assessment.md](01-assessment.md) | Honest assessment, prior work, failure modes | Starting out; questioning scope |
| [02-research-questions.md](02-research-questions.md) | Research questions and novelty positioning | Writing, or defending the contribution |
| [03-architecture.md](03-architecture.md) | System architecture, caching, failure handling | Implementing any pipeline stage |
| [04-design-space.md](04-design-space.md) | Topology families, parameters, materials, constraints | Adding a family; writing the CAD layer |
| [05-simulators.md](05-simulators.md) | Engine-by-engine capability analysis and tiering | Considering a simulator change |
| [06-compliance-rom.md](06-compliance-rom.md) | **Technical core** — reduced-order compliance modelling | Almost always |
| [07-materials.md](07-materials.md) | TPU characterisation without equipment; cyclic softening | Material modelling; hardware protocol |
| [08-metrics.md](08-metrics.md) | Scenario suite, objectives, logged metrics | Implementing evaluation |
| [09-optimiser.md](09-optimiser.md) | Optimiser design, baselines, termination | Implementing the search |
| [10-reality-gap.md](10-reality-gap.md) | Domain randomisation, artifact audits, validity gates | Before trusting any result |
| [11-phases.md](11-phases.md) | Phased plan with go/no-go gates | Planning; at every gate |
| [12-risks.md](12-risks.md) | Risk register | At every gate |
| [13-engineering.md](13-engineering.md) | Engineering practices | Setting up infrastructure |
| [14-cad-toolchain.md](14-cad-toolchain.md) | CAD choice and asset pipeline | Writing the CAD/asset layer |
| [15-extensions.md](15-extensions.md) | Extensions once the core works | After the Phase 3 gate, not before |
| [16-first-week.md](16-first-week.md) | Concrete one-week feasibility spike | **Now** |
| [99-sources.md](99-sources.md) | Bibliography | Writing; verifying a claim |

## Decisions

Non-obvious choices and their reasoning live in [`../decisions/`](../decisions/). Read the
relevant ADR before proposing a change to simulator choice, modelling fidelity or CAD
toolchain — several plausible alternatives were evaluated and rejected for specific reasons.
