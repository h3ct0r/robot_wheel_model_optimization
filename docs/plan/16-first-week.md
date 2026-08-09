# 16 — The first week: ROM feasibility spike

**Do this before building anything else.** It is the highest-information week available, and
it tests the single assumption the whole project rests on.

## Goal

Determine whether an FEA-calibrated segmented-ring model reproduces compliant-wheel behaviour
well enough to optimise against — before investing in infrastructure that assumes it does.

## Steps

| # | Task | Time |
|---|---|---|
| 1 | Freeze the robot spec in `configs/robot.yaml` | half day |
| 2 | build123d script producing **one** compliant-spoke wheel from a parameter dict, exporting STEP + STL + mass properties | 1 day |
| 3 | CalculiX: STEP → mesh → Mooney-Rivlin card from literature → radial compression against a flat plate **and against a step edge** → extract `k_r(δ)` and contact patch | 1.5 days |
| 4 | MuJoCo: 24-segment ring model, joint stiffness fitted to `k_r(δ)`. Compare static load–deflection to the FEA | 1 day |
| 5 | Drive it at a 50 mm step in MuJoCo alongside a rigid wheel of the same radius | half day |
| 6 | **Look hard at the result** | half day |

## What step 6 is checking

- Does the compliant wheel **envelop** the step edge?
- Is its **contact patch larger** than the rigid wheel's under the same load?
- Does it **climb better** and **roll worse**?
- Does loaded rolling radius **decrease** with load?

## Decision

**If it looks like physics:** the project is viable. Commit to the plan in `11-phases.md`.

**If it doesn't:** a week has been spent instead of six months. Fall back to `T6` (soft tread
on rigid hub) — a much easier modelling problem, still a legitimate project, with narrower
claims.

Either way, record the outcome in `docs/experiments/log.md`.

## Deliberately out of scope this week

No optimiser. No scenario suite. No caching layer. No multiple topology families. No material
randomisation. The only question is whether the ROM idea holds.
