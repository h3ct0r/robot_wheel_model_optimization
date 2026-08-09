# 09 — Optimiser design

## Stages

**Stage A — Screening (~400–600 designs).** Sobol/LHS over the full space including material
parameters. Sobol sensitivity indices to prune dimensions. Publish the sensitivity table — it
is useful to others and cheap to produce.

**Stage B — Mixed-variable MOBO (~1,500–2,500 designs).** BoTorch/Ax with **qLogNEHVI**, the
noisy expected-hypervolume-improvement acquisition designed for exactly this noisy
multi-objective setting. Handle topology as a **bandit over families** — one GP per family,
allocate each batch to the family with the highest expected hypervolume improvement per unit
cost. Simpler to debug than a single mixed kernel, and it produces the per-family comparison
for RQ1 for free.

**Stage C — Local refinement (~300–500 designs).** CMA-ES or NSGA-II within the top 1–2
families on continuous parameters. BO is globally sample-efficient but usually loses the final
polish to evolution strategies.

**Stage D — Multi-fidelity (optional).** MFBO across T1/T2b. Add only once the single-fidelity
loop is stable; it roughly doubles implementation complexity.

## Controller co-optimisation — mandatory

Jointly optimise 3–5 controller parameters: velocity-loop PI gains, torque ramp limit, slip
threshold. Holding the controller fixed while morphology varies measures "best wheel *for this
controller*", not "best wheel". Compliance makes this worse — a soft wheel needs a different
torque profile than a rigid one.

Also run the **sequential** variant as an ablation (optimise wheel with fixed controller, then
tune controller). The reference co-design paper compares simultaneous vs sequential
strategies, giving a positioning baseline.

## Noise and robustness

Each design over `k ≥ 8` terrain seeds × `m ≥ 4` material realisations, scored with **CVaR at
25%**. This multiplies per-design cost by ~4, which is precisely why the ROM surrogate
(`06-compliance-rom.md`) matters.

## Termination

Fixed budget `B` evaluations, with early stop if the Pareto **hypervolume improvement over
the last 15% of budget** falls below threshold **and** the bootstrap confidence interval on
that improvement excludes a meaningful gain.

**Never "stop when it stops improving" — measure it.** In a noisy multi-objective setting,
"no improvement" is a statement about sampling noise, not convergence.

## Required baselines

Without these, no claim that the optimiser helps is defensible:

- Random search at equal budget
- NSGA-II at equal budget
- Latin hypercube + pick-best at equal budget

Compare on final Pareto hypervolume with bootstrap confidence intervals.
