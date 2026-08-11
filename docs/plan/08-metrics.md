# 08 — Scenarios, objectives and metrics

## Scenario suite

| Scenario | Randomised over | Primary metric |
|---|---|---|
| **S1 Step** | height 10–200 mm, edge friction, approach angle ±15° | Step height at P(success) = 0.9 |
| **S2 Slope** | incline 5–40°, friction 0.3–1.0 | Max sustained gradient |
| **S3 Gap** | width 20–200 mm | Max gap crossed |
| **S4 Rubble** | procedural rock field, size/density | Traverse success rate, mean speed |
| **S5 Flat sprint** | surface friction, 10 m | **Cost of transport (hysteresis-sensitive)**, top speed |
| **S6 Path track** | ~~figure-8 / slalom~~ **spin-in-place proxy** (decision 2026-08-11): yaw rate at differential torque + per-wheel scrub energy | Turn capability and turning cost. Full path-tracking deferred — it sits behind #38's lateral validation *and* a controller, and would measure the controller as much as the wheel. Revisit if designs differ meaningfully on the spin test |
| **S7 Washboard** | sinusoidal ripple, amplitude and wavelength | **RMS chassis acceleration — where compliance wins** |
| **S8 Sustained load** | nominal + 2× load, long duration | Sag, buckling margin, predicted fatigue cycles |

**S5 and S8 are the anti-degenerate scenarios for compliance.** Without S5 the optimiser makes
everything maximally soft; without S8 it makes everything soft enough to collapse.

## Objectives — multi-objective, never scalarised (ADR-0006)

1. **Maximise** obstacle capability index (S1–S4 normalised)
2. **Minimise** cost of transport (S5/S6) — dominated by hysteretic loss for soft wheels
3. **Minimise** ride harshness, RMS vertical chassis acceleration (S5–S7) — compliance's
   payoff axis
4. **Minimise** wheel mass
5. **Maximise** the stability margin (S1–S4, S7) — worst-moment distance to static tip-over,
   ``1 − max(|pitch|/pitch_crit, |roll|/roll_crit)`` with the critical angles derived from
   the platform's own CG height, wheelbase and track
   (`PlatformSpec.tipover_angles_rad`). Added 2026-08-11: "stability" for this project is
   *not tipping over on obstacles and slopes* — rollover and pitch containment — which
   harshness (objective 3, a comfort axis) does not measure. Aggregated like everything
   else, CVaR at 25% over seeds, which for a worst-moment property is exactly the right
   emphasis: a design that is upright on average and tips on the worst seed has tipped.

Report the 4-D Pareto front. Select finalists by hypervolume contribution and by named
preference profiles (obstacle-first, efficiency-first, balanced).

## Logged but not optimised

Slip ratio, peak motor current, **loaded rolling radius**, **contact patch
area vs load**, **peak spoke stress**, **predicted fatigue cycles**, **buckling margin**,
print time and mass, solver warning count, contact-impulse spectrum, FEA convergence
iterations, ROM fit residual.

## The threshold-metric fix

"Maximum step height cleared" is a **discontinuous, noisy threshold**. Bisection on a
stochastic simulator gives a jittery signal that poisons a GP surrogate.

**Instead:** evaluate a fixed ladder of heights (10 heights × 8 seeds), fit a logistic success
curve, and report the **height at which success probability crosses 0.9** as a continuous
quantity with an uncertainty estimate. This one change materially improves optimiser
convergence.

## Aggregation

Each design is evaluated over `k ≥ 8` terrain seeds **×** `m ≥ 4` material realisations.
Aggregate with **CVaR at 25%** (mean of the worst quartile), not the mean. This produces
robust designs and is what hardware transfer requires. (Invariant 7.)
