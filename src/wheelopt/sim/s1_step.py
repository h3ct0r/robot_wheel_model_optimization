"""Scenario S1 — Step. The first scenario, the first metric, one hard-coded controller.

`docs/plan/08-metrics.md`: *S1 Step, randomised over height 10–200 mm, edge friction and
approach angle ±15°; primary metric, step height at P(success) = 0.9.* And the threshold-metric
fix that goes with it — a fixed ladder of heights across seeds, a logistic success curve, and a
continuous height rather than a bisected one.

So S1 is a **ladder × seeds** grid, not a search. Ten heights times eight terrain seeds is
eighty runs; each run is the whole robot driven at a step under a constant-throttle controller,
and the answer is a fit across all of them. Bisection would be cheaper and is the thing
`08-metrics.md` explicitly rejects: it gives a jittery discontinuous signal that poisons a GP
surrogate, and this whole project is a surrogate-driven search.

**A terrain seed is a terrain, not a coin flip per run.** The same seed gives the same friction
and the same approach angle at *every* rung of the ladder, so a seed is one realisation of the
world being made progressively harder. Re-sampling per rung would make the eight seeds eighty
independent conditions and the success curve would measure the sampler as much as the wheel.

**Each rung is its own row.** `wheelopt.store` keys a row on (design, scenario, seed, material
realisation), and eighty runs at eight seeds would collide eight ways — which the determinism
gate would then read as an evaluation repeated ten times with ten different answers. The height
therefore goes in the scenario name (``S1_step/h=0.050``) so every rung has its own ``run_id``,
and the ladder is recovered with ``WHERE scenario LIKE 'S1_step/%'``.

The controller is constant throttle, and that is the plan's own "one hard-coded controller".
Nothing here steers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..metrics.threshold import ThresholdFit, fit_threshold
from ..platform import PlatformSpec
from ..rom.ring import RingSpec
from ..store import RunRecord, RunStatus, pipeline_versions
from .rover import RoverSpec, run_rover

__all__ = ["S1_NAME", "S1Config", "S1Outcome", "run_s1", "terrain_for_seed"]

#: Scenario name, and the prefix every rung's own name is built from.
S1_NAME = "S1_step"


@dataclass(frozen=True, slots=True)
class S1Config:
    """The ladder and what is randomised across it."""

    #: Rung heights, metres. `08-metrics.md` randomises S1 over 10–200 mm; ten rungs at 20 mm
    #: is the coarse default, and a caller studying one wheel should narrow it around that
    #: wheel's own radius rather than pay for rungs it will never clear.
    heights_m: tuple[float, ...] = tuple(np.round(np.arange(0.02, 0.21, 0.02), 3))
    #: Terrain seeds. `08-metrics.md` asks for k >= 8.
    n_seeds: int = 8
    #: Edge friction is sampled log-uniformly in this range. Log rather than uniform because
    #: the difference between 0.3 and 0.4 matters far more to a climb than 0.9 to 1.0.
    friction_range: tuple[float, float] = (0.3, 1.0)
    #: Approach yaw is sampled uniformly in ±this, degrees.
    approach_deg: float = 15.0
    duration_s: float = 6.0
    throttle: float = 1.0
    #: Which chassis collides — `sim.rover`'s ``chassis_collision``. In the rung digest
    #: because the 2026-08-12 default flip (box → primitives) moves every metric on a rung
    #: the chassis touches, and a changed number under an unchanged run_id is exactly the
    #: lie invariant 5 exists to prevent.
    chassis_collision: str = "primitives"

    def __post_init__(self) -> None:
        if not self.heights_m:
            raise ValueError("the ladder needs at least one rung")
        if len(set(self.heights_m)) != len(self.heights_m):
            raise ValueError("duplicate rung: a repeated height is a repeated run_id")
        if any(h <= 0 for h in self.heights_m):
            raise ValueError("every rung height must be positive")
        if self.n_seeds < 1:
            raise ValueError("need at least one terrain seed")
        low, high = self.friction_range
        if not 0.0 < low <= high:
            raise ValueError(f"friction_range must be positive and ordered; got {(low, high)}")

    def rung_name(self, height_m: float) -> str:
        """Scenario name for one rung: the height, plus a digest of everything else that
        shapes the run.

        The digest is invariant 5 arriving here the hard way. The first version was the
        height alone, and the cross-machine gate caught it on its first day: a ladder run at
        ``--duration 5`` produced the **same run_ids** as the 6-second reference with
        different metrics in them — which reads as non-determinism and is actually two
        different experiments sharing one name. Anything that changes the numbers is in the
        key, by default.

        Two named exclusions, per the invariant's own rule that exclusions are justified one
        at a time: ``n_seeds``, because a row already carries its own seed and the population
        size does not change what seed 3 measured; and ``heights_m`` as a tuple, because the
        rung's own height is in the name and the rest of the ladder does not touch this run.
        """
        from ..hashing import content_digest

        digest = content_digest({
            "duration_s": self.duration_s,
            "throttle": self.throttle,
            "friction_range": list(self.friction_range),
            "approach_deg": self.approach_deg,
            "chassis_collision": self.chassis_collision,
        })[:8]
        return f"{S1_NAME}/h={height_m:.3f}@{digest}"


def terrain_for_seed(seed: int, config: S1Config) -> tuple[float, float]:
    """``(friction, approach_deg)`` for one terrain seed. Pure, and the same at every rung.

    Seeded from the seed alone — not from (seed, height) — for the reason in the module
    docstring: a seed is a realisation of the world, and the ladder makes that one world
    harder. Determinism follows from `numpy`'s PCG64 being specified rather than incidental,
    so this reproduces across machines, which is what the Phase 0 gate asks of it.
    """
    rng = np.random.default_rng(seed)
    low, high = config.friction_range
    friction = float(np.exp(rng.uniform(np.log(low), np.log(high))))
    approach = float(rng.uniform(-config.approach_deg, config.approach_deg))
    return friction, approach


@dataclass(frozen=True, slots=True)
class S1Outcome:
    """The ladder, what it did, and the fitted height. ``records`` go straight to the store."""

    fit: ThresholdFit
    heights_m: np.ndarray
    successes: np.ndarray
    records: list[RunRecord] = field(default_factory=list)
    #: Runs that never produced a verdict — no MuJoCo, an invalid model, a bad scenario.
    #: Excluded from the fit and still stored: a design whose runs fail is a result.
    n_failed: int = 0

    @property
    def ok(self) -> bool:
        return self.fit.ok

    def summary(self) -> str:  # pragma: no cover - display only
        tail = f", {self.n_failed} run(s) failed" if self.n_failed else ""
        return f"{self.fit.summary()}{tail}"


def run_s1(
    platform: PlatformSpec,
    config: S1Config,
    *,
    design_hash: str,
    wheel_radius_m: float,
    wheel_width_m: float,
    wheel_mass_kg: float,
    spec: RingSpec | None = None,
    material_realisation: int = 0,
    params: dict[str, Any] | None = None,
) -> S1Outcome:
    """Drive the full ladder and fit the success curve. Never raises for a failed run.

    Args:
        platform: the robot, from ``configs/robot.yaml``.
        config: the ladder and its randomisation.
        design_hash: what the rows are about. ``WheelParams.design_hash()``.
        wheel_radius_m, wheel_width_m, wheel_mass_kg: the wheel under test.
        spec: when given, the wheels get the **ring's** rotational inertia rather than a solid
            cylinder's. See ``rover.build_rover_mjcf`` — a solid cylinder has half a ring's,
            and four wheels' worth is a real share of a ~9 kg robot's inertia.
        material_realisation: index into the sampled material population. Part of a row's
            identity (invariant 7), not of what it measured.
        params: extra provenance to carry on every row — the design's own parameters, the
            material, the fit residual that produced ``spec``.
    """
    heights: list[float] = []
    successes: list[bool] = []
    records: list[RunRecord] = []
    n_failed = 0
    # The platform is in the run identity (invariant 5, third instance of the same bug):
    # re-measuring the robot must produce NEW run_ids, not the old ids with new numbers —
    # the manifest gate would read the latter as non-determinism.
    versions = {**pipeline_versions(), "platform": platform.digest()}

    for seed in range(config.n_seeds):
        friction, approach = terrain_for_seed(seed, config)
        for height in config.heights_m:
            scenario = RoverSpec(
                step_height_m=float(height), friction=friction, approach_deg=approach,
                duration_s=config.duration_s, throttle=config.throttle,
            )
            result = run_rover(platform, scenario, wheel_radius_m=wheel_radius_m,
                               wheel_width_m=wheel_width_m, wheel_mass_kg=wheel_mass_kg,
                               spec=spec, chassis_collision=config.chassis_collision)
            row_params = {
                **(params or {}),
                "step_height_m": float(height),
                "friction": friction,
                "approach_deg": approach,
                "wheel_radius_m": wheel_radius_m,
                "throttle": config.throttle,
            }
            if not result.ok:
                n_failed += 1
                records.append(RunRecord(
                    design_hash=design_hash, scenario=config.rung_name(float(height)),
                    seed=seed, material_realisation=material_realisation,
                    status=RunStatus.SIM_FAILED, message=result.message, params=row_params,
                    versions=versions,
                ))
                continue
            heights.append(float(height))
            successes.append(bool(result.climbed))
            records.append(RunRecord(
                design_hash=design_hash, scenario=config.rung_name(float(height)),
                seed=seed, material_realisation=material_realisation,
                status=RunStatus.OK, params=row_params,
                metrics={
                    "climbed": float(result.climbed),
                    "distance_m": result.distance_m,
                    "final_clearance_m": result.final_clearance_m,
                    "energy_j": result.energy_j,
                    # Objective 5 (stability). A METRIC, not a diagnostic, since 2026-08-11:
                    # diagnostics are artifact detectors, and this is a number designs are
                    # ranked on. Aggregate with CVaR over seeds like everything else --
                    # stability is a worst-moment property, which is what CVaR rewards.
                    "stability_margin": result.stability_margin,
                },
                diagnostics={
                    "peak_pitch_rad": result.peak_pitch_rad,
                    "peak_roll_rad": result.peak_roll_rad,
                    "chassis_hit_step": float(result.chassis_hit_step),
                },
                versions=versions,
            ))

    if not heights:
        return S1Outcome(
            fit=ThresholdFit(height_m=float("nan"), stderr_m=float("inf"),
                             intercept=float("nan"), slope_per_m=float("nan"),
                             censored="above"),
            heights_m=np.array([]), successes=np.array([], dtype=bool),
            records=records, n_failed=n_failed,
        )
    h = np.array(heights)
    y = np.array(successes)
    return S1Outcome(fit=fit_threshold(h, y), heights_m=h, successes=y,
                     records=records, n_failed=n_failed)
