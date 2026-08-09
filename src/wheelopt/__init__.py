"""wheelopt — compliant wheel design optimisation.

Pipeline stages, in dependency order:

    cad     -> parametric geometry (build123d): theta -> STEP + STL + mass properties
    fea     -> batch quasi-static FEA (CalculiX): STEP -> stiffness curves, patch, hysteresis
    rom     -> reduced-order ring model: FEA results -> MuJoCo joint parameters (+ surrogate)
    sim     -> closed-loop dynamic scenario runners (MuJoCo)
    metrics -> metric extraction and robust (CVaR) aggregation
    opt     -> optimiser drivers and baselines

See docs/plan/03-architecture.md. Invariants are listed in CLAUDE.md and are not optional.
"""

__version__ = "0.0.0"
