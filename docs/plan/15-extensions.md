# 15 — Extensions, once the core works

Ordered roughly by ratio of interest to effort. **None of these should be started before the
Phase 3 gate.**

1. **Variable-stiffness / graded infill.** Vary infill density spatially within one wheel —
   stiff near the hub, soft at the rim. Trivially printable, genuinely novel for robot wheels,
   and a natural extension of the existing design space.

2. **Multi-material printing.** Rigid hub, soft spokes, grippy tread in one print. Adds a
   material-placement field to the design space; the strongest engineering extension.

3. **TPMS / lattice spokes.** The NPT field is actively using triply periodic minimal surfaces
   for adaptive directional stiffness. Directly applicable, well-precedented, and the
   homogenisation machinery already supports it.

4. **Terrain-conditional wheel selection.** Learn a mapping from terrain descriptor → best
   wheel from the Pareto set. Practically useful for swappable wheels; reframes the problem as
   contextual optimisation.

5. **Topology optimisation of the spoke field.** Given outer profile and load cases, run
   structural topology optimisation. Complements the parametric search and produces designs
   that wouldn't have been parameterised.

6. **Learned ROM.** Replace the hand-designed ring model with a learned dynamics residual
   trained on FEA/Chrono rollouts. Higher fidelity per unit cost, and a much stronger
   methodological claim than parameter fitting.

7. **Differentiable simulation.** MuJoCo-XLA gives gradients. With an SDF or implicit geometry
   representation within a family, the geometry → simulation path can be made approximately
   differentiable, giving gradient-based design optimisation and a large sample-efficiency
   win.

8. **Co-optimise suspension and wheel compliance.** These are substitutes — optimising one
   with the other fixed systematically misleads. The most scientifically interesting extension
   and the most expensive.

9. **Full co-design with a learned controller.** A universal policy pretrained across
   morphologies via morphological pretraining, so control isn't relearned per design. The
   correct long-term answer to the controller confound, and a paper in itself.

10. **Fatigue-aware design.** Predict cycles-to-failure and put it in the objective rather
    than the constraints. Almost nobody does this, and it is the top reason optimised
    compliant parts fail in the field.

11. **Active / variable-compliance wheels.** Compliance as a control variable —
    tension-adjustable spokes, or a shape-memory element. Very high effort, very high novelty.
