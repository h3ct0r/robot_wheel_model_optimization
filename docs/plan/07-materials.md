# 07 — Material characterisation with no equipment

Literature values only. Workable, but must be planned around rather than ignored.

## What the literature gives you

- TPU is normally treated as isotropic hyperelastic, but **FDM introduces process-inherent
  anisotropy**, and raster angle clearly affects mechanical properties
  ([Prog. Addit. Manuf., 2024](https://link.springer.com/article/10.1007/s40964-024-00937-x)).
- On model choice: **neo-Hookean fits only the initial linear-elastic zone**, whereas
  **Ogden and Mooney-Rivlin approximate the whole stress–strain curve well**; a **third-order
  Mooney-Rivlin** model accurately described six different specimen groups across FDM
  process-parameter variations ([Polymers, 2025](https://doi.org/10.3390/polym17010026)).
- Infill pattern and density substantially change compression behaviour, studied
  comparatively for TPU across multiple patterns and densities.

**Starting point:** third-order Mooney-Rivlin, one parameter set per
(shore hardness × infill density × pattern), seeded from published fits, with orthotropy
introduced as a stiffness ratio between in-plane and interlayer directions.

## Cheap DIY characterisation — do this anyway

Under €100, and it pays for itself many times over:

| Test | Method | Yields |
|---|---|---|
| **Uniaxial tension** | ISO 37 / ASTM D638 dogbones; calibrated weights or a €20 digital luggage scale as load cell; elongation from photographs with fiducial marks | 10 load points → two- or three-term Mooney-Rivlin fit |
| **Compression** | Printed coupons, weights, dial indicator or calipers. One per infill pattern/density in use | Calibrates homogenised effective properties directly |
| **Damping** | Drop test or free-decay of a printed cantilever, 240 fps phone video; log-decrement | Damping ratio |
| **Hysteresis** | Load/unload a coupon in steps with 30 s holds; plot the loop | Loss factor |
| **Relaxation** | Fixed strain, photograph load reading over 10 min | One- or two-term Prony series |

One week of work. Turns "we used literature values" into "we used literature values verified
against in-house coupon tests," which is a materially stronger claim.

**What is still unavailable:** frequency-dependent viscoelasticity (needs DMA).
**Consequence: hysteretic rolling-resistance predictions will be qualitative, not
quantitative.** Scope claims to *ranking* rather than absolute values, and say so explicitly.

## Turn the limitation into RQ4

Rather than pretending to precision that isn't there, **randomise over material-model
uncertainty in the inner loop**: sample Mooney-Rivlin coefficients from a distribution
spanning published fits for that shore hardness, sample the anisotropy ratio, sample the
damping factor. Then:

- Score designs on **CVaR across material realisations** → designs robust to *not knowing the
  material*, which is what hardware transfer requires.
- Measure the **selection penalty**: how much worse is the design chosen under uncertainty
  than the design chosen with the true parameters (using a held-out "true" parameter set)?
  Clean, quantitative, publishable — and it converts an equipment gap into a contribution.

## The cyclic-softening problem — plan for it now

Under cyclic loading, TPU shows a **steep decline in absorbed energy over the initial
cycles**, after which the response becomes **increasingly dominated by viscous effects, with
continued softening rather than a stable plateau**. TPU's fatigue limit has been reported
around **10.25 MPa**, with cracks initiating at surface micropore aggregations and propagating
at ~45° ([Polymers, 2023](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9958809/)).

Three consequences:

1. **Hardware protocol must specify break-in.** Run every wheel for a fixed number of
   revolutions under nominal load before measurement. Otherwise trial 1 and trial 20 measure
   different wheels and the variance analysis is meaningless.
2. **Report measurements as a function of cycle count**, at least for one wheel, to quantify
   the drift. Genuinely useful to anyone building printed compliant wheels.
3. **Add a stress-margin constraint** on spoke root stress with a large safety factor, and log
   predicted cycles-to-failure as a reported (not optimised) metric.

**Practitioner caveat worth heeding:** TPU is excellent where parts should compress, cushion
or return softly, but is usually *not* the right material for a precise load-bearing spring.
Expect stiffness drift. Design for robustness to stiffness variation rather than for a
knife-edge optimum — which is what the RQ4 approach above already does.
