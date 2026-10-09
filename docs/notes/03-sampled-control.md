# Sampled control: the practical threshold is π/n + ωΔt/2

**Summary.** Section VII reports that randomly placed agents "will eventually diverge if φ is
only 1% above π/n". With the discrete law (43) at the stated parameters (ω = 2, Δt = 0.1;
the CDC 2008 Fig. 4 caption), that does not happen. The practical threshold is about

    φ ≈ π/n + ω Δt / 2,

which is 25% above π/n for n = 8 and 51% above for n = 16. It approaches π/n as ωΔt → 0. The
theory is about the continuous system, and there π/n is exact.

Reproduce: `pytest tests/test_errata.py -k sampled`, or open the demo presets
"Video: 16 agents, φ = 0.20" and "Lemma 3: regular 9-gon on the edge".

## Why

Under (43) an agent turns for whole time steps after losing its target. The overshoot puts
the target back inside the windshield, and the time-averaged bearing comes out at about
φ − ωΔt/2 + vΔt/R. Near a ring of radius R the agents therefore behave as if the windshield
were narrower by about ωΔt/2.

Below the threshold, most random starts settle into a rigidly rotating regular n-gon of radius

    R* ≈ max(v/ω, vΔt / (π/n − φ + ωΔt/2)),     V* = 2n R* sin(π/n).

This fit matches simulations to 0.1–3% for Δt = 0.1…0.01. Some starts stay instead in a
bounded, non-regular oscillation. For n = 8, φ = 0.35 and Δt = 0.1, 8 of 10 seeds reach the
ring (V* ≈ 4.2–4.4), and seeds 1 and 5 oscillate with V ≈ 4.8–5.9. Above the threshold, the
polygon grows at about the continuous rate.

`tests/test_errata.py::test_sampled_threshold_coefficient` pins the ωΔt/2 shift for n = 8 and
16. The fit accuracy, the seed counts and the applet window below come from longer offline runs.

## Our applet's video

The 2019 screen capture of the original Java applet shows 16 agents in three runs:

* φ = 0.2 forms a stable ring ("rendezvous");
* φ = 0.22 and φ = 0.4 diverge.

π/16 = 0.196, so φ = 0.2 is 1.9% above π/n, and its ring is a discretization effect. With ω = 2,
both observations hold together only for 0.0073 < ωΔt < 0.047. The applet therefore probably
used a smaller effective ωΔt than the 0.2 in the CDC caption; its source code has not survived.
At ωΔt = 0.2, φ = 0.22 does not diverge (0 of 6 random seeds over 3000 s).

## Other observations

* **Plateau values.** Fig. 6(b) reports a final V = 0.22 from V₀ = 27.69 for n = 8. With unit
  speed, the regular ring that most starts reach has V* ≈ 4.3 (≥ 3.06 even as Δt → 0),
  independent of V₀. Matching 0.22 would need v/ω about 20× smaller in the figure's units. We
  could not determine those units.
* **Traps.** With ρ ≈ 0 and φ > π/n, a sizable fraction of random starts end in bounded states
  that neither rendezvous nor diverge. Examples are star polygons circling at R ≈ v/ω and
  translating formations.
* **Assignment direction.** In the video, agent i heads toward i + 1. Section VII says each
  agent is assigned "the one with a preceding identifier". The two are equivalent up to
  relabeling; our code follows the text.
