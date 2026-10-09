# Turn rate: Proposition 16 is wrong as stated; Proposition 17 has two slips

Reproduce: `pytest tests/test_errata.py -k "prop16 or prop17"`.

## Proposition 16, eq. (40): ω > v_max/ρ does not keep the target

Eq. (40) claims that ω_i > v_max/ρ is enough for an agent to "either keep its target within
its windshield or merge with its target".

**What the proof misses.** The proof (p. 432, Fig. 11) treats the windshield as a cone
rotating about a fixed point A. It concludes that the target must move across the line of
sight faster than ρω_i to escape. But the pursuer also moves. With the target on the edge of
the windshield at bearing φ and distance ℓ > ρ, the line of sight turns at

    λ' = (v_j sin(·) + v_i sin φ) / ℓ   ≤   (v_j + v_i sin φ) / ℓ.

The second term comes from the pursuer's own translation, and (40) leaves it out. The
condition that actually keeps the target is

    ω_i > (v_max + v_max sin φ) / ρ      (at most 2 v_max / ρ),

which is `rwc.bounds.tracking_omega`. Our CDC 2008 version (Proposition 8) had ω > 2/ρ for unit
speed. That bound is correct, and tight only at φ = π/2.

**Counterexample** (unit speeds, ρ = 1, φ = π/4, ω = 1.2 > v_max/ρ). The pursuer starts at
the origin heading +x and uses the instantaneous law (2)–(3) with ε = 0.02. The target starts
at (0.849376, 0.847679) with heading 2.355194, at distance 1.2 and bearing φ − 10⁻³, and
drives straight across the line of sight.

* The target leaves the windshield at t = 0.0045, at distance 1.197 > ρ.
* It stays out for 1.28 time units (closest approach 1.05) and is never merged.
* A second scenario never ends: the target turns, drives straight, then circles on the
  pursuer's own turning circle. It stays at bearing 0.8236 > φ and distance 1.2227 > ρ forever.
* With ω = 2.1 > (1 + sin φ)/ρ = 1.707, neither scenario escapes.

A realistic 3-cycle in which every agent runs the law shows the same escape. The later results
are unaffected, because Propositions 17 and 18 assume ω ≥ 3v_max/ρ.

## Proposition 17, eq. (50): v_min should be v_max

When speeds vary, the proof needs the disc that is safe from being found by a turning agent.
That disc has radius ρ minus the agent's turning radius. The worst case is the *largest* turning
radius, v_max/ω, but (50) uses v_min/ω.

Counterexample to (50) as a criterion: ρ = 1, v_j = v_max = 1, v_min = 0.1, ω = 2.5.

* (50) as printed holds: 5.03 < 6.03.
* The v_max version fails: 5.03 < 3.77 is false.

The corrected condition, ω > (2v_j + v_max)/ρ, is still implied by (41) with strict inequality.

Two further remarks:

* At ω = 3v_max/ρ exactly, with v_i = v_j = v_max, (48) holds with equality, so (41) should
  read `>`.
* If an agent's speed changes *during* a turn, its trace is not a circle and drifts. Under the
  proof's own safe-disc argument the requirement then rises to ω ρ / v_max > 4 − v_min/v_max,
  which is above the 3 of (41). So (41) covers speeds that are constant over each turn.
