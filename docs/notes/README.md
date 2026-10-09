# Notes on the paper

When reviving this work we checked every quantitative claim of the TAC 2012 paper and its
CDC 2008 version with an independent computation. These were finite differences, direct
optimization of V̇ over geometry, exact enclosing circles and simulation. We found 55 claims
confirmed. Where something is wrong, loose or missing, it is written up here with a
reproduction. The main reproductions are tests in `tests/test_errata.py`, `tests/test_paper.py`
and `tests/test_oracle.py`. A few numbers come from longer offline runs and are marked as such
in the notes: most of the Thm. 13 table, the Fig. 6(b) plateau values and the applet's ωΔt
window.

| Where | Finding | Kind | Note |
| --- | --- | --- | --- |
| Prop. 16, eq. (40) | ω > v_max/ρ does not keep the target; need ω > v_max(1 + sin φ)/ρ (CDC's 2/ρ is right) | **error** | [02](02-turn-rate.md) |
| Sec. VII | "diverge if φ is only 1% above π/n" fails at the stated ω = 2, Δt = 0.1; threshold ≈ π/n + ωΔt/2 | **error** | [03](03-sampled-control.md) |
| Thm. 12 proof | "V̇_k < −1 for k = 2, 3, 4" is false for k = 4 (−0.885); the theorem still holds | proof slip | [01](01-windshield-bounds.md) |
| Prop. 17, eq. (50) | uses v_min/ω where v_max/ω is needed; (41) should be strict | proof slip | [02](02-turn-rate.md) |
| Eq. (11) + Cor. 9 | `≤` allows φ = π/n, where V̇ has no uniform margin; needs `<` | proof slip | [01](01-windshield-bounds.md) |
| CDC eqs. (2), (3), (7), (11), (12) | signs are right only in a y-down screen frame | convention | [04](04-conventions-and-gaps.md) |
| Eq. (43) | an agent whose target starts out of view never finds it | gap | [04](04-conventions-and-gaps.md) |
| Thm. 4, eq. (11) | conservative for n = 3, 4: exact threshold is π/n | conservative | [01](01-windshield-bounds.md) |
| Thm. 12, eq. (27) | the π/5 cap is unnecessary: the threshold is π/n for every connected graph (numerically, n ≤ 12) | conservative | [01](01-windshield-bounds.md) |
| Thm. 13, eq. (32) | reaches only 14–56% of the true critical angle | conservative | [01](01-windshield-bounds.md) |
| p. 428 | bounded-speed intrees do have a V̇ < 0 guarantee when v_max/v_min < (n−2)/(n−3) | stronger result | [01](01-windshield-bounds.md) |
| Prop. 10 | radius V/(2√3) can be V/4 (closed curve of length L fits in a disc of radius L/4) | stronger result | below |
| Sec. IV-A | freezing merged distances does not make V continuous; redirected edges jump by ≤ ρ | slip | [04](04-conventions-and-gaps.md) |
| Fig. 9 | corner labels miss minus signs | typo | [01](01-windshield-bounds.md) |
| Eq. (28) | sums run i = 0…k (k + 1 terms) | typo | [01](01-windshield-bounds.md) |
| Fig. 6(b) | plateau V = 0.22 not reproducible at unit speed (we get ≈ 4.3); units unknown | unclear | [03](03-sampled-control.md) |

Confirmed, among others: eqs. (8)–(10) for simple and self-intersecting polygons, Lemmas 1–3,
Lemma 5 (including its side note that Θ_i may be empty for n ≤ 5), Lemmas 6–8, Lemma 11 (and
that it is tight), Lemma 14 and Fig. 5, Theorem 13 (sufficiency), the algebra of Prop. 17,
Corollary 20, and Figs. 4 and 8.

**Proposition 10.** Every pairwise distance is at most V/2, and Jung's theorem gives radius
V/(2√3). A sharper classical fact is that any closed curve of length L lies in a disc of radius
L/4, centred at the midpoint of two points that split the curve in half. The bound is attained
by degenerate back-and-forth polygons. The conclusion (no blow-up) is unchanged.
