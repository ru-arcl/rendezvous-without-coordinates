# Design

This document is the contract that the Python package (`src/rwc/`) and the browser engine
(`docs/js/rwc-engine.js`) both implement. Parity fixtures in `tests/fixtures/parity/` enforce
that the two give the same numbers.

## Scope

We reproduce the system of our paper *Rendezvous Without Coordinates* (Yu, LaValle,
Liberzon; IEEE TAC 2012, CDC 2008):

* agents are Dubins cars, `p' = v (cos ψ, sin ψ)`, `ψ' = u ∈ {-ω, 0, +ω}`;
* each agent senses only which part of its windshield `(-φ, φ)` its assigned target is in;
* the control law is the three-level law of TAC eqs. (42)–(43), which needs no coordinates;
* an agent merges into its target when they are within the merging radius `ρ`.

There is no surviving source for the original Java applet. Our "original" reference is
therefore the algorithm as the paper states it, and the qualitative behavior in the
2019 screen capture of the applet. Where the paper's algorithm leaves a gap or contains an
error, the default mode fixes it and a `paper` option reproduces it exactly
(see `docs/notes/`).

## Frames and signs

Standard math frame: x right, y up, ψ counter-clockwise from +x, `u > 0` turns left.
The bearing of target `j` seen from `i` is
`β = wrap(atan2(y_j - y_i, x_j - x_i) - ψ_i)` with `wrap(a) = a - 2π·floor((a + π)/2π) ∈ [-π, π)`.
`β > 0` means the target is to the left.

Observation (TAC (42), with our sign convention):

| obs | meaning |
| --- | --- |
| `+1` | target in the left half of the windshield, `0 ≤ β < φ` |
| `-1` | target in the right half, `-φ < β < 0` |
| `0`  | target out of view, `|β| ≥ φ` |

TAC (42) labels the left sector `-1`; we use `+1` so that the control is simply
`u = ω · (side the target left from)`. The rule is the same.

## Controllers

`controller="eq43"` (default) — the four-state automaton of TAC (43):

| obs[k] | obs[k-1] | u[k] |
| --- | --- | --- |
| 0 | +1 | +ω (target left across the left edge: turn left) |
| 0 | -1 | -ω |
| ±1 | 0 | 0 (target is back: go straight) |
| otherwise | | u[k-1] |

with `u[0] = 0` and `obs[-1] = obs[0]`.

`controller="eq3"` — the instantaneous sensor of TAC eq. (2)+(3) extended to the full
circle as in Appendix A, Fig. 7(b): `u = +ω` if `β ≥ φ - ε`, `-ω` if `β ≤ -(φ - ε)`,
otherwise 0.

**Unseen targets.** If a target is out of view at `t = 0`, (43) gives `u = 0` forever (the
paper assumes every target starts in view). With `search_unseen=True` (default) an agent that
has never seen its target turns at `+ω` until it does. `paper=True` turns this off.

**Sensor noise** (Section VIII: "randomly wrong feedback 20% of the time"): with probability
`noise`, the reading is replaced by one of the two other values, chosen uniformly. For `eq43`
the reading is the observation; for `eq3` it is y ∈ {+1, 0, −1} of eq. (2), and `u = ω y`.
Each reading draws one number for the trigger and, if triggered, one more (`< 0.5`) to pick.

## Time stepping

One step of length `dt`, for all agents in index order:

1. sense (positions at the start of the step):
   * an agent with a target updates its controller;
   * with `reassign` on and more than one agent alive, an agent without a target first merges
     into the lowest-index alive agent within `ρ` (Section VI-B); if there is none, it takes as
     target the agent in its windshield with the smallest `|β|` (ties: lowest index), goes
     straight and logs an `assign` event; if there is none either, it turns at `+ω`;
   * otherwise an agent without a target stays put;
2. integrate exactly along the Dubins arc for constant `u`:
   `u = 0`: `x += v dt cos ψ`, `y += v dt sin ψ`;
   `u ≠ 0`: `x += (v/u)(sin(ψ + u dt) - sin ψ)`, `y += (v/u)(cos ψ - cos(ψ + u dt))`, `ψ = wrap(ψ + u dt)`;
   agents without a target do not move (paper's base assumption) unless they are searching
   under `reassign`;
3. merge, in index order `i = 0..n-1`: if `i` is alive, has target `j`, and
   `|p_i - p_j| ≤ ρ`, then `i` merges into `j` (eq. (4)): `i` is removed, its distance is frozen,
   every edge `k → i` becomes `k → j`, and an edge `j → i` becomes "no target" (mutual merge:
   `j` stops). Later indices see the updated graph.
4. with `reassign` on: every alive agent still without a target (a searching agent, which
   cannot turn in place) merges into the lowest-index alive agent now within `ρ`.

All arithmetic uses `sqrt`, `atan2`, `sin`, `cos`, `floor` only (no `hypot`), in the same
order in both engines.

## Lyapunov function

`V = Σ_{edges} ℓ_ij + Σ_{merged} (frozen distance at merge)` (Section IV-A: "the distance
between the two merged agents is frozen"). This keeps the merged edge continuous, but `V` can
still jump at a merge: each redirected edge `k → i` becomes `k → j` and changes length by up to
`ρ`, and a mutual merge drops the edge `j → i`. There are at most `n − 1` merges.
`V_active` omits the frozen part. `V̇` is computed directly from velocities:
`V̇ = Σ_edges ⟨p_j' − p_i', (p_j − p_i)/ℓ_ij⟩`, and checked against eq. (9).

## Random numbers

`mulberry32` with a 32-bit seed, identical in both engines; `randint(m) = floor(m · random())`.
Scenario generation uses one stream, in this order:

1. the graph. `cycle_branches`: `k = 2 + randint(n − 2)` unless given; then a Fisher–Yates
   shuffle of `perm = [0..n−1]` (for `i = n−1` down to 1, swap `perm[i]` and
   `perm[randint(i+1)]`); `perm[m]` targets `perm[m−1]` for `0 < m < k`, `perm[0]` targets
   `perm[k−1]`; then for `m = k..n−1`, `perm[m]` targets `perm[randint(m)]`. `intree`: the same
   shuffle, then for `m = 1..n−1`, `perm[m]` targets `perm[randint(m)]`. Other kinds draw nothing.
2. `x` then `y` for each agent in turn (`regular` draws nothing more);
3. one heading draw per agent, used even when the agent has no target.

Sensor noise uses a separate stream seeded with `seed ^ 0x9E3779B9`.

## Scenarios

* `cycle`: agent `i` targets `i - 1`, agent 0 targets `n - 1` (Section VII).
* `cycle_branches`: a random cycle of `k < n` agents, the rest attached at random (Section VII).
* `intree`: random tree pointing to a root that does not move.
* `fig6e`: TAC Fig. 6(e): cycle 0 → 4 → 3 → 5 → 0, branches 6 → 2 → 0, 7 → 3, 1 → 5.
* `none`: no targets, for reassignment (Proposition 17).
* `regular`: regular n-gon, every agent heading along the circumcircle tangent (Lemma 3, Fig. 8).

Positions are uniform in a `W × H` box. With `heading="in_view"` (default) each agent's heading
is its target's direction plus a uniform offset in `(-0.9 φ, 0.9 φ)`, so the paper's initial
condition holds; `heading="random"` draws uniformly in `[-π, π)`.

## Input checks

Both engines reject: per-agent `omega`/`speeds` of the wrong length, non-positive `omega` or
speeds, `φ ∉ (0, π]`, `dt ≤ 0`, `ρ < 0`, an unknown controller, and `eq3` without `0 < ε < φ`.

## Bounds

`rwc.bounds` returns the paper's sufficient conditions exactly as printed, plus corrected
versions where the printed one is wrong (each cites its note in `docs/notes/`).
