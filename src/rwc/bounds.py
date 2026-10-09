"""Sufficient conditions from the paper, exactly as printed, and corrected versions.

Equation numbers refer to the journal version (IEEE TAC 57(2), 2012) unless marked CDC.
Corrections cite their note in docs/notes/.
"""

import math

PI = math.pi


def thm4_phi(n):
    """Theorem 4, eq. (11): unit-speed cyclic pursuit rendezvouses if 0 < phi <= this.

    Conservative for n = 3, 4 (the true threshold is pi/n), and the '<=' must be '<' for
    n = 2 and n >= 5 to get Corollary 9's uniform decrease; see docs/notes/01-windshield-bounds.md.
    """
    if n < 2:
        raise ValueError("n >= 2")
    if n == 2:
        return PI / 2
    if n in (3, 4):
        return math.acos((n - 1) / n)
    return PI / n


def thm4_phi_cdc(n):
    """CDC 2008 Theorem 4, eq. (10): 0 < phi < min{pi/n, acos((n-1)/n)}."""
    if n < 2:
        raise ValueError("n >= 2")
    return min(PI / n, math.acos((n - 1) / n))


def lemma11_phi(n):
    """Lemma 11: unit-speed intree pursuit has V' < 0 if phi < acos((n-2)/(n-1)) (tight)."""
    if n < 2:
        raise ValueError("n >= 2")
    if n == 2:
        return PI / 2
    return math.acos((n - 2) / (n - 1))


def thm12_phi(n):
    """Theorem 12, eq. (27): any connected single-target graph, unit speed, phi < min{pi/5, pi/n}.

    The pi/5 cap is not needed: numerically the threshold is pi/n for every n tested (2..12);
    see critical_phi.
    """
    if n < 2:
        raise ValueError("n >= 2")
    return min(PI / 5, PI / n)


def thm13_phi(n, v_min, v_max):
    """Theorem 13, eq. (32): bounded-speed cyclic pursuit rendezvouses if phi < this."""
    if not 0 < v_min <= v_max:
        raise ValueError("need 0 < v_min <= v_max")
    return math.acos((n * v_max - v_min * (1 - math.cos(PI / n))) / (n * v_max))


def critical_phi(n):
    """The exact windshield threshold found numerically for unit-speed pursuit: pi/n.

    For a cycle of n agents, and for any connected single-target graph with n agents, V' < 0 at
    every configuration iff phi <= pi/n, and with a uniform margin (sup V' < 0) iff phi < pi/n:
    at phi = pi/n the open windshield keeps V' < 0 but sup V' = 0. Not proved in the paper. Found with the optimizer in rwc.oracle for n = 2..12, with
    a turning-angle argument for cycles; see docs/notes/01-windshield-bounds.md.
    """
    if n < 2:
        raise ValueError("n >= 2")
    return PI / n


def intree_phi(n, v_min=1.0, v_max=1.0):
    """V' < 0 for every intree of n agents with speeds in [v_min, v_max] iff phi < this.

    Generalizes Lemma 11 (v_min = v_max gives acos((n-2)/(n-1))). Returns None when no windshield
    works, i.e. v_max/v_min >= (n-2)/(n-3); the paper only says no such result exists (p. 428).
    """
    if n < 2:
        raise ValueError("n >= 2")
    if n == 2:
        return PI / 2
    c = (n - 2) * v_max / (v_max + (n - 2) * v_min)
    return math.acos(c) if c < 1 else None


def sampled_phi(n, omega, dt):
    """Approximate threshold of the sampled law (43) with time step dt: pi/n + omega*dt/2.

    Below it agents end in a ring of radius about max(v/omega, v dt/(pi/n - phi + omega dt/2));
    above it the polygon grows. Empirical fit, see docs/notes/03-sampled-control.md.
    """
    return PI / n + 0.5 * omega * dt


def lemma14_phi():
    """Lemma 14: bounded-speed intree pursuit merges into the root if phi < pi/2."""
    return PI / 2


def regular_polygon_vdot(n, beta, v=1.0):
    """V' of a regular n-gon whose agents all see their target at bearing beta (eq. (44)).

    Zero at beta = pi/n (Lemma 3), positive above, negative below.
    """
    if n == 2:
        return -4.0 * v * math.cos(beta)
    return -n * v * (math.cos(beta) + math.cos((n - 2) * PI / n + beta))


def lemma7_f_bound(n, phi):
    """Lemma 7, eq. (22): upper bound on f = sum -cos(theta_i + phi_i) for simple n-gons."""
    return max(-n * math.cos(((n - 2) * PI + n * phi) / n), n - 1)


def prop10_radius(V0):
    """Proposition 10: all agents stay in a disc of radius V0 / (2 sqrt 3)."""
    return V0 / (2.0 * math.sqrt(3.0))


def closed_curve_radius(V0):
    """Sharper classical bound: a closed curve of length L lies in a disc of radius L/4."""
    return V0 / 4.0


def prop16_omega(v_max, rho):
    """Proposition 16, eq. (40), as printed: omega > v_max / rho.

    Insufficient in general: it ignores the pursuer's own motion; see
    docs/notes/02-turn-rate.md and ``tracking_omega``.
    """
    return v_max / rho


def prop16_omega_cdc(rho, v=1.0):
    """CDC Proposition 8: omega > 2/rho for unit speed (2 v / rho in general)."""
    return 2.0 * v / rho


def tracking_omega(v_max, rho, phi):
    """Corrected Proposition 16: omega > v_max (1 + sin(phi)) / rho, at most 2 v_max / rho.

    When the target is on the windshield edge at bearing phi and distance l > rho, the line of
    sight turns at up to (v_j + v_i sin(phi)) / l: the target's own motion plus the pursuer's.
    Turning faster than that keeps the target; see docs/notes/02-turn-rate.md.
    """
    return v_max * (1.0 + math.sin(phi)) / rho


def prop17_omega(v_max, rho):
    """Proposition 17, eq. (41): omega >= 3 v_max / rho for reassignment.

    Needs '>' at v_i = v_j = v_max, and assumes each agent's speed is constant over a turn;
    see docs/notes/02-turn-rate.md.
    """
    return 3.0 * v_max / rho


def cor20_rate(n):
    """Corollary 20, eq. (52): unit-speed direct pursuit has V' <= -delta; returns -delta."""
    return max(-1.0, -n * (1.0 + math.cos((n - 2) * PI / n)))
