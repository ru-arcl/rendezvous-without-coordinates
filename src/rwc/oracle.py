"""Numerical oracle: the largest V' any configuration allows, by direct optimization.

Used to test the paper's sufficient conditions independently of its proofs. V' is computed from
velocities (``lyapunov.V_dot``), never from the theta/phi formula. Every agent with a target sees
it strictly inside the windshield: its heading is the line-of-sight angle minus a bearing
``phi * tanh(s)``. Needs numpy and scipy (``pip install rwc[oracle]``).
"""

import math

import numpy as np
from scipy.optimize import minimize


def _unpack(z, n, has_target, targets, phi, v_lo, v_hi):
    x, y, s = z[:n], z[n:2 * n], z[2 * n:3 * n]
    w = z[3 * n:4 * n]
    speeds = v_lo + (v_hi - v_lo) * 0.5 * (1.0 + np.tanh(w))
    psi = np.zeros(n)
    for i in range(n):
        if has_target[i]:
            j = targets[i]
            psi[i] = math.atan2(y[j] - y[i], x[j] - x[i]) - phi * math.tanh(s[i])
    return x, y, psi, speeds


def vdot_of(z, targets, phi, v_lo=1.0, v_hi=1.0):
    n = len(targets)
    has_target = [j >= 0 for j in targets]
    x, y, psi, speeds = _unpack(z, n, has_target, targets, phi, v_lo, v_hi)
    vx = np.where(has_target, speeds * np.cos(psi), 0.0)
    vy = np.where(has_target, speeds * np.sin(psi), 0.0)
    total = 0.0
    for i in range(n):
        j = targets[i]
        if j < 0:
            continue
        dx, dy = x[j] - x[i], y[j] - y[i]
        l = math.sqrt(dx * dx + dy * dy) + 1e-300
        total += ((vx[j] - vx[i]) * dx + (vy[j] - vy[i]) * dy) / l
    return total


def _pursuit_order(targets):
    """Position of each agent along a walk that follows edges backwards from the cycle."""
    n = len(targets)
    order, seen = [], set()
    for start in range(n):
        i = start
        while i >= 0 and i not in seen:
            seen.add(i)
            order.append(i)
            i = targets[i]
    rank = [0] * n
    for r, i in enumerate(order):
        rank[i] = -r     # walking forward along edges goes clockwise
    return rank


def sup_vdot(targets, phi, v_lo=1.0, v_hi=1.0, restarts=40, seed=0, maxiter=400):
    """Approximate sup of V' over positions, in-view bearings and speeds in [v_lo, v_hi].

    Returns (best value, best parameter vector). The sup is usually approached, not attained,
    because the windshield is open; ``phi * tanh(s)`` lets bearings tend to +-phi.
    """
    n = len(targets)
    rng = np.random.default_rng(seed)
    best, best_z = -np.inf, None
    f = lambda z: -vdot_of(z, targets, phi, v_lo, v_hi)  # noqa: E731
    for r_i in range(restarts):
        if r_i % 2:
            xy = rng.uniform(-1, 1, 2 * n)
        else:
            # structured start: agents on a jittered circle in pursuit order, so near-regular
            # polygons (the extremal cycles) are within reach of the local optimizer
            a = 2 * np.pi * np.array(_pursuit_order(targets)) / n
            a = a + rng.normal(0, 0.3 / n, n)
            xy = np.concatenate([np.cos(a), np.sin(a)]) * rng.uniform(0.8, 1.2, 2 * n)
        z0 = np.concatenate([xy, rng.uniform(-3, 3, n), rng.uniform(-3, 3, n)])
        r = minimize(f, z0, method="L-BFGS-B", options={"maxiter": maxiter})
        if -r.fun > best:
            best, best_z = -r.fun, r.x
    return best, best_z


def critical_phi(targets, lo=0.01, hi=math.pi / 2, tol=2e-3, v_lo=1.0, v_hi=1.0, restarts=30,
                 seed=0):
    """Largest phi (to within tol) for which the oracle finds no configuration with V' >= 0."""
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if sup_vdot(targets, mid, v_lo, v_hi, restarts=restarts, seed=seed)[0] < -1e-9:
            lo = mid
        else:
            hi = mid
    return lo
