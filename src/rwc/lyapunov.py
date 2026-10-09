"""The Lyapunov function V (eq. (6)) and its time derivative."""

import math

from .geometry import wrap


def edge_lengths(x, y, targets, alive=None):
    n = len(targets)
    alive = alive if alive is not None else [True] * n
    out = [None] * n
    for i in range(n):
        j = targets[i]
        if alive[i] and j >= 0:
            dx = x[j] - x[i]
            dy = y[j] - y[i]
            out[i] = math.sqrt(dx * dx + dy * dy)
    return out


def V(x, y, targets, alive=None):
    """Sum of distances between agents and their targets (eq. (6))."""
    return sum(l for l in edge_lengths(x, y, targets, alive) if l is not None)


def V_dot(x, y, psi, speeds, targets, moving=None):
    """dV/dt computed directly from the velocities: sum over edges of <p_j' - p_i', unit(p_j - p_i)>.

    ``moving[i]`` False means agent i is stationary (e.g. the root of an intree).
    """
    n = len(targets)
    moving = moving if moving is not None else [targets[i] >= 0 for i in range(n)]
    vx = [speeds[i] * math.cos(psi[i]) if moving[i] else 0.0 for i in range(n)]
    vy = [speeds[i] * math.sin(psi[i]) if moving[i] else 0.0 for i in range(n)]
    total = 0.0
    for i in range(n):
        j = targets[i]
        if j < 0:
            continue
        dx = x[j] - x[i]
        dy = y[j] - y[i]
        l = math.sqrt(dx * dx + dy * dy)
        total += ((vx[j] - vx[i]) * dx + (vy[j] - vy[i]) * dy) / l
    return total


def cycle_angles(x, y, psi, order):
    """theta_i and phi_i of Section IV-B for agents on a pursuit cycle ``order`` (i targets i+1).

    theta_i is the angle at vertex i measured from the ray to the next agent to the ray to the
    previous agent, on the side opposite to phi_i, so that cos(theta_i + phi_i) is the cosine of the
    angle between i's velocity and the ray back to the previous agent (sign convention of Fig. 4).
    phi_i is the bearing of i's target. Returns (thetas, phis) in [0, 2*pi) and [-pi, pi).
    """
    k = len(order)
    thetas, phis = [], []
    for m in range(k):
        i = order[m]
        nxt = order[(m + 1) % k]
        prv = order[(m - 1) % k]
        a_next = math.atan2(y[nxt] - y[i], x[nxt] - x[i])
        a_prev = math.atan2(y[prv] - y[i], x[prv] - x[i])
        ph = wrap(a_next - psi[i])
        if k == 2:
            th = 0.0
        elif ph >= 0:
            # target to the left: the heading is clockwise of the ray to the next agent, so theta
            # is measured counter-clockwise from that ray (the other side, as in Fig. 4)
            th = (a_prev - a_next) % (2 * math.pi)
        else:
            th = (a_next - a_prev) % (2 * math.pi)
        thetas.append(th)
        phis.append(ph)
    # With theta measured on the side away from the heading, theta and phi always start from
    # different sides of the ray, so phi_i = |bearing| >= 0 in the paper's sign convention.
    return thetas, [abs(p) for p in phis]


def V_dot_cycle_formula(thetas, phis, speeds=None):
    """Eq. (9): V' = sum_i -v_i (cos phi_i + cos(theta_i + phi_i))."""
    speeds = speeds if speeds is not None else [1.0] * len(thetas)
    return sum(-v * (math.cos(p) + math.cos(t + p)) for t, p, v in zip(thetas, phis, speeds))
