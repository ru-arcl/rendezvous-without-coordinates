"""Seeded initial conditions, bit-identical to the JavaScript engine.

Draw order (docs/DESIGN.md): graph, then x and y of every agent, then one heading draw per agent.
"""

import math

from .geometry import wrap
from .graph import NONE, cycle
from .rng import Mulberry32
from .sim import Sim

KINDS = ("cycle", "cycle_branches", "intree", "fig6e", "none", "regular")

# TAC Fig. 6(e): agents 0, 4, 3, 5 on a cycle; branches 6 -> 2 -> 0, 7 -> 3, 1 -> 5.
FIG6E_TARGETS = [4, 5, 0, 5, 3, 0, 2, 3]


def _shuffle(rng, n):
    perm = list(range(n))
    for i in range(n - 1, 0, -1):
        j = rng.randint(i + 1)
        perm[i], perm[j] = perm[j], perm[i]
    return perm


def make_graph(kind, n, rng, k=None):
    if kind == "cycle" or kind == "regular":
        return cycle(n)
    if kind == "fig6e":
        if n != 8:
            raise ValueError("fig6e has 8 agents")
        return list(FIG6E_TARGETS)
    if kind == "none":
        return [NONE] * n
    if kind == "cycle_branches":
        if n < 3:
            raise ValueError("cycle_branches needs n >= 3")
        if k is None:
            k = 2 + rng.randint(n - 2)          # 2 <= k <= n-1
        if not 2 <= k < n:
            raise ValueError("cycle length k must satisfy 2 <= k < n")
        perm = _shuffle(rng, n)
        targets = [NONE] * n
        for m in range(k):
            targets[perm[m]] = perm[m - 1] if m > 0 else perm[k - 1]
        for m in range(k, n):
            targets[perm[m]] = perm[rng.randint(m)]
        return targets
    if kind == "intree":
        perm = _shuffle(rng, n)
        targets = [NONE] * n
        for m in range(1, n):
            targets[perm[m]] = perm[rng.randint(m)]
        return targets
    raise ValueError("unknown scenario kind %r; expected one of %s" % (kind, KINDS))


def initial_state(kind, n, seed, phi, width=20.0, height=20.0, heading="in_view", k=None,
                  radius=None, beta=None):
    """Return (x, y, psi, targets) for a scenario.

    heading="in_view": heading = target direction + uniform offset in (-0.9 phi, 0.9 phi), so every
    target starts in the windshield (the paper's initial condition). heading="random": uniform.
    kind="regular": regular n-gon of the given radius, every agent's target at bearing -beta
    (default beta = pi/n, i.e. moving along the circumcircle tangent; Lemma 3 and Fig. 8).
    """
    rng = Mulberry32(seed)
    targets = make_graph(kind, n, rng, k)
    if kind == "regular":
        r = radius if radius is not None else 0.4 * min(width, height)
        b = beta if beta is not None else math.pi / n
        x, y, psi = [], [], []
        for i in range(n):
            a = 2.0 * math.pi * i / n
            x.append(0.5 * width + r * math.cos(a))
            y.append(0.5 * height + r * math.sin(a))
            chord = a - math.pi / n - 0.5 * math.pi       # direction to agent i-1
            psi.append(wrap(chord + b))
        return x, y, psi, targets
    x, y = [], []
    for i in range(n):
        x.append(width * rng.random())
        y.append(height * rng.random())
    psi = []
    for i in range(n):
        r = rng.random()
        j = targets[i]
        if heading == "in_view" and j != NONE:
            to = math.atan2(y[j] - y[i], x[j] - x[i])
            psi.append(wrap(to + (2.0 * r - 1.0) * 0.9 * phi))
        elif heading in ("in_view", "random"):
            psi.append(wrap(-math.pi + 2.0 * math.pi * r))
        else:
            raise ValueError("heading must be 'in_view' or 'random'")
    return x, y, psi, targets


def make_sim(kind="cycle", n=8, seed=1, phi=0.35, omega=2.0, dt=0.1, rho=1e-3, width=20.0,
             height=20.0, heading="in_view", k=None, radius=None, beta=None, **kwargs):
    """Build a Sim for a named scenario; extra keyword arguments go to Sim."""
    x, y, psi, targets = initial_state(kind, n, seed, phi, width, height, heading, k, radius, beta)
    return Sim(x, y, psi, targets, phi=phi, omega=omega, dt=dt, rho=rho, seed=seed, **kwargs)
