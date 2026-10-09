"""Discrete-time simulation of Dubins-car agents rendezvousing without coordinates.

The time stepping, controller and merge order are specified in docs/DESIGN.md; the JavaScript
engine (docs/js/rwc-engine.js) implements the same steps and is checked against parity fixtures.
"""

import math
import numbers

from .geometry import bearing, dist, dubins_step
from .graph import NONE, merge
from .rng import Mulberry32

NOISE_SALT = 0x9E3779B9


class Sim:
    """A system of n Dubins-car agents.

    Parameters
    ----------
    x, y, psi : initial positions and headings.
    targets : targets[i] is agent i's target, or -1.
    phi : half windshield span; the windshield is the open sector (-phi, phi).
    omega : turn rate (scalar or per agent).
    speeds : forward speed (scalar or per agent), default 1.
    dt : time step.
    rho : merging radius.
    controller : "eq43" (TAC eq. (43), default) or "eq3" (instantaneous sensor, eq. (2)-(3)).
    eps : epsilon_phi for controller "eq3".
    search_unseen : an agent that has never seen its target turns at +omega until it does.
    paper : reproduce the paper's algorithm exactly (turns off search_unseen).
    noise : probability that an observation is replaced by a wrong one.
    reassign : agents without targets look for new ones (Proposition 17).
    seed : seed of the sensor-noise stream.
    """

    def __init__(self, x, y, psi, targets, phi, omega=2.0, speeds=1.0, dt=0.1, rho=1e-3,
                 controller="eq43", eps=0.02, search_unseen=True, paper=False, noise=0.0,
                 reassign=False, seed=0):
        n = len(x)
        if not (len(y) == len(psi) == len(targets) == n):
            raise ValueError("x, y, psi and targets must have the same length")
        if controller not in ("eq43", "eq3"):
            raise ValueError("controller must be 'eq43' or 'eq3'")
        if not 0.0 < phi <= math.pi:
            raise ValueError("phi must be in (0, pi]")
        for i, j in enumerate(targets):
            if j != NONE and not (0 <= j < n and j != i):
                raise ValueError("invalid target %r for agent %d" % (j, i))
        self.n = n
        self.x = [float(a) for a in x]
        self.y = [float(a) for a in y]
        self.psi = [float(a) for a in psi]
        self.targets = [int(j) for j in targets]
        self.phi = float(phi)
        self.omega = [float(omega)] * n if _is_scalar(omega) else [float(w) for w in omega]
        self.speeds = [float(speeds)] * n if _is_scalar(speeds) else [float(v) for v in speeds]
        if len(self.omega) != n or len(self.speeds) != n:
            raise ValueError("omega and speeds must be scalars or have one entry per agent")
        if min(self.omega) <= 0 or min(self.speeds) <= 0:
            raise ValueError("omega and speeds must be positive")
        if controller == "eq3" and not 0.0 < eps < phi:
            raise ValueError("controller 'eq3' needs 0 < eps < phi")
        if not (rho >= 0 and dt > 0):
            raise ValueError("need rho >= 0 and dt > 0")
        self.dt = float(dt)
        self.rho = float(rho)
        self.controller = controller
        self.eps = float(eps)
        self.paper = bool(paper)
        self.search_unseen = bool(search_unseen) and not self.paper
        self.noise = float(noise)
        self.reassign = bool(reassign)
        self.rng = Mulberry32((int(seed) ^ NOISE_SALT) & 0xFFFFFFFF)

        self.alive = [True] * n
        self.u = [0.0] * n
        self.frozen = [0.0] * n          # distance frozen at merge time (Section IV-A)
        self.merged_into = [NONE] * n
        self.t = 0.0
        self.steps = 0
        self.events = []                 # (step, kind, i, j)
        self.obs_prev = [0] * n
        self.seen = [False] * n
        for i in range(n):
            if self.targets[i] != NONE:
                o = self._observe_clean(i)
                self.obs_prev[i] = o
                self.seen[i] = o != 0

    # ----------------------------------------------------------------- sensing
    def bearing_to(self, i, j):
        return bearing(self.x[i], self.y[i], self.psi[i], self.x[j], self.y[j])

    def _observe_clean(self, i):
        b = self.bearing_to(i, self.targets[i])
        if -self.phi < b < self.phi:
            return 1 if b >= 0.0 else -1
        return 0

    def _corrupt(self, o):
        """With probability ``noise``, replace o in {-1, 0, 1} by one of the other two values."""
        if self.noise > 0.0 and self.rng.random() < self.noise:
            others = [v for v in (-1, 0, 1) if v != o]
            o = others[0] if self.rng.random() < 0.5 else others[1]
        return o

    def _observe(self, i):
        return self._corrupt(self._observe_clean(i))

    def _control(self, i):
        w = self.omega[i]
        if self.controller == "eq3":
            b = self.bearing_to(i, self.targets[i])
            edge = self.phi - self.eps
            y = 1 if b >= edge else (-1 if b <= -edge else 0)
            return w * self._corrupt(y)
        o = self._observe(i)
        prev = self.obs_prev[i]
        u = self.u[i]
        if o == 0 and prev == 1:
            u = w
        elif o == 0 and prev == -1:
            u = -w
        elif o != 0 and prev == 0:
            u = 0.0
        if o != 0:
            self.seen[i] = True
        elif not self.seen[i] and self.search_unseen:
            u = w
        self.obs_prev[i] = o
        return u

    def _nearby(self, i):
        """Lowest-index alive agent within rho of agent i, or -1."""
        for k in range(self.n):
            if k != i and self.alive[k]:
                if dist(self.x[i], self.y[i], self.x[k], self.y[k]) <= self.rho:
                    return k
        return NONE

    def _reassign_target(self, i):
        best, best_b = NONE, None
        for k in range(self.n):
            if k == i or not self.alive[k]:
                continue
            b = abs(self.bearing_to(i, k))
            if b < self.phi and (best_b is None or b < best_b):
                best, best_b = k, b
        return best

    # ------------------------------------------------------------------ stepping
    def n_alive(self):
        return sum(self.alive)

    def moving(self, i):
        if not self.alive[i]:
            return False
        if self.targets[i] != NONE:
            return True
        return self.reassign and self.n_alive() > 1

    def step(self):
        n = self.n
        # 1. sense and decide (all agents see the positions at the start of the step)
        for i in range(n):
            if not self.alive[i]:
                continue
            if self.targets[i] != NONE:
                self.u[i] = self._control(i)
            elif self.reassign and self.n_alive() > 1:
                near = self._nearby(i)
                if near != NONE:
                    self._merge(i, near, dist(self.x[i], self.y[i], self.x[near], self.y[near]))
                    continue
                k = self._reassign_target(i)
                if k != NONE:
                    self.targets[i] = k
                    self.obs_prev[i] = self._observe_clean(i)
                    self.seen[i] = True
                    self.u[i] = 0.0
                    self.events.append((self.steps, "assign", i, k))
                else:
                    self.u[i] = self.omega[i]
            else:
                self.u[i] = 0.0
        # 2. move
        moving = [self.moving(i) for i in range(n)]
        for i in range(n):
            if moving[i]:
                self.x[i], self.y[i], self.psi[i] = dubins_step(
                    self.x[i], self.y[i], self.psi[i], self.speeds[i], self.u[i], self.dt)
        # 3. merge along edges, in index order (eq. (4))
        for i in range(n):
            j = self.targets[i]
            if self.alive[i] and j != NONE:
                d = dist(self.x[i], self.y[i], self.x[j], self.y[j])
                if d <= self.rho:
                    self._merge(i, j, d)
        # 4. targetless agents that are still searching merge into nearby agents (Sec. VI-B)
        if self.reassign:
            for i in range(n):
                if self.alive[i] and self.targets[i] == NONE:
                    k = self._nearby(i)
                    if k != NONE:
                        self._merge(i, k, dist(self.x[i], self.y[i], self.x[k], self.y[k]))
        self.steps += 1
        self.t = self.steps * self.dt

    def _merge(self, i, j, d):
        merge(self.targets, self.alive, i, j)
        self.frozen[i] = d
        self.merged_into[i] = j
        self.u[i] = 0.0
        self.events.append((self.steps, "merge", i, j))

    def run(self, steps, record_every=1, stop_when_done=True):
        """Advance ``steps`` steps; return the recorded history (see ``snapshot``)."""
        hist = [self.snapshot()]
        for s in range(1, steps + 1):
            self.step()
            if s % record_every == 0:
                hist.append(self.snapshot())
            if stop_when_done and self.done():
                if s % record_every:
                    hist.append(self.snapshot())
                break
        return hist

    def done(self):
        """True when nothing can move any more (rendezvous, or every agent stopped)."""
        return not any(self.moving(i) for i in range(self.n))

    # -------------------------------------------------------------- observables
    def edge_lengths(self):
        """l_i for every agent: distance to target, frozen distance if merged, else None."""
        out = []
        for i in range(self.n):
            j = self.targets[i]
            if not self.alive[i]:
                out.append(self.frozen[i])
            elif j != NONE:
                out.append(dist(self.x[i], self.y[i], self.x[j], self.y[j]))
            else:
                out.append(None)
        return out

    def V(self):
        """Lyapunov function, with each merged edge's length frozen at the moment of merging.

        V is continuous between merges; at a merge, redirected edges can change by up to rho and
        a mutual merge drops one edge (docs/DESIGN.md).
        """
        return sum(l for l in self.edge_lengths() if l is not None)

    def V_active(self):
        return sum(l for i, l in enumerate(self.edge_lengths()) if l is not None and self.alive[i])

    def representative(self, i):
        while self.merged_into[i] != NONE:
            i = self.merged_into[i]
        return i

    def snapshot(self):
        return {
            "t": self.t,
            "x": list(self.x), "y": list(self.y), "psi": list(self.psi),
            "u": list(self.u), "targets": list(self.targets), "alive": list(self.alive),
            "V": self.V(), "V_active": self.V_active(), "l": self.edge_lengths(),
        }


def _is_scalar(a):
    return isinstance(a, numbers.Real) and not isinstance(a, bool)
