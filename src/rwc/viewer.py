"""Live matplotlib viewer: the mouse is a leader that every agent ends up following.

    python -m rwc.viewer [--n 12] [--phi 0.3] [--omega 4] [--chain]

The leader is the root of an intree (or the end of a chain with --chain); it sits wherever the
mouse is. Every other agent only knows which side of its windshield its own target left from.
Keys: up/down change phi, c switches to a closed cycle that ignores the mouse, r resets,
space pauses.
"""

import argparse
import math

from .graph import NONE
from .rng import Mulberry32
from .sim import Sim


def make_leader_sim(n=12, phi=0.3, omega=4.0, dt=0.05, seed=1, chain=False, cyclic=False,
                    size=20.0):
    """n followers plus a leader (index n) whose position the caller sets each frame."""
    rng = Mulberry32(seed)
    m = n + 1
    x = [size * rng.random() for _ in range(m)]
    y = [size * rng.random() for _ in range(m)]
    if cyclic:
        targets = [(i - 1) % n for i in range(n)] + [NONE]
    elif chain:
        targets = [i + 1 for i in range(n)] + [NONE]
    else:
        targets = [n if i == 0 else rng.randint(i) if i > 0 and rng.random() < 0.5 else n
                   for i in range(n)] + [NONE]
    psi = []
    for i in range(m):
        j = targets[i]
        if j == NONE:
            psi.append(0.0)
        else:
            psi.append(math.atan2(y[j] - y[i], x[j] - x[i]) + (2 * rng.random() - 1) * 0.8 * phi)
    return Sim(x, y, psi, targets, phi=phi, omega=omega, dt=dt, rho=1e-9)


class Viewer:
    """State and drawing for the live viewer (kept separate from the event loop for testing)."""

    def __init__(self, n=12, phi=0.3, omega=4.0, dt=0.05, chain=False, seed=1):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Wedge

        self.plt, self.Wedge = plt, Wedge
        self.n, self.phi, self.omega, self.dt, self.chain, self.seed = n, phi, omega, dt, chain, seed
        self.cyclic = False
        self.paused = False
        self.mouse = (10.0, 10.0)
        self.fig, (self.ax, self.axv) = plt.subplots(
            1, 2, figsize=(11, 5.5), gridspec_kw={"width_ratios": [1.6, 1]})
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_move)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.reset()

    def reset(self):
        self.sim = make_leader_sim(self.n, self.phi, self.omega, self.dt, self.seed, self.chain,
                                   self.cyclic)
        self.ts, self.vs = [], []
        self.trails = [[] for _ in range(self.sim.n)]

    def _on_move(self, event):
        if event.inaxes is self.ax and event.xdata is not None:
            self.mouse = (event.xdata, event.ydata)

    def _on_key(self, event):
        if event.key == "up":
            self.phi = min(self.phi + 0.02, 1.5)
        elif event.key == "down":
            self.phi = max(self.phi - 0.02, 0.02)
        elif event.key == "c":
            self.cyclic = not self.cyclic
            self.reset()
        elif event.key == "r":
            self.seed += 1
            self.reset()
        elif event.key == " ":
            self.paused = not self.paused
        self.sim.phi = self.phi

    def update(self, steps=2):
        s = self.sim
        lead = s.n - 1
        if not self.cyclic:
            s.x[lead], s.y[lead] = self.mouse
        for _ in range(steps if not self.paused else 0):
            s.step()
            for i in range(s.n):
                tr = self.trails[i]
                tr.append((s.x[i], s.y[i]))
                if len(tr) > 120:
                    del tr[0]
            self.ts.append(s.t)
            self.vs.append(s.V())
        self.draw()

    def draw(self):
        s, ax = self.sim, self.ax
        ax.clear()
        ax.set_aspect("equal")
        lead = s.n - 1
        xs = [s.x[i] for i in range(s.n) if s.alive[i]]
        ys = [s.y[i] for i in range(s.n) if s.alive[i]]
        cx, cy = 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))
        half = max(max(xs) - min(xs), max(ys) - min(ys), 16.0) * 0.6
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        r = 0.06 * half
        for i in range(s.n):
            if not s.alive[i]:
                continue
            if self.trails[i]:
                tx, ty = zip(*self.trails[i])
                ax.plot(tx, ty, lw=0.8, alpha=0.4, color="C%d" % (i % 10))
            if i == lead and not self.cyclic:
                ax.plot(s.x[i], s.y[i], "o", ms=10, mfc="none", mec="k", mew=2)
                continue
            deg = math.degrees(s.psi[i])
            ph = math.degrees(s.phi)
            ax.add_patch(self.Wedge((s.x[i], s.y[i]), 3 * r, deg - ph, deg + ph, alpha=0.15,
                                    color="C1"))
            ax.arrow(s.x[i], s.y[i], r * math.cos(s.psi[i]), r * math.sin(s.psi[i]),
                     width=0.25 * r, head_width=0.8 * r, head_length=0.8 * r,
                     length_includes_head=True, color="C%d" % (i % 10))
        mode = "cycle (mouse ignored)" if self.cyclic else ("chain" if self.chain else "intree")
        ax.set_title("%s  phi = %.2f  (pi/n = %.2f)   up/down: phi  c: cycle  r: reset"
                     % (mode, s.phi, math.pi / max(self.n, 2)), fontsize=9)
        self.axv.clear()
        self.axv.plot(self.ts[-3000:], self.vs[-3000:], color="k")
        self.axv.set_xlabel("t")
        self.axv.set_title("V = sum of distances to targets", fontsize=9)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--n", type=int, default=12)
    p.add_argument("--phi", type=float, default=0.3)
    p.add_argument("--omega", type=float, default=4.0)
    p.add_argument("--dt", type=float, default=0.05)
    p.add_argument("--chain", action="store_true", help="followers form a chain to the mouse")
    p.add_argument("--seed", type=int, default=1)
    a = p.parse_args(argv)
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    v = Viewer(a.n, a.phi, a.omega, a.dt, a.chain, a.seed)
    v._anim = FuncAnimation(v.fig, lambda _: v.update(), interval=30, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
