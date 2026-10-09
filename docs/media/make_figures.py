"""Regenerate the README figures in this folder from real simulation runs.

    python docs/media/make_figures.py          (needs matplotlib; numpy/scipy for thresholds.png)

model.png      : the vehicle (Dubins car), the three-valued windshield sensor and the control law
behavior.png   : the same 8 agents with phi below / above pi/n (TAC Fig. 6 setup), and V over time
thresholds.png : windshield bounds vs n: the paper's guarantee, pi/n, and the optimizer's answer
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrow, Wedge  # noqa: E402

from rwc import bounds, make_sim  # noqa: E402

INK = "#1d1f24"
MUTED = "#6b7080"
ACCENT = "#d9480f"
BLUE = "#1c7ed6"
GREEN = "#2b8a3e"
PI = math.pi


def agent_colors(n):
    cmap = plt.get_cmap("hsv")
    return [cmap((i / n + 0.05) % 1.0) for i in range(n)]


def draw_agent(ax, x, y, psi, size, color):
    ax.add_patch(FancyArrow(x - 0.5 * size * math.cos(psi), y - 0.5 * size * math.sin(psi),
                            size * math.cos(psi), size * math.sin(psi), width=0.28 * size,
                            head_width=0.75 * size, head_length=0.6 * size,
                            length_includes_head=True, color=color, ec=INK, lw=0.5, zorder=5))


def panel_car(ax):
    """The Dubins car: forward at constant speed, straight or turning at a fixed rate."""
    ax.set_aspect("equal")
    ax.axis("off")
    r = 1.6                                     # turning radius v / omega
    L = 2.6
    ax.plot([0, L], [0, 0], color=INK, lw=2)
    ax.text(L + 0.15, 0, "u = 0", va="center", fontsize=14, color=INK)
    for sgn, label in ((1, "u = +ω\n(left)"), (-1, "u = −ω\n(right)")):
        ts = [k / 40 * 1.25 for k in range(41)]
        xs = [r * math.sin(t) for t in ts]
        ys = [sgn * r * (1 - math.cos(t)) for t in ts]
        ax.plot(xs, ys, color=INK, lw=2)
        ax.text(xs[-1] + 0.2, ys[-1], label, va="center", fontsize=14, color=INK)
    draw_agent(ax, -0.15, 0, 0.0, 0.8, "#f08c00")
    ax.text(-0.2, -1.95, "speed v, turn rate ω", ha="center", fontsize=13, color=MUTED)
    ax.set_xlim(-1.2, 4.6)
    ax.set_ylim(-2.3, 2.6)
    ax.set_title("Vehicle: a Dubins car", fontsize=15, loc="left")


def panel_sensor(ax):
    """The whole sensor: which half of the windshield the target is in, or out of view."""
    phi = 0.5
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Wedge((0, 0), 3.2, 0, math.degrees(phi), color=BLUE, alpha=0.2))
    ax.add_patch(Wedge((0, 0), 3.2, -math.degrees(phi), 0, color=ACCENT, alpha=0.2))
    for a in (phi, -phi):
        ax.plot([0, 3.4 * math.cos(a)], [0, 3.4 * math.sin(a)], color=MUTED, lw=1.2)
    draw_agent(ax, 0, 0, 0.0, 0.9, "#f08c00")
    ax.text(2.35, 0.58, "+1\nleft half", color=BLUE, ha="center", va="center", fontsize=14)
    ax.text(2.35, -0.58, "−1\nright half", color=ACCENT, ha="center", va="center", fontsize=14)
    ax.text(-1.4, 1.3, "0  out of view", color=MUTED, ha="center", fontsize=14)
    ax.annotate("", xy=(3.0 * math.cos(phi), 3.0 * math.sin(phi)), xytext=(3.0, 0),
                arrowprops=dict(arrowstyle="<->", color=INK, lw=1,
                                connectionstyle="arc3,rad=0.25"))
    ax.text(3.35, 0.75, "φ", fontsize=16)
    ax.text(0.8, -1.55, "lost on the left → turn left (+ω)\nlost on the right → turn right (−ω)\n"
            "back in view → drive straight", ha="center", va="top", fontsize=13, color=INK,
            linespacing=1.5)
    ax.set_xlim(-2.4, 4.1)
    ax.set_ylim(-3.2, 2.2)
    ax.set_title("Sensor and control law", fontsize=15, loc="left")


def run(phi, seed, steps):
    sim = make_sim("cycle", n=8, seed=seed, phi=phi, omega=2.0, dt=0.1, rho=1e-6)
    hist = sim.run(steps)
    return sim, hist


def panel_traj(ax, hist, title, n=8, keep=None):
    cols = agent_colors(n)
    ax.set_aspect("equal")
    hist = hist if keep is None else hist[:keep]
    for i in range(n):
        xs = [h["x"][i] for h in hist]
        ys = [h["y"][i] for h in hist]
        ax.plot(xs, ys, color=cols[i], lw=1.3, alpha=0.85)
        ax.plot(xs[0], ys[0], "o", color=cols[i], ms=4)
    last = hist[-1]
    xs, ys = last["x"], last["y"]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    allx = [x for h in hist for x in h["x"]]
    ally = [y for h in hist for y in h["y"]]
    size = 0.045 * max(max(allx) - min(allx), max(ally) - min(ally))
    for i in range(n):
        draw_agent(ax, xs[i], ys[i], last["psi"][i], size, cols[i])
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#d0d0d6")
    ax.set_title(title, fontsize=15, loc="left")
    return span


def panel_V(ax, h_in, h_out, phi_in, phi_out):
    t1 = [h["t"] for h in h_in]
    t2 = [h["t"] for h in h_out]
    ax.plot(t1, [h["V"] for h in h_in], color=BLUE, lw=2, label="φ = %.2f < π/8" % phi_in)
    ax.plot(t2, [h["V"] for h in h_out], color=ACCENT, lw=2, label="φ = %.2f > π/8" % phi_out)
    ax.set_xlabel("time", fontsize=14)
    ax.set_ylabel("V = Σ distance to target", fontsize=14)
    ax.tick_params(labelsize=12)
    ax.legend(frameon=False, fontsize=13)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_title("(c) The Lyapunov function V", fontsize=15, loc="left")


def model():
    fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.3))
    panel_car(axs[0])
    panel_sensor(axs[1])
    fig.tight_layout()
    out = os.path.join(HERE, "model.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print("wrote", os.path.relpath(out))


def behavior():
    phi_in, phi_out = 0.35, 0.6
    steps = 1500
    _, h_in = run(phi_in, 1, steps)
    _, h_out = run(phi_out, 1, steps)
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.9), gridspec_kw={"width_ratios": [1, 1, 1.25]})
    panel_traj(axs[0], h_in, "(a) φ = 0.35 < π/8: they close in", keep=401)
    panel_traj(axs[1], h_out, "(b) φ = 0.60 > π/8: a growing octagon")
    panel_V(axs[2], h_in, h_out, phi_in, phi_out)
    fig.tight_layout()
    out = os.path.join(HERE, "behavior.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print("wrote", os.path.relpath(out))


def thresholds():
    from rwc.graph import cycle
    from rwc.oracle import critical_phi

    ns = list(range(2, 11))
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    ax.plot(ns, [PI / n for n in ns], "-", color=BLUE, lw=2, label="π/n (regular polygon, Lemma 3)")
    ax.plot(ns, [bounds.thm4_phi(n) for n in ns], "s--", color=ACCENT, ms=5,
            label="paper's guarantee, eq. (11)")
    ax.plot(ns, [bounds.thm12_phi(n) for n in ns], "^:", color=GREEN, ms=5,
            label="any connected graph, eq. (27)")
    on = [3, 4, 5, 6]
    ax.plot(on, [critical_phi(cycle(n), restarts=16) for n in on], "o", color=INK, ms=7,
            mfc="none", mew=1.5, label="largest φ with V' < 0 (optimizer)")
    ax.set_xlabel("number of agents n")
    ax.set_ylabel("windshield half-angle φ")
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, fontsize=8.5)
    fig.tight_layout()
    out = os.path.join(HERE, "thresholds.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print("wrote", os.path.relpath(out))


if __name__ == "__main__":
    model()
    behavior()
    if "--no-oracle" not in sys.argv:
        thresholds()
