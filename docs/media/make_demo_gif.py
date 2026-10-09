"""Regenerate docs/media/demo.gif: three demo presets side by side, agents on top, V below.

    python docs/media/make_demo_gif.py          (needs matplotlib and ffmpeg on PATH)

Columns are the demo's presets Fig. 6(a), Fig. 6(c) and Fig. 6(e). Every frame is drawn from
the simulator's state; each column advances its own number of steps per frame (the clock in
the corner is simulation time) and holds its last frame once the run is over.
"""

import colorsys
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrow  # noqa: E402

from rwc import make_sim  # noqa: E402

INK = "#1d1f24"
MUTED = "#6b7080"
GRID = "#d0d0d6"
OUT = os.path.join(HERE, "demo.gif")
FRAMES = 72          # animated frames; the last one is then held
HOLD = 18
FPS = 12
TRAIL = 150          # trail length in simulation steps

# (title, make_sim kwargs, steps per frame, curve colour), as the presets in docs/js/demo.js
CASES = [
    ("(a) φ = 0.35 < π/8: they close in",
     dict(n=8, phi=0.35, rho=0.001, seed=1), 6, "#1c7ed6"),
    ("(b) φ = 0.60 > π/8: a growing octagon",
     dict(n=8, phi=0.6, rho=0.001, seed=2), 21, "#d9480f"),
    ("(c) cycle plus branches: all merge",
     dict(kind="fig6e", n=8, phi=0.35, omega=8, rho=0.3, seed=1), 3, "#2b8a3e"),
]


def agent_color(i, n):
    # same hues as the browser demo: hsl(i * 360 / n + 18, 72%, 44%)
    return colorsys.hls_to_rgb(((i / n) + 0.05) % 1.0, 0.44, 0.72)


def simulate(kwargs, per):
    sim = make_sim(**kwargs)
    hist = sim.run(FRAMES * per)
    frames = [hist[min(f * per, len(hist) - 1)] for f in range(FRAMES + 1)]
    return hist, frames


class Camera:
    """Follows the agents still in play, eased so the view does not jump at merges."""

    def __init__(self, s):
        self.cx, self.cy, self.half = self._fit(s)

    @staticmethod
    def _fit(s):
        pts = [(x, y) for x, y, a in zip(s["x"], s["y"], s["alive"]) if a]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        half = max(max(xs) - min(xs), max(ys) - min(ys), 2.0) / 2 * 1.25 + 0.6
        return (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, half

    def update(self, s, k=0.35):
        cx, cy, half = self._fit(s)
        self.cx += k * (cx - self.cx)
        self.cy += k * (cy - self.cy)
        self.half += k * (half - self.half)


def draw_agents(ax, hist, s, step, cam):
    n = len(s["x"])
    ax.cla()
    ax.set_aspect("equal")
    ax.set_xlim(cam.cx - cam.half, cam.cx + cam.half)
    ax.set_ylim(cam.cy - cam.half, cam.cy + cam.half)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color(GRID)
    lo = max(0, step - TRAIL)
    for i in range(n):
        if not s["alive"][i]:                 # merged: its trail goes with it
            continue
        ax.plot([h["x"][i] for h in hist[lo:step + 1]], [h["y"][i] for h in hist[lo:step + 1]],
                color=agent_color(i, n), lw=1.1, alpha=0.6)
    for i in range(n):                        # pursuit edges: agent -> target
        j = s["targets"][i]
        if s["alive"][i] and j >= 0:
            ax.plot([s["x"][i], s["x"][j]], [s["y"][i], s["y"][j]], color=MUTED, lw=0.6,
                    alpha=0.45, ls=(0, (3, 3)))
    size = 0.075 * cam.half
    for i in range(n):
        if not s["alive"][i]:
            continue
        c, sn = math.cos(s["psi"][i]), math.sin(s["psi"][i])
        ax.add_patch(FancyArrow(s["x"][i] - 0.5 * size * c, s["y"][i] - 0.5 * size * sn,
                                size * c, size * sn, width=0.3 * size, head_width=0.8 * size,
                                head_length=0.6 * size, length_includes_head=True,
                                color=agent_color(i, n), ec=INK, lw=0.5, zorder=5))
    ax.text(0.03, 0.03, "t = %5.1f" % s["t"], transform=ax.transAxes, fontsize=11,
            color=MUTED, family="monospace")
    if sum(s["alive"]) < n:
        ax.text(0.97, 0.03, "%d left" % sum(s["alive"]), transform=ax.transAxes, fontsize=11,
                color=MUTED, ha="right")


def setup_V(ax, hist, color):
    ax.set_xlim(0, hist[-1]["t"])
    ax.set_ylim(0, 1.08 * max(h["V"] for h in hist))
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(labelsize=10)
    ax.set_xlabel("time", fontsize=11)
    line, = ax.plot([], [], color=color, lw=2)
    dot, = ax.plot([], [], "o", color=color, ms=5)
    return line, dot


def main():
    runs = [simulate(kw, per) for _, kw, per, _ in CASES]
    fig, axs = plt.subplots(2, 3, figsize=(10.8, 6.7), gridspec_kw={"height_ratios": [1, 0.55]})
    curves = []
    for col, ((title, _, _, color), (hist, _)) in enumerate(zip(CASES, runs)):
        axs[0, col].set_title(title, fontsize=12, loc="left")
        curves.append(setup_V(axs[1, col], hist, color))
    axs[1, 0].set_ylabel("V = Σ distance to target", fontsize=11)
    cams = [Camera(frames[0]) for _, frames in runs]
    tmp = tempfile.mkdtemp(prefix="rwc-gif-")
    try:
        for f in range(FRAMES + 1):
            for col, ((title, _, per, _), (hist, frames)) in enumerate(zip(CASES, runs)):
                s = frames[f]
                step = min(f * per, len(hist) - 1)
                cams[col].update(s)
                draw_agents(axs[0, col], hist, s, step, cams[col])
                axs[0, col].set_title(title, fontsize=12, loc="left")
                line, dot = curves[col]
                line.set_data([h["t"] for h in hist[:step + 1]], [h["V"] for h in hist[:step + 1]])
                dot.set_data([s["t"]], [s["V"]])
            if f == 0:
                fig.tight_layout(h_pad=1.5, w_pad=1.2)
            fig.savefig(os.path.join(tmp, "f%04d.png" % f), dpi=90)
        for k in range(1, HOLD + 1):
            shutil.copy(os.path.join(tmp, "f%04d.png" % FRAMES),
                        os.path.join(tmp, "f%04d.png" % (FRAMES + k)))
        plt.close(fig)
        pal = os.path.join(tmp, "palette.png")
        src = os.path.join(tmp, "f%04d.png")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", src, "-vf",
                        "palettegen=max_colors=64:stats_mode=diff", pal], check=True)
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", str(FPS), "-i", src,
                        "-i", pal, "-lavfi", "paletteuse=dither=none:diff_mode=rectangle", OUT],
                       check=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("wrote", os.path.relpath(OUT), os.path.getsize(OUT), "bytes")


if __name__ == "__main__":
    main()
