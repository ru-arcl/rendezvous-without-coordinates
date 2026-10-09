"""Regenerate tests/fixtures/parity/cases.json from the Python engine.

The JavaScript engine must reproduce these runs (tests/js/parity.test.js); the Python engine is
checked against them too (tests/test_parity.py), so any change to the step order shows up in both.

    python scripts/make_parity_fixtures.py
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from rwc import bounds  # noqa: E402
from rwc.lyapunov import V_dot  # noqa: E402
from rwc.scenarios import make_sim  # noqa: E402

CASES = [
    dict(name="cycle8_converge", kind="cycle", n=8, seed=1, phi=0.35, steps=400),
    dict(name="cycle8_diverge", kind="cycle", n=8, seed=2, phi=0.6, steps=400),
    dict(name="cycle16_video", kind="cycle", n=16, seed=3, phi=0.2, steps=400, height=28.0),
    dict(name="cycle_branches10", kind="cycle_branches", n=10, seed=4, phi=0.25, steps=400,
         rho=0.05),
    dict(name="intree7", kind="intree", n=7, seed=5, phi=0.5, steps=400, rho=0.2),
    dict(name="fig6e", kind="fig6e", n=8, seed=6, phi=0.35, steps=400, rho=0.05),
    dict(name="regular9_expand", kind="regular", n=9, seed=0, phi=0.4, beta=0.38, steps=200),
    dict(name="noise20", kind="cycle", n=6, seed=7, phi=0.4, noise=0.2, steps=400, rho=0.05),
    dict(name="eq3", kind="cycle", n=6, seed=8, phi=0.4, controller="eq3", eps=0.05, steps=400),
    dict(name="paper_unseen", kind="cycle", n=6, seed=9, phi=0.4, heading="random", paper=True,
         steps=300),
    dict(name="search_unseen", kind="cycle", n=6, seed=9, phi=0.4, heading="random", steps=300),
    dict(name="speeds", kind="cycle", n=5, seed=10, phi=0.3, speeds=[1.0, 0.6, 0.8, 0.5, 0.9],
         omega=3.0, steps=400, rho=0.05),
    dict(name="reassign", kind="none", n=6, seed=11, phi=0.5, heading="random", reassign=True,
         rho=0.6, omega=6.0, width=8.0, height=8.0, steps=800),
    dict(name="reassign_s3", kind="none", n=10, seed=3, phi=0.5, heading="random", reassign=True,
         rho=0.6, omega=6.0, width=10.0, height=10.0, steps=1500),
    dict(name="reassign_s9", kind="none", n=10, seed=9, phi=0.5, heading="random", reassign=True,
         rho=0.6, omega=6.0, width=10.0, height=10.0, steps=3000),
    dict(name="eq3_noise", kind="cycle", n=6, seed=8, phi=0.4, controller="eq3", eps=0.05,
         noise=0.2, steps=400, rho=0.05),
    dict(name="regular9_default_beta", kind="regular", n=9, seed=0, phi=0.45, dt=0.02, steps=300),
    dict(name="omega_array", kind="cycle", n=4, seed=12, phi=0.5, omega=[2.0, 3.0, 4.0, 5.0],
         rho=0.2, steps=400),
    dict(name="branches_k3", kind="cycle_branches", n=7, seed=13, k=3, phi=0.3, omega=8.0,
         rho=0.3, steps=600),
]

RECORD_EVERY = 20


def run_case(case):
    c = dict(case)
    name, steps = c.pop("name"), c.pop("steps")
    sim = make_sim(**c)
    frames = []
    for s in range(steps + 1):
        if s % RECORD_EVERY == 0:
            moving = [sim.moving(i) for i in range(sim.n)]
            frames.append({"step": s, "x": sim.x, "y": sim.y, "psi": sim.psi, "u": sim.u,
                           "targets": sim.targets, "alive": sim.alive, "V": sim.V(),
                           "vdot": V_dot(sim.x, sim.y, sim.psi, sim.speeds,
                                         [t if a else -1 for t, a in zip(sim.targets, sim.alive)],
                                         moving),
                           "done": sim.done()})
            frames[-1] = json.loads(json.dumps(frames[-1]))
        if s < steps:
            sim.step()
    return {"name": name, "params": case, "frames": frames,
            "events": [list(e) for e in sim.events]}


def bounds_table():
    rows = []
    for n in range(2, 21):
        rows.append({
            "n": n, "thm4": bounds.thm4_phi(n), "lemma11": bounds.lemma11_phi(n),
            "thm12": bounds.thm12_phi(n), "critical": bounds.critical_phi(n),
            "thm13": [bounds.thm13_phi(n, r, 1.0) for r in (0.25, 0.5, 1.0)],
            "intree": [bounds.intree_phi(n, r, 1.0) for r in (0.25, 0.5, 0.9, 1.0)],
            "sampled": bounds.sampled_phi(n, 2.0, 0.1),
            "regular": [bounds.regular_polygon_vdot(n, b) for b in (0.1, 0.3, 1.0)],
        })
    return rows


def main():
    out = os.path.join(HERE, "..", "tests", "fixtures", "parity", "cases.json")
    data = {"record_every": RECORD_EVERY, "cases": [run_case(c) for c in CASES],
            "bounds": bounds_table()}
    with open(out, "w") as f:
        json.dump(data, f, separators=(",", ":"))
    print("wrote", os.path.relpath(out), os.path.getsize(out), "bytes")


if __name__ == "__main__":
    main()
