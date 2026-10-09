"""The Python engine reproduces the stored parity runs (shared with the JavaScript engine)."""

import json
import os

import pytest

from rwc import bounds
from rwc.lyapunov import V_dot
from rwc.scenarios import make_sim

PATH = os.path.join(os.path.dirname(__file__), "fixtures", "parity", "cases.json")
DATA = json.load(open(PATH))


@pytest.mark.parametrize("case", DATA["cases"], ids=lambda c: c["name"])
def test_parity_case(case):
    params = dict(case["params"])
    params.pop("name")
    steps = params.pop("steps")
    sim = make_sim(**params)
    frames = {f["step"]: f for f in case["frames"]}
    for s in range(steps + 1):
        if s in frames:
            f = frames[s]
            assert sim.targets == f["targets"] and sim.alive == f["alive"]
            for key in ("x", "y", "psi", "u"):
                assert getattr(sim, key) == pytest.approx(f[key], abs=1e-12)
            assert sim.V() == pytest.approx(f["V"], abs=1e-9)
            live = [t if a else -1 for t, a in zip(sim.targets, sim.alive)]
            moving = [sim.moving(i) for i in range(sim.n)]
            assert V_dot(sim.x, sim.y, sim.psi, sim.speeds, live, moving) == pytest.approx(
                f["vdot"], abs=1e-9)
            assert sim.done() == f["done"]
        if s < steps:
            sim.step()
    assert [list(e) for e in sim.events] == case["events"]


def test_bounds_table():
    for r in DATA["bounds"]:
        n = r["n"]
        assert bounds.thm4_phi(n) == r["thm4"] and bounds.critical_phi(n) == r["critical"]
        assert [bounds.intree_phi(n, v, 1.0) for v in (0.25, 0.5, 0.9, 1.0)] == r["intree"]
