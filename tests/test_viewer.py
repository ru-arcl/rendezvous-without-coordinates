import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from rwc.viewer import Viewer, make_leader_sim  # noqa: E402


def test_followers_converge_on_a_still_mouse():
    sim = make_leader_sim(n=8, phi=0.3, omega=4.0, dt=0.05, seed=3)
    lead = sim.n - 1
    sim.x[lead], sim.y[lead] = 10.0, 10.0
    V0 = sim.V()
    for _ in range(3000):
        sim.step()
    assert sim.V() < 0.25 * V0


@pytest.mark.parametrize("chain", [False, True])
def test_viewer_updates_headless(chain):
    v = Viewer(n=6, chain=chain)
    for _ in range(5):
        v.update()
    v.cyclic = True
    v.reset()
    v.update()
    assert len(v.vs) == 2 and v.sim.targets[-1] == -1
