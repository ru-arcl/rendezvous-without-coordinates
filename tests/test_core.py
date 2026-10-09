import math
import random

import pytest

from rwc.geometry import bearing, dubins_step, wrap
from rwc.graph import (classify, cycle, find_cycle, is_connected, is_live, merge,
                       undirected_components)
from rwc.lyapunov import V, V_dot, V_dot_cycle_formula, cycle_angles
from rwc.rng import Mulberry32


def test_mulberry32_matches_javascript_reference():
    # values printed by the JavaScript reference implementation (node)
    r = Mulberry32(12345)
    assert [r.random() for _ in range(3)] == [0.9797282677609473, 0.3067522644996643,
                                              0.484205421525985]
    r = Mulberry32(0xDEADBEEF)
    assert [r.random() for _ in range(2)] == [0.9413696140982211, 0.26719574979506433]


@pytest.mark.parametrize("a", [-10.0, -math.pi, -1.0, 0.0, 1.0, math.pi, 7.5, 100.0])
def test_wrap_range(a):
    w = wrap(a)
    assert -math.pi <= w < math.pi
    assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-12)
    assert math.isclose(math.sin(w), math.sin(a), abs_tol=1e-12)


def test_bearing_sign_left_positive():
    assert bearing(0, 0, 0.0, 1, 1) == pytest.approx(math.pi / 4)
    assert bearing(0, 0, 0.0, 1, -1) == pytest.approx(-math.pi / 4)


@pytest.mark.parametrize("u", [-2.0, 0.0, 1.3])
def test_dubins_step_is_exact(u):
    x, y, p = 0.3, -0.2, 0.7
    xe, ye, pe = dubins_step(x, y, p, 1.5, u, 0.4)
    m = 40000
    h = 0.4 / m
    for _ in range(m):  # midpoint rule converges to the exact arc
        pm = p + 0.5 * u * h
        x += 1.5 * h * math.cos(pm)
        y += 1.5 * h * math.sin(pm)
        p += u * h
    assert xe == pytest.approx(x, abs=1e-9)
    assert ye == pytest.approx(y, abs=1e-9)
    assert math.cos(pe) == pytest.approx(math.cos(p), abs=1e-9)


def test_cycle_graph_paper_convention():
    # Section VII: agent i targets the one with the preceding identifier, 0 targets n-1
    assert cycle(4) == [3, 0, 1, 2]
    assert classify(cycle(5)) == "cycle"
    assert sorted(find_cycle(cycle(5))) == [0, 1, 2, 3, 4]


def test_merge_redirects_edges_and_mutual_merge_stops():
    t = [1, 2, 0, 2]          # 3-cycle 0->1->2->0 plus branch 3->2
    alive = [True] * 4
    merge(t, alive, 2, 0)     # 2 merges into 0: edges into 2 now point to 0
    assert t == [1, 0, -1, 0]
    assert alive == [True, True, False, True]
    merge(t, alive, 1, 0)     # 0 -> 1 would become a self loop: 0 stops
    assert t[0] == -1 and t[3] == 0 and classify(t, alive) == "intree"


def _random_single_target_graph(rng, n):
    t = [rng.randrange(n - 1) for _ in range(n)]
    return [j if j < i else j + 1 for i, j in enumerate(t)]


def test_lemma1_connectivity_preserved_and_liveness():
    rng = random.Random(0)
    for _ in range(300):
        n = rng.randint(2, 9)
        t = _random_single_target_graph(rng, n)
        alive = [True] * n
        connected = is_connected(t, alive)
        while True:
            edges = [i for i in range(n) if alive[i] and t[i] >= 0]
            if not edges:
                break
            i = rng.choice(edges)
            merge(t, alive, i, t[i])
            assert is_connected(t, alive) or not connected
            if connected:
                assert is_live(t, alive)
        # Lemma 1 (only if): a disconnected graph collapses to >1 vertex with no edges
        assert (sum(alive) == 1) == connected
        assert len(undirected_components(t, alive)) == sum(alive)


def _random_cycle_config(rng, k, phi):
    x = [rng.uniform(-5, 5) for _ in range(k)]
    y = [rng.uniform(-5, 5) for _ in range(k)]
    order = list(range(k))
    targets = [(i + 1) % k for i in range(k)]
    psi = []
    for i in range(k):
        j = targets[i]
        psi.append(math.atan2(y[j] - y[i], x[j] - x[i]) + rng.uniform(-phi, phi))
    return x, y, psi, targets, order


def test_vdot_matches_finite_difference_and_eq9():
    rng = random.Random(1)
    for _ in range(200):
        k = rng.randint(2, 9)
        x, y, psi, targets, order = _random_cycle_config(rng, k, 1.2)
        speeds = [rng.uniform(0.3, 2.0) for _ in range(k)]
        vd = V_dot(x, y, psi, speeds, targets)
        h = 1e-6
        xp = [x[i] + h * speeds[i] * math.cos(psi[i]) for i in range(k)]
        yp = [y[i] + h * speeds[i] * math.sin(psi[i]) for i in range(k)]
        xm = [x[i] - h * speeds[i] * math.cos(psi[i]) for i in range(k)]
        ym = [y[i] - h * speeds[i] * math.sin(psi[i]) for i in range(k)]
        fd = (V(xp, yp, targets) - V(xm, ym, targets)) / (2 * h)
        assert vd == pytest.approx(fd, rel=1e-5, abs=1e-6)
        thetas, phis = cycle_angles(x, y, psi, order)
        assert V_dot_cycle_formula(thetas, phis, speeds) == pytest.approx(vd, abs=1e-9)


def test_reassign_merges_nearby_agent_before_picking_a_target():
    # Sec. VI-B: a targetless agent first merges with agents within rho, then looks for targets
    from rwc.sim import Sim
    s = Sim([5.0, 0.0, 0.1], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [-1, -1, -1], phi=0.5, rho=0.5,
            dt=0.1, reassign=True)
    s.step()
    assert s.alive == [True, False, True] and s.merged_into[1] == 2
    assert s.events[0] == (0, "merge", 1, 2)
    assert s.frozen[1] == pytest.approx(0.1)


def test_reassign_merges_into_lowest_index_and_chains_within_a_step():
    from rwc.sim import Sim
    # all targetless; 0 is within rho of 1 and 2 -> merges into 1 (lowest index, not nearest);
    # then 1 (now targetless) is within rho of 2 -> merges into 2
    s = Sim([0.0, 0.3, 0.1], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [-1, -1, -1], phi=0.3, rho=0.45,
            reassign=True)
    s.step()
    assert s.merged_into == [1, 2, -1] and s.alive == [False, False, True]
    assert s.representative(0) == 2 and s.done()


def test_searching_agent_merges_after_moving_into_lower_index():
    # step 4: agent 1 has no target and sees nobody, so it turns; after moving it is within rho of
    # agent 0 (lower index), which is driving toward a far target
    from rwc.sim import Sim
    s = Sim([0.55 * math.cos(0.5), 0.0, 0.55 * math.cos(0.5)],
            [0.55 * math.sin(0.5), 0.0, -100.0], [-math.pi / 2, 0.0, 0.0], [2, -1, -1],
            phi=0.1, omega=1.0, rho=0.5, dt=0.1, reassign=True)
    assert s._nearby(1) == -1
    s.step()
    assert s.events == [(0, "merge", 1, 0)] and s.merged_into[1] == 0


@pytest.mark.parametrize("kw", [dict(omega=[1.0]), dict(speeds=[1.0, -1.0]), dict(omega=0.0),
                                dict(controller="eq3", eps=0.5), dict(controller="nope"),
                                dict(phi=0.0), dict(dt=0.0), dict(rho=-1.0),
                                dict(dt=float("nan")), dict(rho=float("nan"))])
def test_sim_rejects_bad_parameters(kw):
    from rwc.sim import Sim
    args = dict(phi=0.4)
    args.update(kw)
    with pytest.raises(ValueError):
        Sim([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [1, 0], **args)


def test_sim_accepts_numpy_scalars_and_arrays():
    np = pytest.importorskip("numpy")
    from rwc.sim import Sim
    s = Sim([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [1, 0], phi=np.float64(0.4), omega=np.int64(2),
            speeds=np.array([1.0, 0.5]))
    s.step()
    assert s.omega == [2.0, 2.0]


@pytest.mark.parametrize("controller", ["eq43", "eq3"])
def test_noise_rate_matches_parameter(controller):
    from rwc.sim import Sim
    s = Sim([0.0, 1.0], [0.0, 0.0], [0.0, 0.0], [1, -1], phi=0.4, noise=0.3, seed=5,
            controller=controller, eps=0.05)
    flips, m = 0, 20000
    for _ in range(m):
        clean = 0
        if controller == "eq43":
            flips += s._observe(0) != s._observe_clean(0)
        else:
            flips += s._control(0) != clean * s.omega[0]
    assert flips / m == pytest.approx(0.3, abs=0.015)
