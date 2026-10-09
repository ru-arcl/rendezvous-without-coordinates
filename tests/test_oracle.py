"""Independent oracles for the sufficient conditions: Monte Carlo and direct optimization of V'."""

import math
import random

import pytest

from rwc import bounds
from rwc.graph import cycle
from rwc.lyapunov import V_dot

PI = math.pi


def _random_config(rng, targets, phi, speeds=None):
    n = len(targets)
    x = [rng.gauss(0, 1) for _ in range(n)]
    y = [rng.gauss(0, 1) for _ in range(n)]
    psi = []
    for i in range(n):
        j = targets[i]
        a = math.atan2(y[j] - y[i], x[j] - x[i]) if j >= 0 else 0.0
        # bias towards the windshield edge, where V' is largest
        psi.append(a - phi * rng.choice([-1, 1]) * (1 - rng.random() ** 4))
    return x, y, psi


def _random_connected_graph(rng, n):
    k = rng.randint(2, n)
    perm = list(range(n))
    rng.shuffle(perm)
    t = [-1] * n
    for m in range(k):
        t[perm[m]] = perm[m - 1]
    for m in range(k, n):
        t[perm[m]] = perm[rng.randrange(m)]
    return t


def test_monte_carlo_vdot_negative_below_the_bounds():
    rng = random.Random(7)
    for _ in range(20000):
        n = rng.randint(2, 9)
        if rng.random() < 0.4:
            t, phi = cycle(n), bounds.thm4_phi(n)
        else:
            t, phi = _random_connected_graph(rng, n), bounds.thm12_phi(n)
        x, y, psi = _random_config(rng, t, phi * 0.999)
        assert V_dot(x, y, psi, [1.0] * n, t) < 0


def sup_vdot(*args, **kwargs):
    pytest.importorskip("scipy")
    from rwc.oracle import sup_vdot as f
    return f(*args, **kwargs)


@pytest.mark.parametrize("n", [3, 4, 5])
def test_cycle_threshold_is_pi_over_n(n):
    # sup V' < 0 just below pi/n and > 0 just above (for n = 3, 4 the paper's bound (11) is lower)
    assert sup_vdot(cycle(n), 0.97 * PI / n, restarts=12)[0] < 0
    assert sup_vdot(cycle(n), 1.03 * PI / n, restarts=12)[0] > 0
    assert bounds.thm4_phi(n) <= PI / n + 1e-12


def test_intree_threshold_is_lemma11():
    t = [-1, 0, 1, 1]          # root <- k <- two followers: the worst intree for n = 4
    phi = bounds.lemma11_phi(4)
    assert sup_vdot(t, 0.97 * phi, restarts=12)[0] < 0
    assert sup_vdot(t, 1.03 * phi, restarts=12)[0] > 0


def test_theorem13_bounded_speeds():
    n, vmin = 4, 0.5
    phi = bounds.thm13_phi(n, vmin, 1.0)
    assert sup_vdot(cycle(n), 0.999 * phi, v_lo=vmin, v_hi=1.0, restarts=12)[0] < 0


def test_theorem13_true_threshold_table_entry():
    # docs/notes/01: n = 4, v_min/v_max = 0.5, true critical phi = 0.756 (eq. (32) gives 0.271)
    n, vmin, phi_c = 4, 0.5, 0.756
    assert bounds.thm13_phi(n, vmin, 1.0) == pytest.approx(0.271, abs=1e-3)
    assert sup_vdot(cycle(n), 0.98 * phi_c, v_lo=vmin, v_hi=1.0, restarts=30, maxiter=1000)[0] < 0
    assert sup_vdot(cycle(n), 1.02 * phi_c, v_lo=vmin, v_hi=1.0, restarts=30, maxiter=1000)[0] > 0


def test_cycle_plus_branch_threshold_is_pi_over_n():
    # 4-cycle plus one branch agent (n = 5): pi/5, the same as eq. (27) here
    t = [1, 2, 3, 0, 0]
    assert sup_vdot(t, 0.97 * PI / 5, restarts=20)[0] < 0
    # 3-cycle plus one branch (n = 4): the threshold is pi/4, above eq. (27)'s pi/5
    t = [1, 2, 0, 0]
    assert sup_vdot(t, 0.97 * PI / 4, restarts=20)[0] < 0
    assert bounds.thm12_phi(4) < 0.97 * PI / 4


@pytest.mark.parametrize("n,expected", [(3, -3 + math.sqrt(15) / 2), (4, math.sqrt(7) - 3)])
def test_equality_in_eq11_is_harmless_for_n_3_4(n, expected):
    # sup V' at phi = acos((n-1)/n): -1.0635 (n = 3), -0.3542 (n = 4) (notes/01)
    v = sup_vdot(cycle(n), bounds.thm4_phi(n), restarts=20)[0]
    assert v == pytest.approx(expected, abs=1e-4)
