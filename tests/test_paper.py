"""Worked examples and stated results of the paper, as regression tests.

Equation, lemma and figure numbers refer to the IEEE TAC 2012 version unless marked CDC.
Each expected value was checked by an independent computation before being written here.
"""

import math
import random

import pytest

from rwc import bounds, make_sim
from rwc.graph import classify, find_cycle
from rwc.lyapunov import V_dot
from rwc.scenarios import FIG6E_TARGETS, initial_state

PI = math.pi


# ---------------------------------------------------------------- bounds as printed
@pytest.mark.parametrize("n,expected", [
    (2, PI / 2), (3, math.acos(2 / 3)), (4, math.acos(3 / 4)), (5, PI / 5), (8, PI / 8),
    (16, PI / 16)])
def test_theorem4_eq11(n, expected):
    assert bounds.thm4_phi(n) == pytest.approx(expected, abs=1e-15)


def test_theorem4_bound_is_min_of_both_terms_for_n_ge_3():
    # the proof of Lemma 7 uses pi/n < acos((n-1)/n) for n >= 5; for n = 3, 4 the order flips
    for n in range(3, 40):
        assert bounds.thm4_phi(n) == pytest.approx(min(PI / n, math.acos((n - 1) / n)))
        assert (PI / n < math.acos((n - 1) / n)) == (n >= 5)
    # CDC eq. (10) agrees for n >= 3 and is stricter for n = 2
    assert bounds.thm4_phi_cdc(2) == pytest.approx(PI / 3)
    assert bounds.thm4_phi_cdc(7) == pytest.approx(bounds.thm4_phi(7))


@pytest.mark.parametrize("n,expected", [(3, PI / 3), (4, math.acos(2 / 3)), (6, math.acos(4 / 5))])
def test_lemma11_bound(n, expected):
    assert bounds.lemma11_phi(n) == pytest.approx(expected)


def test_theorem12_and_13_bounds():
    assert bounds.thm12_phi(3) == pytest.approx(PI / 5)
    assert bounds.thm12_phi(9) == pytest.approx(PI / 9)
    # eq. (32) with identical speeds, and always below pi/n (so the proof's phi < pi/n holds)
    assert bounds.thm13_phi(8, 1, 1) == pytest.approx(math.acos(1 - (1 - math.cos(PI / 8)) / 8))
    for n in range(3, 30):
        for r in (0.1, 0.5, 1.0):
            assert bounds.thm13_phi(n, r, 1.0) < PI / n


def test_corollary20_eq52():
    # -n(1 + cos((n-2)pi/n)) = -n(1 - cos(2pi/n)); the max switches to -1 at n = 20
    for n in range(2, 40):
        assert -n * (1 + math.cos((n - 2) * PI / n)) == pytest.approx(-n * (1 - math.cos(2 * PI / n)))
    assert bounds.cor20_rate(3) == -1.0
    assert bounds.cor20_rate(19) == -1.0
    assert bounds.cor20_rate(20) == pytest.approx(-20 * (1 - math.cos(PI / 10)))


# ------------------------------------------------------- Lemma 3, eq. (44), Fig. 8
@pytest.mark.parametrize("n", range(3, 11))
def test_lemma3_regular_polygon_vdot(n):
    assert bounds.regular_polygon_vdot(n, PI / n) == pytest.approx(0.0, abs=1e-12)
    assert bounds.regular_polygon_vdot(n, 1.05 * PI / n) > 0
    assert bounds.regular_polygon_vdot(n, 0.95 * PI / n) < 0
    # the same from geometry: agents on the vertices moving along the circumcircle tangents
    x, y, psi, t = initial_state("regular", n, 0, 0.5, radius=3.0)
    assert V_dot(x, y, psi, [1.0] * n, t) == pytest.approx(0.0, abs=1e-12)


def test_fig8_regular_9gon_bearing_is_pi_over_9():
    sim = make_sim("regular", n=9, phi=0.4, radius=5.0)
    for i in range(9):
        assert sim.bearing_to(i, sim.targets[i]) == pytest.approx(-PI / 9, abs=1e-12)
    sim.step()                       # every target is in view: nobody turns
    assert sim.u == [0.0] * 9


# ---------------------------------------------------------------- Fig. 6 and video
def test_fig6a_converges_and_fig6c_diverges():
    # n = 8, omega = 2.0, dt = 0.1 (CDC Fig. 4 caption), merging inhibited
    conv = make_sim("cycle", n=8, seed=1, phi=0.35, omega=2.0, dt=0.1, rho=1e-6)
    V0 = conv.V()
    conv.run(3000)
    assert conv.V() < 0.08 * V0
    div = make_sim("cycle", n=8, seed=2, phi=0.6, omega=2.0, dt=0.1, rho=1e-6)
    V0 = div.V()
    h = div.run(3000, record_every=100)
    assert min(s["V"] for s in h) < 0.6 * V0     # first closer (random start gives V' < 0) ...
    assert div.V() > 2 * V0                       # ... then the ring grows without bound
    # the diverging agents sit on a near-regular octagon: all edges nearly equal
    l = div.edge_lengths()
    assert max(l) / min(l) < 1.05


def test_fig6e_graph():
    assert classify(FIG6E_TARGETS) == "cycle_branches"
    assert sorted(find_cycle(FIG6E_TARGETS)) == [0, 3, 4, 5]
    sim = make_sim("fig6e", n=8, seed=1, phi=0.35, omega=8.0, rho=0.3)
    sim.run(3000)
    assert sim.n_alive() == 1


def test_video_16_agents():
    # The applet's video: phi = 0.2 'rendezvous', phi = 0.4 'diverging' (pi/16 = 0.196)
    a = make_sim("cycle", n=16, seed=3, phi=0.2, omega=2.0, dt=0.1, rho=1e-6, height=28.0)
    b = make_sim("cycle", n=16, seed=3, phi=0.4, omega=2.0, dt=0.1, rho=1e-6, height=28.0)
    V0 = a.V()
    a.run(6000)
    b.run(6000)
    assert a.V() < 0.06 * V0
    assert b.V() > 1.5 * V0


# ------------------------------------------------------- Lemma 11 is tight; Fig. 5
def _star_intree_vdot(n, beta):
    """root <- k <- (n - 2 agents directly behind k): Lemma 11's worst case.

    k sees the root at bearing beta; each follower sits on the ray opposite to k's velocity and
    sees k at bearing beta, so V' = -cos(beta) + (n - 2)(1 - cos(beta)).
    """
    xs, ys, psis, tg = [0.0], [0.0], [0.0], [-1]
    # agent 1 = k at distance 5 from the root, heading so the root is at bearing beta
    kx, ky = -5.0, 0.0
    kpsi = 0.0 - beta
    xs.append(kx), ys.append(ky), psis.append(kpsi), tg.append(0)
    for m in range(n - 2):
        d = 2.0 + m
        fx, fy = kx - d * math.cos(kpsi), ky - d * math.sin(kpsi)
        xs.append(fx), ys.append(fy), psis.append(kpsi - beta), tg.append(1)
    return V_dot(xs, ys, psis, [1.0] * n, tg)


@pytest.mark.parametrize("n", [3, 4, 6, 8])
def test_lemma11_tight(n):
    phi = bounds.lemma11_phi(n)
    for beta in (0.3 * phi, 0.9 * phi, phi):
        assert _star_intree_vdot(n, beta) == pytest.approx(
            -math.cos(beta) + (n - 2) * (1 - math.cos(beta)), abs=1e-12)
    assert _star_intree_vdot(n, phi) == pytest.approx(0.0, abs=1e-12)
    assert _star_intree_vdot(n, 1.02 * phi) > 0


def test_fig5_bounded_speed_intree_can_have_positive_vdot():
    # r <- i <- {j, k}: i fast, j and k slow and nearly collinear behind i
    x = [0.0, -1.0, -3.0, -3.0]
    y = [0.0, 0.0, 0.05, -0.05]
    t = [-1, 0, 1, 1]
    psi = [0.0, 0.0, math.atan2(-0.05, 2.0), math.atan2(0.05, 2.0)]
    assert V_dot(x, y, psi, [0.0, 1.0, 0.05, 0.05], t) > 0
    # ... yet everybody merges into the root (Lemma 14)
    sim = make_sim("intree", n=6, seed=5, phi=0.5, omega=8.0, rho=0.3,
                   speeds=[1.0, 0.3, 0.7, 0.2, 1.0, 0.5])
    sim.run(5000)
    assert sim.n_alive() == 1


# ------------------------------------------------------------ Proposition 10
def _min_enclosing_radius(pts):
    def circle2(a, b):
        cx, cy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        return cx, cy, math.dist(a, (cx, cy))

    def circle3(a, b, c):
        ax, ay = a
        bx, by = b
        cx, cy = c
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-14:
            return None
        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay)
              + (cx * cx + cy * cy) * (ay - by)) / d
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx)
              + (cx * cx + cy * cy) * (bx - ax)) / d
        return ux, uy, math.dist(a, (ux, uy))

    def inside(c, p):
        return math.dist((c[0], c[1]), p) <= c[2] + 1e-9

    best = None
    n = len(pts)
    for i in range(n):              # O(n^4) brute force is fine for small n
        for j in range(i + 1, n):
            cands = [circle2(pts[i], pts[j])]
            for k in range(j + 1, n):
                c = circle3(pts[i], pts[j], pts[k])
                if c:
                    cands.append(c)
            for c in cands:
                if all(inside(c, p) for p in pts) and (best is None or c[2] < best):
                    best = c[2]
    return best


def test_prop10_bounding_disc():
    rng = random.Random(3)
    worst = 0.0
    for _ in range(300):
        n = rng.randint(3, 8)
        pts = [(rng.gauss(0, 1), rng.gauss(0, 1)) for _ in range(n)]
        perim = sum(math.dist(pts[i], pts[(i + 1) % n]) for i in range(n))
        r = _min_enclosing_radius(pts)
        assert r <= bounds.closed_curve_radius(perim) + 1e-9 <= bounds.prop10_radius(perim)
        worst = max(worst, r / perim)
    assert worst <= 0.25


# ------------------------------------------------------------- control law signs
def test_eq43_keeps_target_and_cdc_sign_loses_it():
    # pursuer at origin heading +x, stationary target just outside the left edge
    from rwc.sim import Sim
    phi = 0.35
    b0 = phi + 0.05
    def run(sign):
        s = Sim([0.0, 10 * math.cos(b0)], [0.0, 10 * math.sin(b0)], [0.0, 0.0], [1, -1],
                phi=phi, omega=2.0, dt=0.1, rho=1e-3)
        s.obs_prev[0] = 1             # it was last seen in the left half
        s.seen[0] = True
        out = []
        for _ in range(10):
            s.step()
            if sign < 0:              # CDC (12): turn the other way
                s.psi[0] -= 2 * s.u[0] * s.dt
            out.append(abs(s.bearing_to(0, 1)))
        return out
    assert run(+1)[0] < phi              # TAC (43): back in view after one step
    cdc = run(-1)                        # CDC (12) in this frame: turns away from the target
    assert min(cdc) > phi and cdc == sorted(cdc)


def test_cdc_sign_regains_target_only_after_almost_a_full_turn():
    from rwc.sim import Sim
    phi = 0.35
    b0 = phi + 0.02
    s = Sim([0.0, 10 * math.cos(b0)], [0.0, 10 * math.sin(b0)], [0.0, 0.0], [1, -1],
            phi=phi, omega=2.0, dt=0.1, rho=1e-3)
    s.obs_prev[0], s.seen[0] = 1, True
    in_view = []
    for _ in range(31):
        s.step()
        s.psi[0] -= 2 * s.u[0] * s.dt      # CDC (12): the opposite turn
        in_view.append(abs(s.bearing_to(0, 1)) < phi)
    first = in_view.index(True)
    assert first >= 25                    # lost for about (2 pi - 2 phi)/omega = 2.8 time units
    assert sum(in_view) <= 4


def test_paper_law_never_finds_an_unseen_target():
    # eq. (43) with u[0] = 0 and the target out of view keeps going straight forever
    from rwc.sim import Sim
    s = Sim([0.0, -5.0], [0.0, 0.0], [0.0, 0.0], [1, -1], phi=0.3, paper=True)
    s.run(200)
    assert s.u[0] == 0.0 and abs(s.bearing_to(0, 1)) > 0.3
    s = Sim([0.0, -5.0], [0.0, 0.0], [0.0, 0.0], [1, -1], phi=0.3)
    closest = 5.0
    for _ in range(200):          # default: search by turning, then pursue
        s.step()
        closest = min(closest, math.dist((s.x[0], s.y[0]), (s.x[1], s.y[1])))
    assert s.seen[0] and closest < 0.5
