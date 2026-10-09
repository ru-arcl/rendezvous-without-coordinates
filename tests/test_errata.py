"""Reproductions for docs/notes/: places where the paper is wrong, loose, or incomplete.

Each test reproduces one finding, with the package's engine or, where a scenario needs a
hand-driven target, with its geometry primitives. A test passing means the finding still stands.
"""

import math

import pytest

from rwc import bounds, make_sim
from rwc.geometry import dist, dubins_step, wrap
from rwc.lyapunov import V_dot
from rwc.scenarios import initial_state
from rwc.sim import Sim

PI = math.pi


# ------------------------------------------------ 01: windshield bounds
@pytest.mark.parametrize("n", [3, 4])
def test_theorem4_is_conservative_for_n_3_4(n):
    # The regular polygon at the paper's bound is far from V' = 0; pi/n is where it reaches 0.
    assert bounds.thm4_phi(n) < bounds.critical_phi(n)
    assert bounds.regular_polygon_vdot(n, bounds.thm4_phi(n)) < -0.3
    assert bounds.regular_polygon_vdot(n, 0.999 * PI / n) < 0


@pytest.mark.parametrize("n", [5, 8, 12])
def test_corollary9_no_uniform_margin_at_equality(n):
    # eq. (11) allows phi = pi/n; then V' of a regular n-gon tends to 0 (bearings -> pi/n inside
    # the open windshield), so there is no delta > 0 with V' < -delta.
    eps = 1e-6
    x, y, psi, t = initial_state("regular", n, 0, PI / n, radius=2.0, beta=PI / n - eps)
    vd = V_dot(x, y, psi, [1.0] * n, t)
    assert -1e-4 < vd < 0
    assert vd / eps == pytest.approx(-2 * n * math.sin(PI / n), rel=1e-3)


def test_theorem12_proof_step_k4_is_wrong():
    # "When k = 2, 3, 4, V'_k is less than -1": a square with all bearings -> pi/5 gives -0.885
    phi = PI / 5
    x, y, psi, t = initial_state("regular", 4, 0, phi, radius=1.0, beta=phi * (1 - 1e-9))
    vd = V_dot(x, y, psi, [1.0] * 4, t)
    assert vd == pytest.approx(4 * (math.sin(phi) - math.cos(phi)), abs=1e-7)
    assert vd == pytest.approx(-0.8849270, abs=1e-6) and vd > -1
    # ... the theorem survives: with one branch agent (n = 5) the sup is still negative
    assert vd + (1 - math.cos(phi)) == pytest.approx(-0.693944, abs=1e-6)
    # the square's value -4 sqrt(2) sin(pi/4 - phi) drops below -1 only for phi < 34.82 degrees
    crossing = PI / 4 - math.asin(1 / (4 * math.sqrt(2)))
    assert math.degrees(crossing) == pytest.approx(34.8179, abs=1e-3)


# ------------------------------------------------ 02: turn rate (Prop. 16, Prop. 17)
def _prop16_scenario(omega, phi=PI / 4, dt=1e-4, T=3.0):
    """Verifier scenario A: target j drives straight, perpendicular to i's line of sight."""
    xj, yj, pj = 0.849376, 0.847679, 2.355194        # l0 = 1.2, bearing phi - 1e-3
    far = 1e6                                         # j's own target: far ahead, so j drives straight
    s = Sim([0.0, xj, xj + far * math.cos(pj)], [0.0, yj, yj + far * math.sin(pj)], [0.0, pj, 0.0],
            [1, 2, -1], phi=phi, omega=omega, dt=dt, rho=1.0, controller="eq3", eps=0.02)
    out, first, closest = 0.0, None, 1e9
    for _ in range(int(T / dt)):
        s.step()
        assert s.alive[0], "merged"
        closest = min(closest, dist(s.x[0], s.y[0], s.x[1], s.y[1]))
        if abs(s.bearing_to(0, 1)) >= phi:
            out += dt
            if first is None:
                first = (s.t, dist(s.x[0], s.y[0], s.x[1], s.y[1]))
    _prop16_scenario.closest = closest
    return first, out


def test_prop16_eq40_is_not_sufficient():
    rho, v = 1.0, 1.0
    omega = 1.2
    assert omega > bounds.prop16_omega(v, rho)                 # satisfies (40) ...
    first, out = _prop16_scenario(omega)
    assert first is not None                                   # ... yet the target escapes
    assert first[0] == pytest.approx(0.0045, abs=3e-4) and first[1] > rho
    assert out == pytest.approx(1.2775, abs=5e-3)              # out of view for ~1.28 time units
    assert _prop16_scenario.closest == pytest.approx(1.0521, abs=2e-3)   # never within rho


def test_prop16_corrected_condition_keeps_the_target():
    omega = 2.1
    assert omega > bounds.tracking_omega(1.0, 1.0, PI / 4)    # (1 + sin(pi/4)) = 1.707
    first, out = _prop16_scenario(omega)
    assert first is None and out == 0.0


def test_prop16_target_never_regained():
    # Verifier scenario B: j turns, drives straight, then circles on i's own turning circle,
    # staying just outside the windshield at distance > rho forever.
    phi, w, rho, dt = PI / 4, 1.2, 1.0, 1e-4
    xi, yi, pi_ = 0.0, 0.0, 0.0
    xj, yj, pj = 0.999225474, 0.997229018, 2.321287905
    T1, S = 0.013695395, 0.561680269
    ui, side = 0.0, 0
    t, out = 0.0, 0.0
    while t < 30.0:
        b = wrap(math.atan2(yj - yi, xj - xi) - pi_)
        if abs(b) >= phi - 0.02:          # eq. (2)-(3) with eps = 0.02: turn toward the target
            side = 1 if b > 0 else -1
            ui = w * side
        elif abs(b) < phi - 0.02:
            ui = 0.0
        uj = w if (t < T1 or t >= T1 + S) else 0.0
        xi, yi, pi_ = dubins_step(xi, yi, pi_, 1.0, ui, dt)
        xj, yj, pj = dubins_step(xj, yj, pj, 1.0, uj, dt)
        t += dt
        assert dist(xi, yi, xj, yj) > rho
        if abs(wrap(math.atan2(yj - yi, xj - xi) - pi_)) >= phi:
            out += dt
    assert out > 29.9
    assert abs(wrap(math.atan2(yj - yi, xj - xi) - pi_)) == pytest.approx(0.8236, abs=2e-3)
    assert dist(xi, yi, xj, yj) == pytest.approx(1.2227, abs=2e-3)


def test_prop17_eq50_should_use_vmax():
    # rho = 1, v_j = v_max = 1, v_min = 0.1, w = 2.5: (50) as printed holds, the v_max version fails
    rho, vj, vmax, vmin, w = 1.0, 1.0, 1.0, 0.1, 2.5
    lhs = 2 * 2 * PI / w
    assert lhs < 2 * PI * (rho - vmin / w) / vj          # printed (50): 5.03 < 6.03
    assert not lhs < 2 * PI * (rho - vmax / w) / vj      # corrected:     5.03 < 3.77 is false
    # with omega = 3 v_max / rho exactly and v_i = v_j = v_max, (48) is an equality, not '<'
    r = vmax / bounds.prop17_omega(vmax, rho)
    assert 2 * 2 * PI * r / vmax == pytest.approx(2 * PI * (rho - r) / vmax)


# ------------------------------------------------ 03: sampled control
def test_sampled_law_threshold_is_above_pi_over_n():
    # n = 16: phi = 0.22 is 12% above pi/16. With the paper's omega = 2, dt = 0.1 the regular
    # start settles into a ring; with dt = 0.01 it grows, as the continuous analysis predicts.
    n, phi = 16, 0.22
    assert bounds.critical_phi(n) < phi < bounds.sampled_phi(n, 2.0, 0.1)
    assert bounds.sampled_phi(n, 2.0, 0.01) < phi
    coarse = make_sim("regular", n=n, phi=phi, omega=2.0, dt=0.1, rho=1e-6, radius=3.0)
    h = coarse.run(3000, record_every=500)
    assert abs(h[-1]["V"] - h[-3]["V"]) < 0.5                  # plateau
    fine = make_sim("regular", n=n, phi=phi, omega=2.0, dt=0.01, rho=1e-6, radius=3.0)
    h = fine.run(30000, record_every=5000)
    assert h[-1]["V"] > h[-2]["V"] + 1.0 > h[-3]["V"] + 2.0     # keeps growing


@pytest.mark.parametrize("n", [8, 16])
def test_sampled_threshold_coefficient(n):
    # the omega*dt/2 shift: a regular start rings below 0.95x the sampled threshold, grows above 1.05x
    th = bounds.sampled_phi(n, 2.0, 0.1)
    lo = make_sim("regular", n=n, phi=0.95 * th, omega=2.0, dt=0.1, rho=1e-6, radius=3.0)
    h = lo.run(12000, record_every=2000)
    assert abs(h[-1]["V"] - h[-2]["V"]) < 0.05 * h[-1]["V"]
    hi = make_sim("regular", n=n, phi=1.05 * th, omega=2.0, dt=0.1, rho=1e-6, radius=3.0)
    h = hi.run(12000, record_every=2000)
    assert h[-1]["V"] > h[-2]["V"] + 10


def test_one_percent_above_pi_over_n_does_not_diverge_at_paper_dt():
    for seed in (1, 2):
        sim = make_sim("cycle", n=8, seed=seed, phi=1.01 * PI / 8, omega=2.0, dt=0.1, rho=1e-6)
        h = sim.run(6000, record_every=1000)
        assert max(s["V"] for s in h[2:]) < 10.0


# ------------------------------------------------ 04: conventions and gaps
# (CDC sign and unseen-target gap: tests/test_paper.py)
def test_redirected_edges_make_V_jump_at_a_merge():
    # 0 -> 1 merges; edge 2 -> 0 becomes 2 -> 1, which is longer by up to rho
    s = Sim([0.0, 0.4, -5.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1, -1, 0], phi=0.5, rho=0.5,
            dt=1e-3)
    V0 = s.V()
    s.step()
    assert s.events[0][1] == "merge" and s.V() - V0 > 0.39


# ------------------------------------------------ 05: improvements
def test_bounded_speed_intree_has_a_guarantee():
    # p. 428 says no equivalent of Lemma 11 exists for bounded speeds; one does when
    # v_max/v_min < (n-2)/(n-3). Worst intree: root <- k <- (n-2) leaves.
    assert bounds.intree_phi(4, 1.0, 1.0) == pytest.approx(bounds.lemma11_phi(4))
    assert bounds.intree_phi(4, 0.6, 1.0) == pytest.approx(0.42969967, abs=1e-8)
    assert bounds.intree_phi(5, 0.5, 1.0) is None                # 2 >= 3/2: impossible
    # star witness at phi just above the bound: V' > 0 with leaves fast and k slow
    n, vmin, vmax = 4, 0.6, 1.0
    phi = bounds.intree_phi(n, vmin, vmax)
    for beta, sign in ((0.99 * phi, -1), (1.01 * phi, 1)):
        kpsi = -beta
        xs, ys, ps, tg, sp = [0.0, -5.0], [0.0, 0.0], [0.0, kpsi], [-1, 0], [0.0, vmax]
        for m in range(n - 2):
            d = 2.0 + m
            xs.append(-5.0 - d * math.cos(kpsi)), ys.append(-d * math.sin(kpsi))
            ps.append(kpsi - beta), tg.append(1), sp.append(vmin)
        assert sign * V_dot(xs, ys, ps, sp, tg) > 0
