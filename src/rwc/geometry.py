"""Angles, bearings and exact Dubins-arc integration."""

import math

PI = math.pi
TWO_PI = 2.0 * math.pi


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return a - TWO_PI * math.floor((a + PI) / TWO_PI)


def dist(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


def bearing(xi, yi, psi_i, xj, yj):
    """Angle of agent j relative to agent i's heading; positive means j is to the left."""
    return wrap(math.atan2(yj - yi, xj - xi) - psi_i)


def dubins_step(x, y, psi, v, u, dt):
    """Exact integration of the Dubins car over dt with constant speed v and turn rate u."""
    if u == 0.0:
        return x + v * dt * math.cos(psi), y + v * dt * math.sin(psi), psi
    p2 = psi + u * dt
    r = v / u
    return (x + r * (math.sin(p2) - math.sin(psi)),
            y + r * (math.cos(psi) - math.cos(p2)),
            wrap(p2))
