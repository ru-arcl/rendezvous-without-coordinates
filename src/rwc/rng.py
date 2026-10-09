"""mulberry32: a tiny 32-bit PRNG, bit-identical to the JavaScript engine."""

_M = 0xFFFFFFFF


def _imul(a, b):
    return (a * b) & _M


class Mulberry32:
    """Deterministic uniform generator; ``random()`` returns floats in [0, 1)."""

    def __init__(self, seed):
        self.state = int(seed) & _M

    def next_u32(self):
        self.state = (self.state + 0x6D2B79F5) & _M
        a = self.state
        t = _imul(a ^ (a >> 15), 1 | a)
        t = ((t + _imul(t ^ (t >> 7), 61 | t)) & _M) ^ t
        return (t ^ (t >> 14)) & _M

    def random(self):
        return self.next_u32() / 4294967296.0

    def uniform(self, lo, hi):
        return lo + (hi - lo) * self.random()

    def randint(self, n):
        """Integer in [0, n)."""
        return int(self.random() * n)
