"""Single-target assignment graphs (Section III).

A graph is a list ``targets`` where ``targets[i]`` is agent i's target or -1.
"""

NONE = -1


def cycle(n):
    """Section VII: each agent targets the one with the preceding identifier; 0 targets n-1."""
    return [(i - 1) % n for i in range(n)] if n > 1 else [NONE]


def undirected_components(targets, alive=None):
    n = len(targets)
    alive = alive if alive is not None else [True] * n
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i, j in enumerate(targets):
        if alive[i] and j != NONE and alive[j]:
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[ri] = rj
    comps = {}
    for i in range(n):
        if alive[i]:
            comps.setdefault(find(i), []).append(i)
    return sorted(comps.values())


def is_connected(targets, alive=None):
    """Weak connectivity of the assignment graph (Lemma 1)."""
    return len(undirected_components(targets, alive)) <= 1


def is_live(targets, alive=None):
    """Live: at least one edge, or a single vertex (Section III)."""
    n = len(targets)
    alive = alive if alive is not None else [True] * n
    count = sum(alive)
    return count <= 1 or any(alive[i] and targets[i] != NONE for i in range(n))


def find_cycle(targets, alive=None):
    """Agents on the (first found) directed cycle, in pursuit order, or [] for a forest."""
    n = len(targets)
    alive = alive if alive is not None else [True] * n
    state = [0] * n
    for s in range(n):
        if not alive[s] or state[s]:
            continue
        path, pos = [], {}
        i = s
        while i != NONE and alive[i] and state[i] == 0:
            state[i] = 1
            pos[i] = len(path)
            path.append(i)
            i = targets[i]
        if i != NONE and alive[i] and i in pos:
            return path[pos[i]:]
        for k in path:
            state[k] = 2
    return []


def classify(targets, alive=None):
    """'cycle', 'cycle_branches', 'intree', 'forest' (disconnected) or 'single'."""
    n = len(targets)
    alive = alive if alive is not None else [True] * n
    m = sum(alive)
    if m <= 1:
        return "single"
    if not is_connected(targets, alive):
        return "forest"
    cyc = find_cycle(targets, alive)
    if not cyc:
        return "intree"
    return "cycle" if len(cyc) == m else "cycle_branches"


def merge(targets, alive, i, j):
    """Merge agent i into its target j (eq. (4)); mutates and returns ``targets``.

    Vertex i and edge (i, j) are deleted; every edge k -> i becomes k -> j. An edge j -> i would
    become a self-loop, so j is left without a target (mutual merging: j stops).
    """
    alive[i] = False
    targets[i] = NONE
    for k in range(len(targets)):
        if alive[k] and targets[k] == i:
            targets[k] = NONE if k == j else j
    return targets
