# Sign conventions and gaps in the stated algorithm

Reproduce: `pytest tests/test_paper.py -k "eq43 or unseen"`.

## The CDC 2008 laws turn the wrong way in the paper's own frame

The CDC 2008 version has several sign problems in the frame drawn in its Fig. 2 (x right, y up,
ψ counter-clockwise):

* **Eqs. (2)–(3).** "+1 = disappears to the right", with u = ω y.
* **Eqs. (11)–(12).** u = −ω after losing the target on the left.
* **Effect.** Both make the agent turn *away* from a lost target. It finds the target again only
  after almost a full revolution, so it is lost for about (2π − 2φ)/ω (in our test it is back in
  view only after step 25 of 31).
* **Eq. (7).** The convention "φ_i positive if ṗ_i is on the left of the segment" gives the
  right derivative only for clockwise-ordered polygons.

All of these are correct in a y-down screen frame, which is what a Java applet draws in. The
journal version fixes them. Eq. (43) turns toward the lost target, and the
"different sides of the ray" rule for φ_i in eq. (8) is independent of orientation.

A harmless relabeling remains in the journal: y = +1 means the left edge in (2), while −1 means
the left sector in (42). Each label is paired with its own law. Our code uses +1 = left
throughout (`docs/DESIGN.md`).

## Eq. (43) never finds a target that starts out of view

With u[0] = 0 and the target out of view, (43) drives straight forever, since "otherwise
u[k−1]" keeps u = 0. The paper assumes every target starts inside the windshield (Section
II-B), and the "start turning" search appears only for target-less agents (Section VI-B).

Our default `search_unseen=True` turns at +ω until the target has been seen once.
`paper=True` reproduces (43) exactly.

## Merging does not make V continuous

Section IV-A says that freezing the distance between two merged agents "gives us a continuous
V". It keeps the merged edge continuous. But eq. (4) also redirects every edge k → i to k → j,
and |p_i − p_j| ≤ ρ, so those edges jump by up to ρ. A mutual merge also removes the edge
j → i. So V can jump by up to (in-degree + 1)·ρ at each of the at most n − 1 merges. The
argument is unaffected: as the paper says of its zero-distance alternative, finitely many jumps
do not change the outcome. Reproduce:
`pytest tests/test_errata.py -k jump`.

## The merge rule's self-loop

Eq. (4) redirects every edge k → i to k → j. For the edge j → i of a 2-cycle this creates a
self-loop j → j. The text (Section II-C) says the merged pair "is considered as having no
target", so the self-loop is dropped (`rwc.graph.merge`).
