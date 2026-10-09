// The JavaScript engine reproduces the Python parity runs in tests/fixtures/parity/cases.json.
"use strict";
const test = require("node:test");
const assert = require("node:assert");
const path = require("node:path");
const fs = require("node:fs");
const RWC = require("../../docs/js/rwc-engine.js");

const data = JSON.parse(fs.readFileSync(path.join(__dirname, "..", "fixtures", "parity", "cases.json"), "utf8"));
const TOL = 1e-9;

function close(a, b, what) {
  assert.strictEqual(a.length, b.length, what + " length");
  for (let i = 0; i < a.length; i++) {
    assert.ok(Math.abs(a[i] - b[i]) <= TOL, `${what}[${i}]: ${a[i]} vs ${b[i]}`);
  }
}

for (const c of data.cases) {
  test("parity: " + c.name, () => {
    const p = c.params;
    const sim = RWC.makeSim(p.kind, p.n, p.seed, p.phi, {
      omega: p.omega, dt: p.dt, rho: p.rho, width: p.width, height: p.height, heading: p.heading,
      k: p.k, radius: p.radius, beta: p.beta, speeds: p.speeds, controller: p.controller,
      eps: p.eps, paper: p.paper, noise: p.noise, reassign: p.reassign
    });
    const frames = new Map(c.frames.map((f) => [f.step, f]));
    for (let s = 0; s <= p.steps; s++) {
      const f = frames.get(s);
      if (f) {
        assert.deepStrictEqual(sim.targets, f.targets, `targets at step ${s}`);
        assert.deepStrictEqual(sim.alive, f.alive, `alive at step ${s}`);
        close(sim.x, f.x, `x@${s}`); close(sim.y, f.y, `y@${s}`);
        close(sim.psi, f.psi, `psi@${s}`); close(sim.u, f.u, `u@${s}`);
        assert.ok(Math.abs(sim.V() - f.V) <= TOL, `V@${s}`);
        assert.ok(Math.abs(sim.Vdot() - f.vdot) <= TOL, `Vdot@${s}: ${sim.Vdot()} vs ${f.vdot}`);
        assert.strictEqual(sim.done(), f.done, `done@${s}`);
      }
      if (s < p.steps) sim.step();
    }
    assert.deepStrictEqual(sim.events, c.events);
  });
}

test("parity: bounds", () => {
  const B = RWC.bounds;
  for (const r of data.bounds) {
    const n = r.n;
    const near = (a, b, what) => assert.ok(a === b || Math.abs(a - b) <= 1e-12, `${what}(n=${n}): ${a} vs ${b}`);
    near(B.thm4(n), r.thm4, "thm4");
    near(B.lemma11(n), r.lemma11, "lemma11");
    near(B.thm12(n), r.thm12, "thm12");
    near(B.critical(n), r.critical, "critical");
    near(B.sampled(n, 2.0, 0.1), r.sampled, "sampled");
    [0.25, 0.5, 1.0].forEach((v, k) => near(B.thm13(n, v, 1.0), r.thm13[k], "thm13"));
    [0.25, 0.5, 0.9, 1.0].forEach((v, k) => near(B.intree(n, v, 1.0), r.intree[k], "intree"));
    [0.1, 0.3, 1.0].forEach((b, k) => near(B.regularVdot(n, b), r.regular[k], "regularVdot"));
  }
});

test("constructor validation", () => {
  const mk = (o) => () => new RWC.Sim([0, 1], [0, 0], [0, 0], [1, 0], Object.assign({ phi: 0.4 }, o));
  assert.throws(mk({ omega: [1] }));
  assert.throws(mk({ speeds: [1, -1] }));
  assert.throws(mk({ controller: "eq3", eps: 0.5 }));
  assert.throws(mk({ controller: "nope" }));
  assert.throws(mk({ phi: 0 }));
  assert.throws(mk({ dt: NaN }));
  assert.throws(mk({ rho: NaN }));
  assert.doesNotThrow(mk({ omega: [1, 2] }));
});
