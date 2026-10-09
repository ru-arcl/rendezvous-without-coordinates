/* Rendezvous without coordinates — simulation engine.
 * A line-for-line port of src/rwc/{rng,geometry,graph,sim,scenarios}.py; the step order is
 * specified in docs/DESIGN.md and checked against Python by tests/js/parity.test.js.
 * Classic script: defines window.RWC in the browser and module.exports under Node.
 */
(function (root) {
  "use strict";

  var PI = Math.PI;
  var TWO_PI = 2 * Math.PI;
  var NONE = -1;
  var NOISE_SALT = 0x9e3779b9;

  // ------------------------------------------------------------------ rng
  function Mulberry32(seed) {
    this.state = seed >>> 0;
  }
  Mulberry32.prototype.nextU32 = function () {
    this.state = (this.state + 0x6d2b79f5) >>> 0;
    var a = this.state;
    var t = Math.imul(a ^ (a >>> 15), 1 | a) >>> 0;
    t = (((t + Math.imul(t ^ (t >>> 7), 61 | t)) >>> 0) ^ t) >>> 0;
    return (t ^ (t >>> 14)) >>> 0;
  };
  Mulberry32.prototype.random = function () {
    return this.nextU32() / 4294967296;
  };
  Mulberry32.prototype.randint = function (n) {
    return Math.floor(this.random() * n);
  };

  // ------------------------------------------------------------- geometry
  function wrap(a) {
    return a - TWO_PI * Math.floor((a + PI) / TWO_PI);
  }
  function dist(x1, y1, x2, y2) {
    var dx = x2 - x1, dy = y2 - y1;
    return Math.sqrt(dx * dx + dy * dy);
  }
  function bearing(xi, yi, psii, xj, yj) {
    return wrap(Math.atan2(yj - yi, xj - xi) - psii);
  }
  function dubinsStep(x, y, psi, v, u, dt) {
    if (u === 0) return [x + v * dt * Math.cos(psi), y + v * dt * Math.sin(psi), psi];
    var p2 = psi + u * dt;
    var r = v / u;
    return [x + r * (Math.sin(p2) - Math.sin(psi)), y + r * (Math.cos(psi) - Math.cos(p2)), wrap(p2)];
  }

  // ---------------------------------------------------------------- graph
  function cycleGraph(n) {
    if (n <= 1) return [NONE];
    var t = [];
    for (var i = 0; i < n; i++) t.push((i - 1 + n) % n);
    return t;
  }
  function mergeGraph(targets, alive, i, j) {
    alive[i] = false;
    targets[i] = NONE;
    for (var k = 0; k < targets.length; k++) {
      if (alive[k] && targets[k] === i) targets[k] = k === j ? NONE : j;
    }
  }
  function findCycle(targets, alive) {
    var n = targets.length, state = new Array(n).fill(0);
    for (var s = 0; s < n; s++) {
      if (!alive[s] || state[s]) continue;
      var path = [], pos = {}, i = s;
      while (i !== NONE && alive[i] && state[i] === 0) {
        state[i] = 1; pos[i] = path.length; path.push(i); i = targets[i];
      }
      if (i !== NONE && alive[i] && pos.hasOwnProperty(i)) return path.slice(pos[i]);
      for (var m = 0; m < path.length; m++) state[path[m]] = 2;
    }
    return [];
  }

  // ------------------------------------------------------------------ sim
  function Sim(x, y, psi, targets, opts) {
    opts = opts || {};
    var n = x.length;
    this.n = n;
    this.x = x.slice(); this.y = y.slice(); this.psi = psi.slice();
    this.targets = targets.slice();
    this.phi = opts.phi;
    if (!(this.phi > 0 && this.phi <= PI)) throw new Error("phi must be in (0, pi]");
    var omega = opts.omega === undefined ? 2.0 : opts.omega;
    var speeds = opts.speeds === undefined ? 1.0 : opts.speeds;
    this.omega = Array.isArray(omega) ? omega.slice() : new Array(n).fill(omega);
    this.speeds = Array.isArray(speeds) ? speeds.slice() : new Array(n).fill(speeds);
    if (this.omega.length !== n || this.speeds.length !== n) throw new Error("omega and speeds must be scalars or have one entry per agent");
    if (Math.min.apply(null, this.omega) <= 0 || Math.min.apply(null, this.speeds) <= 0) throw new Error("omega and speeds must be positive");
    this.dt = opts.dt === undefined ? 0.1 : opts.dt;
    this.rho = opts.rho === undefined ? 1e-3 : opts.rho;
    if (!(this.rho >= 0 && this.dt > 0)) throw new Error("need rho >= 0 and dt > 0");
    this.controller = opts.controller || "eq43";
    if (this.controller !== "eq43" && this.controller !== "eq3") throw new Error("controller must be 'eq43' or 'eq3'");
    this.eps = opts.eps === undefined ? 0.02 : opts.eps;
    if (this.controller === "eq3" && !(this.eps > 0 && this.eps < this.phi)) throw new Error("controller 'eq3' needs 0 < eps < phi");
    this.paper = !!opts.paper;
    this.searchUnseen = (opts.searchUnseen === undefined ? true : !!opts.searchUnseen) && !this.paper;
    this.noise = opts.noise || 0;
    this.reassign = !!opts.reassign;
    this.rng = new Mulberry32(((opts.seed || 0) ^ NOISE_SALT) >>> 0);
    this.alive = new Array(n).fill(true);
    this.u = new Array(n).fill(0);
    this.frozen = new Array(n).fill(0);
    this.mergedInto = new Array(n).fill(NONE);
    this.t = 0; this.steps = 0; this.events = [];
    this.obsPrev = new Array(n).fill(0);
    this.seen = new Array(n).fill(false);
    for (var i = 0; i < n; i++) {
      if (this.targets[i] !== NONE) {
        var o = this.observeClean(i);
        this.obsPrev[i] = o; this.seen[i] = o !== 0;
      }
    }
  }
  Sim.prototype.bearingTo = function (i, j) {
    return bearing(this.x[i], this.y[i], this.psi[i], this.x[j], this.y[j]);
  };
  Sim.prototype.observeClean = function (i) {
    var b = this.bearingTo(i, this.targets[i]);
    if (-this.phi < b && b < this.phi) return b >= 0 ? 1 : -1;
    return 0;
  };
  Sim.prototype.corrupt = function (o) {
    if (this.noise > 0 && this.rng.random() < this.noise) {
      var others = [-1, 0, 1].filter(function (v) { return v !== o; });
      o = this.rng.random() < 0.5 ? others[0] : others[1];
    }
    return o;
  };
  Sim.prototype.observe = function (i) {
    return this.corrupt(this.observeClean(i));
  };
  Sim.prototype.control = function (i) {
    var w = this.omega[i];
    if (this.controller === "eq3") {
      var b = this.bearingTo(i, this.targets[i]);
      var edge = this.phi - this.eps;
      var y = b >= edge ? 1 : (b <= -edge ? -1 : 0);
      return w * this.corrupt(y);
    }
    var o = this.observe(i), prev = this.obsPrev[i], u = this.u[i];
    if (o === 0 && prev === 1) u = w;
    else if (o === 0 && prev === -1) u = -w;
    else if (o !== 0 && prev === 0) u = 0;
    if (o !== 0) this.seen[i] = true;
    else if (!this.seen[i] && this.searchUnseen) u = w;
    this.obsPrev[i] = o;
    return u;
  };
  Sim.prototype.nearby = function (i) {
    for (var k = 0; k < this.n; k++) {
      if (k !== i && this.alive[k] && dist(this.x[i], this.y[i], this.x[k], this.y[k]) <= this.rho) return k;
    }
    return NONE;
  };
  Sim.prototype.reassignTarget = function (i) {
    var best = NONE, bestB = null;
    for (var k = 0; k < this.n; k++) {
      if (k === i || !this.alive[k]) continue;
      var b = Math.abs(this.bearingTo(i, k));
      if (b < this.phi && (bestB === null || b < bestB)) { best = k; bestB = b; }
    }
    return best;
  };
  Sim.prototype.nAlive = function () {
    var c = 0;
    for (var i = 0; i < this.n; i++) if (this.alive[i]) c++;
    return c;
  };
  Sim.prototype.moving = function (i) {
    if (!this.alive[i]) return false;
    if (this.targets[i] !== NONE) return true;
    return this.reassign && this.nAlive() > 1;
  };
  Sim.prototype.step = function () {
    var n = this.n, i, j, k, d;
    for (i = 0; i < n; i++) {
      if (!this.alive[i]) continue;
      if (this.targets[i] !== NONE) {
        this.u[i] = this.control(i);
      } else if (this.reassign && this.nAlive() > 1) {
        k = this.nearby(i);
        if (k !== NONE) {
          this.mergeAgents(i, k, dist(this.x[i], this.y[i], this.x[k], this.y[k]));
          continue;
        }
        k = this.reassignTarget(i);
        if (k !== NONE) {
          this.targets[i] = k;
          this.obsPrev[i] = this.observeClean(i);
          this.seen[i] = true;
          this.u[i] = 0;
          this.events.push([this.steps, "assign", i, k]);
        } else {
          this.u[i] = this.omega[i];
        }
      } else {
        this.u[i] = 0;
      }
    }
    var moving = [];
    for (i = 0; i < n; i++) moving.push(this.moving(i));
    for (i = 0; i < n; i++) {
      if (moving[i]) {
        var s = dubinsStep(this.x[i], this.y[i], this.psi[i], this.speeds[i], this.u[i], this.dt);
        this.x[i] = s[0]; this.y[i] = s[1]; this.psi[i] = s[2];
      }
    }
    for (i = 0; i < n; i++) {
      j = this.targets[i];
      if (this.alive[i] && j !== NONE) {
        d = dist(this.x[i], this.y[i], this.x[j], this.y[j]);
        if (d <= this.rho) this.mergeAgents(i, j, d);
      }
    }
    if (this.reassign) {
      for (i = 0; i < n; i++) {
        if (this.alive[i] && this.targets[i] === NONE) {
          k = this.nearby(i);
          if (k !== NONE) this.mergeAgents(i, k, dist(this.x[i], this.y[i], this.x[k], this.y[k]));
        }
      }
    }
    this.steps += 1;
    this.t = this.steps * this.dt;
  };
  Sim.prototype.mergeAgents = function (i, j, d) {
    mergeGraph(this.targets, this.alive, i, j);
    this.frozen[i] = d;
    this.mergedInto[i] = j;
    this.u[i] = 0;
    this.events.push([this.steps, "merge", i, j]);
  };
  Sim.prototype.done = function () {
    for (var i = 0; i < this.n; i++) if (this.moving(i)) return false;
    return true;
  };
  Sim.prototype.edgeLengths = function () {
    var out = [];
    for (var i = 0; i < this.n; i++) {
      var j = this.targets[i];
      if (!this.alive[i]) out.push(this.frozen[i]);
      else if (j !== NONE) out.push(dist(this.x[i], this.y[i], this.x[j], this.y[j]));
      else out.push(null);
    }
    return out;
  };
  Sim.prototype.V = function () {
    var l = this.edgeLengths(), s = 0;
    for (var i = 0; i < l.length; i++) if (l[i] !== null) s += l[i];
    return s;
  };
  Sim.prototype.Vactive = function () {
    var l = this.edgeLengths(), s = 0;
    for (var i = 0; i < l.length; i++) if (l[i] !== null && this.alive[i]) s += l[i];
    return s;
  };
  Sim.prototype.representative = function (i) {
    while (this.mergedInto[i] !== NONE) i = this.mergedInto[i];
    return i;
  };
  /* dV/dt from velocities (lyapunov.V_dot); stationary agents contribute nothing. */
  Sim.prototype.Vdot = function () {
    var n = this.n, vx = [], vy = [], i, total = 0;
    for (i = 0; i < n; i++) {
      var m = this.moving(i);
      vx.push(m ? this.speeds[i] * Math.cos(this.psi[i]) : 0);
      vy.push(m ? this.speeds[i] * Math.sin(this.psi[i]) : 0);
    }
    for (i = 0; i < n; i++) {
      var j = this.targets[i];
      if (!this.alive[i] || j === NONE) continue;
      var dx = this.x[j] - this.x[i], dy = this.y[j] - this.y[i];
      var l = Math.sqrt(dx * dx + dy * dy);
      total += ((vx[j] - vx[i]) * dx + (vy[j] - vy[i]) * dy) / l;
    }
    return total;
  };

  // ------------------------------------------------------------ scenarios
  var FIG6E_TARGETS = [4, 5, 0, 5, 3, 0, 2, 3];

  function shuffle(rng, n) {
    var perm = [];
    for (var i = 0; i < n; i++) perm.push(i);
    for (i = n - 1; i > 0; i--) {
      var j = rng.randint(i + 1), tmp = perm[i];
      perm[i] = perm[j]; perm[j] = tmp;
    }
    return perm;
  }
  function makeGraph(kind, n, rng, k) {
    var targets, m;
    if (kind === "cycle" || kind === "regular") return cycleGraph(n);
    if (kind === "fig6e") {
      if (n !== 8) throw new Error("fig6e has 8 agents");
      return FIG6E_TARGETS.slice();
    }
    if (kind === "none") return new Array(n).fill(NONE);
    if (kind === "cycle_branches") {
      if (n < 3) throw new Error("cycle_branches needs n >= 3");
      if (k === undefined || k === null) k = 2 + rng.randint(n - 2);
      if (!(k >= 2 && k < n)) throw new Error("cycle length k must satisfy 2 <= k < n");
      var perm = shuffle(rng, n);
      targets = new Array(n).fill(NONE);
      for (m = 0; m < k; m++) targets[perm[m]] = m > 0 ? perm[m - 1] : perm[k - 1];
      for (m = k; m < n; m++) targets[perm[m]] = perm[rng.randint(m)];
      return targets;
    }
    if (kind === "intree") {
      var p = shuffle(rng, n);
      targets = new Array(n).fill(NONE);
      for (m = 1; m < n; m++) targets[p[m]] = p[rng.randint(m)];
      return targets;
    }
    throw new Error("unknown scenario kind " + kind);
  }
  function initialState(kind, n, seed, phi, o) {
    o = o || {};
    var width = o.width === undefined ? 20 : o.width;
    var height = o.height === undefined ? 20 : o.height;
    var heading = o.heading || "in_view";
    var rng = new Mulberry32(seed);
    var targets = makeGraph(kind, n, rng, o.k);
    var x = [], y = [], psi = [], i;
    if (kind === "regular") {
      var r = o.radius === undefined || o.radius === null ? 0.4 * Math.min(width, height) : o.radius;
      var b = o.beta === undefined || o.beta === null ? PI / n : o.beta;
      for (i = 0; i < n; i++) {
        var a = 2 * PI * i / n;
        x.push(0.5 * width + r * Math.cos(a));
        y.push(0.5 * height + r * Math.sin(a));
        psi.push(wrap(a - PI / n - 0.5 * PI + b));
      }
      return { x: x, y: y, psi: psi, targets: targets };
    }
    for (i = 0; i < n; i++) {
      x.push(width * rng.random());
      y.push(height * rng.random());
    }
    for (i = 0; i < n; i++) {
      var rr = rng.random(), j = targets[i];
      if (heading === "in_view" && j !== NONE) {
        var to = Math.atan2(y[j] - y[i], x[j] - x[i]);
        psi.push(wrap(to + (2 * rr - 1) * 0.9 * phi));
      } else {
        psi.push(wrap(-PI + 2 * PI * rr));
      }
    }
    return { x: x, y: y, psi: psi, targets: targets };
  }
  function makeSim(kind, n, seed, phi, o) {
    o = o || {};
    var s = initialState(kind, n, seed, phi, o);
    var opts = {};
    for (var key in o) if (o.hasOwnProperty(key)) opts[key] = o[key];
    opts.phi = phi; opts.seed = seed;
    return new Sim(s.x, s.y, s.psi, s.targets, opts);
  }

  // --------------------------------------------------------------- bounds
  var bounds = {
    thm4: function (n) {
      if (n === 2) return PI / 2;
      if (n === 3 || n === 4) return Math.acos((n - 1) / n);
      return PI / n;
    },
    lemma11: function (n) { return n === 2 ? PI / 2 : Math.acos((n - 2) / (n - 1)); },
    thm12: function (n) { return Math.min(PI / 5, PI / n); },
    thm13: function (n, vmin, vmax) {
      return Math.acos((n * vmax - vmin * (1 - Math.cos(PI / n))) / (n * vmax));
    },
    intree: function (n, vmin, vmax) {
      if (n === 2) return PI / 2;
      var c = (n - 2) * vmax / (vmax + (n - 2) * vmin);
      return c < 1 ? Math.acos(c) : null;
    },
    critical: function (n) { return PI / n; },
    sampled: function (n, omega, dt) { return PI / n + 0.5 * omega * dt; },
    regularVdot: function (n, beta) {
      return n === 2 ? -4 * Math.cos(beta) : -n * (Math.cos(beta) + Math.cos((n - 2) * PI / n + beta));
    }
  };

  var RWC = {
    NONE: NONE, Mulberry32: Mulberry32, wrap: wrap, dist: dist, bearing: bearing,
    dubinsStep: dubinsStep, cycleGraph: cycleGraph, mergeGraph: mergeGraph, findCycle: findCycle,
    Sim: Sim, makeGraph: makeGraph, initialState: initialState, makeSim: makeSim,
    FIG6E_TARGETS: FIG6E_TARGETS, bounds: bounds
  };
  if (typeof module !== "undefined" && module.exports) module.exports = RWC;
  else root.RWC = RWC;
})(typeof window !== "undefined" ? window : this);
