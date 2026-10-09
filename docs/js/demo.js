/* Demo UI for rendezvous without coordinates. Classic script; needs js/rwc-engine.js. */
(function () {
  "use strict";
  var RWC = window.RWC;
  var PI = Math.PI;

  // ------------------------------------------------------------- presets
  var BASE = { kind: "cycle", n: 8, phi: 0.35, omega: 2, dt: 0.1, rho: 0.05, noise: 0, vmin: 1,
               seed: 1, heading: "in_view", reassign: false, beta: null, width: 20, height: 20 };
  var PRESETS = [
    { id: "fig6a", label: "Fig. 6(a): 8 agents close in (φ = 0.35)",
      note: "φ is below π/8 ≈ 0.393, so V falls from about 84 to about 5. With ρ ≈ 0 merging is off (as in the paper): the agents end up chasing each other in a small cluster, where sampled control (Δt = 0.1) makes V wobble.",
      p: { n: 8, phi: 0.35, rho: 0.001, seed: 1 } },
    { id: "fig6c", label: "Fig. 6(c): 8 agents drift apart (φ = 0.6)",
      note: "φ is well above π/8: the agents arrange themselves on a near-regular octagon that keeps growing.",
      p: { n: 8, phi: 0.6, rho: 0.001, seed: 2 } },
    { id: "fig6e", label: "Fig. 6(e): cycle plus branches",
      note: "Agents 0, 4, 3, 5 chase each other in a cycle; 6 → 2 → 0, 7 → 3 and 1 → 5 hang off it. Merging collapses everything to one agent.",
      p: { kind: "fig6e", n: 8, phi: 0.35, omega: 8, rho: 0.3, seed: 1 } },
    { id: "fig1", label: "Fig. 1: seven agents merge",
      note: "Each agent merges into its target within ρ; the graph shrinks until one agent is left. ω = 8 > 2/ρ lets agents keep their targets at close range.",
      p: { n: 7, phi: 0.3, omega: 8, rho: 0.3, seed: 12 } },
    { id: "video02", label: "Video: 16 agents, φ = 0.20",
      note: "From the original applet's video. π/16 ≈ 0.196, yet the agents settle into a stable ring: with sampled control (Δt = 0.1) the practical threshold sits above π/n.",
      p: { n: 16, phi: 0.2, rho: 0.001, seed: 3 } },
    { id: "video04", label: "Video: 16 agents, φ = 0.40",
      note: "Twice π/16: the ring forms, then grows without bound.",
      p: { n: 16, phi: 0.4, rho: 0.001, seed: 3 } },
    { id: "lemma3", label: "Lemma 3: regular 9-gon on the edge",
      note: "Every target starts at bearing π/9 ≈ 0.349 < φ, so nobody turns: V\u0307 = 0 at first, then the polygon expands. Above π/n, V can grow. (Δt = 0.02, so the sampled threshold π/9 + ωΔt/2 ≈ 0.37 is well below φ.)",
      p: { kind: "regular", n: 9, phi: 0.45, dt: 0.02, rho: 0.001, seed: 0 } },
    { id: "intree", label: "Intree: everyone follows a standing root",
      note: "The root has no target and stays put. Lemma 11 guarantees V decreases for φ < arccos((n−2)/(n−1)).",
      p: { kind: "intree", n: 10, phi: 0.3, omega: 8, rho: 0.3, seed: 5 } },
    { id: "branches", label: "Random cycle plus branches (Thm. 12)",
      note: "The cycle collapses first, then the branches merge into it (sequential rendezvous).",
      p: { kind: "cycle_branches", n: 12, phi: 0.2, omega: 8, rho: 0.3, seed: 1 } },
    { id: "noise", label: "20% wrong sensor readings",
      note: "The paper reports the system still behaves as predicted when 20% of readings are wrong.",
      p: { n: 8, phi: 0.3, omega: 8, rho: 0.3, noise: 0.2, seed: 1 } },
    { id: "speeds", label: "Different speeds (Thm. 13)",
      note: "Speeds between v_min and v_max. The guarantee needs a narrower windshield than for identical agents.",
      p: { n: 8, phi: 0.25, rho: 0.3, vmin: 0.5, omega: 8, seed: 2 } },
    { id: "reassign", label: "No targets at all (Prop. 17)",
      note: "Nobody starts with a target. Agents without one turn until something enters the windshield, chase it, and merge when within ρ.",
      p: { kind: "none", n: 10, phi: 0.5, omega: 6, rho: 0.6, heading: "random", reassign: true, width: 10, height: 10, seed: 11 } }
  ];

  // ---------------------------------------------------------------- state
  var $ = function (id) { return document.getElementById(id); };
  var params = {};
  var sim = null, hist = null, trails = null, cam = null, playing = true, lastDone = false;
  var arena = $("arena"), plot = $("plot");
  var actx = arena.getContext("2d"), pctx = plot.getContext("2d");
  var colors = [];
  var view = { grid: true };

  function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
  function agentColor(i, n) {
    var h = Math.round((i * 360 / Math.max(n, 1) + 18) % 360);
    var dark = document.documentElement.dataset.theme === "dark" ||
      (!document.documentElement.dataset.theme && window.matchMedia("(prefers-color-scheme: dark)").matches);
    return "hsl(" + h + ", 72%, " + (dark ? 62 : 44) + "%)";
  }

  function speedsFor(p) {
    if (p.vmin >= 1) return 1.0;
    var r = new RWC.Mulberry32((p.seed + 977) >>> 0), s = [];
    for (var i = 0; i < p.n; i++) s.push(p.vmin + (1 - p.vmin) * r.random());
    s[0] = p.vmin; if (p.n > 1) s[1] = 1;   // make sure both extremes occur
    return s;
  }

  function build() {
    var p = params;
    var n = p.kind === "fig6e" ? 8 : p.n;
    if (p.kind === "cycle_branches" && n < 3) n = 3;
    params.n = n;
    sim = RWC.makeSim(p.kind, n, p.seed, p.phi, {
      omega: p.omega, dt: p.dt, rho: Math.max(p.rho, 1e-6), width: p.width, height: p.height,
      heading: p.heading, beta: p.beta, speeds: speedsFor(p), noise: p.noise,
      paper: p.paper, reassign: p.kind === "none" ? true : p.reassign
    });
    hist = { t: [0], V: [sim.V()], l: [sim.edgeLengths()], stride: 1, count: 0 };
    trails = [];
    for (var i = 0; i < n; i++) trails.push([[sim.x[i], sim.y[i]]]);
    colors = [];
    for (i = 0; i < n; i++) colors.push(agentColor(i, n));
    cam = null; lastDone = false;
    syncUI();
    draw();
  }

  // ------------------------------------------------------------- controls
  var sliders = {
    n: { fmt: function (v) { return String(v); }, int: true },
    phi: { fmt: function (v) { return v.toFixed(3) + "  (" + (v * params.n / PI).toFixed(2) + " π/n)"; } },
    omega: { fmt: function (v) { return v.toFixed(1); } },
    dt: { fmt: function (v) { return v.toFixed(2); } },
    rho: { fmt: function (v) { return v.toFixed(2); } },
    noise: { fmt: function (v) { return Math.round(v * 100) + "%"; } },
    vmin: { fmt: function (v) { return v.toFixed(2); } }
  };

  function boundFor(p) {
    var slow = p.vmin < 1;
    // bounded-speed intrees: the paper gives only Lemma 14 (merging for phi < pi/2, off the slider)
    if (p.kind === "intree") return slow ? PI / 2 : RWC.bounds.lemma11(p.n);
    if (slow) return RWC.bounds.thm13(p.n, p.vmin, 1);       // Thm. 13 (cycles), reused by Thm. 15
    if (p.kind === "cycle" || p.kind === "regular") return RWC.bounds.thm4(p.n);
    return RWC.bounds.thm12(p.n);
  }

  function syncUI() {
    Object.keys(sliders).forEach(function (k) {
      $(k).value = params[k];
      $(k + "-out").textContent = sliders[k].fmt(Number(params[k]));
    });
    $("kind").value = params.kind;
    $("seed").value = params.seed;
    $("paper").checked = !!params.paper;
    $("n").disabled = params.kind === "fig6e";
    var lo = Number($("phi").min), hi = Number($("phi").max);
    var place = function (el, v) {
      el.style.display = v !== null && v >= lo && v <= hi ? "block" : "none";
      el.style.left = (100 * (v - lo) / (hi - lo)) + "%";
    };
    place($("mark-pin"), PI / params.n);
    place($("mark-bound"), boundFor(params));
    // eq. (43) differs from our default only when some agent starts with its target out of view
    var unseen = sim && sim.seen.some(function (s, i) { return sim.targets[i] !== RWC.NONE && !s; });
    $("paper").disabled = !unseen && !params.paper;
    $("paper").parentNode.title = unseen || params.paper
      ? "Agents that start without their target in view keep going straight, as in eq. (43)"
      : "No effect here: every agent starts with its target in view";
  }

  function setPreset(id) {
    var pr = PRESETS.filter(function (q) { return q.id === id; })[0] || PRESETS[0];
    params = {};
    Object.keys(BASE).forEach(function (k) { params[k] = BASE[k]; });
    Object.keys(pr.p).forEach(function (k) { params[k] = pr.p[k]; });
    params.paper = false;
    $("preset").value = pr.id;
    $("preset-note").textContent = pr.note;
    try { history.replaceState(null, "", "#" + pr.id); } catch (e) { /* file:// may refuse */ }
    build();
  }

  function custom() {
    $("preset").value = "custom";
    $("preset-note").textContent = "Custom settings. Pick a scenario to restore it.";
    try { history.replaceState(null, "", location.pathname + location.search); } catch (e) { /* file:// */ }
    build();
  }

  function initControls() {
    var c = document.createElement("option");
    c.value = "custom"; c.textContent = "Custom"; c.disabled = true; c.hidden = true;
    $("preset").appendChild(c);
    PRESETS.forEach(function (pr) {
      var o = document.createElement("option");
      o.value = pr.id; o.textContent = pr.label;
      $("preset").appendChild(o);
    });
    $("preset").addEventListener("change", function () { setPreset(this.value); });
    Object.keys(sliders).forEach(function (k) {
      $(k).addEventListener("input", function () {
        params[k] = sliders[k].int ? parseInt(this.value, 10) : parseFloat(this.value);
        $(k + "-out").textContent = sliders[k].fmt(params[k]);
        custom();
      });
    });
    $("kind").addEventListener("change", function () {
      params.kind = this.value;
      params.reassign = this.value === "none";
      params.heading = this.value === "none" ? "random" : "in_view";
      params.beta = null;
      custom();
    });
    $("seed").addEventListener("change", function () {
      params.seed = Math.max(0, parseInt(this.value, 10) || 0); custom();
    });
    $("reseed").addEventListener("click", function () {
      params.seed = (params.seed * 7919 + 104729) % 1000000; custom();
    });
    $("paper").addEventListener("change", function () { params.paper = this.checked; custom(); });
    ["show-wind", "show-edges", "show-trails", "show-labels"].forEach(function (id) {
      $(id).addEventListener("change", draw);
    });
    $("play").addEventListener("click", togglePlay);
    $("stepbtn").addEventListener("click", function () { playing = false; $("play").textContent = "Play"; advance(1); draw(); });
    $("reset").addEventListener("click", build);
    $("theme").addEventListener("click", function () {
      var root = document.documentElement;
      var dark = root.dataset.theme ? root.dataset.theme === "dark" : window.matchMedia("(prefers-color-scheme: dark)").matches;
      root.dataset.theme = dark ? "light" : "dark";
      try { localStorage.setItem("rwc-theme", root.dataset.theme); } catch (e) { /* storage blocked */ }
      for (var i = 0; i < colors.length; i++) colors[i] = agentColor(i, colors.length);
      draw();
    });
    document.addEventListener("keydown", function (e) {
      var t = e.target;
      if (e.ctrlKey || e.metaKey || e.altKey) return;
      if (t && (t.isContentEditable || /^(INPUT|SELECT|TEXTAREA|BUTTON|SUMMARY|A)$/.test(t.tagName))) return;
      if (e.key === " ") { e.preventDefault(); togglePlay(); }
      else if (e.key === "r") build();
      else if (e.key === "s") { playing = false; $("play").textContent = "Play"; advance(1); draw(); }
    });
  }

  function togglePlay() {
    playing = !playing;
    $("play").textContent = playing ? "Pause" : "Play";
  }

  // ------------------------------------------------------------ stepping
  var MAX_PLOT = 1500, TRAIL = 400;
  function advance(k) {
    for (var s = 0; s < k; s++) {
      if (sim.done()) break;
      sim.step();
      for (var i = 0; i < sim.n; i++) {
        if (!sim.alive[i]) continue;
        var tr = trails[i];
        tr.push([sim.x[i], sim.y[i]]);
        if (tr.length > TRAIL) tr.shift();
      }
      hist.count++;
      if (hist.count % hist.stride === 0) {
        hist.t.push(sim.t); hist.V.push(sim.V()); hist.l.push(sim.edgeLengths());
        if (hist.t.length > MAX_PLOT) {
          var keep = function (a) { return a.filter(function (_, idx) { return idx % 2 === 0; }); };
          hist.t = keep(hist.t); hist.V = keep(hist.V); hist.l = keep(hist.l);
          hist.stride *= 2;
        }
      }
    }
  }

  // -------------------------------------------------------------- drawing
  function fit(canvas, ctx) {
    var r = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    var w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { w: r.width, h: r.height };
  }

  function targetCam(W, H) {
    var x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (var i = 0; i < sim.n; i++) {
      if (!sim.alive[i]) continue;
      x0 = Math.min(x0, sim.x[i]); x1 = Math.max(x1, sim.x[i]);
      y0 = Math.min(y0, sim.y[i]); y1 = Math.max(y1, sim.y[i]);
    }
    var span = Math.max(x1 - x0, (y1 - y0) * W / H, 4);
    span *= 1.25;
    return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, span: span, x0: x0, x1: x1, y0: y0, y1: y1 };
  }

  function draw() {
    drawArena();
    drawPlot();
    drawHud();
  }

  function drawArena() {
    var d = fit(arena, actx), W = d.w, H = d.h;
    var tc = targetCam(W, H);
    if (!cam) cam = tc;
    else {
      var a = 0.06;
      cam.cx += a * (tc.cx - cam.cx); cam.cy += a * (tc.cy - cam.cy);
      cam.span *= Math.pow(tc.span / cam.span, tc.span > cam.span ? 0.2 : a);
      // never let agents leave the view, however fast they drift (smoothing only within limits)
      cam.span = Math.max(cam.span, tc.span);
      var hw = cam.span / 2, hh = hw * H / W, m = 0.08 * cam.span;
      cam.cx = Math.min(Math.max(cam.cx, tc.x1 - hw + m), tc.x0 + hw - m);
      cam.cy = Math.min(Math.max(cam.cy, tc.y1 - hh + m), tc.y0 + hh - m);
    }
    var s = W / cam.span;
    var X = function (x) { return W / 2 + (x - cam.cx) * s; };
    var Y = function (y) { return H / 2 - (y - cam.cy) * s; };
    actx.clearRect(0, 0, W, H);
    actx.fillStyle = css("--arena"); actx.fillRect(0, 0, W, H);

    // grid with a nice step
    var raw = cam.span / 8, mag = Math.pow(10, Math.floor(Math.log10(raw)));
    var step = [1, 2, 5, 10].map(function (m) { return m * mag; }).filter(function (v) { return v >= raw; })[0];
    actx.strokeStyle = view.grid ? css("--grid") : "transparent"; actx.lineWidth = 1; actx.beginPath();
    var gx0 = Math.floor((cam.cx - cam.span) / step) * step;
    for (var gx = gx0; gx < cam.cx + cam.span; gx += step) { actx.moveTo(X(gx), 0); actx.lineTo(X(gx), H); }
    var gy0 = Math.floor((cam.cy - cam.span) / step) * step;
    for (var gy = gy0; gy < cam.cy + cam.span; gy += step) { actx.moveTo(0, Y(gy)); actx.lineTo(W, Y(gy)); }
    actx.stroke();

    var i, j;
    var size = Math.max(6, Math.min(11, 0.022 * Math.min(W, H))), R = 3.2 * size;
    if ($("show-trails").checked) {
      for (i = 0; i < sim.n; i++) {
        var tr = trails[i];
        if (tr.length < 2) continue;
        actx.strokeStyle = colors[i]; actx.globalAlpha = 0.45; actx.lineWidth = 1.3;
        actx.beginPath(); actx.moveTo(X(tr[0][0]), Y(tr[0][1]));
        for (var m = 1; m < tr.length; m++) actx.lineTo(X(tr[m][0]), Y(tr[m][1]));
        actx.stroke();
      }
      actx.globalAlpha = 1;
    }
        if ($("show-wind").checked) {
      actx.fillStyle = css("--wind");
      for (i = 0; i < sim.n; i++) {
        if (!sim.alive[i] || !sim.moving(i)) continue;
        var cx = X(sim.x[i]), cy = Y(sim.y[i]), h = -sim.psi[i];
        actx.beginPath(); actx.moveTo(cx, cy);
        actx.arc(cx, cy, R, h - sim.phi, h + sim.phi); actx.closePath(); actx.fill();
      }
    }
    if ($("show-edges").checked) {
      actx.strokeStyle = css("--edge"); actx.lineWidth = 1; actx.setLineDash([4, 4]);
      actx.beginPath();
      for (i = 0; i < sim.n; i++) {
        j = sim.targets[i];
        if (!sim.alive[i] || j < 0) continue;
        actx.moveTo(X(sim.x[i]), Y(sim.y[i])); actx.lineTo(X(sim.x[j]), Y(sim.y[j]));
      }
      actx.stroke(); actx.setLineDash([]);
    }
    for (i = 0; i < sim.n; i++) {
      if (!sim.alive[i]) continue;
      drawAgent(X(sim.x[i]), Y(sim.y[i]), sim.psi[i], size, colors[i], sim.targets[i] < 0 && !sim.moving(i));
    }
    if ($("show-labels").checked) {
      actx.fillStyle = css("--ink"); actx.font = "12px system-ui, sans-serif";
      for (i = 0; i < sim.n; i++) {
        if (!sim.alive[i]) continue;
        actx.fillText(String(i), X(sim.x[i]) + size + 2, Y(sim.y[i]) - size);
      }
    }
    // scale bar
    actx.fillStyle = css("--muted"); actx.strokeStyle = css("--muted"); actx.lineWidth = 2;
    var bx = W - 16 - step * s, by = H - 16;
    actx.beginPath(); actx.moveTo(bx, by); actx.lineTo(W - 16, by); actx.stroke();
    actx.font = "11px system-ui, sans-serif"; actx.textAlign = "right";
    actx.fillText("length " + fmtNum(step), W - 16, by - 6); actx.textAlign = "left";
  }

  function fmtNum(v) { return v >= 1 ? String(+v.toFixed(2)) : v.toPrecision(1); }

  function drawAgent(x, y, psi, s, color, stopped) {
    actx.save(); actx.translate(x, y); actx.rotate(-psi);
    actx.beginPath();
    actx.moveTo(s * 1.15, 0); actx.lineTo(-s * 0.8, s * 0.72); actx.lineTo(-s * 0.35, 0);
    actx.lineTo(-s * 0.8, -s * 0.72); actx.closePath();
    actx.fillStyle = color; actx.fill();
    actx.lineWidth = 1; actx.strokeStyle = css("--ink"); actx.globalAlpha = 0.6; actx.stroke();
    actx.globalAlpha = 1;
    if (stopped) { actx.beginPath(); actx.arc(0, 0, s * 1.4, 0, 2 * PI); actx.strokeStyle = color; actx.stroke(); }
    actx.restore();
  }

  function drawPlot() {
    var d = fit(plot, pctx), W = d.w, H = d.h;
    pctx.clearRect(0, 0, W, H);
    var L = 48, Rm = 10, T = 28, B = 22;
    var t = hist.t, V = hist.V;
    var tmax = Math.max(t[t.length - 1], 1e-9), vmax = 0;
    for (var k = 0; k < V.length; k++) vmax = Math.max(vmax, V[k]);
    vmax = vmax > 0 ? vmax * 1.08 : 1;
    var X = function (x) { return L + (W - L - Rm) * x / tmax; };
    var Y = function (y) { return T + (H - T - B) * (1 - y / vmax); };
    pctx.strokeStyle = css("--line"); pctx.lineWidth = 1;
    pctx.beginPath(); pctx.moveTo(L, T); pctx.lineTo(L, H - B); pctx.lineTo(W - Rm, H - B); pctx.stroke();
    pctx.fillStyle = css("--muted"); pctx.font = "11px system-ui, sans-serif";
    var top = vmax / 1.08;
    pctx.textAlign = "right";
    pctx.fillText(top >= 100 ? String(Math.round(top)) : fmtNum(top), L - 4, T + 8);
    pctx.fillText("0", L - 4, H - B); pctx.textAlign = "left";
    pctx.fillText("t = " + tmax.toFixed(1), W - Rm - 60, H - 6);
    var n = sim.n, i;
    for (i = 0; i < n; i++) {
      pctx.strokeStyle = colors[i]; pctx.globalAlpha = 0.75; pctx.lineWidth = 1; pctx.beginPath();
      var started = false;
      for (k = 0; k < t.length; k++) {
        var l = hist.l[k][i];
        if (l === null) { started = false; continue; }
        if (!started) { pctx.moveTo(X(t[k]), Y(l)); started = true; } else pctx.lineTo(X(t[k]), Y(l));
      }
      pctx.stroke();
    }
    pctx.globalAlpha = 1; pctx.strokeStyle = css("--vline"); pctx.lineWidth = 2.2; pctx.beginPath();
    for (k = 0; k < t.length; k++) { if (k === 0) pctx.moveTo(X(t[k]), Y(V[k])); else pctx.lineTo(X(t[k]), Y(V[k])); }
    pctx.stroke();
    pctx.fillStyle = css("--ink"); pctx.font = "600 12px system-ui, sans-serif";
    pctx.fillText("V = Σ distances to targets", L + 8, 18);
  }

  function drawHud() {
    var alive = sim.nAlive();
    var lines = [
      "t  " + sim.t.toFixed(1),
      "V  " + sim.V().toFixed(2) + "    dV/dt " + sim.Vdot().toFixed(2),
      "agents " + alive + "/" + sim.n + "    π/n " + (PI / sim.n).toFixed(3)
    ];
    lines = [lines.join("    ")];
    if (sim.done() && alive === 1) lines.push("rendezvous: one agent left");
    else if (sim.done()) lines.push("stopped: no agent can move");
    $("hud").textContent = lines.join("\n");
  }

  // ----------------------------------------------------------------- loop
  function frame() {
    if (playing && sim && !sim.done()) advance(Number($("rate").value));
    draw();
    window.requestAnimationFrame(frame);
  }

  function init() {
    try {
      var th = localStorage.getItem("rwc-theme");
      if (th === "light" || th === "dark") document.documentElement.dataset.theme = th;
    } catch (e) { /* storage blocked */ }
    initControls();
    var id = (location.hash || "").replace("#", "");
    setPreset(PRESETS.some(function (q) { return q.id === id; }) ? id : "fig6a");
    window.addEventListener("resize", draw);
    window.requestAnimationFrame(frame);
    window.__rwcDemo = { presets: PRESETS.map(function (q) { return q.id; }), setPreset: setPreset,
      advance: function (k) { advance(k); draw(); }, sim: function () { return sim; }, view: view, camera: function () { return cam; } };
  }
  init();
})();
