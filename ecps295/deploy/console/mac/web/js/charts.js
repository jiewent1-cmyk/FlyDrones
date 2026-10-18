// Small rolling time-series chart: one y axis, 2 px lines, recessive grid, legend + last-value labels, crosshair
// tooltip on hover. Series colours are the validated dark categorical slots (--s1..--s3).

export class TimeChart {
  constructor(el, { title, unit, series, window_s = 30, min = null, max = null, refs = [] }) {
    this.el = el; this.title = title; this.unit = unit; this.series = series; this.win = window_s;
    this.min = min; this.max = max; this.refs = refs;
    this.data = series.map(() => []);
    el.innerHTML = `<canvas></canvas><div class="ttl"></div><div class="lg"></div><div class="tt"></div>`;
    this.c = el.querySelector("canvas"); this.g = this.c.getContext("2d"); this.tt = el.querySelector(".tt");
    this.setLabels(title, series.map((s) => s.name), refs.map((r) => r.label));
    this.hx = null;
    this.c.addEventListener("mousemove", (e) => { const r = this.c.getBoundingClientRect(); this.hx = e.clientX - r.left; this.draw(true); });
    this.c.addEventListener("mouseleave", () => { this.hx = null; this.tt.style.display = "none"; this.draw(true); });
    this.last = 0;
  }

  /** Title, series names and reference-line labels (again after a language switch). */
  setLabels(title, names = [], refLabels = []) {
    this.title = title;
    names.forEach((n, i) => { if (this.series[i]) this.series[i].name = n; });
    refLabels.forEach((l, i) => { if (this.refs[i]) this.refs[i].label = l; });
    this.el.querySelector(".ttl").textContent = title;
    this.el.querySelector(".lg").innerHTML = this.series.length > 1
      ? this.series.map((s) => `<span><i style="background:${s.color}"></i>${s.name}</span>`).join("") : "";
    if (this.now != null) this.draw(true);
  }

  push(t, values) {
    values.forEach((v, i) => {
      const a = this.data[i];
      if (v == null || !Number.isFinite(v)) { if (a.length && a[a.length - 1][1] !== null) a.push([t, null]); return; }
      a.push([t, v]);
    });
    for (const a of this.data) while (a.length && a[0][0] < t - this.win - 1) a.shift();
    this.now = t;
  }

  draw(force = false) {
    const nowMs = performance.now();
    if (!force && nowMs - this.last < 100) return;
    this.last = nowMs;
    const dpr = window.devicePixelRatio || 1, W = this.el.clientWidth, H = this.el.clientHeight;
    if (!W || !H) return;
    if (this.c.width !== Math.round(W * dpr)) { this.c.width = Math.round(W * dpr); this.c.height = Math.round(H * dpr); }
    const g = this.g;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    const L = 34, R = 44, T = 18, B = 14, pw = W - L - R, ph = H - T - B;
    const t1 = this.now || 0, t0 = t1 - this.win;
    let lo = this.min, hi = this.max;
    if (lo == null || hi == null) {
      let a = Infinity, b = -Infinity;
      for (const s of this.data) for (const [, v] of s) if (v != null) { a = Math.min(a, v); b = Math.max(b, v); }
      for (const r of this.refs) { a = Math.min(a, r.v); b = Math.max(b, r.v); }
      if (!Number.isFinite(a)) { a = 0; b = 1; }
      if (b - a < 1e-3) { a -= 0.5; b += 0.5; }
      const pad = (b - a) * 0.12;
      lo = lo == null ? a - pad : lo; hi = hi == null ? b + pad : hi;
    }
    const X = (t) => L + ((t - t0) / this.win) * pw, Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;
    // grid + axis labels (recessive)
    g.strokeStyle = "rgba(120,160,200,.10)"; g.lineWidth = 1; g.fillStyle = "#8a98a8"; g.font = "10px ui-monospace, Menlo, monospace"; g.textAlign = "right";
    for (let i = 0; i <= 3; i++) {
      const v = lo + ((hi - lo) * i) / 3, y = Y(v);
      g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
      g.fillText(v.toFixed(Math.abs(hi - lo) < 3 ? 2 : 0), L - 4, y + 3);
    }
    g.textAlign = "center";
    for (let s = 0; s <= this.win; s += 10) g.fillText(s === 0 ? "now" : `-${s}s`, X(t1 - s), H - 2);
    for (const r of this.refs) {
      g.strokeStyle = r.color || "rgba(208,59,59,.7)"; g.setLineDash([4, 3]);
      g.beginPath(); g.moveTo(L, Y(r.v)); g.lineTo(L + pw, Y(r.v)); g.stroke(); g.setLineDash([]);
      g.fillStyle = "#b9c4d0"; g.textAlign = "right"; g.fillText(r.label, L + pw - 2, Y(r.v) - 3);
    }
    // series
    g.lineWidth = 2; g.lineJoin = "round";
    this.series.forEach((s, i) => {
      g.strokeStyle = s.color; g.beginPath();
      let pen = false, lastV = null;
      for (const [t, v] of this.data[i]) {
        if (t < t0) continue;
        if (v == null) { pen = false; continue; }
        const x = X(t), y = Y(Math.max(lo, Math.min(hi, v)));
        if (pen) g.lineTo(x, y); else { g.moveTo(x, y); pen = true; }
        lastV = v;
      }
      g.stroke();
      if (lastV != null) {  // direct label: last value at the line end
        g.fillStyle = "#e8eef5"; g.textAlign = "left"; g.font = "10px ui-monospace, Menlo, monospace";
        g.fillText(`${lastV.toFixed(2)}`, L + pw + 4, Y(Math.max(lo, Math.min(hi, lastV))) + 3 + i * 0);
      }
    });
    // crosshair + tooltip
    if (this.hx != null && this.hx > L && this.hx < L + pw) {
      const t = t0 + ((this.hx - L) / pw) * this.win;
      g.strokeStyle = "rgba(232,238,245,.5)"; g.beginPath(); g.moveTo(this.hx, T); g.lineTo(this.hx, T + ph); g.stroke();
      const rows = this.series.map((s, i) => {
        let best = null;
        for (const p of this.data[i]) if (p[1] != null && (!best || Math.abs(p[0] - t) < Math.abs(best[0] - t))) best = p;
        return best && Math.abs(best[0] - t) < 1 ? `${s.name} ${best[1].toFixed(3)} ${this.unit}` : `${s.name} —`;
      });
      this.tt.innerHTML = `${(t - t1).toFixed(1)} s<br>${rows.join("<br>")}`;
      this.tt.style.display = "block";
      this.tt.style.left = `${Math.min(W - 150, this.hx + 8)}px`; this.tt.style.top = `${T}px`;
    }
  }
}
