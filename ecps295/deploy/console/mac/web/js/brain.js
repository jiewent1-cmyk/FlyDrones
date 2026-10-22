// Brain panel, spike raster and motor panel, after FlyDrones Live (docs/live/app.js): the populations of the drone's
// connectome (MiniFly v4: FlyDrones' 850-neuron MiniFly + the ECPS cells LCb / LCn / LCv / MDN) placed like a fly head
// seen from the front; every neuron lights up when it spikes and its outgoing connections pulse. Live from the drone.

import { L } from "./i18n.js";

const ROLE_COL = ["57,255,136", "76,201,240", "255,77,141"]; // interneuron, input, output (FlyDrones)
// population -> [x, y, rx, ry] in panel fractions, left side (the right side is mirrored)
const LAYOUT = {
  "R1-R6": [0.05, 0.40, 0.03, 0.22], T4a: [0.12, 0.26, 0.025, 0.07], T4b: [0.12, 0.44, 0.025, 0.07], T4c: [0.17, 0.30, 0.025, 0.07],
  T4d: [0.17, 0.50, 0.025, 0.07], LPi_h: [0.225, 0.30, 0.02, 0.04], LPi_v: [0.225, 0.46, 0.02, 0.04], HS: [0.265, 0.27, 0.012, 0.03],
  VS: [0.265, 0.43, 0.012, 0.04], LPLC2: [0.25, 0.62, 0.03, 0.04], LC4: [0.29, 0.70, 0.02, 0.03], PVLP: [0.35, 0.55, 0.03, 0.05],
  PVLP_inh: [0.40, 0.64, 0.02, 0.03], LAL_inh: [0.41, 0.36, 0.025, 0.05], DNg02: [0.445, 0.80, 0.02, 0.05], DNp03: [0.465, 0.91, 0.008, 0.01],
  DNp01: [0.48, 0.955, 0.004, 0.004], haltere: [0.36, 0.95, 0.03, 0.02],
  // ECPS295 additions (ecps295/my_minifly.py v2-v4)
  LCb: [0.185, 0.66, 0.022, 0.05], LCn: [0.115, 0.68, 0.02, 0.05], LCv: [0.235, 0.84, 0.03, 0.035], MDN: [0.405, 0.885, 0.006, 0.006],
};
const POP_INFO = {
  "R1-R6": ["感光细胞：亮度", "Photoreceptors: brightness"], T4a: ["运动：由前向后", "Motion: front to back"], T4b: ["运动：由后向前", "Motion: back to front"],
  T4c: ["运动：向上", "Motion: upward"], T4d: ["运动：向下", "Motion: downward"],
  LPi_h: ["小叶板抑制细胞（水平）", "Lobula plate inhibitory cells (horizontal)"], LPi_v: ["小叶板抑制细胞（垂直）", "Lobula plate inhibitory cells (vertical)"],
  HS: ["水平系统：机体转动", "Horizontal system: body rotation"], VS: ["垂直系统：下沉 / 上升", "Vertical system: sinking / rising"],
  LPLC2: ["逼近检测", "Looming detector"], LC4: ["逼近速度", "Looming speed"], PVLP: ["逼近整合", "Looming integration"],
  PVLP_inh: ["决定扫视方向（左右竞争）", "Picks the saccade direction (left-right competition)"], LAL_inh: ["转向抑制", "Turn inhibition"],
  DNg02: ["翅拍幅度 → 油门与偏航", "Wing-stroke amplitude → throttle and yaw"], DNp03: ["逃避扫视", "Escape saccade"], DNp01: ["巨纤维：逃逸", "Giant fiber: escape"],
  haltere: ["平衡棒：转动感觉", "Haltere: rotation sense"],
  LCb: ["ECPS v2 · 正前方失去纹理（白墙）→ PVLP 刹车 / 扫视", "ECPS v2 · texture lost straight ahead (white wall) → PVLP brake / saccade"],
  LCn: ["ECPS v3 · 一侧更近（光流 + 前向 ToF）→ 对侧 HS 居中", "ECPS v3 · one side nearer (optic flow + front ToF) → opposite HS, centering"],
  LCv: ["ECPS v4 · 下方有障碍（下视测距 + 气压）→ MDN", "ECPS v4 · obstacle below (rangefinder + barometer) → MDN"],
  MDN: ["ECPS v4 · 月行者下行神经元 → 后退", "ECPS v4 · moonwalker descending neuron → retreat"],
};
const GROUP_OF = { "R1-R6": "R16", haltere: "HAL" };

export class BrainView {
  constructor(brainCanvas, rasterCanvas, motorCanvas, tip) {
    this.c = brainCanvas; this.g = brainCanvas.getContext("2d");
    this.rc = rasterCanvas; this.rg = rasterCanvas.getContext("2d");
    this.mc = motorCanvas; this.mg = motorCanvas.getContext("2d");
    this.tip = tip;
    this.topo = null;
    this.edgeLayer = null;
    this.pulses = [];
    this.hover = null;
    this.lastBrain = 0;
    this.spikeRate = 0;
    this.c.addEventListener("mousemove", (ev) => this.onMove(ev));
    this.c.addEventListener("mouseleave", () => { this.hover = null; this.tip.style.display = "none"; });
  }

  setTopo(topo) {
    this.topo = topo;
    const n = topo.n;
    this.nodes = new Float32Array(n * 2);
    this.act = new Float32Array(n);
    let s = 12345;
    const rnd = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    for (const [type, side, start, count] of topo.pops) {
      const L = LAYOUT[type] || [0.45, 0.2, 0.03, 0.03];
      for (let i = 0; i < count; i++) {
        const a = rnd() * Math.PI * 2, r = Math.sqrt(rnd());
        let x = L[0] + Math.cos(a) * r * L[2];
        const y = L[1] + Math.sin(a) * r * L[3];
        if (side === "R") x = 1 - x;
        this.nodes[(start + i) * 2] = x; this.nodes[(start + i) * 2 + 1] = y;
      }
    }
    this.role = new Uint8Array(n);
    for (const k of topo.inputs || []) for (const i of topo.groups[k] || []) this.role[i] = 1;
    for (const k of topo.outputs || []) for (const i of topo.groups[k] || []) this.role[i] = 2;
    const rank = { 2: 0, 0: 1, 1: 2 };
    const order = [...Array(n).keys()].sort((a, b) => rank[this.role[a]] - rank[this.role[b]] || a - b);
    this.rowOf = new Int32Array(n);
    order.forEach((i, row) => (this.rowOf[i] = row));
    // pre-rendered static layer: lobes + every connection, faint (green excitatory, pink inhibitory)
    const W = this.c.width, H = this.c.height;
    const e = document.createElement("canvas"); e.width = W; e.height = H;
    const g = e.getContext("2d");
    g.fillStyle = "rgba(76,201,240,0.035)";
    g.beginPath(); g.ellipse(W * 0.16, H * 0.47, W * 0.15, H * 0.35, 0, 0, Math.PI * 2); g.fill();
    g.beginPath(); g.ellipse(W * 0.84, H * 0.47, W * 0.15, H * 0.35, 0, 0, Math.PI * 2); g.fill();
    g.fillStyle = "rgba(57,255,136,0.028)"; g.beginPath(); g.ellipse(W * 0.5, H * 0.5, W * 0.2, H * 0.3, 0, 0, Math.PI * 2); g.fill();
    g.lineWidth = 1;
    const { indptr, indices, data } = topo;
    for (let pre = 0; pre < n; pre++) {
      for (let k = indptr[pre]; k < indptr[pre + 1]; k++) {
        const post = indices[k];
        g.strokeStyle = data[k] > 0 ? "rgba(57,255,136,0.035)" : "rgba(255,77,141,0.035)";
        g.beginPath(); g.moveTo(this.nodes[pre * 2] * W, this.nodes[pre * 2 + 1] * H); g.lineTo(this.nodes[post * 2] * W, this.nodes[post * 2 + 1] * H); g.stroke();
      }
    }
    this.edgeLayer = e;
    this.popCenters = topo.pops.map(([type, side, start, count]) => {
      let cx = 0, cy = 0;
      for (let i = start; i < start + count; i++) { cx += this.nodes[i * 2]; cy += this.nodes[i * 2 + 1]; }
      return { type, side, start, count, x: cx / count, y: cy / count };
    });
    this.rg.clearRect(0, 0, this.rc.width, this.rc.height);
  }

  onBrain(m) {
    if (!this.topo || m.n !== this.topo.n) return;
    const { counts } = m;
    const n = m.n;
    let total = 0;
    const fired = [];
    for (let i = 0; i < n; i++) {
      const c = counts[i];
      if (c) { this.act[i] = 1; total += c; fired.push(i); }
    }
    // a BRAIN message sums the spikes of every control tick since the previous one (1-10 ticks, link level)
    const ticks = this.lastTick != null && m.tick > this.lastTick ? Math.min(20, m.tick - this.lastTick) : 1;
    this.lastTick = m.tick;
    const dt = ticks * 0.05;
    this.spikeRate = 0.8 * this.spikeRate + 0.2 * (total / dt);
    // a sample of the neurons that fired send a pulse along their connections (inputs rarely: they fire constantly)
    let added = 0;
    for (const i of fired) {
      if (added > 120) break;
      if (this.role[i] !== 1 || Math.random() < 0.08) { this.pulses.push([i, 1]); added++; }
    }
    // raster: one column per BRAIN message (1-4 control ticks, depending on the link level)
    const g = this.rg, W = this.rc.width, H = this.rc.height, step = Math.max(3, Math.min(12, Math.round(dt / 0.05) * 3));
    g.globalCompositeOperation = "copy"; g.drawImage(this.rc, -step, 0);
    g.globalCompositeOperation = "source-over";
    for (const i of fired) {
      g.fillStyle = `rgb(${ROLE_COL[this.role[i]]})`;
      g.fillRect(W - step, (this.rowOf[i] / n) * H, 2, 1.6);
    }
    this.lastBrain = performance.now();
  }

  draw(dt, rates) {
    this.rates = rates;
    const g = this.g, W = this.c.width, H = this.c.height;
    g.globalCompositeOperation = "source-over";
    g.clearRect(0, 0, W, H);
    if (!this.topo) {
      g.fillStyle = "rgba(138,152,168,.8)"; g.font = "15px -apple-system, sans-serif"; g.textAlign = "center";
      g.fillText(L("等待机载程序发送连接组…", "Waiting for the companion's connectome…"), W / 2, H / 2);
      return;
    }
    g.drawImage(this.edgeLayer, 0, 0);
    g.font = "600 15px -apple-system, 'PingFang SC', sans-serif"; g.fillStyle = "rgba(138,152,168,.75)"; g.textAlign = "center";
    g.fillText(L("视叶 L", "Optic lobe L"), W * 0.16, H * 0.07); g.fillText(L("视叶 R", "Optic lobe R"), W * 0.84, H * 0.07);
    g.fillText(L("中央脑", "Central brain"), W * 0.5, H * 0.2); g.fillText(L("下行神经元", "Descending neurons"), W * 0.5, H * 0.72);
    const n = this.topo.n, nodes = this.nodes, act = this.act;
    const decay = Math.pow(0.72, dt / 0.05);
    for (let i = 0; i < n; i++) act[i] *= decay;
    const pd = Math.pow(0.5, dt / 0.05);
    for (const p of this.pulses) p[1] *= pd;
    while (this.pulses.length && this.pulses[0][1] < 0.1) this.pulses.shift();
    if (this.pulses.length > 5000) this.pulses.splice(0, this.pulses.length - 5000);
    g.globalCompositeOperation = "lighter";
    g.lineWidth = 1.2;
    const { indptr, indices, data } = this.topo;
    for (const [pre, life] of this.pulses) {
      const a = 0.22 * life;
      for (let k = indptr[pre]; k < indptr[pre + 1]; k++) {
        const post = indices[k];
        g.strokeStyle = data[k] > 0 ? `rgba(57,255,136,${a})` : `rgba(255,77,141,${a})`;
        g.beginPath(); g.moveTo(nodes[pre * 2] * W, nodes[pre * 2 + 1] * H); g.lineTo(nodes[post * 2] * W, nodes[post * 2 + 1] * H); g.stroke();
      }
    }
    const hov = this.hover;
    for (let i = 0; i < n; i++) {
      const a = act[i], x = nodes[i * 2] * W, y = nodes[i * 2 + 1] * H, col = ROLE_COL[this.role[i]];
      const big = this.role[i] === 2 ? 2.2 : 1;
      const lit = hov && i >= hov.start && i < hov.start + hov.count;
      g.fillStyle = `rgba(${col},${lit ? 0.9 : 0.25 + 0.75 * a})`;
      g.beginPath(); g.arc(x, y, (1.6 + 2.2 * a) * big, 0, Math.PI * 2); g.fill();
      if (a > 0.3) { g.fillStyle = `rgba(${col},${0.18 * a})`; g.beginPath(); g.arc(x, y, (7 + 6 * a) * big, 0, Math.PI * 2); g.fill(); }
    }
    g.globalCompositeOperation = "source-over";
    if (this.lastBrain && performance.now() - this.lastBrain > (this.level === 1 ? 2500 : 1500)) {
      g.fillStyle = "rgba(5,7,11,.6)"; g.fillRect(0, 0, W, H);
      g.fillStyle = this.level === 0 ? "#b9c4d0" : "#fab219"; g.font = "700 15px -apple-system, 'PingFang SC', sans-serif"; g.textAlign = "center";
      g.fillText(this.level === 0 ? L("带宽档位「极低」：大脑数据暂停，链路恢复后自动继续", "Bandwidth “very low”: brain paused, resumes with the link")
        : L("▲ 大脑数据中断", "▲ Brain data interrupted"), W / 2, H / 2);
    }
    // firing rates of the descending neurons beside them
    g.font = "700 13px ui-monospace, Menlo, monospace"; g.textAlign = "center";
    for (const [name, x, y] of [["DNg02", 0.445, 0.80], ["DNp03", 0.465, 0.91], ["DNp01", 0.48, 0.955], ["MDN", 0.405, 0.885]]) {
      for (const side of ["L", "R"]) {
        const hz = rates ? rates[`${name}_${side}`] : undefined;
        if (hz === undefined) continue;
        const xx = side === "L" ? x : 1 - x;
        g.fillStyle = hz > 5 ? "#ff8fb8" : "rgba(138,152,168,.8)";
        g.fillText(`${hz.toFixed(0)}`, (side === "L" ? xx - 0.1 : xx + 0.1) * W, y * H + 5);
      }
    }
  }

  onMove(ev) {
    if (!this.popCenters) return;
    const r = this.c.getBoundingClientRect();
    const x = ((ev.clientX - r.left) / r.width), y = ((ev.clientY - r.top) / r.height);
    let best = null, bd = 0.0035;
    for (const p of this.popCenters) {
      const d = (p.x - x) ** 2 + ((p.y - y) * 0.7) ** 2;
      if (d < bd) { bd = d; best = p; }
    }
    this.hover = best;
    if (!best) { this.tip.style.display = "none"; return; }
    const key = `${GROUP_OF[best.type] || best.type}_${best.side}`;
    const hz = this.rates && this.rates[key] !== undefined ? `${this.rates[key].toFixed(1)} Hz` : "—";
    const info = POP_INFO[best.type];
    this.tip.replaceChildren();
    const title = document.createElement("b"); title.textContent = `${best.type} ${best.side}`;
    this.tip.append(title, document.createTextNode(` · ${L(`${best.count} 个神经元`, `${best.count} neurons`)} · ${hz}\n${info ? L(...info) : ""}`));
    this.tip.style.display = "block";
    this.tip.style.left = `${Math.min(window.innerWidth - 290, ev.clientX + 14)}px`;
    this.tip.style.top = `${ev.clientY + 14}px`;
  }

  // ------------------------------------------------------------------ motor panel: DN rates and the commands
  drawMotor(t) {
    const g = this.mg, W = this.mc.width, H = this.mc.height;
    g.clearRect(0, 0, W, H);
    if (!t) return;
    const r = t.rates || {};
    const names = ["DNg02_L", "DNg02_R", "DNp03_L", "DNp03_R", "DNp01_L", "DNp01_R", "MDN_L", "MDN_R"];
    const span = 540, bw = span / names.length, top = 22, bh = 140;
    g.font = "600 15px ui-monospace, Menlo, monospace"; g.textAlign = "left"; g.fillStyle = "#8a98a8";
    g.fillText(L("下行神经元 Hz", "Descending neurons Hz"), 4, 15);
    names.forEach((nm, i) => {
      const hz = r[nm] || 0, v = Math.min(1, hz / 100), x = i * bw + 6, w = bw - 12, h = v * bh;
      const grad = g.createLinearGradient(0, top + bh, 0, top);
      grad.addColorStop(0, "#5a1a35"); grad.addColorStop(1, nm.includes("p01") ? "#ffb020" : nm.startsWith("MDN") ? "#c77dff" : "#ff4d8d");
      g.fillStyle = "rgba(255,255,255,.05)"; g.fillRect(x, top, w, bh);
      g.fillStyle = grad; g.fillRect(x, top + bh - h, w, h);
      g.textAlign = "center"; g.fillStyle = "#e8eef5"; g.font = "700 17px ui-monospace, Menlo, monospace";
      g.fillText(hz.toFixed(0), x + w / 2, Math.max(top + 18, top + bh - h - 6));
      g.fillStyle = "#8a98a8"; g.font = "600 14px ui-monospace, Menlo, monospace";
      g.fillText(nm.replace("_", " "), x + w / 2, top + bh + 22);
    });
    // commands: filled bar = sent to the FC (after console, governor, watchdogs, shield); white tick = the brain's own
    const sent = t.cmd || [0, 0, 0, 0, 0], braw = t.braw || [0, 0, 0, 0];
    const axes = [[L("前进", "Forward"), sent[0], braw[0]], [L("横移", "Lateral"), sent[1], null], [L("偏航", "Yaw"), sent[2], braw[1]], [L("升降", "Vertical"), sent[3], braw[2]]];
    const lx = 580, bx = 660, w = W - bx - 10, mid = bx + w / 2;
    g.textAlign = "left"; g.fillStyle = "#8a98a8"; g.font = "600 15px ui-monospace, Menlo, monospace";
    g.fillText(L("指令（填充 = 发给飞控，白线 = 大脑原始）", "Commands (bar = sent to FC, white = brain's own)"), lx, 15);
    axes.forEach(([label, v, b], i) => {
      const y = 28 + i * 40;
      g.fillStyle = "#b9c4d0"; g.font = "700 16px -apple-system, 'PingFang SC', sans-serif"; g.fillText(label, lx, y + 20);
      g.fillStyle = "rgba(255,255,255,.06)"; g.fillRect(bx, y + 4, w, 22);
      g.fillStyle = v >= 0 ? "#39ff88" : "#4cc9f0";
      g.fillRect(Math.min(mid, mid + (v * w) / 2), y + 4, Math.abs((v * w) / 2), 22);
      g.fillStyle = "rgba(232,238,245,.55)"; g.fillRect(mid - 1, y, 2, 30);
      if (b != null) { g.fillStyle = "#ffffff"; g.fillRect(mid + (b * w) / 2 - 1.5, y + 1, 3, 28); }  // the brain has no lateral axis
      g.fillStyle = "#e8eef5"; g.font = "600 13px ui-monospace, Menlo, monospace"; g.textAlign = "right";
      g.fillText((v >= 0 ? "+" : "") + v.toFixed(2), W - 12, y + 20); g.textAlign = "left";
    });
    g.font = "800 17px -apple-system, 'PingFang SC', sans-serif";
    const d = t.dec || {};
    const y = H - 10;
    if (d.ret) { g.fillStyle = "#c77dff"; g.fillText(L("MDN 后退", "MDN retreat"), lx, y); }
    else if (d.sacc) { g.fillStyle = "#ffb020"; g.fillText(L(`逃避扫视 · ${d.sacc > 0 ? "向右" : "向左"}`, `Escape saccade · ${d.sacc > 0 ? "right" : "left"}`), lx, y); }
    else { g.fillStyle = "#8a98a8"; g.font = "600 14px ui-monospace, Menlo, monospace"; g.fillText((t.note || "—").slice(0, 46), lx, y); }
  }
}

// what the fly sees and does, from the TICK message
export function behaviour(t) {
  const r = t.rates || {}, d = t.dec || {};
  const lcb = (r.LCb_L || 0) + (r.LCb_R || 0), lcn = (r.LCn_L || 0) + (r.LCn_R || 0), loom = (r.LPLC2_L || 0) + (r.LPLC2_R || 0);
  let name, chain;
  if (d.ret) { name = L("后退", "Retreat"); chain = L("LCv（下方有障碍）→ MDN 月行者 → 后退，然后扫视", "LCv (obstacle below) → MDN moonwalker → retreat, then saccade"); }
  else if (d.sacc) {
    name = L(`扫视 ${d.sacc > 0 ? "右" : "左"}`, `Saccade ${d.sacc > 0 ? "right" : "left"}`);
    chain = L("逼近 / 无纹理 → LPLC2 · LC4 · LCb → PVLP → DNp03 → 转开", "Looming / blank → LPLC2 · LC4 · LCb → PVLP → DNp03 → turn away");
  } else if (t.braw && t.braw[3]) { name = L("逃逸", "Escape"); chain = L("LPLC2 + LC4 → 巨纤维 DNp01", "LPLC2 + LC4 → giant fiber DNp01"); }
  else if (lcb > 30) { name = L("前方无纹理", "Blank ahead"); chain = L("白墙 → LCb → PVLP → 刹车", "White wall → LCb → PVLP → brake"); }
  else if ((d.brake || 0) > 0.2) { name = L("刹车", "Brake"); chain = L(`逼近 (LPLC2 ${loom.toFixed(0)} Hz) → PVLP → 刹车`, `Looming (LPLC2 ${loom.toFixed(0)} Hz) → PVLP → brake`); }
  else if (lcn > 15) { name = L("居中", "Centering"); chain = L("一侧更近 → LCn → 对侧 HS → DNg02 → 转向较空的一侧", "One side nearer → LCn → opposite HS → DNg02 → turn to the open side"); }
  else { name = L("巡航", "Cruise"); chain = L("光流 → T4/T5 → HS/VS → DNg02 → 平稳飞行", "Optic flow → T4/T5 → HS/VS → DNg02 → steady flight"); }
  return { name, chain };
}
