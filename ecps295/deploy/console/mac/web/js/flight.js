// Flight parameters: attitude indicator, number tiles, altitude and brain-tick charts, health checks (status colour
// + icon + label, never colour alone). Sources: the FC over MAVLink (state.fc) and the companion's TICK / STATUS.

import { TimeChart } from "./charts.js";
import { L } from "./i18n.js";

const ICON = { good: "●", warn: "▲", serious: "■", critical: "✕", off: "○" };
const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

export class FlightView {
  constructor(adi, tiles, health, chAlt, chTick) {
    this.adi = adi; this.ag = adi.getContext("2d");
    this.tiles = tiles; this.health = health;
    this.alt = new TimeChart(chAlt, { title: "", unit: "m", series: [
      { name: "EKF", color: css("--s1") }, { name: "", color: css("--s2") }, { name: "", color: css("--s3") }] });
    this.tick = new TimeChart(chTick, { title: "", unit: "ms", series: [{ name: "", color: css("--s1") }],
      min: 0, max: 60, refs: [{ v: 50, label: "", color: "rgba(208,59,59,.6)" }, { v: 25, label: "", color: "rgba(250,178,25,.5)" }] });
    this.ms = [];
    this.last = null;  // the arguments of the last render (a language switch draws them again)
    this.relabel();
  }

  relabel() {
    this.alt.setLabels(L("高度 m", "Altitude m"), ["EKF", L("下视测距", "Rangefinder"), L("路线B", "Route B")]);
    this.tick.setLabels(L("机载大脑每拍耗时 ms", "Onboard brain tick time ms"), [L("耗时", "tick")], [L("50 ms 周期", "50 ms period"), L("25 ms 目标", "25 ms target")]);
    if (this.last) this.render(...this.last);
  }

  static status(level, label, text) {
    return `<div title="${label}"><span class="st ${level}">${ICON[level]}</span><span class="t">${label}${text ? " · " + text : ""}</span></div>`;
  }

  static ekfAlt(fc, tick) {
    const lpn = fc.lpn || [];
    return lpn[2] != null ? -lpn[2] : (tick && tick.tel ? tick.tel[2] : null);
  }

  /** A new state (10 Hz): the charts and the brain tick-time window. */
  push(fc, tick) {
    fc = fc || {};
    const now = performance.now() / 1000;
    this.alt.push(now, [FlightView.ekfAlt(fc, tick), fc.rng, fc.viso_z]);
    if (tick && tick.ms != null) { this.tick.push(now, [tick.ms]); this.ms.push(tick.ms); if (this.ms.length > 400) this.ms.shift(); }
  }

  render(fc, tick, status, ecl, app) {
    this.last = [fc, tick, status, ecl, app];
    fc = fc || {};
    const lpn = fc.lpn || [], att = fc.att || [];
    const ekfAlt = FlightView.ekfAlt(fc, tick);
    const f = (v, d = 2) => (v == null || !Number.isFinite(v) ? "—" : v.toFixed(d));
    const hs = lpn[3] != null ? Math.hypot(lpn[3], lpn[4]) : null;
    const yaw = att[2] != null ? (att[2] + 360) % 360 : (tick && tick.tel && tick.tel[3] != null ? (tick.tel[3] + 360) % 360 : null);
    const p95 = this.ms.length ? [...this.ms].sort((a, b) => a - b)[Math.floor(0.95 * (this.ms.length - 1))] : null;
    const flow = fc.flow;
    const vib = fc.vib;
    const tiles = [
      [L("高度（EKF）", "Altitude (EKF)"), f(ekfAlt), "m"], [L("下视测距", "Rangefinder"), f(fc.rng), "m"],
      [L("路线B 高度（机载→飞控）", "Route B height"), f(fc.viso_z != null ? fc.viso_z : tick && tick.exth), "m"], [L("垂直速度", "Vertical speed"), f(lpn[5] != null ? -lpn[5] : null), "m/s"],
      [L("水平速度", "Horizontal speed"), f(hs), "m/s"], [L("航向", "Heading"), f(yaw, 0), "°"],
      [L("位置 N/E", "Position N/E"), lpn[0] != null ? `${f(lpn[0], 1)},${f(lpn[1], 1)}` : "—", "m"], [L("光流质量", "Flow quality"), flow ? `${flow[0]}` : "—", "/255"],
      [L("EKF 方差 位置/速度", "EKF var. pos/vel"), fc.ekf ? `${f(fc.ekf[2])}/${f(fc.ekf[1])}` : "—", ""], [L("振动 max", "Vibration max"), vib ? f(Math.max(vib[0], vib[1], vib[2]), 1) : "—", "m/s²"],
      [L("大脑每拍 p95", "Brain tick p95"), f(p95, 1), "ms"], [L("大脑速度", "Brain vs real time"), tick && tick.rtf != null ? f(tick.rtf, 1) : "—", L("× 实时", "×")],
    ];
    this.tiles.innerHTML = tiles.map(([k, v, u]) => `<div class="tile"><div class="v">${v}<small>${u}</small></div><div class="k">${k}</div></div>`).join("");
    this.drawADI(att[0], att[1], yaw);
    this.alt.draw(); this.tick.draw();

    // health
    const S = FlightView.status, H = [];
    const age = (a, good, warn) => (a == null ? "off" : a < good ? "good" : a < warn ? "warn" : "critical");
    H.push(S(age(fc.hb_age, 1.5, 3), L("飞控心跳", "FC heartbeat"), fc.hb_age != null ? `${fc.mode || ""}` : L("无", "none")));
    H.push(S(age(fc.comp_hb_age, 1.5, 2.5), L("机载心跳（失联保护）", "Companion heartbeat (failsafe)"), fc.comp_hb_age != null ? `${f(fc.comp_hb_age, 1)} s` : L("无", "none")));
    H.push(S(ecl && ecl.connected ? "good" : ecl && ecl.age != null ? "critical" : "off", L("机载遥测", "Telemetry"), ecl && ecl.rtt != null ? `RTT ${f(ecl.rtt, 0)} ms` : L("无", "none")));
    if (fc.ekf) {
      const fl = fc.ekf[0], ok = (fl & 0x0f) === 0x0f, cst = fl & 0x80, err = fl & 0xc00;
      H.push(S(err ? "critical" : cst ? "serious" : ok ? "good" : "warn", "EKF", err ? L("错误标志", "error flags") : cst ? L("常位置模式", "const-pos mode") : ok ? L("姿态/速度/位置正常", "att/vel/pos OK") : `${L("标志", "flags")} 0x${fl.toString(16)}`));
    } else H.push(S("off", "EKF", L("无数据", "no data")));
    H.push(S(fc.rng_age == null ? "off" : age(fc.rng_age, 0.5, 1.5), L("下视测距", "Rangefinder"), fc.rng != null ? `${f(fc.rng)} m` : L("无", "none")));
    H.push(S(fc.viso_age == null ? "off" : age(fc.viso_age, 0.5, 1.0), L("路线B 高度", "Route B height"), fc.viso_age != null ? L(`${f(fc.viso_age, 2)} s 前`, `${f(fc.viso_age, 2)} s ago`) : L("未发送", "not sent")));
    if (flow) H.push(S(flow[0] > 100 ? "good" : flow[0] > 50 ? "warn" : "serious", L("光流", "Optical flow"), `${L("质量", "quality")} ${flow[0]}`));
    else H.push(S("off", L("光流", "Optical flow"), L("无数据", "no data")));
    const ta = tick && tick.age ? tick.age : [null, null, null];
    H.push(S(ta[0] == null ? "off" : age(ta[0] / 1000, 0.3, 1.0), L("相机", "Camera"), ta[0] != null ? `${f(ta[0], 0)} ms` : "—"));
    H.push(S(ta[1] == null || ta[1] === "" ? "off" : age(ta[1] / 1000, 0.3, 1.0), L("ToF 前", "ToF front"), ta[1] !== "" && ta[1] != null ? `${f(ta[1], 0)} ms` : "—"));
    H.push(S(ta[2] == null || ta[2] === "" ? "off" : age(ta[2] / 1000, 0.3, 1.0), L("ToF 后", "ToF rear"), ta[2] !== "" && ta[2] != null ? `${f(ta[2], 0)} ms` : "—"));
    const temps = status && status.sys ? Object.entries(status.sys).filter(([k]) => k.startsWith("T_")).map(([, v]) => v) : [];
    const tmax = temps.length ? Math.max(...temps) : null;
    H.push(S(tmax == null ? "off" : tmax < 70 ? "good" : tmax < 80 ? "warn" : "critical", L("机载 CPU 温度", "Companion CPU temp"), tmax != null ? `${tmax.toFixed(0)} °C` : "—"));
    H.push(S(p95 == null ? "off" : p95 < 25 ? "good" : p95 < 50 ? "warn" : "serious", L("大脑实时性", "Brain real-time"), p95 != null ? `p95 ${f(p95, 1)} ms` : "—"));
    if (vib) H.push(S(Math.max(vib[0], vib[1], vib[2]) < 30 ? "good" : Math.max(vib[0], vib[1], vib[2]) < 60 ? "warn" : "critical", L("振动", "Vibration"), `${L("削顶", "clipping")} ${vib[3] + vib[4] + vib[5]}`));
    if (app && app.lag_ms != null) H.push(S(app.lag_ms < 100 ? "good" : app.lag_ms < 500 ? "warn" : "critical", L("控制台处理", "Console processing"), `${L("延迟", "lag")} ${f(app.lag_ms, 0)} ms`));
    if (fc.link) H.push(S(fc.link.loss_pct == null ? "off" : fc.link.loss_pct < 2 ? "good" : fc.link.loss_pct < 10 ? "warn" : "serious", L("MAVLink 丢包", "MAVLink loss"), fc.link.loss_pct != null ? `${fc.link.loss_pct}%` : "—"));
    this.health.innerHTML = H.join("");
  }

  drawADI(roll, pitch, yaw) {
    const g = this.ag, W = this.adi.width, R = W / 2;
    g.save(); g.clearRect(0, 0, W, W);
    g.beginPath(); g.arc(R, R, R - 2, 0, Math.PI * 2); g.clip();
    const rr = ((roll || 0) * Math.PI) / 180, pp = (pitch || 0) * (R / 45);
    g.translate(R, R); g.rotate(-rr);
    g.fillStyle = "#123a5c"; g.fillRect(-W, -W * 2 + pp, W * 2, W * 2);
    g.fillStyle = "#3a2a18"; g.fillRect(-W, pp, W * 2, W * 2);
    g.strokeStyle = "#e8eef5"; g.lineWidth = 2; g.beginPath(); g.moveTo(-W, pp); g.lineTo(W, pp); g.stroke();
    g.lineWidth = 1.5; g.fillStyle = "#e8eef5"; g.font = "16px ui-monospace, Menlo, monospace"; g.textAlign = "left";
    for (const d of [-20, -10, 10, 20]) {
      const y = pp - d * (R / 45), w = d % 20 === 0 ? 50 : 30;
      g.beginPath(); g.moveTo(-w, y); g.lineTo(w, y); g.stroke(); g.fillText(`${d}`, w + 4, y + 5);
    }
    g.restore();
    // fixed aircraft symbol + roll pointer
    g.strokeStyle = "#ffb020"; g.lineWidth = 5;
    g.beginPath(); g.moveTo(R - 70, R); g.lineTo(R - 25, R); g.lineTo(R - 12, R + 14); g.moveTo(R + 70, R); g.lineTo(R + 25, R); g.lineTo(R + 12, R + 14); g.stroke();
    g.beginPath(); g.arc(R, R, 4, 0, Math.PI * 2); g.fillStyle = "#ffb020"; g.fill();
    g.strokeStyle = "rgba(232,238,245,.7)"; g.lineWidth = 2; g.beginPath(); g.arc(R, R, R - 3, 0, Math.PI * 2); g.stroke();
    g.fillStyle = "rgba(5,7,11,.7)"; g.fillRect(R - 52, W - 46, 104, 32);
    g.fillStyle = "#e8eef5"; g.font = "700 22px ui-monospace, Menlo, monospace"; g.textAlign = "center";
    g.fillText(yaw == null ? "—" : `${yaw.toFixed(0).padStart(3, "0")}°`, R, W - 22);
    g.font = "15px ui-monospace, Menlo, monospace"; g.fillStyle = "#b9c4d0";
    g.fillText(`R ${roll == null ? "—" : roll.toFixed(0)}° P ${pitch == null ? "—" : pitch.toFixed(0)}°`, R, 30);
  }
}
