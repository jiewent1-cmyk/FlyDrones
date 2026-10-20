// ECPS Console UI: wires the data link to the panels and the operator's controls.
import { Link } from "./link.js";
import { BrainView, behaviour } from "./brain.js";
import { EyesView } from "./eyes.js";
import { MapView } from "./map3d.js";
import { FlightView } from "./flight.js";
import { L, lang, setLang, onLang, baseTitle } from "./i18n.js";

const $ = (id) => document.getElementById(id);
const link = new Link();
const brain = new BrainView($("brain"), $("raster"), $("motor"), $("tip"));
const eyes = new EyesView($("eyes"), $("camInfo"));
const flight = new FlightView($("adi"), $("tiles"), $("health"), $("chAlt"), $("chTick"));
const map = new MapView($("map3d"), $("mapwrap"), $("mapLegend"), onPick);

const MODES = { PATROL: ["大脑巡逻", "Brain patrol"], HOLD: ["悬停", "Hold"], GOTO: ["前往目标", "Go to goal"], MANUAL: ["手动", "Manual"], TRACK: ["跟踪目标", "Track target"], LAND: ["降落", "Land"] };
// target tracking (2026-10-18): the pilot's target layer states, and how the TICK reports them (trk, mk)
const TRK = { SEARCH: ["搜索中", "Searching"], APPROACH: ["接近中", "Approaching"], ARRIVED: ["已到达", "Arrived"], LOST: ["丢失", "Lost"], FAILED: ["没找到", "Not found"] };
const modeName = (m, raw = m) => (MODES[m] ? L(...MODES[m]) : raw);
const EKF_ORIGIN = [33.643, -117.842, 20.0];  // companion.yaml mavlink.ekf_origin (= the SITL home)
let S = null;          // native state (10 Hz)
let T = null;          // last companion TICK
let ST = null;         // last companion STATUS
let topo = null;       // companion TOPO (once per connection)
let mapHdr = null;     // header of the last MAP
let tickAt = 0;
let uiDown = false;    // the page lost its WebSocket to the app
let armedAt = null, lastArmed = false, lastFlight = 0;
let wantHandover = localStorage.getItem("ecps.handover") || "hold";
let handoverSent = false;
const seenEv = new Set();
let maxEv = 0;

// ---------------------------------------------------------------- link events
link.addEventListener("state", (e) => {
  S = e.detail;
  followLang(S.settings);
  const fc = S.fc || {};
  if (fc.texts) for (const t of fc.texts) addLog(t[2] <= 3 ? "critical" : t[2] <= 4 ? "warn" : "info", t[1], t[3]);
  flight.push(fc, freshTick());
  onState();
});
link.addEventListener("tick", (e) => { T = e.detail; tickAt = performance.now(); onTick(); });
link.addEventListener("status", (e) => { ST = e.detail; onStatus(); });
link.addEventListener("topo", (e) => { topo = e.detail; brain.setTopo(topo); topoText(); });
link.addEventListener("brain", (e) => { brain.onBrain(e.detail); eyes.onBrain(e.detail); });
link.addEventListener("cam", (e) => eyes.onCam(e.detail));
link.addEventListener("map", (e) => { map.onMap(e.detail); mapHdr = e.detail.hdr; mapText(); });
link.addEventListener("tof", (e) => map.onTof(e.detail));
link.addEventListener("ack", (e) => {
  const a = e.detail;
  if (!a.c || a.c === "stick" || a.c === "lang") return;
  const what = a.op || a.c;
  if (a.op === "handover" && !a.ok && hoAsked === "track") sendHandover("hold");  // TRACK refused here: the safe choice
  $("ackTxt").textContent = L(`${what}：${a.ok ? "✓ 已确认" : "✕ " + (a.msg || "失败")}`, `${what}: ${a.ok ? "✓ acknowledged" : "✕ " + (a.msg || "failed")}`);
  if (!a.ok || a.msg) addLog(a.ok ? "info" : "warn", L("控制台", "Console"), L(`${what}：${a.ok ? "已确认" : "失败"} ${a.msg || ""}`, `${what}: ${a.ok ? "acknowledged" : "failed"} ${a.msg || ""}`));
});
link.addEventListener("event", (e) => addLog(e.detail.level === "critical" ? "critical" : e.detail.level, e.detail.src || L("控制台", "Console"), e.detail.text));
link.addEventListener("open", () => { uiDown = false; natLang = null; addLog("info", L("控制台", "Console"), L("界面已连接到应用", "UI connected to the app")); });
link.addEventListener("close", () => { uiDown = true; $("live").classList.remove("on"); $("live").textContent = L("界面断开", "UI LINK LOST"); });

function topoText() {
  if (!topo) return;
  const name = topo.name.replace("minifly-ecps-", "MiniFly "), nnz = topo.nnz.toLocaleString();
  $("brainName").textContent = L(`${name} · ${topo.n} 神经元`, `${name} · ${topo.n} neurons`);
  $("brainName").title = L(`${topo.n} 个神经元，${nnz} 条连接（${topo.pops.length} 个细胞群）`, `${topo.n} neurons, ${nnz} connections (${topo.pops.length} cell populations)`);
}

function mapText() {
  if (!mapHdr) return;
  const h = mapHdr, nv = h.nvox.toLocaleString(), cm = (h.res * 100).toFixed(0);
  $("mapInfo").textContent = L(`· ${nv} 个占据体素（${cm} cm）· 第 ${h.frames} 帧 ToF`, `· ${nv} voxels (${cm} cm) · ${h.frames} ToF frames`);
}

// ---------------------------------------------------------------- state from the native side
function onState() {
  const fc = S.fc || {}, ecl = S.ecl || {}, sf = S.safety || {};
  const ok = ecl.connected || (fc.hb_age != null && fc.hb_age < 2);
  $("live").classList.toggle("on", !!ok);
  $("live").textContent = ok ? L("在线", "ONLINE") : L("离线", "OFFLINE");
  chips(fc, ecl);
  brain.level = ecl.connected ? ecl.level : null;
  eyes.level = ecl.connected ? ecl.level : null;
  authority(fc, ecl);
  battery(fc);
  armTimer(fc);
  estopBar(fc, sf);
  if (fc.sim && fc.sim[0] != null) {
    const n = (fc.sim[0] - EKF_ORIGIN[0]) * 111319.49, e = (fc.sim[1] - EKF_ORIGIN[1]) * 111319.49 * Math.cos((EKF_ORIGIN[0] * Math.PI) / 180);
    map.setTruth([n, e, fc.sim[2] - EKF_ORIGIN[2], fc.sim[3]]);
  } else map.setTruth(null);
  $("bRec").classList.toggle("on", !!(S.rec && S.rec.active));
  $("bRec").textContent = S.rec && S.rec.active ? L(`● 记录中 ${S.rec.MB} MB`, `● REC ${S.rec.MB} MB`) : L("● 记录", "● Record");
  $("fcLinkTxt").textContent = fc.link ? `MAVLink ${fc.link.host}:${fc.link.port} · ${fc.link.fps} ${L("帧/s", "msg/s")}` : "MAVLink —";
  if (!$("settings").classList.contains("on") && S.settings) fillSettings(S.settings);
  if (S.settings && S.settings.cam && S.settings.cam !== eyes.mode) setCam(S.settings.cam, false);  // the app's saved choice
  flight.render(fc, freshTick(), ST, ecl, S.app);
  dockState();
}

function chip(level, label, value) {
  const icon = { good: "●", warn: "▲", serious: "■", critical: "✕", off: "○" }[level];
  return `<div class="chip"><span class="st ${level}">${icon}</span>${label} <b>${value}</b></div>`;
}

function chips(fc, ecl) {
  const out = [];
  const LV = L("极低,低,中,高", "very low,low,mid,high").split(",");
  const lvTxt = ecl.level != null ? L(` · 带宽${ecl.auto ? "自动" : ""}${LV[ecl.level]}${ecl.forced ? "（急停）" : ""}`,
    ` · BW ${ecl.auto ? "auto " : ""}${LV[ecl.level]}${ecl.forced ? " (E-stop)" : ""}`) : "";
  const slow = ecl.rtt_max != null && ecl.rtt_max > 400;
  out.push(chip(ecl.connected ? (slow ? "warn" : "good") : ecl.age != null ? "critical" : "off", L("机载遥测", "Telemetry"),
    ecl.connected ? `${ecl.rtt != null ? ecl.rtt.toFixed(0) : "?"} ms${lvTxt}` : ecl.age != null ? L(`${ecl.age.toFixed(0)} s 无数据`, `no data for ${ecl.age.toFixed(0)} s`) : L("未连接", "not connected")));
  const hb = fc.hb_age;
  out.push(chip(hb == null ? "off" : hb < 1.5 ? "good" : hb < 3 ? "warn" : "critical", L("飞控", "FC"), hb == null ? L("无心跳", "no heartbeat") : `${fc.mode}${fc.armed ? L(" · 解锁", " · armed") : L(" · 上锁", " · disarmed")}`));
  const ch = fc.comp_hb_age;
  out.push(chip(ch == null ? "off" : ch < 1.5 ? "good" : "critical", L("机载心跳", "Companion HB"), ch == null ? "—" : `${ch.toFixed(1)} s`));
  if (fc.link && fc.link.loss_pct != null) out.push(chip(fc.link.loss_pct < 2 ? "good" : fc.link.loss_pct < 10 ? "warn" : "serious", L("丢包", "Loss"), `${fc.link.loss_pct}%`));
  $("chips").innerHTML = out.join("");
}

function freshTick() { return T && performance.now() - tickAt < 1500 ? T : null; }

function authority(fc, ecl) {
  const t = freshTick();
  let main = "—", sub = "";
  if (fc.hb_age == null || fc.hb_age > 3) {
    main = L("无飞控数据", "No FC data");
    sub = ecl.connected ? L("机载程序在线，飞控心跳缺失", "Companion online, no FC heartbeat") : L("检查 Pi 地址和端口", "Check the Pi address and ports");
  } else if (!fc.armed) {
    main = L("已上锁", "Disarmed");
    const ho = t ? modeName((t.ho || "").toUpperCase(), t.ho) : "";
    sub = t ? L(`交接后：${ho}`, `After hand-over: ${ho}`) : L("等待飞手解锁起飞", "Waiting for the pilot to arm and take off");
  } else if (fc.mode === "LAND") { main = L("降落中", "Landing"); sub = L("飞控 LAND", "FC in LAND"); }
  else if (t && t.ctl) {
    main = t.op === "PATROL" ? L("大脑驾驶", "Brain flying") : L(`控制台指引 · ${modeName(t.op)}`, `Console guidance · ${modeName(t.op)}`);
    sub = L("机载程序控制中（GUIDED）", "Companion in control (GUIDED)") + (t.guard ? " · " + t.guard : "");
  } else if (fc.mode === "GUIDED") {
    main = L("GUIDED · 未接管", "GUIDED · not in control");
    sub = t && t.es ? L("急停锁定中", "E-stop latched") : t && t.guard ? t.guard : L("机载程序未接管（高度 / 传感器 / 急停）", "Companion not in control (height / sensors / E-stop)");
  } else { main = L(`飞手 · ${fc.mode}`, `Pilot · ${fc.mode}`); sub = L("遥控器控制；拨到 GUIDED 交给机载程序", "RC in control; switch to GUIDED to hand over"); }
  $("authMain").textContent = main; $("authSub").textContent = sub;
}

function battery(fc) {
  const b = fc.bat, el = $("bat");
  if (!b || b[0] == null) { el.querySelector(".v").textContent = "— V"; el.className = "bat"; return; }
  const v = b[0], pct = b[2];
  const cells = v < 9 ? 2 : v < 13 ? 3 : 4, per = v / cells;
  const low = (pct != null && pct >= 0 && pct < 35) || per < 3.7, crit = (pct != null && pct >= 0 && pct < 25) || per < 3.55;
  el.className = "bat" + (crit ? " crit" : low ? " low" : "");
  el.querySelector(".v").textContent = `${v.toFixed(2)} V${pct != null && pct >= 0 ? ` ${pct}%` : ""}${crit ? " ✕" : low ? " ▲" : ""}`;
}

function armTimer(fc) {
  const now = performance.now() / 1000;
  if (fc.armed && !lastArmed) armedAt = now;
  if (!fc.armed && lastArmed && armedAt) lastFlight = now - armedAt;
  lastArmed = !!fc.armed;
  const s = fc.armed && armedAt ? now - armedAt : lastFlight;
  $("ftime").textContent = `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
}

function estopBar(fc, sf) {
  const bar = $("estopBar"), t = freshTick();
  const parts = [];
  let ok = true;
  if (sf.kill) {
    parts.push(sf.kill.confirmed ? L(`✓ 已停桨（飞控确认上锁，用时 ${sf.kill.after} s）`, `✓ Motors stopped (the FC confirmed the disarm after ${sf.kill.after} s)`)
      : L(`✕ 停桨中：强制上锁已发送 ${sf.kill.t} s，等待飞控确认…`, `✕ Kill: sending force disarm for ${sf.kill.t} s, waiting for the FC…`));
    ok = ok && sf.kill.confirmed;
  }
  if (sf.land) {
    let s = sf.land.confirmed ? L(`✓ 紧急降落：飞控已确认（${sf.land.after} s）`, `✓ Emergency land: confirmed by the FC (${sf.land.after} s)`)
      : L(`✕ 紧急降落：LAND 已发送 ${sf.land.t} s，等待飞控确认…`, `✕ Emergency land: sending LAND for ${sf.land.t} s, waiting for the FC…`);
    if (sf.land.confirmed && fc.armed && fc.mode !== "LAND") s += L(` · 飞手已改为 ${fc.mode}`, ` · the pilot switched to ${fc.mode}`);
    parts.push(s); ok = ok && sf.land.confirmed;
  }
  if (t && t.es) parts.push(L(`机载程序急停锁：${t.es === 2 ? "停桨" : "降落"}（解除前不会再接管）`, `Companion E-stop latch: ${t.es === 2 ? "kill" : "land"} (no take-over until reset)`));
  if (!parts.length) { bar.className = "estopBar"; bar.innerHTML = ""; return; }
  bar.className = "estopBar on" + (ok ? " ok" : "");
  bar.innerHTML = `<span>${parts.join(" · ")}</span>${!ok ? `<button class="icon" id="bStopSend">${L("停止重发", "Stop resending")}</button>` : ""}<button class="icon" id="bReset">${L("解除急停", "Reset E-stop")}</button>`;
  const r = $("bReset");
  if (r) r.onclick = () => {
    const armed = S && S.fc && S.fc.armed;
    const q = armed ? L("飞控仍处于解锁状态。强制解除急停锁？解除后机载程序可在 GUIDED 下重新接管（先悬停）。",
      "The FC is still armed. Force-reset the E-stop latch? The companion can then take control again in GUIDED (it starts with a hover).") : L("解除急停锁？", "Reset the E-stop latch?");
    if (confirm(q)) link.send({ c: "estop_reset", force: !!armed });
  };
  const st = $("bStopSend");
  if (st) st.onclick = () => link.send({ c: "estop_stop" });
}

// ---------------------------------------------------------------- companion TICK / STATUS
function onTick() {
  map.onTick(T);
  tickText();
  trkChip(T);
  if (!handoverSent && T.ho && T.ho.toLowerCase() !== wantHandover) { handoverSent = true; sendHandover(wantHandover); }
  if (T.ho) handoverSent = true;
}

function tickText() {
  const b = behaviour(T);
  const op = T.op;
  if (T.ctl && op !== "PATROL" && op !== "GOTO" && op !== "TRACK") { $("gname").innerHTML = `${modeName(op)}<small>${L("控制台指令替换了大脑的输出（大脑仍在运行）", "the console overrides the brain (still running)")}</small>`; }
  else $("gname").innerHTML = `${b.name}<small>${T.ctl ? (op === "GOTO" ? L("前往目标（航点层：大脑 + 目标偏置）", "to the goal: brain + waypoint bias")
    : op === "TRACK" ? L("跟踪目标（目标层：搜索、接近、面向标记，大脑的逃逸仍优先）", "tracking: search, approach, face the marker (the brain's escapes still win)")
    : L("大脑驾驶", "brain flying")) : L("未接管：仅显示大脑会怎么做", "not in control: what the brain would do")}</small>`;
  $("chain").textContent = b.chain + (T.illm ? L(` · 错觉：${T.illm}`, ` · illusion: ${T.illm}`) : "");
  $("tclock").textContent = `t ${T.t != null ? T.t.toFixed(1) : "—"} s`;
}

const SRC = { target: ["跟踪", "Tracking"], watchdog: ["看门狗", "Watchdog"], guard: ["传感器守护", "Sensor guard"], governor: ["安全调速", "Governor"], brain: ["大脑", "Brain"], operator: ["指引", "Guidance"], console: ["链路", "Link"] };
function onStatus() {
  if (ST.limits) map.setLimits(ST.limits);
  map.setTrack(!!ST.marker);
  for (const e of ST.events || []) {
    const [id, , src, text] = e;
    if (id < maxEv - 50) { seenEv.clear(); maxEv = 0; }  // a new companion run: ids restart
    if (seenEv.has(id)) continue;
    seenEv.add(id); maxEv = Math.max(maxEv, id);
    const lvl = /stop|LAND|lost|stale|refused|KILL|land/i.test(text) ? "warn" : "info";
    addLog(lvl, SRC[src] ? L(...SRC[src]) : src, zh(text));
  }
}

// ---------------------------------------------------------------- commands
function cmd(op, args = {}) { return link.send({ c: "cmd", op, ...args }); }
let hoAsked = "";  // the last hand-over mode asked for (a refused TRACK falls back to HOLD)
function sendHandover(mode) { hoAsked = mode; cmd("handover", { mode }); }

document.querySelectorAll("[data-op]").forEach((b) => (b.onclick = () => cmd(b.dataset.op)));
$("bLand").onclick = () => link.send({ c: "land" });
$("bEland").onclick = () => link.send({ c: "estop", level: 1 });
$("bHome").onclick = () => cmd("goto", { x: 0, y: 0 });
$("bManual").onclick = () => cmd("manual", { on: !(T && T.opm === "MANUAL") });
$("bTrack").onclick = () => cmd("track");
let picking = false;
$("bPick").onclick = () => setPicking(!picking);
function setPicking(on) {
  picking = on; map.setPicking(on);
  $("bPick").classList.toggle("on", on); $("pickHint").classList.toggle("on", on);
  if (!on) $("goalChip").classList.remove("on");
}
function onPick(n, e) {
  const lim = (ST && ST.limits && ST.limits.goto_limit_m) || 1.6;
  const r = Math.hypot(n, e), chipEl = $("goalChip");
  chipEl.classList.add("on");
  if (r > lim) { chipEl.innerHTML = `<span style="color:#ec835a">${L(`■ 超出目标点范围（${r.toFixed(2)} m > ${lim} m）`, `■ Outside the goal range (${r.toFixed(2)} m > ${lim} m)`)}</span>`; return; }
  chipEl.innerHTML = `${L("目标", "Goal")} N ${n.toFixed(2)} · E ${e.toFixed(2)} <button class="g" id="bGo">${L("✓ 前往", "✓ Go")}</button><button class="icon" id="bNo">✕</button>`;
  $("bGo").onclick = () => { cmd("goto", { x: +n.toFixed(2), y: +e.toFixed(2) }); setPicking(false); };
  $("bNo").onclick = () => setPicking(false);
}
document.querySelectorAll("[data-ho]").forEach((b) => (b.onclick = () => {
  wantHandover = b.dataset.ho; localStorage.setItem("ecps.handover", wantHandover); sendHandover(wantHandover);
}));
document.querySelectorAll("[data-view]").forEach((b) => (b.onclick = () => {
  document.querySelectorAll("[data-view]").forEach((x) => x.classList.toggle("on", x === b)); map.setView(b.dataset.view);
}));
const CAMS = { brain: ["大脑视角", "Brain view"], color: ["彩色", "Color"], off: ["关", "Off"] };
function setCam(mode, send = true) {
  document.querySelectorAll("[data-cam]").forEach((x) => x.classList.toggle("on", x.dataset.cam === mode));
  eyes.mode = mode;
  $("camLabel").textContent = CAMS[mode] ? L(...CAMS[mode]) : mode;
  if (send) link.send({ c: "cam", mode });
}
document.querySelectorAll("[data-cam]").forEach((b) => (b.onclick = () => setCam(b.dataset.cam)));

// kill: hold 1.5 s
(() => {
  const btn = $("bKill"), ring = $("killRing");
  let t0 = null, raf = 0;
  const stop = () => { t0 = null; cancelAnimationFrame(raf); ring.style.width = "0"; };
  const step = () => {
    if (t0 == null) return;
    const f = (performance.now() - t0) / 1500;
    ring.style.width = `${Math.min(100, f * 100)}%`;
    if (f >= 1) { stop(); link.send({ c: "estop", level: 2 }); addLog("critical", L("控制台", "Console"), L("停桨：已按住 1.5 s", "Kill: held for 1.5 s")); return; }
    raf = requestAnimationFrame(step);
  };
  btn.addEventListener("pointerdown", (e) => { e.preventDefault(); t0 = performance.now(); step(); });
  for (const ev of ["pointerup", "pointerleave", "pointercancel"]) btn.addEventListener(ev, stop);
})();

// illusions (FlyDrones gestures): held buttons, re-sent at 5 Hz
(() => {
  let held = null, timer = 0;
  const send = () => { if (held) cmd("gesture", { name: held }); };
  document.querySelectorAll("[data-g]").forEach((b) => {
    b.addEventListener("pointerdown", () => {
      held = b.dataset.g; b.classList.add("on"); send();
      clearInterval(timer); timer = setInterval(send, 200);
      if (held === "rush") { clearInterval(timer); setTimeout(() => { b.classList.remove("on"); held = null; }, 300); }
    });
    for (const ev of ["pointerup", "pointerleave"]) b.addEventListener(ev, () => {
      if (held && held !== "rush") { clearInterval(timer); cmd("gesture", { name: "none" }); }
      held = null; b.classList.remove("on");
    });
  });
})();

// keyboard: shortcuts + manual sticks (Esc is handled natively: emergency LAND)
const keys = new Set();
window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT" || e.metaKey || e.ctrlKey) return;
  const k = e.key.toLowerCase();
  const manual = T && T.opm === "MANUAL";
  if (manual && "wasdqerf".includes(k) && k.length === 1) { keys.add(k); e.preventDefault(); return; }
  if (e.repeat) return;
  if (k === "h") cmd("hold");
  else if (k === "p") cmd("patrol");
  else if (k === "g") setPicking(!picking);
  else if (k === "b") cmd("goto", { x: 0, y: 0 });
  else if (k === "m") $("bManual").onclick();
  else if (k === "t") cmd("track");
  else if (k === "l") link.send({ c: "land" });
});
window.addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));
window.addEventListener("blur", () => keys.clear());
setInterval(() => {
  if (!(T && T.opm === "MANUAL")) return;
  const a = (p, n) => (keys.has(p) ? 1 : 0) - (keys.has(n) ? 1 : 0);
  link.send({ c: "stick", f: a("w", "s"), l: a("e", "q"), y: a("d", "a"), z: a("r", "f") });
}, 100);

function dockState() {
  const t = freshTick(), ecl = (S && S.ecl) || {};
  const on = !!ecl.connected;
  const pose = t && t.tel && t.tel[0] != null;
  const ctl = !!(t && t.ctl);
  document.querySelectorAll("#dock button.g").forEach((b) => {
    b.disabled = !on || !ctl;
    b.title = baseTitle(b) + (on && !ctl ? L("（机载程序接管后可用：飞手起飞并拨到 GUIDED）", " (once the companion is in control: the pilot takes off and switches to GUIDED)") : "");
  });
  $("bPick").disabled = !on || !ctl || !pose; $("bHome").disabled = !on || !ctl || !pose;
  const mk = ST && ST.marker, det = !!(mk && mk.detector);  // marker: null = no target layer in this runtime
  $("bTrack").hidden = !mk;
  $("bTrack").disabled = !on || !ctl || !pose || !det;
  if (mk && !det) $("bTrack").title = baseTitle($("bTrack")) + L("（机载程序没有标记检测：hw 配置 marker.enabled、相机）", " (no marker detector on the companion: hw config marker.enabled, camera)");
  const hoT = document.querySelector('[data-ho="track"]');
  hoT.hidden = !mk; hoT.disabled = !det;
  $("bLand").disabled = false;
  const opm = t ? t.opm : null;
  document.querySelectorAll("[data-op]").forEach((b) => b.classList.toggle("on", ctl && opm === b.dataset.op.toUpperCase()));
  $("bManual").classList.toggle("on", ctl && opm === "MANUAL");
  $("bTrack").classList.toggle("on", ctl && opm === "TRACK");
  $("bPick").classList.toggle("on", picking || (ctl && opm === "GOTO"));
  document.querySelectorAll("[data-ho]").forEach((b) => b.classList.toggle("on", t && (t.ho || "").toLowerCase() === b.dataset.ho));
  const ho = t ? modeName((t.ho || "").toUpperCase(), t.ho) : "";
  $("modeNow").textContent = !t ? (on ? L("等待数据", "Waiting for data") : L("未连接", "Not connected")) : t.ctl ? modeName(t.opm) : L(`待交接 → ${ho}`, `Standby → ${ho}`);
  $("modeNote").textContent = t ? (t.ctl ? (t.opm === "TRACK" && t.trk ? trkText(t) : t.gnote || zh(t.note || "")) : L("飞手起飞后拨到 GUIDED 即交接", "Hand-over: switch to GUIDED")) : "";
  $("manualHint").textContent = opm === "MANUAL" ? L("手动：W/S 前后 · A/D 偏航 · Q/E 平移 · R/F 升降（松开 = 悬停）", "Manual: W/S fwd/back · A/D yaw · Q/E sideways · R/F up/down (release = hover)") : "";
  document.querySelectorAll("[data-g]").forEach((b) => (b.disabled = !(on && t && t.op === "PATROL")));
}

// ---------------------------------------------------------------- target tracking: state text and the map chip
function side(b) { return b >= 0 ? L(`右 ${b.toFixed(0)}°`, `${b.toFixed(0)}° right`) : L(`左 ${(-b).toFixed(0)}°`, `${(-b).toFixed(0)}° left`); }
function trkText(t) {
  const k = t.trk, m = t.mk;
  const seen = m ? L(`标记 ${m[0]} · ${m[1].toFixed(2)} m · ${side(m[2])}`, `marker ${m[0]} · ${m[1].toFixed(2)} m · ${side(m[2])}`) : "";
  if (!k) return seen ? L(`看到${seen}`, `In view: ${seen}`) : "";
  const ss = ST && ST.marker && ST.marker.search_s;
  const clock = k.st === "SEARCH" && k.ts != null ? ` ${Math.round(k.ts)}${ss ? "/" + Math.round(ss) : ""} s` : "";
  return `${TRK[k.st] ? L(...TRK[k.st]) : k.st}${clock}${seen ? " · " + seen : k.st === "SEARCH" ? "" : L(" · 标记不在视野内", " · marker not in view")}`;
}
function trkChip(t) {
  const el = $("trkChip"), s = t ? trkText(t) : "";
  el.classList.toggle("on", !!s);
  if (!s) return;
  const fx = t.trk && t.trk.fx ? L(` · 定位 ${t.trk.fx} 次`, ` · ${t.trk.fx} fixes`) : "";
  el.innerHTML = `<span class="dot">◆</span><span></span>`;
  el.lastChild.textContent = (t.trk ? L("跟踪 · ", "Track · ") : "") + s + fx;
}

// ---------------------------------------------------------------- settings
function fillSettings(s) {
  $("sHost").value = s.host; $("sEcl").value = s.ecl; $("sMav").value = s.mav;
  $("sStreams").checked = !!s.streams; $("sRecAuto").checked = !!s.rec_auto;
  $("sCamHz").value = String(Math.round(s.cam_hz || 5));
  $("sBw").value = String(s.bw == null ? -1 : s.bw);
}
$("bSettings").onclick = () => { if (S) fillSettings(S.settings); $("settings").classList.add("on"); };
$("bSetClose").onclick = () => $("settings").classList.remove("on");
$("bConnect").onclick = () => {
  link.send({ c: "connect", host: $("sHost").value.trim(), ecl: +$("sEcl").value, mav: +$("sMav").value });
  link.send({ c: "streams", on: $("sStreams").checked });
  link.send({ c: "rec", auto: $("sRecAuto").checked });
  link.send({ c: "cam", hz: +$("sCamHz").value, color_w: +$("sColorW").value });
  link.send({ c: "bw", lvl: +$("sBw").value });
  $("settings").classList.remove("on");
  seenEv.clear(); maxEv = 0; handoverSent = false;
  addLog("info", L("控制台", "Console"), L(`连接 ${$("sHost").value.trim()}`, `Connecting to ${$("sHost").value.trim()}`));
};
$("bRec").onclick = () => link.send({ c: "rec", on: !(S && S.rec && S.rec.active) });

// ---------------------------------------------------------------- language (the app keeps it: menu 显示 → 语言, or the header switch)
let natLang = null;  // the app's language as its last state reported it
function followLang(s) {
  if (!s || !s.lang || s.lang === natLang) return;
  natLang = s.lang;
  if (s.lang !== lang) setLang(s.lang);
}
document.querySelectorAll("[data-lang]").forEach((b) => (b.onclick = () => {
  if (b.dataset.lang === lang) return;
  setLang(b.dataset.lang);
  link.send({ c: "lang", lang: b.dataset.lang });
}));
// what the static switch in i18n.js does not reach: texts the scripts wrote, charts, the map legend
function relabel() {
  document.querySelectorAll("[data-lang]").forEach((b) => b.classList.toggle("on", b.dataset.lang === lang));
  topoText(); mapText(); setCam(eyes.mode, false); map.drawLegend(); flight.relabel();
  $("ackTxt").textContent = "";
  if (T) tickText();
  if (S && !uiDown) onState();
  if (uiDown) $("live").textContent = L("界面断开", "UI LINK LOST");
}
onLang(relabel);
relabel();

// ---------------------------------------------------------------- companion texts -> Chinese (in English they stay as sent)
const ZH = [
  [/^escape: ([\w ]+?)(, saccade (left|right))?$/, (m) => `逃逸触发：${{ brake: "刹车", retreat: "MDN 后退", blank: "无纹理", loom: "逼近" }[m[1]] || m[1]}${m[3] ? `，向${m[3] === "left" ? "左" : "右"}扫视` : ""}`],
  [/^mode (\w+)(?: \((.*)\))?$/, (m) => `模式 → ${modeName(m[1])}${m[2] ? `（${zh(m[2])}）` : ""}`],
  [/^hand-over mode (\w+)$/, (m) => `交接后模式 → ${modeName(m[1])}`],
  [/^hand-over$/, () => "交接"],
  [/^(.*) connected from (.*)$/, (m) => `${m[1]} 已连接（${m[2]}）`],
  [/^link lost$/, () => "控制台链路中断"], [/^link restored$/, () => "控制台链路恢复"],
  [/^console link lost$/, () => "控制台链路中断"],
  [/^EMERGENCY STOP: (LAND|KILL MOTORS) \((.*)\)$/, (m) => `急停：${m[1] === "LAND" ? "降落" : "停桨"}（${m[2]}）`],
  [/^emergency stop$/, () => "急停"],
  [/^emergency stop reset \((.*)\)$/, (m) => `急停锁已解除（${m[1]}）`],
  [/^camera stale -> (hover|land)$/, (m) => `相机数据中断 → ${m[1] === "hover" ? "悬停" : "降落"}`],
  [/^height source stale -> (hover|land)$/, (m) => `路线B 高度源中断 → ${m[1] === "hover" ? "悬停" : "降落"}`],
  [/^ToF (\w+) stale \((.*)\) -> (hover|land)$/, (m) => `ToF ${m[1] === "front" ? "前" : m[1] === "rear" ? "后" : m[1]} 中断 → ${m[3] === "hover" ? "悬停" : "降落"}`],
  [/^hand-over refused: (.*)$/, (m) => `拒绝交接：${zh(m[1])}`],
  [/^flight time (\d+) s -> hover, pilot takes over$/, (m) => `飞行 ${m[1]} s 到时 → 悬停，请飞手接管`],
  [/^waypoint (\d+) (reached|skipped)$/, (m) => `航点 ${m[1]} ${m[2] === "reached" ? "已到达" : "已跳过（无进展）"}`],
  [/^waypoints: done -> hover$/, () => "航点完成 → 悬停"],
  [/^goal (reached|skipped|done) \(EKF distance (.*) m\)$/, (m) => `目标${{ reached: "已到达", skipped: "未能到达（已跳过）", done: "完成" }[m[1]]}（EKF 距离 ${m[2]} m）`],
  [/^goal ([-+\d.]+), ([-+\d.]+)( via the waypoint layer)?$/, (m) => `目标 N ${m[1]} · E ${m[2]}${m[3] ? "（航点层）" : ""}`],
  [/^goto timeout$/, () => "前往超时"], [/^arrived \((.*)\)$/, (m) => `到达（${m[1]}）`],
  [/^illusion (\w+)$/, (m) => `错觉刺激：${{ palm: "手掌（爬升）", drop: "放下（下降）", left: "左转", right: "右转", rush: "逼近", fist: "静止" }[m[1]] || m[1]}`],
  [/^fence: turning back$/, () => "围栏：掉头"],
  [/^target search$/, () => "搜索目标（原地转动）"], [/^target not found$/, () => "没找到目标 → 悬停"], [/^target: facing$/, () => "面向目标悬停"],
  [/^target: (\w+)(?: \((.*)\))?$/, (m) => `目标跟踪：${{ search: "搜索", approach: "接近", arrived: "已到达", lost: "丢失", failed: "没找到", off: "停止" }[m[1]] || m[1]}${m[2] ? `（${trkWhy(m[2])}）` : ""}`],
  [/^tracking did not start$/, () => "跟踪没能启动"],
  [/^brain stalled (.*) -> hover$/, (m) => `大脑停顿 ${m[1]} → 悬停`],
  [/^console: (\w+)/, (m) => `控制台：${{ hold: "悬停", land: "降落", manual: "手动", goto: "前往", arrived: "到达" }[m[1]] || m[1]}`],
  [/^ceiling$/, () => "高度上限"], [/^floor$/, () => "高度下限"], [/^geofence$/, () => "围栏"],
  [/^battery low -> land$/, () => "电量低 → 降落"], [/^max flight time -> land$/, () => "最长飞行时间 → 降落"], [/^brain timeout$/, () => "大脑超时"],
  [/^camera stale/, () => "相机数据中断"], [/^no EKF position$/, () => "没有 EKF 位置"],
];
function trkWhy(w) {
  let m;
  if ((m = w.match(/^marker (\d+) at ([-+\d.]+), ([-+\d.]+), ([\d.]+) m$/))) return `标记 ${m[1]} 在 N ${m[2]} · E ${m[3]}，距离 ${m[4]} m`;
  if ((m = w.match(/^range ([\d.]+) m$/))) return `距离 ${m[1]} m`;
  if ((m = w.match(/^nothing in (\d+) s -> hover$/))) return `${m[1]} s 内都没看到 → 悬停`;
  return { "marker moved": "标记移动了", "flying on to the last goal": "飞向最后的目标点", "last goal reached": "已到最后的目标点",
    "not seen at the last fix": "最后位置没看到标记", "out of view": "标记离开视野" }[w] || w;
}
function zh(text) {
  if (lang === "en") return text;
  for (const [re, f] of ZH) { const m = String(text).match(re); if (m) return f(m); }
  return text;
}

// ---------------------------------------------------------------- event log
const logEl = $("log");
function addLog(level, src, text) {
  const d = new Date();
  const icon = { critical: "✕", warn: "▲", warning: "▲", info: "●", serious: "■" }[level] || "●";
  const cls = { critical: "critical", warn: "warn", warning: "warn", serious: "serious" }[level] || "good";
  const row = document.createElement("div");
  row.innerHTML = `<span class="tm">${d.toTimeString().slice(0, 8)}</span><span class="st ${cls}">${icon}</span><span class="src">${src}</span><span class="tx"></span>`;
  row.querySelector(".tx").textContent = text;
  logEl.prepend(row);
  while (logEl.childElementCount > 300) logEl.lastChild.remove();
}
$("bLogClear").onclick = () => (logEl.innerHTML = "");

// ---------------------------------------------------------------- render loop
let prev = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - prev) / 1000); prev = now;
  const t = freshTick();
  brain.draw(dt, t ? t.rates : null);
  brain.drawMotor(t);
  eyes.draw(t);
  map.render();
  const sr = brain.spikeRate;
  $("brainStat").textContent = t ? `${(sr / 1000).toFixed(1)}k ${L("脉冲/s", "spikes/s")} · ${t.rtf != null ? t.rtf.toFixed(0) : "?"}×` : "—";
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
addLog("info", L("控制台", "Console"), L("Esc = 紧急降落（任何时候）；停桨需按住 1.5 秒", "Esc = emergency land (any time); kill needs a 1.5 s hold"));
