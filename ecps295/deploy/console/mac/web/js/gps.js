// GPS map page (2026-11-25): the live GPS fix (GPS_RAW_INT), the FC's flow position (EKF origin + LOCAL_POSITION_NED),
// the home point (HOME_POSITION) and the FC's mission (read-only MAVLink mission protocol).
//
// Two different positions, both shown and clearly labelled: the GPS fix is the raw u-blox M10 fix, the FC's own
// position is the dead-reckoned flow position (the EKF origin the companion sets plus the optical-flow offset) -
// the EKF does not fuse the GPS (EK3_SRC1_POSXY 0), so they must not be mixed up.
//
// Leaflet is vendored (vendor/leaflet): at the field there may be no network, then the tiles fail and the page
// falls back to a metric grid with a scale bar. The page never writes to the FC (no mode change, no mission upload).

import { L } from "./i18n.js";

// Leaflet is vendored and loaded as a classic script (vendor/leaflet/leaflet.js) before this module.
// The i18n helper is called L as well, so the map library is used through its own name.
const LM = globalThis.L;

const TRACK_MIN = 10;             // minutes of track kept
const DEFAULT_VIEW = { lat: 33.6430, lon: -117.8420, zoom: 18 };   // the companion's EKF origin (fallback)
const FIX_ZH = ["无定位", "单点定位", "二维定位", "三维定位"];
const FIX_EN = ["No fix", "Fix", "2D fix", "3D fix"];
const BASE_ZH = { osm: "街道图 (OSM)", sat: "卫星图 (Esri)", none: "无底图（公制网格）" };
const BASE_EN = { osm: "Street (OSM)", sat: "Satellite (Esri)", none: "No tiles (metric grid)" };

// ------------------------------------------------------------------ small geo helpers
const toRad = Math.PI / 180;
function distM(a, b) {                       // metres between two [lat, lon]
  if (!a || !b) return null;
  const R = 6371008.8;
  const dLat = (b[0] - a[0]) * toRad, dLon = (b[1] - a[1]) * toRad;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(a[0] * toRad) * Math.cos(b[0] * toRad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.min(1, Math.sqrt(h)));
}
function bearingDeg(a, b) {                  // degrees from north
  if (!a || !b) return null;
  const dLon = (b[1] - a[1]) * toRad, la1 = a[0] * toRad, la2 = b[0] * toRad;
  const y = Math.sin(dLon) * Math.cos(la2);
  const x = Math.cos(la1) * Math.sin(la2) - Math.sin(la1) * Math.cos(la2) * Math.cos(dLon);
  return (Math.atan2(y, x) / toRad + 360) % 360;
}
function niceStep(mpp) {                    // a round grid spacing (m) for the given metres per pixel
  for (const s of [0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000]) {
    if (s / mpp >= 70) return s;
  }
  return 100000;
}

// ------------------------------------------------------------------ the metric grid (no tiles)
const MetricGrid = LM.Layer.extend({
  onAdd(map) {
    this._map = map;
    this._group = LM.layerGroup().addTo(map);
    map.on("moveend zoomend resize", this._redraw, this);
    this._redraw();
  },
  onRemove(map) {
    map.off("moveend zoomend resize", this._redraw, this);
    this._group.remove();
    this._group = null;
  },
  _redraw() {
    if (!this._group) return;
    const map = this._map, size = map.getSize();
    this._group.clearLayers();
    if (!size.x || !size.y) return;
    const nw = map.containerPointToLatLng(LM.point(0, 0));
    const ne = map.containerPointToLatLng(LM.point(size.x, 0));
    const mpp = map.distance(nw, ne) / size.x;       // metres per pixel
    if (!isFinite(mpp) || mpp <= 0) return;
    const step = niceStep(mpp);
    const stepPx = step / mpp;
    const label = (t) => LM.divIcon({ className: "gridLabel", html: t, iconSize: null });
    const n = Math.ceil(size.x / stepPx) + 1, m = Math.ceil(size.y / stepPx) + 1;
    for (let k = -n; k <= n; k++) {                  // north-south lines: the east offset
      const x = size.x / 2 + k * stepPx;
      if (x < 0 || x > size.x) continue;
      const a = map.containerPointToLatLng(LM.point(x, 0)), b = map.containerPointToLatLng(LM.point(x, size.y));
      this._group.addLayer(LM.polyline([a, b], { color: "rgba(120,160,200,.30)", weight: 1, interactive: false }));
      const off = Math.round(k * step);
      this._group.addLayer(LM.marker(map.containerPointToLatLng(LM.point(x, 6)),
        { interactive: false, icon: label(`${off >= 0 ? "+" : ""}${off} m`) }));
    }
    for (let k = -m; k <= m; k++) {                  // east-west lines: the north offset
      const y = size.y / 2 + k * stepPx;
      if (y < 0 || y > size.y) continue;
      const a = map.containerPointToLatLng(LM.point(0, y)), b = map.containerPointToLatLng(LM.point(size.x, y));
      this._group.addLayer(LM.polyline([a, b], { color: "rgba(120,160,200,.30)", weight: 1, interactive: false }));
      const off = Math.round(k * step);
      this._group.addLayer(LM.marker(map.containerPointToLatLng(LM.point(6, y)),
        { interactive: false, icon: label(`${off >= 0 ? "+" : ""}${off} m`) }));
    }
    this.step = step;
  },
});

// ------------------------------------------------------------------ the page
export class GpsView {
  constructor(el, link, side) {
    this.el = el; this.link = link; this.side = side;
    this.fc = {};
    this.follow = true;
    this.showTracks = true;
    this.showMission = true;
    this.gpsTrack = []; this.fcTrack = [];
    this.base = "osm";
    this.tileOK = 0; this.tileBad = 0; this.baseT0 = performance.now();
    this.lastPanel = 0;
    this.visible = false;
    this.map = null;         // built when the page is shown the first time (a hidden container has no size)
    this.wire();
    this.relabel();
  }

  /** The map, the layers and the markers: built once, when the page becomes visible. */
  build() {
    const el = this.el;
    // fadeAnimation: false - Leaflet fades the tiles in with requestAnimationFrame, and WebKit throttles the
    // animation frames of a window that is not in the front: the tiles would stay at opacity 0 (a black map)
    // until the operator looks at the console again.
    this.map = LM.map(el, { zoomControl: true, attributionControl: true, minZoom: 2, maxZoom: 19, fadeAnimation: false });
    this.map.setView([DEFAULT_VIEW.lat, DEFAULT_VIEW.lon], DEFAULT_VIEW.zoom);
    this.bases = {
      osm: LM.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
        maxZoom: 19, fadeAnimation: false,
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors' }),
      sat: LM.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
        maxZoom: 19, fadeAnimation: false,
        attribution: "Imagery &copy; Esri, Maxar, Earthstar Geographics" }),
    };
    for (const k of Object.keys(this.bases)) {
      const layer = this.bases[k];
      layer.on("tileload", () => { this.tileOK++; });
      layer.on("tileloaderror", () => { this.tileBad++; });
    }
    this.grid = new MetricGrid();
    LM.control.scale({ metric: true, imperial: false, maxWidth: 130 }).addTo(this.map);

    // markers
    this.gpsMark = LM.circleMarker([0, 0], { radius: 8, color: "#39ff88", weight: 2, fillColor: "#39ff88", fillOpacity: .25, interactive: false }).addTo(this.map);
    this.gpsAcc = LM.circle([0, 0], { radius: 10, color: "#39ff88", weight: 1, fillColor: "#39ff88", fillOpacity: .08, interactive: false }).addTo(this.map);
    this.fcMark = LM.marker([0, 0], {
      interactive: false,
      icon: LM.divIcon({ className: "fcIcon", html: '<div class="fcArrow"></div>', iconSize: [18, 20], iconAnchor: [9, 0] })
    }).addTo(this.map);
    this.homeMark = LM.marker([0, 0], { interactive: false, icon: LM.divIcon({ className: "marker home", html: "H", iconSize: [18, 18], iconAnchor: [9, 9] }) }).addTo(this.map);
    this.gpsLine = LM.polyline([], { color: "#39ff88", weight: 2, opacity: .85, interactive: false }).addTo(this.map);
    this.fcLine = LM.polyline([], { color: "#ff4d8d", weight: 2, opacity: .8, interactive: false, dashArray: "4 3" }).addTo(this.map);
    this.missionGroup = LM.layerGroup().addTo(this.map);
    this.setBase(this.base);
  }

  // ------------------------------------------------------------------ controls
  wire() {
    const S = this.side;
    S.base.querySelectorAll("[data-base]").forEach((b) => (b.onclick = () => this.setBase(b.dataset.base)));
    S.follow.onclick = () => { this.follow = !this.follow; this.relabel(); };
    S.home.onclick = () => {
      const h = this.fc.home;
      if (!this.map) return;
      if (h && h.lat != null) this.map.setView([h.lat, h.lon], Math.max(this.map.getZoom(), 16));
      else { this.follow = false; this.relabel(); }
    };
    S.clear.onclick = () => {
      this.gpsTrack = []; this.fcTrack = [];
      if (!this.map) return;
      this.gpsLine.setLatLngs([]); this.fcLine.setLatLngs([]);
    };
    S.mission.onclick = () => this.link.send({ c: "mission" });
    S.tracks.onchange = () => {
      this.showTracks = S.tracks.checked;
      if (!this.map) return;
      this.gpsLine.setStyle({ opacity: this.showTracks ? .85 : 0 });
      this.fcLine.setStyle({ opacity: this.showTracks ? .8 : 0 });
    };
    S.missionOn.onchange = () => { this.showMission = S.missionOn.checked; this.drawMission(); };
  }

  setBase(name) {
    if (!this.map) { this.base = name; this.relabel(); return; }   // the map is built when the page is shown
    if (name !== "none" && !this.bases[name]) return;
    this.base = name;
    this.tileOK = 0; this.tileBad = 0; this.baseT0 = performance.now();
    for (const k of Object.keys(this.bases)) {
      if (k === name) this.bases[k].addTo(this.map); else this.map.removeLayer(this.bases[k]);
    }
    if (name === "none") this.grid.addTo(this.map); else this.map.removeLayer(this.grid);
    this.side.base.querySelectorAll("[data-base]").forEach((b) => b.classList.toggle("on", b.dataset.base === name));
    this.relabel();
  }

  /** The tiles did not come (no network at the field): fall back to the metric grid. */
  checkTiles() {
    if (!this.map || this.base === "none") return;
    if (this.tileBad >= 2 && this.tileOK === 0) this.setBase("none");
    else if (this.tileOK === 0 && performance.now() - this.baseT0 > 8000) this.setBase("none");
  }

  show() {
    this.visible = true;
    if (!this.map) this.build();
    else this.map.invalidateSize();
    this.update(true);
  }
  hide() { this.visible = false; }

  relabel() {
    const S = this.side;
    S.follow.textContent = this.follow ? L("跟随飞机 ✓", "Follow ✓") : L("跟随飞机", "Follow");
    S.follow.classList.toggle("on", this.follow);
    S.tiles.textContent = L(BASE_ZH[this.base], BASE_EN[this.base]);
    this.sidePanel(true);
  }

  // ------------------------------------------------------------------ data
  setState(fc) { this.fc = fc || {}; if (this.visible) this.update(); }

  pushTrack(arr, p) {
    const last = arr.length ? arr[arr.length - 1] : null;
    const now = performance.now();
    if (last && Math.abs(last[0] - p[0]) < 3e-6 && Math.abs(last[1] - p[1]) < 3e-6 && now - last[2] < 2000) return;
    arr.push([p[0], p[1], now]);
    const cut = now - TRACK_MIN * 60000;
    while (arr.length && arr[0][2] < cut) arr.shift();
  }

  update(force) {
    if (!this.map) return;
    const fc = this.fc || {};
    const gps = fc.gps || null;
    const fcpos = fc.fcpos || null;
    const home = fc.home || null;
    const hasFix = !!(gps && gps.fix >= 3 && gps.lat != null && gps.lon != null);
    const gpsLL = hasFix ? [gps.lat, gps.lon] : null;
    const fcLL = fcpos && fcpos.lat != null ? [fcpos.lat, fcpos.lon] : null;
    const homeLL = home && home.lat != null ? [home.lat, home.lon] : null;

    // ---- the GPS marker (grey without a 3D fix, hidden without GPS data at all)
    if (gpsLL) {
      this.gpsMark.setLatLng(gpsLL);
      if (hasFix) this.gpsMark.setStyle({ color: "#39ff88", fillColor: "#39ff88", fillOpacity: .25, opacity: 1 }).setRadius(8);
      else this.gpsMark.setStyle({ color: "#8a98a8", fillColor: "#8a98a8", fillOpacity: .15, opacity: 1 }).setRadius(6);
      const r = gps.hacc != null && gps.hacc > 0 ? gps.hacc : 0;
      this.gpsAcc.setLatLng(gpsLL).setRadius(r);
      this.gpsAcc.setStyle({ opacity: hasFix && r > 0 ? .8 : 0, fillOpacity: hasFix && r > 0 ? .08 : 0 });
    } else {
      this.gpsMark.setStyle({ opacity: 0 });
      this.gpsAcc.setStyle({ opacity: 0 });
    }

    // ---- the FC position (flow position) with the heading arrow
    if (fcLL) {
      this.fcMark.setLatLng(fcLL).setOpacity(1);
      const yaw = fc.att && fc.att[2] != null ? (fc.att[2] + 360) % 360 : null;
      const el = this.fcMark.getElement();
      const arrow = el && el.querySelector(".fcArrow");
      if (arrow) arrow.style.transform = `rotate(${yaw == null ? 0 : yaw}deg)`;
    } else this.fcMark.setOpacity(0);

    // ---- home
    if (homeLL) this.homeMark.setLatLng(homeLL).setOpacity(1); else this.homeMark.setOpacity(0);

    // ---- tracks
    if (this.showTracks) {
      if (gpsLL) this.pushTrack(this.gpsTrack, gpsLL);
      if (fcLL) this.pushTrack(this.fcTrack, fcLL);
      this.gpsLine.setLatLngs(this.gpsTrack);
      this.fcLine.setLatLngs(this.fcTrack);
    }

    // ---- follow
    if (this.follow) {
      const p = gpsLL || fcLL;
      if (p) {
        const c = this.map.getCenter();
        if (Math.abs(c.lat - p[0]) > 1e-6 || Math.abs(c.lng - p[1]) > 1e-6) this.map.panTo(p, { animate: false });
      }
    }
    this.checkTiles();
    this.drawMission();
    this.sidePanel(force);
  }

  drawMission() {
    this.missionGroup.clearLayers();
    const m = (this.fc || {}).mission;
    const items = (m && m.items) || [];
    if (!this.showMission || !items.length) {
      this.side.missionInfo.textContent = "";
      this.side.missionList.innerHTML = "";
      return;
    }
    const cur = m.current != null && m.current >= 0 ? m.current : -1;
    const pts = [];
    for (const it of items) {
      if (it.lat == null || it.lon == null) continue;
      const ll = [it.lat, it.lon];
      pts.push(ll);
      const isCur = it.seq === cur;
      this.missionGroup.addLayer(LM.marker(ll, {
        interactive: false,
        icon: LM.divIcon({ className: "marker mission" + (isCur ? " cur" : ""), html: String(it.seq + 1), iconSize: [18, 18], iconAnchor: [9, 9] })
      }));
    }
    if (pts.length > 1) this.missionGroup.addLayer(LM.polyline(pts, { color: "#4cc9f0", weight: 2, opacity: .8, interactive: false }));
    this.side.missionInfo.textContent = L(`${items.length} 个航点${cur >= 0 ? ` · 当前 ${cur + 1}` : ""}`, `${items.length} items${cur >= 0 ? ` · current ${cur + 1}` : ""}`);
    this.side.missionList.innerHTML = items.map((it) => {
      const isCur = it.seq === cur;
      const p = it.lat != null ? `${it.lat.toFixed(6)}, ${it.lon.toFixed(6)}` : "—";
      return `<div class="${isCur ? "cur" : ""}"><span class="n">${it.seq + 1}</span><span>${p}</span><span>${it.alt != null ? it.alt.toFixed(1) + " m" : ""}</span></div>`;
    }).join("");
  }

  sidePanel(force) {
    const now = performance.now();
    if (!force && now - this.lastPanel < 250) return;    // the state comes at 10 Hz: 4 Hz is enough here
    this.lastPanel = now;
    const S = this.side;
    const fc = this.fc || {};
    const gps = fc.gps || null;
    const fcpos = fc.fcpos || null;
    const home = fc.home || null;
    const hasFix = !!(gps && gps.fix >= 3 && gps.lat != null && gps.lon != null);
    const gpsLL = hasFix ? [gps.lat, gps.lon] : null;
    const fcLL = fcpos && fcpos.lat != null ? [fcpos.lat, fcpos.lon] : null;
    const homeLL = home && home.lat != null ? [home.lat, home.lon] : null;
    const f = (v, d = 2) => (v == null || !Number.isFinite(v) ? "—" : v.toFixed(d));
    const fix = gps ? gps.fix : null;
    const fixTxt = fix == null ? L("无数据", "no data") : L(FIX_ZH[fix] || String(fix), FIX_EN[fix] || String(fix));
    S.fix.className = fix == null ? "" : fix >= 3 ? "st good" : fix > 0 ? "st warn" : "st critical";
    S.fix.textContent = (fix == null ? "" : fix >= 3 ? "● " : fix > 0 ? "▲ " : "✕ ") + fixTxt;

    const dGps = gpsLL && homeLL ? distM(gpsLL, homeLL) : null;
    const bGps = gpsLL && homeLL ? bearingDeg(gpsLL, homeLL) : null;
    const dFc = fcLL && homeLL ? distM(fcLL, homeLL) : null;
    const bFc = fcLL && homeLL ? bearingDeg(fcLL, homeLL) : null;
    const dDiff = gpsLL && fcLL ? distM(gpsLL, fcLL) : null;
    const ekfAlt = fc.lpn && fc.lpn[2] != null ? -fc.lpn[2] : null;
    const relAlt = fc.gpos && fc.gpos.rel != null ? fc.gpos.rel : ekfAlt;

    const tiles = [
      [L("定位", "Fix"), fixTxt, ""],
      [L("卫星数", "Satellites"), gps ? String(gps.sats) : "—", ""],
      [L("HDOP", "HDOP"), f(gps && gps.hdop, 2), ""],
      [L("水平精度 h_acc", "Accuracy h_acc"), f(gps && gps.hacc, 2), "m"],
      [L("GPS 数据年龄", "GPS age"), f(gps && gps.age, 1), "s"],
      [L("GPS→起点 距离", "GPS→home"), f(dGps, 1), "m"],
      [L("GPS→起点 方位", "GPS→home brg"), bGps == null ? "—" : `${bGps.toFixed(0)}°`, ""],
      [L("飞控位置→起点 距离", "FC→home"), f(dFc, 1), "m"],
      [L("飞控位置→起点 方位", "FC→home brg"), bFc == null ? "—" : `${bFc.toFixed(0)}°`, ""],
      [L("GPS 与飞控位置之差", "GPS − FC position"), f(dDiff, 2), "m"],
      [L("GPS 纬度", "GPS latitude"), gps && gps.lat != null ? gps.lat.toFixed(7) : "—", ""],
      [L("GPS 经度", "GPS longitude"), gps && gps.lon != null ? gps.lon.toFixed(7) : "—", ""],
      [L("飞控位置（光流）", "FC position (flow)"), fcpos && fcpos.lat != null ? `${fcpos.lat.toFixed(7)}, ${fcpos.lon.toFixed(7)}` : "—", ""],
      [L("本地 N/E", "Local N/E"), fcpos && fcpos.n != null ? `${f(fcpos.n, 1)}, ${f(fcpos.e, 1)}` : "—", "m"],
      [L("起点（HOME）", "Home"), homeLL ? `${homeLL[0].toFixed(7)}, ${homeLL[1].toFixed(7)}` : "—", ""],
      [L("下视测距", "Rangefinder"), f(fc.rng), "m"],
      [L("EKF 相对高度", "EKF altitude"), f(relAlt), "m"],
      [L("GPS 高度", "GPS altitude"), f(gps && gps.alt, 1), "m"],
      [L("模式", "Mode"), fc.mode || "—", ""],
      [L("解锁", "Armed"), fc.armed ? L("已解锁", "armed") : L("上锁", "disarmed"), ""],
    ];
    S.values.innerHTML = tiles.map(([k, v, u]) => `<div class="tile"><div class="v">${v}<small>${u}</small></div><div class="k">${k}</div></div>`).join("");

    const notes = [];
    if (gps && gps.fix != null && gps.fix < 3) notes.push(L("■ GPS 未定位（fix < 3）：灰色标记，精度圈无效", "■ No GPS fix (fix < 3): the marker is grey, the accuracy circle is meaningless"));
    if (fcpos == null) notes.push(L("■ 没有飞控位置：EKF 原点 / LOCAL_POSITION_NED 还没到", "■ No FC position yet: the EKF origin / LOCAL_POSITION_NED have not arrived"));
    if (this.base === "none") notes.push(L("■ 没有底图（无网络）：显示公制网格与比例尺", "■ No tiles (no network): the metric grid and the scale bar are shown"));
    else notes.push(L(`■ 底图 ${this.tileOK} 张成功 / ${this.tileBad} 张失败`, `■ tiles ${this.tileOK} loaded / ${this.tileBad} failed`));
    S.chip.textContent = notes.join("  ·  ");
  }
}
