// 3D map: the companion's local occupancy map (ecps_map.LocalMap, 5 cm voxels, flow-EKF frame) as instanced cubes
// coloured by height, the drone and its track, the live rays of the two 8x8 ToF sensors, the range shield's
// constraints (the map's nearest obstacle per sector) and query volume, the geofence, the goal; click to pick a goal.
// Frame: north / east / up from the EKF origin; three.js x = east, y = up, z = south.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { L } from "./i18n.js";

const V = (n, e, u) => new THREE.Vector3(e, u, -n);
const MAXVOX = 80000;
// sequential blue ramp (dataviz reference ramp, dark -> light with height)
const RAMP = ["#184f95", "#1c5cab", "#256abf", "#2a78d6", "#3987e5", "#5598e7", "#6da7ec", "#86b6ef", "#9ec5f4", "#b7d3f6", "#cde2fb"];
const rampCol = RAMP.map((h) => new THREE.Color(h));

function rad(d) { return (d * Math.PI) / 180; }

export class MapView {
  constructor(canvas, wrap, legend, onPick) {
    this.canvas = canvas; this.wrap = wrap; this.onPick = onPick; this.legend = legend;
    const r = new THREE.WebGLRenderer({ canvas, antialias: true });
    r.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    r.setClearColor(0x04060a, 1);
    this.renderer = r;
    const scene = new THREE.Scene();
    this.scene = scene;
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 80);
    this.camera.position.set(3.2, 4.2, 4.6);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0.5, 0);
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.view = "orbit";
    scene.add(new THREE.HemisphereLight(0xbfd6ff, 0x101820, 0.9));
    const dl = new THREE.DirectionalLight(0xffffff, 0.9); dl.position.set(3, 6, 2); scene.add(dl);

    // floor grid 8 x 8 m (the map's extent), 0.5 m cells; N / E labels
    const grid = new THREE.GridHelper(8, 16, 0x2b3a4c, 0x182230); scene.add(grid);
    scene.add(this.label("N", V(4.3, 0, 0.02), "#8a98a8")); scene.add(this.label("E", V(0, 4.3, 0.02), "#8a98a8"));
    const axes = new THREE.BufferGeometry().setFromPoints([V(0, 0, 0.005), V(0.6, 0, 0.005), V(0, 0, 0.005), V(0, 0.6, 0.005)]);
    scene.add(new THREE.LineSegments(axes, new THREE.LineBasicMaterial({ color: 0x8a98a8 })));
    // home (EKF origin)
    const home = new THREE.Mesh(new THREE.RingGeometry(0.1, 0.13, 32), new THREE.MeshBasicMaterial({ color: 0x8a98a8, side: THREE.DoubleSide }));
    home.rotation.x = -Math.PI / 2; home.position.y = 0.004; scene.add(home);

    // voxels
    const geo = new THREE.BoxGeometry(1, 1, 1);
    this.vox = new THREE.InstancedMesh(geo, new THREE.MeshLambertMaterial({ color: 0xffffff }), MAXVOX);
    this.vox.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(MAXVOX * 3), 3);
    this.vox.count = 0; this.vox.frustumCulled = false;
    scene.add(this.vox);
    this.mapHdr = null;

    // drone (about the real size: 0.25 m motor-to-motor)
    this.drone = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.035, 0.12), new THREE.MeshLambertMaterial({ color: 0xd7e0ea }));
    this.drone.add(body);
    const armM = new THREE.MeshLambertMaterial({ color: 0x8a98a8 });
    const propM = new THREE.MeshBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.55, side: THREE.DoubleSide });
    for (const [dx, dz] of [[0.088, 0.088], [-0.088, 0.088], [0.088, -0.088], [-0.088, -0.088]]) {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.01, 0.125), armM);
      arm.position.set(dx / 2, 0, dz / 2); arm.rotation.y = Math.atan2(dx, dz); this.drone.add(arm);
      const prop = new THREE.Mesh(new THREE.CircleGeometry(0.06, 24), propM);
      prop.rotation.x = -Math.PI / 2; prop.position.set(dx, 0.02, dz); this.drone.add(prop);
    }
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.03, 0.09, 12), new THREE.MeshBasicMaterial({ color: 0x39ff88 }));
    nose.rotation.x = -Math.PI / 2; nose.position.set(0, 0, -0.11); this.drone.add(nose);
    // ToF frusta (60 x 60 deg, 0.8 m) front and rear, in the body
    for (const b of [0, 180]) this.drone.add(this.frustum(b));
    // shield query volume: cylinder query_m around the drone, +- band_m around its height
    this.query = new THREE.Mesh(new THREE.CylinderGeometry(1, 1, 1, 48, 1, true),
      new THREE.MeshBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.06, side: THREE.DoubleSide, depthWrite: false }));
    scene.add(this.query);
    this.dsafe = new THREE.Mesh(new THREE.RingGeometry(0.245, 0.255, 48), new THREE.MeshBasicMaterial({ color: 0xec835a, side: THREE.DoubleSide, transparent: true, opacity: 0.7 }));
    this.dsafe.rotation.x = -Math.PI / 2; scene.add(this.dsafe);
    scene.add(this.drone);
    this.drone.visible = false;

    // ghost: simulator truth (SITL SIM_STATE), shows the flow-EKF drift
    this.ghost = new THREE.Mesh(new THREE.BoxGeometry(0.25, 0.03, 0.25), new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true, transparent: true, opacity: 0.35 }));
    this.ghost.visible = false; scene.add(this.ghost);

    // track
    this.TRAIL = 1200;
    this.trail = [];
    this.trailGeo = new THREE.BufferGeometry();
    this.trailGeo.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(this.TRAIL * 3), 3));
    this.trailLine = new THREE.Line(this.trailGeo, new THREE.LineBasicMaterial({ color: 0x39ff88, transparent: true, opacity: 0.6 }));
    this.trailLine.frustumCulled = false; scene.add(this.trailLine);

    // ToF rays (2 x 64) and hits
    this.rayGeo = new THREE.BufferGeometry();
    this.rayGeo.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(128 * 2 * 3), 3));
    this.rays = new THREE.LineSegments(this.rayGeo, new THREE.LineBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.22 }));
    this.rays.frustumCulled = false; scene.add(this.rays);
    this.hitGeo = new THREE.BufferGeometry();
    this.hitGeo.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(128 * 3), 3));
    this.hits = new THREE.Points(this.hitGeo, new THREE.PointsMaterial({ color: 0x9fe8ff, size: 0.045 }));
    this.hits.frustumCulled = false; scene.add(this.hits);

    // shield ring: nearest mapped obstacle per sector in the flight band
    this.ringGeo = new THREE.BufferGeometry();
    this.ringGeo.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(72 * 3), 3));
    this.ringGeo.setAttribute("color", new THREE.Float32BufferAttribute(new Float32Array(72 * 3), 3));
    this.ring = new THREE.Points(this.ringGeo, new THREE.PointsMaterial({ size: 0.07, vertexColors: true }));
    this.ring.frustumCulled = false; scene.add(this.ring);

    // geofence + go-to limit
    this.fence = null; this.gotoLim = null; this.ceiling = null;
    this.setLimits({ fence_m: 2.0, goto_limit_m: 1.6, max_alt_m: 1.0 });

    // goal and pick preview
    this.goal = this.pin(0x39ff88); this.goal.visible = false; scene.add(this.goal);
    this.preview = this.pin(0xffffff); this.preview.visible = false; scene.add(this.preview);
    this.picking = false;
    this.ray = new THREE.Raycaster();
    canvas.addEventListener("pointerdown", (e) => (this.down = [e.clientX, e.clientY]));
    canvas.addEventListener("pointerup", (e) => this.click(e));

    this.drawLegend();
    new ResizeObserver(() => this.resize()).observe(wrap);
    this.resize();
    this.pose = null;
  }

  label(text, pos, color) {
    const c = document.createElement("canvas"); c.width = 64; c.height = 64;
    const g = c.getContext("2d"); g.fillStyle = color; g.font = "700 40px -apple-system, sans-serif"; g.textAlign = "center"; g.textBaseline = "middle"; g.fillText(text, 32, 34);
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), transparent: true, depthWrite: false }));
    s.position.copy(pos); s.scale.set(0.35, 0.35, 0.35);
    return s;
  }

  frustum(bearing) {
    const L = 0.8, h = Math.tan(rad(30)) * L;
    const pts = [];
    const corners = [[h, h], [-h, h], [-h, -h], [h, -h]];
    for (const [x, y] of corners) { pts.push(new THREE.Vector3(0, 0, 0), new THREE.Vector3(x, y, -L)); }
    for (let i = 0; i < 4; i++) { const a = corners[i], b = corners[(i + 1) % 4]; pts.push(new THREE.Vector3(a[0], a[1], -L), new THREE.Vector3(b[0], b[1], -L)); }
    const f = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.35 }));
    f.rotation.y = -rad(bearing);
    return f;
  }

  pin(color) {
    const g = new THREE.Group();
    const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 1.0, 8), new THREE.MeshBasicMaterial({ color }));
    stem.position.y = 0.5; g.add(stem);
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.12, 0.16, 32), new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.8 }));
    ring.rotation.x = -Math.PI / 2; ring.position.y = 0.006; g.add(ring);
    return g;
  }

  circle(r, y, color, dashed) {
    const pts = [];
    for (let i = 0; i <= 96; i++) { const a = (i / 96) * Math.PI * 2; pts.push(new THREE.Vector3(Math.cos(a) * r, y, Math.sin(a) * r)); }
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = dashed ? new THREE.LineDashedMaterial({ color, dashSize: 0.08, gapSize: 0.06, transparent: true, opacity: 0.8 })
      : new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.8 });
    const l = new THREE.Line(geo, mat);
    if (dashed) l.computeLineDistances();
    return l;
  }

  setLimits(lim) {
    if (!lim) return;
    const key = JSON.stringify([lim.fence_m, lim.goto_limit_m, lim.max_alt_m]);
    if (key === this.limKey) return;
    this.limKey = key;
    this.limits = lim;
    for (const o of [this.fence, this.gotoLim, this.ceiling]) if (o) this.scene.remove(o);
    this.fence = this.circle(lim.fence_m || 2, 0.006, 0xd03b3b, false); this.scene.add(this.fence);
    this.gotoLim = this.circle(lim.goto_limit_m || 1.6, 0.006, 0x8a98a8, true); this.scene.add(this.gotoLim);
    this.ceiling = this.circle(lim.fence_m || 2, lim.max_alt_m || 1.0, 0xd03b3b, true); this.scene.add(this.ceiling);
    this.drawLegend();
  }

  drawLegend() {
    const grad = `linear-gradient(90deg, ${RAMP.join(",")})`;
    const zmax = (this.mapHdr && this.mapHdr.nz * this.mapHdr.res) || 1.6, fence = this.limits ? this.limits.fence_m : 2;
    this.legend.innerHTML = `
      <div class="row">${L("障碍物高度", "Obstacle height")} 0 m <span class="ramp" style="background:${grad}"></span> ${zmax.toFixed(1)} m</div>
      <div class="row"><span class="sw" style="background:#4cc9f0"></span>${L("ToF 射线 / 命中（前、后 8×8）", "ToF rays / hits (front, rear 8×8)")}</div>
      <div class="row"><span class="sw" style="background:#ec835a"></span>${L("■ 安全盾约束（地图最近障碍） · 橙圈", "■ Shield constraints (nearest mapped obstacle) · orange ring")} d<sub>safe</sub></div>
      <div class="row"><span class="sw" style="background:#d03b3b"></span>${L(`围栏 ${fence} m · 虚线 = 目标点范围`, `Geofence ${fence} m · dashed = goal range`)}</div>
      <div class="row"><span class="sw" style="background:#39ff88"></span>${L("航迹 / 目标点", "Track / goal")}</div>`;
  }

  resize() {
    const w = this.wrap.clientWidth, h = this.wrap.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  onMap(m) {
    const { hdr, idx, conf } = m;
    this.mapHdr = hdr;
    const n = hdr.n, nz = hdr.nz, res = hdr.res, half = hdr.half;
    const cnt = Math.min(idx.length, MAXVOX);
    const mtx = new THREE.Matrix4(), col = new THREE.Color();
    const s = res * 0.92;
    const zmax = nz * res;
    for (let q = 0; q < cnt; q++) {
      const lin = idx[q];
      const k = lin % nz, j = Math.floor(lin / nz) % n, i = Math.floor(lin / (nz * n));
      const north = (i + 0.5) * res - half, east = (j + 0.5) * res - half, up = (k + 0.5) * res;
      mtx.makeScale(s, s, s); mtx.setPosition(east, up, -north);
      this.vox.setMatrixAt(q, mtx);
      const f = Math.min(1, up / zmax) * (RAMP.length - 1), a = Math.floor(f), b = Math.min(RAMP.length - 1, a + 1);
      col.copy(rampCol[a]).lerp(rampCol[b], f - a);
      col.multiplyScalar(0.55 + 0.45 * (conf[q] / 255));  // weakly seen voxels darker
      this.vox.setColorAt(q, col);
    }
    this.vox.count = cnt;
    this.vox.instanceMatrix.needsUpdate = true;
    this.vox.instanceColor.needsUpdate = true;
    if (!this.legendDone) { this.legendDone = true; this.drawLegend(); }
  }

  onTick(t) {
    const tel = t.tel || [];
    const [x, y, alt, yaw] = tel;
    if (x == null || y == null || yaw == null) { this.drone.visible = false; this.pose = null; return; }
    const u = alt == null ? 0 : alt;
    this.pose = { n: x, e: y, u, yaw: rad(yaw) };
    this.drone.visible = true;
    this.drone.position.copy(V(x, y, u));
    this.drone.rotation.set(0, -rad(yaw), 0);
    const hdr = this.mapHdr;
    const qr = hdr ? hdr.query : 1.0, band = hdr ? hdr.band : 0.1;
    this.query.scale.set(qr, 2 * band, qr); this.query.position.copy(V(x, y, u));
    this.dsafe.position.copy(V(x, y, u));
    const last = this.trail[this.trail.length - 1];
    if (!last || Math.hypot(last[0] - x, last[1] - y, last[2] - u) > 0.02) {
      this.trail.push([x, y, u]);
      if (this.trail.length > this.TRAIL) this.trail.shift();
      const p = this.trailGeo.attributes.position.array;
      for (let i = 0; i < this.trail.length; i++) { const q = this.trail[i]; p[i * 3] = q[1]; p[i * 3 + 1] = q[2]; p[i * 3 + 2] = -q[0]; }
      this.trailGeo.setDrawRange(0, this.trail.length);
      this.trailGeo.attributes.position.needsUpdate = true;
    }
    if (t.goal) { this.goal.visible = true; this.goal.position.copy(V(t.goal[0], t.goal[1], 0)); } else this.goal.visible = false;
    // shield constraints: body-frame sector angle (deg, + right) and distance to the obstacle surface (cm)
    const rp = this.ringGeo.attributes.position.array, rc = this.ringGeo.attributes.color.array;
    const ring = t.ring || [];
    const dsafe = 0.25;
    for (let i = 0; i < 72; i++) {
      if (i < ring.length) {
        const a = this.pose.yaw + rad(ring[i][0]), d = ring[i][1] / 100 + 0.06;
        rp[i * 3] = y + Math.sin(a) * d; rp[i * 3 + 1] = u; rp[i * 3 + 2] = -(x + Math.cos(a) * d);
        const near = ring[i][1] / 100 < dsafe + 0.15;
        rc[i * 3] = near ? 0.82 : 0.93; rc[i * 3 + 1] = near ? 0.23 : 0.51; rc[i * 3 + 2] = near ? 0.23 : 0.35;
      } else { rp[i * 3 + 1] = -50; }
    }
    this.ringGeo.attributes.position.needsUpdate = true; this.ringGeo.attributes.color.needsUpdate = true;
  }

  onTof(sensors) {
    if (!this.pose) return;
    const { n: pn, e: pe, u: pu, yaw: h } = this.pose;
    const rp = this.rayGeo.attributes.position.array, hp = this.hitGeo.attributes.position.array;
    let k = 0, kh = 0;
    for (const s of sensors) {
      const N = s.n, hf = rad(60) / N, vf = rad(60) / N, b = rad(s.bearing);
      const on = pn + 0.06 * Math.cos(h + b), oe = pe + 0.06 * Math.sin(h + b);
      for (let row = 0; row < N; row++) for (let col = 0; col < N; col++) {
        const az = b + (col - (N - 1) / 2) * hf, el = ((N - 1) / 2 - row) * vf;
        const bx = Math.cos(el) * Math.cos(az), by = Math.cos(el) * Math.sin(az), bz = Math.sin(el);
        const dn = Math.cos(h) * bx - Math.sin(h) * by, de = Math.sin(h) * bx + Math.cos(h) * by;
        const r = s.r[row * N + col];
        const L = Number.isFinite(r) ? r : 0.25;  // no target: a short stub
        if (k < 128) {
          rp.set([oe, pu, -on, oe + de * L, pu + bz * L, -(on + dn * L)], k * 6); k++;
        }
        if (Number.isFinite(r) && kh < 128) { hp.set([oe + de * r, pu + bz * r, -(on + dn * r)], kh * 3); kh++; }
      }
    }
    this.rayGeo.setDrawRange(0, k * 2); this.rayGeo.attributes.position.needsUpdate = true;
    this.hitGeo.setDrawRange(0, kh); this.hitGeo.attributes.position.needsUpdate = true;
  }

  setTruth(t) {  // [n, e, u, yaw rad] or null
    if (!t) { this.ghost.visible = false; return; }
    this.ghost.visible = true; this.ghost.position.copy(V(t[0], t[1], t[2])); this.ghost.rotation.set(0, -t[3], 0);
  }

  setView(v) {
    this.view = v;
    if (v === "top") { this.camera.position.set(0.001, 9.5, 0.01); this.controls.target.set(0, 0, 0); }
    if (v === "orbit") { this.camera.position.set(3.2, 4.2, 4.6); this.controls.target.set(0, 0.5, 0); }
  }

  setPicking(on) {
    this.picking = on;
    this.preview.visible = false;
    this.canvas.style.cursor = on ? "crosshair" : "";
  }

  click(e) {
    const moved = this.down ? Math.hypot(e.clientX - this.down[0], e.clientY - this.down[1]) : 99;
    if (!this.picking || moved > 5) return;
    const r = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    this.ray.setFromCamera(ndc, this.camera);
    const hit = new THREE.Vector3();
    if (!this.ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), hit)) return;
    const n = -hit.z, east = hit.x;
    this.preview.visible = true; this.preview.position.set(east, 0, -n);
    this.onPick(n, east);
  }

  render() {
    if (this.view === "chase" && this.pose) {
      const { n, e, u, yaw } = this.pose;
      const want = V(n - Math.cos(yaw) * 1.6, e - Math.sin(yaw) * 1.6, u + 0.9);
      this.camera.position.lerp(want, 0.08);
      this.controls.target.lerp(V(n + Math.cos(yaw) * 0.5, e + Math.sin(yaw) * 0.5, u), 0.15);
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
