// Camera panel: the onboard camera, either as the brain sees it (the 192x144 gray frame the retina gets, with the
// ommatidia grid, optic-flow arrows, looming and the ECPS cues LCb / LCn / LCv, after FlyDrones' "Fly eyes" panel)
// or the colour picture.

import { L } from "./i18n.js";

const FEAT = ["ftb", "btf", "up", "down", "loom", "loom_speed", "blank", "near", "ventral"];

export class EyesView {
  constructor(canvas, info) {
    this.c = canvas; this.g = canvas.getContext("2d");
    this.info = info;
    this.bmp = null; this.kind = 0; this.w = 0; this.h = 0;
    this.frames = []; this.lastFrameT = 0; this.grids = null;
    this.mode = "brain";
    this.decoding = false;
  }

  onCam(m) {
    if (this.decoding) return;  // keep only the newest frame
    this.decoding = true;
    createImageBitmap(new Blob([m.jpeg], { type: "image/jpeg" })).then((b) => {
      if (this.bmp) this.bmp.close();
      this.bmp = b; this.kind = m.kind; this.w = m.w; this.h = m.h;
      this.lastFrameT = performance.now(); this.frames.push(this.lastFrameT);
    }).catch(() => {}).finally(() => { this.decoding = false; });
  }

  onBrain(m) { if (m.grids) this.grids = m; }

  grid(eye, f) {
    const m = this.grids, k = FEAT.indexOf(f);
    if (!m || k < 0 || k >= m.nfeat) return null;
    const sz = m.rows * m.cols, off = (eye * m.nfeat + k) * sz;
    return m.grids.subarray(off, off + sz);
  }

  draw(tick) {
    const g = this.g, W = this.c.width, H = this.c.height;
    const now = performance.now();
    this.frames = this.frames.filter((t) => now - t < 2000);
    g.fillStyle = "#030508"; g.fillRect(0, 0, W, H);
    if (this.mode === "off") { this.text(L("相机画面已关闭（不占用带宽）", "Camera off (uses no bandwidth)")); return; }
    if (this.level === 0) { this.text(L("带宽档位「极低」：相机画面暂停", "Bandwidth level “very low”: camera paused")); return; }
    if (!this.bmp) { this.text(L("等待相机画面…", "Waiting for the camera…")); return; }
    g.imageSmoothingEnabled = this.kind === 1;
    g.drawImage(this.bmp, 0, 0, W, H);
    if (this.kind === 0) {  // FlyDrones tint for the brain's view
      g.globalCompositeOperation = "multiply"; g.fillStyle = "rgb(158,230,255)"; g.fillRect(0, 0, W, H);
      g.globalCompositeOperation = "source-over"; g.fillStyle = "rgba(0,0,0,.22)"; g.fillRect(0, 0, W, H);
    }
    const age = tick && tick.age ? tick.age[0] : null;
    const stale = !this.lastFrameT || now - this.lastFrameT > 1500;
    this.info.textContent = `${this.kind === 0 ? L("大脑视角", "Brain view") : L("彩色", "Color")} ${this.w}×${this.h} · ${(this.frames.length / 2).toFixed(0)} fps`
      + (age != null ? L(` · 机载帧龄 ${age.toFixed(0)} ms`, ` · onboard age ${age.toFixed(0)} ms`) : "") + (stale ? L(" · ▲ 画面中断", " · ▲ stalled") : "");
    if (this.kind === 0 && this.grids) this.overlay();
    if (stale) { g.fillStyle = "rgba(5,7,11,.55)"; g.fillRect(0, 0, W, H); }
  }

  text(s) {
    const g = this.g;
    g.fillStyle = "rgba(138,152,168,.85)"; g.font = "16px -apple-system, 'PingFang SC', sans-serif"; g.textAlign = "center";
    g.fillText(s, this.c.width / 2, this.c.height / 2);
    this.info.textContent = L("无画面", "No picture");
  }

  overlay() {
    const g = this.g, W = this.c.width, H = this.c.height, m = this.grids;
    const rows = m.rows, ncols = m.cols * 2, cw = W / ncols, ch = H / rows;
    g.strokeStyle = "rgba(76,201,240,.16)"; g.lineWidth = 1;
    for (let c = 1; c < ncols; c++) { g.beginPath(); g.moveTo(c * cw, 0); g.lineTo(c * cw, H); g.stroke(); }
    for (let r = 1; r < rows; r++) { g.beginPath(); g.moveTo(0, r * ch); g.lineTo(W, r * ch); g.stroke(); }
    g.strokeStyle = "rgba(76,201,240,.7)"; g.lineWidth = 2; g.beginPath(); g.moveTo(W / 2, 0); g.lineTo(W / 2, H); g.stroke();
    // optic flow per ommatidium: image velocity from the four motion channels (front-to-back = away from the midline)
    let loom = 0;
    g.lineWidth = 2.2;
    for (let e = 0; e < 2; e++) {
      const ftb = this.grid(e, "ftb"), btf = this.grid(e, "btf"), up = this.grid(e, "up"), dn = this.grid(e, "down"), lm = this.grid(e, "loom");
      if (!ftb) continue;
      for (let r = 0; r < rows; r++) for (let c = 0; c < m.cols; c++) {
        const k = r * m.cols + c;
        const hx = (ftb[k] - btf[k]) / 255 * (e === 0 ? -1 : 1), vy = (dn[k] - up[k]) / 255;
        loom = Math.max(loom, lm[k] / 255);
        const mag = Math.hypot(hx, vy);
        if (mag < 0.08) continue;
        const cx = (e * m.cols + c + 0.5) * cw, cy = (r + 0.5) * ch;
        const s = Math.min(1, mag), L = Math.min(cw * 0.45, 6 + 14 * s), ux = hx / mag, uy = vy / mag;
        g.strokeStyle = `rgba(57,255,${136 + 60 * s},${0.35 + 0.6 * s})`;
        g.beginPath(); g.moveTo(cx - ux * L, cy - uy * L); g.lineTo(cx + ux * L, cy + uy * L);
        g.lineTo(cx + ux * L - (ux * 6 - uy * 5), cy + uy * L - (uy * 6 + ux * 5));
        g.moveTo(cx + ux * L, cy + uy * L); g.lineTo(cx + ux * L - (ux * 6 + uy * 5), cy + uy * L - (uy * 6 - ux * 5)); g.stroke();
      }
    }
    if (loom > 0.15) {
      g.strokeStyle = `rgba(255,120,40,${0.4 + 0.6 * loom})`; g.lineWidth = 9; g.strokeRect(4, 4, W - 8, H - 8);
      g.font = "900 28px ui-monospace, Menlo, monospace"; g.fillStyle = "#ffb020"; g.textAlign = "center";
      g.fillText(L("LOOMING 逼近", "LOOMING"), W / 2, H / 2 + 10);
    }
    // ECPS cues per eye: LCb (texture lost ahead), LCn (this side nearer), LCv (something below)
    const bars = [[L("LCb 无纹理", "LCb blank"), "blank", "#4cc9f0"], [L("LCn 近侧", "LCn near"), "near", "#39ff88"], [L("LCv 下方", "LCv below"), "ventral", "#c77dff"]];
    g.font = "600 13px -apple-system, 'PingFang SC', sans-serif";
    for (let e = 0; e < 2; e++) {
      bars.forEach(([label, f, col], i) => {
        const a = this.grid(e, f);
        if (!a) return;
        const v = a[0] / 255, x = e === 0 ? 10 : W - 130, y = H - 58 + i * 17;
        g.fillStyle = "rgba(5,7,11,.6)"; g.fillRect(x - 4, y - 12, 126, 16);
        g.fillStyle = "rgba(255,255,255,.12)"; g.fillRect(x + 64, y - 8, 52, 8);
        g.fillStyle = col; g.fillRect(x + 64, y - 8, 52 * v, 8);
        g.fillStyle = "rgba(232,238,245,.85)"; g.textAlign = "left"; g.fillText(label, x, y);
      });
    }
    g.font = "700 16px -apple-system, 'PingFang SC', sans-serif"; g.fillStyle = "rgba(232,238,245,.85)";
    g.textAlign = "right"; g.fillText(L("L 眼", "L eye"), W / 2 - 10, H - 12); g.textAlign = "left"; g.fillText(L("R 眼", "R eye"), W / 2 + 10, H - 12);
  }
}
