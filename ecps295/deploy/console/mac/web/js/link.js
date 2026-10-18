// WebSocket to the app's native side (WebServer.swift): companion messages arrive as binary [1][type][payload]
// (already inflated), the native state / acks / logs as JSON text. Commands go back as JSON text.

export const T = { TICK: 1, BRAIN: 2, CAM: 3, MAP: 4, TOF: 5, TOPO: 6, ACK: 7, STATUS: 8 };
const td = new TextDecoder();

export class Link extends EventTarget {
  constructor() {
    super();
    const qs = new URLSearchParams(location.search);
    this.url = `ws://127.0.0.1:${qs.get("ws") || "8771"}/`;
    this.token = qs.get("t") || "";
    this.ws = null;
    this.open = false;
    this.req = 0;
    this.connect();
  }

  connect() {
    const ws = new WebSocket(this.url);
    ws.binaryType = "arraybuffer";
    ws.onopen = () => { this.open = true; ws.send(JSON.stringify({ auth: this.token })); this.emit("open", {}); };
    ws.onclose = () => { this.open = false; this.emit("close", {}); setTimeout(() => this.connect(), 1000); };
    ws.onerror = () => {};
    ws.onmessage = (ev) => {
      if (typeof ev.data === "string") {
        let m;
        try { m = JSON.parse(ev.data); } catch { return; }
        this.emit(m.k || "msg", m);
        return;
      }
      const u8 = new Uint8Array(ev.data);
      if (u8[0] !== 1 || u8.length < 2) return;
      const type = u8[1];
      const p = u8.subarray(2);
      try { this.decode(type, p); } catch (e) { console.warn("decode", type, e); }
    };
    this.ws = ws;
  }

  send(obj) {
    if (!this.open) return 0;
    obj.req = ++this.req;
    this.ws.send(JSON.stringify(obj));
    return obj.req;
  }

  emit(name, detail) { this.dispatchEvent(new CustomEvent(name, { detail })); }

  decode(type, p) {
    const dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    switch (type) {
      case T.TICK: this.emit("tick", JSON.parse(td.decode(p))); break;
      case T.STATUS: this.emit("status", JSON.parse(td.decode(p))); break;
      case T.ACK: this.emit("eclack", JSON.parse(td.decode(p))); break;
      case T.TOPO: this.emit("topo", JSON.parse(td.decode(p))); break;
      case T.BRAIN: {
        const tick = dv.getUint32(0, true), n = dv.getUint16(4, true), rows = p[6], cols = p[7], nfeat = p[8];
        const counts = p.subarray(9, 9 + n);
        const grids = nfeat ? p.subarray(9 + n, 9 + n + 2 * nfeat * rows * cols) : null;
        this.emit("brain", { tick, n, rows, cols, nfeat, counts, grids });
        break;
      }
      case T.CAM: {
        const kind = p[0], w = dv.getUint16(1, true), h = dv.getUint16(3, true), no = dv.getUint32(5, true);
        this.emit("cam", { kind, w, h, no, jpeg: p.slice(9) });
        break;
      }
      case T.MAP: {
        const hl = dv.getUint16(0, true);
        const hdr = JSON.parse(td.decode(p.subarray(2, 2 + hl)));
        const nv = hdr.nvox, off = 2 + hl;
        const idx = new Float64Array(nv);
        let acc = 0;
        for (let i = 0; i < nv; i++) { acc += dv.getUint32(off + 4 * i, true); idx[i] = acc; }
        const conf = p.subarray(off + 4 * nv, off + 5 * nv);
        this.emit("map", { hdr, idx, conf });
        break;
      }
      case T.TOF: {
        const cnt = p[0];
        let off = 1;
        const sensors = [];
        for (let s = 0; s < cnt; s++) {
          const bearing = dv.getInt16(off, true), n = p[off + 2], age = p[off + 3] * 10;
          off += 4;
          const r = new Float32Array(n * n);
          for (let i = 0; i < n * n; i++) r[i] = f16(dv.getUint16(off + 2 * i, true));
          off += 2 * n * n;
          sensors.push({ bearing, n, age, r });
        }
        this.emit("tof", sensors);
        break;
      }
      default: break;
    }
  }
}

// IEEE half -> float (NaN = no target)
function f16(h) {
  const s = (h & 0x8000) ? -1 : 1, e = (h >> 10) & 0x1f, f = h & 0x3ff;
  if (e === 0) return s * Math.pow(2, -14) * (f / 1024);
  if (e === 31) return f ? NaN : s * Infinity;
  return s * Math.pow(2, e - 15) * (1 + f / 1024);
}
