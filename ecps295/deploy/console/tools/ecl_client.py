#!/usr/bin/env python3
"""Reference / test client for the ECPS295 ground-console link (ECL1, companion/console_link.py), standalone (numpy only).

Connects like the Mac console (HELLO 5 Hz, PING 2 Hz), decodes every message type, prints a 1 Hz summary and can send
commands and the emergency stop. Used by the console tests and handy on the Pi itself for a headless check.

    python ecl_client.py --host 192.168.86.20 --seconds 10
    python ecl_client.py --host 127.0.0.1 --cmd hold --cmd 'goto {"x": 1.0, "y": 0.5}' --seconds 20
    python ecl_client.py --host 127.0.0.1 --estop 1 --seconds 3 --json-out /tmp/ecl.json
    python ecl_client.py --host 127.0.0.1 --save-cam /tmp/cam.jpg --cam color --seconds 3
    python ecl_client.py --host 192.168.86.20 --port 14701 --lvl 3 --cam-hz 20 --seconds 30 --quiet --json-out cam.json
"""

from __future__ import annotations

import argparse
import collections
import json
import socket
import struct
import sys
import time
import zlib

import numpy as np

MAGIC = b"ECL1"
HDR = struct.Struct("<4sBBHHHI")
MAX_PAYLOAD = 1200
F_ZLIB = 1
NAMES = {1: "TICK", 2: "BRAIN", 3: "CAM", 4: "MAP", 5: "TOF", 6: "TOPO", 7: "ACK", 8: "STATUS", 9: "PONG"}
T_HELLO, T_PING, T_CMD, T_ESTOP, T_TOPO_REQ = 64, 65, 66, 67, 68


class Link:
    def __init__(self, host: str, port: int, name: str = "ecl_client", cam: str = "brain", cam_hz: float = 5.0,
                 lvl: int | None = None, color_w: int = 640, color_q: int = 60):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        self.msg = 0
        self.parts: dict = {}
        self.hello = {"name": name, "cam": cam, "cam_hz": cam_hz, "color_w": color_w, "color_q": color_q}
        if lvl is not None:  # without it the companion assumes level 2 (as the console before the bandwidth levels)
            self.hello["lvl"] = lvl
        self.cid = 0
        self.acks: dict = {}
        self.rx_bytes = collections.Counter()  # per message type, every datagram (chunks of incomplete messages too)

    def _send(self, typ: int, payload: bytes, flags: int = 0) -> None:
        self.msg = (self.msg + 1) & 0xFFFF
        n = max(1, -(-len(payload) // MAX_PAYLOAD))
        st = int(time.monotonic() * 1000) & 0xFFFFFFFF
        for i in range(n):
            self.sock.sendto(HDR.pack(MAGIC, typ, flags, self.msg, i, n, st) + payload[i * MAX_PAYLOAD:(i + 1) * MAX_PAYLOAD], self.addr)

    def send_json(self, typ: int, obj) -> None:
        self._send(typ, json.dumps(obj).encode())

    def hello_tick(self) -> None:
        self.send_json(T_HELLO, self.hello)

    def ping(self) -> None:
        self._send(T_PING, struct.pack("<d", time.monotonic()))

    def cmd(self, op: str, **kw) -> int:
        self.cid += 1
        self.send_json(T_CMD, {"id": self.cid, "op": op, **kw})
        return self.cid

    def estop(self, level: int) -> int:
        self.cid += 1
        self.send_json(T_ESTOP, {"id": self.cid, "level": level})
        return self.cid

    def topo(self) -> None:
        self._send(T_TOPO_REQ, b"")

    def recv(self):
        try:
            data, _ = self.sock.recvfrom(65535)
        except (socket.timeout, BlockingIOError):
            return None
        if len(data) < HDR.size:
            return None
        magic, typ, flags, msg, chunk, n, stamp = HDR.unpack_from(data)
        if magic != MAGIC:
            return None
        self.rx_bytes[NAMES.get(typ, str(typ))] += len(data)
        body = data[HDR.size:]
        if n > 1:
            now = time.monotonic()
            ent = self.parts.setdefault((typ, msg), [now, {}])
            ent[1][chunk] = body
            for k in [k for k, v in self.parts.items() if now - v[0] > 1.0]:
                del self.parts[k]
            if len(ent[1]) < n:
                return None
            body = b"".join(ent[1][i] for i in range(n))
            self.parts.pop((typ, msg), None)
        if flags & F_ZLIB:
            body = zlib.decompress(body)
        return typ, body, len(data)


def decode(typ: int, body: bytes) -> dict:
    """Message -> a small dict (what the console shows), for the summary and the tests."""
    if typ in (1, 7, 8):
        return json.loads(body)
    if typ == 2:
        k, n, rows, cols, nfeat = struct.unpack_from("<IHBBB", body)
        raw = zlib.decompress(body[9:])
        counts = np.frombuffer(raw[:n], np.uint8)
        grids = np.frombuffer(raw[n:], np.uint8).reshape(2, nfeat, rows, cols) if nfeat else None
        return {"tick": k, "n": n, "spiking": int((counts > 0).sum()), "spikes": int(counts.sum()),
                "grids": None if grids is None else grids.shape}
    if typ == 3:
        kind, w, h, no = struct.unpack_from("<BHHI", body)
        return {"kind": kind, "w": w, "h": h, "no": no, "jpeg": body[9:]}
    if typ == 4:
        hl = struct.unpack_from("<H", body)[0]
        hdr = json.loads(body[2:2 + hl])
        raw = zlib.decompress(body[2 + hl:])
        nv = hdr["nvox"]
        idx = np.cumsum(np.frombuffer(raw[:4 * nv], np.uint32).astype(np.int64))
        conf = np.frombuffer(raw[4 * nv:], np.uint8)
        n, nz, res, half = hdr["n"], hdr["nz"], hdr["res"], hdr["half"]
        k = idx % nz
        j = (idx // nz) % n
        i = idx // (nz * n)
        pts = np.stack([(i + 0.5) * res - half, (j + 0.5) * res - half, (k + 0.5) * res], axis=1) if nv else np.zeros((0, 3))
        return {**hdr, "points": pts, "conf": conf}
    if typ == 5:
        cnt = body[0]
        off, out = 1, {}
        for _ in range(cnt):
            b, n, age = struct.unpack_from("<hBB", body, off)
            off += 4
            f = np.frombuffer(body[off:off + 2 * n * n], np.float16).astype(np.float32)
            off += 2 * n * n
            out[b] = {"n": n, "age_ms": age * 10, "min": float(np.nanmin(f)) if np.isfinite(f).any() else None,
                      "valid": int(np.isfinite(f).sum())}
        return out
    if typ == 6:
        return json.loads(body)
    if typ == 9:
        t0, tp = struct.unpack_from("<dd", body)
        return {"rtt_ms": (time.monotonic() - t0) * 1000}
    return {}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=14700)
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--cam", default="brain", choices=["brain", "color", "off"])
    ap.add_argument("--cam-hz", type=float, default=5)
    ap.add_argument("--color-w", type=int, default=640)
    ap.add_argument("--color-q", type=int, default=60)
    ap.add_argument("--lvl", type=int, default=None, help="bandwidth level 0-3 to ask for (default: none sent = level 2)")
    ap.add_argument("--cmd", action="append", default=[], help="op [json args], sent 1 s after connecting, 1 s apart")
    ap.add_argument("--estop", type=int, default=0, help="1 land / 2 kill, repeated at 5 Hz from t = 2 s")
    ap.add_argument("--save-cam", default="")
    ap.add_argument("--json-out", default="")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    L = Link(a.host, a.port, cam=a.cam, cam_hz=a.cam_hz, lvl=a.lvl, color_w=a.color_w, color_q=a.color_q)
    t0 = time.monotonic()
    nxt_hello = nxt_ping = nxt_print = t0
    counts = collections.Counter()
    last = {}
    first_status = None  # (time, STATUS): the companion's send counters over the run
    rtts: list[float] = []
    acks: list[dict] = []
    events: dict = {}
    topo = None
    tick_ts: list[float] = []
    cam_ts: list[float] = []
    cam_kb: list[float] = []
    cam_last = [None, 0]  # previous frame's JPEG, frames equal to the one before
    pending = list(a.cmd)
    nxt_cmd = t0 + 1.0
    L.topo()
    while time.monotonic() - t0 < a.seconds:
        now = time.monotonic()
        if now >= nxt_hello:
            L.hello_tick()
            nxt_hello = now + 0.2
        if now >= nxt_ping:
            L.ping()
            nxt_ping = now + 0.5
        if pending and now >= nxt_cmd and counts["TICK"]:
            op, _, js = pending.pop(0).partition(" ")
            L.cmd(op, **(json.loads(js) if js.strip() else {}))
            nxt_cmd = now + 1.0
        if a.estop and now - t0 > 2.0 and int((now - t0) * 5) != int((now - t0 - 0.05) * 5):
            L.estop(a.estop)
        if topo is None and counts["TICK"] and now - t0 > 2 and int(now - t0) % 2 == 0:
            L.topo()
        r = L.recv()
        if r is not None:
            typ, body, _ = r
            name = NAMES.get(typ, str(typ))
            counts[name] += 1
            d = decode(typ, body)
            last[name] = d
            if name == "TICK":
                tick_ts.append(now)
            elif name == "PONG":
                rtts.append(d["rtt_ms"])
            elif name == "ACK":
                acks.append(d)
                if not a.quiet:
                    print(f"ACK {d}")
            elif name == "STATUS":
                if first_status is None:
                    first_status = (now, d)
                last_status_t = now
                for e in d.get("events", []):
                    if e[0] not in events:
                        events[e[0]] = e
                        if not a.quiet:
                            print(f"EVENT {e[2]}: {e[3]}")
            elif name == "TOPO":
                topo = d
            elif name == "CAM":
                cam_ts.append(now)
                cam_kb.append(len(d["jpeg"]) / 1024)
                cam_last[1] += d["jpeg"] == cam_last[0]
                cam_last[0] = d["jpeg"]
                if a.save_cam:
                    open(a.save_cam, "wb").write(d["jpeg"])
        if now >= nxt_print and not a.quiet:
            nxt_print = now + 1.0
            tk = last.get("TICK", {})
            mp = last.get("MAP", {})
            print(f"[{now - t0:5.1f}] " + " ".join(f"{k}:{v}" for k, v in sorted(counts.items()))
                  + f" | {sum(L.rx_bytes.values()) / max(1e-3, now - t0) / 1024:.0f} kB/s rtt {np.median(rtts[-10:]) if rtts else float('nan'):.1f} ms"
                  + f" | fc {tk.get('fc')} ctl {tk.get('ctl')} op {tk.get('op')} es {tk.get('es')} ms {tk.get('ms')}"
                  + f" | map {mp.get('nvox')} vox", flush=True)
    span = (tick_ts[-1] - tick_ts[0]) if len(tick_ts) > 1 else 0
    el = max(1e-3, time.monotonic() - t0)
    pi_tx = None
    if first_status is not None and last.get("STATUS") and last_status_t > first_status[0]:
        c0, c1 = first_status[1].get("counters", {}), last["STATUS"].get("counters", {})
        l0, l1 = first_status[1].get("link", {}), last["STATUS"].get("link", {})
        pi_tx = {"kB_per_s": round((c1.get("tx_kB", 0) - c0.get("tx_kB", 0)) / (last_status_t - first_status[0]), 1),
                 "skipped": l1.get("skipped", 0) - l0.get("skipped", 0), "level": l1.get("level")}
    cam_span = (cam_ts[-1] - cam_ts[0]) if len(cam_ts) > 1 else 0
    gaps = np.diff(cam_ts) * 1000 if len(cam_ts) > 1 else np.zeros(0)
    res = {
        "counts": dict(counts),
        "kB_per_s": round(sum(L.rx_bytes.values()) / el / 1024, 1),
        "kB_per_s_by_type": {k: round(v / el / 1024, 2) for k, v in sorted(L.rx_bytes.items())},
        "pi_tx": pi_tx,  # the companion's own send counter (all consoles) and bulky messages skipped over the run
        "tick_hz": round((len(tick_ts) - 1) / span, 2) if span > 0 else 0,
        "rtt_ms_p50": round(float(np.median(rtts)), 2) if rtts else None,
        "acks": acks,
        "events": [events[k] for k in sorted(events)],
        "last_tick": last.get("TICK"),
        "brain": last.get("BRAIN"),
        "tof": last.get("TOF"),
        "map_nvox": last.get("MAP", {}).get("nvox"),
        "map_frames": last.get("MAP", {}).get("frames"),
        "cam": {k: v for k, v in last.get("CAM", {}).items() if k != "jpeg"},
        # first to last frame; repeats = frames equal to the one before (a camera slower than the requested rate)
        "cam_rx": {"frames": len(cam_ts), "fps": round((len(cam_ts) - 1) / cam_span, 2) if cam_span > 0 else 0,
                   "repeats": cam_last[1], "kB_mean": round(float(np.mean(cam_kb)), 2) if cam_kb else None,
                   "gap_ms_p50": round(float(np.median(gaps)), 1) if len(gaps) else None,
                   "gap_ms_p95": round(float(np.percentile(gaps, 95)), 1) if len(gaps) else None,
                   "gap_ms_max": round(float(gaps.max()), 1) if len(gaps) else None},
        "rtt_ms_p95": round(float(np.percentile(rtts, 95)), 2) if rtts else None,
        "status": {k: v for k, v in (last.get("STATUS") or {}).items() if k != "events"},
        "topo": None if topo is None else {"n": topo["n"], "nnz": topo["nnz"], "pops": len(topo["pops"])},
    }
    if a.json_out:
        json.dump(res, open(a.json_out, "w"), indent=1, default=str)
    if not a.quiet:
        print(json.dumps({k: res[k] for k in ("counts", "kB_per_s", "tick_hz", "rtt_ms_p50", "map_nvox", "topo", "cam", "cam_rx")},
                         default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
