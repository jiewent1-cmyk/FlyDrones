"""Processor-in-the-loop link, Orange Pi side (2026-10-16): Gazebo on the simulation PC (ROG) instead of the camera and
the two VL53L7CX, the real companion pipeline on the Pi.

The simulation PC (ecps295/pil_rog.py, run_g4.py with ECPS_PIL=<pi>:<port>) sends UDP datagrams to --pil <port>:
camera frames (192x144 gray, as gazebo/cam_bridge.py), 8x8 multi-zone frames per sensor bearing (emulated from SIM_STATE
truth exactly as in the local Gazebo runs) and a 10 Hz control datagram with the 'go' flag. SITL sends MAVLink straight
to the companion (--url udpin:0.0.0.0:14561), so RealDrone, the hand-over gate, the route-B height and the watchdogs run
unchanged. The companion takes control only while go is set (patrol phase) and the last control datagram is < 0.5 s
old; every tick it answers with a status datagram (commands, DN rates, compute time) to the sender's address.

Datagram: <4sBBHId magic 'EPL1', type (1 cam, 2 tof, 3 ctl), go, aux (cam: width; tof: bearing deg), seq, sim time>
+ payload (cam: uint8 gray; type 4 cam: JPEG of the gray frame; tof: float32 metres, NaN = no target).
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

import cv2
import numpy as np

MAGIC = b"EPL1"
HDR = struct.Struct("<4sBBHId")
CAM, TOF, CTL, CAM_JPG = 1, 2, 3, 4


class PilCamera:
    """camera.py interface (start/stop/latest, frames, decode_ms, status), fed by the link."""

    def __init__(self):
        self.img = None
        self.stamp = 0.0
        self.frames = 0
        self.errors = 0
        self.decode_ms: list[float] = []
        self.status = "pil: waiting"

    def latest(self):
        return self.img, self.stamp

    def start(self):
        return self

    def stop(self):
        pass


class PilToF:
    """tof_vl53l7cx interface (frame / stamp / age_s / counters), fed by the link."""

    def __init__(self, name: str, n: int = 8, max_m: float = 3.5, **_):
        self.name, self.n, self.max_m = name, n, max_m
        self.frame = None
        self.stamp = 0.0
        self.frames = 0
        self.errors = 0
        self.read_ms: list[float] = []
        self.status = "pil: waiting"
        self.init_s = 0.0

    def age_s(self) -> float:
        return time.monotonic() - self.stamp if self.stamp else float("inf")

    def start(self):
        return self

    def stop(self):
        pass


class PilLink:
    def __init__(self, port: int, timeout_s: float = 0.5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.1)
        self.timeout_s = timeout_s
        self.cam = PilCamera()
        self.tofs: dict[int, PilToF] = {}
        self.peer = None
        self._go = 0
        self._ctl_t = 0.0
        self.ctl_seq = -1
        self.sim_t = 0.0
        self.rx = {"cam": 0, "tof": 0, "ctl": 0, "bad": 0}
        self.go_changes: list = []
        self.finished = False
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, name="pil-rx", daemon=True)

    def tof(self, bearing: int, name: str, **cfg) -> PilToF:
        s = self.tofs[int(bearing) % 360] = PilToF(name, **{k: cfg[k] for k in ("n", "max_m") if k in cfg})
        return s

    @property
    def go(self) -> bool:
        return bool(self._go) and time.monotonic() - self._ctl_t < self.timeout_s

    def start(self) -> "PilLink":
        self._th.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._th.join(timeout=1)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                continue
            if len(data) < HDR.size or data[:4] != MAGIC:
                self.rx["bad"] += 1
                continue
            _, kind, go, aux, seq, sim_t = HDR.unpack_from(data, 0)
            body = data[HDR.size:]
            now = time.monotonic()
            self.peer, self.sim_t = addr, sim_t
            if go != self._go:
                self.go_changes.append((round(now, 3), go))
                if self._go and not go:
                    self.finished = True  # patrol over: the companion exits (no remote kill needed)
            self._go, self._ctl_t = go, now  # every datagram carries the flag
            if kind in (CAM, CAM_JPG) and aux:
                c = self.cam
                t0 = time.perf_counter()
                if kind == CAM_JPG:  # as the real MJPEG camera: the decode runs on the Pi
                    img = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_GRAYSCALE)
                    if img is None:
                        self.rx["bad"] += 1
                        continue
                    if len(c.decode_ms) < 50000:
                        c.decode_ms.append((time.perf_counter() - t0) * 1000)
                    c.img = img
                else:
                    c.img = np.frombuffer(body, np.uint8).reshape(-1, aux).copy()
                c.stamp, c.status = now, "pil"
                c.frames += 1
                self.rx["cam"] += 1
            elif kind == TOF:
                s = self.tofs.get(aux % 360)
                if s is not None:
                    s.frame = np.frombuffer(body, np.float32).astype(float)  # new object per frame (map update)
                    s.stamp, s.status = now, "pil"
                    s.frames += 1
                self.rx["tof"] += 1
            elif kind == CTL:
                self.ctl_seq = seq
                self.rx["ctl"] += 1
                try:  # immediate echo: the sender measures the network round trip
                    self.sock.sendto(json.dumps({"ctl_seq": seq}).encode(), addr)
                except OSError:
                    pass

    def send_status(self, st: dict) -> None:
        if self.peer is None:
            return
        st["ctl_seq"] = self.ctl_seq
        try:
            self.sock.sendto(json.dumps(st, separators=(",", ":")).encode(), self.peer)
        except OSError:
            pass
