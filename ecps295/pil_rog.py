"""Processor-in-the-loop link, ROG side (2026-10-16).

The brain, safety layer and MAVLink backend run on the Orange Pi (companion/ecps_companion.py --pil); Gazebo + SITL run
here. This end streams what the Pi's sensors would see:

  camera  the Gazebo camera frame from the cam_bridge shm (192x144 gray, as gazebo/cam_bridge.py), on every new frame,
          JPEG-encoded (q 85, ~5 kB) unless ECPS_PIL_RAW=1
  ToF     the S7 multi-zone frames (front / rear 8x8) emulated from SIM_STATE truth by mavlink_twin.zones(), with the
          same sample-and-hold rate and noise as in the local Gazebo runs
  ctl     10 Hz: the 'go' flag (1 only during the patrol phase; the Pi takes control only while it is set)

The Pi flies the drone itself over MAVLink (SITL --serial2=udpclient:<pi>:14561) and answers every tick with a status
datagram (commands, DN rates, compute time), which run_g4 logs in place of the local pilot's tick. All traffic is
started from here, so ROG's firewall admits the replies.

Datagram: HDR + payload. type 1 cam: aux = width, payload = gray bytes; type 2 tof: aux = bearing (deg), payload =
float32 zones (NaN = no target); type 3 ctl: no payload.
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np

from shm_frames import ShmFrame

MAGIC = b"EPL1"
HDR = struct.Struct("<4sBBHId")  # magic, type, go, aux, seq, sim time (s)
CAM, TOF, CTL, CAM_JPG = 1, 2, 3, 4
JPEG_Q = 85  # 2026-10-16: JPEG like the real MJPEG camera; raw 28 kB frames (~20 IP fragments) stalled the home WiFi for seconds


class PilRog:
    def __init__(self, target: str, drone, shm: str, sensors: list[dict], hz: float = 200.0, jpeg: bool = True):
        host, port = target.rsplit(":", 1)
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.05)
        self.drone, self.cam, self.sensors, self.dt = drone, ShmFrame(shm), list(sensors), 1.0 / hz
        self.jpeg, self.bytes_cam = jpeg, 0
        self.go = 0
        self.sim_t = 0.0
        self.seq = 0
        self.ctl_sent: dict[int, float] = {}  # ctl seq -> send time, for the round trip
        self.rtt_ms: list[float] = []
        self.sent = {"cam": 0, "tof": 0, "ctl": 0}
        self.status: dict | None = None
        self.status_n = 0
        self.status_t = 0.0
        self.in_control_n = 0
        self.pi_compute_ms: list[float] = []
        self._stop = threading.Event()
        self._th: list[threading.Thread] = []

    # ------------------------------------------------------------------ threads
    def start(self) -> "PilRog":
        for fn in (self._tx, self._rx):
            th = threading.Thread(target=fn, name=f"pil{fn.__name__}", daemon=True)
            th.start()
            self._th.append(th)
        return self

    def stop(self) -> None:
        self.go = 0
        for _ in range(3):  # make sure the Pi sees go = 0
            self._send(CTL, 0, b"")
            time.sleep(0.05)
        self._stop.set()
        for th in self._th:
            th.join(timeout=1)

    def _send(self, kind: int, aux: int, payload: bytes) -> None:
        self.seq += 1
        if kind == CTL:
            self.ctl_sent[self.seq] = time.monotonic()
            if len(self.ctl_sent) > 200:
                for k in sorted(self.ctl_sent)[:100]:
                    self.ctl_sent.pop(k, None)
        try:
            self.sock.sendto(HDR.pack(MAGIC, kind, self.go, aux & 0xFFFF, self.seq, self.sim_t) + payload, self.addr)
        except OSError:
            pass

    def _tx(self) -> None:
        last = [None] * len(self.sensors)
        last_ctl = 0.0
        while not self._stop.is_set():
            t0 = time.monotonic()
            if self.cam.read() and self.cam.img is not None:
                img = self.cam.img if self.cam.img.ndim == 2 else self.cam.img[..., 0]
                if self.jpeg:
                    ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(img, np.uint8), [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                    self._send(CAM_JPG, img.shape[1], buf.tobytes())
                    self.bytes_cam += len(buf)
                else:
                    self._send(CAM, img.shape[1], np.ascontiguousarray(img, np.uint8).tobytes())
                    self.bytes_cam += img.size
                self.sent["cam"] += 1
            for i, sc in enumerate(self.sensors):
                r = self.drone.zones(**sc)
                if r is not None and r is not last[i]:
                    last[i] = r
                    self._send(TOF, int(round(sc["bearing"])) % 360, np.asarray(r, np.float32).tobytes())
                    self.sent["tof"] += 1
            if t0 - last_ctl >= 0.1:
                last_ctl = t0
                self._send(CTL, 0, b"")
                self.sent["ctl"] += 1
            self._stop.wait(max(0.0, self.dt - (time.monotonic() - t0)))

    def _rx(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                continue
            try:
                st = json.loads(data)
            except ValueError:
                continue
            now = time.monotonic()
            sent = self.ctl_sent.pop(st.get("ctl_seq", -1), None)
            if sent is not None and len(self.rtt_ms) < 100000:
                self.rtt_ms.append((now - sent) * 1000)
            if "cmd" in st:
                self.status, self.status_t = st, now
                self.status_n += 1
                self.in_control_n += int(st.get("ctl", 0))
                if len(self.pi_compute_ms) < 100000:
                    self.pi_compute_ms.append(float(st.get("cms", float("nan"))))

    # ------------------------------------------------------------------ run_g4 hooks
    def info(self, tel):
        """A stand-in for the local pilot's TickInfo, from the Pi's latest status (hover if none yet)."""
        st = self.status or {}
        c = st.get("cmd", [0.0, 0.0, 0.0, 0.0, 0])
        r = st.get("raw", [0.0, 0.0, 0.0])
        cmd = SimpleNamespace(forward=c[0], lateral=c[1], yaw=c[2], throttle=c[3], escape=bool(c[4]), note=st.get("note", ""))
        raw = SimpleNamespace(forward=r[0], yaw=r[1], throttle=r[2])
        return SimpleNamespace(tel=tel, cmd=cmd, raw=raw, rates=st.get("rates", {}), rtf=float(st.get("rtf", 0.0)))

    def row(self) -> dict:
        st = self.status or {}
        return {
            "pil_ctl": int(st.get("ctl", 0)),
            "pil_age_ms": round((time.monotonic() - self.status_t) * 1000, 1) if self.status_t else "",
            "pil_cms": st.get("cms", ""),
            "pil_frame_age_ms": st.get("fage", ""),
            "pil_tof_age_ms": st.get("tage", ""),
        }

    def summary(self) -> dict:
        def pct(x):
            x = np.asarray([v for v in x if np.isfinite(v)], float)
            return [round(float(np.percentile(x, q)), 2) for q in (50, 95, 99, 100)] if len(x) else []

        return {
            "pil_target": f"{self.addr[0]}:{self.addr[1]}",
            "pil_sent": self.sent,
            "pil_cam_jpeg": self.jpeg,
            "pil_cam_kB_per_frame": round(self.bytes_cam / max(1, self.sent["cam"]) / 1000, 1),
            "pil_status_n": self.status_n,
            "pil_in_control_ticks": self.in_control_n,
            "pil_rtt_ms_p50_p95_p99_max": pct(self.rtt_ms),
            "pil_compute_ms_p50_p95_p99_max": pct(self.pi_compute_ms),
        }
