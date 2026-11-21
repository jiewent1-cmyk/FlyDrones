"""Offline test of the console's still command (console_link.ConsoleLink._still, camera.SurveyCamera, 2026-11-03).

No FC, no brain, no camera device: the survey camera is the real SurveyCamera with source sim, reading a shared-memory
file in gazebo/cam_bridge.py's format that a thread here keeps writing. Commands go through the real datagram handler,
signed as a console signs them; what the companion sends is caught before signing and reassembled as the console would.

    python tools/still_test.py [companion dir]   (default: ../../orangepi/companion next to this file)

flydrones is stubbed when it is not installed (FlightCommand); numpy and OpenCV are needed.
"""
import hashlib
import hmac
import json
import mmap
import os
import struct
import sys
import tempfile
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

COMP = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2] / "orangepi" / "companion"
sys.path[:0] = [str(COMP), str(COMP.parents[2])]  # the companion, and ecps295/ for shm_frames
try:
    import flydrones.motor.command  # noqa: F401
except ImportError:

    @dataclass
    class FlightCommand:
        throttle: float = 0.5
        yaw: float = 0.0
        forward: float = 0.0
        lateral: float = 0.0
        escape: bool = False
        note: str = ""

    for name in ("flydrones", "flydrones.motor", "flydrones.motor.command", "flydrones.senses", "flydrones.senses.gestures"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["flydrones.motor.command"].FlightCommand = FlightCommand
    sys.modules["flydrones.senses.gestures"].GestureState = tuple
import camera  # noqa: E402
import console_link as cl  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail else ""))


class Pilot:
    def __init__(self):
        self.wp_on, self.wp_active, self.wp_done_t, self.wp_log = False, True, None, []
        self.safety = types.SimpleNamespace(land_requested=False)
        self.brain = types.SimpleNamespace(last_counts=None)
        self.gestures = None

    def set_waypoints(self, points):
        self.wp_on = True


class Drone:
    def armed(self):
        return False


ADDR = ("127.0.0.1", 9)
SEQ = [0]


def signed(typ, obj, key=b""):
    """One datagram as the console sends it (ecl_client.py): F_AUTH, then <QQ seq, Unix ms> and the HMAC."""
    body = json.dumps(obj).encode()
    SEQ[0] += 1
    d = cl.HDR.pack(cl.MAGIC, typ, cl.F_AUTH, SEQ[0] & 0xFFFF, 0, 1, 0) + body
    d += cl.AUTH_FIELDS.pack(SEQ[0], int(time.time() * 1000))
    return d + hmac.new(key, d, hashlib.sha256).digest()


def make(survey_cam=None):
    link = cl.ConsoleLink(0, pilot=Pilot(), drone=Drone(), cam=None, tofs={}, cfg={"safety": {"geofence_radius_m": 2.0}},
                          console_cfg={}, run_info={}, log=lambda *a: None, survey_cam=survey_cam)
    link.out = []  # (type, payload) of every message, reassembled
    link.chunks = []

    def send(addr, dgrams):
        body = b"".join(g[cl.HDR.size:] for g in dgrams)
        typ = cl.HDR.unpack_from(dgrams[0])[1]
        link.chunks.append((typ, len(dgrams), max(len(g) for g in dgrams)))
        link.out.append((typ, body))

    link._send = send
    link._handle(signed(cl.T_HELLO, {"name": "test", "cam": "off"}), ADDR)
    return link


def cmd(link, **m):
    n = len(link.out)
    link._handle(signed(cl.T_CMD, m), ADDR)
    acks = [json.loads(b) for t, b in link.out[n:] if t == cl.T_ACK]
    return acks[-1] if acks else None


def stills(link, timeout=5.0, n=1):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        got = [b for t, b in link.out if t == cl.T_STILL]
        if len(got) >= n and not link._still_busy.is_set():
            return got
        time.sleep(0.02)
    return [b for t, b in link.out if t == cl.T_STILL]


class ShmWriter:
    """gazebo/cam_bridge.py's seqlock file, one new frame every period_s."""

    HDR = struct.Struct("<QdIII")

    def __init__(self, path, img, period_s=0.2):
        self.img, self.period = img, period_s
        h, w = img.shape[:2]
        n = self.HDR.size + img.size
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
        os.ftruncate(fd, n)
        self.mm = mmap.mmap(fd, n)
        self.seq, self.stop = 0, threading.Event()
        self.w, self.h = w, h
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while not self.stop.is_set():
            self.seq += 1
            self.mm[:8] = struct.pack("<Q", self.seq)
            self.mm[self.HDR.size:] = self.img.tobytes()
            self.seq += 1
            self.mm[:self.HDR.size] = self.HDR.pack(self.seq, time.time(), self.w, self.h, 3)
            self.stop.wait(self.period)


tmp = tempfile.mkdtemp()
# a site-like frame (RGB as cam_bridge writes it): a floor, dark panels, AprilTags; compresses as a real render does
W, H = 2592, 1944
img = np.full((H, W, 3), 200, np.uint8)
img = np.clip(img + np.random.default_rng(0).normal(0, 4, img.shape), 0, 255).astype(np.uint8)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
for i, (x, y) in enumerate([(200, 200), (2200, 200), (2200, 1600), (200, 1600)]):
    img[y:y + 200, x:x + 200] = cv2.cvtColor(cv2.aruco.generateImageMarker(d, i + 1, 200), cv2.COLOR_GRAY2RGB)
cv2.rectangle(img, (800, 700), (1700, 1300), (25, 40, 70), -1)
shm = os.path.join(tmp, "survey_cam")
writer = ShmWriter(shm, img)

# ---------------------------------------------------------------- 1 no survey camera: ACK not ok, nothing else sent
assert camera.make_survey(None) is None and camera.make_survey({"source": "off"}) is None
k0 = make(None)
n0 = len(k0.out)
ack = cmd(k0, id=5, op="still")
time.sleep(0.3)
check("no survey_cam section: still is refused with an ACK, no STILL is sent",
      ack is not None and ack["id"] == 5 and ack["op"] == "still" and ack["ok"] is False and "survey" in ack["msg"]
      and not [t for t, _ in k0.out[n0:] if t not in (cl.T_ACK,)], f"{ack}")
ack = cmd(k0, id=6, op="hold")
check("no survey_cam section: other commands are answered as before", ack == {"id": 6, "op": "hold", "ok": True, "msg": ""},
      f"{ack}")

# ---------------------------------------------------------------- 2 a still: ACK ok, then STILL <IHH id, w, h> + JPEG
sc = camera.make_survey({"source": "sim", "shm": shm, "width": W, "height": H, "quality": 90})
k = make(sc)
t0 = time.monotonic()
ack = cmd(k, id=41, op="still")
t_ack = time.monotonic() - t0
got = stills(k)
ok = ack is not None and ack["ok"] is True and len(got) == 1
sid = w = h = None
dec = None
if got:
    sid, w, h = struct.unpack_from("<IHH", got[0])
    dec = cv2.imdecode(np.frombuffer(got[0][8:], np.uint8), cv2.IMREAD_COLOR)
st = [c for c in k.chunks if c[0] == cl.T_STILL]
check("still: ACK ok at once, then one STILL with the command's id and the frame's size",
      ok and (sid, w, h) == (41, W, H) and dec is not None and dec.shape == (H, W, 3) and t_ack < 0.1,
      f"ack {ack} in {t_ack * 1000:.0f} ms, still {(sid, w, h)}, {len(got[0]) if got else 0} bytes")
check("still: chunked within the link's limits", st and st[0][1] <= cl.MAX_CHUNKS and st[0][2] <= cl.HDR.size + cl.MAX_PAYLOAD
      and len(got[0]) - 8 <= sc.max_bytes, f"{st}")
# colour order: cam_bridge writes RGB, the JPEG is BGR as OpenCV writes it; the dark panel must come back (70, 40, 25) BGR
px = dec[1000, 1250].astype(int) if dec is not None else None
check("still: colours in the right order (RGB shared memory -> BGR JPEG)", px is not None and abs(px[0] - 70) < 12
      and abs(px[2] - 25) < 12, f"{px}")
tags = cv2.aruco.ArucoDetector(d).detectMarkers(dec)[1] if dec is not None else None
check("still: the AprilTags survive the JPEG", tags is not None and sorted(tags.ravel().tolist()) == [1, 2, 3, 4], f"{tags}")

# ---------------------------------------------------------------- 3 a frame taken after the request, not before
writer.stop.set()
time.sleep(0.3)
ack = cmd(k, id=42, op="still")
time.sleep(0.2)
n_before = len([1 for t, _ in k.out if t == cl.T_STILL])
t0 = time.monotonic()
while k._still_busy.is_set() and time.monotonic() - t0 < 5:
    time.sleep(0.05)
n_after = len([1 for t, _ in k.out if t == cl.T_STILL])
check("still with a stalled camera: ACK ok, no STILL (no new frame), the failure is logged, the link stays free",
      ack["ok"] and n_before == n_after == 1 and sc.errors == 1 and not k._still_busy.is_set(), f"{ack}, errors {sc.errors}")
writer = ShmWriter(shm, img)

# ---------------------------------------------------------------- 4 one at a time
a1 = cmd(k, id=43, op="still")
a2 = cmd(k, id=44, op="still")
got = stills(k, n=2)
check("a second still while one is being taken is refused", a1["ok"] and not a2["ok"] and "being taken" in a2["msg"]
      and struct.unpack_from("<I", got[-1])[0] == 43, f"{a1} {a2}")

# ---------------------------------------------------------------- 5 too big to send: nothing half-sent
noise = np.random.default_rng(1).integers(0, 256, (H, W, 3), np.uint8)
writer.stop.set()
writer = ShmWriter(shm, noise)
n0 = len([1 for t, _ in k.out if t == cl.T_STILL])
e0 = sc.errors
ack = cmd(k, id=45, op="still")
t0 = time.monotonic()
time.sleep(0.2)
while k._still_busy.is_set() and time.monotonic() - t0 < 10:
    time.sleep(0.05)
check("a frame whose JPEG cannot fit max_bytes: no STILL, an error", ack["ok"] and sc.errors == e0 + 1
      and len([1 for t, _ in k.out if t == cl.T_STILL]) == n0, f"errors {sc.errors}")
writer.stop.set()
writer = ShmWriter(shm, img)

# ---------------------------------------------------------------- 6 emergency stop: refused like every guidance command
k._handle(signed(cl.T_ESTOP, {"id": 46, "level": 1, "trig": 1}), ADDR)
ack = cmd(k, id=47, op="still")
check("still refused while the emergency stop is latched", k.estop == 1 and not ack["ok"] and "emergency" in ack["msg"], f"{ack}")

# ---------------------------------------------------------------- 7 bad id
k.estop = 0
ack = cmd(k, id="x", op="still")
check("still without an integer id is refused", not ack["ok"], f"{ack}")
writer.stop.set()
for link in (k0, k):
    link.sock.close()
print("ALL PASS" if all(results) else "SOME FAILED")
sys.exit(0 if all(results) else 1)
