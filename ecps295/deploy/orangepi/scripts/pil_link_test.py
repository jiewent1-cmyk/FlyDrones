"""Offline test of the PIL link's datagram order handling (companion/pil_link.py, 2026-10-21).

A real PilLink on a loopback port; the test plays the simulation PC (pil_rog.py) and sends the datagrams in the order
they arrive. Case 1 is the one that ended PIL run pil4v s11 on 2026-10-21: a camera JPEG with go = 0 arrived right after
the control datagram with go = 1, so the old link saw go 1 -> 0 -> 1, took that for the end of the patrol and the
companion exited at the patrol start.

    python scripts/pil_link_test.py [pil_link.py]   (default: ../companion/pil_link.py next to this file)

Needs numpy and OpenCV (JPEG camera frames), as the companion does.
"""
import importlib.util
import json
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np

SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "companion" / "pil_link.py"
spec = importlib.util.spec_from_file_location("pil_link", SRC)
pl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pl)
print(f"pil_link: {SRC}")

results = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""), flush=True)


class Sender:
    """The simulation PC's socket; each one binds its own port, as each pil_rog.py run does."""

    def __init__(self, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.addr = ("127.0.0.1", port)

    def send(self, kind: int, go: int, seq: int, aux: int = 0, payload: bytes = b"") -> None:
        self.sock.sendto(pl.HDR.pack(pl.MAGIC, kind, go, aux, seq, seq * 0.005) + payload, self.addr)

    def cam(self, go: int, seq: int, value: int) -> None:  # flat 192 x 144 gray JPEG; its value tells the frames apart
        _, buf = cv2.imencode(".jpg", np.full((144, 192), value, np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 80])
        self.send(pl.CAM_JPG, go, seq, 192, buf.tobytes())

    def tof(self, go: int, seq: int, bearing: int, metres: float) -> None:
        self.send(pl.TOF, go, seq, bearing, np.full(64, metres, np.float32).tobytes())

    def ctl(self, go: int, seq: int) -> None:
        self.send(pl.CTL, go, seq)

    def echoes(self) -> list:
        got = []
        try:
            while True:
                got.append(json.loads(self.sock.recv(65535))["ctl_seq"])
        except socket.timeout:
            return got


def make(timeout_s: float = 0.5):
    link = pl.PilLink(0, timeout_s=timeout_s).start()
    link.tof(0, "front")
    link.tof(180, "rear")
    return link, link.sock.getsockname()[1]


def settle(link, n: int) -> None:
    """Wait until the link has taken n datagrams (each one lands in exactly one rx counter)."""
    t0 = time.monotonic()
    while sum(link.rx.values()) < n and time.monotonic() - t0 < 2.0:
        time.sleep(0.002)


def close(link, *senders) -> None:
    link.stop()
    link.sock.close()
    for s in senders:
        s.sock.close()


def gos(link) -> list:
    return [g for _, g in link.go_changes]


def img(link) -> int:
    return -1 if link.cam.img is None else int(link.cam.img[0, 0])


# ---------------------------------------------------------------- 1 the 10-21 run: late camera frame with go = 0
link, port = make()
s = Sender(port)
s.ctl(0, 1)
s.cam(0, 2, 40)
s.tof(0, 3, 0, 2.0)
s.ctl(1, 5)  # patrol start; camera frame 4 (sent before it, go = 0) arrives after it
s.cam(0, 4, 90)
s.cam(1, 6, 200)
s.ctl(1, 7)
settle(link, 7)
check("patrol start with a late camera frame (go = 0) after go = 1: go stays set, the run goes on",
      link.go and not link.finished and gos(link) == [1], f"go changes {gos(link)}, finished {link.finished}")
# ---------------------------------------------------------------- 2 the patrol end still ends the run
s.ctl(0, 8)
settle(link, 8)
check("patrol end (control datagram with go = 0): finished", link.finished and not link.go and gos(link) == [1, 0],
      f"go changes {gos(link)}")
close(link, s)

# ---------------------------------------------------------------- 3 a late control datagram
link, port = make()
s = Sender(port)
s.ctl(0, 1)
s.ctl(1, 3)
s.ctl(0, 2)  # sent before 3, arrives after it
settle(link, 3)
check("a late control datagram (go = 0 after go = 1) does not change go", link.go and not link.finished and gos(link) == [1],
      f"go changes {gos(link)}, finished {link.finished}")
e = s.echoes()
check("every control datagram is echoed at once, late ones too (the sender's round trip)", sorted(e) == [1, 2, 3], f"echoes {e}")
close(link, s)

# ---------------------------------------------------------------- 4 go only from control datagrams
link, port = make()
s = Sender(port)
s.ctl(0, 1)
s.cam(1, 2, 100)
s.tof(1, 3, 0, 1.5)
settle(link, 3)
check("camera / ToF datagrams with go = 1 do not set go", not link.go and gos(link) == [], f"go changes {gos(link)}")
close(link, s)

# ---------------------------------------------------------------- 5 liveness: control datagrams only
link, port = make(timeout_s=0.3)
s = Sender(port)
s.ctl(1, 1)
settle(link, 1)
go0 = link.go
for i in range(30):  # 0.6 s of camera frames with go = 1, no control datagram
    s.cam(1, 2 + i, 120)
    time.sleep(0.02)
settle(link, 31)
check("without control datagrams go times out (0.3 s here), camera frames do not keep it", go0 and not link.go,
      f"go {go0} after the control datagram, {link.go} after 0.6 s of camera frames")
close(link, s)

# ---------------------------------------------------------------- 6 a late camera frame is not used
link, port = make()
s = Sender(port)
s.cam(0, 2, 200)
s.cam(0, 1, 60)  # older frame, arrives last
settle(link, 2)
check("a late camera frame is dropped: the image stays the newest one",
      abs(img(link) - 200) <= 3 and link.cam.frames == 1 and link.rx.get("late", 0) == 1,
      f"image value {img(link)}, frames {link.cam.frames}, late {link.rx.get('late', 0)}")
close(link, s)

# ---------------------------------------------------------------- 7 order per stream
link, port = make()
s = Sender(port)
s.tof(0, 5, 0, 1.0)
s.tof(0, 3, 180, 2.0)  # other bearing: its own order
s.cam(0, 4, 120)  # older than a ToF frame: the camera has its own order
s.tof(0, 2, 0, 3.0)  # late for bearing 0
settle(link, 4)
f, r = link.tofs[0], link.tofs[180]
check("order is per stream: older datagrams of another stream are kept, a late one of the same bearing is dropped",
      r.frames == 1 and link.cam.frames == 1 and f.frames == 1 and f.frame is not None and f.frame[0] == 1.0,
      f"rear {r.frames}, camera {link.cam.frames}, front {f.frames} frames at {None if f.frame is None else f.frame[0]} m")
close(link, s)

# ---------------------------------------------------------------- 8 a new sender starts afresh
link, port = make()
s1 = Sender(port)
s1.ctl(0, 500)
s1.cam(0, 501, 50)
settle(link, 2)
s2 = Sender(port)  # next run: new port, seq from 1
s2.cam(0, 1, 150)
s2.ctl(1, 2)
settle(link, 4)
check("a new sender (new port, seq from 1) is taken at once", abs(img(link) - 150) <= 3 and link.go,
      f"image value {img(link)}, go {link.go}")
close(link, s1, s2)

# ---------------------------------------------------------------- 9 seq far back on the same port: a restarted sender
link, port = make()
s = Sender(port)
s.cam(0, 5000, 50)
s.cam(0, 3, 150)
settle(link, 2)
check("seq more than 1000 back on the same port counts as a restarted sender, not as late",
      abs(img(link) - 150) <= 3 and link.rx.get("late", 0) == 0, f"image value {img(link)}, late {link.rx.get('late', 0)}")
close(link, s)

print("ALL PASS" if all(results) else f"SOME FAILED ({results.count(False)} of {len(results)})")
sys.exit(0 if all(results) else 1)
