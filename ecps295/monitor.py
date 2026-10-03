"""ECPS295 monitor: MiniFly dashboard (left) + live Gazebo view (right) in one window.

Runs on its own and survives between runs: it connects to the tick stream that run_g4.py publishes on
127.0.0.1:5799 and reads the chase / overview camera frames that gazebo/cam_bridge.py puts in shared memory.
The dashboard is drawn in a background thread, the Gazebo view refreshes at ~30 fps; the flight loop never waits.

    conda activate flydrones; PYTHONPATH=~/sim/FlyDrones/ecps295 python monitor.py      (keys: c = switch camera, q = quit)
"""

from __future__ import annotations

import argparse
import queue
import threading
import time
import types
from multiprocessing.connection import Client

import cv2
import numpy as np
from shm_frames import ShmFrame

from flydrones.brain import Brain, load_connectome

AUTH = b"ecps295"
VIEWS = {"chase": "/dev/shm/ecps295_chase", "overview": "/dev/shm/ecps295_overview"}
BG = (31, 22, 13)  # BGR, close to the dashboard background
FG, MUTED, WARN = (240, 240, 240), (150, 150, 150), (80, 80, 255)
H = 540

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=5799)
ap.add_argument("--view", choices=list(VIEWS), default="chase")
ap.add_argument("--fps", type=float, default=30)
a = ap.parse_args()


def text(img, s, xy, scale=0.6, color=FG, thick=1):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def fit_height(img: np.ndarray, h: int) -> np.ndarray:
    if img.shape[0] == h:
        return img
    w = int(round(img.shape[1] * h / img.shape[0]))
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


class Session:
    """One run: receives ticks, renders the dashboard off the GUI thread."""

    def __init__(self, conn, hello: dict):
        from world_layout import room_from_layout

        from flydrones.viz import Dashboard

        cfg = hello["cfg"]
        brain = Brain(load_connectome(cfg["brain"]["source"]), cfg)  # same seed -> same recorded neurons
        room = room_from_layout(hello["layout"]) if hello.get("layout") else None
        stub = types.SimpleNamespace(name="fly-1", drone=types.SimpleNamespace(room=room))
        self.dash = Dashboard(brain, [stub], title=hello["title"])
        self.conn = conn
        self.q: queue.Queue = queue.Queue()
        self.last_info = None
        self.dash_img: np.ndarray | None = None
        self.ended = False
        self.summary: dict | None = None
        self.lock = threading.Lock()
        threading.Thread(target=self._recv, daemon=True).start()
        threading.Thread(target=self._render, daemon=True).start()

    def _recv(self):
        try:
            while True:
                kind, payload = self.conn.recv()
                if kind == "tick":
                    self.q.put(payload)
                elif kind == "end":
                    self.summary = payload
                    break
        except (EOFError, OSError):
            pass
        self.ended = True
        self.q.put(None)

    def _render(self):
        while True:
            items = [self.q.get()]
            while not self.q.empty():
                items.append(self.q.get_nowait())
            stop = None in items
            items = [i for i in items if i is not None]
            if items:
                for it in items[:-1]:
                    self.dash.push([it])
                rgb = self.dash.render([items[-1]])
                with self.lock:
                    self.dash_img, self.last_info = rgb[..., ::-1].copy(), items[-1]
            if stop:
                return


def connect(port: int):
    try:
        conn = Client(("127.0.0.1", port), authkey=AUTH)
    except (ConnectionRefusedError, OSError):
        return None, None
    kind, hello = conn.recv()
    return (conn, hello) if kind == "hello" else (None, None)


def placeholder(w: int, msg: str) -> np.ndarray:
    img = np.full((H, w, 3), BG, np.uint8)
    text(img, msg, (30, H // 2), 0.7, MUTED)
    return img


views = {k: ShmFrame(p) for k, p in VIEWS.items()}
view = a.view
session: Session | None = None
last_try = 0.0
win = "ECPS295 monitor - MiniFly brain | Gazebo"
cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)  # resizable, no Qt toolbar
cv2.resizeWindow(win, 1280 + 960, H)
period = 1.0 / a.fps
while True:
    t0 = time.monotonic()
    if (session is None or session.ended) and t0 - last_try > 1.0:
        last_try = t0
        conn, hello = connect(a.port)
        if conn is not None:
            session = Session(conn, hello)
            for v in views.values():
                v.reopen()
    # left: dashboard
    if session is not None and session.dash_img is not None:
        with session.lock:
            left, info = fit_height(session.dash_img, H), session.last_info
        if session.ended:
            left = left.copy()
            text(left, "run finished - waiting for the next run_g4.py", (20, H - 15), 0.6, WARN)
    else:
        left, info = placeholder(1280, f"waiting for run_g4.py (tick stream on 127.0.0.1:{a.port}) ..."), None
    # right: Gazebo view
    v = views[view]
    v.read()
    if v.img is not None:
        right = fit_height(v.img[..., ::-1], H).copy()
        text(right, f"Gazebo {view} camera   sim t={v.stamp:7.1f} s   [c] switch  [q] quit", (15, 28), 0.6)
        if info is not None:
            tel = info.tel
            line = f"alt {tel.alt_m:.2f} m   thr {info.cmd.throttle:+.2f}  yaw {info.cmd.yaw:+.2f}  fwd {info.cmd.forward:+.2f}"
            text(right, line, (15, H - 18), 0.6)
            if info.cmd.escape:
                text(right, "ESCAPE", (right.shape[1] - 150, 60), 1.0, WARN, 2)
    else:
        right = placeholder(960, f"no Gazebo {view} camera frames yet")
    cv2.imshow(win, np.hstack([left, right]))
    key = cv2.waitKey(max(1, int((period - (time.monotonic() - t0)) * 1000))) & 0xFF
    if key == ord("q") or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
        break
    if key == ord("c"):
        view = "overview" if view == "chase" else "chase"
cv2.destroyAllWindows()
