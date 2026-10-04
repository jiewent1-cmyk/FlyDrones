"""ECPS295 monitor: MiniFly dashboard + live Gazebo views, filling the whole screen.

Layout (any window size, no empty margins):
    +---------------------------+------------------------+
    |                           | Gazebo chase camera    |  16:9
    |  MiniFly dashboard        +------------------------+
    |  (re-rendered at the      | drone camera, colour   |  4:3  (what the fly sees, before
    |   size of this area)      | 640x480 fisheye        |        grayscale / downsampling)
    +---------------------------+------------------------+

Runs on its own and survives between runs: it connects to the tick stream that run_g4.py publishes on
127.0.0.1:5799 and reads camera frames that gazebo/cam_bridge.py puts in shared memory. The dashboard is drawn
in a background thread, the camera panels refresh at ~30 fps; the flight loop never waits for it.

    conda activate flydrones; PYTHONPATH=~/sim/FlyDrones/ecps295 python monitor.py
    keys: f = fullscreen on/off, c = chase / overview camera, q = quit
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
DRONE_CAM = "/dev/shm/ecps295_cam_rgb"
BG = (31, 22, 13)  # BGR, close to the dashboard background
FG, MUTED, WARN = (240, 240, 240), (150, 150, 150), (80, 80, 255)
VIEW_AR, CAM_AR = 16 / 9, 4 / 3
STALE_S = 1.0  # a camera panel without a new frame for this long is not live

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=5799)
ap.add_argument("--view", choices=list(VIEWS), default="chase")
ap.add_argument("--fps", type=float, default=30)
ap.add_argument("--windowed", action="store_true", help="start windowed instead of fullscreen")
a = ap.parse_args()


def text(img, s, xy, scale=0.6, color=FG, thick=1):
    """Text on a semi-transparent dark band (an outline smears when scaled)."""
    (tw, th), base = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x, y = xy
    x0, y0, x1, y1 = max(0, x - 6), max(0, y - th - 6), min(img.shape[1], x + tw + 6), min(img.shape[0], y + base + 4)
    img[y0:y1, x0:x1] = (img[y0:y1, x0:x1] * 0.35).astype(np.uint8)
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def cover(img: np.ndarray, w: int, h: int) -> np.ndarray:
    """Scale to fill w x h exactly, cropping the overflow (centre)."""
    s = max(w / img.shape[1], h / img.shape[0])
    rw, rh = max(w, int(round(img.shape[1] * s))), max(h, int(round(img.shape[0] * s)))
    r = cv2.resize(img, (rw, rh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    x0, y0 = (rw - w) // 2, (rh - h) // 2
    return r[y0 : y0 + h, x0 : x0 + w]


def layout(W: int, H: int) -> tuple[int, int, int]:
    """Right column width and its two panel heights so that a 16:9 view over a 4:3 view fill the height."""
    wr = int(round(H / (1 / VIEW_AR + 1 / CAM_AR)))
    wr = min(wr, int(W * 0.55))
    hv = int(round(wr / VIEW_AR))
    return wr, hv, H - hv


def placeholder(w: int, h: int, msg: str) -> np.ndarray:
    img = np.full((h, w, 3), BG, np.uint8)
    text(img, msg, (30, h // 2), 0.7, MUTED)
    return img


class Session:
    """One run: receives ticks and renders the dashboard (at the requested pixel size) off the GUI thread."""

    def __init__(self, conn, hello: dict):
        from world_layout import room_from_layout

        cfg = hello["cfg"]
        self.brain = Brain(load_connectome(cfg["brain"]["source"]), cfg)  # same seed -> same recorded neurons
        room = room_from_layout(hello["layout"]) if hello.get("layout") else None
        self.stub = types.SimpleNamespace(name="fly-1", drone=types.SimpleNamespace(room=room))
        self.title = hello["title"]
        self.dash = None
        self.dash_size = (0, 0)
        self.want_size = (1100, 1080)
        self.conn = conn
        self.q: queue.Queue = queue.Queue()
        self.last_info = None
        self.dash_img: np.ndarray | None = None
        self.ended = False
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
                    break
        except (EOFError, OSError):
            pass
        self.ended = True
        self.q.put(None)

    def _ensure_dashboard(self):
        from flydrones.viz import Dashboard

        w, h = self.want_size
        if self.dash is not None and self.dash_size == (w, h):
            return
        new = Dashboard(self.brain, [self.stub], title=self.title, width_in=w / 100, height_in=h / 100)
        if self.dash is not None:  # keep spike / trail history across a resize
            new.raster, new.trails, new.alts = self.dash.raster, self.dash.trails, self.dash.alts
            import matplotlib.pyplot as plt

            plt.close(self.dash.fig)
        self.dash, self.dash_size = new, (w, h)

    def _render(self):
        last = None
        while True:
            try:
                items = [self.q.get(timeout=0.5)]
            except queue.Empty:
                items = []
                if last is None or self.dash_size == self.want_size:
                    continue
            while not self.q.empty():
                items.append(self.q.get_nowait())
            stop = None in items
            items = [i for i in items if i is not None]
            self._ensure_dashboard()
            for it in items[:-1]:
                self.dash.push([it])
            if items:
                last = items[-1]
            if last is not None:
                rgb = self.dash.render([last], push=bool(items))
                with self.lock:
                    self.dash_img, self.last_info = rgb[..., ::-1].copy(), last
            if stop:
                return


def connect(port: int):
    try:
        conn = Client(("127.0.0.1", port), authkey=AUTH)
    except (ConnectionRefusedError, OSError):
        return None, None
    kind, hello = conn.recv()
    return (conn, hello) if kind == "hello" else (None, None)


views = {k: ShmFrame(p) for k, p in VIEWS.items()}
drone_cam = ShmFrame(DRONE_CAM)
view = a.view
session: Session | None = None
last_try = 0.0
win = "ECPS295 monitor - MiniFly brain | Gazebo"
cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO | cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow(win, 1600, 900)
fullscreen = not a.windowed
applied = False  # Qt ignores WND_PROP_FULLSCREEN until the window has shown a frame
period = 1.0 / a.fps
while True:
    t0 = time.monotonic()
    if (session is None or session.ended) and t0 - last_try > 1.0:
        last_try = t0
        conn, hello = connect(a.port)
        if conn is not None:
            session = Session(conn, hello)
            for v in (*views.values(), drone_cam):
                v.reopen()
    try:
        _, _, W, H = cv2.getWindowImageRect(win)
    except cv2.error:
        W, H = 1600, 900
    W, H = max(W, 320), max(H, 240)
    wr, hv, hc = layout(W, H)
    wl = W - wr
    canvas = np.empty((H, W, 3), np.uint8)

    # left: dashboard, rendered at exactly wl x H
    info, dash = None, None
    if session is not None:
        session.want_size = (wl, H)
        with session.lock:
            dash, info = session.dash_img, session.last_info
    if dash is not None:
        left = dash if dash.shape[:2] == (H, wl) else cover(dash, wl, H)
        canvas[:, :wl] = left
        if session.ended:
            text(canvas, "run finished - waiting for the next run_g4.py", (20, H - 15), 0.6, WARN)
    else:
        canvas[:, :wl] = placeholder(wl, H, f"waiting for run_g4.py (tick stream 127.0.0.1:{a.port}) ...")

    # right top: Gazebo view. Plain worlds (experiment runs) have no view camera: fall back to the drone camera
    # instead of showing a frozen frame; anything without a new frame for STALE_S is marked as such.
    v = views[view]
    v.read()
    drone_cam.read()
    top, top_label = None, ""
    if v.img is not None and v.age() < STALE_S:
        top, top_label = v.img, f"Gazebo {view} camera   sim t={v.stamp:7.1f} s"
    elif drone_cam.img is not None and drone_cam.age() < STALE_S:
        top, top_label = drone_cam.img, f"no Gazebo {view} camera in this run (plain world): drone camera shown"
    if top is not None:
        canvas[:hv, wl:] = cover(top[..., ::-1], wr, hv)
        text(canvas, top_label, (wl + 12, 26), 0.55)
        if info is not None:
            tel, c = info.tel, info.cmd
            text(
                canvas,
                f"alt {tel.alt_m:.2f} m  thr {c.throttle:+.2f}  yaw {c.yaw:+.2f}  fwd {c.forward:+.2f}",
                (wl + 12, hv - 14),
                0.6,
            )
            if c.escape:
                text(canvas, "ESCAPE", (W - 150, 60), 1.0, WARN, 2)
    else:
        msg = "no Gazebo frames - is a run going?" if session is None or session.ended else "Gazebo paused (no new frames)"
        canvas[:hv, wl:] = placeholder(wr, hv, msg)

    # right bottom: the drone camera in colour (same topic the brain uses, before grayscale/downsampling)
    if drone_cam.img is not None:
        canvas[hv:, wl:] = cover(drone_cam.img[..., ::-1], wr, hc)
        text(canvas, "drone camera (ELP twin, 120 deg fisheye) - what MiniFly sees", (wl + 12, hv + 26), 0.55)
        if drone_cam.age() >= STALE_S:
            canvas[hv:, wl:] = (canvas[hv:, wl:] * 0.4).astype(np.uint8)
            text(canvas, f"PAUSED - last frame {min(drone_cam.age(), 999):.0f} s ago", (wl + 12, hv + hc // 2), 0.8, WARN, 2)
    else:
        canvas[hv:, wl:] = placeholder(wr, hc, "no drone camera frames")
    cv2.line(canvas, (wl, 0), (wl, H), (70, 70, 70), 1)
    cv2.line(canvas, (wl, hv), (W, hv), (70, 70, 70), 1)
    text(canvas, "[f] fullscreen  [c] camera  [q] quit", (W - 330, H - 14), 0.5, MUTED)

    cv2.imshow(win, canvas)
    if not applied:
        cv2.waitKey(1)
        cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
        applied = True
    key = cv2.waitKey(max(1, int((period - (time.monotonic() - t0)) * 1000))) & 0xFF
    if key == ord("q") or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
        break
    if key == ord("c"):
        view = "overview" if view == "chase" else "chase"
    if key == ord("f"):
        fullscreen = not fullscreen
        cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
cv2.destroyAllWindows()
