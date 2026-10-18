"""Camera sources for the ECPS295 companion: the latest frame as the brain sees it in Gazebo.

Gazebo path (gazebo/cam_bridge.py): render 640x480, convert to gray, area-downsample (box filter) to 192x144 uint8;
FlyDrones' Retina then resizes to its own grid. The real camera does the same here:

  UVCCamera   V4L2 capture thread (OpenCV, MJPEG 640x480 by default; ELP OV7725 or Waveshare OV9281), decode,
              gray, cv2.INTER_AREA to out_w x out_h. Exposure lock and the 12 deg nose-down mount are set on the bench.
  SimCamera   pre-encoded synthetic MJPEG frames (a textured wall that looms and recedes, plus slow yaw) decoded at
              the camera rate, so the decode cost on the Pi is the same as with a real MJPEG camera.
  ReplayCamera  a directory of JPEG frames (run_g4.py --record-frames), looped at fps.

All expose: start(), stop(), latest() -> (frame uint8 HxW or None, stamp monotonic), frames, dropped, decode_ms.
"""

from __future__ import annotations

import glob
import threading
import time

import cv2
import numpy as np


class _Cam:
    def __init__(self, out_w: int = 192, out_h: int = 144, fps: float = 30.0):
        self.out = (out_w, out_h)
        self.fps = fps
        self.img: np.ndarray | None = None
        self.last_bgr: np.ndarray | None = None  # the full decoded frame (ground console colour view)
        self.stamp = 0.0
        self.frames = 0
        self.errors = 0
        self.decode_ms: list[float] = []
        self.status = "init"
        self._stop = threading.Event()
        self._th: threading.Thread | None = None

    def latest(self):
        return self.img, self.stamp

    def start(self):
        self._th = threading.Thread(target=self._run, name=type(self).__name__, daemon=True)
        self._th.start()
        return self

    def stop(self):
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=2)

    def _publish_gray(self, bgr_or_gray: np.ndarray, t0: float) -> None:
        self.last_bgr = bgr_or_gray  # a reference, no copy: each decode makes a new array
        g = bgr_or_gray if bgr_or_gray.ndim == 2 else cv2.cvtColor(bgr_or_gray, cv2.COLOR_BGR2GRAY)
        g = cv2.resize(g, self.out, interpolation=cv2.INTER_AREA)
        if len(self.decode_ms) < 50000:
            self.decode_ms.append((time.perf_counter() - t0) * 1000)
        self.img = g
        self.stamp = time.monotonic()
        self.frames += 1


class UVCCamera(_Cam):
    def __init__(self, device: str = "/dev/video0", width: int = 640, height: int = 480, fourcc: str = "MJPG", **kw):
        super().__init__(**kw)
        self.device, self.size, self.fourcc = device, (width, height), fourcc

    def _run(self) -> None:
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.status = f"cannot open {self.device}"
                self._stop.wait(2.0)
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # newest frame, not a queue of old ones
            self.status = (
                f"capturing {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
                f" @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps"
            )
            while not self._stop.is_set():
                if not cap.grab():
                    self.errors += 1
                    break
                t0 = time.perf_counter()
                ok, f = cap.retrieve()  # MJPEG decode happens here
                if not ok:
                    self.errors += 1
                    continue
                self._publish_gray(f, t0)
            cap.release()
            self._stop.wait(0.5)


class SimCamera(_Cam):
    def __init__(self, width: int = 640, height: int = 480, seed: int = 0, n_frames: int = 240, **kw):
        super().__init__(**kw)
        rng = np.random.default_rng(seed)
        tex = cv2.resize((rng.random((60, 80)) * 255).astype(np.uint8), (1280, 960), interpolation=cv2.INTER_CUBIC)
        tex = cv2.GaussianBlur(tex, (0, 0), 3)
        tex = cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR)
        self.jpegs = []
        for i in range(n_frames):  # 8 s at 30 fps: loom in (scale 1 -> 2.2) and out, with a slow pan
            ph = i / n_frames
            s = 1.0 + 1.2 * (1 - abs(2 * ph - 1))
            dx = 80 * np.sin(2 * np.pi * ph)
            M = np.array([[s, 0, 640 - s * 640 + dx], [0, s, 480 - s * 480]], np.float32)
            img = cv2.warpAffine(tex, M, (1280, 960))[240:720, 320:960]
            img = cv2.resize(img, (width, height))
            self.jpegs.append(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1])
        self.status = "sim"

    def _run(self) -> None:
        nxt, i = time.monotonic(), 0
        while not self._stop.is_set():
            nxt += 1.0 / self.fps
            self._stop.wait(max(0.0, nxt - time.monotonic()))
            t0 = time.perf_counter()
            f = cv2.imdecode(self.jpegs[i % len(self.jpegs)], cv2.IMREAD_COLOR)
            self._publish_gray(f, t0)
            i += 1


class ReplayCamera(_Cam):
    def __init__(self, directory: str, **kw):
        super().__init__(**kw)
        self.files = sorted(glob.glob(f"{directory}/*.jpg"))
        if not self.files:
            raise SystemExit(f"no JPEG frames in {directory}")
        self.jpegs = [np.fromfile(f, np.uint8) for f in self.files]
        self.status = f"replay {len(self.files)} frames"

    _run = SimCamera._run


def make(source: str, cfg: dict):
    """source: 'uvc' | 'sim' | 'replay:<dir>' | 'off'."""
    if source == "off":
        return None
    common = {k: cfg[k] for k in ("out_w", "out_h", "fps") if k in cfg}
    if source == "sim":
        return SimCamera(**common)
    if source.startswith("replay:"):
        return ReplayCamera(source.split(":", 1)[1], **common)
    return UVCCamera(**{k: cfg[k] for k in ("device", "width", "height", "fourcc") if k in cfg}, **common)
