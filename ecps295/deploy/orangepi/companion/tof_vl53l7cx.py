"""VL53L7CX 8x8 multizone ToF on Linux I2C (Orange Pi Zero 3W TWI0 / TWI1) for the ECPS295 companion.

Driver: ST ULD (stm32duino/VL53L7CX, BSD-3, unchanged) + our Linux i2c-dev platform, built as
/opt/ecps295/drivers/vl53l7cx/linux/libvl53l7cx.so and called through ctypes.

ToFSensor runs one thread per sensor: firmware upload + 8x8 continuous ranging, then publishes each frame as a float
array of 64 slant ranges in metres, row-major with row 0 = TOP and column 0 = LEFT (the layout of ecps_map.zone_dirs,
which the twin and Gazebo use). Zones whose target status is not in valid_status, or beyond max_m, are NaN.

Orientation of the ST zone index relative to the scene depends on how the board is mounted: flip_lr / flip_ud /
transpose in the config map the ULD order to ours. They MUST be checked on the bench (tof_probe.py --orient).
range_kind: "radial" (the default here) keeps the reported distance as the slant range; "perpendicular" divides it by
the zone ray's cosine to the boresight. Which one the VL53L7CX reports is a bench item (see the report).

SimToF produces synthetic frames with the same interface (bench and profiling without the sensor).
"""

from __future__ import annotations

import ctypes
import math
import os
import threading
import time

import numpy as np

LIB_DEFAULT = "/opt/ecps295/drivers/vl53l7cx/linux/libvl53l7cx.so"
_lib = None


def _load(path: str = LIB_DEFAULT):
    global _lib
    if _lib is None:
        lib = ctypes.CDLL(path)
        lib.vl7_open.restype = ctypes.c_void_p
        lib.vl7_open.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        for f in ("vl7_start", "vl7_stop", "vl7_ready"):
            getattr(lib, f).argtypes = [ctypes.c_void_p]
            getattr(lib, f).restype = ctypes.c_int
        lib.vl7_init.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        lib.vl7_init.restype = ctypes.c_int
        lib.vl7_read.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int16),
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int8),
        ]
        lib.vl7_read.restype = ctypes.c_int
        lib.vl7_close.argtypes = [ctypes.c_void_p]
        _lib = lib
    return _lib


def _cos_off_axis(n: int, hfov: float, vfov: float) -> np.ndarray:
    az = (np.arange(n) - (n - 1) / 2) * math.radians(hfov) / n
    el = ((n - 1) / 2 - np.arange(n)) * math.radians(vfov) / n
    A, E = np.meshgrid(az, el)
    return (np.cos(A) * np.cos(E)).reshape(-1)


class _Base:
    """Latest-frame holder shared by the real and the simulated sensor."""

    def __init__(self, name: str, n: int = 8, hfov: float = 60.0, vfov: float = 60.0, max_m: float = 3.5):
        self.name, self.n, self.hfov, self.vfov, self.max_m = name, n, hfov, vfov, max_m
        self.frame: np.ndarray | None = None  # (n*n,) metres, NaN = no valid target
        self.stamp = 0.0  # time.monotonic() of the last frame
        self.frames = 0
        self.errors = 0
        self.read_ms: list[float] = []  # I2C transfer time per frame (bounded)
        self.status = "init"
        self._stop = threading.Event()
        self._th: threading.Thread | None = None

    def age_s(self) -> float:
        return time.monotonic() - self.stamp if self.stamp else float("inf")

    def start(self):
        self._th = threading.Thread(target=self._run, name=f"tof-{self.name}", daemon=True)
        self._th.start()
        return self

    def stop(self):
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=2)

    def _publish(self, arr: np.ndarray, ms: float) -> None:
        self.frame = arr  # new object every frame: consumers detect new frames by identity
        self.stamp = time.monotonic()
        self.frames += 1
        if len(self.read_ms) < 20000:
            self.read_ms.append(ms)


class ToFSensor(_Base):
    def __init__(
        self,
        name: str,
        bus: int,
        addr: int = 0x29,
        hz: int = 15,
        integration_ms: int = 0,
        flip_lr: bool = False,
        flip_ud: bool = False,
        transpose: bool = False,
        valid_status=(5, 9),
        range_kind: str = "radial",
        lib: str = LIB_DEFAULT,
        **kw,
    ):
        super().__init__(name, **kw)
        self.bus, self.addr, self.hz, self.integration_ms = bus, addr, hz, integration_ms
        self.flip_lr, self.flip_ud, self.transpose = flip_lr, flip_ud, transpose
        self.valid = np.array(sorted(valid_status), np.uint8)
        self.range_kind = range_kind
        self.lib_path = lib
        self.temp_c = None
        self.init_s = None  # firmware upload + config time
        self._cos = _cos_off_axis(self.n, self.hfov, self.vfov)

    def _orient(self, a: np.ndarray) -> np.ndarray:
        g = a.reshape(self.n, self.n)
        if self.transpose:
            g = g.T
        if self.flip_ud:
            g = g[::-1]
        if self.flip_lr:
            g = g[:, ::-1]
        return np.ascontiguousarray(g).reshape(-1)

    def _run(self) -> None:
        lib = _load(self.lib_path)
        err = ctypes.c_int(0)
        while not self._stop.is_set():  # retry until the sensor answers (hot-plug on the bench)
            h = lib.vl7_open(self.bus, self.addr, ctypes.byref(err))
            if h:
                break
            self.status = f"not found on /dev/i2c-{self.bus} 0x{self.addr:02x} (code {err.value})"
            self._stop.wait(2.0)
        else:
            return
        try:
            t0 = time.monotonic()
            self.status = "uploading firmware"
            st = lib.vl7_init(h, 64 if self.n == 8 else 16, self.hz, self.integration_ms)
            if st:
                self.status = f"init failed (ULD status {st})"
                return
            if lib.vl7_start(h):
                self.status = "start_ranging failed"
                return
            self.init_s = time.monotonic() - t0
            self.status = "ranging"
            nz = self.n * self.n
            dist = (ctypes.c_int16 * nz)()
            stat = (ctypes.c_uint8 * nz)()
            temp = ctypes.c_int8(0)
            period = 1.0 / self.hz
            while not self._stop.is_set():
                r = lib.vl7_ready(h)
                if r < 0:
                    self.errors += 1
                    self._stop.wait(0.01)
                    continue
                if r == 0:
                    self._stop.wait(min(0.004, period / 10))
                    continue
                c0 = time.perf_counter()
                if lib.vl7_read(h, dist, stat, nz, ctypes.byref(temp)):
                    self.errors += 1
                    continue
                ms = (time.perf_counter() - c0) * 1000
                d = np.frombuffer(dist, np.int16).astype(np.float64) / 1000.0
                s = np.frombuffer(stat, np.uint8)
                if self.range_kind == "perpendicular":
                    d = d / self._cos
                ok = np.isin(s, self.valid) & (d > 0) & (d <= self.max_m)
                self.temp_c = int(temp.value)
                self._publish(self._orient(np.where(ok, d, np.nan)), ms)
        finally:
            try:
                lib.vl7_stop(h)
            except Exception:
                pass
            lib.vl7_close(h)


class SimToF(_Base):
    """Synthetic 8x8 frames at hz: a flat wall that approaches and recedes (period 8 s) in front of the sensor, with
    1 cm noise; for the bench / profiling without hardware. wall_m=None: no target (all NaN)."""

    def __init__(self, name: str, hz: int = 15, wall_m=(0.4, 2.5), seed: int = 0, **kw):
        kw.pop("bus", None)
        kw = {k: v for k, v in kw.items() if k in ("n", "hfov", "vfov", "max_m")}
        super().__init__(name, **kw)
        self.hz, self.wall = hz, wall_m
        self.rng = np.random.default_rng(seed)
        self._cos = _cos_off_axis(self.n, self.hfov, self.vfov)
        self.status = "sim"

    def _run(self) -> None:
        t0 = time.monotonic()
        nxt = t0
        while not self._stop.is_set():
            nxt += 1.0 / self.hz
            self._stop.wait(max(0.0, nxt - time.monotonic()))
            c0 = time.perf_counter()
            if self.wall is None:
                d = np.full(self.n * self.n, np.nan)
            else:
                lo, hi = self.wall
                ph = ((time.monotonic() - t0) / 8.0) % 1.0
                w = lo + (hi - lo) * abs(2 * ph - 1)  # triangle wave
                d = w / self._cos + self.rng.normal(0.0, 0.01, self.n * self.n)
                d = np.where(d <= self.max_m, d, np.nan)
            self._publish(d, (time.perf_counter() - c0) * 1000)


def make(cfg: dict, name: str, source: str):
    """source: 'real' | 'sim' | 'off'."""
    if source == "off":
        return None
    if source == "sim":
        return SimToF(name, **cfg)
    return ToFSensor(name, **cfg)


if os.environ.get("ECPS_TOF_SELFTEST"):  # python -c "import tof_vl53l7cx" with the env var: ctypes smoke test
    _load()
    print("libvl53l7cx loaded")
