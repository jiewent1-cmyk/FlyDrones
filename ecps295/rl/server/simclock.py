"""Sim-time clock for run_g4.py on hosts that cannot hold Gazebo + SITL lockstep at real time (RL plan, server runs).

install(path) replaces time.monotonic / time.time / time.sleep in this process with versions driven by the Gazebo sim
time stamped on every camera frame (cam_bridge.py shared-memory header, <QdIII: seq, stamp, w, h, c). The brain then
ticks every DT of *sim* time and every wall-clock timeout (takeoff, EKF wait, watchdogs, pymavlink) is measured in sim
time, so a run at RTF 0.35 behaves like one at RTF 1.0. time.perf_counter is left alone (compute budget stays real).
Reads the header without consuming frames. If sim time stops for 30 s of wall time, sleep() returns anyway.
"""

from __future__ import annotations

import mmap
import os
import struct
import time

_HDR = struct.Struct("<QdIII")
_real_sleep, _real_mono, _real_time = time.sleep, time.monotonic, time.time


class _SimClock:
    def __init__(self, path: str):
        self.path = path
        self.mm = None
        while self.mm is None:
            try:
                fd = os.open(path, os.O_RDONLY)
                self.mm = mmap.mmap(fd, 0, prot=mmap.PROT_READ)
            except (FileNotFoundError, ValueError):
                _real_sleep(0.05)
        while self._stamp() is None:
            _real_sleep(0.01)
        self.epoch0 = _real_time() - self._stamp()  # time.time() = wall epoch at start + sim seconds

    def _stamp(self) -> float | None:
        for _ in range(5):
            seq, stamp, *_ = _HDR.unpack_from(self.mm, 0)
            if seq and not seq % 2:
                return stamp
        return None

    def now(self) -> float:
        s = self._stamp()
        while s is None:
            _real_sleep(0.0005)
            s = self._stamp()
        self.last = s
        return s

    def sleep(self, secs: float) -> None:
        if secs <= 0:
            return
        target = self.now() + secs
        wall_last, sim_last = _real_mono(), self.last
        while True:
            s = self.now()
            if s >= target - 0.002:  # frames are 1/60 s apart: 3 x 16.666 ms must count as 50 ms
                return
            if s != sim_last:
                wall_last, sim_last = _real_mono(), s
            elif _real_mono() - wall_last > 30.0:  # sim stalled (Gazebo gone?): do not hang the run
                return
            _real_sleep(0.001)


def install(path: str) -> _SimClock:
    c = _SimClock(path)
    time.monotonic = c.now
    time.time = lambda: c.epoch0 + c.now()
    time.sleep = c.sleep
    return c
