#!/usr/bin/env python3
"""Offline test of companion/height_guard.py with a fake FC link (no SITL, no hardware).

A 20 Hz stream of RANGEFINDER / ATTITUDE / SCALED_PRESSURE is synthesised for a climb to 2 m and a hover:
  normal        the rangefinder follows the true height, the barometer drifts 1.2 m/min: no trip
  stuck_low     the rangefinder stops at 0.2 m at lift-off (the VL53L0X in daylight): trip, source set, then LAND
  collapse      at 2 m the reading falls to 0.1 m and stays: trip
  baro_lost     SCALED_PRESSURE stops in flight: "blind" reported, no trip
  disabled      the same fault with enabled: false: nothing is sent
  muted         bench mode: trip is recorded, nothing is sent

    python height_guard_test.py        exit code 0 = all pass
"""
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "companion"))
import height_guard  # noqa: E402
from height_guard import HeightGuard  # noqa: E402


class FakeMav:
    def __init__(self, log):
        self.log = log

    def command_long_send(self, _sys, _comp, cmd, _conf, p1, *_rest):
        self.log.append(("cmd", cmd, p1, T.now))

    def statustext_send(self, _sev, text):
        self.log.append(("text", text.decode(), None, T.now))


class FakeDrone:
    def __init__(self, mute=False):
        self.log = []
        self.m = SimpleNamespace(messages={}, mav=FakeMav(self.log), target_system=1, target_component=1,
                                 set_mode=lambda mode: (self.log.append(("mode", mode, None, T.now)), setattr(self, "mode", mode)))
        self.mute, self.mode, self._armed, self.events = mute, "AUTO", False, []

    def armed(self):
        return self._armed

    def fc_mode(self):
        return self.mode

    def _text(self, msg, every_s=5.0):
        self.m.mav.statustext_send(4, msg[:50].encode())

    def _gevent(self, what):
        if not self.events or self.events[-1] != what:
            self.events.append(what)


class T:  # one simulated clock for time.monotonic() and time.time()
    now = 1000.0


height_guard.time = SimpleNamespace(monotonic=lambda: T.now, time=lambda: T.now)


def press(h):  # inverse of the monitor's barometric formula, sea-level reference 1013.25 hPa
    return 1013.25 * (1 - h / 44330.0) ** (1 / 0.1903)


def fly(case, cfg=None, mute=False, seconds=40.0):
    T.now = 1000.0
    d = FakeDrone(mute)
    g = HeightGuard({"enabled": True, **(cfg or {})})
    n = int(seconds * 20)
    for i in range(n):
        t = i / 20
        T.now = 1000.0 + t
        d._armed = t >= 3.0 and not (d.mode == "LAND" and t > 30)
        true_h = 0.0 if t < 4 else min(2.0, 0.5 * (t - 4))            # lift-off at 4 s, 0.5 m/s to 2 m
        rng = max(0.04, true_h)
        if case == "stuck_low":
            rng = min(rng, 0.2)
        if case == "collapse" and t >= 15:
            rng = 0.1
        drift = 0.02 * t                                               # 1.2 m/min baro drift
        noise = 0.05 * math.sin(7.3 * t)
        d.m.messages["RANGEFINDER"] = SimpleNamespace(distance=rng, _timestamp=T.now)
        d.m.messages["ATTITUDE"] = SimpleNamespace(roll=0.03, pitch=-0.02, _timestamp=T.now)
        if not (case == "baro_lost" and t >= 12):
            d.m.messages["SCALED_PRESSURE"] = SimpleNamespace(press_abs=press(true_h + drift + noise), _timestamp=T.now)
        g.tick(d)
    return d, g


def first(log, kind, value=None):
    return next((e for e in log if e[0] == kind and (value is None or e[1] == value)), None)


ok = True


def check(name, cond, detail=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


d, g = fly("normal")
check("normal: no trip with 1.2 m/min baro drift", g.tripped_t is None and not first(d.log, "mode"), f"e={g.e:+.2f} m")

for case, t_fault in (("stuck_low", 4.0), ("collapse", 15.0)):
    d, g = fly(case)
    sw, land = first(d.log, "cmd", height_guard.MAV_CMD_SET_EKF_SOURCE_SET), first(d.log, "mode", "LAND")
    trip = next((e for e in d.events if e.startswith("HEIGHT GUARD")), None)  # the latch itself is released at disarm
    check(f"{case}: trips", trip is not None and sw is not None, f"{(sw[3] if sw else 0) - 1000 - t_fault:.2f} s after the fault: {trip}")
    check(f"{case}: source set 3 first, LAND >= 0.5 s later",
          sw and land and sw[2] == 3.0 and land[3] - sw[3] >= 0.5 - 1e-6, f"switch {sw and sw[3] - 1000:.2f} s, LAND {land and land[3] - 1000:.2f} s")
    check(f"{case}: pilot is told", first(d.log, "text", "ECPS: RANGEFINDER BAD - baro height, LAND") is not None)

d, g = fly("baro_lost")
check("baro_lost: reported blind, no trip", "SCALED_PRESSURE" in g.blind and g.tripped_t is None
      and first(d.log, "text", "ECPS: height guard blind") is not None, f"blind={g.blind}")

d, g = fly("stuck_low", cfg={"enabled": False})
check("disabled: nothing sent", not d.log and g.tripped_t is None)

d, g = fly("stuck_low", mute=True)
check("muted: trip recorded, no command / mode sent", g.tripped_t is not None and not first(d.log, "cmd") and not first(d.log, "mode"))

d, g = fly("stuck_low", seconds=45.0)
check("latch released after disarm", g.tripped_t is None, f"mode {d.mode}")

sys.exit(0 if ok else 1)
