#!/usr/bin/env python3
"""Offline test of companion/height_guard.py with a fake FC link (no SITL, no hardware).

A 20 Hz stream of RANGEFINDER / ATTITUDE / SCALED_PRESSURE is synthesised for a climb to 2 m and a hover:
  normal        the rangefinder follows the true height, the barometer drifts 1.2 m/min: no trip
  stuck_low     the rangefinder stops at 0.2 m at lift-off (the VL53L0X in daylight): trip, source set, then LAND
  collapse      at 2 m the reading falls to 0.1 m and stays: trip
  baro_lost     SCALED_PRESSURE stops in flight: "blind" reported, no trip
  disabled      the same fault with enabled: false: nothing is sent
  muted         bench mode: trip is recorded, nothing is sent
Added 2026-11-22 (safety ablation):
  blip          four 0.2 s out-of-range readings: no trip (they tripped the guard before skip_invalid)
  oor           out of range for 0.25 s: no trip; for good: trip "range lost" after oor_s
  slow_drift    the reading falls behind by 0.05 m/s: trip "bias drift" (never caught without b_rate_m)
  synth         synthetic rangefinder: silent in normal flight, sent after a trip, RNGFND1_MIN raised and put back
  supervision   an exception inside the guard is reported and tick() does not raise; stalled() when tick() stops

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

    def distance_sensor_send(self, _ms, _mn, _mx, dist_cm, _type, sid, orient, _cov):
        self.log.append(("dist", dist_cm / 100.0, (sid, orient), T.now))

    def param_set_send(self, _sys, _comp, name, value, _ptype):
        self.log.append(("pset", name.decode(), value, T.now))
        self.fc.params[name.decode()] = value
        self.fc.m.messages["PARAM_VALUE"] = SimpleNamespace(param_id=name.decode(), param_value=value)

    def param_request_read_send(self, _sys, _comp, name, _idx):
        if self.fc.answers:
            self.fc.m.messages["PARAM_VALUE"] = SimpleNamespace(param_id=name.decode(), param_value=self.fc.params[name.decode()])


class FakeDrone:
    def __init__(self, mute=False):
        self.log = []
        self.m = SimpleNamespace(messages={}, mav=FakeMav(self.log), target_system=1, target_component=1,
                                 set_mode=lambda mode: (self.log.append(("mode", mode, None, T.now)), setattr(self, "mode", mode)))
        self.mute, self.mode, self._armed, self.events = mute, "AUTO", False, []
        self.m.mav.fc = self
        self.params, self.answers, self.broken = {"RNGFND1_MIN": 0.03}, True, False

    def armed(self):
        return self._armed

    def fc_mode(self):
        if self.broken:
            raise RuntimeError("fc_mode broke")
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


def fly(case, cfg=None, mute=False, seconds=40.0, setup=None):
    T.now = 1000.0
    d = FakeDrone(mute)
    if setup:
        setup(d)
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
        if case == "blip" and any(t0 <= t < t0 + 0.2 for t0 in (15, 17, 19, 21)):
            rng = 20.0                                                 # above rng_max_m
        if (case == "oor_short" and 15 <= t < 15.25) or (case == "oor" and t >= 15):
            rng = 10.0
        if case == "slow_drift" and t >= 10:
            rng = max(0.1, rng - 0.05 * (t - 10))
        if case == "broken" and t >= 15:
            d.broken = True
        if case == "stall" and t >= 15:
            continue                                                   # the companion loop no longer calls tick()
        drift = 0.02 * t                                               # 1.2 m/min baro drift
        noise = 0.05 * math.sin(7.3 * t)
        d.m.messages["RANGEFINDER"] = SimpleNamespace(distance=rng, _timestamp=T.now)
        d.m.messages["DISTANCE_SENSOR[0]"] = SimpleNamespace(current_distance=rng * 100, _timestamp=T.now)
        d.true_h = true_h
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

# ---------------------------------------------------------------- 2026-11-22
OLD = {"skip_invalid": False, "oor_s": 0.0, "b_rate_m": 0.0, "t_s": 0.5}       # the guard of b3a6691
trip_of = lambda d: next((e for e in d.events if e.startswith("HEIGHT GUARD:")), None)  # noqa: E731

d, g = fly("normal", seconds=120.0)
check("normal, 120 s: no trip with 1.2 m/min baro drift (bias rate limit on)", trip_of(d) is None, f"e={g.e:+.2f} m")

d, g = fly("blip")
check("blip: 4 x 0.2 s out of range do not trip", trip_of(d) is None and not first(d.log, "mode"))
d, g = fly("blip", cfg=OLD)
check("blip: the old guard trips (the defect this fixes)", trip_of(d) is not None, str(trip_of(d)))

d, g = fly("oor_short")
check("oor_short: 0.25 s out of range does not trip", trip_of(d) is None)
d, g = fly("oor")
sw = first(d.log, "cmd", height_guard.MAV_CMD_SET_EKF_SOURCE_SET)
check("oor: out of range for good trips as 'range lost' within 0.3-0.5 s",
      sw and 0.3 - 1e-6 <= sw[3] - 1015 <= 0.5 and "range lost" in trip_of(d), f"{sw and sw[3] - 1015:.2f} s: {trip_of(d)}")
d, g = fly("oor", cfg=OLD)
check("oor: the old guard never trips", trip_of(d) is None)

d, g = fly("slow_drift", seconds=60.0)
sw = first(d.log, "cmd", height_guard.MAV_CMD_SET_EKF_SOURCE_SET)
check("slow_drift: 0.05 m/s trips as 'bias drift' within 20 s", sw and sw[3] - 1010 <= 20 and "bias drift" in trip_of(d),
      f"{sw and sw[3] - 1010:.1f} s after the drift starts: {trip_of(d)}")
d, g = fly("slow_drift", cfg={"b_rate_m": 0.0}, seconds=28.0)
check("slow_drift: no trip in 18 s of drift without the bias rate limit", trip_of(d) is None, f"e={g.e:+.2f} m")

SY = {"synth": True}
d, g = fly("normal", cfg=SY)
air = [e for e in d.log if e[0] == "dist" and e[3] > 1004.5]
check("synth, normal: sent on the ground (id 2, down), silent in flight, RNGFND1_MIN untouched",
      first(d.log, "dist") and first(d.log, "dist")[2] == (2, 25) and not air and not first(d.log, "pset"), f"{len(air)} in flight")
d, g = fly("collapse", cfg=SY, seconds=45.0)
after = [e for e in d.log if e[0] == "dist" and 1016.5 < e[3] < 1029]
sets = [(e[2], round(e[3] - 1000, 2)) for e in d.log if e[0] == "pset"]
# baro drift of 1.2 m/min goes on after the trip and the bias tracker lags it by ~0.2 m: up to +0.6 m in 14 s is expected
check("synth, collapse: synthetic range follows the true height after the trip",
      after and all(1.9 < e[1] < 2.6 for e in after), f"{len(after)} sent, {min(e[1] for e in after):.2f}-{max(e[1] for e in after):.2f} m at 2.0 m")
check("synth, collapse: RNGFND1_MIN raised at the trip and put back after the disarm",
      sets and sets[0][0] == height_guard.RNG1_MIN_TRIP and 15 < sets[0][1] < 16 and sets[-1][0] == 0.03 and sets[-1][1] > 30
      and d.params["RNGFND1_MIN"] == 0.03 and any("restored" in e for e in d.events), f"first {sets[0]}, last {sets[-1]}")
d, g = fly("normal", cfg=SY, seconds=3.0, setup=lambda d: d.params.update(RNGFND1_MIN=height_guard.RNG1_MIN_TRIP))
check("synth: a left-over RNGFND1_MIN with unknown old value is reported, not guessed",
      d.params["RNGFND1_MIN"] == height_guard.RNG1_MIN_TRIP and any("by hand" in e for e in d.events))
d, g = fly("normal", cfg={**SY, "rng1_min": 0.03}, seconds=3.0, setup=lambda d: d.params.update(RNGFND1_MIN=height_guard.RNG1_MIN_TRIP))
check("synth: a left-over RNGFND1_MIN is put back to cfg rng1_min", d.params["RNGFND1_MIN"] == 0.03)
d, g = fly("collapse", cfg=SY, mute=True)
check("synth, muted: nothing sent", not d.log, f"{len(d.log)} sent")

d, g = fly("broken")
check("supervision: an exception in the guard is reported, tick() does not raise",
      g.fault.startswith("RuntimeError") and first(d.log, "text", "ECPS: HEIGHT GUARD FAILED") is not None
      and "FAILED" in g.note(), g.fault)
d, g = fly("stall", seconds=20.0)
check("supervision: stalled() once tick() is no longer called", g.stalled(True) and not g.stalled(False))
d, g = fly("normal")
check("supervision: not stalled in a normal flight", not g.stalled(True) and not g.fault)

sys.exit(0 if ok else 1)
