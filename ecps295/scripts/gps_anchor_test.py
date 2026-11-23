"""Offline test of the GPS anchor (2026-11-23, ecps_gps_anchor.GpsAnchor). Run from the repository root:
    python3 ecps295/scripts/gps_anchor_test.py
The flow EKF drifts: ekf = truth + drift(t). The FC holds the target the companion sends (first order, tau 0.8 s), the
target is anchor + offset as in mavlink_twin's position hold. GPS = truth + slow wander + noise, 5 Hz."""
import math, sys
sys.path[:0] = ["ecps295", "src"]
import numpy as np
from ecps_gps_anchor import GpsAnchor

LAT0, LON0, R = 33.643, -117.842, 6378137.0


def hover(secs=600.0, drift_v=(0.004, -0.003), wander=0.3, hacc=0.6, cfg=None, step=None, loss=None, fix_type=3, sats=12,
          lm=None, seed=1, takeoff_s=0.0):
    """step = (t, north m): GPS position jumps; loss = (t0, t1): no GPS samples; lm = (t, offset): one landmark fix."""
    rng = np.random.default_rng(seed)
    ga = GpsAnchor({"enabled": True, **(cfg or {})})
    truth, anchor, t, dt, k = np.zeros(2), None, 0.0, 0.05, 0
    w, ph = wander * math.sqrt(2), rng.uniform(0, 6.28, 2)
    sample, errs, events = None, [], []
    lm_off, lm_fixes = None, 0
    while t < secs:
        drift = np.array(drift_v) * t
        ekf = truth + drift
        if k % 4 == 0 and not (loss and loss[0] <= t < loss[1]):  # GPS 5 Hz
            g = truth + w * np.sin(2 * math.pi * t / 300.0 + ph) + rng.normal(0, 0.1, 2)
            if step and t >= step[0]:
                g = g + np.array([step[1], 0.0])
            sample = {"stamp": k, "lat": LAT0 + math.degrees(g[0] / R), "lon": LON0 + math.degrees(g[1] / R / math.cos(math.radians(LAT0))),
                      "fix": fix_type, "hacc": hacc, "sats": sats}
        if lm and lm_fixes == 0 and t >= lm[0]:
            lm_off, lm_fixes = lm[1], 1
        alt = 2.0 if t >= takeoff_s else 0.1
        off = np.array(ga.step(t, float(ekf[0]), float(ekf[1]), sample, True, t, lm_off, lm_fixes, lambda tt, s: events.append((tt, s)), alt))
        if anchor is None:
            anchor = ekf - off  # world frame
        target = anchor + off  # EKF frame
        truth = truth + (target - ekf) * (dt / 0.8)
        errs.append(math.hypot(*truth))
        t += dt; k += 1
    return ga, np.array(errs), events


ok = True


def check(name, cond, detail):
    global ok
    ok &= bool(cond)
    print(("PASS " if cond else "FAIL ") + name + ": " + detail)


free = math.hypot(0.004, 0.003) * 600  # drift without any fix: 3.0 m
ga, e, _ = hover()
check("good GPS bounds the drift", e.max() < 1.5 and e[-1] < 1.3, f"max {e.max():.2f} m, end {e[-1]:.2f} m (free drift {free:.1f} m)")
ga, e, _ = hover(drift_v=(0.0, 0.0), wander=1.0, hacc=2.0)
check("poor GPS, no drift: wander is not followed", e.max() < 0.5, f"max {e.max():.2f} m (GPS wander 1.0 m rms)")
ga, e, ev = hover(drift_v=(0.0, 0.0), step=(120.0, 3.0))
check("GPS jump 3 m is absorbed", e.max() < 0.4 and ga.fix.n_jump >= 1, f"max {e.max():.2f} m, jumps {ga.fix.n_jump}, {ev[-1][1]}")
ga, e, _ = hover(loss=(120.0, 600.0))
ga2, e2, _ = hover(secs=120.0)
check("GPS lost: correction held", np.allclose(ga.fix.c, ga2.fix.c, atol=0.05) and ga.state == "no_gps", f"c {np.round(ga.fix.c, 2)}, state {ga.state}")
ga, e, _ = hover(hacc=3.0)
check("accuracy worse than hacc_max_m: ignored", ga.offset == (0.0, 0.0) and ga.state == "bad_gps", f"offset {ga.offset}, state {ga.state}")
ga, e, _ = hover(sats=5)
check("too few satellites: ignored", ga.offset == (0.0, 0.0), f"offset {ga.offset}")
ga, e, _ = hover(drift_v=(0.01, 0.0), secs=600.0, cfg={"max_m": 1.0})
check("correction limited to max_m", abs(math.hypot(*ga.fix.c) - 1.0) < 1e-6, f"|c| {math.hypot(*ga.fix.c):.2f} m (drift 6 m)")
ga, e, _ = hover(drift_v=(0.03, 0.0), secs=600.0)  # lag v x tau = 1.8 m > gate_m: known limit, the correction stops
check("drift faster than gate_m / tau_s: held, not wrong", ga.state == "gated" and math.hypot(*ga.fix.c) < 3.0, f"|c| {math.hypot(*ga.fix.c):.2f} m, state {ga.state}")
ga, e, _ = hover(secs=300.0, lm=(100.0, (0.4, -0.3)))
check("landmark fix re-bases", abs(ga.base[0] - 0.4) < 1e-9 and math.hypot(*ga.fix.c) < 0.5, f"base {ga.base}, c {np.round(ga.fix.c, 2)}")
ga, e, _ = hover(secs=60.0, takeoff_s=20.0)
check("no reference during take-off", ga.fix.t0 is not None and ga.fix.t0 >= 20.0, f"reference started at {ga.fix.t0:.1f} s")
ga = GpsAnchor({"enabled": True})
for i in range(100):
    ga.step(i * 0.2, 0.0, 0.0, {"stamp": i, "lat": LAT0, "lon": LON0, "fix": 3, "hacc": 0.6, "sats": 12}, True)
ga.fix.c = [0.5, 0.0]
ga.step(30.0, 0.0, 0.0, None, False)
check("disarmed: reset", ga.offset == (0.0, 0.0) and ga.fix.ref is None, f"offset {ga.offset}")
ga = GpsAnchor(None)
check("off by default", not ga.enabled and ga.step(0.0, 1.0, 1.0, {"stamp": 1, "lat": 0, "lon": 0}, True) == (0.0, 0.0), "enabled False")
print("ALL PASS" if ok else "SOME FAILED")
sys.exit(0 if ok else 1)
