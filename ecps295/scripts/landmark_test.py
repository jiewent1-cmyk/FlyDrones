"""Offline test of the landmark position fix (2026-10-22, ecps_landmark.LandmarkFix + the hover hold's world anchor).

The flow EKF drifts: ekf = truth + drift(t). The FC holds the EKF position target the companion sends (first order,
tau 0.8 s), so without a fix the true position walks off with the drift. A marker at a fixed world point is detected
from the TRUE pose (range noise 3 cm, bearing noise 0.5 deg, 10 Hz) and the hold target becomes anchor + offset."""
import math, sys, types
sys.path[:0] = ["ecps295", "src"]
import numpy as np
from ecps_landmark import LandmarkFix
from ecps_target import Detection


class Cam:
    def __init__(self): self.dets, self.stamp = [], 0.0
    def latest_all(self): return self.dets, self.stamp
    def latest(self): return (self.dets[0] if self.dets else None), self.stamp


def hover(secs=60.0, drift_v=(0.012, -0.008), marker=(1.5, 0.3), mid=3, cfg=None, visible=lambda t: True, yaw=10.0,
          known=None, move_at=None):
    rng = np.random.default_rng(2)
    lm = LandmarkFix({"enabled": True, **({"positions": {mid: list(known)}} if known else {}), **(cfg or {})})
    cam = Cam(); lm.source = cam
    ekf = np.zeros(2)  # the FC's estimate, starts at the anchor (origin)
    anchor_w = None
    t, dt = 0.0, 0.05
    errs = []
    while t < secs:
        drift = np.array(drift_v) * t
        truth = ekf - drift
        mk = np.array(marker) + (np.array([0.0, 0.6]) if move_at is not None and t >= move_at else 0.0)
        if round(t / dt) % 2 == 0:  # detector 10 Hz, from the true pose
            dn, de = mk - truth
            r = math.hypot(dn, de) + rng.normal(0, 0.03)
            b = (math.degrees(math.atan2(de, dn)) - yaw + 180) % 360 - 180 + rng.normal(0, 0.5)
            cam.dets = [Detection(mid, r, b, 0.3, 10.0, 60.0, t)] if visible(t) and abs(b) < 60 else []
            cam.stamp = t
        lm.update(t, float(ekf[0]), float(ekf[1]), yaw, t)
        off = np.array(lm.offset) if lm.fixes else np.zeros(2)
        if anchor_w is None:
            anchor_w = ekf - off
        target = anchor_w + off
        ekf = ekf + (target - ekf) * dt / 0.8
        errs.append(float(np.hypot(*(ekf - np.array(drift_v) * t))))  # true distance from the start point
        t += dt
    return lm, errs


ok = True


def check(name, cond, info):
    global ok
    print(("PASS " if cond else "FAIL ") + name, info)
    ok &= bool(cond)


# A: 60 s hover, drift 1.4 cm/s: without the fix the drone walks ~0.86 m; with it it stays near the start
lm, e = hover(cfg={"enabled": True})
base = math.hypot(0.012, -0.008) * 60
check("A hold with a landmark", max(e[100:]) < 0.15 and lm.fixes > 400, f"true drift max {max(e[100:]):.3f} m end {e[-1]:.3f} m "
      f"(no fix: {base:.2f} m) fixes={lm.fixes} log={lm.log[:2]}")

# B: surveyed position instead of auto-register: same, and no registration step
lm, e = hover(known=(1.5, 0.3))
check("B surveyed landmark", max(e) < 0.15 and not any("registered" in x for _, x in lm.log), f"max {max(e):.3f} m log={lm.log[:2]}")

# C: the marker is out of view from 20 to 40 s: the offset stays (drift grows meanwhile), recovers after
lm, e = hover(visible=lambda t: not (20 < t < 40))
mid_err, end_err = max(e[400:800]), e[-1]
check("C occlusion", end_err < 0.1 and mid_err < 0.35, f"max while hidden {mid_err:.3f} m (drift 20 s = {math.hypot(0.012, 0.008) * 20:.2f}) end {end_err:.3f} m")

# D: off the nose (bearing ~ 60+ deg): no fixes are used, the drift goes on
lm, e = hover(yaw=80.0)
check("D marker off the nose -> no fix", lm.fixes == 0 and e[-1] > 0.7, f"fixes={lm.fixes} rejected={lm.rejected} end {e[-1]:.2f} m")

# E: the auto-registered marker is moved 0.6 m at 30 s: it is learnt again (no jump of the drone by 0.6 m)
lm, e = hover(move_at=30.0)
check("E marker moved -> re-registered", any("moved" in x for _, x in lm.log) and max(e[650:]) < 0.25,
      f"max after the move {max(e[650:]):.3f} m log={[x for _, x in lm.log]}")

# F: disabled: nothing changes
lm = LandmarkFix({"enabled": False}); lm.source = Cam()
check("F disabled", lm.update(0.0, 0.0, 0.0, 0.0, 0.0) is False and lm.offset == (0.0, 0.0), "")
print("ALL OK" if ok else "SOME FAILED")
