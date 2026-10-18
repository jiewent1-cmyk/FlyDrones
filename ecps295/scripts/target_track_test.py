"""Offline closed loop of EcpsPilot target tracking: point-mass drone, a 'brain' that only cruises, a rendered camera
image of the marker each 0.1 s through the real MarkerDetector, then TargetTrack -> waypoint layer."""
import math, sys, types
sys.path[:0] = ["ecps295", "src"]
import numpy as np, cv2
from flydrones.motor import FlightCommand
import ecps_pilot
from ecps_target import MarkerDetector, TargetTrack, camera_matrix, marker_image, render_marker

W, H, HFOV, PITCH, SIZE = 640, 480, 70.0, 12.0, 0.15
K = camera_matrix(W, H, HFOV)
TEX = marker_image("4x4_50", 7)
DET = MarkerDetector("4x4_50", 7, SIZE, HFOV, PITCH)
YAW_DPS, V = 60.0, 0.5  # full-scale yaw rate, forward speed at forward = 1


def make_pilot(tcfg=None):
    p = ecps_pilot.EcpsPilot.__new__(ecps_pilot.EcpsPilot)
    p.watchdog_events = []
    p.wp_on, p.wp_points, p.wp_radius = False, [], 0.4
    p.wp_gain, p.wp_brain_w, p.wp_slow_r, p.wp_min_fwd = 1.0, 1.0, 0.8, 0.3
    p.wp_post_escape, p.wp_stuck_s, p.wp_stuck_dm = 1.0, 30.0, 0.25
    p.wp_idx, p.wp_active, p.wp_log, p.wp_done_t, p.wp_ticks = 0, True, [], None, 0
    p._wp_best, p._wp_best_t, p._wp_escape_t = float("inf"), 0.0, -1e9
    p.safety = types.SimpleNamespace(fence=2.0)
    p._t = 0.0
    p.target = TargetTrack(tcfg)
    p.target.on_event = p._event
    return p


class Cam:
    def __init__(self): self.det, self.stamp = None, 0.0
    def latest(self): return self.det, self.stamp


def fly(p, secs, marker=lambda t: (2.5, 1.0, 1.0, 200.0), yaw0=180.0, active=lambda t: True, start_t=0.5):
    x = y = 0.0; yaw = yaw0; t, dt = 0.0, 0.05
    cam = Cam(); p.target.source = cam
    rng = np.random.default_rng(1)
    trace = []
    started = False
    while t < secs:
        if not started and t >= start_t:
            p.track_start(t); started = True
        p.wp_active = active(t)
        if round(t / dt) % 2 == 0:  # camera + detector at 10 Hz
            img = rng.normal(110, 10, (H, W)).clip(0, 255).astype(np.uint8)
            m = marker(t)
            if m is not None:
                render_marker(img, K, (x, y, 1.0), yaw, PITCH, m[:3], m[3], SIZE, TEX)
            d = DET.detect(img, t)
            cam.det, cam.stamp = (d[0] if d else cam.det), t
        tel = types.SimpleNamespace(t=t, x_m=x, y_m=y, yaw_deg=yaw, alt_m=1.0)
        cmd = FlightCommand(throttle=0.0, yaw=0.0, forward=0.5)
        p._t = t
        if p.target.active:
            cmd = p.target.update(p, cmd, tel, t)
        if p.wp_on:
            cmd = p._waypoint(cmd, tel, t)
        if p.wp_active:
            yaw = (yaw + cmd.yaw * YAW_DPS * dt + 180) % 360 - 180
            v = cmd.forward * V
            x, y = x + v * math.cos(math.radians(yaw)) * dt, y + v * math.sin(math.radians(yaw)) * dt
        trace.append((t, x, y, yaw, p.target.state))
        t += dt
    return x, y, yaw, trace


def facing_err(x, y, yaw, m):
    b = math.degrees(math.atan2(m[1] - y, m[0] - x))
    return abs((b - yaw + 180) % 360 - 180)


ok = True
NEAR = (0.85, 1.2)  # hover distance from the marker: standoff 1.0 m
def check(name, cond, info):
    global ok
    print(("PASS " if cond else "FAIL ") + name, info)
    ok &= bool(cond)

# A: marker behind the drone -> search, approach, hover 1 m in front, facing it
p = make_pilot(); M = (1.6, 0.8, 1.0, 200.0)
x, y, yaw, tr = fly(p, 60, marker=lambda t: M)
d = math.hypot(M[0] - x, M[1] - y)
ev = [e[1] for e in p.watchdog_events]
check("A search->approach->arrived", p.target.state == "ARRIVED" and NEAR[0] < d < NEAR[1] and facing_err(x, y, yaw, M) < 10,
      f"state={p.target.state} dist={d:.2f} facing_err={facing_err(x, y, yaw, M):.1f} est={p.target.est} events={ev}")

# B: the marker moves after arrival -> re-approach
p = make_pilot(); M1, M2 = (1.6, 0.8, 1.0, 200.0), (0.6, -1.4, 1.0, 120.0)
x, y, yaw, tr = fly(p, 120, marker=lambda t: M1 if t < 50 else M2)
d = math.hypot(M2[0] - x, M2[1] - y)
check("B marker moved", p.target.state == "ARRIVED" and NEAR[0] < d < NEAR[1] and facing_err(x, y, yaw, M2) < 10,
      f"state={p.target.state} dist={d:.2f} est={p.target.est} events={[e[1] for e in p.watchdog_events]}")

# C: no marker -> search for search_s, then FAILED (hover)
p = make_pilot({"search_s": 20})
x, y, yaw, tr = fly(p, 40, marker=lambda t: None)
x0 = [a for a in tr if a[0] >= 0.5][0]
check("C not found", p.target.state == "FAILED" and math.hypot(x - x0[1], y - x0[2]) < 0.01, f"state={p.target.state} pos=({x:.2f},{y:.2f})")

# D: marker outside the fence (2 m): the goal stays inside fence - margin
p = make_pilot(); M = (3.4, 0.0, 1.0, 180.0)
x, y, yaw, tr = fly(p, 60, marker=lambda t: M, yaw0=0.0)
r = max(math.hypot(a[1], a[2]) for a in tr)
check("D fence clip", r < 1.65 and p.target.state == "ARRIVED", f"max r={r:.2f} state={p.target.state} goal={p.target.goal}")

# E: the pilot / console holds for 15 s mid-approach: nothing moves, the search timer does not run out, then it goes on
p = make_pilot({"search_s": 10}); M = (1.6, 0.8, 1.0, 200.0)
x, y, yaw, tr = fly(p, 80, marker=lambda t: M, active=lambda t: not (2 < t < 17))
d = math.hypot(M[0] - x, M[1] - y)
check("E paused", p.target.state == "ARRIVED" and NEAR[0] < d < NEAR[1], f"state={p.target.state} dist={d:.2f}")

# F: the marker disappears mid-approach (occluded): LOST, fly on to the last goal, then search; it reappears -> arrive
p = make_pilot(); M = (1.6, 0.8, 1.0, 200.0)
x, y, yaw, tr = fly(p, 90, marker=lambda t: None if 14 < t < 30 else M)
states = []
for a in tr:
    if not states or states[-1] != a[4]: states.append(a[4])
d = math.hypot(M[0] - x, M[1] - y)
check("F occluded", p.target.state == "ARRIVED" and NEAR[0] < d < NEAR[1], f"states={states} dist={d:.2f}")
print("ALL OK" if ok else "SOME FAILED")
