"""Offline closed loop of EcpsPilot return-home (2026-10-22): point-mass drone, a 'brain' that cruises (or escapes in
one sector), the rtl layer -> waypoint layer. Cases: home + land, stuck -> land here, timeout, pilot pause, battery."""
import math, sys, types
sys.path[:0] = ["ecps295", "src"]
from flydrones.motor import FlightCommand
import ecps_pilot
from ecps_target import TargetTrack

YAW_DPS, V = 60.0, 0.5


def make_pilot(rtl=None):
    p = ecps_pilot.EcpsPilot.__new__(ecps_pilot.EcpsPilot)
    p.watchdog_events = []
    p.wp_on, p.wp_points, p.wp_radius = False, [], 0.4
    p.wp_gain, p.wp_brain_w, p.wp_slow_r, p.wp_min_fwd = 1.0, 1.0, 0.8, 0.3
    p.wp_post_escape, p.wp_stuck_s, p.wp_stuck_dm = 1.0, 20.0, 0.25
    p.wp_idx, p.wp_active, p.wp_log, p.wp_done_t, p.wp_ticks = 0, True, [], None, 0
    p._wp_best, p._wp_best_t, p._wp_escape_t = float("inf"), 0.0, -1e9
    p.safety = types.SimpleNamespace(fence=2.0, land_requested=False)
    p._t = p._t_prev = 0.0
    p.target = TargetTrack({})
    p.target.on_event = p._event
    rc = {"settle_s": 2.0, "timeout_s": 90.0, "battery_pct": 30.0, **(rtl or {})}
    # the rtl part of EcpsPilot.__init__
    p.rtl_on = bool(rc.get("enabled", True))
    p.rtl_home = tuple(float(v) for v in rc.get("home", (0.0, 0.0)))
    p.rtl_radius = float(rc.get("radius_m", 0.25))
    p.rtl_settle_s = float(rc.get("settle_s", 2.0))
    p.rtl_timeout_s = float(rc.get("timeout_s", 90.0))
    p.rtl_land_far = float(rc.get("land_far_m", 0.6))
    p.rtl_battery = float(rc.get("battery_pct", 0.0))
    p.rtl_state, p.rtl_why, p._rtl_t, p._rtl_radius0, p._rtl_batt_done = "OFF", "", 0.0, None, False
    return p


def fly(p, secs, xy0=(1.4, 0.9), yaw0=30.0, rtl_at=1.0, brain=None, active=lambda t: True, battery=lambda t: 100.0):
    (x, y), yaw = xy0, yaw0
    t, dt = 0.0, 0.05
    land_t = None
    started = False
    while t < secs:
        p.wp_active = active(t)
        tel = types.SimpleNamespace(t=t, x_m=x, y_m=y, yaw_deg=yaw, alt_m=0.8, flying=land_t is None, battery_pct=battery(t))
        if rtl_at is not None and not started and t >= rtl_at:
            p.rtl_start("test", t); started = True
        p._rtl_watch(tel, t)
        p._t_prev, p._t = p._t, t
        cmd = FlightCommand(throttle=0.0, yaw=0.0, forward=0.5) if brain is None else brain(t, x, y, yaw)
        if p.rtl_state != "OFF":
            cmd = p._rtl(cmd, tel, t)
        if p.wp_on:
            cmd = p._waypoint(cmd, tel, t)
        if p.safety.land_requested and land_t is None:
            land_t = t  # the FC lands where it is
        if land_t is None and p.wp_active:
            yaw = (yaw + cmd.yaw * YAW_DPS * dt + 180) % 360 - 180
            v = cmd.forward * V
            x, y = x + v * math.cos(math.radians(yaw)) * dt, y + v * math.sin(math.radians(yaw)) * dt
        t += dt
    return x, y, land_t


ok = True


def check(name, cond, info):
    global ok
    print(("PASS " if cond else "FAIL ") + name, info)
    ok &= bool(cond)


ev = lambda p: [e[1] for e in p.watchdog_events]  # noqa: E731

# A: return from (1.4, 0.9): home, settle, land on the point
p = make_pilot(); x, y, lt = fly(p, 40)
check("A home and land", lt is not None and math.hypot(x, y) < 0.3 and ev(p)[-1].startswith("rtl: landing") and p.rtl_state == "OFF", f"pos=({x:.2f},{y:.2f}) land_t={lt} events={ev(p)}")

# B: a wall across the way home (north = 0.9 m, the brain saccades away whenever it would cross it): the waypoint
# layer skips home as stuck -> lands where it is, on the far side of the wall (state OFF again once landed)
def wall(t, x, y, yaw):
    if x < 1.0 and math.cos(math.radians(yaw)) < 0:
        return FlightCommand(throttle=0.0, yaw=1.0, forward=-0.2, escape=True)
    return FlightCommand(throttle=0.0, yaw=0.0, forward=0.5 if x > 0.95 else 0.0)
p = make_pilot(); x, y, lt = fly(p, 60, xy0=(1.5, 0.3), yaw0=0.0, brain=wall)
check("B stuck -> land here", lt is not None and x > 0.85 and any("landing here" in e for e in ev(p)), f"pos=({x:.2f},{y:.2f}) events={ev(p)}")

# C: timeout shorter than the way home
p = make_pilot({"timeout_s": 2.0}); x, y, lt = fly(p, 20)
check("C timeout -> land here", lt is not None and any("timeout" in e for e in ev(p)), f"pos=({x:.2f},{y:.2f}) land_t={lt}")

# D: the pilot flies (not in control) for 10 s mid-return: nothing moves, the timeout waits, then it goes home
p = make_pilot({"timeout_s": 12.0}); x, y, lt = fly(p, 50, active=lambda t: not (2 < t < 12))
check("D paused", lt is not None and math.hypot(x, y) < 0.3, f"pos=({x:.2f},{y:.2f}) land_t={lt} events={ev(p)}")

# E: battery 35 -> 25 % at t = 3 s starts the return by itself, once
p = make_pilot(); x, y, lt = fly(p, 40, rtl_at=None, battery=lambda t: 35.0 if t < 3 else 25.0)
check("E battery", lt is not None and math.hypot(x, y) < 0.3 and any("battery 25 %" in e for e in ev(p)), f"pos=({x:.2f},{y:.2f}) events={ev(p)}")

# F: cancelled mid-way (the console takes over): no landing, waypoint layer off, radius restored
p = make_pilot(); r0 = p.wp_radius
(x, y), lt = (1.4, 0.9), None
p.rtl_start("test", 0.0); p.rtl_stop(1.0)
check("F cancel", p.rtl_state == "OFF" and not p.wp_on and not p.safety.land_requested and p.wp_radius == r0, f"state={p.rtl_state} wp_on={p.wp_on}")

# G: off (rtl.enabled false): rtl_start refuses
p = make_pilot({"enabled": False})
check("G disabled", p.rtl_start("test", 0.0) is False and p.rtl_state == "OFF", f"state={p.rtl_state}")
print("ALL OK" if ok else "SOME FAILED")
