"""S7 / S7c: no-GPS flight on optical flow + 1.2 m rangefinder (TechRoute v2.2 §10).

P0 set EKF origin          P1 ALT_HOLD takeoff (RC override) to 0.6 m
P2 LOITER hover            P3 GUIDED body-velocity steps
P4 S7c-1: fence off, climb 0.1 m/s to 1.5 m (crosses RNGFND1_MAX=1.2), hold, back to 0.6
P5 S7c-2: fence on, climb 0.3 m/s and keep asking for more: FENCE_ALT_MAX=1.3 must stop it (FENCE_ACTION=2 -> LAND)
Truth vs EKF is analysed afterwards from the DataFlash log (s7_analyze.py).
"""

from __future__ import annotations

import argparse
import json
import time

import sitl_util as su

MAV = su.MAV
ap = argparse.ArgumentParser()
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--origin", default="33.6430,-117.8420,20")
ap.add_argument("--phases", default="0,1,2,3,4,5")
a = ap.parse_args()
phases = {int(p) for p in a.phases.split(",")}
events: list[tuple[float, str]] = []
T0 = time.time()
latest: dict = {}


def ev(msg: str) -> None:
    events.append((round(time.time() - T0, 2), msg))
    print(f"[{time.time() - T0:6.1f}] {msg}", flush=True)


m = su.connect(
    a.url,
    rates={
        "LOCAL_POSITION_NED": 20,
        "ATTITUDE": 10,
        "EKF_STATUS_REPORT": 4,
        "RANGEFINDER": 10,
        "OPTICAL_FLOW": 10,
        "SYS_STATUS": 2,
        "HEARTBEAT": 2,
    },
)
rc = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
last_rc = 0.0


def pump(timeout: float = 0.05):
    """Read one message, keep RC override alive, record statustext / mode changes."""
    global last_rc
    if time.time() - last_rc > 0.1:
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
        last_rc = time.time()
    msg = m.recv_match(blocking=True, timeout=timeout)
    if msg is None:
        return None
    k = msg.get_type()
    latest[k] = msg
    if k == "STATUSTEXT":
        ev("FC: " + msg.text)
    elif k == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
        mode = su.mavutil.mode_string_v10(msg)
        if latest.get("_mode") != mode:
            latest["_mode"] = mode
            ev(f"mode -> {mode} armed={bool(msg.base_mode & 128)}")
    return msg


def run_for(secs: float, fn=None) -> None:
    t_end = time.time() + secs
    nxt = 0.0
    while time.time() < t_end:
        if fn and time.time() >= nxt:
            fn()
            nxt = time.time() + 0.05
        pump()


def alt() -> float:
    p = latest.get("LOCAL_POSITION_NED")
    return -p.z if p else float("nan")


def rng() -> float:
    r = latest.get("RANGEFINDER")
    return r.distance if r else float("nan")


def ekf_flags() -> int:
    e = latest.get("EKF_STATUS_REPORT")
    return e.flags if e else 0


def snapshot(tag: str) -> None:
    p = latest.get("LOCAL_POSITION_NED")
    e = latest.get("EKF_STATUS_REPORT")
    if p is None or e is None:
        ev(f"{tag}: no LOCAL_POSITION_NED / EKF_STATUS_REPORT yet")
        return
    ev(
        f"{tag}: ekf_pos=({p.x:+.2f},{p.y:+.2f}) alt={-p.z:.2f} v=({p.vx:+.2f},{p.vy:+.2f},{-p.vz:+.2f}) rng={rng():.2f} "
        f"flags=0x{ekf_flags():04x} var(v={e.velocity_variance:.2f} ph={e.pos_horiz_variance:.2f} pv={e.pos_vert_variance:.2f} "
        f"terr={e.terrain_alt_variance:.2f})"
    )


# ---------------------------------------------------------------- P0
t0 = time.time()
while time.time() - t0 < 60 and not (ekf_flags() & 0x01):
    pump(0.2)
ev(f"EKF attitude ok flags=0x{ekf_flags():04x}")
if 0 in phases:
    lat, lon, alt0 = (float(x) for x in a.origin.split(","))
    m.mav.set_gps_global_origin_send(m.target_system, int(lat * 1e7), int(lon * 1e7), int(alt0 * 1000))
    run_for(5)
    ev(f"origin sent; flags=0x{ekf_flags():04x}")
for _ in range(100):
    pump(0.1)
    if all(k in latest for k in ("LOCAL_POSITION_NED", "RANGEFINDER", "EKF_STATUS_REPORT")):
        break
of = latest.get("OPTICAL_FLOW")
ev(
    f"on ground: rng={rng():.3f} flow_quality={of.quality if of else None} sensors_health=0x{latest['SYS_STATUS'].onboard_control_sensors_health:08x}"
)

# ---------------------------------------------------------------- P1
if 1 in phases:
    m.set_mode("ALT_HOLD")
    run_for(1)
    for _ in range(20):
        m.arducopter_arm()
        run_for(1.5)
        if m.motors_armed():
            break
    if not m.motors_armed():
        ev("ARM FAILED")
        raise SystemExit(1)
    rc[2] = 1700
    t1 = time.time()
    while time.time() - t1 < 20 and not alt() >= 0.6:
        pump()
    rc[2] = 1500
    run_for(4)
    snapshot("P1 airborne ALT_HOLD")

# ---------------------------------------------------------------- P2
if 2 in phases:
    m.set_mode("LOITER")
    run_for(1)
    ev(f"mode now {latest.get('_mode')}")
    p0 = latest["LOCAL_POSITION_NED"]
    run_for(15)
    snapshot("P2 after 15 s LOITER")

# ---------------------------------------------------------------- P3
if 3 in phases:
    m.set_mode("GUIDED")
    run_for(1)
    ev(f"mode now {latest.get('_mode')}")
    for vx, dur in ((0.3, 3), (0, 3), (-0.3, 3), (0, 3)):
        run_for(dur, lambda vx=vx: su.send_vel(m, vx, 0, 0, 0))
        snapshot(f"P3 after vx={vx:+.1f} for {dur}s")

# ---------------------------------------------------------------- P4  (S7c-1)
if 4 in phases:
    su.set_param(m, "FENCE_ENABLE", 0)
    run_for(1)
    ev("P4 fence disabled; climbing 0.1 m/s to 1.5 m")
    t4 = time.time()
    last_print = 0.0
    while time.time() - t4 < 25 and alt() < 1.5:
        su.send_vel(m, 0, 0, -0.1, 0)
        run_for(0.05)
        if time.time() - last_print > 1:
            snapshot("P4 climb")
            last_print = time.time()
    run_for(10, lambda: su.send_vel(m, 0, 0, 0, 0))
    snapshot("P4 hold 10 s at top")
    t4 = time.time()
    while time.time() - t4 < 20 and alt() > 0.6:
        su.send_vel(m, 0, 0, 0.2, 0)
        run_for(0.05)
    run_for(5, lambda: su.send_vel(m, 0, 0, 0, 0))
    snapshot("P4 back at 0.6 m")


# ---------------------------------------------------------------- P5  (S7c-2)
def climb_until_stopped(tag: str, secs: float = 20) -> float:
    t5 = time.time()
    peak = 0.0
    last_print = 0.0
    while time.time() - t5 < secs and m.motors_armed() and latest.get("_mode") == "GUIDED":
        su.send_vel(m, 0, 0, -0.3, 0)
        run_for(0.05)
        peak = max(peak, alt())
        if time.time() - last_print > 1:
            snapshot(tag)
            last_print = time.time()
    ev(f"{tag}: peak EKF alt {peak:.2f} m; mode {latest.get('_mode')} armed={m.motors_armed()}")
    return peak


if 5 in phases:
    su.set_param(m, "FENCE_ENABLE", 1)
    run_for(2)
    ev("P5a fence on + avoidance on: commanding +0.3 m/s climb for 20 s")
    climb_until_stopped("P5a")
    if m.motors_armed():
        su.set_param(m, "AVOID_ENABLE", 0)
        run_for(1)
        ev("P5b avoidance off: commanding +0.3 m/s climb; expect breach at 1.3 -> LAND")
        climb_until_stopped("P5b")
        t6 = time.time()
        while time.time() - t6 < 30 and m.motors_armed():
            pump(0.2)

if m.motors_armed():
    ev(f"landed: {su.land(m)}")
json.dump(events, open("s7_events.json", "w"), indent=0)
