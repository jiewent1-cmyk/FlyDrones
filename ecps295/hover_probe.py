"""Hover in GUIDED and print position/velocity/EKF variances at 2 Hz to diagnose drift."""

import argparse
import time

import sitl_util as su

ap = argparse.ArgumentParser()
ap.add_argument("--alt", type=float, default=1.0)
ap.add_argument("--secs", type=float, default=40)
a = ap.parse_args()
m = su.connect(rates={"LOCAL_POSITION_NED": 20, "EKF_STATUS_REPORT": 2, "ATTITUDE": 10, "VFR_HUD": 2})
print(f"EKF ready after {su.wait_ekf(m):.0f}s", flush=True)
su.arm_takeoff(m, a.alt, settle=0)
t0 = time.time()
nxt = 0
last = {}
while time.time() - t0 < a.secs:
    msg = m.recv_match(type=["LOCAL_POSITION_NED", "EKF_STATUS_REPORT", "ATTITUDE", "VFR_HUD"], blocking=True, timeout=0.1)
    su.drain_text(m)
    if msg:
        last[msg.get_type()] = msg
    if time.time() >= nxt and "LOCAL_POSITION_NED" in last:
        p = last["LOCAL_POSITION_NED"]
        e = last.get("EKF_STATUS_REPORT")
        at = last.get("ATTITUDE")
        h = last.get("VFR_HUD")
        print(
            f"t={time.time() - t0:5.1f} pos=({p.x:+.2f},{p.y:+.2f},{-p.z:.2f}) v=({p.vx:+.2f},{p.vy:+.2f},{-p.vz:+.2f})"
            + (f" rp=({at.roll * 57.3:+.1f},{at.pitch * 57.3:+.1f}) yaw={at.yaw * 57.3:+.0f}" if at else "")
            + (f" thr={h.throttle}% " if h else "")
            + (
                f" ekf vel={e.velocity_variance:.2f} pos={e.pos_horiz_variance:.2f} hgt={e.pos_vert_variance:.2f} mag={e.compass_variance:.2f}"
                if e
                else ""
            ),
            flush=True,
        )
        nxt = time.time() + 0.5
print("landed:", su.land(m))
