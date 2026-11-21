#!/usr/bin/env python3
"""ECPS295: hand-held check of a newly fitted optical flow + rangefinder (MTF-01). Props OFF, FC disarmed.

Runs on the Orange Pi against the mavlink-router diagnostics tap. It only requests message streams; it sends no
commands that move anything.

    flow_check.py [--seconds 120]

Hold the drone level about 0.5 m above the floor, nose forward, press Enter-free: the script takes the first second as
the start point, then prints once per second how far the FC thinks the drone has moved IN THE BODY FRAME (forward,
right), the rangefinder height, and the flow quality. Move it 1 m forward, back, 1 m to the right, back.

    forward must go positive when you move the nose forward, right positive when you move to the right;
    a wrong sign or swapped axes = FLOW_ORIENT_YAW is wrong (do not fly: the position loop would run away);
    the distance shown against the distance really moved gives the flow scale error (FLOW_FXSCALER / FLOW_FYSCALER).
"""
import argparse
import math
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="udpin:127.0.0.1:14570")
ap.add_argument("--seconds", type=float, default=120.0)
a = ap.parse_args()

m = mavutil.mavlink_connection(a.url, source_system=250, source_component=191)
m.wait_heartbeat(timeout=10)
for mid, us in ((32, 100000), (30, 100000), (100, 100000), (173, 100000), (132, 100000), (0, 500000)):
    m.mav.command_long_send(1, 1, 511, 0, mid, us, 0, 0, 0, 0, 0)

st = {"x": None, "y": None, "yaw": None, "rng": float("nan"), "q": -1, "fx": 0.0, "fy": 0.0, "armed": False, "ds": float("nan")}
x0 = y0 = yaw0 = None
t0 = last = time.time()
peak_f = peak_r = 0.0
print("hold level ~0.5 m above the floor, nose forward; start point is taken after 1 s", flush=True)
while time.time() - t0 < a.seconds:
    x = m.recv_match(blocking=True, timeout=0.3)
    if x is not None:
        k = x.get_type()
        if k == "LOCAL_POSITION_NED":
            st["x"], st["y"] = x.x, x.y
        elif k == "ATTITUDE":
            st["yaw"] = x.yaw
        elif k == "RANGEFINDER":
            st["rng"] = x.distance
        elif k == "DISTANCE_SENSOR" and x.orientation == 25:
            st["ds"] = x.current_distance / 100
        elif k == "OPTICAL_FLOW":
            st["q"], st["fx"], st["fy"] = x.quality, x.flow_comp_m_x, x.flow_comp_m_y
        elif k == "HEARTBEAT" and x.get_srcSystem() == 1 and x.get_srcComponent() == 1:
            st["armed"] = bool(x.base_mode & 128)
    now = time.time()
    if st["x"] is None or st["yaw"] is None:
        continue
    if x0 is None and now - t0 >= 1.0:
        x0, y0, yaw0 = st["x"], st["y"], st["yaw"]
        print("start point taken", flush=True)
    if x0 is not None and now - last >= 1.0:
        last = now
        dn, de = st["x"] - x0, st["y"] - y0
        fwd = dn * math.cos(yaw0) + de * math.sin(yaw0)   # body frame of the start heading
        rgt = -dn * math.sin(yaw0) + de * math.cos(yaw0)
        peak_f = fwd if abs(fwd) > abs(peak_f) else peak_f
        peak_r = rgt if abs(rgt) > abs(peak_r) else peak_r
        print("%5.0f s  forward %+6.2f m  right %+6.2f m | height %.2f m | flow quality %3d | yaw %+4.0f deg%s"
              % (now - t0, fwd, rgt, st["rng"], st["q"], math.degrees(st["yaw"] - yaw0),
                 "  *** ARMED ***" if st["armed"] else ""), flush=True)
print("largest excursion: forward %+.2f m, right %+.2f m" % (peak_f, peak_r), flush=True)
