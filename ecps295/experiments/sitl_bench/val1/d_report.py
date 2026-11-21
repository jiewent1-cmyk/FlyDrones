#!/usr/bin/env python3
"""d_report.py: D-group extra metrics from dataflash truth (SIM2):
trigger point (AUTO start + 40 s), mode switch to LAND, truth alt min/max after trigger,
distance from trigger point to touchdown point."""
import glob
import math
import os
import re
import sys

from pymavlink import mavutil

D = '/home/jacky/sim/val1'
TRIG = 40.0  # s after AUTO start (fault/cut time given on the command line)


def body_pt(p, T0, yaw0):
    dn = p[1] - T0[1]
    de = p[2] - T0[2]
    c, s = math.cos(yaw0), math.sin(yaw0)
    return (c * dn + s * de, -s * dn + c * de)


for tag in sys.argv[1:]:
    rd = os.path.join(D, 'runs', tag)
    logs = sorted(f for f in os.listdir(os.path.join(rd, 'logs')) if f.endswith('.BIN'))
    sim, msgs, modes, arms, yaws = [], [], [], [], []
    for f in logs:
        m = mavlink_connection = mavutil.mavlink_connection(os.path.join(rd, 'logs', f))
        while True:
            x = m.recv_match(type=['SIM2', 'SIM', 'MSG', 'ARM', 'MODE'])
            if x is None:
                break
            t = x.TimeUS / 1e6
            k = x.get_type()
            if k == 'SIM2':
                sim.append((t, x.PN, x.PE, x.PD))
            elif k == 'SIM':
                yaws.append((t, x.Yaw))
            elif k == 'MSG':
                msgs.append((t, x.Message))
            elif k == 'ARM':
                arms.append((t, x.ArmState))
            elif k == 'MODE':
                modes.append((t, x.ModeNum))
    if not sim:
        print(tag, 'no SIM2')
        continue
    t_arm = next((t for t, s in arms if s == 1), sim[0][0])
    T0 = min((p for p in sim if p[0] >= t_arm), key=lambda p: p[0])
    y0 = min((y for y in yaws if y[0] >= t_arm), key=lambda y: y[0])[1]
    yaw0 = math.radians(y0 if abs(y0) < 180 else y0 - 360)
    t_m1 = next((t for t, msg in msgs if msg.startswith('Mission: 1')), None)
    t_trig = (t_m1 + TRIG) if t_m1 else None
    land_t = next((t for t, n in modes if t_trig and t > t_trig and n == 9), None)
    def at(t):
        return min(sim, key=lambda p: abs(p[0] - t))
    def agl(p):
        return -(p[3] - T0[3])
    trig = at(t_trig) if t_trig else None
    down = [p for p in sim if agl(p) < 0.15]
    td = down[-1] if down else sim[-1]
    dist = None
    if trig:
        a = body_pt(trig, T0, yaw0)
        b = body_pt(td, T0, yaw0)
        dist = round(math.hypot(b[0] - a[0], b[1] - a[1]), 3)
    span = [p for p in sim if t_trig and p[0] >= t_trig]
    print('%-18s t_m1=%.1f t_trig=%.1f land_switch=%s dt_switch=%s alt_max=%.3f alt_min=%.3f land_dist=%s td_t=%.1f' % (
        tag, t_m1 or -1, t_trig or -1,
        ('%.1f' % land_t) if land_t else 'none',
        ('%.1f' % (land_t - t_trig)) if (land_t and t_trig) else '-',
        max(agl(p) for p in span) if span else -1,
        min(agl(p) for p in span) if span else -1,
        dist, td[0]))
