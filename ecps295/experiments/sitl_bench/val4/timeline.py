#!/usr/bin/env python3
"""Timeline of an SQ run relative to AUTO mode-on: liftoff, pass 1 m up, reach 2 m,
corner arrivals (Reached command #N), Land start, pass 1 m down, touchdown."""
import glob, math, os, sys

from pymavlink import mavutil

rd = sys.argv[1]
sim, msgs, modes, arms = [], [], [], []
for f in sorted(glob.glob(os.path.join(rd, "logs", "*.BIN"))):
    m = mavutil.mavlink_connection(f)
    while True:
        x = m.recv_match(type=["SIM2", "MSG", "MODE", "ARM"])
        if x is None:
            break
        k = x.get_type()
        t = x.TimeUS / 1e6
        if k == "SIM2":
            sim.append((t, x.PN, x.PE, x.PD))
        elif k == "MSG":
            msgs.append((t, x.Message))
        elif k == "MODE":
            modes.append((t, x.ModeNum))
        elif k == "ARM":
            arms.append((t, x.ArmState))
sim.sort()
t0 = next((t for t, mn in modes if mn == 3), None)  # AUTO on
if t0 is None:
    sys.exit("no AUTO mode in " + rd)
T0 = [p for p in sim if p[0] <= t0][-1]  # home pos for AGL


def agl(p):
    return -(p[3] - T0[3])


def first(pred):
    return next((round(p[0] - t0, 1) for p in sim if p[0] >= t0 and pred(p)), None)


def msg(pat, occ=1):
    n = 0
    for t, x in msgs:
        if x.startswith(pat):
            n += 1
            if n == occ:
                return round(t - t0, 1)
    return None


fl = [p for p in sim if p[0] >= t0]
t_land = msg("Mission: 6 Land")
down = [p for p in fl if t_land and p[0] >= t_land + t0 and agl(p) <= 1.0]
out = {
    "auto_on": 0.0,
    "liftoff": first(lambda p: agl(p) > 0.05),
    "agl_ge_1m": first(lambda p: agl(p) >= 1.0),
    "agl_ge_2m": first(lambda p: agl(p) >= 1.95),
    "agl_max": round(max(agl(p) for p in fl[:len(fl)//3]) if False else max(agl(p) for p in fl), 2),
    "rc2_first_corner": msg("Reached command #2"),
    "rc3": msg("Reached command #3"),
    "rc4": msg("Reached command #4"),
    "rc5_last_wp": msg("Reached command #5"),
    "land_start": t_land,
    "agl_le_1m_down": round(down[0][0] - t0, 1) if down else None,
    "hit_ground": msg("SIM Hit ground"),
    "disarm": next((round(t - t0, 1) for t, s in arms if t >= t0 and s == 0), None),
}
# body-frame x,y of key points (heading of square legs) for corner/leg identification
print(rd, out)
