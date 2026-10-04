"""Altitude controller view of a SITL log window: python ctun.py LOG T_FROM T_TO [STEP] (seconds since log start)."""

import sys

from pymavlink import mavutil

m = mavutil.mavlink_connection(sys.argv[1])
t_from, t_to = float(sys.argv[2]), float(sys.argv[3])
step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
t0, nxt, last = None, 0.0, {}
while True:
    x = m.recv_match(type=["CTUN", "RFND", "XKF5", "SIM"], blocking=False)
    if x is None:
        break
    t0 = t0 or x.TimeUS
    t = (x.TimeUS - t0) / 1e6
    last[x.get_type()] = x
    if x.get_type() == "CTUN" and t >= nxt and t_from <= t <= t_to:
        nxt = t + step
        c, r, k5, s = x, last.get("RFND"), last.get("XKF5"), last.get("SIM")
        print(
            f"{t:5.1f} DAlt {c.DAlt:5.2f} Alt {c.Alt:5.2f} BAlt {c.BAlt:5.2f} SAlt {c.SAlt:5.2f} "
            f"DCRt {c.DCRt:+5.2f} CRt {c.CRt:+5.2f} ThO {c.ThO:.2f} | rfnd {getattr(r, 'Dist', 0):.2f} "
            f"st {getattr(r, 'Stat', None)} | HAGL {getattr(k5, 'HAGL', 0):.2f} offset {getattr(k5, 'offset', 0):.2f} "
            f"| truth alt {getattr(s, 'Alt', 0) - 20:.2f}"
        )
