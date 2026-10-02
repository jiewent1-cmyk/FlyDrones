"""Print selected DataFlash fields at a fixed period: logdump.py LOG T0 T1 PERIOD MSG.FIELD [MSG.FIELD ...]"""

import sys

from pymavlink import mavutil

log, t0, t1, per = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
fields = [f.split(".") for f in sys.argv[5:]]
types = sorted({m for m, _ in fields})
mlog = mavutil.mavlink_connection(log)
cur, last = {}, -1e9
while (x := mlog.recv_match(type=types)) is not None:
    t = x.TimeUS / 1e6
    cur[x.get_type()] = x
    if t0 <= t <= t1 and t - last >= per and all(m in cur for m, _ in fields):
        last = t
        print(f"t={t:6.1f} " + " ".join(f"{m}.{f}={getattr(cur[m], f):.3g}" for m, f in fields))
