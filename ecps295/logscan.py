"""Timeline after arming from a SITL dataflash log: attitude vs desired, rangefinder, flow, EKF velocity, altitude."""

import collections
import sys

from pymavlink import mavutil

m = mavutil.mavlink_connection(sys.argv[1])
span = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25
c = collections.Counter()
evs, rows = [], []
while True:
    msg = m.recv_match(blocking=False)
    if msg is None:
        break
    k = msg.get_type()
    c[k] += 1
    if k == "EV":
        evs.append((msg.TimeUS / 1e6, msg.Id))
    if k in ("ATT", "RFND", "OF", "XKF1", "CTUN", "MSG"):
        rows.append((msg.TimeUS / 1e6, k, msg))
print("types:", ", ".join(f"{k}:{v}" for k, v in sorted(c.items()) if k in ("ATT", "RFND", "OF", "XKF1", "CTUN", "IMU", "SIM")))
print("events:", evs)
t_arm = next((t for t, i in evs if i == 10), rows[0][0] if rows else 0.0)  # logging may start at arming
last, nxt = {}, t_arm
for t, k, msg in rows:
    if t < t_arm:
        continue
    if k == "MSG":
        print(f"  t={t - t_arm:5.2f} MSG {msg.Message}")
        continue
    last[k] = msg
    if t >= nxt and "ATT" in last:
        nxt += step
        a, o, r, x, ct = (last.get(n) for n in ("ATT", "OF", "RFND", "XKF1", "CTUN"))
        print(
            f"t={t - t_arm:5.2f} roll {a.Roll:6.1f}/{a.DesRoll:6.1f} pitch {a.Pitch:6.1f}/{a.DesPitch:6.1f} "
            f"rfnd {getattr(r, 'Dist', None)} of q{getattr(o, 'Qual', None)} fx {getattr(o, 'flowX', 0):+.2f} "
            f"fy {getattr(o, 'flowY', 0):+.2f} ekfV {getattr(x, 'VN', 0):+.2f},{getattr(x, 'VE', 0):+.2f} "
            f"alt {getattr(ct, 'Alt', 0):.2f} thO {getattr(ct, 'ThO', 0):.2f}"
        )
    if t - t_arm > span:
        break
