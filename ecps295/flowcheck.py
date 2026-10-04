"""Compare logged optical flow with simulator truth: is flow (and the EKF velocity) consistent with the real motion?

Truth body velocity from SIM (lat/lng/alt + yaw) differentiated; expected sensor flow for ArduPilot's convention:
flowX ~ -v_right/range + gyroX, flowY ~ v_fwd/range + gyroY (OF logs flow rate and body rate separately).
"""

import math
import sys

from pymavlink import mavutil

m = mavutil.mavlink_connection(sys.argv[1])
t_from = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
t_to = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
sim, of, rf, xk, imu = [], [], [], [], []
while True:
    msg = m.recv_match(type=["SIM", "OF", "RFND", "XKF1"], blocking=False)
    if msg is None:
        break
    k = msg.get_type()
    t = msg.TimeUS / 1e6
    if k == "SIM":
        sim.append((t, msg.Lat, msg.Lng, msg.Alt, math.radians(msg.Yaw)))
    elif k == "OF":
        of.append((t, msg.flowX, msg.flowY, msg.bodyX, msg.bodyY))
    elif k == "RFND":
        rf.append((t, msg.Dist))
    elif k == "XKF1" and msg.C == 0:
        xk.append((t, msg.VN, msg.VE))
t0 = sim[0][0]


def at(series, t):
    return min(series, key=lambda r: abs(r[0] - t))


k_lat = 111319.49
print(" t     truthVN truthVE | v_fwd v_rgt  rng | exp fx  fy | of fx  fy (flow-body) | ekf VN  VE")
for i in range(10, len(sim), 10):
    t = sim[i][0]
    if not t_from <= t - t0 <= t_to:
        continue
    a, b = sim[i - 10], sim[i]
    dt = b[0] - a[0]
    vn = (b[1] - a[1]) * k_lat / dt
    ve = (b[2] - a[2]) * k_lat * math.cos(math.radians(b[1])) / dt
    yaw = b[4]
    vf = vn * math.cos(yaw) + ve * math.sin(yaw)
    vr = -vn * math.sin(yaw) + ve * math.cos(yaw)
    r = at(rf, t)[1]
    o = at(of, t)
    x = at(xk, t)
    if r < 0.05:
        continue
    print(
        f"{t - t0:5.1f} {vn:+7.2f} {ve:+7.2f} | {vf:+5.2f} {vr:+5.2f} {r:4.2f} | {-vr / r:+6.2f} {vf / r:+5.2f} | "
        f"{o[1] - o[3]:+6.2f} {o[2] - o[4]:+5.2f} | {x[1]:+6.2f} {x[2]:+5.2f}"
    )
