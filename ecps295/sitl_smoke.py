"""Headless SITL smoke test: GUIDED takeoff -> hover -> LAND, report altitude stats and learned hover throttle."""

import argparse
import statistics
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
ap.add_argument("--alt", type=float, default=0.6)
ap.add_argument("--hover", type=float, default=30)
a = ap.parse_args()

m = mavutil.mavlink_connection(a.url, source_system=255)
m.wait_heartbeat(timeout=60)
print("heartbeat from sys", m.target_system)
for name, rate in (
    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
    (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
):
    m.mav.command_long_send(
        m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, name, 1e6 / rate, 0, 0, 0, 0, 0
    )


def param(name):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
    return msg.param_value if msg else None


def statustext_drain():
    while s := m.recv_match(type="STATUSTEXT", blocking=False):
        print("  [FC]", s.text)


t0 = time.time()
while time.time() - t0 < 120:
    r = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
    statustext_drain()
    if r and (r.flags & 0x1F) == 0x1F and (r.flags & (1 << 7)) == 0:  # att, vel h/v, pos h rel/abs; not const_pos
        break
print(f"EKF ready after {time.time() - t0:.0f}s")
print("MOT_THST_HOVER before:", param("MOT_THST_HOVER"), " BATT_CAPACITY:", param("BATT_CAPACITY"))

m.set_mode("GUIDED")
for _ in range(30):
    m.arducopter_arm()
    m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if m.motors_armed():
        break
    statustext_drain()
    time.sleep(1)
print("armed:", m.motors_armed())
if not m.motors_armed():
    raise SystemExit("ARM FAILED")
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, a.alt)

alts, tstart = [], time.time()
while time.time() - tstart < 15 + a.hover:
    p = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
    statustext_drain()
    if p and time.time() - tstart > 15:
        alts.append(-p.z)
print(
    f"hover {a.hover:.0f}s: n={len(alts)} mean={statistics.mean(alts):.3f} m  sd={statistics.pstdev(alts):.3f} m  "
    f"min={min(alts):.3f} max={max(alts):.3f}"
)
s = m.recv_match(type="SYS_STATUS", blocking=True, timeout=3)
if s:
    print(f"battery: {s.voltage_battery / 1000:.2f} V  {s.current_battery / 100:.1f} A  remaining {s.battery_remaining}%")
print("MOT_THST_HOVER after:", param("MOT_THST_HOVER"))
m.set_mode("LAND")
t1 = time.time()
while time.time() - t1 < 40 and m.motors_armed():
    m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    statustext_drain()
print("landed+disarmed:", not m.motors_armed(), f"after {time.time() - t1:.0f}s")
