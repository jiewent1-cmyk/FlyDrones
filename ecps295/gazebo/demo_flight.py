"""Short GUIDED demo flight for watching ecps295_quad and its camera in the Gazebo GUI (camtest world: boxes ahead).

takeoff 0.6 m -> full 360 deg yaw at 30 deg/s -> creep 0.75 m toward the boxes and back -> climb to 0.9 m -> land
"""

import argparse
import math
import time

import sitl_util as su

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="tcp:127.0.0.1:5760")
a = ap.parse_args()

m = su.connect(a.url)
print(f"EKF ready after {su.wait_ekf(m):.0f}s", flush=True)
su.arm_takeoff(m, 0.6, settle=6)
print(f"airborne; steady after {su.wait_steady(m, 0.6):.1f}s", flush=True)


def hold(vx: float, vz_up: float, yaw_dps: float, secs: float, label: str) -> None:
    print(label, flush=True)
    t_end = time.time() + secs
    while time.time() < t_end:
        su.send_vel(m, vx, 0, -vz_up, math.radians(yaw_dps))
        m.recv_match(blocking=True, timeout=0.05)
        su.drain_text(m)


hold(0, 0, 0, 3, "hover")
hold(0, 0, 30, 12, "yaw 360 deg at 30 deg/s")
hold(0, 0, 0, 2, "hover")
hold(0.25, 0, 0, 3, "forward 0.25 m/s toward the boxes")
hold(0, 0, 0, 3, "hover close to the boxes")
hold(-0.25, 0, 0, 3, "back 0.25 m/s")
hold(0, 0.15, 0, 2, "climb to ~0.9 m")
hold(0, 0, 0, 3, "hover")
print("landed:", su.land(m), flush=True)
