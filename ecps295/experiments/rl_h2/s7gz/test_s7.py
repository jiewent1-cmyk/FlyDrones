"""S7 Gazebo emulation checks (flydrones env, PYTHONPATH=ecps295): zones() vs the scalar _ray3, scalar multi-zone path,
27-deg cone path unchanged, map shield end to end on a stub."""
import math, os, types
os.environ["ECPS_TOF_CONE3D"] = "1"
import numpy as np
import mavlink_twin as mt
from ecps_map import zone_dirs
cls = next(v for v in vars(mt).values() if isinstance(v, type) and hasattr(v, "zones"))
S = lambda roll=0.0, pitch=0.0, yaw=0.0: types.SimpleNamespace(roll=roll, pitch=pitch, yaw=yaw)
def box(n0, n1, e0, e1, top, name="b"): return types.SimpleNamespace(lo=(n0, e0, 0.0), hi=(n1, e1, top), name=name)
def drone(boxes, st=S(), pos=(0.0, 0.0, 0.75)):
    d = object.__new__(cls); d.room = types.SimpleNamespace(boxes=boxes)
    d.truth_ned = lambda: pos; d.m = types.SimpleNamespace(messages={"SIM_STATE": st}); return d
ok = True
def check(name, got, want, tol=0.01):
    global ok
    good = abs(got - want) < tol; ok &= good
    print(f"{name:52s} got {got:.3f} want {want:.3f} {'OK' if good else 'MISMATCH'}")
stack = box(0.40, 0.80, -0.5, 0.5, 1.10)
for st, name in ((S(), "level"), (S(pitch=-0.15), "pitched -0.15 rad"), (S(yaw=0.4, roll=0.1), "yaw 0.4, roll 0.1")):
    d = drone([stack], st)
    z = d.zones(0.0, 60.0, 60.0, 8, 15.0, 0.0, 3.5)
    B = zone_dirs(0.0, 60.0, 60.0, 8)
    worst = 0.0
    for k in range(64):  # same ray through the scalar _ray3 with the same rotation
        x, y, zf = B[k, 0], B[k, 1], -B[k, 2]
        cr, sr, cp, sp = math.cos(st.roll), math.sin(st.roll), math.cos(st.pitch), math.sin(st.pitch); cy, sy = math.cos(st.yaw), math.sin(st.yaw)
        dn = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * zf
        de = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * zf
        dd = -sp * x + sr * cp * y + cr * cp * zf
        o = (0.06 * math.cos(st.yaw), 0.06 * math.sin(st.yaw), 0.75)
        ref = d._ray3(o, (dn, de, -dd))
        got = z[k] if np.isfinite(z[k]) else 2.0
        worst = max(worst, abs(min(got, 2.0) - min(ref, 2.0)))
    check(f"zones() == _ray3 on all 64 rays ({name})", worst, 0.0, 1e-6)
d = drone([stack])
check("scalar 5.6-deg beam -> zone column (1.1 m stack 0.40)", d.ranges((0.0,), 5.6, 15.0, 0.0)[0], 0.40 / math.cos(0.5 * math.radians(5.6)) - 0.06)
d = drone([stack])
check("27-deg beam still the cone", d.ranges((0.0,), 27.0, 10.0, 0.0)[0], d._cone3(0, 0, 0.75, S(), 0.0, math.radians(13.5)) - 0.06)
from ecps_struct import RangeShield
from flydrones.motor.command import FlightCommand
sh = RangeShield({"beams": [], "unstick_s": 1.0, "map": {"enabled": True, "sensors": [{"bearing": 0.0, "hfov": 60, "vfov": 60, "max_m": 3.5}]}})
wall = box(0.35, 0.45, -1.0, 1.0, 1.5)
d = drone([wall]); d.v_max = 0.5
tel = types.SimpleNamespace(x_m=0.0, y_m=0.0, yaw_deg=0.0, alt_m=0.75)
cmd = sh.filter(FlightCommand(forward=1.0), d, 0.0, tel)
# voxel quantisation + 1 cm range noise put the nearest mapped surface up to one voxel (5 cm) short: forward between a slight
# retreat and +0.08 (the exact-reading clamp)
check("map shield: forward 1.0 clamped near the wall (0.35 m)", min(max(cmd.forward, -0.15), 0.1), cmd.forward, 1e-9)
print("ALL OK" if ok else "SOME CHECKS FAILED")
