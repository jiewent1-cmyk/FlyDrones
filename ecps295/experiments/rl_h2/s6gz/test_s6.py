"""S6 Gazebo emulation checks (flydrones env, PYTHONPATH=ecps295): old path unchanged, zones column, camera model."""
import math, os, types
os.environ["ECPS_TOF_CONE3D"] = "1"
import mavlink_twin as mt
from world_layout import room_from_layout
cls = next(v for v in vars(mt).values() if isinstance(v, type) and hasattr(v, "_zones3"))
S = lambda roll=0.0, pitch=0.0, yaw=0.0: types.SimpleNamespace(roll=roll, pitch=pitch, yaw=yaw)
def box(n0, n1, e0, e1, top, name="b"): return types.SimpleNamespace(lo=(n0, e0, 0.0), hi=(n1, e1, top), name=name)
def drone(boxes, st=S(), pos=(0.0, 0.0, 0.75)):
    d = object.__new__(cls); d.room = types.SimpleNamespace(boxes=boxes)
    d.truth_ned = lambda: pos; d.m = types.SimpleNamespace(messages={"SIM_STATE": st}); return d
ok = True
def check(name, got, want, tol=0.01):
    global ok
    good = abs(got - want) < tol; ok &= good
    print(f"{name:44s} got {got:.3f} want {want:.3f} {'OK' if good else 'MISMATCH'}")
p56 = math.radians(5.625)
check("zones: 1.1 m stack 0.40 ahead", drone([box(0.40, 0.80, -0.3, 0.3, 1.10)])._zones3(0, 0, 0.75, S(), 0.0, p56), 0.40 / math.cos(0.5 * p56))
check("zones: top 10 cm below, 0.40 ahead", drone([box(0.40, 0.80, -0.3, 0.3, 0.65)])._zones3(0, 0, 0.75, S(), 0.0, p56),
      min(0.40 / math.cos(el) for el in [1.5 * p56, 2.5 * p56, 3.5 * p56] if 0.40 * math.tan(el) >= 0.10))
check("zones: open floor, level, 0.75 m", drone([])._zones3(0, 0, 0.75, S(), 0.0, p56), min(2.0, 0.75 / math.sin(3.5 * p56)))
# old signature path unchanged vs the pre-S6 formula (noise 0)
d = drone([box(0.40, 0.80, -0.3, 0.3, 1.10)])
r_old = d.ranges((0.0, 90.0), 27.0, 10.0, 0.0)
check("old path (cone) front", r_old[0], d._cone3(0, 0, 0.75, S(), 0.0, math.radians(13.5)) - 0.06)
# per-beam list: zone column + single-zone cone
d = drone([box(0.40, 0.80, -0.3, 0.3, 1.10)])
r = d.ranges((0.0, 180.0), (5.6, 27.0), 15.0, 0.0)
check("list path: zone column front", r[0], 0.40 / math.cos(0.5 * math.radians(5.6)) - 0.06)
check("list path: rear cone (floor only)", r[1], min(2.0, d._cone3(0, 0, 0.75, S(), math.pi, math.radians(13.5))) - 0.06)
# camera model: planar, nets seen per p_net, noise grows with d^2, max_m
net = box(1.0, 1.02, -2, 2, 3.0, "net_n")
dn = drone([net])  # each new spec gets its own RNG (seed 86000 + number of specs so far)
seen = sum(dn.ranges((0.0,), 10.0, 15.0 + 0.001 * i, 0.0, noise_k=0.0001, p_net=0.5)[0] < 1.5 for i in range(400))
check("camera: net seen in ~half of the runs", seen / 400, 0.5, tol=0.1)
far = drone([box(1.8, 2.0, -0.3, 0.3, 1.1)]).ranges((0.0,), 10.0, 15.0, 0.0, noise_k=0.0001, max_m=1.5)[0]
check("camera: nothing beyond max_m", far, 2.0)
names = sorted(mt._box_name(b) for b in room_from_layout(os.path.expanduser("~/sim/rl_h2/ecps295/gazebo/worlds/ecps295_cage20.layout.json")).boxes)
print("layout box names:", names); ok &= any(n.startswith("net_") for n in names)
print("ALL OK" if ok else "SOME CHECKS FAILED")
