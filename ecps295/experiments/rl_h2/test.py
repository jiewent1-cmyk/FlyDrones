"""Geometry checks for MavlinkTwin._ray3/_cone3 (run with the flydrones env, PYTHONPATH=ecps295)."""
import math, types
import mavlink_twin as mt
cls = next(v for v in vars(mt).values() if isinstance(v, type) and hasattr(v, "_cone3"))
Box = types.SimpleNamespace
def drone(boxes):
    d = object.__new__(cls); d.room = types.SimpleNamespace(boxes=boxes); return d
def box(n0, n1, e0, e1, top): return Box(lo=(n0, e0, 0.0), hi=(n1, e1, top))
S = lambda roll=0.0, pitch=0.0, yaw=0.0: types.SimpleNamespace(roll=roll, pitch=pitch, yaw=yaw)
h = math.radians(13.5)
for name, bx, st, want in [
    ("top 2 cm below, 0.40 ahead", box(0.40, 0.80, -0.3, 0.3, 0.73), S(), 0.40 / math.cos(h)),
    ("top 25 cm below (fly-over)", box(0.40, 0.80, -0.3, 0.3, 0.50), S(), 2.0),
    ("top 10 cm below", box(0.40, 0.80, -0.3, 0.3, 0.65), S(), (0.10 / math.tan(h)) / math.cos(h)),
    ("1.1 m stack 0.40 ahead", box(0.40, 0.80, -0.3, 0.3, 1.10), S(), 0.40),
    ("no box, pitched 15 deg nose down", None, S(pitch=-math.radians(15)), 0.75 / math.sin(h + math.radians(15))),
    ("stack to the east, yaw 90 deg", box(-0.3, 0.3, 0.40, 0.80, 1.10), S(yaw=math.pi / 2), 0.40),
]:
    d = drone([bx] if bx else [])
    got = d._cone3(0.0, 0.0, 0.75, st, 0.0, h)
    planar = min(d._ray(0.0, 0.0, 0.75, st.yaw + o) for o in (-h, 0.0, h))
    print(f"{name:34s} cone3 {got:.3f} (expect {min(want, 2.0):.3f})  planar {planar:.3f}  {'OK' if abs(got - min(want, 2.0)) < 0.01 else 'MISMATCH'}")
