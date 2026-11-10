"""2026-11-09 waypoint altitude floor (ecps_pilot.EcpsPilot._wp_alt_floor), no simulator."""
# run: PYTHONPATH=src:ecps295 python ecps295/wp_alt_floor_test.py
import math, sys, types
from ecps_pilot import EcpsPilot
from flydrones.motor.command import FlightCommand

def layer(**wp):
    p = EcpsPilot.__new__(EcpsPilot)  # only the floor's own fields
    p.wp_min_alt = float(wp.get("min_alt_m", 0.85)); p.wp_alt_gain = float(wp.get("alt_gain", 1.5))
    p.wp_alt_max_climb = float(wp.get("alt_max_climb", 0.3)); p.wp_alt_ticks = 0
    return p

T = lambda a: types.SimpleNamespace(alt_m=a)
ok = 0
def check(name, cond):
    global ok
    print(("PASS " if cond else "FAIL ") + name); ok += bool(cond)
    if not cond: sys.exit(1)

p = layer()
c = FlightCommand(throttle=-0.2, yaw=0.3, forward=-0.3, escape=True, note="brain escape")
r = p._wp_alt_floor(c, T(0.70))
check("below floor: descent replaced by a climb", math.isclose(r.throttle, 0.225) and r.yaw == 0.3 and r.forward == -0.3 and r.escape)
check("climb bounded by alt_max_climb", math.isclose(p._wp_alt_floor(c, T(0.2)).throttle, 0.3))
check("above floor: unchanged", p._wp_alt_floor(c, T(0.95)) is c)
up = FlightCommand(throttle=0.5)
check("a stronger brain climb is kept", p._wp_alt_floor(up, T(0.6)) is up)
check("no height: unchanged", p._wp_alt_floor(c, T(None)) is c and p._wp_alt_floor(c, T(float("nan"))) is c)
check("min_alt_m 0 = off", layer(min_alt_m=0)._wp_alt_floor(c, T(0.2)) is c)
check("ticks counted", p.wp_alt_ticks == 2)
print(f"{ok}/7 passed")
