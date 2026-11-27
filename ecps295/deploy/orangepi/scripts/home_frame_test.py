#!/usr/bin/env python3
"""Offline test of RealDrone._to_home_frame (2026-11-25): the pilot's x / y start at the arming point.

No flight controller: the method is called on a stand-in object with the attributes it uses.

    python3 ecps295/deploy/orangepi/scripts/home_frame_test.py
"""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ecps295"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "companion"))
from flydrones.safety import Telemetry  # noqa: E402
from real_drone import RealDrone  # noqa: E402

fails = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global fails
    fails += not ok
    print(("PASS " if ok else "FAIL ") + name + ("" if ok else "  " + detail))


def stand_in(home_frame: bool = True):
    d = SimpleNamespace(home_frame=home_frame, _home_xy=None, _armed=False, events=[])
    d.armed = lambda: d._armed
    d._gevent = lambda what: d.events.append(what)
    return d


def tel(x, y, alt=0.05):
    return Telemetry(alt_m=alt, vz_mps=0.0, yaw_deg=30.0, x_m=x, y_m=y, battery_pct=80.0)


f = RealDrone._to_home_frame

# 1. disarmed: FC frame unchanged, no home
d = stand_in()
t = f(d, tel(9.0, 2.9))
check("disarmed: x / y unchanged", (t.x_m, t.y_m) == (9.0, 2.9), str((t.x_m, t.y_m)))
check("disarmed: no home", d._home_xy is None)

# 2. arming fixes the home at the current FC position; the pilot sees (0, 0)
d._armed = True
raw = tel(9.0, 2.9)
t = f(d, raw)
check("armed: pilot at (0, 0)", abs(t.x_m) < 1e-9 and abs(t.y_m) < 1e-9, str((t.x_m, t.y_m)))
check("armed: one event", len(d.events) == 1 and "take-off point" in d.events[0], str(d.events))
check("armed: input object not modified", (raw.x_m, raw.y_m) == (9.0, 2.9), str((raw.x_m, raw.y_m)))
check("armed: other fields kept", t.alt_m == 0.05 and t.yaw_deg == 30.0 and t.battery_pct == 80.0)

# 3. flying 1.5 m north / 0.5 m west of the take-off point
t = f(d, tel(10.5, 2.4, alt=1.0))
check("in flight: relative x / y", abs(t.x_m - 1.5) < 1e-9 and abs(t.y_m + 0.5) < 1e-9, str((t.x_m, t.y_m)))
check("in flight: home not moved", d._home_xy == (9.0, 2.9), str(d._home_xy))

# 4. disarm clears the home; the next arming takes the new spot
d._armed = False
f(d, tel(10.4, 2.5))
check("disarm clears home", d._home_xy is None)
d._armed = True
t = f(d, tel(-31.4, 44.0))
check("re-arm: new home", d._home_xy == (-31.4, 44.0) and abs(t.x_m) < 1e-9 and abs(t.y_m) < 1e-9, str(d._home_xy))

# 5. no position yet: passed through, no home taken
d = stand_in()
d._armed = True
t = f(d, Telemetry(alt_m=None, x_m=None, y_m=None))
check("no position: passed through", t.x_m is None and d._home_xy is None)

# 6. switched off: FC frame even when armed (config home_frame: false)
d = stand_in(home_frame=False)
d._armed = True
t = f(d, tel(9.0, 2.9))
check("home_frame off: FC frame", (t.x_m, t.y_m) == (9.0, 2.9) and d._home_xy is None)

# 7. sim drone arming at the origin: no-op
d = stand_in()
d._armed = True
t = f(d, tel(0.0, 0.0))
t = f(d, tel(0.7, -0.2, alt=0.8))
check("origin arming: no-op", abs(t.x_m - 0.7) < 1e-9 and abs(t.y_m + 0.2) < 1e-9)

print("%d failed" % fails)
sys.exit(1 if fails else 0)
