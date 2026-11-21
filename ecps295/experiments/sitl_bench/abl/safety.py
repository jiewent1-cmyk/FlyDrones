#!/usr/bin/env python3
"""Safety metrics of one run from simulator truth (dataflash SIM2) for a 6 m net cage.

The 2 m square is flown from one corner: mission frame x = ahead of the nose at start, y = to the right, the square
is x, y in [0, 2]. The cage is centred on the square: walls at x, y = -2 and +4, top net at 6 m.
Prints one JSON object: clearances, extents, class.
  UNSAFE    clearance to a net < 0.3 m, or touch-down faster than 1.0 m/s
  DEGRADED  clearance < 1.0 m, or true height > 3.0 m, or touch-down faster than 0.8 m/s
  ABORT     none of the above, mission not completed (landed early by a guard / failsafe)
  OK        mission completed inside the limits
  NOFLY     never armed / never left the ground
"""
import glob
import json
import math
import os
import re
import sys

from pymavlink import mavutil

rd = sys.argv[1]
met = json.load(open(rd + "/metrics.json")) if os.path.exists(rd + "/metrics.json") else {}
out = {"tag": os.path.basename(rd.rstrip("/"))}
logs = sorted(glob.glob(rd + "/logs/*.BIN"))
sim = []
if logs:
    m = mavutil.mavlink_connection(logs[-1])
    while True:
        x = m.recv_match(type=["SIM2"])
        if x is None:
            break
        sim.append((x.TimeUS * 1e-6, x.PN, x.PE, x.PD))
psi = math.radians(met.get("true_yaw0_deg") or 0.0)
air = []
if sim:
    n0, e0, d0 = sim[0][1], sim[0][2], sim[0][3]
    for t, n, e, d in sim:
        alt = -(d - d0)
        if alt > 0.10:
            dn, de = n - n0, e - e0
            air.append((t, dn * math.cos(psi) + de * math.sin(psi), -dn * math.sin(psi) + de * math.cos(psi), alt))
if not air:
    out["cls"] = "NOFLY"
else:
    xs, ys, zs = [a[1] for a in air], [a[2] for a in air], [a[3] for a in air]
    wall = min(min(x + 2.0, 4.0 - x, y + 2.0, 4.0 - y) for x, y in zip(xs, ys))
    top = 6.0 - max(zs)
    td = met.get("touchdown_mps")
    td = abs(td) if isinstance(td, (int, float)) else None
    out.update(alt_max=round(max(zs), 3), x_min=round(min(xs), 2), x_max=round(max(xs), 2), y_min=round(min(ys), 2),
               y_max=round(max(ys), 2), clr_wall=round(wall, 2), clr_top=round(top, 2), clr=round(min(wall, top), 2),
               touchdown=td, complete=bool(met.get("mission_complete")), air_s=round(air[-1][0] - air[0][0], 1),
               n_variance=met.get("n_variance"), land_offset=met.get("land_offset"), track_rms=met.get("track_rms"))
    clr = min(wall, top)
    if clr < 0.3 or (td is not None and td > 1.0):
        out["cls"] = "UNSAFE"
    elif clr < 1.0 or max(zs) > 3.0 or (td is not None and td > 0.8):
        out["cls"] = "DEGRADED"
    else:
        out["cls"] = "OK" if out["complete"] else "ABORT"
hg = rd + "/hg.log"
if os.path.exists(hg):
    txt = open(hg).read()
    arm = re.search(r"^\s*([\d.]+) FC ARMED", txt, re.M)
    trip = re.search(r"^\s*([\d.]+) EVENT HEIGHT GUARD", txt, re.M)
    out["guard"] = "tripped" if trip else ("ran" if "guard cfg" in txt else "no link")
    out["guard_trip_after_arm_s"] = round(float(trip.group(1)) - float(arm.group(1)), 2) if trip and arm else None
    out["guard_blind"] = "height guard blind" in txt
    rtf = [float(v) for v in re.findall(r'"rtf": ([\d.]+)', txt)]
    out["rtf_min"] = min(rtf) if rtf else None
else:
    out["guard"] = "off"
txts = " | ".join(str(t) for t in (met.get("msgs") or []))[:0]
print(json.dumps(out))
