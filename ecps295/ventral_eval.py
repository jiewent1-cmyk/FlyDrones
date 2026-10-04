"""Offline check of ecps_ventral.VentralCue on SITL DataFlash logs of S7d runs (BARO, RFND, SIM truth).

Truth "over an obstacle": the ToF cone footprint (radius height x tan 13.5 deg) overlaps a layout box whose top is
at least --min-step below the sensor. Per crossing: detected if the level reaches 0.5 between 0.3 s before entering
and 1 s after; latency = first level >= 0.5 - entry. False alarms: level >= 0.5 episodes that start with no box under
the cone within 0.5 s.

    python ventral_eval.py LAYOUT.json RUN_DIR [...] [--home 33.6430,-117.8420,20]
"""

import argparse
import glob
import json
import math
import os

from ecps_ventral import VentralCue
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("layout")
ap.add_argument("runs", nargs="+")
ap.add_argument("--home", default="33.6430,-117.8420,20")
ap.add_argument("--min-step", type=float, default=0.12)
ap.add_argument("--hz", type=float, default=20.0)
ap.add_argument("--rise", type=float, default=0.12)
ap.add_argument("--smooth", type=float, default=0.4)
ap.add_argument("--drop", type=float, default=0.12)
ap.add_argument("--rate", type=float, default=0.8)
ap.add_argument("--no-step", action="store_true", help="rise part only")
ap.add_argument("--no-rise", action="store_true", help="step part only")
ap.add_argument("-v", action="store_true")
a = ap.parse_args()

lat0, lon0, alt0 = (float(x) for x in a.home.split(","))
K = 111319.49
boxes = []
for b in json.load(open(a.layout))["boxes"]:
    if b["name"].startswith("net"):
        continue
    (e, n, z), (se, sn, sz) = b["center_enu"], b["size_enu"]
    boxes.append((b["name"], n - sn / 2, n + sn / 2, e - se / 2, e + se / 2, z + sz / 2))
TAN, SENSOR_DZ = math.tan(math.radians(13.5)), 0.015


def under(n, e, alt):
    h = alt - SENSOR_DZ
    for name, n0, n1, e0, e1, top in boxes:
        if top < h - a.min_step and math.hypot(max(n0 - n, 0, n - n1), max(e0 - e, 0, e - e1)) <= h * TAN:
            return name
    return None


def nearest(n, e, alt):
    """(gap m from the cone footprint edge to the nearest box, how far its top is below the sensor)"""
    h = alt - SENSOR_DZ
    best = (9.0, 0.0, "-")
    for name, n0, n1, e0, e1, top in boxes:
        gap = math.hypot(max(n0 - n, 0, n - n1), max(e0 - e, 0, e - e1)) - h * TAN
        best = min(best, (round(gap, 2), round(h - top, 2), name))
    return best


def load(log):
    m = mavutil.mavlink_connection(log)
    ev = []
    while True:
        x = m.recv_match(type=["BARO", "RFND", "SIM"], blocking=False)
        if x is None:
            return ev
        k = x.get_type()
        if k == "BARO" and getattr(x, "I", 0) != 0:
            continue
        ev.append((x.TimeUS / 1e6, k, x))


tot = {"cross": 0, "det": 0, "false": 0, "lat": [], "flight_s": 0.0}
for d in a.runs:
    logs = sorted(glob.glob(os.path.join(d, "logs", "*.BIN")))
    if not logs:
        continue
    ev = load(logs[-1])
    cue = VentralCue(rise_m=a.rise, smooth_s=a.smooth, drop_m=a.drop, min_rate_mps=a.rate)
    if a.no_step:
        cue.drop_m = 99.0
    if a.no_rise:
        cue.rise_m = 99.0
    last = {}
    i, t, step = 0, ev[0][0], 1.0 / a.hz
    trace = []  # (t, level, box under the cone, truth alt)
    while i < len(ev):
        while i < len(ev) and ev[i][0] <= t:
            last[ev[i][1]] = ev[i][2]
            i += 1
        s, r, b = last.get("SIM"), last.get("RFND"), last.get("BARO")
        if s is not None and r is not None and b is not None:
            n = (s.Lat - lat0) * K
            e = (s.Lng - lon0) * K * math.cos(math.radians(lat0))
            alt = s.Alt - alt0
            lvl = cue.update(t, r.Dist, b.Alt, step)
            if alt > 0.2:  # in flight
                trace.append((t, lvl, under(n, e, alt), alt, nearest(n, e, alt), cue.step, cue.rise))
        t += step
    cross, det, lat, falses = 0, 0, [], 0
    prev_u = None
    for tt, _lvl, u, alt, *_ in trace:
        if u and not prev_u:
            cross += 1
            hit = next((x[0] for x in trace if tt - 0.3 <= x[0] <= tt + 1.0 and x[1] >= 0.5), None)
            if hit is not None:
                det += 1
                lat.append(hit - tt)
            elif a.v:
                print(f"    missed {u} at {tt - trace[0][0]:6.1f} s alt {alt:.2f}")
        prev_u = u
    on = False
    for tt, lvl, _u, alt, nb, stp, _ris in trace:
        if lvl >= 0.5 and not on and not any(x[2] for x in trace if abs(x[0] - tt) <= 0.5):
            falses += 1
            if a.v:
                print(
                    f"    false at {tt - trace[0][0]:6.1f} s alt {alt:.2f} by {'step' if stp else 'rise'} nearest box (gap, below) {nb}"
                )
        on = lvl >= 0.5
    tot["cross"] += cross
    tot["det"] += det
    tot["false"] += falses
    tot["lat"] += lat
    tot["flight_s"] += len(trace) / a.hz
    print(f"{os.path.basename(d.rstrip('/')):22s} crossings {cross:2d} detected {det:2d} false {falses:2d}")
lat = sorted(tot["lat"])
med = lat[len(lat) // 2] if lat else float("nan")
print(
    f"TOTAL crossings {tot['cross']} detected {tot['det']} ({tot['det'] / max(1, tot['cross']):.0%}), "
    f"latency median {med:+.2f} s; false alarms {tot['false']} in {tot['flight_s'] / 60:.1f} min of flight"
)
