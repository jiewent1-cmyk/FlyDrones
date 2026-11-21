#!/usr/bin/env python3
"""Summarise one mtf_feed.py run: per phase what the FC reported, and the
timeouts after the feeder stops sending."""
import json
import os
import statistics
import sys

rd = sys.argv[1]
d = json.load(open(os.path.join(rd, "feed.json")))
a = d["args"]
out = {"tag": os.path.basename(rd), "sysid": a["sysid"], "mode": a["mode"]}
ds = d["fc_distance_sensor"]
of = d["fc_optical_flow"]
ss = d["fc_sys_status"]


def phases(spec):
    """schedule spec -> [(t0, t1, [values])]; spec may be a string or the parsed list"""
    items = []
    if isinstance(spec, str):
        for part in spec.split(","):
            if not part:
                continue
            f = part.split(":")
            items.append([float(v) for v in f])
    else:
        items = [list(x) for x in spec]
    res = []
    t = 0.0
    for it in items:
        dur = it[-1]
        res.append((t, t + dur, it[:-1]))
        t += dur
    return res


if a.get("dist_schedule"):
    out["dist_phases"] = []
    for t0, t1, v in phases(a["dist_schedule"]):
        sel = [x for x in ds if t0 <= x[0] < t1]
        rec = {"t0": t0, "t1": t1, "sent_m": v, "n": len(sel)}
        if sel:
            rec["fc_cm"] = sorted(set(x[1] for x in sel))
            rec["fc_orient"] = sorted(set(x[4] for x in sel))
            rec["fc_qual"] = sorted(set(x[5] for x in sel))
            rec["fc_minmax_cm"] = sorted(set((x[2], x[3]) for x in sel))
        out["dist_phases"].append(rec)

if a.get("flow_schedule"):
    out["flow_phases"] = []
    for t0, t1, v in phases(a["flow_schedule"]):
        sel = [x for x in of if t0 <= x[0] < t1]
        rec = {"t0": t0, "t1": t1, "sent_x_y": v, "n": len(sel)}
        if sel:
            rec["fc_rate_x"] = round(statistics.median([x[3] for x in sel]), 5)
            rec["fc_rate_y"] = round(statistics.median([x[4] for x in sel]), 5)
            rec["fc_legacy_x"] = sorted(set(x[1] for x in sel))
            rec["fc_legacy_y"] = sorted(set(x[2] for x in sel))
            rec["fc_qual"] = sorted(set(x[5] for x in sel))
        out["flow_phases"].append(rec)

# quality schedule
if a.get("quality_schedule"):
    out["quality_phases"] = []
    for t0, t1, v in phases(a["quality_schedule"]):
        sel = [x for x in of if t0 <= x[0] < t1]
        rec = {"t0": t0, "t1": t1, "sent_q": v, "n": len(sel)}
        if sel:
            rec["fc_q"] = sorted(set(x[5] for x in sel))
            rec["fc_rate_x"] = round(statistics.median([x[3] for x in sel]), 5)
        out["quality_phases"].append(rec)

# timeouts: when did the FC stop reporting after the feeder stopped sending
if a.get("stop_dist_at", -1) >= 0:
    stop = a["stop_dist_at"]
    after = [x for x in ds if x[0] >= stop]
    out["dist_stop_at"] = stop
    out["dist_last_fc_report"] = after[-1][0] if after else None
    out["dist_last_fc_cm"] = after[-1][1] if after else None
    out["dist_silence_after_s"] = round(after[-1][0] - stop, 3) if after else None
if a.get("stop_flow_at", -1) >= 0:
    stop = a["stop_flow_at"]
    after = [x for x in of if x[0] >= stop]
    out["flow_stop_at"] = stop
    out["flow_last_fc_report"] = after[-1][0] if after else None
    out["flow_silence_after_s"] = round(after[-1][0] - stop, 3) if after else None

# SYS_STATUS: bit 64 = MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW
out["sysstatus_bits"] = []
for t, pres, enab, heal in ss:
    out["sysstatus_bits"].append([t, bool(pres & 64), bool(enab & 64), bool(heal & 64)])
if ss:
    oflow = [x for x in out["sysstatus_bits"] if x[3]]
    out["optflow_health_bit_set"] = len(oflow)
    out["optflow_health_bit_first"] = oflow[0][0] if oflow else None
    out["optflow_health_bit_last"] = oflow[-1][0] if oflow else None
    out["sysstatus_last"] = [ss[-1][0], ss[-1][1], ss[-1][2], ss[-1][3]]

out["sent"] = d["sent"]
out["rx_counts"] = d.get("rx_counts")
out["texts"] = [t[1] for t in d["statustext"]]
print(json.dumps(out, indent=1))
