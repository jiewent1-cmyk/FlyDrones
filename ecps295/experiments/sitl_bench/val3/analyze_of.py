#!/usr/bin/env python3
"""Extract the OF (optical flow) dataflash messages of one run directory + summary."""
import json
import math
import os
import sys

from pymavlink import mavutil

rd = sys.argv[1]
ld = os.path.join(rd, "logs")
out = {"tag": os.path.basename(rd)}
of = []
msgs = []
if os.path.isdir(ld):
    for f in sorted(os.listdir(ld)):
        if not f.endswith(".BIN"):
            continue
        m = mavutil.mavlink_connection(os.path.join(ld, f))
        while True:
            x = m.recv_match(type=["OF", "MSG"])
            if x is None:
                break
            if x.get_type() == "OF":
                of.append((x.TimeUS / 1e6, x.Qual, x.flowX, x.flowY, x.bodyX, x.bodyY))
            else:
                msgs.append((x.TimeUS / 1e6, x.Message))
out["of_count"] = len(of)
if of:
    t0, t1 = of[0][0], of[-1][0]
    out["of_t0"] = round(t0, 3)
    out["of_t1"] = round(t1, 3)
    out["of_span_s"] = round(t1 - t0, 3)
    out["of_hz"] = round((len(of) - 1) / (t1 - t0), 2) if t1 > t0 else None
    for i, name in ((1, "qual"), (2, "flowX"), (3, "flowY"), (4, "bodyX"), (5, "bodyY")):
        vals = [r[i] for r in of]
        out["of_%s_mean" % name] = round(sum(vals) / len(vals), 6)
        out["of_%s_min" % name] = round(min(vals), 6)
        out["of_%s_max" % name] = round(max(vals), 6)
    # gaps between OF messages (the FC logs one OF per flow update)
    gaps = [of[i + 1][0] - of[i][0] for i in range(len(of) - 1)]
    if gaps:
        out["of_gap_mean"] = round(sum(gaps) / len(gaps), 4)
        out["of_gap_max"] = round(max(gaps), 4)
    out["of_sample"] = [[round(v, 6) for v in r] for r in of[:40]]
    out["of_sample_last"] = [[round(v, 6) for v in r] for r in of[-10:]]
    per_s = {}
    for r in of:
        k = int(r[0] - t0)
        per_s[k] = per_s.get(k, 0) + 1
    out["of_per_s"] = [[k, per_s[k]] for k in sorted(per_s)]
out["msgs"] = [m for _, m in msgs][-30:]
print(json.dumps(out, indent=1))
