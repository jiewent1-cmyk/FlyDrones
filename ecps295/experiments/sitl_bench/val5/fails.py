#!/usr/bin/env python3
"""List runs that are not clean: mission not completed, mode change, failsafe/variance/fence, or no metrics."""
import glob
import json
import os
import re

D = "/home/jacky/sim/val5"
for f in sorted(glob.glob(D + "/runs/*/metrics.json")):
    r = json.load(open(f)) if os.path.getsize(f) else None
    if r is None:
        print(os.path.basename(os.path.dirname(f)), "| metrics.json empty (run in progress?)")
        continue
    tag = r["tag"]
    msgs = r.get("msgs_all") or r.get("msgs") or []
    flags = []
    if not r.get("mission_complete"):
        flags.append("mission not complete")
    if r.get("left_auto_at") is not None:
        flags.append("left AUTO at %s" % r.get("left_auto_at"))
    if r.get("final_mode") not in (3, None):
        flags.append("final mode %s" % r.get("final_mode"))
    if not r.get("auto_armed", True):
        flags.append("not armed")
    for m in msgs:
        if re.search("variance", m, re.I):
            flags.append("EKF variance: " + m)
        if re.search("failsafe|FS_", m, re.I):
            flags.append("failsafe: " + m)
        if re.search("breach|Fence action|FENCE_ACTION", m, re.I):
            flags.append("fence: " + m)
    if flags:
        print(tag, "|", "; ".join(flags))
for d in sorted(glob.glob(D + "/runs/*/")):
    if not os.path.exists(os.path.join(d, "metrics.json")):
        print(os.path.basename(d.rstrip("/")), "| NO metrics.json")
