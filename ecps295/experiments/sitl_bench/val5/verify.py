#!/usr/bin/env python3
"""Injection evidence for one run dir: comp.csv event counts + sample rows."""
import csv
import json
import os
import sys

rd = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(rd, "comp.csv"))))
ev = {}
for r in rows:
    ev[r["ev"]] = ev.get(r["ev"], 0) + 1
print("events:", ev)
for kind in ("obs_bad", "obs_badyaw", "block"):
    sel = [r for r in rows if r["ev"] == kind]
    if not sel:
        continue
    print(kind, "first:", sel[0], "last:", sel[-1], "n=", len(sel))
sent = [r for r in rows if r["ev"] == "sent"]
obs = [r for r in rows if r["ev"].startswith("obs")]
print("n obs", len(obs), "n sent", len(sent))
mj = None
for line in open(os.path.join(rd, "run.log")):
    if line.startswith("RESULT_JSON"):
        mj = json.loads(line[12:])
if mj:
    print("inj:", mj.get("inj"), "t_auto:", mj.get("t_auto"), "fixes_sent:", mj.get("fixes_sent"),
          "innov_rej:", mj.get("gate_innov_rej"))
    print("companion msgs:", [t for t in mj.get("texts", []) if "COMPANION" in t])
