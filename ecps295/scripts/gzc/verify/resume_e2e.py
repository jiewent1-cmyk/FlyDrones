"""Route resume against Gazebo + SITL (2026-11-04): during a route hide the FC heartbeat from the companion for <gap> s
(hb_gap_relay.py: SIGSTOP of arducopter stalled the whole simulation for over 30 s) and watch the console mode and events.

usage: resume_e2e.py --inst 5 --key-file console.key --tag T --out o.json --flag gap.flag --pts '[[1,0],[1,1],[0,1]]' --gaps 6,18
Expected: a gap whose control loss stays within route_resume_s resumes the route; a longer one is a normal hand-over.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ecl_client import Link, decode  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--inst", type=int, required=True)
ap.add_argument("--key-file", required=True)
ap.add_argument("--tag", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--pts", required=True)
ap.add_argument("--gaps", required=True)
ap.add_argument("--flag", required=True)
a = ap.parse_args()
pts = json.loads(a.pts)
link = Link("127.0.0.1", 14700 + 10 * a.inst, name="resume_e2e", cam="off", lvl=3, key=Path(a.key_file).read_bytes())
st = {"tick": {}, "events": {}, "acks": {}, "status": {}}
hello = 0.0


def pump(secs, until=lambda: False):
    global hello
    end = time.time() + secs
    while True:
        now = time.time()
        if now - hello >= 0.2:
            link.hello_tick()
            hello = now
        m = link.recv()
        if m is not None:
            typ, body, _ = m
            if typ in (1, 7, 8):
                d = decode(typ, body)
                if typ == 1:
                    st["tick"] = d
                elif typ == 7:
                    st["acks"][d.get("id")] = d
                else:
                    st["status"] = d
                    for e in d.get("events", []):
                        if e[0] not in st["events"]:
                            st["events"][e[0]] = [round(now, 2), e[2], e[3]]
        if until():
            return True
        if now >= end:
            return False


def send(op, **kw):
    for _ in range(5):
        cid = link.cmd(op, **kw)
        if pump(1.5, lambda: cid in st["acks"]):
            return st["acks"][cid]
    return None


def ctl():
    return st["tick"].get("ctl") == 1 and st["tick"].get("arm") == 1


res = {"cases": []}
pump(120, lambda: st["status"])
res["companion_tag"] = (st["status"].get("run") or {}).get("tag")
assert res["companion_tag"] == a.tag, res["companion_tag"]
assert pump(300, ctl), "no control"
send("hold")
pump(4)
for gap in [float(g) for g in a.gaps.split(",")]:
    c = {"gap": gap}
    assert (send("route", pts=pts) or {}).get("ok"), "route refused"
    pump(10)
    c["op_before"], c["ri_before"] = st["tick"].get("op"), st["tick"].get("ri")
    t0 = time.time()
    Path(a.flag).touch()
    c["lost"] = pump(gap, lambda: st["tick"].get("ctl") != 1)
    pump(max(0.0, gap - (time.time() - t0)))
    Path(a.flag).unlink()
    c["back"] = pump(60, ctl)
    pump(3)
    c["t_back"] = round(time.time() - t0, 1)
    c["op_after"], c["ri_after"] = st["tick"].get("op"), st["tick"].get("ri")
    c["resumed"] = any("route resumed" in e[2] and e[0] >= t0 for e in st["events"].values())
    if c["op_after"] == "GOTO":
        pump(180, lambda: st["tick"].get("op") != "GOTO")
    c["op_end"] = st["tick"].get("op")
    c["events"] = [e for e in sorted(st["events"].values()) if e[0] >= t0 - 1 and e[1] == "operator"][:25]
    c["t_end"] = round(time.time() - t0, 1)
    res["cases"].append(c)
    print(json.dumps({k: v for k, v in c.items() if k != "events"}), flush=True)
    send("hold")
    pump(5)
send("land")
pump(60, lambda: st["tick"].get("arm") == 0)
Path(a.out).write_text(json.dumps(res, indent=1))
