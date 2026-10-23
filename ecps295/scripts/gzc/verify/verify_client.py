"""Scripted ground console for the planner verification (2026-10-22): plays a plan from gen_routes.py over the console link
(ECL, same protocol as the Mac app, via deploy/console/tools/ecl_client.py) and records what happened.

usage: verify_client.py --port 14750 --plan plan.json --out client.json [--host 127.0.0.1] [--timeout 2400]

Per step (route): send it, wait for GOTO to end (the console goes back to HOLD with a reason) or a timeout
(45 s per waypoint + 30 s), then HOLD 3 s. Optional keys:
  change_after s + change_pts   send a different route mid-flight
  estop_after s                 emergency LAND mid-route, wait for the disarm, estop_reset, wait until the sim pilot has
                                taken off and the companion is in control again, then fly after_pts
  expect_refuse                 the console must refuse the route (ACK false); nothing flies
All console events (STATUS event log, de-duplicated) and the TICK stream (mode, control, position) are kept with wall
times, so the analysis can cut Gazebo truth (run.csv "wall") per step.
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
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, required=True)
ap.add_argument("--plan", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--timeout", type=float, default=2400)
ap.add_argument("--control-timeout", type=float, default=300)
ap.add_argument("--tag", default="", help="the session tag: the companion must report it (STATUS run.tag), else abort")
a = ap.parse_args()

plan = json.load(open(a.plan))
link = Link(a.host, a.port, name="verify_client", cam="off", lvl=3)
st = {"tick": None, "events": {}, "acks": {}, "ticks": [], "status": None}
T0 = time.time()
_last_hello = 0.0


def pump(secs: float = 0.0, until=None) -> bool:
    """Keep the link alive and read messages for secs (or until until() is true). Returns until()'s last value."""
    global _last_hello
    end = time.time() + secs
    while True:
        now = time.time()
        if now - _last_hello >= 0.2:
            link.hello_tick()
            _last_hello = now
        m = link.recv()
        if m is not None:
            typ, body, _ = m
            if typ in (1, 7, 8):
                d = decode(typ, body)
                if typ == 1:
                    st["tick"] = d
                    if len(st["ticks"]) == 0 or now - st["ticks"][-1][0] >= 0.2:
                        tel = d.get("tel") or [None, None, None, None]
                        st["ticks"].append([round(now, 3), d.get("op"), d.get("ctl"), d.get("arm"), d.get("fc"), tel[0], tel[1], tel[2],
                                            d.get("es"), d.get("ri")])
                elif typ == 7:
                    st["acks"][d.get("id")] = d
                else:
                    st["status"] = d
                    for e in d.get("events", []):
                        if e[0] not in st["events"]:
                            st["events"][e[0]] = [round(now, 3), e[2], e[3]]
        if until is not None and until():
            return True
        if now >= end:
            return bool(until()) if until is not None else True
        if now - T0 > a.timeout:
            raise TimeoutError("client timeout")


def tick(k, default=None):
    t = st["tick"]
    return default if t is None else t.get(k, default)


def send(op: str, **kw) -> dict | None:
    for _ in range(5):
        cid = link.cmd(op, **kw)
        if pump(1.5, lambda: cid in st["acks"]):
            return st["acks"][cid]
    return None


def wait_control(secs: float) -> bool:
    return pump(secs, lambda: tick("ctl") == 1 and tick("arm") == 1)


def fly_route(pts, rec: dict, step: dict) -> None:
    ack = send("route", pts=pts)
    rec["ack"] = ack
    if ack is None or not ack.get("ok"):
        return
    t_start = time.time()
    limit = 45.0 * len(pts) + 30.0
    seen = pump(5.0, lambda: tick("op") == "GOTO")
    rec["goto_seen"] = seen
    changed = estopped = False
    while time.time() - t_start < limit:
        pump(0.5)
        el = time.time() - t_start
        if not changed and step.get("change_after") and el > step["change_after"]:
            changed = True
            rec["change_ack"] = send("route", pts=step["change_pts"])
            limit = el + 45.0 * len(step["change_pts"]) + 30.0
            continue
        if not estopped and step.get("estop_after") and el > step["estop_after"]:
            estopped = True
            rec["estop"] = estop_cycle()
            if rec["estop"].get("ok"):
                rec["after_ack"] = send("route", pts=step["after_pts"])
                t_start, limit = time.time(), 45.0 * len(step["after_pts"]) + 30.0
                pump(5.0, lambda: tick("op") == "GOTO")
            continue
        if tick("op") != "GOTO" or tick("ctl") != 1:
            break
    rec["timeout"] = tick("op") == "GOTO"


def estop_cycle() -> dict:
    out = {"t_estop": round(time.time(), 3)}
    for _ in range(3):  # the app repeats the stop until the FC confirms it
        link.estop(1)
        pump(0.3)
    out["disarmed"] = pump(90.0, lambda: tick("arm") == 0)
    out["t_disarm"] = round(time.time(), 3)
    out["reset_ack"] = send("estop_reset")
    out["control_again"] = wait_control(a.control_timeout)
    out["t_control"] = round(time.time(), 3)
    out["ok"] = bool(out["disarmed"] and out["reset_ack"] and out["reset_ack"].get("ok") and out["control_again"])
    if out["control_again"]:
        send("hold")
        pump(3.0)
    return out


result = {"plan": plan, "steps": [], "t0": T0}
try:
    if a.tag:  # never test the wrong companion (10-22: a leftover one answered on the port)
        pump(120.0, lambda: st["status"] is not None)
        got = ((st["status"] or {}).get("run") or {}).get("tag")
        result["companion_tag"] = got
        if got != a.tag:
            raise TimeoutError(f"wrong companion on the port: tag {got!r}, expected {a.tag!r}")
    result["control"] = wait_control(a.control_timeout)
    result["t_control"] = round(time.time(), 3)
    if result["control"]:
        send("hold")
        pump(4.0)
        for step in plan["steps"]:
            rec = {"tag": step["tag"], "pts": step["pts"], "t0": round(time.time(), 3)}
            if step.get("expect_refuse"):
                ack = send("route", pts=step["pts"])
                rec["ack"], rec["refused_ok"] = ack, ack is not None and not ack.get("ok")
            else:
                if tick("ctl") != 1:
                    rec["skipped_no_control"] = not wait_control(a.control_timeout)
                if not rec.get("skipped_no_control"):
                    fly_route(step["pts"], rec, step)
            rec["t1"] = round(time.time(), 3)
            result["steps"].append(rec)
            send("hold")
            pump(3.0)
        send("land")
        result["landed"] = pump(90.0, lambda: tick("arm") == 0)
except TimeoutError as e:
    result["error"] = str(e)
finally:
    result["t1"] = round(time.time(), 3)
    result["events"] = sorted(st["events"].values())
    result["ticks"] = st["ticks"]
    result["status"] = st["status"]
    Path(a.out).write_text(json.dumps(result))
    print(f"client: {len(result['steps'])} steps, control={result.get('control')}, error={result.get('error')}")
