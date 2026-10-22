#!/usr/bin/env python3
"""End-to-end test of the ground console against the SITL rig on the Orange Pi (tools/sitl_rig.py), from the Mac.

This script plays the pilot (sysid 255: arm in ALT_HOLD with an RC throttle override, climb, switch to GUIDED, take
back with LOITER) and a console (ECL1 link: guidance, link loss, emergency stop), and checks what the companion and the
flight controller do. Needs pymavlink.

    python console_e2e.py --pi 192.168.86.20 --out e2e.json                 # the automated run (P1-P12)
    python console_e2e.py --pi 192.168.86.20 --scenario app --out app.json   # pilot only: the Mac app does the E-stops

Phases
  P1  before the hand-over: hand-over mode HOLD acknowledged, no control
  P2  take-off and hand-over: control taken, HOLD (zero commands, little drift)
  P3  PATROL: the brain flies; the map fills with the cage, ToF frames arrive
  P4  GOTO a point through the waypoint layer: reached -> HOLD
  P5  return to the start point
  P6  MANUAL: forward stick moves the drone forward, releasing it stops it
  P7  console link lost during GOTO: HOLD within the link timeout
  P8  illusion "left" during PATROL (informational)
  P9  emergency LAND on both paths: FC in LAND, companion latched and released; landed and disarmed
  P9b right after the stop the pilot switches to ALT_HOLD: kept for 3 s (nothing forces LAND again), still latched;
      also the FC mode the companion reports in its ticks may not go back to LAND after ALT_HOLD (2026-10-18: a 0.5 s
      LAND forced over the pilot fell between two of the pilot's 1 Hz heartbeats on 10-17)
  P10 latch reset after disarm
  P11 second flight, normal LAND (console: FC mode command + companion "land")
  P12 third flight, KILL: force disarm in the air

Scenario app_full (2026-10-17, LAN test of the deployed runtime): the Mac app is the only console and this script
presses its buttons - it sends what the page sends ({"c": ...} on the app's local WebSocket, token from the app's log
line "UI http://127.0.0.1:<port>/index.html?ws=<port>&t=<token>") and reads what the page gets. Start the app with its
stdout in a file and ECPS_SNAPSHOT=<snap-dir>/ui.png, then
    python console_e2e.py --scenario app_full --ecl 14700 --app-log app.out --app-pid <pid> --snap-dir snaps --out full.json
  A1  flight 1: hand-over in the app's hand-over mode (HOLD)
  A2  patrol from the app: brain, camera, ToF and map reach the app
  A3  go to a point picked in the app, A4 back to the start point
  A5  manual from the app's sticks (as the W key)
  A6  the app freezes (SIGSTOP 4 s) during a go-to: the companion holds; the app resumes (SIGCONT)
  A7  emergency LAND from the app (big red button / Esc), A7b the pilot overrides it (checked as P9b), A8 reset from the app
  A9  flight 2: normal land from the app
  A10 flight 3: kill from the app (after the 1.5 s hold), reset

Scenarios track / app_track (2026-10-18, target tracking): the rig renders an ArUco marker into the SITL camera
(sitl_rig.py --marker, the same --marker here; truth from SIM_STATE). track is an ECL client like the automated run:
  T1  hand-over mode TRACK accepted before take-off; the STATUS shows the marker detector
  T2  take-off and hand-over: TRACK starts a search at once
  T3  search -> approach -> arrived: 0.75-1.35 m from the marker (truth), facing it within 15 deg, inside the fence;
      it must have started more than 1.4 m from the marker (restart the rig between runs: a landed drone stays put)
  T4  HOLD stops the tracker      T5  TRACK again: a fresh search, arrived again
  T6  GOTO while tracking: the tracker lets go, the goal is reached
  T7  console link lost while tracking: HOLD, tracker off
  T8  the pilot takes over (LOITER) while tracking and hands back: a fresh search (hand-over mode TRACK)
  T9  emergency LAND while tracking: tracker off, landed and disarmed; reset, hand-over mode back to HOLD
app_track does T1-T5 and a LAND through the Mac app's page channel, with snapshots of the map (as app_full).
    ssh root@<pi> 'sh /root/ecps_console_test/rig_start.sh 2400 --marker -1.6,0.8,0.7,-27,7'
    python console_e2e.py --scenario track --marker=-1.6,0.8,0.7,-27,7 --out trk.json   (with "=": the value starts with "-")
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import re
import shutil
import signal
import struct
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ecl_client import Link  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--pi", default="192.168.86.20")
ap.add_argument("--mav", type=int, default=14651)
ap.add_argument("--ecl", type=int, default=14701)
ap.add_argument("--key-file", required=True, help="secure copy of the Pi's 32-byte console.key")
ap.add_argument("--scenario", default="auto", choices=["auto", "app", "app_kill", "app_full", "track", "app_track"])
ap.add_argument("--app-log", default="", help="app_full: the app's stdout (with its 'UI http://...' line)")
ap.add_argument("--app-pid", type=int, default=0, help="app_full: the app's PID (SIGSTOP / SIGCONT, SIGUSR1 snapshots)")
ap.add_argument("--snap-dir", default="", help="app_full: the app runs with ECPS_SNAPSHOT=<snap-dir>/ui.png")
ap.add_argument("--marker", default="-1.6,0.8,0.7,-27,7", help="track: the rig's marker n,e,up,yaw,id (sitl_rig.py --marker)")
ap.add_argument("--out", default="console_e2e.json")
a = ap.parse_args()


class AppUI:
    """The Mac app's page channel: commands as its buttons send them, and everything its page receives."""

    def __init__(self, log_path: str):
        from websockets.sync.client import connect

        m = re.search(r"index\.html\?ws=(\d+)&t=([0-9a-f]+)", Path(log_path).read_text(errors="replace"))
        if m is None:
            raise SystemExit(f"no 'UI http://...' line in {log_path}")
        self.ws = connect(f"ws://127.0.0.1:{m.group(1)}/", max_size=None, open_timeout=5)
        self.ws.send(json.dumps({"auth": m.group(2)}))
        self.q: queue.Queue = queue.Queue()
        self.req = 0
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self) -> None:
        try:
            for msg in self.ws:
                self.q.put(msg)
        except Exception as e:  # the app quit or the socket broke: the scenario sees no more ticks
            self.q.put(e)

    def send(self, obj: dict) -> int:
        self.req += 1
        self.ws.send(json.dumps({**obj, "req": self.req}))
        return self.req


g = mavutil.mavlink_connection(f"udpout:{a.pi}:{a.mav}", source_system=255, source_component=190)
L = Link(a.pi, a.ecl, name="console_e2e", cam="off", cam_hz=0, key=Path(a.key_file).read_bytes())
UI = AppUI(a.app_log) if a.scenario in ("app_full", "app_track") else None
TRACKING = a.scenario in ("track", "app_track")
T0 = time.monotonic()
st = {"rc": None, "rc_t": 0.0, "hb_t": 0.0, "hello_t": 0.0, "ecl_on": UI is None, "req_t": 0.0}
obs = {"modes": [], "lpn": [], "ticks": [], "events": {}, "acks": [], "texts": [], "map": [], "tof": 0, "brain": 0, "cam": 0,
       "app_events": [], "app_acks": {}, "app_state": {}, "app_errors": [], "snaps": [], "truth": []}
last = {"tick": None, "status": None, "mode": "?", "armed": False, "hb": 0.0}
rep: dict = {"phases": {}, "scenario": a.scenario}


def now():
    return time.monotonic() - T0


def on_companion(typ: int, body: bytes) -> None:
    """One companion message, from the ECL link or as the app forwards it (inflated, same layout otherwise)."""
    if typ == 1:
        tk = json.loads(body)
        last["tick"] = tk
        obs["ticks"].append((now(), tk))
    elif typ == 8:
        s = json.loads(body)
        last["status"] = s
        for e in s.get("events", []):
            obs["events"].setdefault(e[0], (round(now(), 2), e[2], e[3]))
    elif typ == 7:
        obs["acks"].append((round(now(), 2), json.loads(body)))
    elif typ == 4:  # the header is plain JSON on both paths
        hl = struct.unpack_from("<H", body)[0]
        hdr = json.loads(body[2:2 + hl])
        obs["map"].append((now(), hdr["nvox"], hdr["frames"]))
    elif typ == 5:
        obs["tof"] += 1
    elif typ == 2:
        obs["brain"] += 1
    elif typ == 3:
        obs["cam"] += 1


def on_app(msg) -> None:
    if isinstance(msg, Exception):
        obs["app_errors"].append((round(now(), 2), repr(msg)))
    elif isinstance(msg, bytes):
        if len(msg) >= 2 and msg[0] == 1:
            on_companion(msg[1], msg[2:])
    else:
        o = json.loads(msg)
        k = o.get("k")
        if k == "state":
            obs["app_state"] = o
        elif k == "ack":
            obs["app_acks"][o.get("req")] = (round(now(), 2), o)
        elif k == "event":
            obs["app_events"].append((round(now(), 2), o.get("level"), o.get("text")))


def pump(sec: float) -> None:
    end = time.monotonic() + sec
    while time.monotonic() < end:
        t = time.monotonic()
        if t - st["rc_t"] > 0.1:
            ch = (0, 0, 0, 0) if st["rc"] is None else (1500, 1500, st["rc"], 1500)
            g.mav.rc_channels_override_send(1, 1, *ch, 0, 0, 0, 0)
            st["rc_t"] = t
        if t - st["hb_t"] > 1.0:
            g.mav.heartbeat_send(6, 8, 0, 0, 0)
            st["hb_t"] = t
        if t - st["req_t"] > 5.0:  # display streams for this test (LOCAL_POSITION_NED, EKF), as the e2e pilot's own view
            for mid, hz in ((32, 10), (193, 2)) + (((108, 10),) if TRACKING else ()):  # + SIM_STATE (truth)
                g.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
            st["req_t"] = t
        if st["ecl_on"] and t - st["hello_t"] > 0.2:
            L.hello_tick()
            if int(t * 2) != int(st["hello_t"] * 2):
                L.ping()
            st["hello_t"] = t
        m = g.recv_match(blocking=False)
        while m is not None:
            k, src = m.get_type(), m.get_srcSystem()
            if src == 1 and k == "HEARTBEAT":
                mode, armed = mavutil.mode_string_v10(m), bool(m.base_mode & 128)
                last.update(mode=mode, armed=armed, hb=t)
                if not obs["modes"] or obs["modes"][-1][1:] != (mode, armed):
                    obs["modes"].append((round(now(), 2), mode, armed))
            elif src == 1 and k == "LOCAL_POSITION_NED":
                obs["lpn"].append((now(), m.x, m.y, -m.z, m.vx, m.vy))
            elif src == 1 and k == "SIM_STATE":
                obs["truth"].append((now(), m.lat_int, m.lon_int, m.yaw))
            elif k == "STATUSTEXT":
                obs["texts"].append((round(now(), 2), src, m.text))
            m = g.recv_match(blocking=False)
        r = L.recv() if st["ecl_on"] else None
        while r is not None:
            typ, body, _ = r
            on_companion(typ, body)
            r = L.recv()
        while UI is not None:
            try:
                on_app(UI.q.get_nowait())
            except queue.Empty:
                break
        time.sleep(0.005)


def tick():
    return last["tick"] or {}


def wait(cond, timeout: float, step: float = 0.1) -> float | None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return time.monotonic() - t0
        pump(step)
    return None


def cmd(op: str, **kw) -> dict | None:
    cid = L.cmd(op, **kw)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5:
        pump(0.05)
        for _, ack in obs["acks"]:
            if ack.get("id") == cid:
                return ack
    return None


def pos():
    return obs["lpn"][-1] if obs["lpn"] else (now(), float("nan"), float("nan"), float("nan"), 0.0, 0.0)


MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "GUIDED": 4, "LOITER": 5, "LAND": 9}


def mode_cmd(name: str, timeout: float = 6.0) -> bool:
    """DO_SET_MODE, re-sent every 0.5 s until the FC's heartbeat reports the mode (one datagram can be lost on Wi-Fi)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        g.mav.command_long_send(1, 1, 176, 0, 1, MODES[name], 0, 0, 0, 0, 0)
        if wait(lambda: last["mode"] == name, 0.5, 0.05) is not None:
            return True
    return False


def takeoff_handover(tag: str) -> dict:
    """The pilot: ALT_HOLD, arm, climb to ~0.6 m, hold the stick at mid, GUIDED."""
    out = {}
    mode_cmd("ALT_HOLD")
    st["rc"] = 1000
    pump(1.0)
    for _ in range(12):
        if last["armed"]:
            break
        g.mav.command_long_send(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        pump(1.5)
    out["armed"] = last["armed"]
    st["rc"] = 1650
    t = wait(lambda: pos()[3] > 0.6, 25)
    st["rc"] = 1500
    pump(1.0)
    out["climb_s"] = None if t is None else round(t, 1)
    out["alt_handover"] = round(pos()[3], 2)
    mode_cmd("GUIDED")
    out["ctl_after_s"] = wait(lambda: tick().get("ctl") == 1, 8)
    out["op_at_handover"] = tick().get("op")
    print(f"[{now():6.1f}] {tag}: {out}", flush=True)
    return out


def land_and_disarm(timeout: float = 40) -> float | None:
    return wait(lambda: not last["armed"], timeout, 0.2)


def path_len(t0: float, t1: float) -> float:
    p = [(x, y) for t, x, y, *_ in obs["lpn"] if t0 <= t <= t1]
    return sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(p, p[1:]))


def fc_seen(t0: float, t1: float) -> list:
    """The FC mode as the companion reports it in its ticks (5-20 Hz) between t0 and t1, changes only: finer than the
    pilot's 1 Hz heartbeats (on 2026-10-17 a 0.5 s LAND forced over the pilot's ALT_HOLD fell between two of them)."""
    seq = []
    for t, tk in obs["ticks"]:
        m = tk.get("fc")
        if t0 <= t <= t1 and m and (not seq or seq[-1][1] != m):
            seq.append((round(t, 2), m))
    return seq


def relanded(seq: list) -> bool:
    """LAND again after the pilot's ALT_HOLD showed up."""
    seen = False
    for _, m in seq:
        seen = seen or m == "ALT_HOLD"
        if seen and m == "LAND":
            return True
    return False


def fc_actions():
    """The companion's E-stop commands to the FC so far (STATUS, 0.5-2 Hz, so informational)."""
    return ((last["status"] or {}).get("counters") or {}).get("estop_fc_actions")


def body_vx() -> float:
    _, _, _, _, vx, vy = pos()
    yaw = math.radians(tick().get("tel", [0, 0, 0, 0])[3] or 0.0)
    return vx * math.cos(yaw) + vy * math.sin(yaw)


def phase(name: str, ok: bool, **kw) -> None:
    rep["phases"][name] = {"pass": bool(ok), **kw}
    print(f"[{now():6.1f}] {name}: {'PASS' if ok else 'FAIL'} {kw}", flush=True)


def app_cmd(op: str, **kw) -> dict | None:
    """A guidance button in the app: the app's ack comes once the companion acknowledged (or the app gave up, ~2.4 s)."""
    req = UI.send({"c": "cmd", "op": op, **kw})
    wait(lambda: req in obs["app_acks"], 4.0, 0.05)
    got = obs["app_acks"].get(req)
    return None if got is None else {"ok": got[1].get("ok"), "msg": got[1].get("msg"), "after_s": got[0]}


def app_press(c: str, **kw) -> float:
    """A safety button in the app (estop / land / estop_reset): sent at once, the app acks it right away."""
    UI.send({"c": c, **kw})
    return now()


def app_safety() -> dict:
    return obs["app_state"].get("safety", {})


def app_link() -> dict:
    s = obs["app_state"]
    e, f = s.get("ecl", {}), s.get("fc", {})
    return {"ecl_level": e.get("level"), "ecl_auto": e.get("auto"), "ecl_rtt_max": e.get("rtt_max"), "ecl_lost": e.get("lost"),
            "fc_loss_pct": (f.get("link") or {}).get("loss_pct"), "fc_hb_age": f.get("hb_age"), "app_lag_ms": s.get("app", {}).get("lag_ms")}


def snap(name: str) -> None:
    """The app's page as a PNG (SIGUSR1 -> ECPS_SNAPSHOT), kept as <snap-dir>/<name>.png."""
    if not (a.app_pid and a.snap_dir):
        return
    f = Path(a.snap_dir) / "ui.png"
    t0 = f.stat().st_mtime if f.exists() else 0
    os.kill(a.app_pid, signal.SIGUSR1)
    if wait(lambda: f.exists() and f.stat().st_mtime > t0, 4.0, 0.1) is not None:
        pump(0.3)  # the write may still be going on
        shutil.copy(f, Path(a.snap_dir) / f"{name}.png")
        obs["snaps"].append(name)


def app_full() -> None:
    """The Mac app is the only console (no ECL link of our own): every console action goes through the app."""
    # ---------------------------------------------------------------- A1 flight 1: hand-over in the app's mode (HOLD)
    ho = takeoff_handover("app flight 1")
    t0 = now()
    pump(4.0)
    win = [tk for t, tk in obs["ticks"] if t >= t0]
    sent = [abs(tk["cmd"][0]) + abs(tk["cmd"][2]) for tk in win if tk.get("ctl")]
    phase("A1_handover_hold", ho["ctl_after_s"] is not None and ho["op_at_handover"] == "HOLD" and max(sent or [9]) < 0.05,
          **ho, max_cmd=round(max(sent or [9]), 3), drift_m=round(path_len(t0, now()), 2), link=app_link())
    snap("1_hold")
    # ---------------------------------------------------------------- A2 patrol: brain, camera, ToF and map in the app
    ack = app_cmd("patrol")
    t0 = now()
    nv0 = obs["map"][-1][1] if obs["map"] else 0
    tof0, br0, cam0 = obs["tof"], obs["brain"], obs["cam"]
    pump(15.0)
    snap("2_patrol")
    nv1 = obs["map"][-1][1] if obs["map"] else 0
    plen = path_len(t0, now())
    got = {"tof": obs["tof"] - tof0, "brain": obs["brain"] - br0, "cam": obs["cam"] - cam0}
    phase("A2_patrol", bool(ack and ack["ok"]) and plen > 0.4 and nv1 > 100 and min(got.values()) > 5, ack=ack,
          path_m=round(plen, 2), map_vox=[nv0, nv1], msgs_15s=got, link=app_link())
    # ---------------------------------------------------------------- A3 go to a picked point, A4 back to the start
    for name, goal in (("A3_goto", (1.0, 0.6)), ("A4_home", (0.0, 0.0))):
        ack = app_cmd("goto", x=goal[0], y=goal[1])
        t0 = now()
        wait(lambda: tick().get("opm") == "GOTO", 2)
        t = wait(lambda: tick().get("opm") == "HOLD", 60)
        d = [math.hypot(x - goal[0], y - goal[1]) for tt, x, y, *_ in obs["lpn"] if tt >= t0]
        phase(name, bool(ack and ack["ok"]) and t is not None and min(d or [9]) < 0.45, ack=ack, to_hold_s=t,
              min_dist_m=round(min(d or [9]), 2))
        if name == "A3_goto":
            snap("3_goto")
    # ---------------------------------------------------------------- A5 manual: the app's stick messages (W key)
    ack = app_cmd("manual", on=True)
    pump(1.0)
    vx_before = body_vx()
    t0 = time.monotonic()
    vmax = 0.0
    while time.monotonic() - t0 < 2.5:
        UI.send({"c": "stick", "f": 1.0, "l": 0.0, "y": 0.0, "z": 0.0})
        pump(0.1)
        vmax = max(vmax, body_vx())
    stop_t = wait(lambda: abs(body_vx()) < 0.05, 4)  # key released: the app stops sending, the companion's stick timeout
    phase("A5_manual", bool(ack and ack["ok"]) and vmax > 0.08 and stop_t is not None, ack=ack, vx_before=round(vx_before, 3),
          vx_max=round(vmax, 3), stopped_after_s=stop_t)
    app_cmd("hold")
    # ---------------------------------------------------------------- A6 the app freezes during a go-to
    ack = app_cmd("goto", x=-1.0, y=-0.5)
    pump(2.0)
    t_lost = now()
    os.kill(a.app_pid, signal.SIGSTOP)
    try:
        pump(4.0)  # nothing from the app (its page channel is frozen too); the pilot's MAVLink keeps going
        fc_mode_frozen = last["mode"]
        drift = path_len(t_lost + 1.6, now())  # after the companion's 1.5 s link timeout
    finally:
        os.kill(a.app_pid, signal.SIGCONT)
    pump(3.0)
    ev = [e for k, e in sorted(obs["events"].items()) if e[0] >= t_lost - 0.5]
    lost_hold = any("console link lost" in e[2] for e in ev)
    phase("A6_app_frozen", tick().get("opm") == "HOLD" and lost_hold and fc_mode_frozen == "GUIDED", ack=ack,
          opm=tick().get("opm"), fc_mode_while_frozen=fc_mode_frozen, drift_after_timeout_m=round(drift, 2),
          events=[e[2] for e in ev][:6], link_after=app_link())
    # ---------------------------------------------------------------- A7 emergency LAND from the app
    pump(1.0)
    t_press = app_press("estop", level=1)
    t_land = wait(lambda: last["mode"] == "LAND", 6, 0.02)  # the pilot's view: the FC heartbeat (1 Hz)
    t_latch = wait(lambda: tick().get("es", 0) >= 1 and tick().get("ctl") == 0, 3, 0.02)
    wait(lambda: (app_safety().get("land") or {}).get("confirmed"), 3)
    land_app = app_safety().get("land")
    st["rc"] = 1500  # the pilot overrides the stop with the mode switch, in the air: LAND puts it down in ~1.2 s
    t_sw, fa0 = now(), fc_actions()
    ovr = mode_cmd("ALT_HOLD", 3.0)
    t_o = time.monotonic()
    kept = True
    while time.monotonic() - t_o < 3.0:
        pump(0.1)
        kept = kept and last["mode"] == "ALT_HOLD" and last["armed"]
    latched = tick().get("es", 0) >= 1 and tick().get("ctl") == 0
    seen, fa1 = fc_seen(t_sw, now()), fc_actions()
    snap("4_estop_land")  # after the override (a snapshot can take seconds; before it, 10-18 run 2 overrode on the ground)
    mode_cmd("LAND")
    dis = land_and_disarm(45)
    phase("A7_app_emergency_land", t_land is not None and t_latch is not None and bool(land_app and land_app.get("confirmed"))
          and dis is not None, fc_land_seen_by_pilot_s=t_land, app_confirmed_after_s=(land_app or {}).get("after"), companion_latched_s=t_latch, disarmed_after_s=dis,
          app_events=[e[2] for e in obs["app_events"] if e[0] >= t_press - 0.1][:6])
    phase("A7b_pilot_override", ovr and kept and latched and any(m == "ALT_HOLD" for _, m in seen) and not relanded(seen),
          override_after_s=round(t_sw - t_press, 2), override_accepted=ovr, alt_hold_kept_3s=kept, companion_still_latched=latched, fc_mode_seen_by_companion=seen[:10],
          companion_fc_actions=[fa0, fa1])
    st["rc"] = None
    pump(2.0)
    app_press("estop_reset")
    t = wait(lambda: tick().get("es") == 0, 4)
    phase("A8_reset", t is not None, after_s=t, es=tick().get("es"))
    # ---------------------------------------------------------------- A9 flight 2: normal land from the app
    ho = takeoff_handover("app flight 2")
    pump(2.0)
    t_press = app_press("land")
    t = wait(lambda: last["mode"] == "LAND", 6, 0.02)
    dis = land_and_disarm(45)
    phase("A9_app_land", ho["ctl_after_s"] is not None and t is not None and dis is not None, handover=ho, fc_land_seen_s=t,
          disarmed_after_s=dis, app_events=[e[2] for e in obs["app_events"] if e[0] >= t_press - 0.1][:4])
    st["rc"] = None
    pump(3.0)
    # ---------------------------------------------------------------- A10 flight 3: kill from the app
    ho = takeoff_handover("app flight 3")
    pump(2.0)
    alt_k = pos()[3]
    t_press = app_press("estop", level=2)
    t_dis = wait(lambda: not last["armed"], 5, 0.02)
    pump(2.0)
    kill_app = app_safety().get("kill")
    snap("5_kill")
    phase("A10_app_kill", t_dis is not None and tick().get("es") == 2 and bool(kill_app and kill_app.get("confirmed")),
          handover=ho, alt_at_kill=round(alt_k, 2), disarmed_seen_by_pilot_s=t_dis, app_confirmed_after_s=(kill_app or {}).get("after"),
          companion_es=tick().get("es"), app_events=[e[2] for e in obs["app_events"] if e[0] >= t_press - 0.1][:4])
    st["rc"] = None
    app_press("estop_reset")
    pump(3.0)


MK = [float(v) for v in a.marker.split(",")]
LAT0, LON0 = 33.6430, -117.8420  # SITL home = the EKF origin (sitl_rig.py, companion.yaml mavlink.ekf_origin)


def truth(i: int = -1):
    """Simulator truth (SIM_STATE): (t, north, east, yaw deg) from the take-off point."""
    if not obs["truth"]:
        return None
    t, la, lo, yaw = obs["truth"][i]
    k = 111319.49e-7
    return t, (la - LAT0 * 1e7) * k, (lo - LON0 * 1e7) * k * math.cos(math.radians(LAT0)), math.degrees(yaw)


def at_marker() -> dict:
    """Where the drone is (truth) relative to the marker: distance, facing error; the marker estimate's error."""
    tr = truth()
    if tr is None:
        return {"in_place": False, "why": "no SIM_STATE"}
    _, n, e, yaw = tr
    d = math.hypot(MK[0] - n, MK[1] - e)
    face = abs((math.degrees(math.atan2(MK[1] - e, MK[0] - n)) - yaw + 180) % 360 - 180)
    est = (tick().get("trk") or {}).get("est")
    out = {"dist_m": round(d, 2), "facing_err_deg": round(face, 1), "truth_ne": [round(n, 2), round(e, 2)],
           "est_err_m": None if not est else round(math.hypot(est[0] - MK[0], est[1] - MK[1]), 2), "mk": tick().get("mk")}
    out["in_place"] = 0.75 <= d <= 1.35 and face < 15
    return out


def r_max(t0: float) -> float:
    pts = [truth(i) for i in range(len(obs["truth"])) if obs["truth"][i][0] >= t0]
    return round(max((math.hypot(p[1], p[2]) for p in pts), default=0.0), 2)


def trk() -> dict:
    return tick().get("trk") or {}


def trk_states(t0: float) -> list:
    seq = []
    for t, tk in obs["ticks"]:
        s_ = (tk.get("trk") or {}).get("st", "OFF")
        if t >= t0 and (not seq or seq[-1][1] != s_):
            seq.append((round(t, 1), s_))
    return seq


def target_events(t0: float) -> list:
    return [e[2] for e in sorted(obs["events"].values()) if e[0] >= t0 and e[2].startswith("target")]


def track_auto() -> None:
    """TRACK through the ECL link (as the automated run): see T1-T9 in the docstring."""
    # ---------------------------------------------------------------- T1 hand-over mode TRACK before take-off
    ack = cmd("handover", mode="track")
    pump(1.0)
    mst = (last["status"] or {}).get("marker") or {}
    phase("T1_handover_track", bool(ack and ack.get("ok")) and tick().get("ho") == "TRACK" and bool(mst.get("detector")),
          ack=ack, ho=tick().get("ho"), marker=mst)
    # ---------------------------------------------------------------- T2 take-off: the search starts at the hand-over
    ho = takeoff_handover("track flight 1")
    t_ho = now()
    tr0 = truth()
    start = None if tr0 is None else {"ne": [round(tr0[1], 2), round(tr0[2], 2)], "yaw": round(tr0[3]),
                                      "to_marker_m": round(math.hypot(MK[0] - tr0[1], MK[1] - tr0[2]), 2)}
    t_s = wait(lambda: trk().get("st") in ("SEARCH", "APPROACH", "ARRIVED"), 3)
    phase("T2_handover_starts_search", ho["ctl_after_s"] is not None and ho["op_at_handover"] == "TRACK" and t_s is not None,
          **ho, start=start, trk=trk())
    # ---------------------------------------------------------------- T3 arrived: in front of the marker, facing it
    t_arr = wait(lambda: trk().get("st") == "ARRIVED", 120, 0.2)
    pump(5.0)
    res = at_marker()
    rm = r_max(t_ho)
    far = start is not None and start["to_marker_m"] > 1.4  # it had to fly there (a fresh rig starts at home, nose north)
    phase("T3_track_arrived", t_arr is not None and res["in_place"] and rm < 2.0 and far, arrived_after_s=None if t_arr is None else round(t_arr, 1),
          **res, r_max_m=rm, started_far=far, states=trk_states(t_ho)[:8], events=target_events(t_ho)[:8])
    # ---------------------------------------------------------------- T4 HOLD stops the tracker
    ack = cmd("hold")
    t0 = now()
    off = wait(lambda: "trk" not in tick() and tick().get("opm") == "HOLD", 2)
    pump(3.0)
    phase("T4_hold_stops_tracking", bool(ack and ack["ok"]) and off is not None, ack=ack, after_s=off,
          drift_m=round(path_len(t0, now()), 2))
    # ---------------------------------------------------------------- T5 TRACK again: a fresh search
    ack = cmd("track")
    t0 = now()
    t_arr = wait(lambda: trk().get("st") == "ARRIVED", 90, 0.2)
    pump(3.0)
    res = at_marker()
    ev = target_events(t0)
    phase("T5_track_again", bool(ack and ack["ok"]) and t_arr is not None and res["in_place"] and any(x.startswith("target: search") for x in ev),
          ack=ack, arrived_after_s=None if t_arr is None else round(t_arr, 1), **res, events=ev[:6])
    # ---------------------------------------------------------------- T6 GOTO while tracking
    ack = cmd("goto", x=0.0, y=0.0)
    t0 = now()
    off = wait(lambda: "trk" not in tick() and tick().get("opm") == "GOTO", 2)
    t = wait(lambda: tick().get("opm") == "HOLD", 60)
    dd = [math.hypot(x, y) for tt, x, y, *_ in obs["lpn"] if tt >= t0]
    phase("T6_goto_while_tracking", bool(ack and ack["ok"]) and off is not None and t is not None and min(dd or [9]) < 0.45, ack=ack,
          tracker_off_after_s=off, to_hold_s=t, min_dist_m=round(min(dd or [9]), 2))
    # ---------------------------------------------------------------- T7 console link lost while tracking
    cmd("track")
    pump(3.0)
    st["ecl_on"] = False
    t_lost = now()
    pump(4.0)
    st["ecl_on"] = True
    pump(2.5)
    ev = [e[2] for k_, e in sorted(obs["events"].items()) if e[0] >= t_lost - 0.5]
    phase("T7_link_lost_while_tracking", tick().get("opm") == "HOLD" and "trk" not in tick() and any("console link lost" in x for x in ev),
          opm=tick().get("opm"), trk=trk(), events=ev[:8])
    # ---------------------------------------------------------------- T8 the pilot takes over and hands back
    cmd("track")
    pump(4.0)
    lo = mode_cmd("LOITER")
    rel = wait(lambda: tick().get("ctl") == 0, 3)
    pump(3.0)
    t0 = now()
    mode_cmd("GUIDED")
    back = wait(lambda: tick().get("ctl") == 1, 8)
    fresh = wait(lambda: any(x.startswith("target: search") for x in target_events(t0)), 4)
    phase("T8_pilot_takeover_and_back", lo and rel is not None and back is not None and tick().get("opm") == "TRACK" and fresh is not None,
          loiter=lo, released_s=rel, back_s=back, opm=tick().get("opm"), events=target_events(t0 - 4)[:6])
    pump(3.0)
    # ---------------------------------------------------------------- T9 emergency LAND while tracking
    t_es = time.monotonic()
    own = g.mav.srcSystem
    t_land = t_latch = None
    while time.monotonic() - t_es < 6:
        L.estop(1)
        g.mav.srcSystem = 250
        g.mav.command_long_send(1, 1, 176, 0, 1, 9, 0, 0, 0, 0, 0)
        g.mav.srcSystem = own
        pump(0.2)
        if t_land is None and last["mode"] == "LAND":
            t_land = time.monotonic() - t_es
        if t_latch is None and tick().get("es", 0) >= 1 and tick().get("ctl") == 0:
            t_latch = time.monotonic() - t_es
        if t_land is not None and t_latch is not None:
            break
    off = wait(lambda: "trk" not in tick(), 2)
    dis = land_and_disarm(45)
    phase("T9_estop_while_tracking", t_land is not None and t_latch is not None and off is not None and dis is not None,
          fc_land_s=t_land, companion_latched_s=t_latch, tracker_off_s=off, opm=tick().get("opm"), disarmed_after_s=dis)
    st["rc"] = None
    pump(2.0)
    cmd("estop_reset")
    cmd("handover", mode="hold")
    pump(1.0)


def app_track() -> None:
    """TRACK through the Mac app: hand-over mode and buttons as the page sends them, snapshots of the map."""
    ack = app_cmd("handover", mode="track")
    pump(1.0)
    phase("AT1_app_handover_track", bool(ack and ack["ok"]) and tick().get("ho") == "TRACK", ack=ack, ho=tick().get("ho"))
    ho = takeoff_handover("app track flight")
    t_ho = now()
    tr0 = truth()
    d0 = None if tr0 is None else round(math.hypot(MK[0] - tr0[1], MK[1] - tr0[2]), 2)
    wait(lambda: trk().get("st") == "APPROACH", 60, 0.2)
    pump(1.5)
    snap("t0_approach")
    t_arr = wait(lambda: trk().get("st") == "ARRIVED", 120, 0.2)
    pump(5.0)
    res = at_marker()
    snap("t1_arrived")
    phase("AT2_app_track_arrived", ho["ctl_after_s"] is not None and t_arr is not None and res["in_place"] and (d0 or 0) > 1.4, handover=ho,
          start_to_marker_m=d0,
          arrived_after_s=None if t_arr is None else round(t_arr, 1), **res, states=trk_states(t_ho)[:8], link=app_link())
    ack = app_cmd("hold")
    off = wait(lambda: "trk" not in tick() and tick().get("opm") == "HOLD", 2)
    phase("AT3_app_hold", bool(ack and ack["ok"]) and off is not None, ack=ack, after_s=off)
    ack = app_cmd("track")
    t_arr = wait(lambda: trk().get("st") == "ARRIVED", 90, 0.2)
    pump(3.0)
    res = at_marker()
    snap("t2_track_again")
    phase("AT4_app_track_again", bool(ack and ack["ok"]) and t_arr is not None and res["in_place"], ack=ack,
          arrived_after_s=None if t_arr is None else round(t_arr, 1), **res)
    app_press("land")
    t = wait(lambda: last["mode"] == "LAND", 6, 0.02)
    off = wait(lambda: "trk" not in tick(), 2)
    dis = land_and_disarm(45)
    phase("AT5_app_land", t is not None and off is not None and dis is not None, fc_land_s=t, tracker_off_s=off, disarmed_after_s=dis)
    st["rc"] = None
    pump(2.0)
    app_cmd("handover", mode="hold")
    pump(1.0)


try:
    print(f"[{now():6.1f}] waiting for the FC and the companion...", flush=True)
    ok = wait(lambda: last["hb"] > 0 and tick().get("k", 0) > 0 and len(obs["lpn"]) > 5, 150, 0.5)
    if ok is None:
        raise SystemExit(f"no FC / companion / position: hb {last['hb']} tick {bool(last['tick'])} lpn {len(obs['lpn'])}")
    pump(8.0)  # EKF settles on flow + route B
    if a.scenario in ("app_full", "app_track", "track"):
        {"app_full": app_full, "app_track": app_track, "track": track_auto}[a.scenario]()
        raise SystemExit(0)
    if a.scenario in ("app", "app_kill"):
        # pilot only: the Mac app does the emergency stops; we record what the FC does
        for flight, want in ((1, "LAND"), (2, "KILL")) if a.scenario == "app" else ((1, "KILL"),):
            ho = takeoff_handover(f"app flight {flight}")
            print(f"[{now():6.1f}] waiting for the app's emergency {want} (up to 240 s)...", flush=True)
            t_in = now()
            if want == "LAND":
                t = wait(lambda: last["mode"] == "LAND", 240)
                es = wait(lambda: tick().get("es", 0) >= 1 and tick().get("ctl") == 0, 3)
                dis = land_and_disarm(45)
                phase(f"A{flight}_app_emergency_land", t is not None and es is not None and dis is not None, handover=ho,
                      fc_land_after_wait_s=t, companion_latched_s=es, disarmed_after_s=dis)
                cmd("estop_reset")
                pump(2.0)
            else:
                armed_at = now()
                t = wait(lambda: not last["armed"], 240)
                phase(f"A{flight}_app_kill", t is not None and tick().get("es") == 2, handover=ho, disarmed_after_wait_s=t,
                      companion_es=tick().get("es"),
                      alt_before=round(obs["lpn"][-1][3], 2) if obs["lpn"] else None)
                cmd("estop_reset")
            st["rc"] = None
            pump(3.0)
        raise SystemExit(0)

    # ---------------------------------------------------------------- P1
    ack = cmd("handover", mode="hold")
    pump(0.5)
    phase("P1_before_handover", bool(ack and ack.get("ok")) and tick().get("ho") == "HOLD" and tick().get("ctl") == 0,
          ack=ack, ho=tick().get("ho"), ctl=tick().get("ctl"), fc=last["mode"])
    # ---------------------------------------------------------------- P2
    ho = takeoff_handover("flight 1")
    t0 = now()
    pump(4.0)
    win = [tk for t, tk in obs["ticks"] if t >= t0]
    sent = [abs(tk["cmd"][0]) + abs(tk["cmd"][2]) for tk in win if tk.get("ctl")]
    drift = path_len(t0, now())
    phase("P2_handover_hold", ho["ctl_after_s"] is not None and ho["op_at_handover"] == "HOLD" and max(sent or [9]) < 0.05,
          **ho, max_cmd=round(max(sent or [9]), 3), drift_m=round(drift, 2))
    # ---------------------------------------------------------------- P3
    ack = cmd("patrol")
    t0 = now()
    nv0 = obs["map"][-1][1] if obs["map"] else 0
    tof0, br0 = obs["tof"], obs["brain"]
    pump(15.0)
    win = [tk for t, tk in obs["ticks"] if t >= t0]
    nv1 = obs["map"][-1][1] if obs["map"] else 0
    plen = path_len(t0, now())
    # ToF: a client without "lvl" gets level 2, 4 Hz checked once per companion tick, i.e. 3.3-3.6 Hz = 50-54 frames in
    # 15 s; ">= 45" leaves room for jitter (the old "> 50" failed with exactly 50 on 2026-10-19)
    phase("P3_patrol", bool(ack and ack["ok"]) and plen > 0.4 and nv1 > 100 and obs["tof"] - tof0 >= 45,
          path_m=round(plen, 2), map_vox=[nv0, nv1], tof_msgs=obs["tof"] - tof0, brain_msgs=obs["brain"] - br0,
          op=[tk.get("op") for tk in win[-1:]], saccades=sum(1 for tk in win if tk.get("dec", {}).get("sacc")),
          shield_clamps=sum(1 for tk in win if (tk.get("sh") or [0, 0, 0, 0])[3]))
    # ---------------------------------------------------------------- P4
    goal = (1.0, 0.6)
    ack = cmd("goto", x=goal[0], y=goal[1])
    t0 = now()
    wait(lambda: tick().get("opm") == "GOTO", 2)  # the next TICK carries the new mode
    t = wait(lambda: tick().get("opm") == "HOLD", 60)
    d = [math.hypot(x - goal[0], y - goal[1]) for tt, x, y, *_ in obs["lpn"] if tt >= t0]
    ev = [e for e in obs["events"].values() if e[0] >= t0 - 1]
    phase("P4_goto", bool(ack and ack["ok"]) and t is not None and min(d or [9]) < 0.45, ack=ack, to_hold_s=t,
          min_dist_m=round(min(d or [9]), 2), events=[e[2] for e in ev][-6:])
    # ---------------------------------------------------------------- P5
    ack = cmd("goto", x=0.0, y=0.0)
    t0 = now()
    wait(lambda: tick().get("opm") == "GOTO", 2)
    t = wait(lambda: tick().get("opm") == "HOLD", 60)
    d = [math.hypot(x, y) for tt, x, y, *_ in obs["lpn"] if tt >= t0]
    phase("P5_home", bool(ack and ack["ok"]) and t is not None and min(d or [9]) < 0.45, to_hold_s=t, min_dist_m=round(min(d or [9]), 2))
    # ---------------------------------------------------------------- P6
    ack = cmd("manual", on=True)
    pump(1.0)
    vx_before = body_vx()
    t0 = time.monotonic()
    vmax = 0.0
    while time.monotonic() - t0 < 2.5:
        L.cmd("stick", f=1.0, l=0.0, y=0.0, z=0.0)
        pump(0.1)
        vmax = max(vmax, body_vx())
    t_stop = time.monotonic()
    stop_t = wait(lambda: abs(body_vx()) < 0.05, 4)
    phase("P6_manual", bool(ack and ack["ok"]) and vmax > 0.08 and stop_t is not None, vx_before=round(vx_before, 3),
          vx_max=round(vmax, 3), stopped_after_s=stop_t, op=tick().get("op"))
    cmd("hold")
    # ---------------------------------------------------------------- P7
    ack = cmd("goto", x=-1.0, y=-0.5)
    pump(2.0)
    st["ecl_on"] = False
    t_lost = now()
    pump(4.0)  # MAVLink keeps going (the pilot); the console link is silent
    st["ecl_on"] = True
    pump(2.5)
    ev = [e for k, e in sorted(obs["events"].items()) if e[0] >= t_lost - 0.5]
    lost_hold = any("console link lost" in e[2] for e in ev)
    phase("P7_link_lost", tick().get("opm") == "HOLD" and lost_hold, opm=tick().get("opm"), events=[e[2] for e in ev][:6])
    # ---------------------------------------------------------------- P8 (informational)
    cmd("patrol")
    tb = now()
    pump(3.0)
    base = [tk for t, tk in obs["ticks"] if t >= tb]
    t0 = now()
    for _ in range(15):
        L.cmd("gesture", name="left")
        pump(0.2)
    win = [tk for t, tk in obs["ticks"] if t >= t0]
    cmd("gesture", name="none")
    mean = lambda w: round(sum(tk["braw"][1] for tk in w if not tk["braw"][3]) / max(1, sum(1 for tk in w if not tk["braw"][3])), 3)  # noqa: E731
    phase("P8_illusion_left", True, illusion=sorted({tk.get("illm") for tk in win if tk.get("illm")}),
          brain_yaw_baseline=mean(base), brain_yaw_illusion=mean(win), note="informational (brain behaviour, not a console check)")
    cmd("hold")
    pump(1.0)
    # ---------------------------------------------------------------- P9 emergency LAND, both paths (as the app does)
    t_es = time.monotonic()
    own = g.mav.srcSystem
    t_land = t_latch = None
    while time.monotonic() - t_es < 6:
        L.estop(1)
        g.mav.srcSystem = 250  # the console's MAVLink path (sysid 250)
        g.mav.command_long_send(1, 1, 176, 0, 1, 9, 0, 0, 0, 0, 0)
        g.mav.set_mode_send(1, 1, 9)
        g.mav.srcSystem = own
        pump(0.2)
        if t_land is None and last["mode"] == "LAND":
            t_land = time.monotonic() - t_es
        if t_latch is None and tick().get("es", 0) >= 1 and tick().get("ctl") == 0:
            t_latch = time.monotonic() - t_es
        if t_land is not None and t_latch is not None:
            break
    # the pilot overrides the stop (mode switch to ALT_HOLD): nothing may force LAND / disarm again
    st["rc"] = 1500
    t_sw, fa0 = now(), fc_actions()
    ovr = mode_cmd("ALT_HOLD", 3.0)
    t_o = time.monotonic()
    kept = True
    while time.monotonic() - t_o < 3.0:
        pump(0.1)
        kept = kept and last["mode"] == "ALT_HOLD" and last["armed"]
    latched = tick().get("es", 0) >= 1 and tick().get("ctl") == 0
    seen, fa1 = fc_seen(t_sw, now()), fc_actions()
    mode_cmd("LAND")
    dis = land_and_disarm(45)
    phase("P9_emergency_land", t_land is not None and t_latch is not None and dis is not None, fc_land_s=t_land,
          companion_latched_s=t_latch, disarmed_after_s=dis)
    phase("P9b_pilot_override_after_stop", ovr and kept and latched and any(m == "ALT_HOLD" for _, m in seen) and not relanded(seen),
          override_after_s=round(t_sw - (t_es - T0), 2), override_accepted=ovr, alt_hold_kept_3s=kept, companion_still_latched=latched, fc_mode_seen_by_companion=seen[:10],
          companion_fc_actions=[fa0, fa1])
    st["rc"] = None
    # ---------------------------------------------------------------- P10
    pump(2.0)
    ack = cmd("estop_reset")
    pump(0.5)
    phase("P10_reset", bool(ack and ack["ok"]) and tick().get("es") == 0, ack=ack, es=tick().get("es"))
    # ---------------------------------------------------------------- P11 second flight: normal LAND
    ho = takeoff_handover("flight 2")
    pump(2.0)
    t_l = time.monotonic()
    ack = cmd("land")
    g.mav.srcSystem = 250
    g.mav.command_long_send(1, 1, 176, 0, 1, 9, 0, 0, 0, 0, 0)
    g.mav.srcSystem = own
    t = wait(lambda: last["mode"] == "LAND", 5, 0.05)
    dis = land_and_disarm(45)
    phase("P11_normal_land", ho["ctl_after_s"] is not None and t is not None and dis is not None, handover=ho, ack=ack,
          fc_land_s=t, disarmed_after_s=dis)
    st["rc"] = None
    pump(3.0)
    # ---------------------------------------------------------------- P12 third flight: KILL in the air
    ho = takeoff_handover("flight 3")
    pump(2.0)
    alt_k = pos()[3]
    t_k = time.monotonic()
    t_dis = None
    while time.monotonic() - t_k < 4:
        L.estop(2)
        g.mav.srcSystem = 250
        g.mav.command_long_send(1, 1, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
        g.mav.srcSystem = own
        pump(0.15)
        if not last["armed"]:
            t_dis = time.monotonic() - t_k
            break
    pump(3.0)
    phase("P12_kill", t_dis is not None and tick().get("es") == 2, handover=ho, alt_at_kill=round(alt_k, 2),
          disarmed_s=t_dis, companion_es=tick().get("es"), alt_after=round(pos()[3], 2))
    st["rc"] = None
    cmd("estop_reset")
    pump(2.0)
except Exception as e:  # a crash mid-run is a FAIL, whatever the phases so far say
    rep["error"] = f"{type(e).__name__}: {e}"
    raise
finally:
    rep["modes"] = obs["modes"]
    rep["fc_texts"] = [x for x in obs["texts"] if x[1] == 1][-40:]
    rep["companion_texts"] = [x for x in obs["texts"] if x[1] == 254][-40:]
    rep["events"] = [obs["events"][k] for k in sorted(obs["events"])][-120:]
    rep["ticks"] = len(obs["ticks"])
    rep["tick_hz"] = round(len(obs["ticks"]) / max(1, now()), 1)
    rep["companion_msgs"] = {"brain": obs["brain"], "tof": obs["tof"], "cam": obs["cam"], "map": len(obs["map"])}
    if UI is not None:
        rep["app_events"] = obs["app_events"][-120:]
        rep["app_errors"] = obs["app_errors"]
        rep["app_link_end"] = app_link()
        rep["snaps"] = obs["snaps"]
    rep["pass"] = all(p["pass"] for p in rep["phases"].values()) and "error" not in rep if rep["phases"] else False
    json.dump(rep, open(a.out, "w"), indent=1, default=str)
    print("PASS" if rep["pass"] else "FAIL", "->", a.out, flush=True)
