"""Offline test of the console's TRACK mode (console_link.Operator / ConsoleLink, 2026-10-18).

No FC, no camera, no brain: a fake pilot with the EcpsPilot surface console_link uses (waypoint layer, target layer
track_start / track_stop / target) records every call and the thread it came from. Console commands run on a thread
named "rx" (as the companion's receive thread), the per-tick hook on one named "control" (the control loop). Checks
what reaches the pilot, in which order and on which thread; the emergency stop goes through the real datagram handler.

    python tools/track_mode_test.py [companion dir]   (default: ../../orangepi/companion next to this file)

flydrones is stubbed when it is not installed (FlightCommand, and GestureState for the illusions); numpy is needed.
"""
import sys
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path

COMP = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2] / "orangepi" / "companion"
sys.path.insert(0, str(COMP))
try:
    import flydrones.motor.command  # noqa: F401
except ImportError:

    @dataclass
    class FlightCommand:
        throttle: float = 0.5
        yaw: float = 0.0
        forward: float = 0.0
        lateral: float = 0.0
        escape: bool = False
        note: str = ""

    for name in ("flydrones", "flydrones.motor", "flydrones.motor.command", "flydrones.senses", "flydrones.senses.gestures"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["flydrones.motor.command"].FlightCommand = FlightCommand
    sys.modules["flydrones.senses.gestures"].GestureState = tuple  # ConsoleGestures keeps the class (illusions only)
import console_link  # noqa: E402


@dataclass
class Det:  # ecps_target.Detection, the fields console_link reads
    marker_id: int
    range_m: float
    bearing_deg: float
    stamp: float
    below_m: float = 0.0


class Target:  # ecps_target.TargetTrack, the state console_link reads
    def __init__(self, source: bool):
        self.state, self.t_state, self.est, self.est_z, self.goal, self.fixes = "OFF", 0.0, None, None, None, 0
        self.standoff, self.search_s, self.det = 1.0, 30.0, None
        self.source = None
        if source:
            self.source = types.SimpleNamespace(latest=lambda: (self.det, 0.0), status="running", frames=50, hits=12,
                                                errors=0, detector=types.SimpleNamespace(marker_id=7, size=0.15))

    @property
    def active(self) -> bool:
        return self.state != "OFF"


class WaypointPilot:
    """EcpsPilot without a target layer (the runtime before 2026-10-18)."""

    def __init__(self):
        self.wp_on, self.wp_active, self.wp_points, self.wp_done_t, self.wp_log, self._t = False, True, [], None, [], 0.0
        self.calls: list[tuple[str, str]] = []
        self.safety = types.SimpleNamespace(land_requested=False, fence=2.0)
        self.brain = types.SimpleNamespace(last_counts=None)
        self.gestures = None

    def _rec(self, name: str) -> None:
        self.calls.append((name, threading.current_thread().name))

    def set_waypoints(self, points) -> None:  # as EcpsPilot.set_waypoints
        self._rec("set_waypoints")
        self.wp_points, self.wp_on, self.wp_idx, self.wp_done_t = list(points), True, 0, None


class TrackPilot(WaypointPilot):
    def __init__(self, source: bool = True, fail_start: bool = False):
        super().__init__()
        self.target = Target(source)
        self.fail_start = fail_start

    def track_start(self, t=None) -> None:  # as EcpsPilot.track_start
        self._rec("track_start")
        if self.fail_start:
            raise RuntimeError("no camera frames")
        self.wp_on = False
        self.target.state = "SEARCH"

    def track_stop(self, t=None) -> None:
        self._rec("track_stop")
        self.target.state = "OFF"
        self.wp_on = False


class Drone:
    def armed(self) -> bool:
        return False


def make(pilot):
    link = console_link.ConsoleLink(0, pilot=pilot, drone=Drone(), cam=None, tofs={}, cfg={"safety": {"geofence_radius_m": 2.0}},
                                    console_cfg={}, run_info={}, log=lambda *a: None)
    link.events_seen = []
    ev = link.event
    link.event = lambda src, text: (link.events_seen.append((src, text)), ev(src, text))
    link.ops.on_event = link.event
    return link


def on(thread: str, fn, *args):
    out = {}
    th = threading.Thread(target=lambda: out.setdefault("r", fn(*args)), name=thread)
    th.start()
    th.join()
    return out.get("r")


CLIENT = console_link.Client(("127.0.0.1", 9))
CLIENT.name = "test"


def rx(link, op, **m):
    return on("rx", link._command, op, m, CLIENT)


def tick(link, inc=True):
    """One control tick: the companion sets wp_active = in control, then the console hook runs on the control thread."""
    link.pilot.wp_active = inc
    on("control", link.ops.on_control, inc)


def names(p, since=0):
    return [c[0] for c in p.calls[since:]]


results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail else ""))


# ---------------------------------------------------------------- 1 TRACK: queued, then started on the control thread
p = TrackPilot()
k = make(p)
tick(k, False)
tick(k)  # hand-over (hand-over mode PATROL)
n0 = len(p.calls)
ok, msg = rx(k, "track")
before = names(p, n0)
tick(k)
check("TRACK: nothing reaches the pilot from the receive thread; the next tick starts the search",
      ok and before == [] and names(p, n0) == ["track_start"] and k.ops.mode == "TRACK" and p.target.state == "SEARCH"
      and p.wp_active, f"ack {ok} {msg!r}, calls {names(p, n0)}, wp_active {p.wp_active}")

# ---------------------------------------------------------------- 2 TRACK again: a fresh search
n0 = len(p.calls)
rx(k, "track")
tick(k)
check("TRACK again: stop, then start (a fresh search)", names(p, n0) == ["track_stop", "track_start"], f"{names(p, n0)}")

# ---------------------------------------------------------------- 3 HOLD ends it
n0 = len(p.calls)
rx(k, "hold")
tick(k)
check("HOLD: the tracker stops once, the waypoint layer is gated off",
      names(p, n0) == ["track_stop"] and p.target.state == "OFF" and not p.wp_active and k.ops.mode == "HOLD", f"{names(p, n0)}")

# ---------------------------------------------------------------- 4 GOTO during TRACK
rx(k, "track")
tick(k)
n0 = len(p.calls)
ok, msg = rx(k, "goto", x=0.5, y=0.2)
tick(k)
seq = names(p, n0)
check("GOTO while tracking: the tracker lets go first, the goal survives it",
      ok and seq == ["track_stop", "set_waypoints"] and p.wp_on and p.wp_points == [(0.5, 0.2)] and p.target.state == "OFF"
      and k.ops.mode == "GOTO" and p.wp_active, f"{seq}, wp_on {p.wp_on}, points {p.wp_points}")
p.wp_done_t, p.wp_log = 3.0, [(0, 0, "reached", 0.12)]
tick(k)
check("GOTO: the waypoint layer reports the goal reached -> HOLD", k.ops.mode == "HOLD", k.ops.mode)

# ---------------------------------------------------------------- 5 a new goal while the old one still reads "done"
rx(k, "goto", x=0.0, y=0.0)
on("control", k.ops._goto_progress)  # a progress check before the queued goal reached the pilot
mode_early = k.ops.mode
tick(k)
check("a new goal is not taken for reached by the previous goal's done flag",
      mode_early == "GOTO" and k.ops.mode == "GOTO" and p.wp_done_t is None and p.wp_points == [(0.0, 0.0)], f"{mode_early} -> {k.ops.mode}")
rx(k, "hold")
tick(k)

# ---------------------------------------------------------------- 6 hand-over in TRACK: a fresh search each time
ok, msg = rx(k, "handover", mode="track")
n0 = len(p.calls)
tick(k, False)  # the pilot takes over
tick(k, True)  # and hands back
first = names(p, n0)
n1 = len(p.calls)
tick(k, False)
tick(k, True)
second = names(p, n1)
check("hand-over mode TRACK: a search starts at every hand-over (stopped first if it ran)",
      ok and k.ops.handover == "TRACK" and first == ["track_start"] and second == ["track_stop", "track_start"]
      and k.ops.mode == "TRACK", f"ack {ok} {msg!r}, {first}, {second}")

# ---------------------------------------------------------------- 7 hand-over in HOLD while tracking
rx(k, "handover", mode="hold")
n0 = len(p.calls)
tick(k, False)
tick(k, True)
check("hand-over mode HOLD while tracking: the tracker stops", names(p, n0) == ["track_stop"] and k.ops.mode == "HOLD"
      and p.target.state == "OFF", f"{names(p, n0)}")

# ---------------------------------------------------------------- 8 emergency stop (the real datagram path)
rx(k, "track")
tick(k)
n0 = len(p.calls)
dg = k.pk.json(console_link.T_ESTOP, {"id": 1, "level": 1, "trig": 1})
on("rx", k._handle, dg[0], CLIENT.addr)
tick(k, False)  # the latch releases control
refused = rx(k, "track")
check("emergency stop in TRACK: LAND, the tracker stops, TRACK refused while latched",
      k.estop == 1 and k.ops.mode == "LAND" and names(p, n0) == ["track_stop"] and refused[0] is False,
      f"es {k.estop}, mode {k.ops.mode}, {names(p, n0)}, track -> {refused}")
rx(k, "estop_reset")
tick(k, True)

# ---------------------------------------------------------------- 9 console link lost in TRACK
rx(k, "track")
tick(k)
n0 = len(p.calls)
k.had_console = True  # a console was connected; no client is live now (the E-stop above came from one)
for c in k.clients.values():
    c.last = 0.0
row, now = {}, time.monotonic()
on("control", k.on_tick, None, row, True, now)
on("control", k.on_tick, None, row, True, now + 0.05)
check("console link lost in TRACK: HOLD, the tracker stops", k.ops.mode == "HOLD" and names(p, n0) == ["track_stop"],
      f"mode {k.ops.mode}, {names(p, n0)}")
k.had_console, k.lost_since = False, None

# ---------------------------------------------------------------- 10 a tracker the console did not start (--track)
rx(k, "patrol")
tick(k)
on("control", p.track_start)  # ecps_companion.py --track at a hand-over
n0 = len(p.calls)
rx(k, "hold")
tick(k)
check("tracker started by --track: an explicit HOLD from the console stops it", names(p, n0) == ["track_stop"], f"{names(p, n0)}")

# ---------------------------------------------------------------- 11 telemetry
p.target.state, p.target.t_state, p._t = "APPROACH", 10.0, 12.0
p.target.est, p.target.est_z, p.target.goal, p.target.fixes = (-1.6, 0.8), 0.7, (-0.71, 0.35), 5
p.target.det = Det(7, 1.85, 152.0, time.monotonic() - 0.2, -0.05)
tj = k._track_json()
p.target.det = Det(7, 1.85, 152.0, time.monotonic() - 1.5)
old = k._track_json()
p.target.state = "OFF"
off = k._track_json()
mk_ok = tj.get("mk", [None])[:3] == [7, 1.85, 152.0] and 0.15 <= tj["mk"][3] <= 0.3 and tj["mk"][4] == -0.05
check("telemetry: trk while the layer runs, mk only for a sighting younger than 1 s",
      tj.get("trk") == {"st": "APPROACH", "est": [-1.6, 0.8, 0.7], "goal": [-0.71, 0.35], "fx": 5, "ts": 2.0} and mk_ok
      and "mk" not in old and "trk" not in off and k._marker_json()["id"] == 7, f"{tj} | {old} | {off}")

# ---------------------------------------------------------------- 12 refusals
q = make(TrackPilot(source=False))
tick(q, False)
tick(q)
r1, r2 = rx(q, "track"), rx(q, "handover", mode="track")
w = make(WaypointPilot())
tick(w, False)
tick(w)
r3, r4 = rx(w, "track"), rx(w, "handover", mode="track")
check("TRACK refused without a marker detector, and in a runtime without a target layer",
      r1[0] is False and "detector" in r1[1] and r2[0] is False and r3[0] is False and "tracking" in r3[1] and r4[0] is False
      and w._marker_json() is None and w._track_json.__self__ is w and not w.ops.trk, f"{r1} {r2} {r3} {r4}")

# ---------------------------------------------------------------- 13 a failing start
f = TrackPilot(fail_start=True)
fk = make(f)
tick(fk, False)
tick(fk)
rx(fk, "track")
tick(fk)
tick(fk)  # the loop goes on
check("track_start raising: reported, TRACK -> HOLD, the control loop keeps ticking",
      fk.ops.mode == "HOLD" and any("failed" in t for _, t in fk.events_seen), f"mode {fk.ops.mode}, {fk.events_seen[-3:]}")

# ---------------------------------------------------------------- 14 threads
threads = {t for pl in (p, f) for _, t in pl.calls}
check("every pilot change ran on the control thread", threads == {"control"}, f"{threads}")
for link in (k, q, w, fk):
    link.sock.close()
print("ALL PASS" if all(results) else "SOME FAILED")
sys.exit(0 if all(results) else 1)
