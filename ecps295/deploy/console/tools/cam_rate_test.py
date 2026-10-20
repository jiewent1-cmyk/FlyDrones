"""Offline test of the console camera rate (console_link.ConsoleLink._tx, 2026-10-20).

The camera frames for a console go out on a fixed schedule of 1/cam_hz (cam_hz capped by the bandwidth level, at most
20 fps at the top level) that the sender thread checks once per control tick (20 Hz). This drives the real _tx with
simulated ticks (start times 50 ms apart +-1 ms, passed in the snapshot as on_tick does): the sender wakes 0.3-4 ms
after each tick, 3 % of the ticks 10-40 ms late, and misses 1 % of them (still busy sending: tick_ev coalesces). It counts the CAM messages per console: the rate must match the setting at
every level, with no bursts after a rate change; the HELLO cap goes through the real datagram handler.

    python tools/cam_rate_test.py [companion dir]   (default: ../../orangepi/companion next to this file)

flydrones is stubbed when it is not installed (FlightCommand, GestureState); numpy is needed.
"""
import collections
import random
import sys
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
    sys.modules["flydrones.senses.gestures"].GestureState = tuple
import console_link  # noqa: E402

TICK = 0.05  # the companion's control loop, 20 Hz


class Pilot:  # the EcpsPilot surface ConsoleLink reads when nothing is commanded
    def __init__(self):
        self.wp_on, self.wp_active, self.wp_points, self.wp_done_t, self.wp_log, self._t = False, True, [], None, [], 0.0
        self.safety = types.SimpleNamespace(land_requested=False, fence=2.0)
        self.brain = types.SimpleNamespace(last_counts=None)
        self.gestures = None


class Drone:
    def armed(self) -> bool:
        return False


def make():
    link = console_link.ConsoleLink(0, pilot=Pilot(), drone=Drone(), cam=None, tofs={}, cfg={}, console_cfg={},
                                    run_info={}, log=lambda *a: None)
    link.cam_sent = []  # (time, client port, wake-up number) per CAM message
    link.wakes = []  # times the sender woke

    def send(addr, dgrams):
        if dgrams and dgrams[0][4] == console_link.T_CAM:  # header <4sB...: the type follows the magic
            link.cam_sent.append((link.now, addr[1], len(link.wakes)))

    link._send = send
    link._tick_json = lambda *a: {}
    link._status_json = lambda live: {}
    link._tof_bin = lambda: b""
    link._outq = lambda: 0
    link._cam_jpeg = lambda c: b"\0" * 3000  # a brain-view frame: 3 datagrams
    return link


def client(port, lvl, hz, cam="brain"):
    c = console_link.Client(("127.0.0.1", port))
    c.lvl, c.cam_hz, c.cam = lvl, hz, cam
    return c


def run(link, clients, seconds, rng, t0=1000.0):
    """Control ticks every 50 ms from t0; returns the time after the last one."""
    n = int(round(seconds / TICK))
    for i in range(n):
        link.k += 1
        t_tick = t0 + i * TICK + rng.uniform(-0.001, 0.001)
        if rng.random() < 0.01:  # the sender was still busy: this tick never woke it
            continue
        now = t_tick + rng.uniform(0.0003, 0.004) + (rng.uniform(0.010, 0.040) if rng.random() < 0.03 else 0.0)
        link.now = now
        link.wakes.append(now)
        for c in clients:
            c.last = now
        link._tx(clients, (None, {}, True, t_tick, None, None, []), now)  # on_tick's snapshot (TICK etc. are stubbed)
    return t0 + n * TICK


def fps(link, port, t_from, t_to):
    return sum(1 for t, p, _ in link.cam_sent if p == port and t_from <= t < t_to) / (t_to - t_from)


def wakes(link, t_from, t_to):
    return sum(1 for t in link.wakes if t_from <= t < t_to)


results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail else ""))


rng = random.Random(20261020)
S = 60.0

# ---------------------------------------------------------------- 1 top level: every setting comes out at its rate
rows = []
for hz in (1, 2, 3, 5, 10, 15, 20):
    link = make()
    run(link, [client(9, 3, hz)], S, rng)
    out = fps(link, 9, 1000.0, 1000.0 + S)
    gaps = [b[0] - a[0] for a, b in zip(link.cam_sent, link.cam_sent[1:])]
    per_wake = max(collections.Counter(w for _, _, w in link.cam_sent).values())
    rows.append((hz, out, len(link.cam_sent), len(link.wakes), per_wake, max(gaps)))
    link.sock.close()
check("top level (3): 1-15 fps come out at the set rate (within 2 %), 20 fps on every tick the sender woke",
      all(abs(o - hz) <= 0.02 * hz for hz, o, _, _, _, _ in rows if hz < 20)
      and all(n == w for hz, _, n, w, _, _ in rows if hz == 20),
      " ".join(f"{hz}->{o:.2f}" for hz, o, _, _, _, _ in rows) + " (20 fps: %d frames, %d wake-ups)" % rows[-1][2:4])
check("at most one frame per wake-up, no gap longer than one period + 3 ticks (missed and late ticks included)",
      all(pw == 1 and hi <= 1.0 / hz + 3 * TICK for hz, _, _, _, pw, hi in rows),
      " ".join(f"{hz}:{hi * 1000:.0f}ms" for hz, _, _, _, _, hi in rows))

# ---------------------------------------------------------------- 2 the level caps 20 fps
caps = {}
for lvl in (2, 1, 0):
    link = make()
    run(link, [client(9, lvl, 20)], S, rng)
    caps[lvl] = fps(link, 9, 1000.0, 1000.0 + S)
    link.sock.close()
check("lower levels cap a 20 fps setting: level 2 -> 3, level 1 -> 1, level 0 -> none",
      abs(caps[2] - 3) <= 0.06 and abs(caps[1] - 1) <= 0.02 and caps[0] == 0, f"{caps}")

# ---------------------------------------------------------------- 3 two consoles, the lowest level applies to both
link = make()
a, b = client(9, 3, 20), client(10, 3, 5)
run(link, [a, b], S, rng)
fa, fb = fps(link, 9, 1000.0, 1000.0 + S), fps(link, 10, 1000.0, 1000.0 + S)
b.lvl = 2  # the second console drops to level 2: both are capped at 3 fps
t1 = run(link, [a, b], S, rng, t0=1000.0 + S)
fa2, fb2 = fps(link, 9, 1000.0 + S, t1), fps(link, 10, 1000.0 + S, t1)
link.sock.close()
check("two consoles keep their own rates; at level 2 both get 3 fps",
      abs(fa - 20) <= 0.4 and abs(fb - 5) <= 0.1 and abs(fa2 - 3) <= 0.06 and abs(fb2 - 3) <= 0.06,
      f"20 fps -> {fa:.2f}, 5 fps -> {fb:.2f}; level 2: {fa2:.2f} / {fb2:.2f}")

# ---------------------------------------------------------------- 4 a level change: no burst, the new rate at once
link = make()
c = client(9, 2, 20)
t1 = run(link, [c], 10.0, rng)
c.lvl = 3
t2 = run(link, [c], 10.0, rng, t0=t1)
after = sum(1 for t, _, _ in link.cam_sent if t >= t1)
link.sock.close()
check("level 2 -> 3 at 20 fps: no burst, a frame on every tick the sender woke from the first one on",
      after == wakes(link, t1, t2), f"{after} frames, {wakes(link, t1, t2)} wake-ups in the 10 s after the change")

# ---------------------------------------------------------------- 5 the HELLO cap (real datagram handler)
link = make()
got = []
for hz in (30, 20, 12.5, -1):
    link._handle(link.pk.json(console_link.T_HELLO, {"name": "test", "cam": "brain", "cam_hz": hz, "lvl": 3})[0],
                 ("127.0.0.1", 9))
    got.append(link.clients[("127.0.0.1", 9)].cam_hz)
link.sock.close()
check("HELLO cam_hz is capped to 0-20 fps", got == [20.0, 20.0, 12.5, 0.0], f"30, 20, 12.5, -1 -> {got}")

print("ALL PASS" if all(results) else "SOME FAILED")
sys.exit(0 if all(results) else 1)
