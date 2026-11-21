"""A second console's emergency stop, over real UDP with the reference client (tools/ecl_client.py).

One console holds the link. A stop from any other address with the right key must act at once, without taking the link
or letting that address do anything else. No Pi, no FC: ConsoleLink's own receive and send paths run on a local socket;
the mode switch and the FC action are recorded instead.

    python tools/estop_second_console_test.py [companion dir]   (default: ../../orangepi/companion next to this file)

flydrones is stubbed when it is not installed (console_link only imports FlightCommand from it); numpy is needed.
"""
import json
import socket
import sys
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
COMP = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE.parents[1] / "orangepi" / "companion"
sys.path[:0] = [str(COMP), str(HERE)]
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

    for name in ("flydrones", "flydrones.motor", "flydrones.motor.command"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["flydrones.motor.command"].FlightCommand = FlightCommand
import console_link  # noqa: E402
from ecl_client import Link  # noqa: E402

KEY, TIMEOUT = bytes(range(32)), 0.6


def start_companion():
    """The real receive and send paths; only what they would do to the FC is recorded."""
    k = console_link.ConsoleLink.__new__(console_link.ConsoleLink)
    k.cfg = {**console_link.DEFAULTS, "link_timeout_s": TIMEOUT}
    k.auth_key, k.clients, k._rx_seq = KEY, {}, {}
    k._foreign_estops = {}
    k.lock, k._tx_lock, k._tx_seq, k.tx_bytes, k.tx_err = threading.Lock(), threading.Lock(), 0, 0, 0
    k.pk, k.rx_n, k.had_console = console_link.Packer(), 0, False
    k.estop, k.estop_t, k.estop_trig, k.estop_done, k._fc_sent, k._fc_wall0 = 0, 0.0, None, False, 0.0, None
    k.modes, k.actions, k.events = [], [], []
    k.ops = types.SimpleNamespace(set_mode=lambda mode, why="", again=False: k.modes.append(mode),
                                  stop_target=lambda: None, _drop_goal=lambda: None)
    k.event = lambda src, text: k.events.append(text)
    k._estop_action = lambda: k.actions.append(k.estop)
    k.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    k.sock.bind(("127.0.0.1", 0))
    k.sock.settimeout(0.05)

    def loop():
        while not k.done:
            try:
                data, addr = k.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                k._handle(data, addr)
            except Exception as e:  # as the real loop: a bad message is logged and the link goes on
                k.events.append(f"bad message: {e}")

    k.done = False
    threading.Thread(target=loop, daemon=True).start()
    return k


def link(k, name, key=KEY):
    return Link("127.0.0.1", k.sock.getsockname()[1], name=name, key=key)


def settle():
    time.sleep(0.1)


def acks(c):
    """ACKs this client receives in the next moment (recv returns None on its 50 ms timeout)."""
    got, until = [], time.monotonic() + 0.2
    while time.monotonic() < until:
        r = c.recv()
        if r and r[0] == console_link.T_ACK:
            got.append(json.loads(r[1]))
    return got


results = []


def check(name, ok, why=""):
    results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {name}{': ' + str(why) if why and not ok else ''}")


k = start_companion()
a, b = link(k, "holder"), link(k, "second")

a.hello_tick()
settle()
holder = next(iter(k.clients.values()), None)
check("the first console holds the link", holder is not None and holder.name == "holder", k.clients)
keep_alive = threading.Event()


def beat():
    while not keep_alive.is_set():
        a.hello_tick()
        time.sleep(0.1)


threading.Thread(target=beat, daemon=True).start()

b.hello_tick()
b.cmd("hold")
settle()
check("a second console's HELLO and commands are dropped",
      list(k.clients) == [holder.addr] and not k.modes and k.estop == 0, (k.clients, k.modes))

wrong = link(k, "wrong key", key=bytes(32))
wrong.estop(1)
settle()
check("a stop with the wrong key is dropped", k.estop == 0 and not k.actions, k.estop)

big = b"x" * (console_link.MAX_PAYLOAD + 10)
b._send(console_link.T_ESTOP, big)
settle()
check("a stop in several datagrams from a second console is dropped", k.estop == 0, k.events)

sent = []


class Tap:
    """The second console's socket, keeping a copy of what it sends."""

    def __init__(self, sock):
        self.sock = sock

    def sendto(self, d, addr):
        sent.append(bytes(d))
        return self.sock.sendto(d, addr)

    def __getattr__(self, name):
        return getattr(self.sock, name)


b.sock = Tap(b.sock)
t0 = time.monotonic()
cid = b.estop(1)
while not k.actions and time.monotonic() - t0 < 1.0:
    time.sleep(0.002)
took = time.monotonic() - t0
check("a second console's stop acts at once", k.estop == 1 and k.modes == ["LAND"] and k.actions, (k.estop, k.modes))
print(f"      {took * 1000:.1f} ms on the loopback")
got = acks(b)
check("the ACK goes to the console that sent the stop", [(m["id"], m["ok"]) for m in got] == [(cid, True)], got)
check("the link stays with the holder", list(k.clients) == [holder.addr] and k.clients[holder.addr] is holder, k.clients)
check("the event names the sender", any("EMERGENCY STOP" in e and "127.0.0.1" in e for e in k.events), k.events)

b.cmd("estop_reset", force=True)
settle()
check("a second console cannot release the stop", k.estop == 1, k.estop)

cid = b.estop(2)
settle()
check("a second console can raise land to kill", k.estop == 2 and [m["id"] for m in acks(b)] == [cid], k.estop)

raw = sent[0]
k.estop, k.modes[:] = 0, []
third = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
third.sendto(raw, k.sock.getsockname())
settle()
check("a captured stop sent again from a new address is dropped", k.estop == 0 and not k.modes, k.estop)
third.close()

cid = a.estop(1)
settle()
check("the holder's own stop still works", k.estop == 1 and k.modes == ["LAND"], (k.estop, k.modes))
got = [m for m in acks(a) if m["id"] == cid]
check("the holder gets its ACK", len(got) == 1 and got[0]["ok"], got)

keep_alive.set()
time.sleep(TIMEOUT + 0.2)
b.hello_tick()
settle()
check("once the holder has been silent past link_timeout_s, the other console takes the link",
      [c.name for c in k.clients.values()] == ["second"], k.clients)

k.done = True
print("ALL PASS" if all(results) else "SOME FAILED")
sys.exit(0 if all(results) else 1)
