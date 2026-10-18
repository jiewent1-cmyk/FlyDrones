"""Offline replay test of the companion's emergency-stop path (console_link.ConsoleLink._estop_action).

No sockets, no FC: a fake drone replays an FC mode / armed / COMMAND_ACK timeline while the stop is driven at the
companion's tick rate (20 Hz), with a fake clock. Checks what is sent to the FC and when the stop counts as confirmed.
Case 1 is the 2026-10-18 LAN e2e P9b race: the FC sits in LAND for 0.25 s, then the pilot flips to ALT_HOLD; the old
code only checked the confirmation at its 0.25 s resend cadence, saw ALT_HOLD and forced LAND again.

    python tools/estop_confirm_test.py [companion dir]   (default: ../../orangepi/companion next to this file)

flydrones is stubbed when it is not installed (console_link only imports FlightCommand from it); numpy is needed.
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

    for name in ("flydrones", "flydrones.motor", "flydrones.motor.command"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["flydrones.motor.command"].FlightCommand = FlightCommand
import console_link  # noqa: E402

T0, WALL = 1000.0, 5e8  # fake monotonic start, fake wall-clock offset
clock = {"t": T0}
time.monotonic = lambda: clock["t"]  # console_link reads both through the time module
time.time = lambda: clock["t"] + WALL


class FakeDrone:
    """FC state from a timeline [(t, mode, armed)]; COMMAND_ACKs [(t, command, result)] land in pymavlink's
    per-system message table, where console_link looks for them."""

    fc_sysid = 1

    def __init__(self, timeline, acks=()):
        self.timeline, self.acks, self.sent = timeline, list(acks), []
        self.st = types.SimpleNamespace(messages={})
        mav = types.SimpleNamespace(command_long_send=lambda *a: self.sent.append((self._t(), "cmd", a[2])),
                                    statustext_send=lambda *a: None)
        self.m = types.SimpleNamespace(mav=mav, target_system=1, target_component=1, sysid_state={1: self.st},
                                       set_mode=lambda mode: self.sent.append((self._t(), "mode", mode)))

    @staticmethod
    def _t():
        return round(clock["t"] - T0, 2)

    def _now(self):
        t = clock["t"] - T0 + 1e-9
        for ta, cmd, res in self.acks:
            if ta <= t:
                self.st.messages["COMMAND_ACK"] = types.SimpleNamespace(command=cmd, result=res, _timestamp=T0 + ta + WALL)
        return [row for row in self.timeline if row[0] <= t][-1]

    def fc_mode(self):
        return self._now()[1]

    def armed(self):
        return self._now()[2]


def case(name, timeline, acks=(), level=1, until=1.5, sends=(1, 1), confirm=None):
    clock["t"] = T0
    d = FakeDrone(timeline, acks)
    c = console_link.ConsoleLink.__new__(console_link.ConsoleLink)  # only the state _estop_action uses
    c.lock, c.drone, c.estop, c.estop_done, c._fc_sent, c._fc_wall0, c.fc_actions = threading.Lock(), d, level, False, 0.0, None, 0
    c.events = []
    c.event = lambda src, text: c.events.append((FakeDrone._t(), text))
    c._estop_action()  # as the ESTOP handler: latch, then act at once
    while clock["t"] - T0 < until:
        clock["t"] += 0.05
        c._estop_action()
    got = c.events[0] if c.events else None
    ok = sends[0] <= len(d.sent) <= sends[1] and (confirm is None and got is None or confirm is not None and got and confirm in got[1])
    print(f"{'PASS' if ok else 'FAIL'}  {name}: sent {d.sent}; confirmed {got}")
    return ok


G, L, A = "GUIDED", "LAND", "ALT_HOLD"
results = [
    case("FC in LAND at 0.03 s, pilot flips to ALT_HOLD at 0.25 s (P9b)", [(0, G, True), (0.03, L, True), (0.25, A, True)],
         confirm="(LAND)"),
    case("heartbeat late, our DO_SET_MODE accepted at 0.04 s", [(0, G, True), (0.3, A, True)], acks=[(0.04, 176, 0)],
         confirm="COMMAND_ACK"),
    case("DO_SET_MODE rejected, FC never in LAND: keep resending", [(0, G, True)], acks=[(0.04, 176, 4)], until=1.0,
         sends=(4, 5)),
    case("an ACK from before the press does not count", [(0, G, True)], acks=[(-0.5, 176, 0)], until=0.6, sends=(2, 3)),
    case("kill: force disarm accepted at 0.05 s", [(0, G, True), (0.5, G, False)], acks=[(0.05, 400, 0)], level=2,
         confirm="COMMAND_ACK"),
    case("kill: no ACK, resend until the heartbeat says disarmed", [(0, G, True), (0.5, G, False)], level=2,
         sends=(2, 3), confirm="disarmed"),
    case("already disarmed at the press: nothing sent", [(0, A, False)], sends=(0, 0), confirm="disarmed"),
]
print("ALL PASS" if all(results) else "SOME FAILED")
sys.exit(0 if all(results) else 1)
