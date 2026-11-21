#!/usr/bin/env python3
"""Run the companion's real HeightGuard (companion/height_guard.py, commit b3a6691) against a SITL FC.

It connects to the FC's second MAVLink port as its own client, exactly like the guard's own path on the drone: it
requests the three streams it needs, ticks the guard at 20 Hz and lets it send the source-set switch and LAND.
Nothing else in the run knows about it (the mission script runs with its own built-in monitor off).
"""
import argparse
import json
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0] + "/companion")
from height_guard import HeightGuard  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--url", required=True)
ap.add_argument("--cfg", default="{}")
a = ap.parse_args()
T0 = time.time()


def say(msg):
    print("%8.2f %s" % (time.time() - T0, msg), flush=True)


class Drone:  # the part of RealDrone the guard uses
    mute = False

    def __init__(self, m):
        self.m = m
        self.events = []

    def armed(self):
        return bool(self.m.motors_armed())

    def fc_mode(self):
        return self.m.flightmode

    def _text(self, msg, every_s=5.0):
        now = time.monotonic()
        if now - getattr(self, "_lt", 0.0) >= every_s:
            self._lt = now
            self.m.mav.statustext_send(4, msg[:50].encode())
            say("TEXT " + msg)

    def _gevent(self, what):
        if not self.events or self.events[-1] != what:
            self.events.append(what)
            say("EVENT " + what)


m = None
for _ in range(60):
    try:
        m = mavutil.mavlink_connection(a.url, source_system=254, source_component=191, autoreconnect=False)
        if m.wait_heartbeat(timeout=5) is not None:
            break
    except Exception as e:  # SITL not listening yet
        last = e
        time.sleep(1)
if m is None or m.target_system == 0:
    say("no FC heartbeat")
    sys.exit(2)
say(f"connected, FC sysid {m.target_system}")
mav = mavutil.mavlink


def streams():
    for mid in (mav.MAVLINK_MSG_ID_RANGEFINDER, mav.MAVLINK_MSG_ID_ATTITUDE, mav.MAVLINK_MSG_ID_SCALED_PRESSURE):
        m.mav.command_long_send(m.target_system, m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 50000, 0, 0, 0, 0, 0)
    m.mav.command_long_send(m.target_system, m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 0, 200000, 0, 0, 0, 0, 0)


d = Drone(m)
g = HeightGuard({"enabled": True, **json.loads(a.cfg)})
say("guard cfg " + json.dumps(g.cfg))
next_tick = time.monotonic()
last_req = last_log = 0.0
was_armed = False
while True:
    try:
        x = m.recv_match(blocking=True, timeout=0.01)
    except Exception as e:
        say(f"link closed: {type(e).__name__}")
        break
    now = time.monotonic()
    if now - last_req > 5.0:
        last_req = now
        streams()
    if x is not None and x.get_type() == "COMMAND_ACK" and x.command in (42007, 11, 176):
        say(f"ACK cmd {x.command} result {x.result}")
    if now >= next_tick:
        next_tick += 0.05
        if next_tick < now:
            next_tick = now + 0.05
        g.tick(d)
        arm = d.armed()
        if arm != was_armed:
            was_armed = arm
            say("FC " + ("ARMED" if arm else "disarmed") + f" mode {d.fc_mode()}")
        if now - last_log >= 1.0:
            last_log = now
            r = m.messages.get("RANGEFINDER")
            say("st " + json.dumps({**g.status(), "mode": d.fc_mode(), "rng": round(r.distance, 2) if r else None}))
