"""ECPS295 ground-console link (2026-10-17): the companion streams what the brain sees, does and decides to the Mac
console (deploy/console, "ECPS Console.app") and takes the operator's guidance and the emergency stop back.

Off unless enabled (ecps_companion.py --console PORT, or hw config console.port). With it off nothing below runs and
the companion is unchanged. With it on but no console connected, only the receive thread runs (no telemetry is built).

Protocol ECL1 (UDP; the Pi listens on console.port, default 14700). Every datagram is <= 1216 bytes, so it passes a
1280-byte tunnel MTU (NetBird / WireGuard) without IP fragmentation:
    header <4sBBHHHI: magic 'ECL1', type, flags (bit 0: zlib payload), msg (u16 counter per sender), chunk, nchunk,
    stamp (sender monotonic ms); payload <= 1200 bytes; longer messages are split into chunks of one msg
  Pi -> console
    TICK 1    JSON every control tick (20 Hz): pose, brain raw / operator / sent command, DN + input rates, decoder,
              shield and map-ring state, timing, sensor ages
    BRAIN 2   <IHBBB tick, n, rows, cols, nfeat> + zlib(uint8 spike counts per neuron in this tick, then uint8 eye grids
              [eye L, R][feature][rows*cols] in VISION_FEATURES order, 0..1 -> 0..255)
    CAM 3     <BHHI kind (0 brain view gray, 1 colour), w, h, frame no> + JPEG; chunked
    MAP 4     <H json length> + JSON {res, half, n, nz, nvox, frames, l_occ, l_hi} + zlib(uint32 delta-coded linear
              voxel indices ((i*n + j)*nz + k: i north, j east, k up) + uint8 log-odds (l_occ..l_hi -> 0..255)); 2 Hz
    TOF 5     <B count> + per sensor <hBB bearing, n, age (10 ms units)> + float16[n*n] slant ranges (NaN: none)
    TOPO 6    zlib(JSON brain topology: populations, CSC weights, input / output groups); chunked; on request
    ACK 7     JSON {id, op, ok, msg}
    STATUS 8  JSON 2 Hz: config and limits, event log tail, system (temperatures, clocks, load), counters
    PONG 9    PING payload echoed
  console -> Pi
    HELLO 64  JSON {name, cam: brain|color|off, cam_hz, color_w, color_q, lvl: 0-3}; 5 Hz, also the console heartbeat
    PING 65   8 bytes, echoed in a PONG
    CMD 66    JSON {id, op, ...}: hold | patrol | goto {x, y} | manual {on} | stick {f, l, y, z} (no ACK) | land |
              gesture {name} | handover {mode} | estop_reset {force}
    ESTOP 67  JSON {id, level: 1 land | 2 kill}; the console repeats it until the FC reports LAND / disarmed
    TOPO_REQ 68

Bandwidth (2026-10-17): the Pi Zero 3W's Wi-Fi (AIC8800, SDIO) stalls for seconds once its uplink carries more than
roughly 40 kB/s (PIL and console tests), so everything is sent at a level 0-3 that the console adapts from its round-trip
times (LEVELS; the lowest level of the connected consoles applies), spike counts are summed over the ticks between two
BRAIN messages, and while the socket's send queue holds more than outq_max bytes the camera, map, brain and ToF messages
are skipped (TICK, STATUS, ACK and PONG always go).

Operator layer (Operator.apply wraps pilot.decoder.update): it replaces the brain's raw command BEFORE the safety
governor, so everything downstream still applies: governor limits, ceiling, floor and geofence, the camera / height
/ ToF watchdogs, the fence turn-back, the range shield with the local 3D map and the hand-over gate.
  PATROL  the brain's command, unchanged (what the companion does without a console)
  HOLD    hover; the shield still backs the drone off an obstacle closer than d_safe
  GOTO    heading controller to (x, y) in the map frame (flow-EKF local NED); while the brain escapes (saccade, MDN
          retreat, giant fiber) its command wins; arrival -> HOLD; goto_timeout_s -> HOLD
  MANUAL  the operator's sticks (scaled to the manual limits); zero when older than stick_timeout_s
  LAND    hover command (the FC is in LAND)
At every hand-over (the pilot switches to GUIDED and the companion latches control) the mode is reset to the
console's hand-over choice (default PATROL, i.e. as without a console; the console UI preselects HOLD).
Console link lost (no HELLO for link_timeout_s): GOTO / MANUAL -> HOLD; still lost after lost_land_s while the
companion is in control and holding -> LAND (watchdog path). PATROL continues, as without a console.
Emergency stop: ESTOP land latches; the companion releases control at once (in_control gate) and sends LAND to the FC
itself, rate-limited, until the FC reports LAND (or disarmed) once; the console sends LAND straight through
mavlink-router as well. ESTOP kill forces a disarm (MAV_CMD_COMPONENT_ARM_DISARM 0, 21196) until the FC reports
disarmed once. After that confirmation nothing is re-sent: the pilot can still take over with the mode switch or re-arm
(the RC transmitter keeps the last word), and a new press on the console (ESTOP "trig" changes) acts again.
Heartbeat and route-B height keep going (LAND needs the height). The latch itself keeps the brain from taking control
again until estop_reset (FC disarmed, or force from the console).
"""

from __future__ import annotations

import collections
import json
import math
import os
import socket
import struct
import threading
import time
import zlib
from pathlib import Path

import numpy as np

from flydrones.motor.command import FlightCommand

MAGIC = b"ECL1"
HDR = struct.Struct("<4sBBHHHI")
MAX_PAYLOAD = 1200
F_ZLIB = 1
T_TICK, T_BRAIN, T_CAM, T_MAP, T_TOF, T_TOPO, T_ACK, T_STATUS, T_PONG = range(1, 10)
T_HELLO, T_PING, T_CMD, T_ESTOP, T_TOPO_REQ = 64, 65, 66, 67, 68
VISION_FEATURES = ("ftb", "btf", "up", "down", "loom", "loom_speed", "blank", "near", "ventral")
# per level: TICK / BRAIN every n control ticks (20 Hz / n; 0 = never), ToF / map / status rates (Hz), camera fps cap.
# Measured on the home Wi-Fi (2026-10-17, one console, brain-view camera): level 0 ~4 kB/s, 1 ~10 kB/s, 2 ~25 kB/s,
# 3 ~60-80 kB/s (a good link only: above ~40 kB/s the AIC8800 uplink stalls for seconds).
LEVELS = {
    3: {"tick": 1, "brain": 1, "tof_hz": 10.0, "map_hz": 2.0, "status_hz": 2.0, "cam_max": 10.0},
    2: {"tick": 2, "brain": 4, "tof_hz": 4.0, "map_hz": 0.5, "status_hz": 1.0, "cam_max": 3.0},
    1: {"tick": 4, "brain": 10, "tof_hz": 1.0, "map_hz": 0.25, "status_hz": 0.5, "cam_max": 1.0},
    0: {"tick": 4, "brain": 0, "tof_hz": 0.0, "map_hz": 0.1, "status_hz": 0.5, "cam_max": 0.0},
}
MODES = ("PATROL", "HOLD", "GOTO", "MANUAL", "LAND")
ESTOP_NAMES = {0: "", 1: "land", 2: "kill"}

DEFAULTS = {
    "port": 0,
    "link_timeout_s": 1.5,  # no HELLO for this long: console lost
    "lost_land_s": 20.0,  # console lost and holding this long while in control: LAND (0 = never)
    "stick_timeout_s": 0.5,
    "goto_speed": 0.5,  # forward command (x v_max, then the governor's max_forward)
    "goto_arrive_m": 0.25,
    "goto_slow_m": 0.6,
    "goto_turn_deg": 60.0,  # heading error beyond this: turn in place
    "goto_timeout_s": 90.0,
    "goto_fence_margin_m": 0.4,  # goals farther than geofence - margin from the origin are refused
    "manual_scale": {"f": 0.6, "l": 0.6, "y": 0.6, "z": 0.5},
    "handover": "patrol",  # mode at hand-over until a console chooses
    "status_hz": 2.0,  # before the first tick
    "outq_max": 24000,  # bytes in the socket send queue above which the bulky messages are skipped
}


def _ms() -> int:
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


def _r(v, nd=3):
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return round(f, nd) if math.isfinite(f) else None


class Packer:
    """Message -> datagrams (header + <= MAX_PAYLOAD bytes each)."""

    def __init__(self):
        self.msg = 0

    def datagrams(self, typ: int, payload: bytes, flags: int = 0) -> list[bytes]:
        self.msg = (self.msg + 1) & 0xFFFF
        n = max(1, -(-len(payload) // MAX_PAYLOAD))
        st = _ms()
        return [HDR.pack(MAGIC, typ, flags, self.msg, i, n, st) + payload[i * MAX_PAYLOAD:(i + 1) * MAX_PAYLOAD]
                for i in range(n)]

    def json(self, typ: int, obj) -> list[bytes]:
        b = json.dumps(obj, separators=(",", ":")).encode()
        if len(b) > MAX_PAYLOAD - 100:
            return self.datagrams(typ, zlib.compress(b, 3), F_ZLIB)
        return self.datagrams(typ, b)


class Reassembler:
    """Datagrams from one sender -> complete (type, payload, stamp); incomplete messages are dropped after timeout_s."""

    def __init__(self, timeout_s: float = 1.0):
        self.timeout_s = timeout_s
        self.parts: dict = {}
        self.bad = 0

    def push(self, data: bytes):
        if len(data) < HDR.size:
            self.bad += 1
            return None
        magic, typ, flags, msg, chunk, n, stamp = HDR.unpack_from(data)
        if magic != MAGIC or n == 0 or chunk >= n:
            self.bad += 1
            return None
        body = data[HDR.size:]
        if n > 1:
            now = time.monotonic()
            key = (typ, msg)
            ent = self.parts.get(key)
            if ent is None:
                for k in [k for k, v in self.parts.items() if now - v[0] > self.timeout_s]:
                    del self.parts[k]
                ent = self.parts[key] = [now, {}]
            ent[1][chunk] = body
            if len(ent[1]) < n:
                return None
            body = b"".join(ent[1][i] for i in range(n))
            del self.parts[key]
        if flags & F_ZLIB:
            try:
                body = zlib.decompress(body)
            except zlib.error:
                self.bad += 1
                return None
        return typ, body, stamp


# ---------------------------------------------------------------------------------------------- brain topology
def brain_topology(brain) -> dict:
    """Populations (consecutive neurons of one type and side), CSC weights (column = presynaptic neuron) and the named
    input / output groups, for the console's brain panel (as tools/export_web_brain.py does for the browser demo)."""
    c = brain.connectome
    W = c.weights.tocsc()
    pops, start = [], 0
    for i in range(1, c.n + 1):
        if i == c.n or c.types[i] != c.types[start] or c.sides[i] != c.sides[start]:
            pops.append([str(c.types[start]), str(c.sides[start]), start, i - start])
            start = i
    groups = {}
    for k in list(brain.input_specs) + list(brain.output_specs):
        groups[k] = c.group(k).astype(int).tolist()
    return {
        "name": str(c.name),
        "n": int(c.n),
        "nnz": int(W.nnz),
        "indptr": W.indptr.astype(int).tolist(),
        "indices": W.indices.astype(int).tolist(),
        "data": np.rint(W.data).astype(int).tolist(),
        "pops": pops,
        "groups": groups,
        "inputs": list(brain.input_specs),
        "outputs": list(brain.output_specs),
    }


# ---------------------------------------------------------------------------------------------- illusions
class ConsoleGestures:
    """pilot.gestures backend driven by the console's illusion buttons (flydrones GestureIllusion, as the FlyDrones
    browser demo's gesture dock): palm = sinking illusion (climb), drop = rising illusion (descend), left / right =
    rotation (optomotor turn), fist = still scene, rush = looming (escape). Held buttons are re-sent at 5 Hz; without
    an update for hold_s the illusion ends."""

    PRESET = {
        "palm": (True, 0.95, 0.0, 0.20, "open palm"),
        "fist": (True, 0.10, 0.0, 0.15, "fist"),
        "left": (True, 0.10, -0.85, 0.15, "hand left"),
        "right": (True, 0.10, 0.85, 0.15, "hand right"),
    }

    def __init__(self, hold_s: float = 0.6):
        from flydrones.senses.gestures import GestureState

        self.GS = GestureState
        self.hold_s = hold_s
        self.name = "none"
        self.t_set = 0.0
        self.t_start = 0.0

    def set(self, name: str) -> None:
        now = time.monotonic()
        if name != self.name:
            self.t_start = now
        self.name, self.t_set = name, now

    def active(self) -> bool:
        return self.name != "none" and time.monotonic() - self.t_set < self.hold_s + (1.6 if self.name == "rush" else 0)

    def read(self, t: float, frame=None):
        GS = self.GS
        if not self.active():
            return GS(True, 0.10, 0.0, 0.0, 0.15, "fist")
        el = time.monotonic() - self.t_start
        if self.name == "rush":  # the FlyDrones dock: fist, the hand rushes at the camera, fist again
            return GS(True, 0.10, 0.0, 0.0, 0.6 if 0.1 <= el < 1.4 else 0.15, "hand rushes at camera")
        if self.name == "drop":
            return GS(False, 0.0, 0.0, 1.0, 0.0, "hand dropped")
        p, o, x, s, label = self.PRESET.get(self.name, self.PRESET["fist"])
        return GS(p, o, x, 0.0, s, label)


# ---------------------------------------------------------------------------------------------- operator layer
class Operator:
    """Console modes on top of the brain (see the module docstring). GOTO uses the pilot's own waypoint layer
    (EcpsPilot.set_waypoints, 2026-10-17: the brain keeps flying, a goal bias is added to its yaw after the governor)
    when the runtime has it; otherwise a heading controller replaces the brain's command outside its escapes."""

    def __init__(self, cfg: dict, fence_m: float, pilot, drone):
        self.cfg = cfg
        self.fence_m = fence_m
        self.pilot = pilot
        self.drone = drone
        self.wp = hasattr(pilot, "set_waypoints")  # waypoint layer in EcpsPilot
        self.wp_goal = False  # the console replaced the pilot's mission with its goal
        self.lock = threading.Lock()
        self.mode = "PATROL"
        self.handover = str(cfg.get("handover", "patrol")).upper()
        if self.handover not in ("PATROL", "HOLD"):
            self.handover = "PATROL"
        self.goal: tuple[float, float] | None = None
        self.goal_t0 = 0.0
        self.stick = (0.0, 0.0, 0.0, 0.0)
        self.stick_t = 0.0
        self.inc = False  # companion in control (previous tick)
        self.note = ""
        self.brain_raw = None
        self.out = None
        self.changes = 0
        self.on_event = lambda src, text: None

    def set_mode(self, mode: str, why: str = "") -> None:
        with self.lock:
            if mode == self.mode:
                return
            self.mode = mode
            self.changes += 1
        if mode != "GOTO" and self.wp_goal and mode != "LAND":
            self._drop_goal()
        self.on_event("operator", f"mode {mode}" + (f" ({why})" if why else ""))

    def _drop_goal(self) -> None:
        """Forget the console's goal; a mission the console replaced does not come back (pure brain patrol)."""
        if self.wp_goal:
            self.pilot.wp_on = False
            self.wp_goal = False

    def set_goal(self, x: float, y: float) -> str:
        if not (math.isfinite(x) and math.isfinite(y)):
            return "bad goal"
        r = math.hypot(x, y)
        lim = self.fence_m - float(self.cfg.get("goto_fence_margin_m", 0.4))
        if r > lim:
            return f"goal {r:.2f} m from the origin is outside the geofence ({lim:.2f} m)"
        with self.lock:
            self.goal = (float(x), float(y))
            self.goal_t0 = time.monotonic()
        if self.wp:
            self.pilot.set_waypoints([(float(x), float(y))])
            self.wp_goal = True
        with self.lock:
            self.mode = "PATROL" if self.mode == "GOTO" else self.mode  # a new goal while in GOTO: log it again
        self.set_mode("GOTO", f"goal {x:+.2f}, {y:+.2f}" + (" via the waypoint layer" if self.wp else ""))
        return ""

    def set_stick(self, f: float, l_: float, y: float, z: float) -> None:
        c = lambda v: max(-1.0, min(1.0, float(v)))  # noqa: E731
        with self.lock:
            self.stick = (c(f), c(l_), c(y), c(z))
            self.stick_t = time.monotonic()

    def on_control(self, inc: bool) -> None:
        """Once per tick with the hand-over gate's result: a new hand-over resets the mode to the hand-over choice."""
        if inc and not self.inc:
            with self.lock:
                self.goal = None
            self._drop_goal()
            self.set_mode(self.handover, "hand-over")
        self.inc = inc
        if self.mode == "GOTO":
            self._goto_progress()
        self.gate_waypoints()

    def gate_waypoints(self) -> None:
        """The pilot's waypoint layer only steers in PATROL (a configured mission) and GOTO (the console's goal)."""
        if self.wp and getattr(self.pilot, "wp_active", False) and self.effective() not in ("PATROL", "GOTO"):
            self.pilot.wp_active = False

    def _goto_progress(self) -> None:
        with self.lock:
            t0 = self.goal_t0
        if self.wp and self.wp_goal and getattr(self.pilot, "wp_done_t", None) is not None:
            log = getattr(self.pilot, "wp_log", None) or []
            what = log[-1][2] if log else "done"
            d = log[-1][3] if log else float("nan")
            self.set_mode("HOLD", f"goal {what} (EKF distance {d:.2f} m)")
        elif self.inc and time.monotonic() - t0 > float(self.cfg.get("goto_timeout_s", 90.0)):
            self.set_mode("HOLD", "goto timeout")

    def effective(self) -> str:
        return self.mode if self.inc else self.handover

    def apply(self, raw):
        """The brain's raw command -> the command handed to the safety governor."""
        self.brain_raw = raw
        mode = self.effective()
        out = raw
        if mode in ("HOLD", "LAND"):
            out = FlightCommand.hover(f"console: {mode.lower()}")
        elif mode == "GOTO":
            out = self._goto(raw)
        elif mode == "MANUAL":
            with self.lock:
                f, l_, y, z = self.stick
                age = time.monotonic() - self.stick_t
            if age > float(self.cfg.get("stick_timeout_s", 0.5)):
                out = FlightCommand.hover("console: manual (no stick)")
            else:
                s = self.cfg.get("manual_scale", {})
                out = FlightCommand(throttle=z * s.get("z", 0.5), yaw=y * s.get("y", 0.6), forward=f * s.get("f", 0.6),
                                    lateral=l_ * s.get("l", 0.6), note="console: manual")
        self.gate_waypoints()
        self.out = out
        return out

    def _goto(self, raw):
        with self.lock:
            goal = self.goal
        tel = getattr(self.drone, "_tel", None)
        if goal is None or tel is None or tel.x_m is None or tel.y_m is None or tel.yaw_deg is None:
            self.note = "goto: no pose"
            return FlightCommand.hover("console: goto (no pose)")
        dx, dy = goal[0] - tel.x_m, goal[1] - tel.y_m
        dist = math.hypot(dx, dy)
        err = math.atan2(dy, dx) - math.radians(tel.yaw_deg)
        err = math.atan2(math.sin(err), math.cos(err))
        self.note = f"goto {dist:.2f} m, heading error {math.degrees(err):+.0f} deg"
        if self.wp:  # the pilot's waypoint layer steers the brain's own command toward the goal
            return raw
        # fallback (runtime without EcpsPilot.set_waypoints): heading controller outside the brain's escapes
        if dist < float(self.cfg.get("goto_arrive_m", 0.25)):
            self.set_mode("HOLD", f"arrived ({dist:.2f} m)")
            return FlightCommand.hover("console: arrived")
        if raw.escape:
            self.note = "goto: brain escape"
            return raw
        turn = math.radians(float(self.cfg.get("goto_turn_deg", 60.0)))
        yaw = max(-1.0, min(1.0, err / turn))
        fwd = 0.0
        if abs(err) < turn:
            slow = max(0.3, min(1.0, dist / float(self.cfg.get("goto_slow_m", 0.6))))
            fwd = float(self.cfg.get("goto_speed", 0.5)) * math.cos(err) ** 2 * slow
        return FlightCommand(throttle=0.0, yaw=yaw, forward=fwd, note=f"console: goto {dist:.1f} m")


# ---------------------------------------------------------------------------------------------- link
class Client:
    def __init__(self, addr):
        self.addr = addr
        self.last = 0.0
        self.name = "?"
        self.cam = "brain"
        self.cam_hz = 10.0
        self.color_w = 640
        self.color_q = 60
        self.lvl = 2
        self.rx = Reassembler()
        self.cmds = 0
        self.cam_t = 0.0


class ConsoleLink:
    def __init__(self, port: int, *, pilot, drone, cam, tofs: dict, cfg: dict, console_cfg: dict, run_info: dict,
                 log=print):
        self.cfg = {**DEFAULTS, **(console_cfg or {})}
        self.port = port
        self.pilot, self.drone, self.cam, self.tofs = pilot, drone, cam, tofs
        self.run_info = run_info
        self.log = log
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.pk = Packer()
        self.clients: dict = {}
        self.had_console = False
        self.estop = 0
        self.estop_t = 0.0
        self.estop_trig = None  # the console's press number: a new press acts again after a confirmed one
        self.estop_done = False  # the FC confirmed (LAND / disarmed): stop forcing, the pilot may override
        self._fc_sent = 0.0
        self.fc_actions = 0
        self.events = collections.deque(maxlen=300)
        self.ev_id = 0
        self._seen = {}
        self.stop_ev = threading.Event()
        self.tick_ev = threading.Event()
        self._snap = None
        self.k = 0
        self.tx_bytes = 0
        self.tx_err = 0
        self.rx_n = 0
        self.lost_since: float | None = None
        self.topo_payload: bytes | None = None
        self.vision = None
        self.ring: list = []
        self.cam_frames = 0
        self._last = {"cam": 0.0, "map": 0.0, "tof": 0.0, "status": 0.0}
        self._acc = None  # spike counts summed since the last BRAIN message
        self._acc_lock = threading.Lock()
        self._sent_k = {"tick": -1, "brain": -1}
        self.level = 2
        self.skipped = 0
        self._tof_stamp = {}
        safety = cfg.get("safety", {})
        self.fence_m = float(safety.get("geofence_radius_m", 3.0))
        self.ops = Operator(self.cfg, self.fence_m, pilot, drone)
        self.ops.on_event = self.event
        self.gestures = ConsoleGestures()
        self.limits = {
            "fence_m": self.fence_m,
            "max_alt_m": _r(safety.get("max_alt_m")),
            "min_alt_m": _r(safety.get("min_alt_m")),
            "max_forward": _r(safety.get("max_forward")),
            "v_max": _r(getattr(drone, "v_max", None)),
            "vz_max": _r(getattr(drone, "vz_max", None)),
            "goto_limit_m": round(self.fence_m - float(self.cfg["goto_fence_margin_m"]), 2),
        }
        self.shield = getattr(getattr(pilot, "struct", None), "shield", None)
        self.map = getattr(self.shield, "map", None)

    # ------------------------------------------------------------------ wiring
    def install(self) -> "ConsoleLink":
        p, d = self.pilot, self.drone
        dec_update = p.decoder.update

        def update(rates, dt):
            return self.ops.apply(dec_update(rates, dt))

        p.decoder.update = update
        enc = p.retina.encode

        def encode(frame):
            vf = enc(frame)
            self.vision = vf
            return vf

        p.retina.encode = encode
        if self.shield is not None and self.map is not None:
            mu = self.shield._map_update

            def map_update(drone, tel, t):
                r = mu(drone, tel, t)
                self.ring = r
                return r

            self.shield._map_update = map_update
        gate = d.in_control
        d.in_control = lambda: gate() and self.estop == 0
        self.topo_payload = zlib.compress(json.dumps(brain_topology(p.brain), separators=(",", ":")).encode(), 6)
        return self

    def start(self) -> "ConsoleLink":
        for fn, name in ((self._rx_loop, "console-rx"), (self._tx_loop, "console-tx")):
            threading.Thread(target=fn, name=name, daemon=True).start()
        self.log(f"console link on udp {self.port} (topology {len(self.topo_payload or b'') / 1024:.0f} kB)")
        return self

    def stop(self) -> None:
        self.stop_ev.set()
        self.tick_ev.set()

    # ------------------------------------------------------------------ events
    def event(self, src: str, text: str) -> None:
        with self.lock:
            self.ev_id += 1
            self.events.append((self.ev_id, round(time.monotonic(), 2), src, text))
        self.log(f"[console] {src}: {text}")

    def _poll_events(self) -> None:
        """New entries of the companion's own event lists -> the console log (read only, as the lists grow)."""
        srcs = (
            ("watchdog", getattr(self.pilot, "watchdog_events", None), lambda e: f"{e[1]}"),
            ("guard", getattr(self.drone, "guard_events", None), lambda e: f"{e[1]}"),
            ("governor", getattr(self.pilot.safety, "events", None), lambda e: f"{e}"),
            ("brain", getattr(self.pilot.decoder, "escapes", None),
             lambda e: f"escape: {e[1]}" + (f", saccade {'right' if e[2] > 0 else 'left'}" if len(e) > 2 and e[2] else "")),
        )
        for name, lst, fmt in srcs:
            if lst is None:
                continue
            n0 = self._seen.get(name, 0)
            n = len(lst)
            if n > n0:
                for e in list(lst[n0:n])[-20:]:
                    self.event(name, fmt(e))
                self._seen[name] = n

    # ------------------------------------------------------------------ main-loop hook (cheap: reference hand-off)
    def on_tick(self, info, row: dict, inc: bool, now: float) -> None:
        self.ops.on_control(inc)
        lost = self.had_console and not self._live(now)
        if lost:
            if self.lost_since is None:
                self.lost_since = now
                self.event("console", "link lost")
            if self.ops.mode in ("GOTO", "MANUAL"):
                self.ops.set_mode("HOLD", "console link lost")
            ll = float(self.cfg.get("lost_land_s", 20.0))
            if ll > 0 and inc and self.ops.mode == "HOLD" and now - self.lost_since > ll and not self.pilot.safety.land_requested:
                self.pilot.safety.land_requested = True
                self.ops.set_mode("LAND", f"console lost {now - self.lost_since:.0f} s")
        elif self.lost_since is not None:
            self.lost_since = None
            self.event("console", "link restored")
        if self.gestures.name != "none" and not self.gestures.active():
            self.gestures.name = "none"
            self.pilot.gestures = None
        counts = getattr(self.pilot.brain, "last_counts", None)
        if counts is not None and self.clients and LEVELS[self.level]["brain"]:  # level 0 sends no BRAIN: no sum either
            with self._acc_lock:
                self._acc = np.asarray(counts, np.int32).copy() if self._acc is None else self._acc + counts
        self._snap = (info, row, inc, now, counts, self.vision, self.ring)
        self.k += 1
        self.tick_ev.set()

    def _live(self, now=None) -> bool:
        now = time.monotonic() if now is None else now
        return any(now - c.last < float(self.cfg["link_timeout_s"]) for c in list(self.clients.values()))

    # ------------------------------------------------------------------ receive
    def _rx_loop(self) -> None:
        while not self.stop_ev.is_set():
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                self._estop_action()
                continue
            except OSError:
                if self.stop_ev.is_set():
                    return
                time.sleep(0.05)
                continue
            try:
                self._handle(data, addr)
            except Exception as e:  # never let a bad datagram end the receive thread
                self.event("console", f"bad message from {addr[0]}: {type(e).__name__}: {e}")
            self._estop_action()

    def _send(self, addr, dgrams: list[bytes]) -> None:
        for i, g in enumerate(dgrams):
            if i:
                time.sleep(0.001)  # pace the chunks of one message: bursts make the AIC8800 stall sooner
            try:
                self.sock.sendto(g, addr)
                self.tx_bytes += len(g)
            except OSError:
                self.tx_err += 1

    def _outq(self) -> int:
        """Bytes still in this socket's send queue (Linux SIOCOUTQ), 0 where unsupported."""
        try:
            import fcntl
            import termios

            return struct.unpack("i", fcntl.ioctl(self.sock.fileno(), termios.TIOCOUTQ, b"\0\0\0\0"))[0]
        except (OSError, ImportError, AttributeError):
            return 0

    def _ack(self, c: Client, cid, op: str, ok: bool, msg: str = "") -> None:
        self._send(c.addr, self.pk.json(T_ACK, {"id": cid, "op": op, "ok": ok, "msg": msg}))

    def _handle(self, data: bytes, addr) -> None:
        c = self.clients.get(addr)
        if c is None:
            if len(self.clients) >= 8:  # forget the stalest
                del self.clients[min(self.clients, key=lambda a: self.clients[a].last)]
            c = self.clients[addr] = Client(addr)
        r = c.rx.push(data)
        if r is None:
            return
        typ, body, _ = r
        self.rx_n += 1
        now = time.monotonic()
        if typ == T_PING:
            c.last = now
            self._send(addr, self.pk.datagrams(T_PONG, body[:8] + struct.pack("<d", now)))
            return
        if typ == T_HELLO:
            h = json.loads(body or b"{}")
            first = c.last == 0.0 or now - c.last > float(self.cfg["link_timeout_s"])
            c.last = now
            c.name = str(h.get("name", "?"))[:40]
            c.cam = str(h.get("cam", c.cam))
            c.cam_hz = max(0.0, min(15.0, float(h.get("cam_hz", c.cam_hz))))
            c.color_w = int(max(160, min(640, int(h.get("color_w", c.color_w)))))
            c.color_q = int(max(20, min(90, int(h.get("color_q", c.color_q)))))
            c.lvl = int(max(0, min(3, int(h.get("lvl", c.lvl)))))
            if first:
                self.had_console = True
                self.event("console", f"{c.name} connected from {addr[0]}")
            return
        if typ == T_TOPO_REQ:
            c.last = now
            if self.topo_payload:
                self._send(addr, self.pk.datagrams(T_TOPO, self.topo_payload, F_ZLIB))
            return
        if typ == T_ESTOP:
            m = json.loads(body)
            level = 2 if int(m.get("level", 1)) >= 2 else 1
            trig = m.get("trig")
            c.last = now
            with self.lock:
                new = level > self.estop
                self.estop = max(self.estop, level)
                if new or (trig is not None and trig != self.estop_trig):
                    self.estop_trig = trig
                    self.estop_done = False
                    self._fc_sent = 0.0
                if new:
                    self.estop_t = now
            if new:
                self.ops.set_mode("LAND", "emergency stop")
                self.event("console", f"EMERGENCY STOP: {'LAND' if level == 1 else 'KILL MOTORS'} ({c.name})")
            self._estop_action()
            self._ack(c, m.get("id"), "estop", True, ESTOP_NAMES[self.estop])
            return
        if typ == T_CMD:
            m = json.loads(body)
            c.last = now
            op = str(m.get("op", ""))
            if op == "stick":
                self.ops.set_stick(m.get("f", 0), m.get("l", 0), m.get("y", 0), m.get("z", 0))
                return
            c.cmds += 1
            ok, msg = self._command(op, m, c)
            self._ack(c, m.get("id"), op, ok, msg)

    def _command(self, op: str, m: dict, c: Client) -> tuple[bool, str]:
        if self.estop and op not in ("estop_reset", "handover", "gesture"):
            return False, "emergency stop latched: reset it first"
        if op == "hold":
            self.ops.set_mode("HOLD", c.name)
        elif op == "patrol":
            self.ops.set_mode("PATROL", c.name)
            self.ops._drop_goal()
        elif op == "goto":
            why = self.ops.set_goal(float(m.get("x", "nan")), float(m.get("y", "nan")))
            if why:
                return False, why
        elif op == "manual":
            if m.get("on", True):
                self.ops.set_stick(0, 0, 0, 0)
                self.ops.set_mode("MANUAL", c.name)
            else:
                self.ops.set_mode("HOLD", c.name)
        elif op == "land":
            self.ops.set_mode("LAND", c.name)
            if self.ops.inc:
                self.pilot.safety.land_requested = True  # EcpsPilot -> drone.land() next tick (in control only)
                return True, "companion LAND"
            return True, "companion not in control: the console sends LAND to the FC"
        elif op == "handover":
            mode = str(m.get("mode", "hold")).upper()
            if mode not in ("PATROL", "HOLD"):
                return False, f"hand-over mode {mode}?"
            self.ops.handover = mode
            self.event("operator", f"hand-over mode {mode}")
        elif op == "gesture":
            name = str(m.get("name", "none"))
            if name not in ("none", "palm", "fist", "left", "right", "rush", "drop"):
                return False, f"gesture {name}?"
            self.gestures.set(name)
            if name == "none":
                self.pilot.gestures = None
            elif self.pilot.gestures is not self.gestures:
                self.pilot.gestures = self.gestures
                self.event("operator", f"illusion {name}")
        elif op == "estop_reset":
            armed = self._armed()
            if armed and not m.get("force"):
                return False, "FC still armed: confirm with force"
            with self.lock:
                self.estop = 0
                self.estop_done = False
            self.ops.set_mode("HOLD", "emergency stop reset")
            self.event("console", f"emergency stop reset ({c.name}{', forced while armed' if armed else ''})")
        else:
            return False, f"unknown op {op}"
        return True, ""

    # ------------------------------------------------------------------ emergency stop: our own path to the FC
    def _armed(self) -> bool:
        try:
            return bool(self.drone.armed())
        except Exception:
            return False

    def _estop_action(self) -> None:
        if not self.estop or self.estop_done:
            return
        now = time.monotonic()
        if now - self._fc_sent < (0.2 if self.estop == 2 else 0.25):
            return
        m = getattr(self.drone, "m", None)
        if m is None:
            return
        mode = self.drone.fc_mode()
        if not self._armed() or (self.estop == 1 and mode == "LAND"):
            self.estop_done = True  # confirmed: from here on the pilot's mode switch / arming wins
            self.event("console", f"emergency stop confirmed by the FC ({'disarmed' if not self._armed() else 'LAND'})")
            return
        try:
            if self.estop == 2:
                m.mav.command_long_send(m.target_system, m.target_component, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
                text = b"ECPS: E-STOP KILL from console"
            else:
                m.set_mode("LAND")
                text = b"ECPS: E-STOP LAND from console"
            if self.fc_actions == 0 or now - self._fc_sent > 2.0:
                m.mav.statustext_send(2, text)  # MAV_SEVERITY_CRITICAL: the pilot sees why on the OSD / GCS
            self._fc_sent = now
            self.fc_actions += 1
        except Exception as e:
            self.event("console", f"emergency stop send failed: {e}")
            self._fc_sent = now

    # ------------------------------------------------------------------ transmit
    def _tx_loop(self) -> None:
        while not self.stop_ev.is_set():
            self.tick_ev.wait(0.5)
            self.tick_ev.clear()
            if self.stop_ev.is_set():
                return
            self._estop_action()
            now = time.monotonic()
            live = [c for c in list(self.clients.values()) if now - c.last < 3.0]
            if not live:
                continue
            snap = self._snap
            try:
                self._poll_events()
                if snap is not None:
                    self._tx(live, snap, now)
                elif now - self._last["status"] >= 1.0 / float(self.cfg["status_hz"]):  # before the first tick
                    self._last["status"] = now
                    self._bcast(live, self.pk.json(T_STATUS, self._status_json(live)))
            except Exception as e:
                self.tx_err += 1
                if self.tx_err < 20 or self.tx_err % 100 == 0:
                    self.log(f"[console] tx error: {type(e).__name__}: {e}")

    def _bcast(self, live, dgrams) -> None:
        for c in live:
            self._send(c.addr, dgrams)

    def _tx(self, live, snap, now) -> None:
        info, row, inc, t_mono, counts, vision, ring = snap
        self.level = min(c.lvl for c in live)
        L = LEVELS[self.level]
        k = self.k
        if k - self._sent_k["tick"] >= L["tick"]:
            self._sent_k["tick"] = k
            self._bcast(live, self.pk.json(T_TICK, self._tick_json(info, row, inc, ring)))
        if now - self._last["status"] >= 1.0 / L["status_hz"]:
            self._last["status"] = now
            self._bcast(live, self.pk.json(T_STATUS, self._status_json(live)))
        if self._outq() > int(self.cfg["outq_max"]):  # the Wi-Fi is not keeping up: only the small messages
            self.skipped += 1
            return
        if L["brain"] and k - self._sent_k["brain"] >= L["brain"]:
            with self._acc_lock:
                acc, self._acc = self._acc, None
            if acc is not None:
                self._sent_k["brain"] = k
                self._bcast(live, self.pk.datagrams(T_BRAIN, self._brain_bin(acc, vision)))
        if L["tof_hz"] and now - self._last["tof"] >= 1.0 / L["tof_hz"]:
            p = self._tof_bin()
            if p:
                self._last["tof"] = now
                self._bcast(live, self.pk.datagrams(T_TOF, p))
        for c in live:
            hz = min(c.cam_hz, L["cam_max"])
            if c.cam != "off" and hz > 0 and now - c.cam_t >= 1.0 / hz:
                p = self._cam_jpeg(c)
                if p:
                    c.cam_t = now
                    self._send(c.addr, self.pk.datagrams(T_CAM, p))
        if self.map is not None and now - self._last["map"] >= 1.0 / L["map_hz"]:
            self._last["map"] = now
            self._bcast(live, self.pk.datagrams(T_MAP, self._map_bin()))

    def _tick_json(self, info, row, inc, ring) -> dict:
        tel, raw, cmd = info.tel, info.raw, info.cmd
        braw = self.ops.brain_raw or raw
        dec = self.pilot.decoder
        el = getattr(dec, "_elapsed", 0.0)
        d = {
            "k": self.k,
            "t": row.get("t"),
            "fc": row.get("fc_mode"),
            "arm": row.get("armed"),
            "ctl": int(inc),
            "op": self.ops.effective(),
            "opm": self.ops.mode,
            "ho": self.ops.handover,
            "es": self.estop,
            "tel": [_r(tel.x_m), _r(tel.y_m), _r(tel.alt_m), _r(tel.yaw_deg, 1), _r(tel.vz_mps), _r(tel.yaw_rate_dps, 1),
                    _r(tel.battery_pct, 0)],
            "rng": _r(row.get("rng")),
            "exth": _r(row.get("ext_h")),
            "exts": row.get("ext_src", ""),
            "braw": [_r(braw.forward), _r(braw.yaw), _r(braw.throttle), int(bool(braw.escape))],
            "opc": [_r(raw.forward), _r(raw.lateral), _r(raw.yaw), _r(raw.throttle)],
            "cmd": [_r(cmd.forward), _r(cmd.lateral), _r(cmd.yaw), _r(cmd.throttle), int(bool(cmd.escape))],
            "note": (cmd.note or "")[:80],
            "bnote": (braw.note or "")[:60],
            "guard": row.get("guard", ""),
            "rates": {k: round(float(v), 1) for k, v in info.rates.items()},
            "dec": {
                "brake": _r(getattr(dec, "_brake", 0.0), 2),
                "sacc": float(getattr(dec, "_sacc_dir", 0.0)) if el <= getattr(dec, "_sacc_until", -1.0) else 0.0,
                "ret": int(bool(getattr(dec, "_ret_active", False))),
                "gf": _r(getattr(dec, "_gf", 0.0), 1),
                "mdn": _r(getattr(dec, "_mdn", 0.0), 1),
                "esc": len(getattr(dec, "escapes", []) or []),
            },
            "ms": row.get("compute_ms"),
            "dt": row.get("dt_ms"),
            "stg": {k[3:]: row[k] for k in row if k.startswith("ms_") and row[k]},
            "rtf": _r(info.rtf, 2),
            "spk": int(info.spikes),
            "age": [row.get("frame_age_ms"), row.get("tof_f_age_ms"), row.get("tof_r_age_ms")],
            "goal": list(self.ops.goal) if self.ops.goal else None,
            "gnote": self.ops.note if self.ops.effective() == "GOTO" else "",
            "ill": self.gestures.name if self.pilot.gestures is self.gestures else "none",
            "illm": self.pilot.illusion.mode if self.pilot.gestures is not None else "",
        }
        if self.shield is not None:
            nd, nb, nv = self.shield.dbg
            d["sh"] = [nd, nb, nv, int("shield" in (cmd.note or ""))]
        if ring:
            d["ring"] = [[round(math.degrees(math.atan2(u[1], u[0]))), round(di * 100)] for u, di in ring]
        return d

    def _brain_bin(self, counts, vision) -> bytes:
        cnt = np.minimum(np.asarray(counts), 255).astype(np.uint8)
        rows, cols = 6, 8
        grids = b""
        if vision is not None and getattr(vision, "eyes", None):
            g0 = vision.eyes["L"].grids.get("ftb")
            if g0 is not None:
                rows, cols = g0.shape
            parts = []
            for e in ("L", "R"):
                gr = vision.eyes[e].grids
                for f in VISION_FEATURES:
                    a = gr.get(f)
                    a = np.zeros((rows, cols), np.float32) if a is None else np.asarray(a, np.float32).reshape(rows, cols)
                    parts.append(np.clip(a * 255.0, 0, 255).astype(np.uint8).tobytes())
            grids = b"".join(parts)
        nfeat = len(VISION_FEATURES) if grids else 0
        return struct.pack("<IHBBB", self.k, cnt.size, rows, cols, nfeat) + zlib.compress(cnt.tobytes() + grids, 3)

    def _tof_bin(self) -> bytes:
        out = []
        changed = False
        for b, s in sorted(self.tofs.items()):
            f = getattr(s, "frame", None)
            if f is None:
                continue
            if self._tof_stamp.get(b) is not f:
                changed = True
                self._tof_stamp[b] = f
            a = np.asarray(f, np.float32).ravel()
            n = int(round(math.sqrt(a.size)))
            age = min(255, int(min(99.0, s.age_s()) * 100))
            out.append(struct.pack("<hBB", int(b), n, age) + a.astype(np.float16).tobytes())
        if not out or not changed:
            return b""
        return struct.pack("<B", len(out)) + b"".join(out)

    def _cam_jpeg(self, c: Client) -> bytes:
        import cv2

        if c.cam == "color":
            img = getattr(self.cam, "last_bgr", None)
            kind = 1
            if img is not None and img.shape[1] > c.color_w:
                h = int(round(img.shape[0] * c.color_w / img.shape[1]))
                img = cv2.resize(img, (c.color_w, h), interpolation=cv2.INTER_AREA)
            q = c.color_q
        else:
            img = None
            kind = 0
            q = 60
        if img is None:  # brain view: the 192x144 gray frame the retina gets (q 60: ~3 kB)
            img = None if self.cam is None else self.cam.latest()[0]
            kind = 0
            q = 60
        if img is None:
            return b""
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(q)])
        if not ok:
            return b""
        self.cam_frames += 1
        return struct.pack("<BHHI", kind, img.shape[1], img.shape[0], self.cam_frames) + buf.tobytes()

    def _map_bin(self) -> bytes:
        m = self.map
        L = m.L
        occ = np.flatnonzero(L > m.l_occ)
        conf = np.clip((L[occ] - m.l_occ) / (m.l_hi - m.l_occ) * 255.0, 0, 255).astype(np.uint8)
        delta = np.diff(occ, prepend=0).astype(np.uint32)
        hdr = json.dumps({"res": m.res, "half": m.half, "n": m.n, "nz": m.nz, "nvox": int(occ.size), "frames": m.frames,
                          "l_occ": m.l_occ, "l_hi": m.l_hi, "band": m.band, "query": m.query},
                         separators=(",", ":")).encode()
        return struct.pack("<H", len(hdr)) + hdr + zlib.compress(delta.tobytes() + conf.tobytes(), 3)

    def _status_json(self, live) -> dict:
        si = {}
        try:
            for z in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
                name = (z / "type").read_text().strip().replace("_thermal_zone", "").replace("_zone", "")
                si[f"T_{name}"] = int((z / "temp").read_text()) / 1000
        except OSError:
            pass
        try:
            si["load1"] = round(os.getloadavg()[0], 2)
        except OSError:
            pass
        with self.lock:
            ev = list(self.events)[-40:]
        dr = self.drone
        tofs = {}
        for b, s in self.tofs.items():
            tofs[s.name] = {"bearing": b, "status": str(getattr(s, "status", ""))[:40], "frames": getattr(s, "frames", 0),
                            "errors": getattr(s, "errors", 0)}
        return {
            "phase": "run" if self._snap is not None else "start",
            "run": self.run_info,
            "limits": self.limits,
            "events": ev,
            "sys": si,
            "cam": {"status": str(getattr(self.cam, "status", "off"))[:60], "frames": getattr(self.cam, "frames", 0)},
            "tof": tofs,
            "link": {"level": self.level, "skipped": self.skipped, "outq": self._outq()},
            "counters": {"cmd_sent": getattr(dr, "cmd_sent", 0), "cmd_blocked": getattr(dr, "cmd_blocked", 0),
                         "land_sent": getattr(dr, "land_sent", 0), "op_changes": self.ops.changes,
                         "estop_fc_actions": self.fc_actions, "tx_kB": round(self.tx_bytes / 1024), "tx_err": self.tx_err,
                         **(self.pilot.struct.summary() if self.pilot.struct is not None else {})},
            "consoles": [{"name": c.name, "addr": c.addr[0], "cam": c.cam} for c in live],
            "estop": {"level": self.estop, "since": round(time.monotonic() - self.estop_t, 1) if self.estop else None,
                      "confirmed": self.estop_done},
            "lost": round(time.monotonic() - self.lost_since, 1) if self.lost_since else None,
            "map": None if self.map is None else {"res": self.map.res, "half": self.map.half, "frames": self.map.frames},
        }

    def summary(self) -> dict:
        return {"port": self.port, "rx": self.rx_n, "tx_kB": round(self.tx_bytes / 1024), "tx_err": self.tx_err,
                "events": [e[1:] for e in list(self.events)[-100:]], "estop": self.estop, "op_changes": self.ops.changes}


def make(port: int, **kw) -> ConsoleLink:
    return ConsoleLink(port, **kw).install().start()
