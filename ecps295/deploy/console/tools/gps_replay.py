#!/usr/bin/env python3
"""gps_replay.py - a fake MAVLink flight controller for the ECPS Console "GPS map" page.

The console (mac/Sources/FCLink.swift) is a MAVLink ground station: it sends a
heartbeat every second and MAVLink commands to <host>:<port> over UDP from an
ephemeral source port, and it only accepts frames that pass the CRC check.
This script plays the role of the flight controller on the other end so the GPS
map page can be developed and checked without the drone.

Design notes
------------
* Socket: plain UDP bound to 127.0.0.2:14551 by default.  127.0.0.2 is a
  loopback alias on macOS; binding the specific alias (instead of 0.0.0.0)
  keeps this socket out of the way of the console's own MAVLink listener,
  which binds the same port number on all addresses.  We answer to the address
  of the last peer that sent us something, because the console's source port
  is ephemeral and unknown in advance.
  The bind is exclusive (no SO_REUSEADDR): start this script before the
  console, so the console's MAVLink listener cannot take the port and the
  console talks to us over its client path instead of to itself.
  Use --reuse when the console is already running.
* Frames are built with pymavlink (dialect ardupilotmega, MAVLink 2).
* Telemetry is replayed from a recorded outdoor GPS log (CSV).  The recorded
  time base is honoured (scaled by --speed) and the log loops.
* The flight controller's own position estimate (GLOBAL_POSITION_INT,
  LOCAL_POSITION_NED) is the EKF origin, NOT the GPS position: the fake FC
  does not fuse the GPS.  That difference is exactly what the GPS map page
  has to show, so it is deliberate, not a bug.

Usage
-----
    python3 tools/gps_replay.py [--csv PATH] [--host 127.0.0.2] [--port 14551]
                                [--push-port 14550] [--speed 1.0] [--armed]
                                [--mode N] [--origin LAT,LON] [--current N]
                                [--sim-motion] [--motion-radius M]
                                [--motion-period S] [--duration S] [--reuse]

The usual run next to the console (the console's MAVLink listener is on 14550,
the fake FC pushes to it, the way the Pi's mavlink-router pushes to the Mac):

    python3 tools/gps_replay.py --host 127.0.0.1 --port 14552 --push-port 14550
    "ECPS Console.app/Contents/MacOS/ECPSConsole" -host 127.0.0.1 -mavPort 14550
"""

import argparse
import csv
import errno
import math
import os
import socket
import sys
import time

from pymavlink.dialects.v20 import ardupilotmega as mav

DEFAULT_CSV = ("/Users/jacky_tan/Documents/UCI/课程文件/ECPS295/course_files_export/"
               "项目文档/09_台架测试/实测_20261125/e3_gps_static_20261125_145125.csv")

# The console is a ground station with sysid 250, compid 190 (FCLink.swift).
GCS_SYSID = 250
GCS_COMPID = 190

# FC identity
FC_SYSID = 1
FC_COMPID = 1

# MAVLink frame ids we produce / consume
MSG_HEARTBEAT = 0
MSG_GPS_RAW_INT = 24
MSG_ATTITUDE = 30
MSG_LOCAL_POSITION_NED = 32
MSG_GLOBAL_POSITION_INT = 33
MSG_MISSION_ITEM = 39
MSG_MISSION_REQUEST = 40
MSG_MISSION_CURRENT = 42
MSG_MISSION_REQUEST_LIST = 43
MSG_MISSION_COUNT = 44
MSG_MISSION_ACK = 47
MSG_GPS_GLOBAL_ORIGIN = 49
MSG_MISSION_REQUEST_INT = 51
MSG_RANGEFINDER = 173
MSG_HOME_POSITION = 242

MISSION_COUNT = 5

# Altitude the fake FC reports as "above MSL" (mm).  The real ground is at
# roughly 46 m ASL at the test site; the rangefinder reading is added on top.
ALT_MSL_MM = 46000

# CSV columns we know about.  Missing columns / unparsable cells become NaN
# and are treated as "no value" instead of crashing the replay.
CSV_COLUMNS = ["t", "rng", "ds", "q", "baro", "fix", "sats", "hdop", "hacc",
                "lat", "lon", "mag", "mhdg", "roll", "pitch"]

M_PER_DEG = 111319.49


def bind_socket(sock, host, port):
    """Bind to host:port; fall back to 127.0.0.1 when the loopback alias is missing.

    macOS only has 127.0.0.1 on lo0 by default, so 127.0.0.2 has to be
    aliased once:  sudo ifconfig lo0 alias 127.0.0.2
    Without the alias we fall back to 127.0.0.1 and say so loudly.
    """
    try:
        sock.bind((host, port))
        return host, port
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print("gps_replay: %s:%d is already in use (%s)" % (host, port, exc), flush=True)
            print("gps_replay: start this script before the console (the console's MAVLink listener takes "
                  "the same port), or use another --port", flush=True)
            raise
        if exc.errno != errno.EADDRNOTAVAIL:
            raise
        if host.startswith("127.") and host != "127.0.0.1":
            print("gps_replay: cannot bind %s:%d (%s)" % (host, port, exc), flush=True)
            print("gps_replay: add the loopback alias once with: "
                  "sudo ifconfig lo0 alias %s" % host, flush=True)
            print("gps_replay: falling back to 127.0.0.1 - point the console at 127.0.0.1:%d"
                  % port, flush=True)
            sock.bind(("127.0.0.1", port))
            return "127.0.0.1", port
        raise


def fnum(value):
    """float(value) or NaN when the cell is empty or unparsable."""
    try:
        if value is None:
            return float("nan")
        s = value.strip()
        if s == "":
            return float("nan")
        return float(s)
    except (TypeError, ValueError):
        return float("nan")


def isnan(v):
    return v != v


class Sink(object):
    """Minimal file-like sink so pymavlink can write frames into a buffer."""

    def __init__(self):
        self.buf = bytearray()

    def write(self, b):
        self.buf.extend(b)

    def flush(self):
        pass


class Rows(object):
    """The replayed CSV rows with float values."""

    def __init__(self, path):
        with open(path, "r", newline="") as fh:
            raw = list(csv.DictReader(fh))
        self.rows = []
        for r in raw:
            self.rows.append({k: fnum(r.get(k)) for k in CSV_COLUMNS})
        self.rows = [r for r in self.rows if not isnan(r["t"])]
        if not self.rows:
            raise ValueError("no usable rows in %s" % path)
        self.rows.sort(key=lambda r: r["t"])

    def __len__(self):
        return len(self.rows)


def first_valid_fix(rows):
    """First row with a usable GPS fix (lat/lon present, fix >= 1)."""
    for r in rows.rows:
        if not isnan(r["lat"]) and not isnan(r["lon"]) and not isnan(r["fix"]) and r["fix"] >= 1:
            return r
    return None


class Replay(object):
    """Walks the rows along the recorded time base, looping at the end."""

    def __init__(self, rows):
        self.rows = rows.rows
        self.n = len(self.rows)
        self.t0 = self.rows[0]["t"]
        if self.n > 1:
            diffs = [self.rows[i + 1]["t"] - self.rows[i]["t"] for i in range(self.n - 1)]
            diffs = [d for d in diffs if d > 0]
            self.dt = sorted(diffs)[len(diffs) // 2] if diffs else 0.21
        else:
            self.dt = 0.21
        self.period = (self.rows[-1]["t"] - self.t0) + self.dt
        self.idx = 0
        self.cycle = -1

    def sample(self, elapsed):
        """elapsed = replay seconds since start; returns (row, previous row)."""
        if self.period <= 0:
            return self.rows[0], self.rows[0]
        cyc = int(elapsed // self.period)
        local = elapsed - cyc * self.period + self.t0
        if cyc != self.cycle:
            self.cycle = cyc
            self.idx = 0
        while self.idx + 1 < self.n and self.rows[self.idx + 1]["t"] <= local:
            self.idx += 1
        prev = self.rows[self.idx - 1] if self.idx > 0 else self.rows[-1]
        return self.rows[self.idx], prev


def sim_offset(lat, lon, elapsed, radius, period):
    """Slow circular drift added to the GPS position (metres)."""
    if radius <= 0 or period <= 0:
        return 0.0, 0.0
    ang = 2.0 * math.pi * (elapsed / period)
    north = radius * math.cos(ang)
    east = radius * math.sin(ang)
    dlat = north / M_PER_DEG
    dlon = east / (M_PER_DEG * math.cos(math.radians(lat))) if lat else 0.0
    return dlat, dlon


class FakeFC(object):
    def __init__(self, args):
        self.args = args
        self.rows = Rows(args.csv)
        self.replay = Replay(self.rows)

        # EKF origin / home
        if args.origin:
            self.origin_lat, self.origin_lon = args.origin
        else:
            fix = first_valid_fix(self.rows)
            if fix is None:
                raise ValueError("no valid GPS fix in %s; pass --origin LAT,LON" % args.csv)
            self.origin_lat, self.origin_lon = fix["lat"], fix["lon"]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if args.reuse:
            # The console's own MAVLink listener binds the same port number on all addresses; with
            # SO_REUSEPORT both sockets can hold it (macOS picks one of them per datagram).
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                try:
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except OSError:
                    pass
        # Without --reuse the bind is exclusive: the console's MAVLink listener cannot take the same port
        # and its own heartbeat cannot loop back into it (the console then uses its client path, which is
        # what we answer).
        self.bind_addr = bind_socket(self.sock, args.host, args.port)
        self.sock.setblocking(False)
        self.peer = None
        # Push mode: send every frame to the console's MAVLink listener port, the way the Pi's mavlink-router
        # pushes to the Mac. The console's listener accepts the flow and answers to this socket.
        self.dest = ("127.0.0.1", args.push_port) if args.push_port else None

        self.tx_sink = Sink()
        self.mav = mav.MAVLink(self.tx_sink, srcSystem=FC_SYSID, srcComponent=FC_COMPID)
        self.rx_sink = Sink()
        self.rx = mav.MAVLink(self.rx_sink)

        self.start = time.monotonic()
        self.due = {}          # message kind -> next replay time
        self.counts = {}       # message kind -> number of frames sent
        self.mission_items = set()
        self.mission_delivered = 0
        self.mission_requested = False
        self.last_warn = 0.0
        self.last_status = 0.0

    # ------------------------------------------------------------------ io
    def send(self, msg):
        dest = self.dest or self.peer
        if dest is None:
            return
        self.tx_sink.buf.clear()
        self.mav.send(msg)
        data = bytes(self.tx_sink.buf)
        try:
            self.sock.sendto(data, dest)
        except OSError as exc:
            self.warn("send failed: %s" % exc)
            return
        kind = msg.get_type()
        self.counts[kind] = self.counts.get(kind, 0) + 1

    def warn(self, text):
        now = time.monotonic()
        if now - self.last_warn >= 1.0:
            self.last_warn = now
            print("gps_replay: %s" % text, flush=True)

    def drain(self):
        """Read pending datagrams and answer the mission protocol."""
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError as exc:
                self.warn("recv failed: %s" % exc)
                return
            self.peer = addr
            if not data:
                continue
            try:
                msgs = self.rx.parse_buffer(data)
            except Exception as exc:                      # malformed frame
                self.warn("malformed datagram from %s: %s" % (addr[0], exc))
                continue
            if not msgs:
                self.warn("malformed datagram from %s (CRC)" % addr[0])
                continue
            if isinstance(msgs, tuple):                   # older pymavlink API
                msgs = [msgs[0]]
            for msg in msgs:
                self.handle(msg)

    # ------------------------------------------------------- mission protocol
    def handle(self, msg):
        kind = msg.get_type()
        if kind == "MISSION_REQUEST_LIST":
            self.mission_requested = True
            self.send(self.mav.mission_count_encode(
                target_system=msg.get_srcSystem() or GCS_SYSID,
                target_component=msg.get_srcComponent() or GCS_COMPID,
                count=MISSION_COUNT))
        elif kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = int(msg.seq)
            self.send_mission_item(seq, msg.get_srcSystem() or GCS_SYSID,
                                   msg.get_srcComponent() or GCS_COMPID)
        elif kind == "COMMAND_LONG":
            # The console's display stream requests: ignore them silently.
            pass

    def mission_items_list(self):
        """Five waypoints: a 20 m square around the EKF origin."""
        dlat = 20.0 / M_PER_DEG
        coslat = math.cos(math.radians(self.origin_lat))
        dlon = (20.0 / M_PER_DEG) / coslat if coslat else 0.0
        olat, olon = self.origin_lat, self.origin_lon
        offsets = [(0.0, 0.0), (dlat, 0.0), (dlat, dlon), (0.0, dlon), (0.0, 0.0)]
        return [(olat + dlat_o, olon + dlon_o) for dlat_o, dlon_o in offsets]

    def send_mission_item(self, seq, target_system, target_component):
        items = self.mission_items_list()
        if seq < 0 or seq >= len(items):
            return
        lat, lon = items[seq]
        self.send(self.mav.mission_item_encode(
            target_system=target_system,
            target_component=target_component,
            seq=seq,
            frame=self.args.mission_frame,
            command=mav.MAV_CMD_NAV_WAYPOINT,
            current=1 if seq == 0 else 0,
            autocontinue=1,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
            x=lat, y=lon, z=30.0))
        self.mission_items.add(seq)
        self.mission_delivered = len(self.mission_items)
        if len(self.mission_items) >= MISSION_COUNT:
            self.send(self.mav.mission_ack_encode(
                target_system=GCS_SYSID, target_component=GCS_COMPID,
                type=mav.MAV_MISSION_ACCEPTED))
            print("gps_replay: mission delivered (%d items)" % MISSION_COUNT, flush=True)
            self.mission_items = set()   # allow a later re-download

    # ------------------------------------------------------------- telemetry
    def due_now(self, kind, interval, elapsed):
        nxt = self.due.get(kind)
        if nxt is None or elapsed >= nxt:
            self.due[kind] = elapsed + interval
            return True
        return False

    def emit(self, row, prev, elapsed):
        args = self.args
        lat = row["lat"]
        lon = row["lon"]
        fix = row["fix"]
        has_fix = (not isnan(lat) and not isnan(lon) and not isnan(fix) and fix >= 1)

        # ---- HEARTBEAT (1 Hz)
        if self.due_now("HEARTBEAT", 1.0, elapsed):
            self.send(self.mav.heartbeat_encode(
                type=mav.MAV_TYPE_QUADROTOR,
                autopilot=mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode=128 if args.armed else 0,
                custom_mode=args.mode,
                system_status=mav.MAV_STATE_ACTIVE))

        # ---- MISSION_CURRENT (1 Hz)
        if self.due_now("MISSION_CURRENT", 1.0, elapsed):
            self.send(self.mav.mission_current_encode(seq=args.current, total=MISSION_COUNT))

        # ---- GPS / position messages (5 Hz)
        if self.due_now("GPS", 0.2, elapsed):
            rng = row["rng"]
            if isnan(rng):
                rng = 0.0
            sats = row["sats"]
            if isnan(sats) or sats < 0:
                sats = 0
            sats = int(sats)
            hdop = row["hdop"]
            if isnan(hdop):
                hdop = 0.0
            hacc = row["hacc"]
            if isnan(hacc):
                hacc = 0.0
            mhdg = row["mhdg"]
            if isnan(mhdg):
                mhdg = 0.0
            alt_mm = ALT_MSL_MM + int(rng * 1000)

            gps_lat, gps_lon = lat, lon
            if has_fix and args.sim_motion:
                dlat, dlon = sim_offset(lat, lon, elapsed, args.motion_radius,
                                        args.motion_period)
                gps_lat += dlat
                gps_lon += dlon

            # velocity from the recorded track
            vel_cms, cog_cdeg = 0, 0
            if has_fix and not isnan(prev["lat"]) and not isnan(prev["lon"]):
                dt = row["t"] - prev["t"]
                if dt > 0:
                    p_lat, p_lon = prev["lat"], prev["lon"]
                    if args.sim_motion:
                        p_dlat, p_dlon = sim_offset(p_lat, p_lon, elapsed - dt,
                                                    args.motion_radius, args.motion_period)
                        p_lat += p_dlat
                        p_lon += p_dlon
                    dn = (gps_lat - p_lat) * M_PER_DEG
                    de = (gps_lon - p_lon) * M_PER_DEG * math.cos(math.radians(gps_lat))
                    vn, ve = dn / dt, de / dt
                    vel_cms = int(round(math.hypot(vn, ve) * 100.0))
                    cog_cdeg = int(round((math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0 * 100.0))

            fix_type = 0
            if not isnan(fix):
                if fix >= 3:
                    fix_type = 3      # 3D fix
                elif fix >= 1:
                    fix_type = 1      # fix

            # GPS_RAW_INT: the GPS receiver's own output
            if has_fix:
                self.send(self.mav.gps_raw_int_encode(
                    time_usec=int(elapsed * 1e6),
                    fix_type=fix_type,
                    lat=int(round(gps_lat * 1e7)),
                    lon=int(round(gps_lon * 1e7)),
                    alt=alt_mm,
                    eph=int(round(hdop * 100)),
                    epv=int(round(hdop * 100)),
                    vel=vel_cms,
                    cog=cog_cdeg,
                    satellites_visible=sats,
                    alt_ellipsoid=alt_mm,
                    h_acc=int(round(hacc * 1000)),
                    v_acc=int(round(hacc * 1000)),
                    vel_acc=100,
                    hdg_acc=100,
                    yaw=int(round(mhdg * 100))))

            # GLOBAL_POSITION_INT: the FC's estimate = the EKF origin (always available)
            self.send(self.mav.global_position_int_encode(
                time_boot_ms=int(elapsed * 1000),
                lat=int(round(self.origin_lat * 1e7)),
                lon=int(round(self.origin_lon * 1e7)),
                alt=alt_mm,
                relative_alt=int(round(rng * 1000)),
                vx=0, vy=0, vz=0,
                hdg=65535))

            # LOCAL_POSITION_NED: GPS position relative to the EKF origin
            if has_fix:
                n = (gps_lat - self.origin_lat) * M_PER_DEG
                e = (gps_lon - self.origin_lon) * M_PER_DEG * math.cos(math.radians(self.origin_lat))
                self.send(self.mav.local_position_ned_encode(
                    time_boot_ms=int(elapsed * 1000),
                    x=n, y=e, z=-rng,
                    vx=0.0, vy=0.0, vz=0.0))

            # ATTITUDE (5 Hz)
            roll = row["roll"]
            pitch = row["pitch"]
            if isnan(roll):
                roll = 0.0
            if isnan(pitch):
                pitch = 0.0
            self.send(self.mav.attitude_encode(
                time_boot_ms=int(elapsed * 1000),
                roll=math.radians(roll),
                pitch=math.radians(pitch),
                yaw=math.radians(mhdg),
                rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0))

            # RANGEFINDER (5 Hz)
            self.send(self.mav.rangefinder_encode(distance=rng, voltage=0.0))

        # ---- GPS_GLOBAL_ORIGIN / HOME_POSITION (0.2 Hz)
        if self.due_now("ORIGIN", 5.0, elapsed):
            self.send(self.mav.gps_global_origin_encode(
                latitude=int(round(self.origin_lat * 1e7)),
                longitude=int(round(self.origin_lon * 1e7)),
                altitude=ALT_MSL_MM,
                time_usec=int(elapsed * 1e6)))
            self.send(self.mav.home_position_encode(
                latitude=int(round(self.origin_lat * 1e7)),
                longitude=int(round(self.origin_lon * 1e7)),
                altitude=ALT_MSL_MM,
                x=0.0, y=0.0, z=0.0,
                q=[1.0, 0.0, 0.0, 0.0],
                approach_x=0.0, approach_y=0.0, approach_z=0.0))

    # ------------------------------------------------------------------ loop
    def run(self):
        args = self.args
        print("gps_replay: FC sysid %d compid %d, csv %s" % (FC_SYSID, FC_COMPID, args.csv), flush=True)
        print("gps_replay: EKF origin (home) = %.7f, %.7f  alt %d mm" %
              (self.origin_lat, self.origin_lon, ALT_MSL_MM), flush=True)
        print("gps_replay: note: the GPS position and the FC position are different things - "
              "this fake FC does not fuse the GPS, its estimate stays at the EKF origin.", flush=True)
        print("gps_replay: listening on %s:%d, waiting for the ground station..." %
              (self.bind_addr[0], self.bind_addr[1]), flush=True)
        if self.dest:
            print("gps_replay: pushing every frame to %s:%d (the console's MAVLink listener port)" %
                  (self.dest[0], self.dest[1]), flush=True)

        while True:
            now = time.monotonic()
            elapsed = (now - self.start) * args.speed
            if args.duration is not None and (now - self.start) >= args.duration:
                break
            self.drain()
            row, prev = self.replay.sample(elapsed)
            self.emit(row, prev, elapsed)

            if now - self.last_status >= 1.0:
                self.last_status = now
                peer = ("%s:%d" % self.peer) if self.peer else "-"
                if self.dest:
                    peer = "%s:%d(push)" % (self.dest[0], self.dest[1])
                print("t=%.1fs peer=%s gps=%d sent=%d mission=%s items=%d/%d" % (
                    now - self.start, peer,
                    self.counts.get("GPS_RAW_INT", 0),
                    sum(self.counts.values()),
                    "yes" if self.mission_items or self.mission_requested else "no",
                    self.mission_delivered, MISSION_COUNT), flush=True)
            time.sleep(0.005)

        print("gps_replay: stopped after %.1f s" % (time.monotonic() - self.start), flush=True)


def parse_origin(text):
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("--origin expects LAT,LON")
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        raise argparse.ArgumentTypeError("--origin expects LAT,LON")


def main(argv=None):
    ap = argparse.ArgumentParser(description="fake MAVLink FC replaying a recorded GPS log")
    ap.add_argument("--csv", default=DEFAULT_CSV, help="CSV log to replay")
    ap.add_argument("--host", default="127.0.0.2", help="UDP address to bind")
    ap.add_argument("--port", type=int, default=14551, help="UDP port to bind")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    ap.add_argument("--armed", action="store_true", help="report the FC as armed")
    ap.add_argument("--mode", type=int, default=0, help="ArduCopter custom_mode (0 = STABILIZE)")
    ap.add_argument("--origin", type=parse_origin, default=None, help="EKF origin LAT,LON")
    ap.add_argument("--current", type=int, default=1, help="MISSION_CURRENT seq")
    ap.add_argument("--mission-frame", type=int, default=mav.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    help="MAV_FRAME of the mission items (default MAV_FRAME_GLOBAL_RELATIVE_ALT=3; "
                         "use 6 for MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)")
    ap.add_argument("--sim-motion", action="store_true", help="add a slow circular GPS drift")
    ap.add_argument("--motion-radius", type=float, default=12.0, help="drift radius in m")
    ap.add_argument("--motion-period", type=float, default=45.0, help="drift period in s")
    ap.add_argument("--duration", type=float, default=None, help="stop after N seconds")
    ap.add_argument("--push-port", type=int, default=None,
                    help="the console's MAVLink listener port: every frame is pushed to 127.0.0.1:<port>, "
                         "the way the Pi's mavlink-router pushes to the Mac. Without it the frames go to the "
                         "last peer that talked to us (the console's client path).")
    ap.add_argument("--reuse", action="store_true",
                    help="allow the port to be shared (SO_REUSEADDR/SO_REUSEPORT): only when the console is "
                         "already running; the default is an exclusive bind, so the console's own MAVLink "
                         "listener cannot take the port and talk to itself")
    args = ap.parse_args(argv)

    if not os.path.exists(args.csv):
        print("gps_replay: CSV not found: %s" % args.csv, file=sys.stderr)
        return 2
    if args.speed <= 0:
        print("gps_replay: --speed must be > 0", file=sys.stderr)
        return 2

    try:
        fc = FakeFC(args)
    except (OSError, ValueError) as exc:
        print("gps_replay: %s" % exc, file=sys.stderr)
        return 2

    try:
        fc.run()
    except KeyboardInterrupt:
        print("gps_replay: stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
