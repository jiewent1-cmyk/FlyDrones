#!/usr/bin/env python3
"""ECPS295 course mission tool: the 20 ft cage AUTO waypoint flight (take off, square, return, land).

The FC flies the mission itself (ArduCopter AUTO) on optical flow + the companion's route-B height.
There is no GPS position: the companion sets a fixed EKF origin (33.6430, -117.8420, 20 m) while the
vehicle is disarmed, so every waypoint is a position relative to that origin.

Sub-commands
    plan       build the mission: a Mission Planner .waypoints file (QGC WPL 110) + a JSON sidecar
    params     compare the FC with the field parameter set, optionally write it (--apply --yes)
    upload     upload the .waypoints file with the MAVLink mission protocol and read it back
    preflight  read-only checks, PASS / FAIL per line
    start      switch LOITER -> AUTO, but only if the pilot has already armed in LOITER
    monitor    one line per second until the vehicle disarms, plus a CSV

The tool NEVER arms, NEVER takes off by command and NEVER touches the fence parameters.
Arming is the pilot's job with the RC (the flight manual: RC to LOITER, arm with the RC, then the
computer sets AUTO). The fence is the user's decision; `params` only reports it.

Start order (AUTO_OPTIONS 0): arm in LOITER with the RC, run `start` within a few seconds (the FC disarms by
itself after DISARM_DELAY = 10 s on the ground with the throttle at the minimum), then raise the throttle above
mid-stick: the mission's TAKEOFF begins. Raising the throttle BEFORE the switch to AUTO takes off in LOITER instead.

Examples
    course_mission.py plan --alt 2.0 --side 1.5 --end rtl --out ~/sim/mission/course_s15
    course_mission.py params --url udpin:127.0.0.1:14570
    course_mission.py params --apply --yes
    course_mission.py upload --file ~/sim/mission/course_s15.waypoints
    course_mission.py preflight --file ~/sim/mission/course_s15.waypoints
    course_mission.py start
    course_mission.py monitor --csv ~/sim/mission/flight.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

from pymavlink import mavutil

# --------------------------------------------------------------------------------------------------------------
# constants

# the EKF origin the companion sets while the vehicle is disarmed (see the Pi companion)
DEFAULT_ORIGIN = (33.6430, -117.8420, 20.0)

# the companion's MAVLink system id; the FC treats it as the GCS link (MAV_GCS_SYSID 254)
COMPANION_SYSID = 254

# this tool's own MAVLink ids: NOT 254, so the tool is not counted as the FC's GCS link
TOOL_SYSID = 250
TOOL_COMPID = 191

# ArduCopter 4.7.0 mode numbers (CopterMode enum)
MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
         6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
         15: "AUTOTUNE", 16: "SMART_RTL", 17: "FLOWHOLD", 18: "FOLLOW", 19: "ZIGZAG",
         20: "BRAKE", 21: "THROW", 22: "AVOID_ADSB", 23: "GUIDED_NOGPS", 24: "Turtle"}

MAV_CMD = {16: "WAYPOINT", 17: "LOITER_UNLIM", 18: "LOITER_TURNS", 19: "LOITER_TIME",
           20: "RTL", 21: "LAND", 22: "TAKEOFF", 31: "DO_FOLLOW", 82: "SPLINE_WAYPOINT",
           93: "DO_DELAY"}

ORIGIN_MAX_M = 0.5    # review 2026-11-25: max distance of the vehicle from the EKF origin at the start (see start)
FRAME_GLOBAL_RELATIVE_ALT = 3        # Mission Planner .waypoints frame column
FRAME_GLOBAL_RELATIVE_ALT_INT = 6    # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

# the field parameter set this mission needs. name: (value, where the value comes from)
# checked against ArduCopter 4.7.0 (~/sim/ap-fw470): the 4.7 renames are applied below.
WANTED = {
    "RTL_ALT_M": (2.0, "flight manual: RTL_ALT_M 2, marked critical"),
    "RTL_SPEED_MS": (0.4, "same speed as WP_SPD; the FC value 0 means 'inherit WP_SPD'"),
    "RTL_CLIMB_MIN_M": (0.0, "RTL from 2 m to RTL_ALT_M 2 m needs no climb"),
    "RTL_LOIT_TIME": (5000.0, "FC value: 5 s hover at home before the descent"),
    "WP_SPD": (0.4, "2026-11-18 tool + SITL study: 0.4 m/s inside the 20 ft cage"),
    "WP_SPD_UP": (0.5, "same study: climb 0.5 m/s"),
    "WP_SPD_DN": (0.4, "same study: descend 0.4 m/s"),
    "WP_RADIUS_M": (0.2, "same study: 0.2 m so the corners of the square are really reached"),
    "WP_ACC": (1.0, "same study: 1.0 m/s^2"),
    "LOIT_SPEED_MS": (1.0, "same study: 1.0 m/s in LOITER"),
    "LAND_SPD_MS": (0.5, "4.7.0 name of the old LAND_SPEED; FC already 0.5"),
    "AUTO_OPTIONS": (0.0, "no arming in Auto and no take-off without raising the throttle: the "
                            "pilot raises the throttle as the manual says"),
    "FS_THR_ENABLE": (3.0, "flight manual: throttle failsafe always lands"),
}

# reported, never written: the fence is the user's decision (the pilot disabled it on 2026-11-25
# after an altitude-fence LAND ended in a crash) and these are context for the flight.
REPORT_ONLY = {
    # review 2026-11-25: the battery thresholds stay the FC's 6.6 / 6.4 V (2S), which are MORE conservative than the
    # manual's 6.1 V (decided 2026-11-18); a 5.8 V critical level would be 2.9 V per cell. Reported, not written.
    "BATT_LOW_VOLT": (6.6, "FC value, above the manual's 6.1 V on purpose (2S: 3.3 V per cell)"),
    "BATT_CRT_VOLT": (6.4, "FC value, must stay below BATT_LOW_VOLT"),
    "FENCE_ENABLE": (0.0, "FC value: the pilot disabled the fence; this tool never writes it"),
    "FENCE_ACTION": (2.0, "flight manual: always land (report only, never written)"),
    "FENCE_ALT_MAX": (1.3, "FC value: 1.3 m would break a 2 m mission if the fence were on"),
    "FENCE_MARGIN": (0.3, "FC value"),
    "FENCE_RADIUS": (3.0, "FC value: the far corner of a 2.0 m square is 2.83 m from the origin"),
    "FENCE_TYPE": (1.0, "FC value"),
    "FS_GCS_ENABLE": (5.0, "FC value 5 = always land; this is what a companion loss does"),
    "FS_OPTIONS": (16.0, "FC value: bit 4 = continue in pilot controlled modes on GCS loss"),
    "FS_GCS_TIMEOUT": (5.0, "FC value: 5 s without the companion heartbeat before the failsafe"),
    "EK3_SRC1_POSXY": (0.0, "FC value: no GPS position is fused (flow only)"),
    "EK3_SRC1_VELXY": (5.0, "FC value: optical flow velocity"),
    "EK3_SRC1_POSZ": (6.0, "FC value: the companion's route-B height (external nav / vision)"),
    "FLOW_FYSCALER": (220.0, "FC value: +22 % on the forward axis, suspect (see the report)"),
}

# parameters of the flight itself that the tool needs but does not write
CONTEXT_ONLY = ("EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ", "FS_GCS_ENABLE", "FS_OPTIONS",
                "FS_GCS_TIMEOUT", "FLOW_FYSCALER")


# --------------------------------------------------------------------------------------------------------------
# small helpers

def m_per_deg(lat_deg):
    """Metres per degree of latitude / longitude at a latitude (WGS84, ~1 cm accurate)."""
    lat = math.radians(lat_deg)
    m_lat = 111132.92 - 559.82 * math.cos(2 * lat) + 1.175 * math.cos(4 * lat) - 0.0023 * math.cos(6 * lat)
    m_lon = 111412.84 * math.cos(lat) - 93.5 * math.cos(3 * lat) + 0.118 * math.cos(5 * lat)
    return m_lat, m_lon


def offset_to_ne(x, y, heading_deg):
    """Body frame offset (x forward, y right) to north / east metres for a heading (clockwise from north)."""
    h = math.radians(heading_deg)
    return x * math.cos(h) - y * math.sin(h), x * math.sin(h) + y * math.cos(h)


def ne_to_latlon(north_m, east_m, lat0, lon0):
    """North / east metres from the origin to latitude / longitude."""
    m_lat, m_lon = m_per_deg(lat0)
    return lat0 + north_m / m_lat, lon0 + east_m / m_lon


def latlon_to_ne(lat, lon, lat0, lon0):
    """Latitude / longitude to north / east metres from the origin."""
    m_lat, m_lon = m_per_deg(lat0)
    return (lat - lat0) * m_lat, (lon - lon0) * m_lon


def parse_origin(text):
    """'lat,lon,alt' -> (lat, lon, alt)."""
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("origin must be lat,lon,alt")
    lat, lon, alt = float(parts[0]), float(parts[1]), float(parts[2])
    if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
        raise argparse.ArgumentTypeError("origin lat/lon out of range")
    return lat, lon, alt


def parse_waypoints(text):
    """'x,y;x,y;...' -> [(x, y), ...] in metres, body frame (x forward, y right)."""
    pts = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.split(",")
        if len(parts) != 2:
            raise argparse.ArgumentTypeError("waypoint '%s' must be x,y" % chunk)
        pts.append((float(parts[0]), float(parts[1])))
    if not pts:
        raise argparse.ArgumentTypeError("no waypoints given")
    return pts


def square_points(side):
    """The square of `side` metres, starting at the arming point (0,0) and returning to it."""
    return [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]


def square_points_ccw(side):
    """The square of `side` metres the other way round (also returns to the arming point)."""
    return [(0.0, side), (side, side), (side, 0.0), (0.0, 0.0)]


# --------------------------------------------------------------------------------------------------------------
# the mission

class Mission:
    """The mission: a list of items, each (command, x_body, y_body, alt_m, hold_s) plus the origin."""

    def __init__(self, origin, heading_deg, alt, hold, end):
        self.origin = origin
        self.heading_deg = heading_deg
        self.alt = alt
        self.hold = hold
        self.end = end
        self.items = []   # (command, x, y, alt, hold)

    def add(self, command, x, y, alt, hold=0.0):
        self.items.append((command, x, y, alt, hold))

    # -- geometry ------------------------------------------------------------------------------------------
    def ne(self, x, y):
        return offset_to_ne(x, y, self.heading_deg)

    def global_xy(self, x, y):
        lat0, lon0, _ = self.origin
        north, east = self.ne(x, y)
        return ne_to_latlon(north, east, lat0, lon0)

    # -- files ---------------------------------------------------------------------------------------------
    def to_json(self):
        lat0, lon0, alt0 = self.origin
        out = {
            "tool": "course_mission.py",
            "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "origin": {"lat": lat0, "lon": lon0, "alt": alt0,
                       "note": "the EKF origin the companion sets while the vehicle is disarmed"},
            "heading_deg": self.heading_deg,
            "heading_note": ("the vehicle's heading at arming; x = forward, y = right in that frame. "
                             "0 means the x axis points north, which is only right if the vehicle "
                             "faces north when it is armed"),
            "waypoint_alt_m": self.alt,
            "hold_s": self.hold,
            "end": self.end,
            "items": [],
        }
        for seq, (cmd, x, y, alt, hold) in enumerate(self.items):
            lat, lon = self.global_xy(x, y)
            out["items"].append({
                "seq": seq, "command": cmd, "command_name": MAV_CMD.get(cmd, str(cmd)),
                "x_body_m": x, "y_body_m": y, "alt_m": alt, "hold_s": hold,
                "lat": round(lat, 7), "lon": round(lon, 7),
            })
        return out

    def header_comment(self):
        lat0, lon0, alt0 = self.origin
        lines = [
            "# ECPS295 course mission (20 ft cage), written by course_mission.py",
            "# origin (EKF origin set by the companion while disarmed): %.7f %.7f %.1f m" % (lat0, lon0, alt0),
            "# altitude column: metres RELATIVE TO HOME (= relative to the origin above)",
            "# offsets: the body frame of the vehicle's heading at arming, x = forward, y = right",
            "# heading at arming used here: %.1f deg clockwise from north" % self.heading_deg,
            "#   with heading 0 the x axis points NORTH and the y axis EAST, so the square is",
            "#   north-aligned; with --heading-deg H the x axis points H deg clockwise from north",
            "#   and the square is rotated accordingly. The tool cannot know the heading at",
            "#   arming when the mission is written (it is written before the pilot arms), so",
            "#   --heading-deg must be set to the heading the vehicle will have when it is armed.",
            "# waypoint altitude %.2f m, hold %.1f s per waypoint, ends with %s" %
            (self.alt, self.hold, self.end.upper()),
            "# item 0 is the home slot: the FC replaces it with its own home position",
            "# columns: seq current frame command p1 p2 p3 p4 x(lat) y(lon) z(alt) autocontinue",
        ]
        return lines

    def to_wpl(self):
        """The QGC WPL 110 text (importable in the Mission Planner PLAN page).

        QGC WPL 110 columns: seq, current, frame, command, param1..param4, x(lat), y(lon), z(alt),
        autocontinue.
        """
        lines = ["QGC WPL 110"] + self.header_comment()
        for seq, (cmd, x, y, alt, hold) in enumerate(self.items):
            lat, lon = self.global_xy(x, y)
            lines.append("%d\t%d\t%d\t%d\t%.3f\t%.3f\t%.3f\t%.3f\t%.7f\t%.7f\t%.3f\t1" % (
                seq, 1 if seq == 0 else 0, FRAME_GLOBAL_RELATIVE_ALT, cmd,
                hold, 0.0, 0.0, 0.0, lat, lon, alt))
        return "\n".join(lines) + "\n"


def build_mission(args):
    """Build the mission from the command line (a square or an explicit waypoint list)."""
    m = Mission(args.origin, args.heading_deg, args.alt, args.hold, args.end)
    if args.wp:
        pts = parse_waypoints(args.wp)
    elif args.side > 0:
        pts = square_points_ccw(args.side) if args.ccw else square_points(args.side)
    else:
        pts = [(0.0, 0.0)]
    # item 0: the home slot (the FC replaces it with its own home)
    m.add(16, 0.0, 0.0, 0.0)
    # item 1: take off to the mission altitude
    m.add(22, 0.0, 0.0, args.alt)
    # the waypoints
    for x, y in pts:
        m.add(16, x, y, args.alt, args.hold)
    # the last item: RTL or LAND
    if args.end == "land":
        lx, ly = pts[-1]
        m.add(21, lx, ly, 0.0)
    else:
        m.add(20, 0.0, 0.0, 0.0)
    return m, pts


def write_waypoints_file(m, path):
    with open(path, "w") as f:
        f.write(m.to_wpl())


def parse_wpl_file(path):
    """Read a .waypoints file back -> list of dicts.

    QGC WPL 110 columns: seq, current, frame, command, param1..param4, x(lat), y(lon), z(alt),
    autocontinue. Files with the 11 column variant (no 'current') are accepted as well.
    """
    items = []
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("QGC WPL"):
                continue
            cols = line.split()
            if len(cols) < 11:
                continue
            if len(cols) >= 12:
                seq, current, frame, command = int(cols[0]), int(cols[1]), int(cols[2]), int(cols[3])
                p1, p2, p3, p4 = (float(cols[4]), float(cols[5]), float(cols[6]), float(cols[7]))
                lat, lon, alt = float(cols[8]), float(cols[9]), float(cols[10])
                autocontinue = int(cols[11])
            else:
                seq, frame, command = int(cols[0]), int(cols[1]), int(cols[2])
                p1, p2, p3, p4 = (float(cols[3]), float(cols[4]), float(cols[5]), float(cols[6]))
                lat, lon, alt = float(cols[7]), float(cols[8]), float(cols[9])
                autocontinue = int(cols[10])
            items.append({
                "seq": seq, "current": current, "frame": frame, "command": command,
                "p1": p1, "p2": p2, "p3": p3, "p4": p4,
                "lat": lat, "lon": lon, "alt": alt, "autocontinue": autocontinue,
            })
    return items


# --------------------------------------------------------------------------------------------------------------
# the FC connection

class FC:
    """A MAVLink connection to the FC, with the small helpers this tool needs."""

    def __init__(self, url, timeout=30.0):
        self.url = url
        self.m = mavutil.mavlink_connection(url, source_system=TOOL_SYSID, source_component=TOOL_COMPID)
        self.state = {
            "mode": None, "armed": False, "hb": 0.0, "comp_hb": 0.0,
            "lat": None, "lon": None, "alt": None, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0,
            "ekf_flags": 0, "ekf_vs": None, "rng": None, "rng_status": None,
            "flow_q": None, "flow_dist": None, "volt": None, "curr": None,
            "seq": -1, "reached": [], "texts": [], "rc_ok": False, "rc_ch": 0, "rc_values": [],
            "home_lat": None, "home_lon": None, "home_alt": None,
        }
        self.t0 = time.time()
        self._last_print = 0.0

    def say(self, msg):
        print("[%6.1f] %s" % (time.time() - self.t0, msg), flush=True)

    def pump(self, sec=0.05):
        """Read messages for `sec` seconds and update the state."""
        end = time.time() + sec
        while True:
            msg = self.m.recv_match(blocking=True, timeout=0.01)
            if msg is not None:
                self._handle(msg)
            if time.time() >= end:
                return

    def _handle(self, msg):
        st = self.state
        k = msg.get_type()
        now = time.time()
        if k == "HEARTBEAT":
            if msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                st["hb"] = now
                st["mode"] = MODES.get(msg.custom_mode, str(msg.custom_mode))
                st["armed"] = bool(msg.base_mode & 128)
                st["mode_num"] = msg.custom_mode
            elif msg.get_srcSystem() == COMPANION_SYSID:
                st["comp_hb"] = now
        elif k == "STATUSTEXT":
            st["texts"].append(msg.text)
        elif k == "LOCAL_POSITION_NED":
            st["x"], st["y"], st["z"] = msg.x, msg.y, msg.z
        elif k == "ATTITUDE":
            st["yaw"] = msg.yaw
        elif k == "GLOBAL_POSITION_INT":
            st["lat"], st["lon"] = msg.lat / 1e7, msg.lon / 1e7
            st["alt"] = msg.relative_alt / 1000.0
        elif k == "GLOBAL_POSITION_INT_COV":
            pass
        elif k == "HOME_POSITION":
            st["home_lat"], st["home_lon"], st["home_alt"] = msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0
        elif k == "EKF_STATUS_REPORT":
            st["ekf_flags"] = msg.flags
            st["ekf_vs"] = msg.velocity_variance
        elif k == "RANGEFINDER":
            st["rng"] = msg.distance
            st["rng_status"] = getattr(msg, "status", None)
        elif k == "DISTANCE_SENSOR":
            if st["rng"] is None:
                st["rng"] = msg.current_distance / 100.0
        elif k == "OPTICAL_FLOW":
            st["flow_q"] = msg.quality
            st["flow_dist"] = msg.distance
        elif k == "SYS_STATUS":
            if msg.voltage_battery:
                st["volt"] = msg.voltage_battery / 1000.0
            if msg.current_battery:
                st["curr"] = msg.current_battery / 100.0
        elif k == "BATTERY_STATUS":
            if msg.voltages and msg.voltages[0] != 65535:
                st["volt"] = msg.voltages[0] / 1000.0
        elif k == "VFR_HUD":
            st["thr"] = msg.throttle
            st["groundspeed"] = msg.groundspeed
        elif k == "RC_CHANNELS_RAW":
            st["rc_ok"] = msg.chancount > 0
            st["rc_ch"] = msg.chancount
            st["rc_values"] = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]
        elif k == "RC_CHANNELS":
            st["rc_ok"] = msg.chancount > 0
            st["rc_ch"] = msg.chancount
            st["rc_values"] = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]
        elif k == "MISSION_CURRENT":
            st["seq"] = msg.seq
        elif k == "MISSION_ITEM_REACHED":
            st["reached"].append(msg.seq)

    def wait_heartbeat(self, timeout=30.0):
        """Wait for the FC's own HEARTBEAT (sysid 1, comp 1) through _handle, so mode / armed are set on return.
        2026-11-26: mavutil's wait_heartbeat consumed the first heartbeat without _handle and the next one (1 Hz on
        the real FC) often came after the 0.5 s pump, so start saw armed=False and refused (found in a 1x SITL run)."""
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.2)
            if self.state["hb"]:
                self.pump(0.5)
                return True
        return False

    def request_streams(self):
        """Ask for the messages this tool needs (message ids, not stream ids)."""
        for mid, hz in ((1, 2), (24, 2), (30, 10), (32, 10), (33, 5), (42, 2), (65, 2), (74, 5),
                        (173, 20), (193, 2)):
            self.m.mav.command_long_send(1, 1, 511, 0, mid, 1e6 / hz, 0, 0, 0, 0, 0)
        self.pump(0.5)

    # -- parameters ----------------------------------------------------------------------------------------
    def param_get(self, name, tries=4):
        for _ in range(tries):
            self.m.mav.param_request_read_send(1, 1, name.encode(), -1)
            t = time.time()
            while time.time() - t < 1.0:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
                if msg is not None and msg.param_id == name:
                    return msg.param_value, msg.param_type
        return None, None

    def param_set(self, name, value):
        old, ptype = self.param_get(name)
        if old is None:
            return None, None
        if ptype is None:
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        for _ in range(4):
            self.m.mav.param_set_send(1, 1, name.encode(), float(value), ptype)
            got, _ = self.param_get(name)
            if got is not None and abs(got - float(value)) <= max(1e-4, abs(value) * 1e-4):
                return old, got
        return old, self.param_get(name)[0]

    # -- modes ---------------------------------------------------------------------------------------------
    def set_mode(self, mode_num):
        # DO_SET_MODE with param1 = MAV_MODE_FLAG_CUSTOM_MODE (1) and param2 = the CopterMode number
        self.m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode_num, 0, 0, 0, 0, 0)

    # -- mission -------------------------------------------------------------------------------------------
    def mission_read(self, timeout=5.0):
        """Read the whole mission from the FC -> list of dicts."""
        self.m.mav.mission_request_list_send(1, 1, 0)
        cnt = self.m.recv_match(type="MISSION_COUNT", blocking=True, timeout=timeout)
        if cnt is None:
            return None
        items = []
        for seq in range(cnt.count):
            self.m.mav.mission_request_int_send(1, 1, seq, 0)
            it = self.m.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=timeout)
            if it is None:
                break
            items.append({
                "seq": it.seq, "frame": it.frame, "command": it.command,
                "p1": it.param1, "p2": it.param2, "p3": it.param3, "p4": it.param4,
                "lat": it.x / 1e7, "lon": it.y / 1e7, "alt": it.z,
            })
        return items

    def mission_upload(self, items, timeout=20.0):
        """Upload a list of items; returns (ok, message)."""
        self.m.mav.mission_clear_all_send(1, 1, 0)
        self.pump(0.4)
        self.m.mav.mission_count_send(1, 1, len(items), 0)
        t = time.time()
        sent = 0
        seen_request = False
        while time.time() - t < timeout:
            msg = self.m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                    blocking=True, timeout=1.0)
            if msg is None:
                continue
            k = msg.get_type()
            if k == "MISSION_ACK":
                # the ACK of the "clear all" that precedes the upload has the same value, so only
                # take the ACK once the FC has asked for at least one item
                if not seen_request:
                    continue
                if msg.type == 0:
                    return True, "accepted (%d items)" % len(items)
                return False, "MISSION_ACK type %d" % msg.type
            seen_request = True
            seq = msg.seq
            if seq >= len(items):
                continue
            it = items[seq]
            # the .waypoints frame column: 3 = relative to home, 1 = absolute AMSL
            frame = FRAME_GLOBAL_RELATIVE_ALT_INT if it.get("frame", 3) == 3 else 10
            self.m.mav.mission_item_int_send(
                1, 1, seq, frame, it["command"], 0, 1,
                it["p1"], it["p2"], it["p3"], it["p4"],
                int(round(it["lat"] * 1e7)), int(round(it["lon"] * 1e7)), it["alt"], 0)
            sent += 1
        return False, "timeout after %d of %d items" % (sent, len(items))


def compare_mission(want, got, home_tol_m=2.0):
    """Compare the file's items with the FC's. Item 0 is the home slot: the FC replaces it with its
    own home position and altitude, so only the command and the position are checked there.
    MAV_CMD_NAV_RETURN_TO_LAUNCH (20) carries no position, so none is compared.
    Returns (ok, list of problem strings)."""
    problems = []
    if got is None:
        return False, ["the FC did not answer the mission request"]
    if len(want) != len(got):
        problems.append("item count: file %d, FC %d" % (len(want), len(got)))
    lat0 = want[0]["lat"] if want else 0.0
    lon0 = want[0]["lon"] if want else 0.0
    for i, w in enumerate(want):
        if i >= len(got):
            problems.append("item %d missing on the FC" % i)
            continue
        g = got[i]
        if g["command"] != w["command"]:
            problems.append("item %d: command file %d, FC %d" % (i, w["command"], g["command"]))
        north, east = latlon_to_ne(g["lat"], g["lon"], lat0, lon0)
        if i == 0:
            # the home slot: the FC puts its own home there
            d = math.hypot(north, east)
            if d > home_tol_m:
                problems.append("item 0 (home): FC home is %.2f m from the file origin" % d)
            continue
        if abs(g["alt"] - w["alt"]) > 0.02:
            problems.append("item %d: alt file %.2f, FC %.2f" % (i, w["alt"], g["alt"]))
        if w["command"] == 20:
            continue                      # RTL: no position
        if abs(g["p1"] - w["p1"]) > 0.01:
            problems.append("item %d: p1 file %.2f, FC %.2f" % (i, w["p1"], g["p1"]))
        want_north, want_east = latlon_to_ne(w["lat"], w["lon"], lat0, lon0)
        dw = math.hypot(north - want_north, east - want_east)
        if dw > 0.05:
            problems.append("item %d: position differs by %.3f m" % (i, dw))
    return (len(problems) == 0), problems


# --------------------------------------------------------------------------------------------------------------
# sub-commands

def cmd_plan(args):
    m, pts = build_mission(args)
    base = args.out
    wpl_path = base + ".waypoints"
    json_path = base + ".json"
    write_waypoints_file(m, wpl_path)
    with open(json_path, "w") as f:
        json.dump(m.to_json(), f, indent=1)
    print("mission: %d items, altitude %.2f m, hold %.1f s, ends with %s" %
          (len(m.items), m.alt, m.hold, m.end.upper()))
    print("origin (EKF origin set by the companion): %.7f %.7f %.1f m" % m.origin)
    print("heading at arming used: %.1f deg clockwise from north" % m.heading_deg)
    for seq, (cmd, x, y, alt, hold) in enumerate(m.items):
        lat, lon = m.global_xy(x, y)
        print("  item %d %-8s offset x %+.2f y %+.2f m -> %.7f %.7f  alt %.2f m  hold %.1f s" %
              (seq, MAV_CMD.get(cmd, str(cmd)), x, y, lat, lon, alt, hold))
    print("wrote %s" % wpl_path)
    print("wrote %s" % json_path)
    return 0


def cmd_params(args):
    fc = FC(args.url)
    if not fc.wait_heartbeat(timeout=args.timeout):
        print("FAIL: no heartbeat on %s" % args.url)
        return 1
    wanted = dict(WANTED)
    for kv in args.set:
        if "=" not in kv:
            print("FAIL: --set needs NAME=VALUE, got '%s'" % kv)
            return 1
        name, value = kv.split("=", 1)
        wanted[name.strip()] = (float(value), "overridden on the command line")
    print("%-16s %-12s %-12s %-8s %s" % ("PARAMETER", "FC", "WANTED", "STATE", "WHY"))
    print("-" * 110)
    old_values = {}
    for name, (want, why) in wanted.items():
        cur, _ = fc.param_get(name)
        if cur is None:
            state = "MISSING"
            print("%-16s %-12s %-12s %-8s %s" % (name, "absent", want, state, why))
            continue
        old_values[name] = cur
        same = abs(cur - want) <= max(1e-4, abs(want) * 1e-4)
        state = "OK" if same else "DIFF"
        print("%-16s %-12s %-12s %-8s %s" % (name, cur, want, state, why))
    print("-" * 110)
    print("reported only, never written by this tool:")
    for name, (want, why) in REPORT_ONLY.items():
        cur, _ = fc.param_get(name)
        if cur is None:
            print("%-16s %-12s %-12s %-8s %s" % (name, "absent", want, "MISSING", why))
        else:
            state = "OK" if abs(cur - want) <= 1e-4 else "note"
            print("%-16s %-12s %-12s %-8s %s" % (name, cur, want, state, why))

    if not args.apply:
        print("\nnothing written (use --apply --yes to write the mission parameter set)")
        return 0
    if not args.yes:
        print("\nrefusing to write without --yes")
        return 1

    backup = args.out or ("fc_params_before_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(backup, "w") as f:
        json.dump(old_values, f, indent=1)
    print("\nold values saved to %s" % backup)
    print("%-16s %-12s %-12s %s" % ("PARAMETER", "OLD", "NEW", "READ BACK"))
    bad = []
    for name, (want, _why) in wanted.items():
        old, new = fc.param_set(name, want)
        ok = new is not None and abs(new - want) <= max(1e-4, abs(want) * 1e-4)
        print("%-16s %-12s %-12s %s" % (name, old, new, "OK" if ok else "FAILED"))
        if not ok:
            bad.append(name)
    if bad:
        print("\nFAILED to write: %s" % ", ".join(bad))
        return 1
    print("\nall %d parameters written and read back" % len(wanted))
    return 0


def cmd_upload(args):
    items = parse_wpl_file(args.file)
    if not items:
        print("FAIL: no mission items in %s" % args.file)
        return 1
    fc = FC(args.url)
    if not fc.wait_heartbeat(timeout=args.timeout):
        print("FAIL: no heartbeat on %s" % args.url)
        return 1
    print("uploading %d items from %s" % (len(items), args.file))
    ok, msg = fc.mission_upload(items)
    print("upload: %s" % ("OK - " + msg if ok else "FAILED - " + msg))
    if not ok:
        return 1
    got = fc.mission_read()
    same, problems = compare_mission(items, got)
    if got is None:
        print("read back: FAILED - the FC did not answer")
        return 1
    print("read back: %d items" % len(got))
    for i, g in enumerate(got):
        w = items[i] if i < len(items) else None
        name = MAV_CMD.get(g["command"], str(g["command"]))
        extra = ""
        if w is not None and g["command"] != 20:
            north, east = latlon_to_ne(g["lat"], g["lon"], items[0]["lat"], items[0]["lon"])
            extra = "  offset x %+.2f y %+.2f m" % (north, east)
        print("  item %d %-8s alt %+.2f m  hold %.1f s%s" % (i, name, g["alt"], g["p1"], extra))
    if not same:
        print("\nread back MISMATCH:")
        for p in problems:
            print("  " + p)
        return 1
    print("\nread back matches the file item by item")
    return 0


def cmd_preflight(args):
    fc = FC(args.url)
    fails = []
    notes = []

    def check(ok, name, detail=""):
        print("%-4s %-34s %s" % ("PASS" if ok else "FAIL", name, detail))
        if not ok:
            fails.append(name)

    def note(name, detail=""):
        print("%-4s %-34s %s" % ("INFO", name, detail))

    if not fc.wait_heartbeat(timeout=args.timeout):
        print("FAIL  heartbeat from the FC on %s" % args.url)
        return 1
    fc.request_streams()
    fc.pump(2.0)
    st = fc.state

    check(True, "heartbeat from the FC", "on %s" % args.url)
    note("mode", "%s, %s" % (st["mode"], "ARMED" if st["armed"] else "disarmed"))
    note("armed", str(st["armed"]))

    # EKF flags (EKF_Status.h): 0x0001 attitude, 0x0002 velocity, 0x0004 horiz pos,
    # 0x0008 vert pos, 0x0010 horiz pos rel, 0x0020 vert pos rel, 0x0200 gps fix
    flags = st["ekf_flags"]
    check(bool(flags & 0x0004), "EKF horizontal position valid", "flags 0x%04x" % flags)
    check(bool(flags & 0x0008), "EKF vertical position valid", "flags 0x%04x" % flags)
    check(bool(flags & 0x0001), "EKF attitude valid", "flags 0x%04x" % flags)
    # the waypoints are EKF origin + offsets, so the vehicle must start at the origin: the flow position runs away on
    # the ground and moves when the drone is carried after power-up (2026-11-25: 9 m and 63 m before take-off)
    r0 = math.hypot(st["x"], st["y"])
    check(r0 <= ORIGIN_MAX_M, "vehicle at the EKF origin (mission anchor)",
          "%.2f m from the origin (x %+.2f, y %+.2f); limit %.1f m. If not: power-cycle the drone at the take-off "
          "spot and do not move it" % (r0, st["x"], st["y"], ORIGIN_MAX_M))

    rng_max, _ = fc.param_get("RNGFND1_MAX")
    rng_min, _ = fc.param_get("RNGFND1_MIN")
    rng = st["rng"]
    check(rng is not None, "rangefinder reading", "%.3f m" % rng if rng is not None else "no RANGEFINDER message")
    if rng is not None:
        check(rng_min is None or rng >= rng_min - 0.01, "rangefinder plausible (>= RNGFND1_MIN)",
              "%.3f m, min %s" % (rng, rng_min))
        check(rng_max is None or rng <= rng_max, "rangefinder plausible (<= RNGFND1_MAX)",
              "%.3f m, max %s" % (rng, rng_max))
        check(rng <= 4.0, "rangefinder usable for the 2 m mission", "%.3f m" % rng)

    q = st["flow_q"]
    flow_type, _ = fc.param_get("FLOW_TYPE")
    if flow_type == 10:
        # the SITL flow backend feeds the EKF directly and does not forward the message
        note("optical flow", "FLOW_TYPE 10 (SITL): the backend does not forward OPTICAL_FLOW")
    else:
        check(q is not None and q > 0, "optical flow present", "quality %s" % q)
    flow_use, _ = fc.param_get("EK3_FLOW_USE")
    check(flow_use is not None and flow_use > 0, "EKF fuses the optical flow", "EK3_FLOW_USE %s" % flow_use)

    volt = st["volt"]
    lowv, _ = fc.param_get("BATT_LOW_VOLT")
    check(volt is not None and volt > 3.0, "battery voltage", "%.2f V" % volt if volt else "no SYS_STATUS")
    if volt is not None and lowv is not None:
        check(volt > lowv, "battery above BATT_LOW_VOLT", "%.2f V > %.2f V" % (volt, lowv))

    check(st["rc_ok"], "RC input present", "%d channels, first four raw %s" %
          (st["rc_ch"], st["rc_values"]))
    rc7, _ = fc.param_get("RC7_OPTION")
    note("arm switch", "RC7_OPTION %s (153 = arm)" % rc7)

    age = time.time() - st["comp_hb"] if st["comp_hb"] else 1e9
    check(age < 5.0, "companion heartbeat (sysid %d)" % COMPANION_SYSID,
          "%.1f s ago" % age if st["comp_hb"] else "none seen")

    # RTL and failsafe parameters
    for name in ("RTL_ALT_M", "RTL_SPEED_MS", "RTL_LOIT_TIME", "FS_THR_ENABLE", "FS_GCS_ENABLE",
                 "FS_GCS_TIMEOUT", "FS_OPTIONS", "WP_SPD", "WP_RADIUS_M", "AUTO_OPTIONS"):
        cur, _ = fc.param_get(name)
        want = WANTED.get(name, (None, ""))[0]
        if want is None:
            note("%s" % name, "FC %s" % cur)
            continue
        ok = cur is not None and abs(cur - want) <= max(1e-4, abs(want) * 1e-4)
        check(ok, "%s" % name, "FC %s, wanted %s" % (cur, want))

    # the mission on the FC
    if args.file:
        want = parse_wpl_file(args.file)
        got = fc.mission_read()
        if got is None:
            check(False, "mission on the FC", "the FC did not answer")
        else:
            same, problems = compare_mission(want, got)
            check(same, "mission matches %s" % os.path.basename(args.file),
                  "%d items" % len(got) if same else "; ".join(problems[:4]))
    else:
        got = fc.mission_read()
        note("mission on the FC", "%d items" % len(got) if got is not None else "no answer")

    # PreArm checks
    print("... requesting the PreArm checks")
    fc.state["texts"] = []
    fc.m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, 0)
    fc.pump(4.0)
    texts = [t for t in fc.state["texts"] if t.strip()]
    bad = [t for t in texts if t.lower().startswith(("prearm", "arm: ", "arming")) or "prearm" in t.lower()]
    check(len(bad) == 0, "PreArm checks", "no PreArm error" if not bad else "; ".join(bad[:4]))
    for t in texts:
        print("     FC: %s" % t)

    print("\n%d checks failed" % len(fails))
    for f in fails:
        print("  FAIL %s" % f)
    return 1 if fails else 0


def cmd_start(args):
    fc = FC(args.url)
    if not fc.wait_heartbeat(timeout=args.timeout):
        print("FAIL: no heartbeat on %s" % args.url)
        return 1
    fc.pump(0.3)
    st = fc.state
    if not st["armed"]:
        print("refusing: the vehicle is not armed. The pilot arms it with the RC in LOITER "
              "(the flight manual); this tool never arms.")
        return 1
    if st["mode"] != "LOITER":
        print("refusing: the vehicle is in %s, not LOITER. Put the mode switch to LOITER and arm "
              "with the RC first." % st["mode"])
        return 1
    r0 = math.hypot(st["x"], st["y"])
    if r0 > ORIGIN_MAX_M and not args.force_origin:
        print("refusing: the vehicle is %.2f m from the EKF origin (x %+.2f, y %+.2f) and the waypoints are anchored "
              "there, so AUTO would first fly back to it. Disarm, power-cycle at the take-off spot, do not move it, "
              "and arm again (--force-origin overrides)." % (r0, st["x"], st["y"]))
        return 1
    print("vehicle is ARMED in LOITER, %.2f m from the EKF origin, switching to AUTO" % r0)
    fc.set_mode(3)
    # the pilot is waiting with the throttle at the minimum: ArduPilot auto-disarms after
    # DISARM_DELAY (10 s) if the throttle stays there on the ground, so this has to be quick
    fc.pump(0.4)
    if fc.state["mode"] != "AUTO":
        print("FAIL: the FC is in %s, not AUTO" % fc.state["mode"])
        return 1
    print("the FC is in AUTO, the mission is running")
    return 0


def cmd_monitor(args):
    fc = FC(args.url)
    if not fc.wait_heartbeat(timeout=args.timeout):
        print("FAIL: no heartbeat on %s" % args.url)
        return 1
    fc.request_streams()

    mission = None
    origin = (0.0, 0.0)
    if args.file:
        items = parse_wpl_file(args.file)
        if items:
            origin = (items[0]["lat"], items[0]["lon"])
            mission = items
    else:
        got = fc.mission_read()
        if got:
            origin = (got[0]["lat"], got[0]["lon"])
            mission = got

    csvf = None
    writer = None
    if args.csv:
        csvf = open(args.csv, "w", newline="")
        writer = csv.writer(csvf)
        writer.writerow(["t_s", "mode", "armed", "mission_item", "dist_next_m", "alt_m", "batt_v", "text"])
        csvf.flush()

    print("%7s %-9s %-8s %5s %8s %8s %7s" % ("t", "mode", "armed", "item", "dist", "alt", "batt"))
    t0 = time.time()
    last_texts = 0
    was_armed = fc.state["armed"]
    while True:
        fc.pump(0.2)
        st = fc.state
        now = time.time() - t0
        if st["hb"] and time.time() - st["hb"] > 5.0:
            print("the FC heartbeat is gone, the link is dead")
            break
        # one line per second
        if now - fc._last_print >= 1.0:
            fc._last_print = now
            dist = None
            if mission and st["lat"] is not None:
                nxt = None
                for it in mission:
                    if it["command"] in (16, 20, 21, 22) and it["seq"] >= st["seq"]:
                        nxt = it
                        break
                if nxt is not None:
                    north, east = latlon_to_ne(st["lat"], st["lon"], origin[0], origin[1])
                    nn, ee = latlon_to_ne(nxt["lat"], nxt["lon"], origin[0], origin[1])
                    dist = math.hypot(nn - north, ee - east)
            alt = st["alt"] if st["alt"] is not None else float("nan")
            print("%7.1f %-9s %-8s %5d %8s %8.2f %7.2f" % (
                now, st["mode"], "ARMED" if st["armed"] else "disarmed", st["seq"],
                ("%.2f" % dist) if dist is not None else "-", alt, st["volt"] or 0.0))
            if writer is not None:
                writer.writerow([round(now, 2), st["mode"], int(st["armed"]), st["seq"],
                                 round(dist, 3) if dist is not None else "",
                                 round(alt, 3) if alt == alt else "", st["volt"] or ""])
                csvf.flush()
        # every STATUSTEXT
        for t in fc.state["texts"][last_texts:]:
            print("  FC: %s" % t)
            if writer is not None:
                writer.writerow([round(now, 2), "text", "", "", "", "", "", t])
                csvf.flush()
        last_texts = len(fc.state["texts"])
        if was_armed and not st["armed"]:
            print("%7.1f the vehicle disarmed, the flight is over" % (time.time() - t0))
            break
        was_armed = st["armed"]
        if time.time() - t0 > args.seconds:
            print("timeout after %.0f s" % args.seconds)
            break
    if csvf is not None:
        csvf.close()
    print("reached items: %s" % fc.state["reached"])
    return 0


# --------------------------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--url", default="udpin:127.0.0.1:14570",
                       help="Pi: the mavlink-router diagnostics tap (default). A SITL tcp: URL works too.")
        p.add_argument("--timeout", type=float, default=30.0, help="heartbeat timeout, s")

    p = sub.add_parser("plan", help="build the mission files")
    common(p)
    p.add_argument("--alt", type=float, default=2.0, help="take-off and waypoint altitude, m")
    p.add_argument("--side", type=float, default=1.5, help="the side of the square, m (0 = no square)")
    p.add_argument("--wp", default="", help="explicit waypoint offsets 'x,y;x,y;...' in metres")
    p.add_argument("--ccw", action="store_true", help="square the other way round")
    p.add_argument("--hold", type=float, default=2.0, help="hold time at each waypoint, s")
    p.add_argument("--end", choices=["rtl", "land"], default="rtl", help="the last mission item")
    p.add_argument("--origin", type=parse_origin, default=DEFAULT_ORIGIN, help="lat,lon,alt of the EKF origin")
    p.add_argument("--heading-deg", type=float, default=0.0,
                   help="the vehicle's heading at arming, clockwise from north (0 = the square is north aligned)")
    p.add_argument("--out", default="course_mission", help="output file base name")
    p.set_defaults(func=cmd_plan)

    p = sub.add_parser("params", help="compare / write the field parameter set")
    common(p)
    p.add_argument("--apply", action="store_true", help="write the wanted values to the FC")
    p.add_argument("--yes", action="store_true", help="confirm --apply")
    p.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                   help="extra / overriding parameter to write with --apply")
    p.add_argument("--out", default="", help="where to save the old values (default: a timestamped json)")
    p.set_defaults(func=cmd_params)

    p = sub.add_parser("upload", help="upload the mission and read it back")
    common(p)
    p.add_argument("--file", required=True, help="the .waypoints file")
    p.set_defaults(func=cmd_upload)

    p = sub.add_parser("preflight", help="read-only checks")
    common(p)
    p.add_argument("--file", default="", help="a .waypoints file to compare the FC's mission with")
    p.set_defaults(func=cmd_preflight)

    p = sub.add_parser("start", help="switch LOITER -> AUTO (never arms)")
    common(p)
    p.add_argument("--force-origin", action="store_true",
                   help="start even if the vehicle is more than %.1f m from the EKF origin" % ORIGIN_MAX_M)
    p.set_defaults(func=cmd_start)

    p = sub.add_parser("monitor", help="watch the flight")
    common(p)
    p.add_argument("--file", default="", help="the .waypoints file (for the distance to the next waypoint)")
    p.add_argument("--csv", default="", help="write a CSV here")
    p.add_argument("--seconds", type=float, default=600.0, help="give up after this long, s")
    p.set_defaults(func=cmd_monitor)

    args = ap.parse_args()
    if not hasattr(args, "func"):
        ap.print_help()
        return 1
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
