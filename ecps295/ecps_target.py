"""Target recognition and tracking (2026-10-18): fly to an ArUco marker and hover in front of it, facing it.

MarkerDetector   OpenCV ArUco on the full-resolution camera frame (the brain keeps its 192x144 gray view), pose of the
                 marker by solvePnP (IPPE_SQUARE, known side length) with a pinhole model from the horizontal FOV (or a
                 calibrated K / distortion); the camera is pitched nose-down by pitch_deg. Result in the body frame:
                 horizontal range, bearing (deg, clockwise like yaw), height of the marker below the camera.
MarkerThread     runs the detector on its own thread at hz on the given CPUs (A55 on the Orange Pi; the brain owns the
                 A76 cores), on the newest frame of a grab() callable -> (image, monotonic stamp).
TargetTrack      the layer EcpsPilot runs before its waypoint layer (ecps_pilot.py `target:`). Started by the ground
                 console (TRACK) or a test; it never takes off, lands or leaves the safety stack:
  SEARCH    no fix yet: hover and yaw slowly on the spot for search_s; after a full turn without a fix, fly relocate_m
            toward the take-off point (fence centre) and turn again, up to relocate_max times (each restarts search_s):
            started too close to the marker, it can stay out of the camera's vertical field of view (Gazebo trk2 cage10)
  APPROACH  fix: the marker's position in the map frame (EKF local NED, the pose at the frame's time) is filtered; the
            goal is standoff_m short of it on the line of sight, clipped inside the geofence, and is flown by the
            waypoint layer (goal bias on the brain, then fence turn and range shield); a moved marker moves the goal
  ARRIVED   at the goal (waypoint reached or stuck, or the measured range <= standoff_m + arrive_tol_m): hover and yaw
            to face the marker; a marker that moves more than reapproach_m starts APPROACH again
  LOST      no detection for lost_s: fly on to the last goal, then SEARCH again (also when it vanished while ARRIVED)
  FAILED    nothing found within search_s: hover (the console / pilot decides)
Events go to pilot.watchdog_events as "target: ..." (forwarded as STATUSTEXT by the companion).

render_marker() draws a marker into a camera image from a pose (pinhole, same model as the detector): SITL camera
source and offline tests only.
"""

from __future__ import annotations

import collections
import math
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from flydrones.motor import FlightCommand

DICTS = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "6x6_50": cv2.aruco.DICT_6X6_50,
    "apriltag_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def camera_matrix(width: int, height: int, hfov_deg: float) -> np.ndarray:
    f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])


@dataclass
class Detection:
    marker_id: int
    range_m: float  # horizontal distance camera -> marker centre
    bearing_deg: float  # clockwise from the nose
    below_m: float  # marker centre below the camera
    facing_deg: float  # marker normal vs the line of sight (0 = seen square-on)
    px: float  # side length in pixels (size in the image)
    stamp: float  # monotonic time of the frame


class MarkerDetector:
    def __init__(self, dictionary: str = "4x4_50", marker_id: int | None = None, size_m: float = 0.15,
                 hfov_deg: float = 70.0, pitch_deg: float = 12.0, K=None, dist=None, min_px: float = 12.0, **_):
        self.dict = cv2.aruco.getPredefinedDictionary(DICTS[dictionary])
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(self.dict, params)
        self.marker_id = None if marker_id is None else int(marker_id)
        self.size = float(size_m)
        self.hfov = float(hfov_deg)
        self.pitch = math.radians(float(pitch_deg))  # nose-down mount
        self.K0 = None if K is None else np.asarray(K, float)
        self.dist = np.zeros(5) if dist is None else np.asarray(dist, float)
        self.min_px = float(min_px)
        self._K: dict[tuple[int, int], np.ndarray] = {}
        h = self.size / 2.0
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)  # ArUco corner order

    def K(self, w: int, h: int) -> np.ndarray:
        if self.K0 is not None:
            return self.K0
        if (w, h) not in self._K:
            self._K[(w, h)] = camera_matrix(w, h, self.hfov)
        return self._K[(w, h)]

    def detect(self, img: np.ndarray, stamp: float = 0.0) -> list[Detection]:
        gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.det.detectMarkers(gray)
        out: list[Detection] = []
        if ids is None:
            return out
        K = self.K(gray.shape[1], gray.shape[0])
        for c, i in zip(corners, ids.ravel()):
            if self.marker_id is not None and int(i) != self.marker_id:
                continue
            c = c.reshape(4, 2)
            px = float(np.mean(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)))
            if px < self.min_px:
                continue
            ok, rvec, tvec = cv2.solvePnP(self.obj, c.astype(np.float32), K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            xc, yc, zc = (float(v) for v in tvec.ravel())  # camera: x right, y down, z forward
            cp, sp = math.cos(self.pitch), math.sin(self.pitch)
            fwd, right, down = zc * cp - yc * sp, xc, zc * sp + yc * cp  # body frame (pitch undone)
            R = cv2.Rodrigues(rvec)[0]
            n_cam = R[:, 2]  # marker normal (out of the printed face is -z of the marker frame, toward the camera)
            los = np.array([xc, yc, zc]) / max(1e-6, math.sqrt(xc * xc + yc * yc + zc * zc))
            facing = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(n_cam @ los))))))
            out.append(Detection(int(i), math.hypot(fwd, right), math.degrees(math.atan2(right, fwd)), down, facing, px, stamp))
        out.sort(key=lambda d: -d.px)
        return out


class MarkerThread:
    """Detector on its own thread: latest() -> (Detection or None, stamp of the frame it came from)."""

    def __init__(self, grab, detector: MarkerDetector, hz: float = 10.0, cpus: str = ""):
        self.grab, self.detector, self.hz, self.cpus = grab, detector, float(hz), cpus
        self.det: Detection | None = None
        self.stamp = 0.0  # frame stamp of the last processed frame (with or without a detection)
        self.frames = 0
        self.hits = 0
        self.errors = 0
        self.det_ms: list[float] = []
        self.status = "init"
        self._last_frame = 0.0
        self._stop = threading.Event()
        self._th: threading.Thread | None = None

    def latest(self):
        return self.det, self.stamp

    def start(self) -> "MarkerThread":
        self._th = threading.Thread(target=self._run, name="marker", daemon=True)
        self._th.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=2)

    def _run(self) -> None:
        if self.cpus and hasattr(os, "sched_setaffinity"):  # this thread only (Linux: pid 0 = the calling thread)
            from_spec = set()
            for part in self.cpus.split(","):
                lo, _, hi = part.partition("-")
                from_spec.update(range(int(lo), int(hi or lo) + 1))
            try:
                os.sched_setaffinity(0, from_spec)
            except OSError as e:
                self.status = f"affinity {self.cpus}: {e}"
        cv2.setNumThreads(1)
        nxt = time.monotonic()
        self.status = "running"
        while not self._stop.is_set():
            nxt += 1.0 / self.hz
            self._stop.wait(max(0.0, nxt - time.monotonic()))
            nxt = max(nxt, time.monotonic() - 1.0 / self.hz)  # no burst after a stall
            try:
                img, stamp = self.grab()
            except Exception:  # noqa: BLE001 - a camera hiccup: count, keep going
                self.errors += 1
                continue
            if img is None or stamp <= self._last_frame:
                continue
            self._last_frame = stamp
            c0 = time.perf_counter()
            try:
                dets = self.detector.detect(img, stamp)
            except cv2.error:
                self.errors += 1
                continue
            if len(self.det_ms) < 50000:
                self.det_ms.append((time.perf_counter() - c0) * 1000)
            self.frames += 1
            if dets:
                self.hits += 1
                self.det = dets[0]
            self.stamp = stamp


class TargetTrack:
    def __init__(self, cfg: dict | None = None):
        c = cfg or {}
        self.standoff = float(c.get("standoff_m", 1.0))
        self.arrive_tol = float(c.get("arrive_tol_m", 0.1))
        self.goal_radius = float(c.get("goal_radius_m", 0.15))  # waypoint arrival radius while approaching the goal
        self.reapproach = float(c.get("reapproach_m", 0.4))
        self.goal_move = float(c.get("goal_update_m", 0.15))
        self.fence_margin = float(c.get("fence_margin_m", 0.4))
        self.search_s = float(c.get("search_s", 30.0))
        self.search_yaw = float(c.get("search_yaw", 0.3))  # yaw command while searching (fraction of max)
        self.lost_s = float(c.get("lost_s", 2.0))
        self.max_age = float(c.get("max_age_s", 0.5))  # a detection older than this is not a fix
        self.alpha = float(c.get("filter_alpha", 0.3))  # EMA weight of a new fix
        self.jump = float(c.get("jump_m", 0.6))  # a fix this far from the estimate needs confirm_n in a row
        self.confirm_n = int(c.get("confirm_n", 3))
        self.face_gain = float(c.get("face_gain", 1.0))
        self.min_range = float(c.get("min_range_m", 0.3))
        self.max_range = float(c.get("max_range_m", 5.0))
        # 2026-10-19 (Gazebo trk1: 13 of 39 runs lost to brain escapes): the brain's looming escape fires on the marker
        # itself at the standoff. With the marker fresh in view within escape_trust_deg of the nose and no closer than
        # standoff - 0.3 m, APPROACH / ARRIVED ignore brain escapes (the RangeShield after this layer still clamps on
        # the ToF and the map); 0 = escapes always win, as before. SEARCH turns on through escapes (hovering in place).
        self.escape_trust = float(c.get("escape_trust_deg", 30.0))
        self.search_through_escape = bool(c.get("search_through_escape", True))
        self.hold_gain = float(c.get("hold_gain", 1.0))  # ARRIVED: forward per metre of range error (position hold)
        self.hold_max = float(c.get("hold_max", 0.3))
        self.cruise = float(c.get("cruise", 0.5))  # forward handed to the waypoint layer when an escape is ignored
        self.escapes_ignored = 0
        # 2026-10-20 (Gazebo trk2 cage10_k2: started 0.6 m from the marker and 0.3 m above it, a full turn never had it
        # in the frame): after turn_deg of search without a fix, relocate toward the fence centre and search again
        self.turn_deg = float(c.get("search_turn_deg", 360.0))
        self.relocate = float(c.get("relocate_m", 0.8))
        self.relocate_min_r = float(c.get("relocate_min_r_m", 0.4))  # nearer the centre than this: no point moving
        self.relocate_max = int(c.get("relocate_max", 2))
        self.relocate_s = float(c.get("relocate_s", 12.0))  # give up a relocation leg after this (stuck)
        self._turned = 0.0
        self._last_yaw: float | None = None
        self._reloc_t: float | None = None  # relocation leg in progress since
        self.relocations = 0
        self.source = None  # MarkerThread (or anything with latest() -> (Detection | None, stamp))
        self.state = "OFF"
        self.t_state = 0.0
        self.est: tuple[float, float] | None = None  # filtered marker position (north, east)
        self.est_z: float | None = None  # marker height above the take-off point
        self.goal: tuple[float, float] | None = None
        self.est_arrived: tuple[float, float] | None = None  # marker estimate when we arrived (re-approach reference)
        self._wp_radius: float | None = None  # the waypoint layer's own radius, restored when tracking ends
        self.last_fix_t = -1e9
        self.fixes = 0
        self.last_det: Detection | None = None
        self._used_stamp = 0.0
        self._jump_n = 0
        self._poses: collections.deque = collections.deque(maxlen=60)  # (monotonic, x, y, yaw_deg, alt)
        self.log: list[tuple[float, str]] = []
        self.on_event = None  # EcpsPilot._event: state changes go to the watchdog event log

    # ------------------------------------------------------------------ control from the console / tests
    def start(self, t: float = 0.0) -> None:
        self.est = self.est_z = self.goal = self.est_arrived = None
        self.fixes, self._jump_n, self.last_fix_t = 0, 0, -1e9
        self._turned, self._last_yaw, self._reloc_t, self.relocations = 0.0, None, None, 0
        self._set("SEARCH", t)

    def stop(self, t: float = 0.0, pilot=None) -> None:
        self._set("OFF", t)
        if pilot is not None and self._wp_radius is not None:
            pilot.wp_radius = self._wp_radius
        self._wp_radius = None

    @property
    def active(self) -> bool:
        return self.state != "OFF"

    def _set(self, state: str, t: float, why: str = "") -> None:
        if state != self.state:
            if state == "ARRIVED":
                self.est_arrived = self.est
            self._turned, self._last_yaw, self._reloc_t = 0.0, None, None  # a new search starts with a full turn
            self.state, self.t_state = state, t
            what = f"target: {state.lower()}" + (f" ({why})" if why else "")
            self.log.append((round(t, 2), what))
            if self.on_event is not None:
                self.on_event(t, what)

    # ------------------------------------------------------------------ fixes
    def _pose_at(self, stamp: float):
        best = None
        for p in self._poses:
            if best is None or abs(p[0] - stamp) < abs(best[0] - stamp):
                best = p
        return best

    def _fix(self, now_mono: float) -> bool:
        """Fold a new detection into the estimate; True if a fresh fix was used."""
        if self.source is None:
            return False
        det, stamp = self.source.latest()
        if det is None or det.stamp <= self._used_stamp or now_mono - det.stamp > self.max_age:
            return False
        self._used_stamp = det.stamp
        if not self.min_range <= det.range_m <= self.max_range:
            return False
        p = self._pose_at(det.stamp)
        if p is None:
            return False
        _, x, y, yaw, alt = p
        a = math.radians(yaw + det.bearing_deg)
        n, e = x + det.range_m * math.cos(a), y + det.range_m * math.sin(a)
        self.last_det = det
        if self.est is not None and math.hypot(n - self.est[0], e - self.est[1]) > self.jump:
            self._jump_n += 1
            if self._jump_n < self.confirm_n:
                return False
            self.est = None  # the marker really moved (or the first fix was wrong): restart the filter
        self._jump_n = 0
        if self.est is None:
            self.est = (n, e)
        else:
            k = self.alpha
            self.est = (self.est[0] + k * (n - self.est[0]), self.est[1] + k * (e - self.est[1]))
        if alt is not None:
            self.est_z = alt - det.below_m
        self.fixes += 1
        return True

    def _goal_for(self, x: float, y: float, fence: float) -> tuple[float, float]:
        n, e = self.est
        dn, de = n - x, e - y
        d = math.hypot(dn, de)
        if d > self.standoff:
            gn, ge = n - self.standoff * dn / d, e - self.standoff * de / d
        else:  # already inside the standoff: stay where we are (the shield keeps the gap)
            gn, ge = x, y
        r, rmax = math.hypot(gn, ge), fence - self.fence_margin
        if r > rmax > 0:
            gn, ge = gn * rmax / r, ge * rmax / r
        return gn, ge

    def _search_turn(self, pilot, tel, t: float) -> None:
        """SEARCH bookkeeping: count the turn; after a full one without a fix, relocate toward the fence centre."""
        x, y = tel.x_m, tel.y_m
        if self._reloc_t is not None:  # relocation leg: done when reached / stuck / timed out, then turn again
            if pilot.wp_done_t is not None or not pilot.wp_on or t - self._reloc_t > self.relocate_s:
                pilot.wp_on = False
                self._reloc_t, self._turned, self._last_yaw = None, 0.0, None
                self.t_state = t  # a fresh search_s for the new spot
                self._note(t, f"target: search again at {x:+.2f}, {y:+.2f}")
            return
        if self._last_yaw is not None:
            self._turned += abs((tel.yaw_deg - self._last_yaw + 180) % 360 - 180)
        self._last_yaw = tel.yaw_deg
        r = math.hypot(x, y)
        if self._turned < self.turn_deg or self.relocations >= self.relocate_max or r < self.relocate_min_r:
            return
        k = max(0.0, r - self.relocate) / r
        self.relocations += 1
        self._reloc_t = t
        pilot.set_waypoints([(x * k, y * k)])
        self._note(t, f"target: search: nothing in a full turn -> moving to {x * k:+.2f}, {y * k:+.2f}")

    def _note(self, t: float, what: str) -> None:
        self.log.append((round(t, 2), what))
        if self.on_event is not None:
            self.on_event(t, what)

    # ------------------------------------------------------------------ the layer
    def update(self, pilot, cmd: FlightCommand, tel, t: float) -> FlightCommand:
        if self.state == "OFF":
            return cmd
        now = tel.t if getattr(tel, "t", None) is not None else time.monotonic()
        self._poses.append((now, tel.x_m, tel.y_m, tel.yaw_deg, tel.alt_m))
        if self._wp_radius is None:  # approach the goal closer than a mission point: the goal already holds the standoff
            self._wp_radius, pilot.wp_radius = pilot.wp_radius, self.goal_radius
        if not pilot.wp_active:  # not under our control (pilot flying, console HOLD / MANUAL): timers wait
            self.t_state = t if self.state in ("SEARCH", "LOST") else self.t_state
            self._last_yaw = None
            if self._reloc_t is not None:
                self._reloc_t = t
            return cmd
        fresh = self._fix(now)
        if fresh:
            self.last_fix_t = t
        seen = t - self.last_fix_t < self.lost_s
        x, y = tel.x_m, tel.y_m
        fence = pilot.safety.fence

        if self.state in ("SEARCH", "LOST", "FAILED") and fresh and self.est is not None:
            self._set("APPROACH", t, f"marker {self.last_det.marker_id} at {self.est[0]:+.2f}, {self.est[1]:+.2f}"
                                     f", {self.last_det.range_m:.2f} m")
            self.goal = None
        if self.state == "APPROACH":
            if fresh:
                g = self._goal_for(x, y, fence)
                if self.goal is None:
                    self.goal = g
                    pilot.set_waypoints([g])
                elif math.hypot(g[0] - self.goal[0], g[1] - self.goal[1]) > self.goal_move:
                    self.goal = g
                    pilot.wp_points = [g]  # same mission, moved goal: keep the stuck timer running
                    pilot.wp_idx, pilot.wp_done_t = 0, None
            close = seen and self.last_det is not None and self.last_det.range_m <= self.standoff + self.arrive_tol
            if close or pilot.wp_done_t is not None:
                if not seen:  # reached the last goal without seeing it again
                    self._set("SEARCH", t, "not seen at the last fix")
                else:
                    self._set("ARRIVED", t, f"range {self.last_det.range_m:.2f} m")
                pilot.wp_on = False
            elif not seen and t - self.last_fix_t > self.lost_s and self.state == "APPROACH":
                self._set("LOST", t, "flying on to the last goal")
        elif self.state == "LOST":
            if pilot.wp_done_t is not None or not pilot.wp_on:
                pilot.wp_on = False
                self._set("SEARCH", t, "last goal reached")
        elif self.state == "ARRIVED":
            ref = self.est_arrived or self.est
            if fresh and math.hypot(self.est[0] - ref[0], self.est[1] - ref[1]) > self.reapproach:
                self.goal = self._goal_for(x, y, fence)
                pilot.set_waypoints([self.goal])
                self._set("APPROACH", t, "marker moved")
            elif t - self.last_fix_t > 3 * self.lost_s:
                self._set("SEARCH", t, "out of view")

        if self.state == "SEARCH":
            self._search_turn(pilot, tel, t)
        if self.state in ("SEARCH", "FAILED") and pilot.wp_on and self._reloc_t is None:
            pilot.wp_on = False
        if self.state == "SEARCH" and self._reloc_t is None and t - self.t_state > self.search_s:
            self._set("FAILED", t, f"nothing in {self.search_s:.0f} s -> hover")

        # ------------------------------------------------------------------ command shaping
        d = self.last_det
        ahead = (self.escape_trust > 0 and d is not None and t - self.last_fix_t < self.max_age
                 and abs(d.bearing_deg) < self.escape_trust and d.range_m >= self.standoff - 0.3)
        if cmd.escape and self.state in ("APPROACH", "ARRIVED") and ahead:
            self.escapes_ignored += 1  # the looming thing is the marker we are flying to
            cmd = FlightCommand(throttle=cmd.throttle, yaw=0.0, forward=self.cruise, note="target: escape ignored")
        if self.state == "SEARCH" and self._reloc_t is not None:
            return cmd  # relocating: the waypoint layer flies toward the centre (escapes win)
        if self.state == "SEARCH" and (not cmd.escape or self.search_through_escape):
            return FlightCommand(throttle=cmd.throttle, yaw=self.search_yaw, forward=min(cmd.forward, 0.0), note="target search")
        if cmd.escape:
            return cmd
        if self.state == "FAILED":
            return FlightCommand(throttle=cmd.throttle, yaw=0.0, forward=min(cmd.forward, 0.0), note="target not found")
        if self.state == "ARRIVED" and self.est is not None:
            b = math.atan2(self.est[1] - y, self.est[0] - x)
            err = math.atan2(math.sin(b - math.radians(tel.yaw_deg)), math.cos(b - math.radians(tel.yaw_deg)))
            yaw = max(-1.0, min(1.0, self.face_gain * err / math.radians(60)))
            seen = d is not None and t - self.last_fix_t < self.lost_s
            rng = d.range_m if seen else math.hypot(self.est[0] - x, self.est[1] - y)
            # hold the goal's range to the marker: the standoff, or more where the goal was pulled inside the fence
            want = max(self.standoff, math.hypot(self.est[0] - self.goal[0], self.est[1] - self.goal[1])) if self.goal else self.standoff
            fwd = max(-self.hold_max, min(self.hold_max, self.hold_gain * (rng - want)))
            return FlightCommand(throttle=cmd.throttle, yaw=yaw, forward=fwd, note="target: holding")
        return cmd  # APPROACH / LOST: the waypoint layer steers to self.goal


def marker_image(dictionary: str, marker_id: int, px: int = 200, border_frac: float = 0.25) -> np.ndarray:
    """Printed marker with a white quiet zone (border_frac of the marker side on each side), uint8 gray."""
    m = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(DICTS[dictionary]), int(marker_id), px)
    b = int(round(px * border_frac))
    return cv2.copyMakeBorder(m, b, b, b, b, cv2.BORDER_CONSTANT, value=255)


def render_marker(img: np.ndarray, K: np.ndarray, cam_ned: tuple[float, float, float], yaw_deg: float, pitch_deg: float,
                  marker_ned: tuple[float, float, float], marker_yaw_deg: float, size_m: float, tex: np.ndarray,
                  border_frac: float = 0.25) -> bool:
    """Draw the marker (centre marker_ned = north, east, up; its printed face looks toward marker_yaw_deg) into img
    (in place) for a camera at cam_ned (north, east, up) with heading yaw_deg, pitched nose-down by pitch_deg. True if
    all four corners of the quiet zone are in front of the camera."""
    s = size_m * (1 + 2 * border_frac) / 2.0
    my = math.radians(marker_yaw_deg)
    right = np.array([-math.sin(my), math.cos(my), 0.0])  # seen from the front: the face's right is to the viewer's left
    right = -right
    up = np.array([0.0, 0.0, 1.0])
    c = np.array(marker_ned, float)
    corners = [c - s * right + s * up, c + s * right + s * up, c + s * right - s * up, c - s * right - s * up]
    yw, p = math.radians(yaw_deg), math.radians(pitch_deg)
    fwd_h = np.array([math.cos(yw), math.sin(yw), 0.0])
    rgt = np.array([-math.sin(yw), math.cos(yw), 0.0])
    fwd = math.cos(p) * fwd_h - math.sin(p) * up  # pitched down
    dwn = -(math.sin(p) * fwd_h + math.cos(p) * up)
    pts = []
    for w in corners:
        v = w - np.array(cam_ned, float)
        zc = float(v @ fwd)
        if zc < 0.05:
            return False
        pts.append([K[0, 0] * float(v @ rgt) / zc + K[0, 2], K[1, 1] * float(v @ dwn) / zc + K[1, 2]])
    h, w_ = tex.shape[:2]
    src = np.array([[0, 0], [w_, 0], [w_, h], [0, h]], np.float32)
    H = cv2.getPerspectiveTransform(src, np.array(pts, np.float32))
    warped = cv2.warpPerspective(tex, H, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR, borderValue=0)
    mask = cv2.warpPerspective(np.full((h, w_), 255, np.uint8), H, (img.shape[1], img.shape[0]), flags=cv2.INTER_NEAREST)
    if img.ndim == 3:
        warped, mask = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR), mask[..., None]
    np.copyto(img, warped, where=mask > 0)
    return True
