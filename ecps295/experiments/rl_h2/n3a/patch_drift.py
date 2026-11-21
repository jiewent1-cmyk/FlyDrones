"""N4 drift-aware range shield (2026-10-15), default off (old configs bit-identical). Run from the ecps295 tree root.
ecps_struct.RangeShield config `drift: {enabled, tau_s, resp_s, win_s, max_mps}`: the barrier filter only sees the commanded
velocity, so a drift the commands do not contain (wind, flow scale error, GUIDED steady-state error) carries the drone
into an obstacle while the command looks safe. Estimate it in the (EKF) estimate frame as measured velocity (position
difference over win_s) minus the modelled response to the commands actually sent (first-order lag resp_s), low-passed
(tau_s) and capped (max_mps); each constraint v.u <= alpha (d - d_safe) is tightened by max(0, w.u) (drift away from an
obstacle never loosens it). Backup ecps_struct.py.pre_drift."""
import shutil
from pathlib import Path

p = Path("ecps_struct.py")
s = p.read_text()
assert "drift_on" not in s, "already patched"
R = [
("""        self.map_ticks = 0
        self.rear_blind =""", """        self.map_ticks = 0
        dr = cfg.get("drift", {}) or {}
        self.drift_on = bool(dr.get("enabled", False))
        self.drift_tau, self.drift_resp = float(dr.get("tau_s", 1.0)), float(dr.get("resp_s", 0.5))
        self.drift_win, self.drift_max = float(dr.get("win_s", 0.5)), float(dr.get("max_mps", 0.15))
        self._dr_hist: list = []  # (t, x, y) estimate positions over win_s
        self._vmod = (0.0, 0.0)  # modelled response to the commands sent, estimate frame
        self.w_hat = (0.0, 0.0)  # drift estimate, estimate frame
        self._cmd_sent = (0.0, 0.0)  # last command after the filter, body frame (m/s)
        self.drift_ticks = 0
        self.rear_blind ="""),
("""        retreat = False
        for _ in range(3):  # cyclic projection""", """        if self.drift_on and tel_ok:
            wbx, wby = self._drift(tel, t, dt)
            tight = [max(0.0, wbx * ux + wby * uy) for (ux, uy), _ in cons]
            if any(w > 1e-6 and vx * ux + vy * uy > self.alpha * (di - self.d_safe) - w for w, ((ux, uy), di) in zip(tight, cons)):
                self.drift_ticks += 1
            cons = [((ux, uy), di - w / self.alpha) for w, ((ux, uy), di) in zip(tight, cons)]
        retreat = False
        for _ in range(3):  # cyclic projection"""),
("""        fwd, lat = max(-1.0, min(1.0, vx / vm)), max(-1.0, min(1.0, vy / vm))
        if abs(fwd - cmd.forward) > 0.04 or abs(lat - cmd.lateral) > 0.04:""", """        fwd, lat = max(-1.0, min(1.0, vx / vm)), max(-1.0, min(1.0, vy / vm))
        if self.drift_on:
            self._cmd_sent = (fwd * vm, lat * vm)
        if abs(fwd - cmd.forward) > 0.04 or abs(lat - cmd.lateral) > 0.04:"""),
("""    def _memory(self, d, tel, t: float):""", """    def _drift(self, tel, t: float, dt: float) -> tuple:
        \"\"\"Drift estimate (see the module patch note), returned in the body frame (x forward, y right).\"\"\"
        x, y, h = tel.x_m, tel.y_m, math.radians(tel.yaw_deg)
        ch, sh = math.cos(h), math.sin(h)
        self._dr_hist.append((t, x, y))
        self._dr_hist = [q for q in self._dr_hist if t - q[0] <= self.drift_win + 1e-9]
        cx, cy = self._cmd_sent
        k = 1 - math.exp(-dt / max(self.drift_resp, 1e-3)) if dt > 0 else 0.0
        self._vmod = (self._vmod[0] + (ch * cx - sh * cy - self._vmod[0]) * k, self._vmod[1] + (sh * cx + ch * cy - self._vmod[1]) * k)
        t0, x0, y0 = self._dr_hist[0]
        if t - t0 > 0.5 * self.drift_win:
            vmx, vmy = (x - x0) / (t - t0), (y - y0) / (t - t0)
            kw = 1 - math.exp(-dt / max(self.drift_tau, 1e-3)) if dt > 0 else 0.0
            wx = self.w_hat[0] + (vmx - self._vmod[0] - self.w_hat[0]) * kw
            wy = self.w_hat[1] + (vmy - self._vmod[1] - self.w_hat[1]) * kw
            n = math.hypot(wx, wy)
            if n > self.drift_max:
                wx, wy = wx * self.drift_max / n, wy * self.drift_max / n
            self.w_hat = (wx, wy)
        wx, wy = self.w_hat
        return (ch * wx + sh * wy, -sh * wx + ch * wy)

    def _memory(self, d, tel, t: float):"""),
]
for a, b in R:
    assert s.count(a) == 1, a[:70]
    s = s.replace(a, b)
shutil.copy2(p, str(p) + ".pre_drift")
p.write_text(s)
print("patched ecps_struct.py")
