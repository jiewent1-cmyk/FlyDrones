"""Apply the target-tracking hooks to run_g4.py (2026-10-18, ~/sim/rl_trk tree only). Off unless --track is given."""
import sys
from pathlib import Path

p = Path(sys.argv[1])
s = p.read_text()
assert "--track" not in s, "already patched"


def sub(old, new, count=1):
    global s
    assert s.count(old) == count, (old, s.count(old))
    s = s.replace(old, new)


sub('ap.add_argument("--out", default="g4")\n',
    'ap.add_argument("--out", default="g4")\n'
    'ap.add_argument("--track", default="", help="2026-10-18 target tracking: <world>_mk.marker.json (trk_gz.py)")\n'
    'ap.add_argument("--track-at", type=float, default=6.0, help="flight time [s] of pilot.track_start()")\n'
    'ap.add_argument("--track-hold", type=float, default=10.0, help="end the run this long after target: arrived")\n'
    'ap.add_argument("--shm-rgb", default="/dev/shm/ecps295_cam_rgb", help="colour drone camera (cam_bridge rgb stream)")\n')
sub('HZ = cfg["control"]["hz"]\n',
    'HZ = cfg["control"]["hz"]\n'
    'TRK = None  # 2026-10-18 target tracking (trk_gz.py): marker detection on the colour camera, truth scoring\n'
    'if a.track:\n'
    '    from trk_gz import GzMarker\n'
    '\n'
    '    TRK = GzMarker(a.track, a.shm_rgb, cfg.get("marker", {}), out_dir=str(Path(a.out).resolve().parent))\n'
    '    pilot.target.source = TRK\n')
sub('    c0 = time.perf_counter()\n    info = pilot.tick(t, dt)',
    '    if TRK is not None:\n'
    '        if not TRK.started and t >= a.track_at:\n'
    '            pilot.track_start(t)\n'
    '            TRK.started, TRK.t_start = True, round(t, 2)\n'
    '        TRK.step(t, drone)\n'
    '    c0 = time.perf_counter()\n    info = pilot.tick(t, dt)')
sub('            **(pil.row() if pil is not None else {}),\n',
    '            **(pil.row() if pil is not None else {}),\n'
    '            **(TRK.row(pilot) if TRK is not None else {}),\n')
sub('    done = getattr(pilot, "wp_done_t", None)\n',
    '    if TRK is not None:  # tracking flies through the waypoint layer: its wp_done_t is not the end of the run\n'
    '        return TRK.done(pilot, clock["t"], a.track_hold)\n'
    '    done = getattr(pilot, "wp_done_t", None)\n')
sub('json.dump(summary, open(f"{a.out}.json", "w"), indent=1)\n',
    'if TRK is not None:\n'
    '    summary.update(TRK.summary(pilot, rows, a.mode))\n'
    'json.dump(summary, open(f"{a.out}.json", "w"), indent=1)\n')
p.write_text(s)
print("patched", p)
