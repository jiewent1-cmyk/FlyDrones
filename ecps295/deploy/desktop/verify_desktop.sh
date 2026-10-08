#!/usr/bin/env bash
# Acceptance test of a deployed desktop (setup_desktop.sh). Run it on an idle machine; takes ~15 min with the defaults.
#   1. versions (GPU, driver, kernel, Gazebo, ArduPilot, python packages) and MiniFly weight hashes vs reference.json
#   2. python -m rl.gates all in the run tree (sensory / smoke / anchor: the twin must be bit-identical to the server)
#   3. lowbox_s0, v4, flow + route B, 120 s patrol, seed 0 (the S8 / server h0 setting), first on 1 instance and then
#      on N parallel instances (all flying the same episode) -> whole-run RTF per instance and the flight results,
#      printed next to the ROG and server runs of the same episode
# usage: bash ~/sim/verify_desktop.sh            NS="1 3 4" (instance counts), ECPS_TREE (default ~/sim/hw/ecps295)
#        SKIP_GZ=1 only steps 1-2
set -uo pipefail
export ECPS_TREE=${ECPS_TREE:-$HOME/sim/hw/ecps295}
NS=${NS:-1 3 4}
E=$ECPS_TREE
PY=$HOME/miniconda3/envs/flydrones/bin/python
OUT=$HOME/sim/runs/desktop_verify
REF=$HOME/sim/reference.json
mkdir -p "$OUT"
LOG=$OUT/verify.log
exec > >(tee -a "$LOG") 2>&1
echo "=== verify_desktop $(date '+%F %T')  tree $E"

echo "--- 1. versions"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || echo "nvidia-smi FAILED"
echo "kernel $(uname -r); CPU $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 | xargs) ($(nproc) threads)"
gz sim --version 2>/dev/null | head -1
echo "ArduPilot $(git -C ~/sim/ardupilot describe --tags 2>/dev/null) (patched files: $(git -C ~/sim/ardupilot diff --name-only | xargs))"
echo "ardupilot_gazebo $(git -C ~/sim/ardupilot_gazebo log --oneline -1 2>/dev/null)"
cat "$E/../DEPLOY_SOURCE.txt" 2>/dev/null
"$PY" - "$E" "$REF" <<'PY'
import hashlib, json, sys
from pathlib import Path
import numpy, scipy
e, ref = Path(sys.argv[1]), json.load(open(sys.argv[2]))
print(f"python {sys.version.split()[0]}  numpy {numpy.__version__}  scipy {scipy.__version__}")
for name, want in ref["minifly_sha256"].items():
    p = e / "minifly" / name
    got = hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else "missing"
    print(f"{name}: {'OK' if got == want else 'DIFFERENT'} {got[:16]}")
PY

echo "--- 2. rl.gates all (anchors from $E/rl/anchors.json)"
(cd "$E" && PYTHONPATH=$(dirname "$E")/src:. "$PY" -m rl.gates all)
echo "gates exit code $?"
[ -n "${SKIP_GZ:-}" ] && exit 0

echo "--- 3. Gazebo + SITL: lowbox_s0 v4 (flow + route B), instances: $NS"
busy=$(pgrep -fc "^gz sim|arducopter" || true)
[ "$busy" -gt 0 ] && { echo "Gazebo/SITL already running ($busy processes): stop them first"; exit 1; }
load=$(cut -d' ' -f1 /proc/loadavg)
awk -v l="$load" 'BEGIN {exit !(l > 1.5)}' && echo "WARN: load $load before the test, RTF numbers will be low"
V=$E/sitl/variants
ARGS="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
for n in $NS; do
  M=$OUT/n$n.txt
  : > "$M"
  for r in $(seq 0 $((n - 1))); do echo "dsk_n${n}_r${r}__lowbox_s0_k0 lowbox_s0 v4 0 $ARGS" >> "$M"; done
  rm -rf $(sed 's#^\([^ ]*\).*#'"$HOME"'/sim/runs/\1#' "$M") "$OUT/n$n.log"
  echo "$(date +%T) $n instance(s)..."
  NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash ~/sim/run_matrix_par.sh "$M" "$OUT/n$n.log" "$n"
done

"$PY" - "$OUT" "$REF" $NS <<'PY'
import json, os, statistics, sys
out, ref, ns = sys.argv[1], json.load(open(sys.argv[2])), [int(x) for x in sys.argv[3:]]
runs = os.path.expanduser("~/sim/runs")
keys = ["contact_episodes", "near_miss_episodes", "min_clearance_m", "path_m", "coverage", "alt_true_max_m", "fc_land"]
rows, summary, bad = [], {}, []
for n in ns:
    rtfs = []
    for r in range(n):
        tag = f"dsk_n{n}_r{r}__lowbox_s0_k0"
        d = os.path.join(runs, tag)
        try:
            j = json.load(open(os.path.join(d, "run.json")))
        except Exception:
            rows.append((f"{n} inst #{r}", None, {}))
            bad.append(f"{tag}: no run.json (see {d}/gz.log, sitl.out, bridge.log, exp.log)")
            continue
        rtf = next((float(l.split()[2]) for l in open(os.path.join(d, "rtf.log")) if l.startswith("RTF run")), None) if os.path.exists(os.path.join(d, "rtf.log")) else None
        rtfs.append(rtf or 0.0)
        rows.append((f"{n} inst #{r}", rtf, j))
        if j.get("fc_land") or (j.get("alt_true_max_m") or 0) > 1.3:
            bad.append(f"{tag}: FC landing or above the 1.3 m fence")
        if (j.get("path_m") or 0) < 5:
            bad.append(f"{tag}: path {j.get('path_m')} m (stuck?)")
    summary[n] = {"rtf_median": statistics.median(rtfs) if rtfs else None, "rtf_min": min(rtfs) if rtfs else None,
                  "valid": sum(x >= 0.95 for x in rtfs), "runs": n}
print("\nlowbox_s0, v4, flow + route B (desktop runs, then the references)")
print(f"{'run':34s} {'RTF':>6s} " + " ".join(f"{k[:14]:>14s}" for k in keys))
for name, rtf, j in rows:
    print(f"{'desktop ' + name:34s} {('%.3f' % rtf) if rtf else '-':>6s} " + " ".join(f"{str(j.get(k, '-')):>14s}" for k in keys))
for r in ref["lowbox_s0_v4"]:
    print(f"{r['where'][:34]:34s} {'':>6s} " + " ".join(f"{str(r.get(k, '-')):>14s}" for k in keys))
print("\nRTF by instance count (whole-run RTF >= 0.95 is the validity rule on ROG):")
for n, s in summary.items():
    med = s["rtf_median"]
    print(f"  {n} instance(s): median {med if med is None else round(med, 3)}, min {s['rtf_min'] if s['rtf_min'] is None else round(s['rtf_min'], 3)}, valid {s['valid']}/{s['runs']}")
print("\nproblems:" if bad else "\nno failed / stuck / fenced runs")
for b in bad:
    print("  " + b)
json.dump({"summary": summary, "problems": bad}, open(os.path.join(out, "summary.json"), "w"), indent=1)
PY
echo "full log: $LOG"
