# usage: run_matrix.sh <matrix file> <log>     each line: <tag> <world> <variant> <seed> <run_g4 mode args...>
# runs every line through gz_g4.sh (custom MiniFly read-out, pilot and retina) and logs one summary line per run
# NAV=flow / EXTRA_PARM=... in the environment are passed on to gz_g4.sh
M=~/sim/FlyDrones/ecps295/minifly
while read -r tag world variant seed args; do
  [ -z "$tag" ] || [ "${tag:0:1}" = "#" ] && continue
  bash ~/sim/gz_g4.sh ecps295_$world.sdf $tag "$args --seed $seed --config $M/$variant.yaml --decoder ecps --pilot ecps --retina ecps --out run" > /dev/null 2>&1
  /usr/bin/python3 - "$tag" "$HOME/sim/runs/$tag/run.json" <<'PY' >> "$2"
import json, os, sys
tag, path = sys.argv[1], sys.argv[2]
try:
    j = json.load(open(path))
except Exception as e:
    print(f"{tag} FAILED {e!r}"); sys.exit()
keys = ["contact_episodes", "near_miss_episodes", "min_clearance_m", "path_m", "coverage", "fence_turn_s", "escapes",
        "nav", "ekf_err_xy_max_m", "radius_true_max_m"]
rtf = None  # whole-run "RTF run X over N s" (gz_g4.sh); the 40-sample window mean was too noisy to judge a run
for line in open(path.replace("run.json", "rtf.log")) if os.path.exists(path.replace("run.json", "rtf.log")) else []:
    if line.startswith("RTF run"):
        rtf = line.split()[2]
print(tag, " ".join(f"{k}={j.get(k)}" for k in keys), f"rtf_run={rtf}", "saccades=" + ",".join(("L" if e[2] < 0 else "R") for e in j["escape_log"]))
PY
done < "$1"
echo DONE >> "$2"
