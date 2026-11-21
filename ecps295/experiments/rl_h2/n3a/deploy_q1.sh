#!/bin/bash
# 2026-10-15: prepare ~/sim/rl_h2/ecps295 for queue 1 (run on ROG only after matrix_s7c.log says DONE: running Gazebo
# instances import these files). Both patches leave old configs unchanged (debug columns only / band_lo_m defaults to band_m).
set -e
grep -q "^DONE" ~/sim/runs/matrix_s7c.log
cd ~/sim/rl_h2/ecps295
md5sum ecps_struct.py ecps_map.py run_g4.py
python3 ../n3a/patch_shdbg.py
python3 ../n3a/patch_bandlo.py
python3 -m py_compile ecps_struct.py ecps_map.py run_g4.py
md5sum ecps_struct.py ecps_map.py run_g4.py
python3 ../n3a/gz_export.py ../n3a/n3s1.json
python3 ../n3a/gz_export.py ../n3a/s7_w2map_lo45.json
python3 - <<'PY'
import yaml
def flat(d, p=""):
    o = {}
    for k, v in d.items():
        q = f"{p}.{k}" if p else str(k)
        o.update(flat(v, q) if isinstance(v, dict) else {q: v})
    return o
b = flat(yaml.safe_load(open("minifly/s7_w2map.yaml")))
for n in ("n3s1", "s7_w2map_lo45"):
    c = flat(yaml.safe_load(open(f"minifly/{n}.yaml")))
    print(n, "keys changed:", sorted(k for k in set(b) | set(c) if b.get(k, "<absent>") != c.get(k, "<absent>")))
PY
echo deploy ok
