#!/bin/bash
# ROG queue 4 (2026-10-15, N4 drift stress); waits for queue 3 ("q3 done" in chain_q3.out), then
#   deploy: patch_drift.py (ecps_struct drift-aware shield, default off), patch_drift_gz.py (mavlink_twin drift injection
#           ECPS_DRIFT_MPS, run_g4 drift direction from the brain seed + run.json fields), minifly/s7_w2map_dr.yaml
#   1. ECPS_DRIFT_MPS=0.03, set A, interleaved: g7_w2map_wd (reference shield) and g7_w2map_drwd (drift-aware shield), 116 runs
#   2. no drift, set A: g7_w2map_dr (drift-aware shield; must not cost anything without drift), 58 runs
# Twin v2 (rl.twin ood wind: 0.03 m/s + gusts): reference 0/50 -> 3/50 lowbox and 0/32 -> 4/32 cage20 contact runs.
set -e
cd ~/sim/rl_h2
until grep -q "^q3 done" ~/sim/runs/chain_q3.out 2>/dev/null; do sleep 60; done
cd ecps295
md5sum ecps_struct.py mavlink_twin.py run_g4.py
python3 ../n3a/patch_drift.py
python3 ../n3a/patch_drift_gz.py
python3 -m py_compile ecps_struct.py mavlink_twin.py run_g4.py
md5sum ecps_struct.py mavlink_twin.py run_g4.py
python3 ../n3a/gz_export.py ../n3a/s7_w2map_dr.json
cd ..
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
lines() {  # <tag> <variant yaml>
  for k in $(seq 0 49); do echo "$1__lowbox_s${k}_k${k} lowbox_s${k} $2@intact ${k} $A"; done
  for k in $(seq 0 7); do echo "$1__cage20_k${k} cage20 $2@intact ${k} $A"; done
}
paste -d '\n' <(lines g7_w2map_wd s7_w2map) <(lines g7_w2map_drwd s7_w2map_dr) > n3a/q4a_matrix.txt
lines g7_w2map_dr s7_w2map_dr > n3a/q4b_matrix.txt
echo "launch q4a $(wc -l < n3a/q4a_matrix.txt) runs $(date)"
ECPS_DRIFT_MPS=0.03 NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh n3a/q4a_matrix.txt ~/sim/runs/matrix_q4a.log 3
echo "launch q4b $(wc -l < n3a/q4b_matrix.txt) runs $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh n3a/q4b_matrix.txt ~/sim/runs/matrix_q4b.log 3
echo "q4 done $(date)"
