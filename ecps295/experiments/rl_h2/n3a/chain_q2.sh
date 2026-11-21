#!/bin/bash
# ROG queue 2 (2026-10-15); waits for queue 1 (matrix_q1.log DONE), then:
#   1. N5 g7_w2map_tf10 = s7_w2map + sitl/variants/H_terrain_fast.parm (EK3_TERR_GRAD 10, EK3_RNG_M_NSE 0.1: the EKF terrain
#      state follows box tops, so the flow is scaled correctly over boxes); set A, 58 runs, paired with g7_w2map__*.
#   2. if n3a/q2b_matrix.txt exists when step 1 ends: run it with the default params (prepared: n3s2 set A).
# Quiet hours (no start 00:55-07:00 PT) are enforced by run_matrix_mine.sh.
set -e
cd ~/sim/rl_h2
until grep -q "^DONE" ~/sim/runs/matrix_q1.log 2>/dev/null; do sleep 60; done
V=~/sim/rl_h2/ecps295/sitl/variants
test -f $V/H_terrain_fast.parm
export ECPS_TOF_CONE3D=1
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
{ for k in $(seq 0 49); do echo "g7_w2map_tf10__lowbox_s${k}_k${k} lowbox_s${k} s7_w2map@intact ${k} $A"; done
  for k in $(seq 0 7); do echo "g7_w2map_tf10__cage20_k${k} cage20 s7_w2map@intact ${k} $A"; done; } > n3a/q2a_matrix.txt
echo "launch q2a $(wc -l < n3a/q2a_matrix.txt) runs $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm bash run_matrix_par_mine.sh n3a/q2a_matrix.txt ~/sim/runs/matrix_q2a.log 3
if [ -f n3a/q2b_matrix.txt ]; then
  echo "launch q2b $(wc -l < n3a/q2b_matrix.txt) runs $(date)"
  NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh n3a/q2b_matrix.txt ~/sim/runs/matrix_q2b.log 3
fi
echo "q2 done $(date)"
