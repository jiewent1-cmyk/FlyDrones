#!/bin/bash
# ROG queue 3 (2026-10-15); waits for queue 2 ("q2 done" in chain_q2.out), then:
#   1. g7_w2map_r2: the reference s7_w2map again on set A (58 runs), default EKF params. Day control for queue 2: tf10 flew
#      1.2 m less (p = 0.002) with the same commands and no flight-controller difference in the logs.
#   2. g7_w2map_tf10 on set B (58 runs), EKF terrain params as queue 2 (sitl/variants/H_terrain_fast.parm).
# Quiet hours (no start 00:55-07:00 PT) are enforced by run_matrix_mine.sh.
set -e
cd ~/sim/rl_h2
until grep -q "^q2 done" ~/sim/runs/chain_q2.out 2>/dev/null; do sleep 60; done
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
{ for k in $(seq 0 49); do echo "g7_w2map_r2__lowbox_s${k}_k${k} lowbox_s${k} s7_w2map@intact ${k} $A"; done
  for k in $(seq 0 7); do echo "g7_w2map_r2__cage20_k${k} cage20 s7_w2map@intact ${k} $A"; done; } > n3a/q3a_matrix.txt
{ for k in $(seq 0 49); do echo "g7_w2map_tf10__lowbox_s${k}_k$((k + 100)) lowbox_s${k} s7_w2map@intact $((k + 100)) $A"; done
  for k in $(seq 8 15); do echo "g7_w2map_tf10__cage20_k${k} cage20 s7_w2map@intact ${k} $A"; done; } > n3a/q3b_matrix.txt
echo "launch q3a $(wc -l < n3a/q3a_matrix.txt) runs $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh n3a/q3a_matrix.txt ~/sim/runs/matrix_q3a.log 3
echo "launch q3b $(wc -l < n3a/q3b_matrix.txt) runs $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm bash run_matrix_par_mine.sh n3a/q3b_matrix.txt ~/sim/runs/matrix_q3b.log 3
echo "q3 done $(date)"
